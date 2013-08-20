/*
 * Copyright (c) 2013 Matteo Mazzarella <matteo@dancingbear.it>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This device driver was written as exercise while reading the book:  
 * "FreeBSD Device Drivers: A Guide for the Intrepid"
 *
 * There is no need to use a kernel device driver for your weather station.
 * You can have easy access to your wmr by using libusb from userspace.
 *
 * Tested on a wmr 88
 *
 * FreeBSD dharma 9.1-RELEASE-p5 FreeBSD 9.1-RELEASE-p5 #0
 * root@amd64-builder.daemonology.net:/usr/obj/usr/src/sys/GENERIC  amd64
 *
 */

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/syslog.h>
#include <sys/fcntl.h>

#include <dev/usb/usb.h>
#define USB_DEBUG_VAR wmr_debug
#include <dev/usb/usb_debug.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>

#include "wmr.h"

#ifdef USB_DEBUG
static int wmr_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, wmr, CTLFLAG_RW, 0, "USB wmr weather station");
SYSCTL_INT(_hw_usb_wmr, OID_AUTO, debug, CTLFLAG_RW,
    &wmr_debug, 0, "Debug level");
#endif

#define WMR_VENDOR		0x0fde
#define WMR_PRODUCT		0xca01

#define WMR_INTR_BUF_SIZE	8
#define WMR_PKT_SIZE		WMR_INTR_BUF_SIZE*4
#define WMR_FIFO_BUF_SIZE	sizeof(struct wmr_header) 
#define WMR_FIFO_QUEUE_MAXLEN	50

enum {
	WMR_INTR_RD,
	WMR_N_TRANSFER
};

struct wmr_softc {
	struct usb_xfer *sc_xfer[WMR_N_TRANSFER];

	device_t		sc_dev;
	struct usb_device      *sc_udev;
	struct mtx		sc_mutex;
	struct usb_fifo_sc	sc_fifo;
	uint8_t			sc_pkt[WMR_PKT_SIZE];
	uint8_t			sc_tmp[WMR_PKT_SIZE];
	uint8_t			sc_pktlen;
	uint8_t			sc_marker;

	int sc_fflags;
};

static device_probe_t		wmr_probe;
static device_attach_t		wmr_attach;
static device_detach_t		wmr_detach;

static usb_callback_t		wmr_intr_callback;

static usb_fifo_open_t	wmr_open;
static usb_fifo_close_t	wmr_close;
static usb_fifo_cmd_t 	wmr_start_read, 
			wmr_stop_read;
static usb_fifo_ioctl_t	wmr_ioctl;


/* lsusb -v -d idVendor:idProduct */
static const struct usb_config wmr_config[WMR_N_TRANSFER] = 
{
	[WMR_INTR_RD] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = WMR_INTR_BUF_SIZE,
		.callback = &wmr_intr_callback,
	},
};

static struct usb_fifo_methods wmr_fifo_methods = {
	.f_open 	= &wmr_open,
	.f_close 	= &wmr_close,
	.f_ioctl	= &wmr_ioctl,
	.f_start_read	= &wmr_start_read,
	.f_stop_read	= &wmr_stop_read,
	.basename[0] = "wmr"
};

static device_method_t wmr_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_probe,		wmr_probe),
	DEVMETHOD(device_attach,	wmr_attach),
	DEVMETHOD(device_detach,	wmr_detach),
	{ 0, 0 }
};

static driver_t wmr_driver = {
	"wmr",
	wmr_methods,
	sizeof(struct wmr_softc)
};

static devclass_t wmr_devclass;

DRIVER_MODULE(wmr, uhub, wmr_driver, wmr_devclass, 0, 0);
MODULE_DEPEND(wmr, usb, 1, 1, 1);

static const STRUCT_USB_HOST_ID wmr_devs[] = {
	{USB_VPI(WMR_VENDOR, WMR_PRODUCT, 0)}
};

static int
wmr_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST)
                return (ENXIO);

	return (usbd_lookup_id_by_uaa(wmr_devs, sizeof(wmr_devs), uaa));
}

static usb_error_t
wmr_do(struct usb_device *udev, int requesttype, int request,
    uint16_t value, void *data, int index, uint16_t length)  
{
        struct usb_device_request req;
        usb_error_t err;

        req.bmRequestType = requesttype;
        req.bRequest = request;
	USETW(req.wValue, value);
        USETW(req.wIndex, index);
        USETW(req.wLength, length);

        err = usbd_do_request(udev, NULL, &req, data);
        if (err) {
                DPRINTF("wmr_do error=%s\n", usbd_errstr(err));
                return (1);
        }

        return (0);
}

static void
wmr_init(struct wmr_softc *sc)
{
	unsigned char INIT_PKT[] = { 0x20, 0x00, 0x08, 0x01, 0x00, 0x00,
		0x00, 0x00 };

	/* XXX */
	wmr_do(sc->sc_udev, UT_WRITE_CLASS_INTERFACE, 0x09, 0x02<<8,
                (unsigned char *)INIT_PKT, 0, 8);
}

static int
wmr_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct wmr_softc *sc = device_get_softc(dev);
	int unit = device_get_unit(dev);
	int error;

	sc->sc_dev = dev;
	sc->sc_udev = uaa->device;
	
	sc->sc_pktlen = 0;
	sc->sc_marker = 0;

	device_set_usb_desc(dev);

	mtx_init(&sc->sc_mutex, "wmr lock", NULL, MTX_DEF | MTX_RECURSE);

	error = usbd_transfer_setup(uaa->device,
		&uaa->info.bIfaceIndex, sc->sc_xfer, wmr_config,
		WMR_N_TRANSFER, sc, &sc->sc_mutex);	
	if (error)
		goto detach;

	mtx_lock(&sc->sc_mutex);
        usbd_xfer_set_stall(sc->sc_xfer[WMR_INTR_RD]);
        mtx_unlock(&sc->sc_mutex);

	error = usb_fifo_attach(uaa->device, sc, &sc->sc_mutex, 
		&wmr_fifo_methods, &sc->sc_fifo, unit, -1,
		uaa->info.bIfaceIndex, UID_ROOT, GID_OPERATOR, 0644);
	if (error)
		goto detach;

	return (0);

detach:
	wmr_detach(dev);
	return (ENOMEM);
}

static int
wmr_detach(device_t dev)
{
	struct wmr_softc *sc = device_get_softc(dev);

	usb_fifo_detach(&sc->sc_fifo);
	usbd_transfer_unsetup(sc->sc_xfer, WMR_N_TRANSFER);
	mtx_destroy(&sc->sc_mutex);

	return 0;
}

/* based on wsr.c */
static int
verify_checksum(unsigned char *buf, int len) 
{
	int i, ret = 0, chk;

	for (i = 0; i < len - 2; ++i)
		ret += buf[i];

	chk = buf[len-2] + (buf[len-1] << 8);
    
	if (ret != chk) {
		DPRINTF("wmr bad checksum: %d / %d\n", ret, chk);
		return -1;
	}   

	return 0;
}

static void
wmr_put_queue(struct wmr_softc *sc, void *data, int len)
{
	usb_fifo_put_data_linear(sc->sc_fifo.fp[USB_FIFO_RX], data,
		len, 1);
}

static void
wmr_intr_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct wmr_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;
	uint8_t *buf, *pkt;
	int i, len;

	buf = sc->sc_tmp;
	pkt = sc->sc_pkt;

	usbd_xfer_status(xfer, &len, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		if (len <= 1) 
			goto st_setup;

		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, buf, len);

		DPRINTF("data = %02x %02x %02x %02x "
                    "%02x %02x %02x %02x\n",
                    (len > 0) ? buf[0] : 0, (len > 1) ? buf[1] : 0,
                    (len > 2) ? buf[2] : 0, (len > 3) ? buf[3] : 0,
                    (len > 4) ? buf[4] : 0, (len > 5) ? buf[5] : 0,
                    (len > 6) ? buf[6] : 0, (len > 7) ? buf[7] : 0);


		/* check if pkt is too big */
		if (buf[0] >= 0xa) {
			/* discard incomplete data */
			sc->sc_pktlen = sc->sc_marker = 0;
			goto st_setup;
		}
 
		/*
		 * end of data is received by 2 8-byte transfer data 
		 * for example:
		 *
		 * prev buf = 01 ff 00 ba 8b 00 0c 01
		 * next buf = 01 ff 00 ba 8b 00 0c 01
		 *
		 * - the first byte (01) tell us to consider only 
		 *   the next byte (0xff)
		 *
		 * - the second byte (0xff) is our marker
		 *
		 * - other bytes can be ignored
		 *
		 */
		if (buf[0] == 0x1 && buf[1] == 0xff &&
		   (sc->sc_pktlen == 0 || sc->sc_marker++ == 0)) 
			goto st_setup;

		/* check marker */
		if (sc->sc_marker >= 2) {
			struct wmr_header h;

			if (verify_checksum(pkt, sc->sc_pktlen) < 0)
				goto st_setup;
	
			h.wh_type = pkt[1];

			switch(pkt[1]) {
			case WMR_IS_TEMPERATURE:
				h.wmr_temp.temperature = (pkt[3] + \
					((pkt[4] & 0x0f) << 8)) / 10;

				h.wmr_temp.sensor = pkt[2] & 0x0f;
				h.wmr_temp.humidity = pkt[5];
				break;
			case WMR_IS_PRESSURE:
				h.wmr_press.pressure = pkt[2] + \
					((pkt[3] & 0x0f) << 8);
				h.wmr_press.forecast = pkt[3] >> 4;
				h.wmr_press.alt_pressure = pkt[4] + \
					((pkt[5] & 0x0f) << 8);
				h.wmr_press.alt_forecast = pkt[5] >> 4;
				break;
			/* and so on... */
			}	

			wmr_put_queue(sc, &h, sizeof h);

			sc->sc_pktlen = sc->sc_marker = 0;
			break;	
		}

		for (i=1; i <= buf[0] && sc->sc_pktlen < WMR_PKT_SIZE; i++)
			pkt[sc->sc_pktlen++] = buf[i]; 

st_setup:
	case USB_ST_SETUP:
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
                usbd_transfer_submit(xfer);

		break;
		
	default:
		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			goto st_setup;
		}
	}
}


static int
wmr_open(struct usb_fifo *fifo, int fflags)
{
	struct wmr_softc *sc = usb_fifo_softc(fifo);

	wmr_init(sc);

	if (fflags & FREAD) {
		if (usb_fifo_alloc_buffer(fifo,
			WMR_FIFO_BUF_SIZE, WMR_FIFO_QUEUE_MAXLEN))
			return (ENOMEM);
	}

	sc->sc_fflags |= fflags & FREAD;

	return 0;
}

static void
wmr_close(struct usb_fifo *fifo, int fflags)
{
	struct wmr_softc *sc = usb_fifo_softc(fifo);

	if (fflags & FREAD)
		usb_fifo_free_buffer(fifo);
	
	sc->sc_fflags &= ~(fflags & FREAD);
}

static void
wmr_start_read(struct usb_fifo *fifo)
{
	struct wmr_softc *sc = usb_fifo_softc(fifo);

	usbd_transfer_start(sc->sc_xfer[WMR_INTR_RD]);
}

static void
wmr_stop_read(struct usb_fifo *fifo)
{
	struct wmr_softc *sc = usb_fifo_softc(fifo);

	usbd_transfer_stop(sc->sc_xfer[WMR_INTR_RD]);
}

static int 
wmr_ioctl(struct usb_fifo *fifo, u_long cmd, void *data,
	int fflags)
{
	return (ENODEV);
}
