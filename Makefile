KMOD= 	wmr	
SRCS=	wmr.c
SRCS+=	opt_usb.h
SRCS+=	device_if.h bus_if.h usb_if.h

.include <bsd.kmod.mk>
