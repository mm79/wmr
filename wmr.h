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

#define WMR_IS_RAIN		0x41
#define WMR_IS_TEMPERATURE	0x42
#define WMR_IS_PRESSURE		0x46
#define WMR_IS_UV		0x47
#define WMR_IS_WIND		0x48
#define WMR_IS_CLOCK		0x60

struct wmr_temperature {
	uint8_t dewpoint;
	uint8_t temperature;
	uint8_t temp;
	uint8_t temp_dec;
	uint8_t	sensor;
	uint8_t smile;
	uint8_t trend;
	uint8_t humidity;
};

struct wmr_pressure {
	int16_t pressure;
	int16_t forecast;
	int16_t alt_pressure;
	int16_t alt_forecast;
};

struct wmr_header {
	uint8_t wh_type;
	union {
		struct wmr_temperature temp;
		struct wmr_pressure press;
	} wmr_data;		
};


#define wmr_temp 	wmr_data.temp
#define wmr_press	wmr_data.press
