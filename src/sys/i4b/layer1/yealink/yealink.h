/*-
 *
 * Copyright (c) 2005 Henk Vergonet. All rights reserved.
 *
 * Copyright (c) 2009 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *---------------------------------------------------------------------------*/

#ifndef _YEALINK_H_
#define	_YEALINK_H_

#define	YEALINK_MINFRAMES		25	/* units */
#define	YEALINK_BPF			((8000 * 2) / 1000)	/* bytes/frame */
#define	YEALINK_BUFSIZE			(YEALINK_BPF * YEALINK_MINFRAMES)
#define	YEALINK_CONFIG_INDEX		0
#define	YEALINK_IFACE_INDEX		0
#define	YEALINK_PKT_LEN			16	/* bytes */
#define	YEALINK_CMD_HANDSET_QUERY       0x8d
#define	YEALINK_CMD_INIT                0x8e
#define	YEALINK_CMD_KEYPRESS            0x80
#define	YEALINK_CMD_SCANCODE            0x81
#define	YEALINK_CMD_LCD                 0x04
#define	YEALINK_CMD_LED                 0x05
#define	YEALINK_CMD_RING_VOLUME         0x11
#define	YEALINK_CMD_RING_NOTE           0x02
#define	YEALINK_CMD_RINGTONE            0x03
#define	YEALINK_CMD_DIALTONE            0x09

enum {
	YEALINK_ST_INIT,
	YEALINK_ST_SET_MIC,
	YEALINK_ST_SET_PCM,
	YEALINK_ST_USB_ON,
	YEALINK_ST_USB_OFF,
	YEALINK_ST_LED_ON,
	YEALINK_ST_LED_OFF,
	YEALINK_ST_TONE_ON,
	YEALINK_ST_TONE_OFF,
	YEALINK_ST_RING_ON,
	YEALINK_ST_RING_OFF,
	YEALINK_ST_HANDSET_QUERY,
	YEALINK_ST_KEY_QUERY,
	YEALINK_ST_KEY_CODE,
	YEALINK_ST_MAX,
};

enum {
	YEALINK_XFER_CTRL,
	YEALINK_XFER_INTR,
	YEALINK_XFER_ISOC_IN_0,
	YEALINK_XFER_ISOC_IN_1,
	YEALINK_XFER_ISOC_OUT_0,
	YEALINK_XFER_ISOC_OUT_1,
	YEALINK_XFER_MAX,
};

struct yealink_ctl_packet {
	uByte	raw[0];
	uByte	cmd;
	uByte	size;
	uWord	offset;
	uByte	data[11];
	uByte	sum;
} __packed;

struct yealink_ctrl {
	struct usb_device_request req;
	struct yealink_ctl_packet data;
};

struct yealink_intr {
	struct yealink_ctl_packet data;
	uint8_t	rem[YEALINK_PKT_LEN - sizeof(struct yealink_ctl_packet)];
};

struct yealink_softc {

	struct dss1_lite sc_dl;

	struct yealink_ctrl sc_ctrl;
	struct yealink_intr sc_intr;

	struct mtx *sc_pmtx;

	struct usb_device *sc_udev;
	struct usb_xfer *sc_xfer[YEALINK_XFER_MAX];

	int	sc_last_ring;

	uint8_t	sc_buffer[YEALINK_BUFSIZE * 4];

	uint8_t	sc_st_data[YEALINK_ST_MAX];
	uint8_t	sc_st_index;
	uint8_t	sc_iface_no;
	uint8_t	sc_key_state;
	uint8_t	sc_hook_state;
	uint8_t	sc_no_interrupt;
	uint8_t	sc_is_ringing;
};

#endif					/* _YEALINK_H_ */
