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
 *---------------------------------------------------------------------------
 *
 *      Yealink driver for ISDN4BSD
 *      ---------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

/*
 * NOTE: The yealink.c driver in Linux has been used as hardware
 * documentation when making this driver:
 *
 * Description:
 *   Driver for the USB-P1K VOIP USB phone.
 *   This device is produced by Yealink Network Technology Co Ltd,
 *   but may be branded under several names:
 *      - Yealink usb-p1k
 *      - Tiptel 115
 *      - ...
 *
 * This yealink.c driver in Linux is based on:
 *   - the usbb2k-api   http://savannah.nongnu.org/projects/usbb2k-api/
 *   - information from http://memeteau.free.fr/usbb2k
 *
 * Thanks to:
 *   - Olivier Vandorpe, for providing the usbb2k-api.
 */

#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/linker_set.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/priv.h>

#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>
#include <dev/usb/usbdi.h>

#define	USB_DEBUG_VAR yealink_debug

#include <dev/usb/usb_core.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_device.h>
#include <dev/usb/usb_request.h>
#include <dev/usb/usb_debug.h>
#include <dev/usb/usb_hub.h>
#include <dev/usb/usb_util.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_transfer.h>
#include <dev/usb/usb_dynamic.h>

#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>

#include <i4b/include/i4b_controller.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/dss1/dss1_lite.h>

#include <i4b/layer1/yealink/yealink.h>

#ifdef USB_DEBUG
static int yealink_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, yealink, CTLFLAG_RW, 0, "USB VoIP");
SYSCTL_INT(_hw_usb_yealink, OID_AUTO, debug, CTLFLAG_RW, &yealink_debug, 0,
    "Debug level");
#endif

static device_probe_t yealink_probe;
static device_attach_t yealink_attach;
static device_detach_t yealink_detach;

static devclass_t yealink_devclass;

static device_method_t yealink_methods[] = {
	DEVMETHOD(device_probe, yealink_probe),
	DEVMETHOD(device_attach, yealink_attach),
	DEVMETHOD(device_detach, yealink_detach),
	{0, 0}
};

static driver_t yealink_driver = {
	.name = "yealink",
	.methods = yealink_methods,
	.size = sizeof(struct yealink_softc)
};

DRIVER_MODULE(yealink, uhub, yealink_driver, yealink_devclass, NULL, 0);
MODULE_DEPEND(yealink, usb, 1, 1, 1);

static const struct dss1_lite_methods yealink_dl_methods = {
	DSS1_LITE_DEFAULT_METHODS,
};

/* static structures */

static const struct yealink_lcd_map yealink_lcd_map[YEALINK_LCD_LINE5_OFFSET] = {
	YEALINK_SEG('1', 0, 0, 22, 2, 22, 2, 0, 0, 0, 0, 0, 0, 0, 0),
	YEALINK_SEG('8', 20, 1, 20, 2, 20, 4, 20, 8, 21, 4, 21, 2, 21, 1),
	YEALINK_SYM('.', 22, 1, "M"),
	YEALINK_SEG('e', 18, 1, 18, 2, 18, 4, 18, 1, 19, 2, 18, 1, 19, 1),
	YEALINK_SEG('8', 16, 1, 16, 2, 16, 4, 16, 8, 17, 4, 17, 2, 17, 1),
	YEALINK_SYM('.', 15, 8, "D"),
	YEALINK_SEG('M', 14, 1, 14, 2, 14, 4, 14, 1, 15, 4, 15, 2, 15, 1),
	YEALINK_SEG('8', 12, 1, 12, 2, 12, 4, 12, 8, 13, 4, 13, 2, 13, 1),
	YEALINK_SYM('.', 11, 8, ":"),
	YEALINK_SEG('8', 10, 1, 10, 2, 10, 4, 10, 8, 11, 4, 11, 2, 11, 1),
	YEALINK_SEG('8', 8, 1, 8, 2, 8, 4, 8, 8, 9, 4, 9, 2, 9, 1),
	YEALINK_SYM('.', 7, 1, "IN"),
	YEALINK_SYM('.', 7, 2, "OUT"),
	YEALINK_SYM('.', 7, 4, "STORE"),
	YEALINK_SEG('1', 0, 0, 5, 1, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0),
	YEALINK_SEG('8', 4, 1, 4, 2, 4, 4, 4, 8, 5, 8, 5, 4, 5, 2),
	YEALINK_SEG('8', 2, 1, 2, 2, 2, 4, 2, 8, 3, 4, 3, 2, 3, 1),
	YEALINK_SYM('.', 23, 2, "NEW"),
	YEALINK_SYM('.', 23, 4, "REP"),
	YEALINK_SYM('.', 1, 8, "SU"),
	YEALINK_SYM('.', 1, 4, "MO"),
	YEALINK_SYM('.', 1, 2, "TU"),
	YEALINK_SYM('.', 1, 1, "WE"),
	YEALINK_SYM('.', 0, 1, "TH"),
	YEALINK_SYM('.', 0, 2, "FR"),
	YEALINK_SYM('.', 0, 4, "SA"),
	YEALINK_SEG('8', 22, 16, 22, 32, 22, 64, 22, 128, 23, 128, 23, 64, 23, 32),
	YEALINK_SEG('8', 20, 16, 20, 32, 20, 64, 20, 128, 21, 128, 21, 64, 21, 32),
	YEALINK_SEG('8', 18, 16, 18, 32, 18, 64, 18, 128, 19, 128, 19, 64, 19, 32),
	YEALINK_SEG('8', 16, 16, 16, 32, 16, 64, 16, 128, 17, 128, 17, 64, 17, 32),
	YEALINK_SEG('8', 14, 16, 14, 32, 14, 64, 14, 128, 15, 128, 15, 64, 15, 32),
	YEALINK_SEG('8', 12, 16, 12, 32, 12, 64, 12, 128, 13, 128, 13, 64, 13, 32),
	YEALINK_SEG('8', 10, 16, 10, 32, 10, 64, 10, 128, 11, 128, 11, 64, 11, 32),
	YEALINK_SEG('8', 8, 16, 8, 32, 8, 64, 8, 128, 9, 128, 9, 64, 9, 32),
	YEALINK_SEG('8', 6, 16, 6, 32, 6, 64, 6, 128, 7, 128, 7, 64, 7, 32),
	YEALINK_SEG('8', 4, 16, 4, 32, 4, 64, 4, 128, 5, 128, 5, 64, 5, 32),
	YEALINK_SEG('8', 2, 16, 2, 32, 2, 64, 2, 128, 3, 128, 3, 64, 3, 32),
	YEALINK_SEG('8', 0, 16, 0, 32, 0, 64, 0, 128, 1, 128, 1, 64, 1, 32),
	YEALINK_SYM('.', 24, 0x01, "LED"),
	YEALINK_SYM('.', 25, 0x01, "DIALTONE"),
	YEALINK_SYM('.', 26, 0x24, "RINGTONE"),
};

static const uint8_t yealink_ringtone[] = {
	0xEF,				/* volume [0-255] */
	0xFB, 0x1E, 0x00, 0x0C,		/* 1250 [hz], 12/100 [s] */
	0xFC, 0x18, 0x00, 0x0C,		/* 1000 [hz], 12/100 [s] */
	0xFB, 0x1E, 0x00, 0x0C,
	0xFC, 0x18, 0x00, 0x0C,
	0xFB, 0x1E, 0x00, 0x0C,
	0xFC, 0x18, 0x00, 0x0C,
	0xFB, 0x1E, 0x00, 0x0C,
	0xFC, 0x18, 0x00, 0x0C,
	0xFF, 0xFF, 0x01, 0x90,		/* silent, 400/100 [s] */
	0x00, 0x00			/* end of sequence */
};

static const uint8_t yealink_digitmap[10] = {
	0xFF ^ (SEG_G | SEG_DP),
	0x00 ^ (SEG_B | SEG_C | SEG_DP),
	0xFF ^ (SEG_C | SEG_F | SEG_DP),
	0xFF ^ (SEG_E | SEG_F | SEG_DP),
	0xFF ^ (SEG_A | SEG_D | SEG_E | SEG_DP),
	0xFF ^ (SEG_B | SEG_E | SEG_DP),
	0xFF ^ (SEG_B | SEG_DP),
	0xFF ^ (SEG_D | SEG_E | SEG_F | SEG_DP),
	0xFF ^ (SEG_DP),
	0xFF ^ (SEG_D | SEG_E | SEG_DP),
};

static const struct usb_device_id yealink_devs_table[] = {
	{USB_VPI(0x6993, 0xb001, 0)},
};

static usb_callback_t yealink_ctrl_callback;
static usb_callback_t yealink_intr_callback;
static usb_callback_t yealink_isoc_read_callback;
static usb_callback_t yealink_isoc_write_callback;

static const struct usb_config yealink_config[YEALINK_XFER_MAX] = {
	[YEALINK_XFER_CTRL] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(struct yealink_ctrl),
		.flags = {.ext_buffer = 1,},
		.callback = &yealink_ctrl_callback,
		.timeout = 1000,	/* 1 second */
		.if_index = 0,
	},

	[YEALINK_XFER_INTR] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = sizeof(struct yealink_intr),
		.flags = {.ext_buffer = 1,.short_xfer_ok = 1,},
		.callback = &yealink_intr_callback,
		.if_index = 0,
	},

	[YEALINK_XFER_ISOC_IN_0] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = YEALINK_BUFSIZE,
		.frames = YEALINK_MINFRAMES,
		.flags = {.short_xfer_ok = 1,.ext_buffer = 1,},
		.callback = &yealink_isoc_read_callback,
		.if_index = 1,
	},

	[YEALINK_XFER_ISOC_IN_1] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = YEALINK_BUFSIZE,
		.frames = YEALINK_MINFRAMES,
		.flags = {.short_xfer_ok = 1,.ext_buffer = 1,},
		.callback = &yealink_isoc_read_callback,
		.if_index = 1,
	},

	[YEALINK_XFER_ISOC_OUT_0] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.bufsize = YEALINK_BUFSIZE,
		.frames = YEALINK_MINFRAMES,
		.flags = {.ext_buffer = 1,},
		.callback = &yealink_isoc_write_callback,
		.if_index = 2,
	},

	[YEALINK_XFER_ISOC_OUT_1] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.bufsize = YEALINK_BUFSIZE,
		.frames = YEALINK_MINFRAMES,
		.flags = {.ext_buffer = 1,},
		.callback = &yealink_isoc_write_callback,
		.if_index = 2,
	},
};

static int
yealink_set_char(struct yealink_softc *sc, uint8_t pos, char c)
{
	uint8_t i;
	uint8_t a;
	uint8_t m;
	uint8_t val;

	if (pos >= YEALINK_LCD_LINE5_OFFSET)
		return (EINVAL);

	if ((c == '\t') || (c == '\n') || (c == '\r'))
		return (0);

	sc->sc_lcd_map[pos] = c;

	if (yealink_lcd_map[pos].type == '.') {

		/* get mask and address from map */

		a = yealink_lcd_map[pos].u.p.a;
		m = yealink_lcd_map[pos].u.p.m;

		if (c != ' ')
			sc->sc_status.raw[a] |= m;
		else
			sc->sc_status.raw[a] &= ~m;

		return (0);		/* success */
	}
	if ((c >= '0') && (c <= '9'))
		val = yealink_digitmap[c - '0'];
	else
		val = 0;

	for (i = 0; i != 7; i++) {

		a = yealink_lcd_map[pos].u.s[i].a;
		m = yealink_lcd_map[pos].u.s[i].m;

		if (val & 1)
			sc->sc_status.raw[a] |= m;
		else
			sc->sc_status.raw[a] &= ~m;

		val = val >> 1;
	}
	return (0);			/* success */
}

static char
yealink_p1k_to_ascii(uint8_t scancode)
{
	;				/* indent fix */
#if 0
	switch (scancode) {		/* Phone keys: */
	case 0x23:
		return (KEY_LEFT);	/* IN */
	case 0x33:
		return (KEY_UP);	/* up */
	case 0x04:
		return (KEY_RIGHT);	/* OUT */
	case 0x24:
		return (KEY_DOWN);	/* down   */
	case 0x03:
		return (KEY_ENTER);	/* pickup   */
	case 0x14:
		return (KEY_BACKSPACE);	/* C */
	case 0x13:
		return (KEY_ESC);	/* hangup   */
	case 0x00:
		return (KEY_1);		/* 1 */
	case 0x01:
		return (KEY_2);		/* 2 */
	case 0x02:
		return (KEY_3);		/* 3 */
	case 0x10:
		return (KEY_4);		/* 4 */
	case 0x11:
		return (KEY_5);		/* 5 */
	case 0x12:
		return (KEY_6);		/* 6 */
	case 0x20:
		return (KEY_7);		/* 7 */
	case 0x21:
		return (KEY_8);		/* 8 */
	case 0x22:
		return (KEY_9);		/* 9 */
	case 0x30:
		return (KEY_KPASTERISK);/* * */
	case 0x31:
		return (KEY_0);		/* 0 */
	case 0x32:
		return (KEY_LEFTSHIFT |
		    KEY_3 << 8);	/* # */
	}
#endif
	return (0);
}

static void
yealink_cmd(struct yealink_softc *sc, struct yealink_ctl_packet *p)
{
	struct usb_device_request req;

	uint8_t i;
	uint8_t sum = 0;

	for (i = 0; i < (YEALINK_PKT_LEN - 1); i++)
		sum -= p->raw[i];
	p->sum = sum;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UR_SET_CONFIG;
	USETW(req.wValue, 0x0200);
	req.wIndex[0] = sc->sc_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, sizeof(*p));

	usbd_do_request_flags(sc->sc_udev, NULL,
	    &req, p, 0, 0, 1000 /* ms */ );
}

static int
yealink_set_ringtone(struct yealink_softc *sc, const uint8_t *buf, uint8_t size)
{
	struct yealink_ctl_packet pkt;
	uint8_t offset;
	uint8_t len;

	if (size <= 0)
		return (-EINVAL);

	memset(&pkt, 0, sizeof(pkt));

	pkt.cmd = YEALINK_CMD_RING_VOLUME;
	pkt.size = 1;
	pkt.data[0] = buf[0];
	yealink_cmd(sc, &pkt);

	buf++;
	size--;

	pkt.cmd = YEALINK_CMD_RING_NOTE;

	offset = 0;
	while (offset != size) {

		len = size - offset;
		if (len > sizeof(pkt.data))
			len = sizeof(pkt.data);
		pkt.size = len;
		pkt.offset[1] = offset;	/* big endian offset */
		memcpy(pkt.data, buf + offset, len);
		yealink_cmd(sc, &pkt);
		offset += len;
	}
	return (0);
}

static uint8_t
yealink_do_idle_tasks(struct yealink_softc *sc)
{
	uint8_t val;
	uint8_t offset;
	uint8_t i;
	uint8_t len;

	offset = sc->sc_curr_offset;

	memset(&sc->sc_ctrl.data, 0, sizeof(sc->sc_ctrl.data));

	sc->sc_ctrl.data.cmd = YEALINK_CMD_KEYPRESS;
	sc->sc_ctrl.data.size = 1;
	sc->sc_ctrl.data.sum =
	    (uint8_t)(0 - 1 - YEALINK_CMD_KEYPRESS);

	if (offset >= sizeof(sc->sc_status)) {
		sc->sc_curr_offset = 0;
		return (0);
	}
	for (; offset < sizeof(sc->sc_status); offset++) {
		val = sc->sc_status.raw[offset];
		if (val != sc->sc_shadow.raw[offset])
			goto do_update;
	}

	sc->sc_curr_offset = 0;
	return (1);

do_update:

	printf("Update %d\n", offset);

	/* update shadow register(s) */
	sc->sc_shadow.raw[offset] = val;
	sc->sc_ctrl.data.data[0] = val;

	switch (offset) {
	case 24:
		sc->sc_ctrl.data.cmd = YEALINK_CMD_LED;
		sc->sc_ctrl.data.sum =
		    (uint8_t)(0 - 1 - YEALINK_CMD_LED - val);
		break;
	case 25:
		sc->sc_ctrl.data.cmd = YEALINK_CMD_DIALTONE;
		sc->sc_ctrl.data.sum =
		    (uint8_t)(0 - 1 - YEALINK_CMD_DIALTONE - val);
		break;
	case 26:
		sc->sc_ctrl.data.cmd = YEALINK_CMD_RINGTONE;
		sc->sc_ctrl.data.sum =
		    (uint8_t)(0 - 1 - YEALINK_CMD_RINGTONE - val);
		break;
	case 27:
		val--;
		val &= 0x1f;
		sc->sc_ctrl.data.cmd = YEALINK_CMD_SCANCODE;
		sc->sc_ctrl.data.offset[1] = val;
		sc->sc_ctrl.data.data[0] = 0;
		sc->sc_ctrl.data.sum =
		    (uint8_t)(0 - 1 - YEALINK_CMD_SCANCODE - val);
		break;
	default:
		len = sizeof(sc->sc_status.lcd) - offset;
		if (len > sizeof(sc->sc_ctrl.data.data))
			len = sizeof(sc->sc_ctrl.data.data);

		sc->sc_ctrl.data.cmd = YEALINK_CMD_LCD;
		sc->sc_ctrl.data.offset[1] = offset;	/* big endian */
		sc->sc_ctrl.data.size = len;
		sc->sc_ctrl.data.sum =
		    (uint8_t)(0 - YEALINK_CMD_LCD - offset - val - len);

		for (i = 1; i < len; i++) {
			offset++;
			val = sc->sc_status.raw[offset];
			sc->sc_shadow.raw[offset] = val;
			sc->sc_ctrl.data.data[i] = val;
			sc->sc_ctrl.data.sum -= val;
		}
	}
	sc->sc_curr_offset = offset + 1;
	return (0);
}

static void
yealink_intr_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct yealink_softc *sc;
	int actlen;
	uint8_t val;

	sc = usbd_xfer_softc(xfer);

	DPRINTFN(0, "\n");

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

		if (actlen >= sizeof(sc->sc_intr.data)) {

			val = sc->sc_intr.data.data[0];

			switch (sc->sc_intr.data.cmd) {
			case YEALINK_CMD_KEYPRESS:
				DPRINTFN(0, "Keypress 0x%02x\n", val);
				sc->sc_status.keynum = val;
				break;

			case YEALINK_CMD_SCANCODE:
				DPRINTFN(0, "Scancode 0x%02x\n", val);
				yealink_p1k_to_ascii(val);
				break;
			default:
				DPRINTFN(1, "Unexpected "
				    "response 0x%02x\n", val);
				break;
			}
		}
		/* start control transfer again */
		usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_CTRL]);
		break;

	case USB_ST_SETUP:
tr_setup:
		/* setup USB transfer */
		usbd_xfer_set_frame_data(xfer, 0,
		    &sc->sc_intr, YEALINK_INTR_BUF_SIZE);
		usbd_xfer_set_frames(xfer, 1);
		usbd_transfer_submit(xfer);
		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		break;
	}
}

static void
yealink_ctrl_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct yealink_softc *sc;
	uint8_t i;

	sc = usbd_xfer_softc(xfer);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		switch (sc->sc_ctrl.data.cmd) {
		case YEALINK_CMD_KEYPRESS:
		case YEALINK_CMD_SCANCODE:
			/* get data on interrupt endpoint */
			usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_INTR]);
			//return;
		default:
			break;
		}

	case USB_ST_SETUP:
tr_setup:
		if (sc->sc_was_opened) {
			memset(&sc->sc_ctrl.data, 0, sizeof(sc->sc_ctrl.data));
			sc->sc_ctrl.data.cmd = YEALINK_CMD_INIT;
			sc->sc_ctrl.data.size = 10;
			sc->sc_ctrl.data.sum =
			    (uint8_t)(0 - YEALINK_CMD_INIT - 10);
			sc->sc_curr_offset = 0;
			sc->sc_was_opened--;

			/* make sure all data is re-loaded */
			for (i = 0; i != sizeof(sc->sc_status); i++)
				sc->sc_shadow.raw[i] = ~sc->sc_status.raw[i];

			/* no pre-transfer delay */
			usbd_xfer_set_interval(xfer, 0 /* ms */ );
		} else {

			if (yealink_do_idle_tasks(sc))
				usbd_xfer_set_interval(xfer, 25 /* ms */ );
			else
				usbd_xfer_set_interval(xfer, 0 /* ms */ );
		}

		sc->sc_ctrl.req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
		sc->sc_ctrl.req.bRequest = UR_SET_CONFIG;
		USETW(sc->sc_ctrl.req.wValue, 0x0200);
		sc->sc_ctrl.req.wIndex[0] = sc->sc_iface_no;
		sc->sc_ctrl.req.wIndex[1] = 0;
		USETW(sc->sc_ctrl.req.wLength, YEALINK_PKT_LEN);

		/* setup USB transfer */
		usbd_xfer_set_frame_data(xfer, 0,
		    &sc->sc_ctrl.req, sizeof(sc->sc_ctrl.req));
		usbd_xfer_set_frame_data(xfer, 1,
		    &sc->sc_ctrl.data, sizeof(sc->sc_ctrl.data));
		usbd_xfer_set_frames(xfer, 2);
		usbd_transfer_submit(xfer);
		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED) {
			goto tr_setup;
		}
		break;
	}
}

static void
yealink_isoc_read_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct yealink_softc *sc = usbd_xfer_softc(xfer);
	struct dss1_lite_fifo *f = &sc->sc_dl.dl_fifo[sc->sc_dl.dl_audio_channel];
	uint8_t *buf;
	uint16_t i;
	uint16_t j;
	uint16_t k;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:

		buf = usbd_xfer_get_priv(xfer);

		for (i = 0; i != YEALINK_MINFRAMES; i++) {

			k = usbd_xfer_frame_len(xfer, i) & -2UL;

			for (j = 0; j < k; j += 2) {
				dss1_lite_l5_put_sample(&sc->sc_dl, f, UGETW(buf + j));
			}

			for (; j < YEALINK_BPF; j += 2) {
				dss1_lite_l5_put_sample(&sc->sc_dl, f, f->m_tx_last_sample);
			}
		}

	case USB_ST_SETUP:
tr_setup:

		buf = usbd_xfer_get_priv(xfer);

		/* setup USB transfer */
		usbd_xfer_set_frame_data(xfer, 0, buf, 0);

		for (i = 0; i != YEALINK_MINFRAMES; i++) {
			usbd_xfer_set_frame_len(xfer, i, YEALINK_BPF);
		}
		usbd_xfer_set_frames(xfer, YEALINK_MINFRAMES);

		usbd_transfer_submit(xfer);
		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED) {
			goto tr_setup;
		}
		break;
	}
}

static void
yealink_isoc_write_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct yealink_softc *sc = usbd_xfer_softc(xfer);
	struct dss1_lite_fifo *f = &sc->sc_dl.dl_fifo[sc->sc_dl.dl_audio_channel];
	uint8_t *buf;
	uint16_t i;
	uint16_t temp;


	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
	case USB_ST_SETUP:
tr_setup:
		buf = usbd_xfer_get_priv(xfer);

		for (i = 0; i != YEALINK_BUFSIZE; i += 2) {
			temp = dss1_lite_l5_get_sample(&sc->sc_dl, f);
			USETW(buf + i, temp);
		}

		/* setup USB transfer */
		usbd_xfer_set_frame_data(xfer, 0, buf, 0);

		for (i = 0; i != YEALINK_MINFRAMES; i++) {
			usbd_xfer_set_frame_len(xfer, i, YEALINK_BPF);
		}
		usbd_xfer_set_frames(xfer, YEALINK_MINFRAMES);

		usbd_transfer_submit(xfer);
		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED) {
			goto tr_setup;
		}
		break;
	}
}

#if 0
static void
yealink_store_line(struct yealink_softc *sc, const char *buf,
    uint8_t count, uint8_t pos, uint8_t len)
{
	uint8_t i;

	if (len > count)
		len = count;
	for (i = 0; i < len; i++, pos++)
		yealink_set_char(sc, pos, buf[i]);

}

#endif

static void
yealink_set_icon(struct yealink_softc *sc, const char *buf,
    uint8_t count, char c)
{
	uint8_t i;

	for (i = 0; i != YEALINK_LCD_LINE5_OFFSET; i++) {
		if (yealink_lcd_map[i].type != '.')
			continue;
		if (strncmp(buf, yealink_lcd_map[i].u.p.desc, count) == 0) {
			yealink_set_char(sc, i, c);
			break;
		}
	}
}

static int
yealink_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (ENXIO);
	}
	if (uaa->info.bConfigIndex != YEALINK_CONFIG_INDEX) {
		return (ENXIO);
	}
	if (uaa->info.bIfaceIndex != YEALINK_IFACE_INDEX) {
		return (ENXIO);
	}
	if (usbd_get_speed(uaa->device) != USB_SPEED_FULL) {
		return (ENXIO);
	}
	return (usbd_lookup_id_by_uaa(yealink_devs_table,
	    sizeof(yealink_devs_table), uaa));
}

static int
yealink_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct yealink_softc *sc = device_get_softc(dev);
	struct usb_interface *iface = NULL;
	struct usb_interface_descriptor *idesc = NULL;
	struct usb_endpoint_descriptor *ed = NULL;
	struct i4b_controller *ctrl;
	uint8_t i;
	uint8_t iface_index[3];

	ctrl = i4b_controller_allocate(1, 1, 4, NULL);
	if (ctrl == NULL) {
		device_printf(dev, "Could not allocate I4B controller.\n");
		return (ENXIO);
	}
	sc->sc_pmtx = CNTL_GET_LOCK(ctrl);

	//device_set_usb_desc(dev);

	device_set_desc(dev, "VoIP adapter");

	sc->sc_udev = uaa->device;

	iface_index[0] = 0;
	iface_index[1] = 0;
	iface_index[2] = 0;

	/* claim all interfaces */

	for (i = 0;; i++) {
		if (i == iface_index[0])
			continue;
		iface = usbd_get_iface(uaa->device, i);
		if (iface == NULL)
			break;
		idesc = usbd_get_interface_descriptor(iface);
		if (idesc == NULL)
			continue;

		usbd_set_parent_iface(uaa->device, i, iface_index[0]);

		if (idesc->bInterfaceClass == UICLASS_AUDIO) {
			if (idesc->bInterfaceSubClass == UISUBCLASS_AUDIOSTREAM) {
				usbd_set_alt_interface_index(uaa->device, i, 1);
				ed = usbd_find_descriptor(uaa->device,
				    NULL, i, UDESC_ENDPOINT, 0xFF, 0, 0);
				if (ed != NULL) {
					if (ed->bEndpointAddress & UE_DIR_IN)
						iface_index[1] = i;
					else
						iface_index[2] = i;
				}
			}
			if (idesc->bInterfaceSubClass == UISUBCLASS_AUDIOCONTROL) {
				/* Nothing to do */
			}
		}
		if (idesc->bInterfaceClass == UICLASS_HID) {
			iface_index[0] = i;
			sc->sc_iface_no = idesc->bInterfaceNumber;
		}
	}

	DPRINTF("iface_no=%d\n", sc->sc_iface_no);

	for (i = 0; i != YEALINK_LCD_LINE5_OFFSET; i++) {
		yealink_set_char(sc, i, ' ');
	}

	yealink_set_ringtone(sc, yealink_ringtone, sizeof(yealink_ringtone));

	if (usbd_transfer_setup(sc->sc_udev, iface_index, sc->sc_xfer,
	    yealink_config, YEALINK_XFER_MAX, sc, sc->sc_pmtx)) {
		DPRINTF("could not allocate USB transfers!\n");
		goto error;
	}
	usbd_xfer_set_priv(sc->sc_xfer[YEALINK_XFER_ISOC_IN_0], sc->sc_buffer + (YEALINK_BUFSIZE * 0));
	usbd_xfer_set_priv(sc->sc_xfer[YEALINK_XFER_ISOC_IN_1], sc->sc_buffer + (YEALINK_BUFSIZE * 1));
	usbd_xfer_set_priv(sc->sc_xfer[YEALINK_XFER_ISOC_OUT_0], sc->sc_buffer + (YEALINK_BUFSIZE * 2));
	usbd_xfer_set_priv(sc->sc_xfer[YEALINK_XFER_ISOC_OUT_1], sc->sc_buffer + (YEALINK_BUFSIZE * 3));

	mtx_lock(sc->sc_pmtx);
	sc->sc_was_opened = 1;
	usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_CTRL]);
	//usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_INTR]);
	mtx_unlock(sc->sc_pmtx);

	//yealink_set_icon(sc, "RINGTONE", sizeof("RINGTONE") - 1, 'x');
	//yealink_set_icon(sc, "D", sizeof("D") - 1, 'x');
	//yealink_set_icon(sc, "M", sizeof("M") - 1, 'x');
	//yealink_set_icon(sc, "DIALTONE", sizeof("DIALTONE") - 1, 'x');
	//yealink_set_icon(sc, "IN", sizeof("IN") - 1, 'x');
	yealink_set_icon(sc, "LED", sizeof("LED") - 1, 'x');
	//yealink_set_char(sc, YEALINK_LCD_LINE3_OFFSET, '9');

	if (dss1_lite_attach(&sc->sc_dl, dev,
	    ctrl, &yealink_dl_methods)) {
		device_printf(dev, "DSS1 lite attach failed\n");
		goto error;
	}
	return (0);

error:
	i4b_controller_free(ctrl, 1);

	yealink_detach(dev);

	return (ENXIO);
}

static int
yealink_detach(device_t dev)
{
	struct yealink_softc *sc = device_get_softc(dev);

	usbd_transfer_unsetup(sc->sc_xfer, YEALINK_XFER_MAX);

	dss1_lite_detach(&sc->sc_dl);

	return (0);			/* success */
}
