/*-
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
 *      Yealink (USB-P1K) driver for ISDN4BSD
 *      -------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

/*
 * NOTE: The following documents have been used as hardware reference:
 *
 *   - yealink.c driver in the Linux Kernel
 *   - the usbb2k-api   http://savannah.nongnu.org/projects/usbb2k-api/
 *   - information from http://memeteau.free.fr/usbb2k
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
MODULE_DEPEND(yealink, ihfcpnp, 1, 1, 1);

static dss1_lite_set_ring_t yealink_set_ring;
static dss1_lite_set_protocol_t yealink_set_protocol;

static const struct dss1_lite_methods yealink_dl_methods = {
	DSS1_LITE_DEFAULT_METHODS,
	.set_ring = yealink_set_ring,
	.set_protocol = yealink_set_protocol,
	.support_echo_cancel = 1,
};

/* static structures */

/*
 * The last byte in the Yealink commands is the negative sum of all
 * the prior command bytes!
 */

static const uint8_t yealink_commands[YEALINK_ST_MAX][YEALINK_PKT_LEN] = {
	[YEALINK_ST_INIT] = {0x8e, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68},
	[YEALINK_ST_USB_ON] = {0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF1},
	[YEALINK_ST_USB_OFF] = {0x0E, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0},
	[YEALINK_ST_LED_ON] = {0x05, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA},
	[YEALINK_ST_LED_OFF] = {0x05, 0x02, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA},
	[YEALINK_ST_TONE_ON] = {0x09, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF5},
	[YEALINK_ST_TONE_OFF] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF6},
	[YEALINK_ST_RING_ON] = {0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFd},
	[YEALINK_ST_RING_OFF] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFe},
	[YEALINK_ST_HANDSET_QUERY] = {0x8d, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72},
	[YEALINK_ST_KEY_QUERY] = {0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F},
	[YEALINK_ST_KEY_CODE] = {0x81, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e},
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
		.timeout = 1000,	/* 1 second */
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

static void
yealink_p1k_to_ascii(struct yealink_softc *sc, uint8_t what)
{
	;				/* indent fix */
	switch (what) {
	case 0x00:
		dss1_lite_dtmf_event(&sc->sc_dl, "0");
		break;
	case 0x01:
		dss1_lite_dtmf_event(&sc->sc_dl, "1");
		break;
	case 0x02:
		dss1_lite_dtmf_event(&sc->sc_dl, "2");
		break;
	case 0x03:
		dss1_lite_dtmf_event(&sc->sc_dl, "3");
		break;
	case 0x04:
		dss1_lite_dtmf_event(&sc->sc_dl, "4");
		break;
	case 0x05:
		dss1_lite_dtmf_event(&sc->sc_dl, "5");
		break;
	case 0x06:
		dss1_lite_dtmf_event(&sc->sc_dl, "6");
		break;
	case 0x07:
		dss1_lite_dtmf_event(&sc->sc_dl, "7");
		break;
	case 0x08:
		dss1_lite_dtmf_event(&sc->sc_dl, "8");
		break;
	case 0x09:
		dss1_lite_dtmf_event(&sc->sc_dl, "9");
		break;
	case 0x0b:
		dss1_lite_dtmf_event(&sc->sc_dl, "*");
		break;
	case 0x0c:
		dss1_lite_dtmf_event(&sc->sc_dl, "#");
		break;
	default:
		break;
	}
	return;
}

static void
yealink_init(struct yealink_softc *sc, uint8_t on)
{
	sc->sc_st_data[YEALINK_ST_INIT] = on;
	sc->sc_st_index = 0;
}

static void
yealink_set_usb(struct yealink_softc *sc, uint8_t on)
{
	sc->sc_st_data[YEALINK_ST_USB_ON] = 0;
	sc->sc_st_data[YEALINK_ST_USB_OFF] = 0;

	if (on)
		sc->sc_st_data[YEALINK_ST_USB_ON] = 1;
	else
		sc->sc_st_data[YEALINK_ST_USB_OFF] = 1;
}

static void
yealink_set_led(struct yealink_softc *sc, uint8_t on)
{
	sc->sc_st_data[YEALINK_ST_LED_ON] = 0;
	sc->sc_st_data[YEALINK_ST_LED_OFF] = 0;

	if (on)
		sc->sc_st_data[YEALINK_ST_LED_ON] = 1;
	else
		sc->sc_st_data[YEALINK_ST_LED_OFF] = 1;
}

static void
yealink_set_tone_st(struct yealink_softc *sc, uint8_t on)
{
	sc->sc_st_data[YEALINK_ST_TONE_ON] = 0;
	sc->sc_st_data[YEALINK_ST_TONE_OFF] = 0;

	if (on)
		sc->sc_st_data[YEALINK_ST_TONE_ON] = 1;
	else
		sc->sc_st_data[YEALINK_ST_TONE_OFF] = 1;
}

static void
yealink_set_ring_st(struct yealink_softc *sc, uint8_t on)
{
	sc->sc_st_data[YEALINK_ST_RING_ON] = 0;
	sc->sc_st_data[YEALINK_ST_RING_OFF] = 0;

	if (on)
		sc->sc_st_data[YEALINK_ST_RING_ON] = 1;
	else
		sc->sc_st_data[YEALINK_ST_RING_OFF] = 1;
}

static void
yealink_handset_query(struct yealink_softc *sc, uint8_t on)
{
	sc->sc_st_data[YEALINK_ST_HANDSET_QUERY] = on;
}

static void
yealink_key_query(struct yealink_softc *sc, uint8_t on)
{
	sc->sc_st_data[YEALINK_ST_KEY_QUERY] = on;
}

static void
yealink_key_code(struct yealink_softc *sc, uint8_t on)
{
	sc->sc_st_data[YEALINK_ST_KEY_CODE] = on;
}

static void
yealink_set_ring(struct dss1_lite *pdl, uint8_t on)
{
	struct yealink_softc *sc = pdl->dl_softc;

	yealink_set_tone_st(sc, 0);
	yealink_set_ring_st(sc, on);
}

static void
yealink_set_protocol(struct dss1_lite *pdl,
    struct dss1_lite_fifo *f, struct i4b_protocol *p)
{
	struct yealink_softc *sc = pdl->dl_softc;
	uint8_t fn;
	uint8_t enable;

	fn = f - pdl->dl_fifo;

	enable = (p->protocol_1 != P_DISABLE);

	if (fn == 0) {
		yealink_set_usb(sc, enable);
	} else {
		yealink_set_led(sc, enable);
		if (enable) {
			usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_ISOC_IN_0]);
			usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_ISOC_IN_1]);
			usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_ISOC_OUT_0]);
			usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_ISOC_OUT_1]);
		} else {
			usbd_transfer_stop(sc->sc_xfer[YEALINK_XFER_ISOC_IN_0]);
			usbd_transfer_stop(sc->sc_xfer[YEALINK_XFER_ISOC_IN_1]);
			usbd_transfer_stop(sc->sc_xfer[YEALINK_XFER_ISOC_OUT_0]);
			usbd_transfer_stop(sc->sc_xfer[YEALINK_XFER_ISOC_OUT_1]);
		}
	}
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

static void
yealink_set_mixer(struct yealink_softc *sc, uint16_t wIndex,
    uint16_t wValue, uint16_t wLength, uint16_t vol)
{
	struct usb_device_request req;
	uint8_t buf[2];

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = 0x01;		/* SET_CUR */
	USETW(req.wValue, wValue);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, wLength);
	USETW(buf, vol);

	usbd_do_request_flags(sc->sc_udev, NULL,
	    &req, buf, 0, 0, 1000 /* ms */ );
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

static void
yealink_intr_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct yealink_softc *sc;
	int actlen;
	uint8_t val;

	sc = usbd_xfer_softc(xfer);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

		DPRINTFN(15, "%d bytes\n", actlen);

		if (actlen >= sizeof(sc->sc_intr.data)) {

			val = sc->sc_intr.data.data[0];

			switch (sc->sc_ctrl.data.cmd) {
			case YEALINK_CMD_KEYPRESS:
				if (sc->sc_key_state != val) {
					DPRINTFN(1, "Keypress 0x%02x\n", val);
					sc->sc_key_state = val;
					yealink_key_code(sc, 1);
				}
				break;

			case YEALINK_CMD_SCANCODE:
				DPRINTFN(1, "Scancode 0x%02x\n", val);
				yealink_p1k_to_ascii(sc, val);
				break;

			case YEALINK_CMD_HANDSET_QUERY:
				if (val == 0x02) {
					/* off hook */
					if (sc->sc_hook_state != 2) {
						sc->sc_hook_state = 2;
						dss1_lite_hook_off(&sc->sc_dl);
					}
				} else {
					/* on hook */
					if (sc->sc_hook_state != 1) {
						sc->sc_hook_state = 1;
						dss1_lite_hook_on(&sc->sc_dl);
					}
				}
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
		/* setup USB transfer */
		usbd_xfer_set_frame_data(xfer, 0,
		    &sc->sc_intr, YEALINK_PKT_LEN);
		usbd_xfer_set_frames(xfer, 1);
		usbd_transfer_submit(xfer);
		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			/* start control transfer again */
			usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_CTRL]);
		}
		break;
	}
}

static void
yealink_ctrl_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct yealink_softc *sc;
	uint8_t i;
	uint8_t j;

	sc = usbd_xfer_softc(xfer);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		/* get data on interrupt endpoint */
		usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_INTR]);
		break;

	case USB_ST_SETUP:
tr_setup:
		yealink_handset_query(sc, 1);
		yealink_key_query(sc, 1);

		while (1) {

			if (sc->sc_st_index >= YEALINK_ST_MAX)
				sc->sc_st_index = 0;

			i = sc->sc_st_index;
			sc->sc_st_index++;

			if (sc->sc_st_data[i] != 0) {
				sc->sc_st_data[i] = 0;

				memcpy(sc->sc_ctrl.data.raw,
				    yealink_commands[i],
				    YEALINK_PKT_LEN);

				if (i == YEALINK_ST_KEY_CODE) {
					/* need to ask for specific key */

					j = sc->sc_key_state;
					if (j == 0)
						j = 0x7E;
					else
						j--;

					sc->sc_ctrl.data.offset[1] = j;
					sc->sc_ctrl.data.sum -= j;
				}
				break;
			}
		}

		if (i == YEALINK_ST_KEY_QUERY)
			usbd_xfer_set_interval(xfer, 25 /* ms */ );
		else
			usbd_xfer_set_interval(xfer, 0 /* ms */ );

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

		/* make sure the DSS1 code gets a chance to run */
		dss1_lite_process(&sc->sc_dl);
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

		f->tx_timestamp = (usbd_xfer_get_timestamp(xfer) * 8);

		buf = usbd_xfer_get_priv(xfer);

		for (i = 0; i != YEALINK_MINFRAMES; i++) {

			k = usbd_xfer_frame_len(xfer, i) & -2UL;

			for (j = 0; j < k; j += 2) {
				dss1_lite_l5_put_sample(&sc->sc_dl, f, (int16_t)UGETW(buf + j));
			}

			for (; j < YEALINK_BPF; j += 2) {
				dss1_lite_l5_put_sample(&sc->sc_dl, f, (int16_t)f->m_tx_last_sample);
			}

			buf += YEALINK_BPF;
		}

		dss1_lite_l5_put_sample_complete(&sc->sc_dl, f);

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
	uint16_t timestamp = (usbd_xfer_get_timestamp(xfer) * 8) + YEALINK_BUFSIZE;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
	case USB_ST_SETUP:
tr_setup:
		buf = usbd_xfer_get_priv(xfer);

		for (i = 0; i != YEALINK_BUFSIZE; i += 2) {
			temp = dss1_lite_l5_get_sample(&sc->sc_dl, f);
			USETW(buf + i, temp);
		}

		dss1_lite_l5_get_sample_complete(&sc->sc_dl, f);

		/* setup USB transfer */
		usbd_xfer_set_frame_data(xfer, 0, buf, 0);

		for (i = 0; i != YEALINK_MINFRAMES; i++) {
			usbd_xfer_set_frame_len(xfer, i, YEALINK_BPF);
		}
		usbd_xfer_set_frames(xfer, YEALINK_MINFRAMES);

		usbd_transfer_submit(xfer);

		/* set timestamp for end next transfer */

		f->rx_timestamp = timestamp;

		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED) {
			goto tr_setup;
		}
		break;
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

	device_set_desc(dev, "USB Phone Adapter");

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

	yealink_set_ringtone(sc, yealink_ringtone, sizeof(yealink_ringtone));

	if (usbd_transfer_setup(sc->sc_udev, iface_index, sc->sc_xfer,
	    yealink_config, YEALINK_XFER_MAX, sc, sc->sc_pmtx)) {
		DPRINTF("could not allocate USB transfers!\n");
		goto error;
	}
	usbd_xfer_set_priv(sc->sc_xfer[YEALINK_XFER_ISOC_IN_0],
	    sc->sc_buffer + (YEALINK_BUFSIZE * 0));
	usbd_xfer_set_priv(sc->sc_xfer[YEALINK_XFER_ISOC_IN_1],
	    sc->sc_buffer + (YEALINK_BUFSIZE * 1));
	usbd_xfer_set_priv(sc->sc_xfer[YEALINK_XFER_ISOC_OUT_0],
	    sc->sc_buffer + (YEALINK_BUFSIZE * 2));
	usbd_xfer_set_priv(sc->sc_xfer[YEALINK_XFER_ISOC_OUT_1],
	    sc->sc_buffer + (YEALINK_BUFSIZE * 3));

	yealink_init(sc, 1);
	yealink_set_usb(sc, 0);
	yealink_set_led(sc, 0);
	yealink_set_tone_st(sc, 0);
	yealink_set_ring_st(sc, 0);

	sc->sc_dl.dl_softc = sc;

	if (dss1_lite_attach(&sc->sc_dl, dev,
	    ctrl, &yealink_dl_methods)) {
		device_printf(dev, "DSS1 lite attach failed\n");
		goto error;
	}
	mtx_lock(sc->sc_pmtx);
	usbd_transfer_start(sc->sc_xfer[YEALINK_XFER_CTRL]);
	mtx_unlock(sc->sc_pmtx);

	/* Wait for init */

	pause("WMIX", hz / 16);

	yealink_set_mixer(sc, 0x0500, 0x0200, 2, 0xbf40);
	yealink_set_mixer(sc, 0x0600, 0x0200, 2, 0xbf40);

	pause("WMIX", hz / 16);

	/* Set Audio Volume */

	yealink_set_mixer(sc, 0x0500, 0x0200, 2, 0xbf3f);
 	yealink_set_mixer(sc, 0x0600, 0x0200, 2, 0x7eff);

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
