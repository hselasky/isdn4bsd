/*-
 * Copyright (c) 2004 Dag-Erling Coïdan Smørgrav
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in this position and unchanged.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Device driver for Cypress CY7C637xx and CY7C640/1xx series USB to
 * RS232 bridges.
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_hid.h>

#include "usbdevs.h"

#include <dev/usb/ucomvar.h>

__FBSDID("$FreeBSD: src/sys/dev/usb/ucycom.c,v 1.6 2007/06/21 14:42:33 imp Exp $");

#define	UCYCOM_MAX_IOLEN	(1024 + 2)	/* bytes */

#define	UCYCOM_ENDPT_MAX	3	/* units */
#define	UCYCOM_IFACE_INDEX	0

#define	DPRINTF(...) do { } while (0)

struct ucycom_softc {
	struct ucom_super_softc sc_super_ucom;
	struct ucom_softc sc_ucom;

	struct usbd_device *sc_udev;
	struct usbd_xfer *sc_xfer[UCYCOM_ENDPT_MAX];

	uint32_t sc_model;
#define	MODEL_CY7C63743		0x63743
#define	MODEL_CY7C64013		0x64013

	uint16_t sc_flen;		/* feature report length */
	uint16_t sc_ilen;		/* input report length */
	uint16_t sc_olen;		/* output report length */

	uint8_t	sc_fid;			/* feature report id */
	uint8_t	sc_iid;			/* input report id */
	uint8_t	sc_oid;			/* output report id */
	uint8_t	sc_cfg;
#define	UCYCOM_CFG_RESET	0x80
#define	UCYCOM_CFG_PARODD	0x20
#define	UCYCOM_CFG_PAREN	0x10
#define	UCYCOM_CFG_STOPB	0x08
#define	UCYCOM_CFG_DATAB	0x03
	uint8_t	sc_ist;			/* status flags from last input */
	uint8_t	sc_flags;
#define	UCYCOM_FLAG_INTR_STALL     0x01
	uint8_t	sc_name[16];
	uint8_t	sc_iface_no;
	uint8_t	sc_temp_cfg[32];
};

/* prototypes */

static device_probe_t ucycom_probe;
static device_attach_t ucycom_attach;
static device_detach_t ucycom_detach;

static usbd_callback_t ucycom_ctrl_write_callback;
static usbd_callback_t ucycom_intr_read_clear_stall_callback;
static usbd_callback_t ucycom_intr_read_callback;

static void ucycom_cfg_open(struct ucom_softc *ucom);
static void ucycom_start_read(struct ucom_softc *ucom);
static void ucycom_stop_read(struct ucom_softc *ucom);
static void ucycom_start_write(struct ucom_softc *ucom);
static void ucycom_stop_write(struct ucom_softc *ucom);
static void ucycom_cfg_write(struct ucycom_softc *sc, uint32_t baud, uint8_t cfg);
static int ucycom_pre_param(struct ucom_softc *ucom, struct termios *t);
static void ucycom_cfg_param(struct ucom_softc *ucom, struct termios *t);

static const struct usbd_config ucycom_config[UCYCOM_ENDPT_MAX] = {

	[0] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = (sizeof(usb_device_request_t) + UCYCOM_MAX_IOLEN),
		.mh.flags = {},
		.mh.callback = &ucycom_ctrl_write_callback,
		.mh.timeout = 1000,	/* 1 second */
	},

	[1] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.bufsize = UCYCOM_MAX_IOLEN,
		.mh.callback = &ucycom_intr_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &ucycom_intr_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.interval = 50,		/* 50ms */
	},
};

static const struct ucom_callback ucycom_callback = {
	.ucom_cfg_param = &ucycom_cfg_param,
	.ucom_cfg_open = &ucycom_cfg_open,
	.ucom_pre_param = &ucycom_pre_param,
	.ucom_start_read = &ucycom_start_read,
	.ucom_stop_read = &ucycom_stop_read,
	.ucom_start_write = &ucycom_start_write,
	.ucom_stop_write = &ucycom_stop_write,
};

static device_method_t ucycom_methods[] = {
	DEVMETHOD(device_probe, ucycom_probe),
	DEVMETHOD(device_attach, ucycom_attach),
	DEVMETHOD(device_detach, ucycom_detach),
	{0, 0}
};

static devclass_t ucycom_devclass;

static driver_t ucycom_driver = {
	.name = "ucycom",
	.methods = ucycom_methods,
	.size = sizeof(struct ucycom_softc),
};

DRIVER_MODULE(ucycom, uhub, ucycom_driver, ucycom_devclass, usbd_driver_load, 0);
MODULE_VERSION(ucycom, 1);
MODULE_DEPEND(ucycom, usb, 1, 1, 1);

/*
 * Supported devices
 */

struct ucycom_device {
	uint16_t vendor;
	uint16_t product;
	uint32_t model;
};

static const struct ucycom_device ucycom_devices[] = {
	{USB_VENDOR_DELORME, USB_PRODUCT_DELORME_EARTHMATE, MODEL_CY7C64013},
	{0, 0, 0},
};

#define	UCYCOM_DEFAULT_RATE	 4800
#define	UCYCOM_DEFAULT_CFG	 0x03	/* N-8-1 */

static int
ucycom_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	const struct ucycom_device *ud;

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->iface != NULL) {
		return (UMATCH_NONE);
	}
	for (ud = ucycom_devices; ud->model != 0; ++ud) {
		if ((ud->vendor == uaa->vendor) &&
		    (ud->product == uaa->product)) {
			return (UMATCH_VENDOR_PRODUCT);
		}
	}

	return (UMATCH_NONE);
}

static int
ucycom_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ucycom_softc *sc = device_get_softc(dev);
	const struct ucycom_device *ud;
	struct usbd_interface *iface;
	void *urd_ptr = NULL;
	int32_t error;
	uint16_t urd_len;
	uint8_t iface_index;

	if (sc == NULL) {
		return (ENOMEM);
	}
	sc->sc_udev = uaa->device;

	usbd_set_device_desc(dev);

	snprintf(sc->sc_name, sizeof(sc->sc_name),
	    "%s", device_get_nameunit(dev));

	DPRINTF(sc, 0, "\n");

	/* get chip model */

	for (ud = ucycom_devices; ud->model != 0; ++ud) {
		if ((ud->vendor == uaa->vendor) &&
		    (ud->product == uaa->product)) {
			sc->sc_model = ud->model;
		}
	}

	if (sc->sc_model == 0) {
		device_printf(dev, "unsupported device\n");
		goto detach;
	}
	device_printf(dev, "Cypress CY7C%X USB to RS232 bridge\n", sc->sc_model);

	/* select configuration */

	error = usbd_set_config_index(sc->sc_udev, 0, 1 /* verbose */ );

	if (error) {
		device_printf(dev, "failed to select "
		    "configuration: %s\n",
		    usbd_errstr(error));
		goto detach;
	}
	/* get report descriptor */

	error = hid_read_report_desc_from_usb
	    (uaa->device, &usb_global_lock,
	    &urd_ptr, &urd_len, M_USBDEV,
	    UCYCOM_IFACE_INDEX);

	if (error) {
		device_printf(dev, "failed to get report "
		    "descriptor: %s\n",
		    usbd_errstr(error));
		goto detach;
	}
	/* get report sizes */

	sc->sc_flen = hid_report_size(urd_ptr, urd_len, hid_feature, &sc->sc_fid);
	sc->sc_ilen = hid_report_size(urd_ptr, urd_len, hid_input, &sc->sc_iid);
	sc->sc_olen = hid_report_size(urd_ptr, urd_len, hid_output, &sc->sc_oid);

	if ((sc->sc_ilen > UCYCOM_MAX_IOLEN) || (sc->sc_ilen < 1) ||
	    (sc->sc_olen > UCYCOM_MAX_IOLEN) || (sc->sc_olen < 2) ||
	    (sc->sc_flen > UCYCOM_MAX_IOLEN) || (sc->sc_flen < 5)) {
		device_printf(dev, "invalid report size i=%d, o=%d, f=%d, max=%d\n",
		    sc->sc_ilen, sc->sc_olen, sc->sc_flen,
		    UCYCOM_MAX_IOLEN);
		goto detach;
	}
	iface = usbd_get_iface(uaa->device, UCYCOM_IFACE_INDEX);

	if (iface == NULL) {
		device_printf(dev, "no interface!\n");
		goto detach;
	}
	if (iface->idesc == NULL) {
		device_printf(dev, "no interface descriptor!\n");
		goto detach;
	}
	sc->sc_iface_no = iface->idesc->bInterfaceNumber;

	iface_index = UCYCOM_IFACE_INDEX;
	error = usbd_transfer_setup(uaa->device, &iface_index,
	    sc->sc_xfer, ucycom_config, UCYCOM_ENDPT_MAX,
	    sc, &Giant);
	if (error) {
		device_printf(dev, "allocating USB "
		    "transfers failed!\n");
		goto detach;
	}
	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
	    &ucycom_callback, &Giant);

	if (error) {
		goto detach;
	}
	if (urd_ptr) {
		free(urd_ptr, M_USBDEV);
	}
	return (0);			/* success */

detach:
	if (urd_ptr) {
		free(urd_ptr, M_USBDEV);
	}
	ucycom_detach(dev);
	return (ENXIO);
}

static int
ucycom_detach(device_t dev)
{
	struct ucycom_softc *sc = device_get_softc(dev);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UCYCOM_ENDPT_MAX);

	return (0);
}

static void
ucycom_cfg_open(struct ucom_softc *ucom)
{
	struct ucycom_softc *sc = ucom->sc_parent;

	/* set default configuration */
	ucycom_cfg_write(sc, UCYCOM_DEFAULT_RATE, UCYCOM_DEFAULT_CFG);
	return;
}

static void
ucycom_start_read(struct ucom_softc *ucom)
{
	struct ucycom_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
ucycom_stop_read(struct ucom_softc *ucom)
{
	struct ucycom_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
ucycom_start_write(struct ucom_softc *ucom)
{
	struct ucycom_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
ucycom_stop_write(struct ucom_softc *ucom)
{
	struct ucycom_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

static void
ucycom_ctrl_write_callback(struct usbd_xfer *xfer)
{
	struct ucycom_softc *sc = xfer->priv_sc;
	usb_device_request_t req;
	uint8_t data[2];
	uint8_t offset;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
tr_transferred:
	case USBD_ST_SETUP:

		switch (sc->sc_model) {
		case MODEL_CY7C63743:
			offset = 1;
			break;
		case MODEL_CY7C64013:
			offset = 2;
			break;
		default:
			offset = 0;
			break;
		}

		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers + 1, offset,
		    sc->sc_olen - offset, &actlen)) {

			req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
			req.bRequest = UR_SET_REPORT;
			USETW2(req.wValue, UHID_OUTPUT_REPORT, sc->sc_oid);
			req.wIndex[0] = sc->sc_iface_no;
			req.wIndex[1] = 0;
			USETW(req.wLength, sc->sc_olen);

			switch (sc->sc_model) {
			case MODEL_CY7C63743:
				data[0] = actlen;
				break;
			case MODEL_CY7C64013:
				data[0] = 0;
				data[1] = actlen;
				break;
			default:
				break;
			}

			usbd_copy_in(xfer->frbuffers + 0, 0, &(req), sizeof(req));
			usbd_copy_in(xfer->frbuffers + 1, 0, data, offset);

			xfer->frlengths[0] = sizeof(req);
			xfer->frlengths[1] = sc->sc_olen;
			xfer->nframes = xfer->frlengths[1] ? 2 : 1;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error == USBD_CANCELLED) {
			return;
		}
		DPRINTF(sc, 0, "error=%s\n",
		    usbd_errstr(xfer->error));
		goto tr_transferred;
	}
}

static void
ucycom_cfg_write(struct ucycom_softc *sc, uint32_t baud, uint8_t cfg)
{
	usb_device_request_t req;
	uint16_t len;
	usbd_status_t err;

	len = sc->sc_flen;
	if (len > sizeof(sc->sc_temp_cfg)) {
		len = sizeof(sc->sc_temp_cfg);
	}
	sc->sc_cfg = cfg;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UR_SET_REPORT;
	USETW2(req.wValue, UHID_FEATURE_REPORT, sc->sc_fid);
	req.wIndex[0] = sc->sc_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, len);

	sc->sc_temp_cfg[0] = (baud & 0xff);
	sc->sc_temp_cfg[1] = (baud >> 8) & 0xff;
	sc->sc_temp_cfg[2] = (baud >> 16) & 0xff;
	sc->sc_temp_cfg[3] = (baud >> 24) & 0xff;
	sc->sc_temp_cfg[4] = cfg;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
		return;
	}
	err = usbd_do_request_flags
	    (sc->sc_udev, &Giant, &req, sc->sc_temp_cfg, 0, NULL, 1000);

	if (err) {
		DPRINTF(-1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));
	}
	return;
}

static int
ucycom_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	switch (t->c_ospeed) {
		case 600:
		case 1200:
		case 2400:
		case 4800:
		case 9600:
		case 19200:
		case 38400:
		case 57600:
#if 0
		/*
		 * Stock chips only support standard baud rates in the 600 - 57600
		 * range, but higher rates can be achieved using custom firmware.
		 */
		case 115200:
		case 153600:
		case 192000:
#endif
		break;
	default:
		return (EINVAL);
	}
	return (0);
}

static void
ucycom_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct ucycom_softc *sc = ucom->sc_parent;
	uint8_t cfg;

	DPRINTF(sc, 0, "\n");

	if (t->c_cflag & CIGNORE) {
		cfg = sc->sc_cfg;
	} else {
		cfg = 0;
		switch (t->c_cflag & CSIZE) {
		default:
		case CS8:
			++cfg;
		case CS7:
			++cfg;
		case CS6:
			++cfg;
		case CS5:
			break;
		}

		if (t->c_cflag & CSTOPB)
			cfg |= UCYCOM_CFG_STOPB;
		if (t->c_cflag & PARENB)
			cfg |= UCYCOM_CFG_PAREN;
		if (t->c_cflag & PARODD)
			cfg |= UCYCOM_CFG_PARODD;
	}

	ucycom_cfg_write(sc, t->c_ospeed, cfg);
	return;
}

static void
ucycom_intr_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ucycom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~UCYCOM_FLAG_INTR_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ucycom_intr_read_callback(struct usbd_xfer *xfer)
{
	struct ucycom_softc *sc = xfer->priv_sc;
	uint8_t buf[2];
	uint32_t offset;
	uint32_t len;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		switch (sc->sc_model) {
		case MODEL_CY7C63743:
			if (xfer->actlen < 1) {
				goto tr_setup;
			}
			usbd_copy_out(xfer->frbuffers + 0, 0, buf, 1);

			sc->sc_ist = buf[0] & ~0x07;
			len = buf[0] & 0x07;

			(xfer->actlen)--;

			offset = 1;

			break;

		case MODEL_CY7C64013:
			if (xfer->actlen < 2) {
				goto tr_setup;
			}
			usbd_copy_out(xfer->frbuffers + 0, 0, buf, 2);

			sc->sc_ist = buf[0] & ~0x07;
			len = buf[1];

			(xfer->actlen) -= 2;

			offset = 2;

			break;

		default:
			DPRINTF(-1, "unsupported model number!\n");
			goto tr_setup;
		}

		if (len > xfer->actlen) {
			len = xfer->actlen;
		}
		if (len) {
			ucom_put_data(&(sc->sc_ucom), xfer->frbuffers + 0,
			    offset, len);
		}
	case USBD_ST_SETUP:
tr_setup:
		if (sc->sc_flags & UCYCOM_FLAG_INTR_STALL) {
			usbd_transfer_start(sc->sc_xfer[2]);
		} else {
			xfer->frlengths[0] = sc->sc_ilen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_CANCELLED) {
			sc->sc_flags |= UCYCOM_FLAG_INTR_STALL;
			usbd_transfer_start(sc->sc_xfer[2]);
		}
		return;

	}
}
