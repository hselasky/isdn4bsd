/*	$OpenBSD: uark.c,v 1.1 2006/08/14 08:30:22 jsg Exp $	*/

/*
 * Copyright (c) 2006 Jonathan Gray <jsg@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * $FreeBSD: src/sys/dev/usb/uark.c,v 1.7 2007/06/30 02:42:36 imp Exp $
 */

/*
 * NOTE: all function names beginning like "uark_cfg_" can only
 * be called from within the config thread function !
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

#define	DPRINTF(...) do { } while (0)

#define	UARK_BUF_SIZE		1024	/* bytes */

#define	UARK_N_TRANSFER	4		/* units */

#define	UARK_SET_DATA_BITS(x)	((x) - 5)

#define	UARK_PARITY_NONE	0x00
#define	UARK_PARITY_ODD		0x08
#define	UARK_PARITY_EVEN	0x18

#define	UARK_STOP_BITS_1	0x00
#define	UARK_STOP_BITS_2	0x04

#define	UARK_BAUD_REF		3000000

#define	UARK_WRITE		0x40
#define	UARK_READ		0xc0

#define	UARK_REQUEST		0xfe

#define	UARK_CONFIG_INDEX	0
#define	UARK_IFACE_INDEX	0

struct uark_softc {
	struct ucom_super_softc sc_super_ucom;
	struct ucom_softc sc_ucom;

	struct usbd_xfer *sc_xfer[UARK_N_TRANSFER];
	struct usbd_device *sc_udev;

	uint8_t	sc_flags;
#define	UARK_FLAG_BULK_READ_STALL	0x01
#define	UARK_FLAG_BULK_WRITE_STALL	0x02
	uint8_t	sc_msr;
	uint8_t	sc_lsr;
};

/* prototypes */

static device_probe_t uark_probe;
static device_attach_t uark_attach;
static device_detach_t uark_detach;

static usbd_callback_t uark_bulk_write_callback;
static usbd_callback_t uark_bulk_write_clear_stall_callback;
static usbd_callback_t uark_bulk_read_callback;
static usbd_callback_t uark_bulk_read_clear_stall_callback;

static void uark_start_read(struct ucom_softc *ucom);
static void uark_stop_read(struct ucom_softc *ucom);
static void uark_start_write(struct ucom_softc *ucom);
static void uark_stop_write(struct ucom_softc *ucom);

static int uark_pre_param(struct ucom_softc *ucom, struct termios *t);
static void uark_cfg_param(struct ucom_softc *ucom, struct termios *t);
static void uark_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr);
static void uark_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff);
static void uark_cfg_write(struct uark_softc *sc, uint16_t index, uint16_t value);

static const struct usbd_config
	uark_xfer_config[UARK_N_TRANSFER] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.mh.bufsize = UARK_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &uark_bulk_write_callback,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.bufsize = UARK_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &uark_bulk_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &uark_bulk_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &uark_bulk_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},
};

static const struct ucom_callback uark_callback = {
	.ucom_cfg_get_status = &uark_cfg_get_status,
	.ucom_cfg_set_break = &uark_cfg_set_break,
	.ucom_cfg_param = &uark_cfg_param,
	.ucom_pre_param = &uark_pre_param,
	.ucom_start_read = &uark_start_read,
	.ucom_stop_read = &uark_stop_read,
	.ucom_start_write = &uark_start_write,
	.ucom_stop_write = &uark_stop_write,
};

static device_method_t uark_methods[] = {
	/* Device methods */
	DEVMETHOD(device_probe, uark_probe),
	DEVMETHOD(device_attach, uark_attach),
	DEVMETHOD(device_detach, uark_detach),
	{0, 0}
};

static devclass_t uark_devclass;

static driver_t uark_driver = {
	.name = "uark",
	.methods = uark_methods,
	.size = sizeof(struct uark_softc),
};

DRIVER_MODULE(uark, uhub, uark_driver, uark_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uark, usb, 1, 1, 1);
MODULE_DEPEND(uark, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static const struct usb_devno uark_devs[] = {
	{USB_VENDOR_ARKMICRO, USB_PRODUCT_ARKMICRO_ARK3116},
};

#define	uark_lookup(v, p) usb_lookup(uark_devs, v, p)

static int
uark_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->iface) {
		return (UMATCH_NONE);
	}
	return (uark_lookup(uaa->vendor, uaa->product) ?
	    UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

static int
uark_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uark_softc *sc = device_get_softc(dev);
	int32_t error;
	uint8_t iface_index;

	if (sc == NULL) {
		return (ENOMEM);
	}
	usbd_set_device_desc(dev);

	sc->sc_udev = uaa->device;

	/* Move the device into the configured state */
	error = usbd_set_config_index(uaa->device, UARK_CONFIG_INDEX, 1);
	if (error) {
		device_printf(dev, "failed to set configuration, err=%s\n",
		    usbd_errstr(error));
		goto detach;
	}
	iface_index = UARK_IFACE_INDEX;
	error = usbd_transfer_setup
	    (uaa->device, &iface_index, sc->sc_xfer,
	    uark_xfer_config, UARK_N_TRANSFER, sc, &Giant);

	if (error) {
		device_printf(dev, "allocating control USB "
		    "transfers failed!\n");
		goto detach;
	}
	/* clear stall at first run */
	sc->sc_flags |= (UARK_FLAG_BULK_WRITE_STALL |
	    UARK_FLAG_BULK_READ_STALL);

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
	    &uark_callback, &Giant);
	if (error) {
		DPRINTF(sc, 0, "ucom_attach failed\n");
		goto detach;
	}
	return (0);			/* success */

detach:
	uark_detach(dev);
	return (ENXIO);			/* failure */
}

static int
uark_detach(device_t dev)
{
	struct uark_softc *sc = device_get_softc(dev);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UARK_N_TRANSFER);

	return (0);
}

static void
uark_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct uark_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
	case USBD_ST_TRANSFERRED:
		if (sc->sc_flags & UARK_FLAG_BULK_WRITE_STALL) {
			usbd_transfer_start(sc->sc_xfer[2]);
			return;
		}
		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers, 0,
		    UARK_BUF_SIZE, &actlen)) {
			xfer->frlengths[0] = actlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flags |= UARK_FLAG_BULK_WRITE_STALL;
			usbd_transfer_start(sc->sc_xfer[2]);
		}
		return;

	}
}

static void
uark_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uark_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~UARK_FLAG_BULK_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uark_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct uark_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		ucom_put_data(&(sc->sc_ucom), xfer->frbuffers, 0,
		    xfer->actlen);

	case USBD_ST_SETUP:
		if (sc->sc_flags & UARK_FLAG_BULK_READ_STALL) {
			usbd_transfer_start(sc->sc_xfer[3]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flags |= UARK_FLAG_BULK_READ_STALL;
			usbd_transfer_start(sc->sc_xfer[3]);
		}
		return;

	}
}

static void
uark_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uark_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~UARK_FLAG_BULK_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uark_start_read(struct ucom_softc *ucom)
{
	struct uark_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
uark_stop_read(struct ucom_softc *ucom)
{
	struct uark_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
uark_start_write(struct ucom_softc *ucom)
{
	struct uark_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
uark_stop_write(struct ucom_softc *ucom)
{
	struct uark_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

static int
uark_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	switch (t->c_ospeed) {
		case 300:
		case 600:
		case 1200:
		case 1800:
		case 2400:
		case 4800:
		case 9600:
		case 19200:
		case 38400:
		case 57600:
		case 115200:
		break;
	default:
		return (EINVAL);
	}
	return (0);
}

static void
uark_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uark_softc *sc = ucom->sc_parent;
	uint16_t data;

	switch (t->c_ospeed) {
	case 300:
	case 600:
	case 1200:
	case 1800:
	case 2400:
	case 4800:
	case 9600:
	case 19200:
	case 38400:
	case 57600:
	case 115200:
		data = (UARK_BAUD_REF / t->c_ospeed);
		uark_cfg_write(sc, 3, 0x83);
		uark_cfg_write(sc, 0, data & 0xFF);
		uark_cfg_write(sc, 1, data >> 8);
		uark_cfg_write(sc, 3, 0x03);
		break;
	default:
		return;
	}

	if (t->c_cflag & CSTOPB)
		data = UARK_STOP_BITS_2;
	else
		data = UARK_STOP_BITS_1;

	if (t->c_cflag & PARENB) {
		if (t->c_cflag & PARODD)
			data |= UARK_PARITY_ODD;
		else
			data |= UARK_PARITY_EVEN;
	} else
		data |= UARK_PARITY_NONE;

	switch (t->c_cflag & CSIZE) {
	case CS5:
		data |= UARK_SET_DATA_BITS(5);
		break;
	case CS6:
		data |= UARK_SET_DATA_BITS(6);
		break;
	case CS7:
		data |= UARK_SET_DATA_BITS(7);
		break;
	default:
	case CS8:
		data |= UARK_SET_DATA_BITS(8);
		break;
	}
	uark_cfg_write(sc, 3, 0x00);
	uark_cfg_write(sc, 3, data);
	return;
}

static void
uark_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr)
{
	struct uark_softc *sc = ucom->sc_parent;

	*lsr = sc->sc_lsr;
	*msr = sc->sc_msr;
	return;
}

static void
uark_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uark_softc *sc = ucom->sc_parent;

	DPRINTF(sc, 0, "onoff=%d\n", onoff);

	uark_cfg_write(sc, 4, onoff ? 0x01 : 0x00);
	return;
}

static void
uark_cfg_write(struct uark_softc *sc, uint16_t index, uint16_t value)
{
	usb_device_request_t req;
	usbd_status_t err;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
		return;
	}
	req.bmRequestType = UARK_WRITE;
	req.bRequest = UARK_REQUEST;
	USETW(req.wValue, value);
	USETW(req.wIndex, index);
	USETW(req.wLength, 0);

	err = usbd_do_request_flags
	    (sc->sc_udev, &Giant, &req, NULL, 0, NULL, 1000);

	if (err) {
		DPRINTF(sc, -1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));
	}
	return;
}
