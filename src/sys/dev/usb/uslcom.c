/*	$FreeBSD: src/sys/dev/usb/uslcom.c,v 1.2 2008/03/05 14:18:29 rink Exp $ */
/*	$OpenBSD: uslcom.c,v 1.17 2007/11/24 10:52:12 jsg Exp $	*/

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
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

#ifdef USB_DEBUG
#define	DPRINTF(n,fmt,...)						\
  do { if (uslcom_debug > (n)) {						\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uslcom_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, uslcom, CTLFLAG_RW, 0, "USB uslcom");
SYSCTL_INT(_hw_usb_uslcom, OID_AUTO, debug, CTLFLAG_RW,
    &uslcom_debug, 0, "uslcom debug level");
#else
#define	DPRINTF(...) do { } while (0)
#endif

#define	USLCOM_BUF_SIZE 1024
#define	USLCOM_N_DATA_TRANSFER 4
#define	USLCOM_CONFIG_INDEX	0
#define	USLCOM_IFACE_INDEX	0

#define	USLCOM_SET_DATA_BITS(x)	((x) << 8)

#define	USLCOM_WRITE		0x41
#define	USLCOM_READ		0xc1

#define	USLCOM_UART		0x00
#define	USLCOM_BAUD_RATE	0x01
#define	USLCOM_DATA		0x03
#define	USLCOM_BREAK		0x05
#define	USLCOM_CTRL		0x07

#define	USLCOM_UART_DISABLE	0x00
#define	USLCOM_UART_ENABLE	0x01

#define	USLCOM_CTRL_DTR_ON	0x0001
#define	USLCOM_CTRL_DTR_SET	0x0100
#define	USLCOM_CTRL_RTS_ON	0x0002
#define	USLCOM_CTRL_RTS_SET	0x0200
#define	USLCOM_CTRL_CTS		0x0010
#define	USLCOM_CTRL_DSR		0x0020
#define	USLCOM_CTRL_DCD		0x0080

#define	USLCOM_BAUD_REF		0x384000

#define	USLCOM_STOP_BITS_1	0x00
#define	USLCOM_STOP_BITS_2	0x02

#define	USLCOM_PARITY_NONE	0x00
#define	USLCOM_PARITY_ODD	0x10
#define	USLCOM_PARITY_EVEN	0x20

#define	USLCOM_BREAK_OFF	0x00
#define	USLCOM_BREAK_ON		0x01

struct uslcom_softc {
	struct ucom_super_softc sc_super_ucom;
	struct ucom_softc sc_ucom;

	struct usbd_xfer *sc_xfer_data[USLCOM_N_DATA_TRANSFER];
	struct usbd_device *sc_udev;

	uint8_t	sc_msr;
	uint8_t	sc_lsr;

	uint8_t	sc_flag;
#define	USLCOM_FLAG_READ_STALL  0x01
#define	USLCOM_FLAG_WRITE_STALL 0x02
#define	USLCOM_FLAG_INTR_STALL  0x04
};

static device_probe_t uslcom_probe;
static device_attach_t uslcom_attach;
static device_detach_t uslcom_detach;

static usbd_callback_t uslcom_write_callback;
static usbd_callback_t uslcom_read_callback;
static usbd_callback_t uslcom_write_clear_stall_callback;
static usbd_callback_t uslcom_read_clear_stall_callback;

static void uslcom_start_read(struct ucom_softc *ucom);
static void uslcom_stop_read(struct ucom_softc *ucom);
static void uslcom_start_write(struct ucom_softc *ucom);
static void uslcom_stop_write(struct ucom_softc *ucom);

static void uslcom_cfg_open(struct ucom_softc *ucom);
static void uslcom_cfg_close(struct ucom_softc *ucom);
static int uslcom_pre_param(struct ucom_softc *ucom, struct termios *t);
static void uslcom_cfg_param(struct ucom_softc *ucom, struct termios *t);
static void uslcom_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff);
static void uslcom_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff);
static void uslcom_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff);
static void uslcom_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr);
static void uslcom_cfg_do_request(struct uslcom_softc *sc, usb_device_request_t *req, void *data);

static const struct usbd_config uslcom_config_data[USLCOM_N_DATA_TRANSFER] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.mh.bufsize = USLCOM_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &uslcom_write_callback,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.bufsize = USLCOM_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &uslcom_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uslcom_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uslcom_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},
};

static const struct ucom_callback uslcom_callback = {
	.ucom_cfg_open = &uslcom_cfg_open,
	.ucom_cfg_close = &uslcom_cfg_close,
	.ucom_cfg_get_status = &uslcom_cfg_get_status,
	.ucom_cfg_set_dtr = &uslcom_cfg_set_dtr,
	.ucom_cfg_set_rts = &uslcom_cfg_set_rts,
	.ucom_cfg_set_break = &uslcom_cfg_set_break,
	.ucom_cfg_param = &uslcom_cfg_param,
	.ucom_pre_param = &uslcom_pre_param,
	.ucom_start_read = &uslcom_start_read,
	.ucom_stop_read = &uslcom_stop_read,
	.ucom_start_write = &uslcom_start_write,
	.ucom_stop_write = &uslcom_stop_write,
};

static const struct usb_devno uslcom_devs[] = {
	{USB_VENDOR_BALTECH, USB_PRODUCT_BALTECH_CARDREADER},
	{USB_VENDOR_DYNASTREAM, USB_PRODUCT_DYNASTREAM_ANTDEVBOARD},
	{USB_VENDOR_JABLOTRON, USB_PRODUCT_JABLOTRON_PC60B},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_ARGUSISP},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_CRUMB128},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_DEGREE},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_BURNSIDE},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_HELICOM},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_LIPOWSKY_HARP},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_LIPOWSKY_JTAG},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_LIPOWSKY_LIN},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_POLOLU},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_CP2102},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_CP210X_2},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_SUUNTO},
	{USB_VENDOR_SILABS, USB_PRODUCT_SILABS_TRAQMATE},
	{USB_VENDOR_SILABS2, USB_PRODUCT_SILABS2_DCU11CLONE},
	{USB_VENDOR_USI, USB_PRODUCT_USI_MC60}
};

static device_method_t uslcom_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, uslcom_probe),
	DEVMETHOD(device_attach, uslcom_attach),
	DEVMETHOD(device_detach, uslcom_detach),
	{0, 0}
};

static devclass_t uslcom_devclass;

static driver_t uslcom_driver = {
	.name = "uslcom",
	.methods = uslcom_methods,
	.size = sizeof(struct uslcom_softc),
};

DRIVER_MODULE(uslcom, uhub, uslcom_driver, uslcom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uslcom, usb, 1, 1, 1);
MODULE_DEPEND(uslcom, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);
MODULE_VERSION(uslcom, 1);

static int
uslcom_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (UMATCH_NONE);

	if (uaa->iface != NULL)
		return (UMATCH_NONE);

	return (usb_lookup(uslcom_devs, uaa->vendor, uaa->product) != NULL) ?
	    UMATCH_VENDOR_PRODUCT : UMATCH_NONE;
}

static int
uslcom_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uslcom_softc *sc = device_get_softc(dev);
	uint8_t ifaces[1];
	int error;

	if (sc == NULL) {
		return (ENOMEM);
	}
	usbd_set_device_desc(dev);

	sc->sc_udev = uaa->device;

	if (usbd_set_config_index(uaa->device, USLCOM_CONFIG_INDEX, 1) != 0) {
		device_printf(dev, "Could not set configuration 0.\n");
		return (ENXIO);
	}
	ifaces[0] = USLCOM_IFACE_INDEX;

	error = usbd_transfer_setup(uaa->device,
	    ifaces, sc->sc_xfer_data,
	    uslcom_config_data, USLCOM_N_DATA_TRANSFER,
	    sc, &Giant);
	if (error) {
		goto detach;
	}
	/* clear stall at first run */
	sc->sc_flag |= (USLCOM_FLAG_READ_STALL |
	    USLCOM_FLAG_WRITE_STALL);

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
	    &uslcom_callback, &Giant);
	if (error) {
		goto detach;
	}
	return (0);

detach:
	uslcom_detach(dev);
	return (ENXIO);
}

static int
uslcom_detach(device_t dev)
{
	struct uslcom_softc *sc = device_get_softc(dev);

	DPRINTF(0, "sc=%p\n", sc);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer_data, USLCOM_N_DATA_TRANSFER);

	return (0);
}

static void
uslcom_start_read(struct ucom_softc *ucom)
{
	struct uslcom_softc *sc = ucom->sc_parent;

	/* start read endpoint */
	usbd_transfer_start(sc->sc_xfer_data[1]);
	return;
}

static void
uslcom_stop_read(struct ucom_softc *ucom)
{
	struct uslcom_softc *sc = ucom->sc_parent;

	/* stop read endpoint */
	usbd_transfer_stop(sc->sc_xfer_data[3]);
	usbd_transfer_stop(sc->sc_xfer_data[1]);
	return;
}

static void
uslcom_start_write(struct ucom_softc *ucom)
{
	struct uslcom_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer_data[0]);
	return;
}

static void
uslcom_stop_write(struct ucom_softc *ucom)
{
	struct uslcom_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer_data[2]);
	usbd_transfer_stop(sc->sc_xfer_data[0]);
	return;
}

static void
uslcom_cfg_open(struct ucom_softc *ucom)
{
	struct uslcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;

	req.bmRequestType = USLCOM_WRITE;
	req.bRequest = USLCOM_UART;
	USETW(req.wValue, USLCOM_UART_ENABLE);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	uslcom_cfg_do_request(sc, &req, NULL);
	return;
}

void
uslcom_cfg_close(struct ucom_softc *ucom)
{
	struct uslcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;

	req.bmRequestType = USLCOM_WRITE;
	req.bRequest = USLCOM_UART;
	USETW(req.wValue, USLCOM_UART_DISABLE);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	uslcom_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uslcom_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uslcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;
	uint16_t ctl;

	DPRINTF(0, "onoff=%d\n", onoff);

	if (onoff)
		ctl = USLCOM_CTRL_DTR_ON;
	else
		ctl = 0;

	ctl |= USLCOM_CTRL_DTR_SET;

	req.bmRequestType = USLCOM_WRITE;
	req.bRequest = USLCOM_CTRL;
	USETW(req.wValue, ctl);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	uslcom_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uslcom_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uslcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;
	uint16_t ctl;

	DPRINTF(0, "onoff=%d\n", onoff);

	if (onoff)
		ctl = USLCOM_CTRL_RTS_ON;
	else
		ctl = 0;

	ctl |= USLCOM_CTRL_RTS_SET;

	req.bmRequestType = USLCOM_WRITE;
	req.bRequest = USLCOM_CTRL;
	USETW(req.wValue, ctl);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	uslcom_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uslcom_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uslcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;
	uint16_t brk;

	DPRINTF(0, "onoff=%d\n", onoff);

	if (onoff)
		brk = USLCOM_BREAK_ON;
	else
		brk = USLCOM_BREAK_OFF;

	req.bmRequestType = USLCOM_WRITE;
	req.bRequest = USLCOM_BREAK;
	USETW(req.wValue, brk);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	uslcom_cfg_do_request(sc, &req, NULL);
	return;
}

static int
uslcom_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	if ((t->c_ospeed < 1) || (t->c_ospeed > 921600))
		return (EINVAL);
	else
		return (0);
}

static void
uslcom_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uslcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;
	uint16_t data;

	DPRINTF(0, "sc=%p\n", sc);

	data = USLCOM_BAUD_REF / t->c_ospeed;

	req.bmRequestType = USLCOM_WRITE;
	req.bRequest = USLCOM_BAUD_RATE;
	USETW(req.wValue, data);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	uslcom_cfg_do_request(sc, &req, NULL);

	if (t->c_cflag & CSTOPB)
		data = USLCOM_STOP_BITS_2;
	else
		data = USLCOM_STOP_BITS_1;
	if (t->c_cflag & PARENB) {
		if (t->c_cflag & PARODD)
			data |= USLCOM_PARITY_ODD;
		else
			data |= USLCOM_PARITY_EVEN;
	} else
		data |= USLCOM_PARITY_NONE;

	switch (t->c_cflag & CSIZE) {
	case CS5:
		data |= USLCOM_SET_DATA_BITS(5);
		break;
	case CS6:
		data |= USLCOM_SET_DATA_BITS(6);
		break;
	case CS7:
		data |= USLCOM_SET_DATA_BITS(7);
		break;
	case CS8:
		data |= USLCOM_SET_DATA_BITS(8);
		break;
	}

	req.bmRequestType = USLCOM_WRITE;
	req.bRequest = USLCOM_DATA;
	USETW(req.wValue, data);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	uslcom_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uslcom_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr)
{
	struct uslcom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "\n");

	*lsr = sc->sc_lsr;
	*msr = sc->sc_msr;
	return;
}

static void
uslcom_cfg_do_request(struct uslcom_softc *sc, usb_device_request_t *req,
    void *data)
{
	uint16_t length;
	usbd_status_t err;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
		goto error;
	}
	err = usbd_do_request_flags(sc->sc_udev, &Giant, req,
	    data, 0, NULL, 1000);

	if (err) {

		DPRINTF(-1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));

error:
		length = UGETW(req->wLength);

		if ((req->bmRequestType & UT_READ) && length) {
			bzero(data, length);
		}
	}
	return;
}

static void
uslcom_write_callback(struct usbd_xfer *xfer)
{
	struct uslcom_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
	case USBD_ST_TRANSFERRED:
		if (sc->sc_flag & USLCOM_FLAG_WRITE_STALL) {
			usbd_transfer_start(sc->sc_xfer_data[2]);
			return;
		}
		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers, 0,
		    USLCOM_BUF_SIZE, &actlen)) {

			xfer->frlengths[0] = actlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= USLCOM_FLAG_WRITE_STALL;
			usbd_transfer_start(sc->sc_xfer_data[2]);
		}
		return;

	}
}

static void
uslcom_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uslcom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer_data[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flag &= ~USLCOM_FLAG_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uslcom_read_callback(struct usbd_xfer *xfer)
{
	struct uslcom_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		DPRINTF(0, "actlen=%d\n", xfer->actlen);

		ucom_put_data(&(sc->sc_ucom), xfer->frbuffers, 0,
		    xfer->actlen);

	case USBD_ST_SETUP:
		if (sc->sc_flag & USLCOM_FLAG_READ_STALL) {
			usbd_transfer_start(sc->sc_xfer_data[3]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= USLCOM_FLAG_READ_STALL;
			usbd_transfer_start(sc->sc_xfer_data[3]);
		}
		return;

	}
}

static void
uslcom_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uslcom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer_data[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flag &= ~USLCOM_FLAG_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}
