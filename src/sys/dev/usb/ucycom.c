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
 *
 * $FreeBSD: src/sys/dev/usb/ucycom.c,v 1.4 2005/10/16 20:22:56 phk Exp $
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
#include <sys/taskqueue.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_hid.h>

#include "usbdevs.h"

#include <dev/usb/ucomvar.h>

__FBSDID("$FreeBSD: src/sys/dev/usb/ucycom.c,v 1.4 2005/10/16 20:22:56 phk Exp $");

#define UCYCOM_MAX_IOLEN	(256 + 2) /* bytes */

#define UCYCOM_ENDPT_MAX	4 /* units */
#define UCYCOM_IFACE_INDEX	0

#define DPRINTF(...)

struct ucycom_softc {
	struct ucom_softc	sc_ucom;

	struct usbd_device *	sc_udev;
	struct usbd_xfer *	sc_xfer[UCYCOM_ENDPT_MAX];
	device_t		sc_dev;

	u_int32_t		sc_model;
#define	MODEL_CY7C63743		0x63743
#define	MODEL_CY7C64013		0x64013
	u_int32_t		sc_baud;
	u_int32_t		sc_unit;

	u_int16_t		sc_flen; /* feature report length */
	u_int16_t		sc_ilen; /* input report length */
	u_int16_t		sc_olen; /* output report length */

	u_int8_t		sc_fid; /* feature report id */
	u_int8_t		sc_iid; /* input report id */
	u_int8_t		sc_oid; /* output report id */
	u_int8_t		sc_cfg;
#define UCYCOM_CFG_RESET	0x80
#define UCYCOM_CFG_PARODD	0x20
#define UCYCOM_CFG_PAREN	0x10
#define UCYCOM_CFG_STOPB	0x08
#define UCYCOM_CFG_DATAB	0x03
	u_int8_t		sc_ist; /* status flags from last input */
	u_int8_t		sc_flags;
#define UCYCOM_FLAG_RELOAD_CONFIG  0x01
#define UCYCOM_FLAG_INTR_STALL     0x02
	u_int8_t		sc_name[16];
	u_int8_t		sc_iface_no;
};

/* prototypes */

static device_probe_t ucycom_probe;
static device_attach_t ucycom_attach;
static device_detach_t ucycom_detach;

static int
ucycom_open(struct ucom_softc *ucom);

static void
ucycom_start_read(struct ucom_softc *ucom);

static void
ucycom_stop_read(struct ucom_softc *ucom);

static void
ucycom_start_write(struct ucom_softc *ucom);

static void
ucycom_stop_write(struct ucom_softc *ucom);

static void
ucycom_ctrl_write_callback(struct usbd_xfer *xfer);

static void
ucycom_config_callback(struct usbd_xfer *xfer);

static int
ucycom_param(struct ucom_softc *ucom, struct termios *t);

static int
ucycom_configure(struct ucycom_softc *sc, u_int32_t baud, u_int8_t cfg);

static void
ucycom_intr_read_clear_stall_callback(struct usbd_xfer *xfer);

static void
ucycom_intr_read_callback(struct usbd_xfer *xfer);

static const struct usbd_config ucycom_config[UCYCOM_ENDPT_MAX] = {

    [0] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = (sizeof(usb_device_request_t) + UCYCOM_MAX_IOLEN),
      .flags     = 0,
      .callback  = &ucycom_ctrl_write_callback,
      .timeout   = 1000, /* 1 second */
    },

    [1] = {
      .type      = UE_INTERRUPT,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .flags     = USBD_SHORT_XFER_OK,
      .bufsize   = UCYCOM_MAX_IOLEN,
      .callback  = &ucycom_intr_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = (sizeof(usb_device_request_t) + UCYCOM_MAX_IOLEN),
      .flags     = 0,
      .callback  = &ucycom_config_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &ucycom_intr_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static const struct ucom_callback ucycom_callback = {
    .ucom_param       = &ucycom_param,
    .ucom_open        = &ucycom_open,
    .ucom_start_read  = &ucycom_start_read,
    .ucom_stop_read   = &ucycom_stop_read,
    .ucom_start_write = &ucycom_start_write,
    .ucom_stop_write  = &ucycom_stop_write,
};

static device_method_t ucycom_methods[] = {
    DEVMETHOD(device_probe, ucycom_probe),
    DEVMETHOD(device_attach, ucycom_attach),
    DEVMETHOD(device_detach, ucycom_detach),
    { 0, 0 }
};

static devclass_t ucycom_devclass;

static driver_t ucycom_driver = {
    .name    = "ucycom",
    .methods = ucycom_methods,
    .size    = sizeof(struct ucycom_softc),
};

DRIVER_MODULE(ucycom, uhub, ucycom_driver, ucycom_devclass, usbd_driver_load, 0);
MODULE_VERSION(ucycom, 1);
MODULE_DEPEND(ucycom, usb, 1, 1, 1);

/*
 * Supported devices
 */

static struct ucycom_device {
	uint16_t		 vendor;
	uint16_t		 product;
	u_int32_t		 model;
} ucycom_devices[] = {
	{ USB_VENDOR_DELORME, USB_PRODUCT_DELORME_EARTHMATE, MODEL_CY7C64013 },
	{ 0, 0, 0 },
};

#define UCYCOM_DEFAULT_RATE	 4800
#define UCYCOM_DEFAULT_CFG	 0x03 /* N-8-1 */

static int
ucycom_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ucycom_device *ud;

	if (uaa->iface != NULL) {
	    return UMATCH_NONE;
	}

	for (ud = ucycom_devices; ud->model != 0; ++ud) {
	    if ((ud->vendor == uaa->vendor) && 
		(ud->product == uaa->product)) {
	        return UMATCH_VENDOR_PRODUCT;
	    }
	}

	return UMATCH_NONE;
}

static int
ucycom_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ucycom_softc *sc = device_get_softc(dev);
	struct ucycom_device *ud;
	struct usbd_interface *iface;
	void *urd_ptr = NULL;
	int32_t error;
	int32_t urd_len;

	if (sc == NULL) {
	    return ENOMEM;
	}

	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;
	sc->sc_unit = device_get_unit(dev);

	usbd_set_desc(dev, uaa->device);

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

	error = usbd_set_config_index(sc->sc_udev, 0, 1 /* verbose */);

	if (error) {
	    device_printf(dev, "failed to select "
			  "configuration: %s\n",
			  usbd_errstr(error));
	    goto detach;
	}

	/* get report descriptor */

	error = usbreq_read_report_desc(uaa->device, UCYCOM_IFACE_INDEX, 
					&urd_ptr, &urd_len, M_USBDEV);
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

	error = usbd_transfer_setup(uaa->device, UCYCOM_IFACE_INDEX,
				    sc->sc_xfer, ucycom_config, UCYCOM_ENDPT_MAX,
				    sc, &Giant);
	if (error) {
	    device_printf(dev, "allocating USB "
			  "transfers failed!\n");
	    goto detach;
	}

	error = ucom_attach(&(sc->sc_ucom), 1, sc,
			    &ucycom_callback, &Giant);

	if (error) {
	    goto detach;
	}

	if (urd_ptr) {
	    free(urd_ptr, M_USBDEV);
	}

	return 0; /* success */

 detach:
	if (urd_ptr) {
	    free(urd_ptr, M_USBDEV);
	}
	ucycom_detach(dev);
	return ENXIO;
}

static int
ucycom_detach(device_t dev)
{
	struct ucycom_softc *sc = device_get_softc(dev);

	ucom_detach(&(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UCYCOM_ENDPT_MAX);

	return 0;
}

static int
ucycom_open(struct ucom_softc *ucom)
{
	struct ucycom_softc *sc = ucom->sc_parent;

	/* set default configuration */
	ucycom_configure(sc, UCYCOM_DEFAULT_RATE, UCYCOM_DEFAULT_CFG);

	sc->sc_flags |= UCYCOM_FLAG_INTR_STALL;

	return 0;
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
	usbd_transfer_stop(sc->sc_xfer[3]);
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
	usb_device_request_t *req = xfer->buffer;
	u_int8_t offset;
	u_int32_t actlen;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
	    return;
	}
	DPRINTF(sc, 0, "error=%s\n", 
		usbd_errstr(xfer->error));
 tr_transferred:
 tr_setup:

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

	if(ucom_get_data(&(sc->sc_ucom), req->bData + offset,
			 sc->sc_olen - offset, &actlen)) {

	    req->bmRequestType = UT_WRITE_CLASS_INTERFACE;
	    req->bRequest = UR_SET_REPORT;
	    USETW2(req->wValue, UHID_OUTPUT_REPORT, sc->sc_oid);
	    USETW(req->wIndex, sc->sc_iface_no);
	    USETW(req->wLength, sc->sc_olen);

	    switch (sc->sc_model) {
	    case MODEL_CY7C63743:
	        req->bData[0] = actlen;
		break;
	    case MODEL_CY7C64013:
	        req->bData[0] = 0;
		req->bData[1] = actlen;
		break;
	    default:
	        panic("invalid model number!\n");
		break;
	    }

	    xfer->length = (sc->sc_olen + sizeof(*req));

	    usbd_start_hardware(xfer);
	}
	return;
}

static void
ucycom_config_callback(struct usbd_xfer *xfer)
{
	struct ucycom_softc *sc = xfer->priv_sc;
	usb_device_request_t *req = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
	    return;
	}
	DPRINTF(sc, 0, "error=%s\n", 
		usbd_errstr(xfer->error));
 tr_transferred:
 tr_setup:
	if (sc->sc_flags &   UCYCOM_FLAG_RELOAD_CONFIG) {
	    sc->sc_flags &= ~UCYCOM_FLAG_RELOAD_CONFIG;

	    req->bmRequestType = UT_WRITE_CLASS_INTERFACE;
	    req->bRequest = UR_SET_REPORT;
	    USETW2(req->wValue, UHID_FEATURE_REPORT, sc->sc_fid);
	    USETW(req->wIndex, sc->sc_iface_no);
	    USETW(req->wLength, sc->sc_flen);

	    req->bData[0] = (sc->sc_baud & 0xff);
	    req->bData[1] = (sc->sc_baud >> 8) & 0xff;
	    req->bData[2] = (sc->sc_baud >> 16) & 0xff;
	    req->bData[3] = (sc->sc_baud >> 24) & 0xff;
	    req->bData[4] = sc->sc_cfg;

	    xfer->length = (sc->sc_flen + sizeof(*req));

	    usbd_start_hardware(xfer);
	}
	return;
}

static int
ucycom_param(struct ucom_softc *ucom, struct termios *t)
{
	struct ucycom_softc *sc = ucom->sc_parent;
	u_int32_t baud;
	u_int8_t cfg;
	int error;

	DPRINTF(sc, 0, "\n");

	if (t->c_ispeed != t->c_ospeed) {
	    return (EINVAL);
	}

	baud = t->c_ispeed;

	if (t->c_cflag & CIGNORE) {
		cfg = sc->sc_cfg;
	} else {
		cfg = 0;
		switch (t->c_cflag & CSIZE) {
		case CS8:
			++cfg;
		case CS7:
			++cfg;
		case CS6:
			++cfg;
		case CS5:
			break;
		default:
			return (EINVAL);
		}
		if (t->c_cflag & CSTOPB)
			cfg |= UCYCOM_CFG_STOPB;
		if (t->c_cflag & PARENB)
			cfg |= UCYCOM_CFG_PAREN;
		if (t->c_cflag & PARODD)
			cfg |= UCYCOM_CFG_PARODD;
	}

	error = ucycom_configure(sc, baud, cfg);
	return error;
}

static int
ucycom_configure(struct ucycom_softc *sc, u_int32_t baud, u_int8_t cfg)
{
	switch (baud) {
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
		return EINVAL;
	}

	sc->sc_flags |= UCYCOM_FLAG_RELOAD_CONFIG;
	sc->sc_baud = baud;
	sc->sc_cfg = cfg;

	usbd_transfer_start(sc->sc_xfer[2]);

	return 0;
}

static void
ucycom_intr_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ucycom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);
	sc->sc_flags &= ~UCYCOM_FLAG_INTR_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	sc->sc_flags &= ~UCYCOM_FLAG_INTR_STALL;
	DPRINTF(sc, 0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
ucycom_intr_read_callback(struct usbd_xfer *xfer)
{
	struct ucycom_softc *sc = xfer->priv_sc;
	u_int8_t *ptr = xfer->buffer;
	u_int32_t len;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flags |= UCYCOM_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;

 tr_transferred:
	switch (sc->sc_model) {
	case MODEL_CY7C63743:
	    if (xfer->actlen < 1) {
	        goto tr_setup;
	    }
	    sc->sc_ist = ptr[0] & ~0x07;
	    len = ptr[0] & 0x07;

	    xfer->actlen --;
	    ptr ++;

	    xfer->actlen = min(xfer->actlen, len);
	    break;

	case MODEL_CY7C64013:
	    if (xfer->actlen < 2) {
	        goto tr_setup;
	    }
	    sc->sc_ist = ptr[0] & ~0x07;
	    len = ptr[1];

	    xfer->actlen -= 2;
	    ptr += 2;

	    xfer->actlen = min(xfer->actlen, len);
	    break;

	default:
	    panic("unsupported model number!");
	    break;
	}

	if (xfer->actlen) {
	    ucom_put_data(&(sc->sc_ucom), ptr, xfer->actlen);
	}

 tr_setup:
	if (sc->sc_flags & UCYCOM_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    xfer->length = sc->sc_ilen;
	    usbd_start_hardware(xfer);
	}
	return;
}
