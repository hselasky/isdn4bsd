/*	$NetBSD: uipaq.c,v 1.4 2006/11/16 01:33:27 christos Exp $	*/
/*	$OpenBSD: uipaq.c,v 1.1 2005/06/17 23:50:33 deraadt Exp $	*/

/*
 * Copyright (c) 2000-2005 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * iPAQ driver
 * 
 * 19 July 2003:	Incorporated changes suggested by Sam Lawrance from
 * 			the uppc module
 *
 *
 * Contact isis@cs.umd.edu if you have any questions/comments about this driver
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uipaq.c,v 1.7 2007/06/21 14:42:33 imp Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_cdc.h>

#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

#define UIPAQ_CONFIG_NO		1
#define UIPAQ_IFACE_INDEX	0

#define UIPAQ_BUF_SIZE		1024
#define UIPAQ_N_DATA_TRANSFER	4

#define	DPRINTF(...) do { } while (0)

struct uipaq_softc {
	struct ucom_super_softc	sc_super_ucom;
	struct ucom_softc	sc_ucom;

	struct usbd_xfer	*sc_xfer_data[UIPAQ_N_DATA_TRANSFER];
	struct usbd_device	*sc_udev;

	uint16_t	sc_line;

	uint8_t		sc_lsr;	/* local status register */
	uint8_t		sc_msr;	/* modem status register */
	uint8_t		sc_flag;
#define UIPAQ_FLAG_READ_STALL  0x01
#define UIPAQ_FLAG_WRITE_STALL 0x02
#define UIPAQ_FLAG_INTR_STALL  0x04
};

static device_probe_t uipaq_probe;
static device_attach_t uipaq_attach;
static device_detach_t uipaq_detach;

static usbd_callback_t uipaq_write_callback;
static usbd_callback_t uipaq_read_callback;
static usbd_callback_t uipaq_write_clear_stall_callback;
static usbd_callback_t uipaq_read_clear_stall_callback;

static void	uipaq_start_read(struct ucom_softc *ucom);
static void	uipaq_stop_read(struct ucom_softc *ucom);
static void	uipaq_start_write(struct ucom_softc *ucom);
static void	uipaq_stop_write(struct ucom_softc *ucom);
static void	uipaq_cfg_do_request(struct uipaq_softc *sc, usb_device_request_t *req, void *data);
static void	uipaq_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff);
static void	uipaq_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff);
static void	uipaq_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff);

static const struct usbd_config uipaq_config_data[UIPAQ_N_DATA_TRANSFER] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_OUT,
      .bufsize   = UIPAQ_BUF_SIZE,
      .flags     = (USBD_PIPE_BOF),
      .callback  = &uipaq_write_callback,
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_IN,
      .bufsize   = UIPAQ_BUF_SIZE,
      .flags     = (USBD_PIPE_BOF|USBD_SHORT_XFER_OK),
      .callback  = &uipaq_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uipaq_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
      .interval  = 50, /* 50ms */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uipaq_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
      .interval  = 50, /* 50ms */
    },
};

static const struct ucom_callback uipaq_callback = {
  .ucom_cfg_set_dtr     = &uipaq_cfg_set_dtr,
  .ucom_cfg_set_rts     = &uipaq_cfg_set_rts,
  .ucom_cfg_set_break   = &uipaq_cfg_set_break,
  .ucom_start_read      = &uipaq_start_read,
  .ucom_stop_read       = &uipaq_stop_read,
  .ucom_start_write     = &uipaq_start_write,
  .ucom_stop_write      = &uipaq_stop_write,
};

static const struct usb_devno uipaq_devs[] = {
	{ USB_VENDOR_HP, USB_PRODUCT_HP_2215 },
	{ USB_VENDOR_HP, USB_PRODUCT_HP_568J },
	{ USB_VENDOR_COMPAQ, USB_PRODUCT_COMPAQ_IPAQPOCKETPC },
	{ USB_VENDOR_CASIO, USB_PRODUCT_CASIO_BE300 },
	{ USB_VENDOR_SHARP, USB_PRODUCT_SHARP_WZERO3ES },
};

#define uipaq_lookup(v, p) ((const void *)usb_lookup(uipaq_devs, v, p))

static device_method_t uipaq_methods[] = {
    DEVMETHOD(device_probe, uipaq_probe),
    DEVMETHOD(device_attach, uipaq_attach),
    DEVMETHOD(device_detach, uipaq_detach),
    { 0, 0 }
};

static devclass_t uipaq_devclass;

static driver_t uipaq_driver = {
    .name    = "uipaq",
    .methods = uipaq_methods,
    .size    = sizeof(struct uipaq_softc),
};

DRIVER_MODULE(uipaq, uhub, uipaq_driver, uipaq_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uipaq, usb, 1, 1, 1);
MODULE_DEPEND(uipaq, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static int
uipaq_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	const struct usb_devno *up;

	if (uaa->iface == NULL) {
	    up = uipaq_lookup(uaa->vendor, uaa->product);
	    if (up) {
	        return UMATCH_VENDOR_PRODUCT;
	    }
	}
	return UMATCH_NONE;
}

static int
uipaq_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uipaq_softc *sc = device_get_softc(dev);
	int error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	sc->sc_udev = uaa->device;

	usbd_set_desc(dev, uaa->device);

	error = usbd_set_config_no(uaa->device, UIPAQ_CONFIG_NO, 1);

	if (error) {
		device_printf(dev, "setting config "
			      "number failed!\n");
		goto detach;
	}

	error = usbd_transfer_setup(uaa->device, UIPAQ_IFACE_INDEX,
				    sc->sc_xfer_data, uipaq_config_data, 
				    UIPAQ_N_DATA_TRANSFER,
				    sc, &Giant);
	if (error) {
	    goto detach;
	}

	/* clear stall at first run */
	sc->sc_flag |= (UIPAQ_FLAG_READ_STALL|
			UIPAQ_FLAG_WRITE_STALL);

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
			    &uipaq_callback, &Giant);
	if (error) {
	    goto detach;
	}

	return 0;

 detach:
	uipaq_detach(dev);
	return ENXIO;
}

int
uipaq_detach(device_t dev)
{
	struct uipaq_softc *sc = device_get_softc(dev);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer_data, UIPAQ_N_DATA_TRANSFER);

	return 0;
}

static void
uipaq_start_read(struct ucom_softc *ucom)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	/* start read endpoint */
	usbd_transfer_start(sc->sc_xfer_data[1]);
	return;
}

static void
uipaq_stop_read(struct ucom_softc *ucom)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	/* stop read endpoint */
	usbd_transfer_stop(sc->sc_xfer_data[3]);
	usbd_transfer_stop(sc->sc_xfer_data[1]);
	return;
}

static void
uipaq_start_write(struct ucom_softc *ucom)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer_data[0]);
	return;
}

static void
uipaq_stop_write(struct ucom_softc *ucom)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer_data[2]);
	usbd_transfer_stop(sc->sc_xfer_data[0]);
	return;
}

static void
uipaq_cfg_do_request(struct uipaq_softc *sc, usb_device_request_t *req, 
		      void *data)
{
	uint16_t length;
	usbd_status err;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
	    goto error;
	}

	err = usbd_do_request_flags_mtx(sc->sc_udev, &Giant, req, 
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
uipaq_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	usb_device_request_t req;

	DPRINTF(0, "onoff=%d\n", onoff);

	if (onoff)
	  sc->sc_line |= UCDC_LINE_DTR;
	else
	  sc->sc_line &= ~UCDC_LINE_DTR;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SET_CONTROL_LINE_STATE;
	USETW(req.wValue, sc->sc_line);
	req.wIndex[0] = UIPAQ_IFACE_INDEX;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	uipaq_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uipaq_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	usb_device_request_t req;

	DPRINTF(0, "onoff=%d\n", onoff);

	if (onoff)
	  sc->sc_line |= UCDC_LINE_RTS;
	else
	  sc->sc_line &= ~UCDC_LINE_RTS;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SET_CONTROL_LINE_STATE;
	USETW(req.wValue, sc->sc_line);
	req.wIndex[0] = UIPAQ_IFACE_INDEX;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	uipaq_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uipaq_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	usb_device_request_t req;
	uint16_t temp;

	temp = onoff ? UCDC_BREAK_ON : UCDC_BREAK_OFF;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SEND_BREAK;
	USETW(req.wValue, temp);
	req.wIndex[0] = UIPAQ_IFACE_INDEX;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	uipaq_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uipaq_write_callback(struct usbd_xfer *xfer)
{
	struct uipaq_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	USBD_CHECK_STATUS(xfer);

tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UIPAQ_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer_data[2]);
	}
	return;

tr_setup:
tr_transferred:
	if (sc->sc_flag & UIPAQ_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer_data[2]);
	    return;
	}

	if(ucom_get_data(&(sc->sc_ucom), xfer->buffer,
			 UIPAQ_BUF_SIZE, &actlen)) {

	    xfer->length = actlen;

	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uipaq_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uipaq_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer_data[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
	    DPRINTF(0, "stall cleared\n");
	    sc->sc_flag &= ~UIPAQ_FLAG_WRITE_STALL;
	    usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uipaq_read_callback(struct usbd_xfer *xfer)
{
	struct uipaq_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UIPAQ_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer_data[3]);
	}
	return;

 tr_transferred:
	ucom_put_data(&(sc->sc_ucom), xfer->buffer, xfer->actlen);

 tr_setup:
	if (sc->sc_flag & UIPAQ_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer_data[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uipaq_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uipaq_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer_data[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
	    DPRINTF(0, "stall cleared\n");
	    sc->sc_flag &= ~UIPAQ_FLAG_READ_STALL;
	    usbd_transfer_start(xfer_other);
	}
	return;
}
