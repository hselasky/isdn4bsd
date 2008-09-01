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
__FBSDID("$FreeBSD$");

#include <dev/usb2/include/usb2_devid.h>
#include <dev/usb2/include/usb2_standard.h>
#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_error.h>
#include <dev/usb2/include/usb2_cdc.h>

#define	USB_DEBUG_VAR usb2_debug

#include <dev/usb2/core/usb2_core.h>
#include <dev/usb2/core/usb2_debug.h>
#include <dev/usb2/core/usb2_process.h>
#include <dev/usb2/core/usb2_config_td.h>
#include <dev/usb2/core/usb2_request.h>
#include <dev/usb2/core/usb2_lookup.h>
#include <dev/usb2/core/usb2_util.h>
#include <dev/usb2/core/usb2_busdma.h>

#include <dev/usb2/serial/usb2_serial.h>

#define	UIPAQ_CONFIG_INDEX	0	/* config number 1 */
#define	UIPAQ_IFACE_INDEX	0

#define	UIPAQ_BUF_SIZE		1024
#define	UIPAQ_N_DATA_TRANSFER	4

struct uipaq_softc {
	struct usb2_com_super_softc sc_super_ucom;
	struct usb2_com_softc sc_ucom;

	struct usb2_xfer *sc_xfer_data[UIPAQ_N_DATA_TRANSFER];
	struct usb2_device *sc_udev;

	uint16_t sc_line;

	uint8_t	sc_lsr;			/* local status register */
	uint8_t	sc_msr;			/* modem status register */
	uint8_t	sc_flag;
#define	UIPAQ_FLAG_READ_STALL  0x01
#define	UIPAQ_FLAG_WRITE_STALL 0x02
#define	UIPAQ_FLAG_INTR_STALL  0x04
};

static device_probe_t uipaq_probe;
static device_attach_t uipaq_attach;
static device_detach_t uipaq_detach;

static usb2_callback_t uipaq_write_callback;
static usb2_callback_t uipaq_read_callback;
static usb2_callback_t uipaq_write_clear_stall_callback;
static usb2_callback_t uipaq_read_clear_stall_callback;

static void uipaq_start_read(struct usb2_com_softc *ucom);
static void uipaq_stop_read(struct usb2_com_softc *ucom);
static void uipaq_start_write(struct usb2_com_softc *ucom);
static void uipaq_stop_write(struct usb2_com_softc *ucom);
static void uipaq_cfg_do_request(struct uipaq_softc *sc, struct usb2_device_request *req, void *data);
static void uipaq_cfg_set_dtr(struct usb2_com_softc *ucom, uint8_t onoff);
static void uipaq_cfg_set_rts(struct usb2_com_softc *ucom, uint8_t onoff);
static void uipaq_cfg_set_break(struct usb2_com_softc *ucom, uint8_t onoff);

static const struct usb2_config uipaq_config_data[UIPAQ_N_DATA_TRANSFER] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.mh.bufsize = UIPAQ_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &uipaq_write_callback,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.bufsize = UIPAQ_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &uipaq_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(struct usb2_device_request),
		.mh.callback = &uipaq_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(struct usb2_device_request),
		.mh.callback = &uipaq_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},
};

static const struct usb2_com_callback uipaq_callback = {
	.usb2_com_cfg_set_dtr = &uipaq_cfg_set_dtr,
	.usb2_com_cfg_set_rts = &uipaq_cfg_set_rts,
	.usb2_com_cfg_set_break = &uipaq_cfg_set_break,
	.usb2_com_start_read = &uipaq_start_read,
	.usb2_com_stop_read = &uipaq_stop_read,
	.usb2_com_start_write = &uipaq_start_write,
	.usb2_com_stop_write = &uipaq_stop_write,
};

static const struct usb2_device_id uipaq_devs[] = {
	{USB_VPI(USB_VENDOR_HTC, USB_PRODUCT_HTC_SMARTPHONE, 0)},
	{USB_VPI(USB_VENDOR_HP, USB_PRODUCT_HP_2215, 0)},
	{USB_VPI(USB_VENDOR_HP, USB_PRODUCT_HP_568J, 0)},
	{USB_VPI(USB_VENDOR_HTC, USB_PRODUCT_HTC_WINMOBILE, 0)},
	{USB_VPI(USB_VENDOR_HTC, USB_PRODUCT_HTC_PPC6700MODEM, 0)},
	{USB_VPI(USB_VENDOR_COMPAQ, USB_PRODUCT_COMPAQ_IPAQPOCKETPC, 0)},
	{USB_VPI(USB_VENDOR_CASIO, USB_PRODUCT_CASIO_BE300, 0)},
	{USB_VPI(USB_VENDOR_SHARP, USB_PRODUCT_SHARP_WZERO3ES, 0)},
	{USB_VPI(USB_VENDOR_ASUS, USB_PRODUCT_ASUS_P535, 0)},
};

static device_method_t uipaq_methods[] = {
	DEVMETHOD(device_probe, uipaq_probe),
	DEVMETHOD(device_attach, uipaq_attach),
	DEVMETHOD(device_detach, uipaq_detach),
	{0, 0}
};

static devclass_t uipaq_devclass;

static driver_t uipaq_driver = {
	.name = "uipaq",
	.methods = uipaq_methods,
	.size = sizeof(struct uipaq_softc),
};

DRIVER_MODULE(uipaq, ushub, uipaq_driver, uipaq_devclass, NULL, 0);
MODULE_DEPEND(uipaq, usb2_core, 1, 1, 1);
MODULE_DEPEND(uipaq, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static int
uipaq_probe(device_t dev)
{
	struct usb2_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb2_mode != USB_MODE_HOST) {
		return (ENXIO);
	}
	if (uaa->info.bConfigIndex != UIPAQ_CONFIG_INDEX) {
		return (ENXIO);
	}
	if (uaa->info.bIfaceIndex != UIPAQ_IFACE_INDEX) {
		return (ENXIO);
	}
	return (usb2_lookup_id_by_uaa(uipaq_devs, sizeof(uipaq_devs), uaa));
}

static int
uipaq_attach(device_t dev)
{
	struct usb2_attach_arg *uaa = device_get_ivars(dev);
	struct uipaq_softc *sc = device_get_softc(dev);
	int error;
	uint8_t iface_index;

	if (sc == NULL) {
		return (ENOMEM);
	}
	sc->sc_udev = uaa->device;

	device_set_usb2_desc(dev);

	iface_index = UIPAQ_IFACE_INDEX;
	error = usb2_transfer_setup(uaa->device, &iface_index,
	    sc->sc_xfer_data, uipaq_config_data,
	    UIPAQ_N_DATA_TRANSFER, sc, &Giant);

	if (error) {
		goto detach;
	}
	/* clear stall at first run */
	sc->sc_flag |= (UIPAQ_FLAG_READ_STALL |
	    UIPAQ_FLAG_WRITE_STALL);

	error = usb2_com_attach(&sc->sc_super_ucom, &(sc->sc_ucom), 1, sc,
	    &uipaq_callback, &Giant);
	if (error) {
		goto detach;
	}
	return (0);

detach:
	uipaq_detach(dev);
	return (ENXIO);
}

int
uipaq_detach(device_t dev)
{
	struct uipaq_softc *sc = device_get_softc(dev);

	usb2_com_detach(&sc->sc_super_ucom, &(sc->sc_ucom), 1);

	usb2_transfer_unsetup(sc->sc_xfer_data, UIPAQ_N_DATA_TRANSFER);

	return (0);
}

static void
uipaq_start_read(struct usb2_com_softc *ucom)
{
	struct uipaq_softc *sc = ucom->sc_parent;

	/* start read endpoint */
	usb2_transfer_start(sc->sc_xfer_data[1]);
	return;
}

static void
uipaq_stop_read(struct usb2_com_softc *ucom)
{
	struct uipaq_softc *sc = ucom->sc_parent;

	/* stop read endpoint */
	usb2_transfer_stop(sc->sc_xfer_data[3]);
	usb2_transfer_stop(sc->sc_xfer_data[1]);
	return;
}

static void
uipaq_start_write(struct usb2_com_softc *ucom)
{
	struct uipaq_softc *sc = ucom->sc_parent;

	usb2_transfer_start(sc->sc_xfer_data[0]);
	return;
}

static void
uipaq_stop_write(struct usb2_com_softc *ucom)
{
	struct uipaq_softc *sc = ucom->sc_parent;

	usb2_transfer_stop(sc->sc_xfer_data[2]);
	usb2_transfer_stop(sc->sc_xfer_data[0]);
	return;
}

static void
uipaq_cfg_do_request(struct uipaq_softc *sc, struct usb2_device_request *req,
    void *data)
{
	uint16_t length;
	usb2_error_t err;

	if (usb2_com_cfg_is_gone(&sc->sc_ucom)) {
		goto error;
	}
	err = usb2_do_request(sc->sc_udev, &Giant, req, data);

	if (err) {

		DPRINTFN(0, "device request failed, err=%s "
		    "(ignored)\n", usb2_errstr(err));

error:
		length = UGETW(req->wLength);

		if ((req->bmRequestType & UT_READ) && length) {
			bzero(data, length);
		}
	}
	return;
}

static void
uipaq_cfg_set_dtr(struct usb2_com_softc *ucom, uint8_t onoff)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	struct usb2_device_request req;

	DPRINTF("onoff=%d\n", onoff);

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
uipaq_cfg_set_rts(struct usb2_com_softc *ucom, uint8_t onoff)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	struct usb2_device_request req;

	DPRINTF("onoff=%d\n", onoff);

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
uipaq_cfg_set_break(struct usb2_com_softc *ucom, uint8_t onoff)
{
	struct uipaq_softc *sc = ucom->sc_parent;
	struct usb2_device_request req;
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
uipaq_write_callback(struct usb2_xfer *xfer)
{
	struct uipaq_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_SETUP:
	case USB_ST_TRANSFERRED:
		if (sc->sc_flag & UIPAQ_FLAG_WRITE_STALL) {
			usb2_transfer_start(sc->sc_xfer_data[2]);
			return;
		}
		if (usb2_com_get_data(&sc->sc_ucom, xfer->frbuffers, 0,
		    UIPAQ_BUF_SIZE, &actlen)) {

			xfer->frlengths[0] = actlen;
			usb2_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USB_ERR_CANCELLED) {
			sc->sc_flag |= UIPAQ_FLAG_WRITE_STALL;
			usb2_transfer_start(sc->sc_xfer_data[2]);
		}
		return;

	}
}

static void
uipaq_write_clear_stall_callback(struct usb2_xfer *xfer)
{
	struct uipaq_softc *sc = xfer->priv_sc;
	struct usb2_xfer *xfer_other = sc->sc_xfer_data[0];

	if (usb2_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF("stall cleared\n");
		sc->sc_flag &= ~UIPAQ_FLAG_WRITE_STALL;
		usb2_transfer_start(xfer_other);
	}
	return;
}

static void
uipaq_read_callback(struct usb2_xfer *xfer)
{
	struct uipaq_softc *sc = xfer->priv_sc;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		usb2_com_put_data(&sc->sc_ucom, xfer->frbuffers, 0,
		    xfer->actlen);

	case USB_ST_SETUP:
		if (sc->sc_flag & UIPAQ_FLAG_READ_STALL) {
			usb2_transfer_start(sc->sc_xfer_data[3]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usb2_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USB_ERR_CANCELLED) {
			sc->sc_flag |= UIPAQ_FLAG_READ_STALL;
			usb2_transfer_start(sc->sc_xfer_data[3]);
		}
		return;
	}
}

static void
uipaq_read_clear_stall_callback(struct usb2_xfer *xfer)
{
	struct uipaq_softc *sc = xfer->priv_sc;
	struct usb2_xfer *xfer_other = sc->sc_xfer_data[1];

	if (usb2_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF("stall cleared\n");
		sc->sc_flag &= ~UIPAQ_FLAG_READ_STALL;
		usb2_transfer_start(xfer_other);
	}
	return;
}
