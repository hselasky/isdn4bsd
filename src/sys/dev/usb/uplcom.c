/*	$NetBSD: uplcom.c,v 1.21 2001/11/13 06:24:56 lukem Exp $	*/

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uplcom.c,v 1.50 2007/06/20 05:10:53 imp Exp $");

/*-
 * Copyright (c) 2001-2003, 2005 Shunsuke Akiyama <akiyama@jp.FreeBSD.org>.
 * All rights reserved.
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
 */

/*-
 * Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Ichiro FUKUHARA (ichiro@ichiro.org).
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
 * This driver supports several USB-to-RS232 serial adapters driven by
 * Prolific PL-2303, PL-2303X and probably PL-2303HX USB-to-RS232
 * bridge chip.  The adapters are sold under many different brand
 * names.
 *
 * Datasheets are available at Prolific www site at
 * http://www.prolific.com.tw.  The datasheets don't contain full
 * programming information for the chip.
 *
 * PL-2303HX is probably programmed the same as PL-2303X.
 *
 * There are several differences between PL-2303 and PL-2303(H)X.
 * PL-2303(H)X can do higher bitrate in bulk mode, has _probably_
 * different command for controlling CRTSCTS and needs special
 * sequence of commands for initialization which aren't also
 * documented in the datasheet.
 */

#include "opt_uplcom.h"			/* XXX remove this */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>
#include <dev/usb/usb_cdc.h>

#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

#ifdef USB_DEBUG
#define	DPRINTF(n,fmt,...)						\
  do { if (uplcom_debug > (n)) {					\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uplcom_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, uplcom, CTLFLAG_RW, 0, "USB uplcom");
SYSCTL_INT(_hw_usb_uplcom, OID_AUTO, debug, CTLFLAG_RW,
    &uplcom_debug, 0, "uplcom debug level");
#else
#define	DPRINTF(...)
#endif

#define	UPLCOM_MODVER			1	/* module version */

#define	UPLCOM_CONFIG_INDEX		0
#define	UPLCOM_IFACE_INDEX		0
#define	UPLCOM_SECOND_IFACE_INDEX	1

#ifndef UPLCOM_INTR_INTERVAL
#define	UPLCOM_INTR_INTERVAL		0	/* default */
#endif

#define	UPLCOM_BULK_BUF_SIZE 1024	/* bytes */
#define	UPLCOM_N_TRANSFER 6

#define	UPLCOM_SET_REQUEST		0x01
#define	UPLCOM_SET_CRTSCTS		0x41
#define	UPLCOM_SET_CRTSCTS_PL2303X	0x61
#define	RSAQ_STATUS_CTS			0x80
#define	RSAQ_STATUS_DSR			0x02
#define	RSAQ_STATUS_DCD			0x01

#define	TYPE_PL2303			0
#define	TYPE_PL2303X			1

struct uplcom_softc {
	struct ucom_super_softc sc_super_ucom;
	struct ucom_softc sc_ucom;

	struct usbd_xfer *sc_xfer[UPLCOM_N_TRANSFER];
	struct usbd_device *sc_udev;

	uint16_t sc_line;

	uint8_t	sc_flag;
#define	UPLCOM_FLAG_INTR_STALL  0x01
#define	UPLCOM_FLAG_READ_STALL  0x02
#define	UPLCOM_FLAG_WRITE_STALL 0x04

	uint8_t	sc_lsr;			/* local status register */
	uint8_t	sc_msr;			/* uplcom status register */
	uint8_t	sc_chiptype;		/* type of chip */
	uint8_t	sc_ctrl_iface_no;
	uint8_t	sc_data_iface_no;
	uint8_t	sc_iface_index[2];
};

/* prototypes */

static const struct uplcom_product *uplcom_find_up(struct usb_attach_arg *uaa);

static usbd_status_t uplcom_reset(struct uplcom_softc *sc, struct usbd_device *udev);
static int uplcom_pl2303x_init(struct usbd_device *udev);
static void uplcom_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff);
static void uplcom_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff);
static void uplcom_cfg_set_break(struct ucom_softc *sc, uint8_t onoff);
static int uplcom_pre_param(struct ucom_softc *ucom, struct termios *t);
static void uplcom_cfg_param(struct ucom_softc *ucom, struct termios *t);
static void uplcom_start_read(struct ucom_softc *ucom);
static void uplcom_stop_read(struct ucom_softc *ucom);
static void uplcom_start_write(struct ucom_softc *ucom);
static void uplcom_stop_write(struct ucom_softc *ucom);
static void uplcom_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr);
static int uplcom_ioctl(struct ucom_softc *ucom, uint32_t cmd, caddr_t data, int flag, struct thread *td);
static void uplcom_cfg_do_request(struct uplcom_softc *sc, usb_device_request_t *req, void *data);

static device_probe_t uplcom_probe;
static device_attach_t uplcom_attach;
static device_detach_t uplcom_detach;

static usbd_callback_t uplcom_intr_callback;
static usbd_callback_t uplcom_intr_clear_stall_callback;
static usbd_callback_t uplcom_write_callback;
static usbd_callback_t uplcom_write_clear_stall_callback;
static usbd_callback_t uplcom_read_callback;
static usbd_callback_t uplcom_read_clear_stall_callback;

static const struct usbd_config uplcom_config_data[UPLCOM_N_TRANSFER] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.mh.bufsize = UPLCOM_BULK_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &uplcom_write_callback,
		.if_index = 0,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.bufsize = UPLCOM_BULK_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &uplcom_read_callback,
		.if_index = 0,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uplcom_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
		.if_index = 0,
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uplcom_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
		.if_index = 0,
	},

	[4] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.bufsize = 0,	/* use wMaxPacketSize */
		.mh.callback = &uplcom_intr_callback,
		.if_index = 1,
	},

	[5] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uplcom_intr_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
		.if_index = 1,
	},
};

struct ucom_callback uplcom_callback = {
	.ucom_cfg_get_status = &uplcom_cfg_get_status,
	.ucom_cfg_set_dtr = &uplcom_cfg_set_dtr,
	.ucom_cfg_set_rts = &uplcom_cfg_set_rts,
	.ucom_cfg_set_break = &uplcom_cfg_set_break,
	.ucom_cfg_param = &uplcom_cfg_param,
	.ucom_pre_param = &uplcom_pre_param,
	.ucom_ioctl = &uplcom_ioctl,
	.ucom_start_read = &uplcom_start_read,
	.ucom_stop_read = &uplcom_stop_read,
	.ucom_start_write = &uplcom_start_write,
	.ucom_stop_write = &uplcom_stop_write,
};

struct uplcom_product {
	uint16_t vendor;
	uint16_t product;
	uint16_t release;		/* the highest release value accepted,
					 * 0xFFFF means any value, first match
					 * wins */
	uint8_t	chiptype;
};

static const struct uplcom_product uplcom_products[] = {
	/* I/O DATA USB-RSAQ */
	{USB_VENDOR_IODATA, USB_PRODUCT_IODATA_USBRSAQ, 0xFFFF, TYPE_PL2303},
	/* I/O DATA USB-RSAQ2 */
	{USB_VENDOR_PROLIFIC, USB_PRODUCT_PROLIFIC_RSAQ2, 0xFFFF, TYPE_PL2303},
	/* I/O DATA USB-RSAQ3 */
	{USB_VENDOR_PROLIFIC, USB_PRODUCT_PROLIFIC_RSAQ3, 0xFFFF, TYPE_PL2303X},
	/* PLANEX USB-RS232 URS-03 */
	{USB_VENDOR_ATEN, USB_PRODUCT_ATEN_UC232A, 0xFFFF, TYPE_PL2303},
	/* ST Lab USB-SERIAL-4 */
	{USB_VENDOR_PROLIFIC, USB_PRODUCT_PROLIFIC_PL2303, 0x0300, TYPE_PL2303X},
	/* IOGEAR/ATEN UC-232A (also ST Lab USB-SERIAL-1) */
	{USB_VENDOR_PROLIFIC, USB_PRODUCT_PROLIFIC_PL2303, 0xFFFF, TYPE_PL2303},
	/* TDK USB-PHS Adapter UHA6400 */
	{USB_VENDOR_TDK, USB_PRODUCT_TDK_UHA6400, 0xFFFF, TYPE_PL2303},
	/* RATOC REX-USB60 */
	{USB_VENDOR_RATOC, USB_PRODUCT_RATOC_REXUSB60, 0xFFFF, TYPE_PL2303},
	/* ELECOM UC-SGT */
	{USB_VENDOR_ELECOM, USB_PRODUCT_ELECOM_UCSGT, 0xFFFF, TYPE_PL2303},
	{USB_VENDOR_ELECOM, USB_PRODUCT_ELECOM_UCSGT0, 0xFFFF, TYPE_PL2303},
	/* Sagem USB-Serial Controller */
	{USB_VENDOR_SAGEM, USB_PRODUCT_SAGEM_USBSERIAL, 0xFFFF, TYPE_PL2303X},
	/* Sony Ericsson USB Cable */
	{USB_VENDOR_SONYERICSSON, USB_PRODUCT_SONYERICSSON_DCU10, 0xFFFF, TYPE_PL2303},
	/* SOURCENEXT KeikaiDenwa 8 */
	{USB_VENDOR_SOURCENEXT, USB_PRODUCT_SOURCENEXT_KEIKAI8, 0xFFFF, TYPE_PL2303},
	/* SOURCENEXT KeikaiDenwa 8 with charger */
	{USB_VENDOR_SOURCENEXT, USB_PRODUCT_SOURCENEXT_KEIKAI8_CHG,
	0xFFFF, TYPE_PL2303},
	/* HAL Corporation Crossam2+USB */
	{USB_VENDOR_HAL, USB_PRODUCT_HAL_IMR001, 0xFFFF, TYPE_PL2303},
	/* Sitecom USB to Serial */
	{USB_VENDOR_SITECOM, USB_PRODUCT_SITECOM_SERIAL, 0xFFFF, TYPE_PL2303},
	/* Tripp-Lite U209-000-R */
	{USB_VENDOR_TRIPPLITE, USB_PRODUCT_TRIPPLITE_U209, 0xFFFF, TYPE_PL2303X},
	{USB_VENDOR_RADIOSHACK, USB_PRODUCT_RADIOSHACK_USBCABLE, 0xFFFF, TYPE_PL2303},
	/* Prolific Pharos */
	{USB_VENDOR_PROLIFIC, USB_PRODUCT_PROLIFIC_PHAROS, 0xFFFF, TYPE_PL2303},
	/* Willcom W-SIM */
	{USB_VENDOR_PROLIFIC2, USB_PRODUCT_PROLIFIC2_WSIM, 0xFFFF, TYPE_PL2303X},
	{0, 0}
};

static device_method_t uplcom_methods[] = {
	DEVMETHOD(device_probe, uplcom_probe),
	DEVMETHOD(device_attach, uplcom_attach),
	DEVMETHOD(device_detach, uplcom_detach),
	{0, 0}
};

static devclass_t uplcom_devclass;

static driver_t uplcom_driver = {
	.name = "uplcom",
	.methods = uplcom_methods,
	.size = sizeof(struct uplcom_softc),
};

DRIVER_MODULE(uplcom, uhub, uplcom_driver, uplcom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uplcom, usb, 1, 1, 1);
MODULE_DEPEND(uplcom, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);
MODULE_VERSION(uplcom, UPLCOM_MODVER);

static const struct uplcom_product *
uplcom_find_up(struct usb_attach_arg *uaa)
{
	const struct uplcom_product *up = uplcom_products;

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (NULL);
	}
	if (uaa->iface == NULL) {
		while (up->vendor) {
			if ((up->vendor == uaa->vendor) &&
			    (up->product == uaa->product) &&
			    ((up->release <= uaa->release) ||
			    (up->release == 0xFFFF))) {
				return (up);
			}
			up++;
		}
	}
	return (NULL);
}

static int
uplcom_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	DPRINTF(10, "\n");

	return (uplcom_find_up(uaa) ? UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

static int
uplcom_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	const struct uplcom_product *up = uplcom_find_up(uaa);
	struct uplcom_softc *sc = device_get_softc(dev);
	struct usbd_interface *iface;
	usb_interface_descriptor_t *id;
	int error;

	DPRINTF(10, "\n");

	if (sc == NULL) {
		return (ENOMEM);
	}
	usbd_set_device_desc(dev);

	DPRINTF(0, "sc = %p\n", sc);

	sc->sc_chiptype = up->chiptype;
	sc->sc_udev = uaa->device;

	DPRINTF(0, "chiptype: %s\n",
	    (sc->sc_chiptype == TYPE_PL2303X) ?
	    "2303X" : "2303");

	/* configure the chip */

	error = usbd_set_config_index(uaa->device, UPLCOM_CONFIG_INDEX, 1);

	if (error) {
		device_printf(dev, "failed to set configuration, "
		    "error=%s\n", usbd_errstr(error));
		goto detach;
	}
	/*
	 * USB-RSAQ1 has two interface
	 *
	 *  USB-RSAQ1       | USB-RSAQ2
	 * -----------------+-----------------
	 * Interface 0      |Interface 0
	 *  Interrupt(0x81) | Interrupt(0x81)
	 * -----------------+ BulkIN(0x02)
	 * Interface 1	    | BulkOUT(0x83)
	 *   BulkIN(0x02)   |
	 *   BulkOUT(0x83)  |
	 */
	iface = usbd_get_iface(uaa->device, UPLCOM_IFACE_INDEX);

	if (iface == NULL) {
		DPRINTF(0, "no interface (1)!\n");
		goto detach;
	}
	id = usbd_get_interface_descriptor(iface);

	if (id == NULL) {
		DPRINTF(0, "no interface descriptor (1)!\n");
		goto detach;
	}
	sc->sc_ctrl_iface_no = id->bInterfaceNumber;
	sc->sc_iface_index[1] = UPLCOM_IFACE_INDEX;

	iface = usbd_get_iface(uaa->device, UPLCOM_SECOND_IFACE_INDEX);

	if (iface) {
		id = usbd_get_interface_descriptor(iface);

		if (id == NULL) {
			device_printf(dev, "no interface descriptor (2)!\n");
			goto detach;
		}
		sc->sc_data_iface_no = id->bInterfaceNumber;
		sc->sc_iface_index[0] = UPLCOM_SECOND_IFACE_INDEX;
	} else {
		sc->sc_data_iface_no = sc->sc_ctrl_iface_no;
		sc->sc_iface_index[0] = UPLCOM_IFACE_INDEX;
	}

	error = usbd_transfer_setup(uaa->device,
	    sc->sc_iface_index, sc->sc_xfer, uplcom_config_data,
	    UPLCOM_N_TRANSFER, sc, &Giant);

	if (error) {
		DPRINTF(0, "one or more missing USB endpoints, "
		    "error=%s\n", usbd_errstr(error));
		goto detach;
	}
	error = uplcom_reset(sc, uaa->device);
	if (error) {
		device_printf(dev, "reset failed, error=%s\n",
		    usbd_errstr(error));
		goto detach;
	}
	/* clear stall at first run */
	sc->sc_flag |= (UPLCOM_FLAG_READ_STALL |
	    UPLCOM_FLAG_WRITE_STALL);

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
	    &uplcom_callback, &Giant);
	if (error) {
		goto detach;
	}
	/*
	 * do the initialization during attach so that the system does not
	 * sleep during open:
	 */
	if (sc->sc_chiptype == TYPE_PL2303X) {
		if (uplcom_pl2303x_init(uaa->device)) {
			device_printf(dev, "init failed!\n");
			goto detach;
		}
	}
	return (0);

detach:
	uplcom_detach(dev);
	return (ENXIO);
}

static int
uplcom_detach(device_t dev)
{
	struct uplcom_softc *sc = device_get_softc(dev);

	DPRINTF(0, "sc=%p\n", sc);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UPLCOM_N_TRANSFER);

	return (0);
}

static usbd_status_t
uplcom_reset(struct uplcom_softc *sc, struct usbd_device *udev)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = UPLCOM_SET_REQUEST;
	USETW(req.wValue, 0);
	req.wIndex[0] = sc->sc_data_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	return (usbd_do_request(udev, &Giant, &req, NULL));
}

struct pl2303x_init {
	uint8_t	req_type;
	uint8_t	request;
	uint16_t value;
	uint16_t index;
	uint16_t length;
};

static const struct pl2303x_init pl2303x[] = {
	{UT_READ_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0x8484, 0, 1},
	{UT_WRITE_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0x0404, 0, 0},
	{UT_READ_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0x8484, 0, 1},
	{UT_READ_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0x8383, 0, 1},
	{UT_READ_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0x8484, 0, 1},
	{UT_WRITE_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0x0404, 1, 0},
	{UT_READ_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0x8484, 0, 1},
	{UT_READ_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0x8383, 0, 1},
	{UT_WRITE_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 0, 1, 0},
	{UT_WRITE_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 1, 0, 0},
	{UT_WRITE_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 2, 0x44, 0},
	{UT_WRITE_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 8, 0, 0},
	{UT_WRITE_VENDOR_DEVICE, UPLCOM_SET_REQUEST, 9, 0, 0},
};

#define	N_PL2302X_INIT	(sizeof(pl2303x)/sizeof(pl2303x[0]))

static int
uplcom_pl2303x_init(struct usbd_device *udev)
{
	usb_device_request_t req;
	usbd_status_t err;
	uint8_t buf[4];
	uint8_t i;

	for (i = 0; i != N_PL2302X_INIT; i++) {
		req.bmRequestType = pl2303x[i].req_type;
		req.bRequest = pl2303x[i].request;
		USETW(req.wValue, pl2303x[i].value);
		USETW(req.wIndex, pl2303x[i].index);
		USETW(req.wLength, pl2303x[i].length);

		err = usbd_do_request(udev, &Giant, &req, buf);
		if (err) {
			DPRINTF(0, "error=%s\n", usbd_errstr(err));
			return (EIO);
		}
	}
	return (0);
}

static void
uplcom_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uplcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (onoff)
		sc->sc_line |= UCDC_LINE_DTR;
	else
		sc->sc_line &= ~UCDC_LINE_DTR;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SET_CONTROL_LINE_STATE;
	USETW(req.wValue, sc->sc_line);
	req.wIndex[0] = sc->sc_data_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	uplcom_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uplcom_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uplcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (onoff)
		sc->sc_line |= UCDC_LINE_RTS;
	else
		sc->sc_line &= ~UCDC_LINE_RTS;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SET_CONTROL_LINE_STATE;
	USETW(req.wValue, sc->sc_line);
	req.wIndex[0] = sc->sc_data_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	uplcom_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uplcom_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uplcom_softc *sc = ucom->sc_parent;
	usb_device_request_t req;
	uint16_t temp;

	DPRINTF(0, "onoff = %d\n", onoff);

	temp = (onoff ? UCDC_BREAK_ON : UCDC_BREAK_OFF);

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SEND_BREAK;
	USETW(req.wValue, temp);
	req.wIndex[0] = sc->sc_data_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	uplcom_cfg_do_request(sc, &req, NULL);
	return;
}

static const int32_t uplcom_rates[] = {
	75, 150, 300, 600, 1200, 1800, 2400, 3600, 4800, 7200, 9600, 14400,
	19200, 28800, 38400, 57600, 115200,
	/*
	 * Higher speeds are probably possible. PL2303X supports up to
	 * 6Mb and can set any rate
	 */
	230400, 460800, 614400, 921600, 1228800
};

#define	N_UPLCOM_RATES	(sizeof(uplcom_rates)/sizeof(uplcom_rates[0]))

static int
uplcom_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	uint8_t i;

	DPRINTF(0, "\n");

	/* check requested baud rate */

	for (i = 0;; i++) {

		if (i != N_UPLCOM_RATES) {
			if (uplcom_rates[i] == t->c_ospeed) {
				break;
			}
		} else {
			DPRINTF(0, "invalid baud rate (%d)\n", t->c_ospeed);
			return (EIO);
		}
	}

	return (0);
}

static void
uplcom_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uplcom_softc *sc = ucom->sc_parent;
	usb_cdc_line_state_t ls;
	usb_device_request_t req;

	DPRINTF(0, "sc = %p\n", sc);

	bzero(&ls, sizeof(ls));

	USETDW(ls.dwDTERate, t->c_ospeed);

	if (t->c_cflag & CSTOPB) {
		ls.bCharFormat = UCDC_STOP_BIT_2;
	} else {
		ls.bCharFormat = UCDC_STOP_BIT_1;
	}

	if (t->c_cflag & PARENB) {
		if (t->c_cflag & PARODD) {
			ls.bParityType = UCDC_PARITY_ODD;
		} else {
			ls.bParityType = UCDC_PARITY_EVEN;
		}
	} else {
		ls.bParityType = UCDC_PARITY_NONE;
	}

	switch (t->c_cflag & CSIZE) {
	case CS5:
		ls.bDataBits = 5;
		break;
	case CS6:
		ls.bDataBits = 6;
		break;
	case CS7:
		ls.bDataBits = 7;
		break;
	case CS8:
		ls.bDataBits = 8;
		break;
	}

	DPRINTF(0, "rate=%d fmt=%d parity=%d bits=%d\n",
	    UGETDW(ls.dwDTERate), ls.bCharFormat,
	    ls.bParityType, ls.bDataBits);

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SET_LINE_CODING;
	USETW(req.wValue, 0);
	req.wIndex[0] = sc->sc_data_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, UCDC_LINE_STATE_LENGTH);

	uplcom_cfg_do_request(sc, &req, &ls);

	if (t->c_cflag & CRTSCTS) {

		DPRINTF(0, "crtscts = on\n");

		req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
		req.bRequest = UPLCOM_SET_REQUEST;
		USETW(req.wValue, 0);
		if (sc->sc_chiptype == TYPE_PL2303X)
			USETW(req.wIndex, UPLCOM_SET_CRTSCTS_PL2303X);
		else
			USETW(req.wIndex, UPLCOM_SET_CRTSCTS);
		USETW(req.wLength, 0);

		uplcom_cfg_do_request(sc, &req, NULL);
	} else {
		req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
		req.bRequest = UPLCOM_SET_REQUEST;
		USETW(req.wValue, 0);
		USETW(req.wIndex, 0);
		USETW(req.wLength, 0);
		uplcom_cfg_do_request(sc, &req, NULL);
	}
	return;
}

static void
uplcom_start_read(struct ucom_softc *ucom)
{
	struct uplcom_softc *sc = ucom->sc_parent;

	/* start interrupt endpoint */
	usbd_transfer_start(sc->sc_xfer[4]);

	/* start read endpoint */
	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
uplcom_stop_read(struct ucom_softc *ucom)
{
	struct uplcom_softc *sc = ucom->sc_parent;

	/* stop interrupt endpoint */
	usbd_transfer_stop(sc->sc_xfer[4]);

	/* stop read endpoint */
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
uplcom_start_write(struct ucom_softc *ucom)
{
	struct uplcom_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
uplcom_stop_write(struct ucom_softc *ucom)
{
	struct uplcom_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

static void
uplcom_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr)
{
	struct uplcom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "\n");

	*lsr = sc->sc_lsr;
	*msr = sc->sc_msr;
	return;
}

static int
uplcom_ioctl(struct ucom_softc *ucom, uint32_t cmd, caddr_t data, int flag,
    struct thread *td)
{
	return (ENOTTY);
}

static void
uplcom_intr_callback(struct usbd_xfer *xfer)
{
	struct uplcom_softc *sc = xfer->priv_sc;
	uint8_t buf[9];

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		DPRINTF(0, "actlen = %u\n", xfer->actlen);

		if (xfer->actlen >= 9) {

			usbd_copy_out(xfer->frbuffers + 0, 0, buf, sizeof(buf));

			DPRINTF(0, "status = 0x%02x\n", buf[8]);

			sc->sc_lsr = 0;
			sc->sc_msr = 0;

			if (buf[8] & RSAQ_STATUS_CTS) {
				sc->sc_msr |= SER_CTS;
			}
			if (buf[8] & RSAQ_STATUS_DSR) {
				sc->sc_msr |= SER_DSR;
			}
			if (buf[8] & RSAQ_STATUS_DCD) {
				sc->sc_msr |= SER_DCD;
			}
			ucom_status_change(&(sc->sc_ucom));
		}
	case USBD_ST_SETUP:
		if (sc->sc_flag & UPLCOM_FLAG_INTR_STALL) {
			usbd_transfer_start(sc->sc_xfer[5]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= UPLCOM_FLAG_INTR_STALL;
			usbd_transfer_start(sc->sc_xfer[5]);
		}
		return;

	}
}

static void
uplcom_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uplcom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flag &= ~UPLCOM_FLAG_INTR_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uplcom_write_callback(struct usbd_xfer *xfer)
{
	struct uplcom_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
	case USBD_ST_TRANSFERRED:
		if (sc->sc_flag & UPLCOM_FLAG_WRITE_STALL) {
			usbd_transfer_start(sc->sc_xfer[2]);
			return;
		}
		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers + 0, 0,
		    UPLCOM_BULK_BUF_SIZE, &actlen)) {

			DPRINTF(0, "actlen = %d\n", actlen);

			xfer->frlengths[0] = actlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= UPLCOM_FLAG_WRITE_STALL;
			usbd_transfer_start(sc->sc_xfer[2]);
		}
		return;

	}
}

static void
uplcom_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uplcom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flag &= ~UPLCOM_FLAG_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uplcom_read_callback(struct usbd_xfer *xfer)
{
	struct uplcom_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		ucom_put_data(&(sc->sc_ucom), xfer->frbuffers + 0, 0, xfer->actlen);

	case USBD_ST_SETUP:
		if (sc->sc_flag & UPLCOM_FLAG_READ_STALL) {
			usbd_transfer_start(sc->sc_xfer[3]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= UPLCOM_FLAG_READ_STALL;
			usbd_transfer_start(sc->sc_xfer[3]);
		}
		return;

	}
}

static void
uplcom_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uplcom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flag &= ~UPLCOM_FLAG_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uplcom_cfg_do_request(struct uplcom_softc *sc, usb_device_request_t *req,
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
