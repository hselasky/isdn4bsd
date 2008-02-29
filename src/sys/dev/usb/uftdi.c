/*	$NetBSD: uftdi.c,v 1.13 2002/09/23 05:51:23 simonb Exp $	*/

/*-
 * Copyright (c) 2000 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net).
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uftdi.c,v 1.37 2007/06/22 05:53:05 imp Exp $");

/*
 * NOTE: all function names beginning like "uftdi_cfg_" can only
 * be called from within the config thread function !
 */

/*
 * FTDI FT8U100AX serial adapter driver
 */

#include <sys/cdefs.h>
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
#include <dev/usb/uftdireg.h>

#include "usbdevs.h"

#ifdef USB_DEBUG
#define	DPRINTF(sc,n,fmt,...)					\
  do { if (uftdi_debug > (n)) {				\
      printf("%s: %s: " fmt, (sc)->sc_name,			\
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uftdi_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, uftdi, CTLFLAG_RW, 0, "USB uftdi");
SYSCTL_INT(_hw_usb_uftdi, OID_AUTO, debug, CTLFLAG_RW,
    &uftdi_debug, 0, "uftdi debug level");
#else
#define	DPRINTF(...) do { } while (0)
#endif

#define	UFTDI_CONFIG_INDEX	0
#define	UFTDI_IFACE_INDEX	0
#define	UFTDI_ENDPT_MAX		4

#define	UFTDI_IBUFSIZE 64		/* bytes, maximum number of bytes per
					 * frame */
#define	UFTDI_OBUFSIZE 64		/* bytes, cannot be increased due to
					 * do size encoding */

struct uftdi_softc {
	struct ucom_super_softc sc_super_ucom;
	struct ucom_softc sc_ucom;

	struct usbd_device *sc_udev;
	struct usbd_xfer *sc_xfer[UFTDI_ENDPT_MAX];
	device_t sc_dev;

	uint32_t sc_unit;
	enum uftdi_type sc_type;

	uint16_t sc_last_lcr;

	uint8_t	sc_iface_index;
	uint8_t	sc_hdrlen;

	uint8_t	sc_msr;
	uint8_t	sc_lsr;

	uint8_t	sc_flag;
#define	UFTDI_FLAG_WRITE_STALL  0x01
#define	UFTDI_FLAG_READ_STALL   0x02

	uint8_t	sc_name[16];
};

struct uftdi_param_config {
	uint16_t rate;
	uint16_t lcr;
	uint8_t	v_start;
	uint8_t	v_stop;
	uint8_t	v_flow;
};

/* prototypes */

static device_probe_t uftdi_probe;
static device_attach_t uftdi_attach;
static device_detach_t uftdi_detach;

static usbd_callback_t uftdi_write_callback;
static usbd_callback_t uftdi_write_clear_stall_callback;
static usbd_callback_t uftdi_read_callback;
static usbd_callback_t uftdi_read_clear_stall_callback;

static void uftdi_cfg_do_request(struct uftdi_softc *sc, usb_device_request_t *req, void *data);
static void uftdi_cfg_open(struct ucom_softc *ucom);
static void uftdi_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff);
static void uftdi_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff);
static void uftdi_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff);
static int uftdi_set_parm_soft(struct termios *t, struct uftdi_param_config *cfg, uint8_t type);
static int uftdi_pre_param(struct ucom_softc *ucom, struct termios *t);
static void uftdi_cfg_param(struct ucom_softc *ucom, struct termios *t);
static void uftdi_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr);
static void uftdi_start_read(struct ucom_softc *ucom);
static void uftdi_stop_read(struct ucom_softc *ucom);
static void uftdi_start_write(struct ucom_softc *ucom);
static void uftdi_stop_write(struct ucom_softc *ucom);

static const struct usbd_config uftdi_config[UFTDI_ENDPT_MAX] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.mh.bufsize = UFTDI_OBUFSIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &uftdi_write_callback,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.bufsize = UFTDI_IBUFSIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &uftdi_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &uftdi_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &uftdi_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},
};

static const struct ucom_callback uftdi_callback = {
	.ucom_cfg_get_status = &uftdi_cfg_get_status,
	.ucom_cfg_set_dtr = &uftdi_cfg_set_dtr,
	.ucom_cfg_set_rts = &uftdi_cfg_set_rts,
	.ucom_cfg_set_break = &uftdi_cfg_set_break,
	.ucom_cfg_param = &uftdi_cfg_param,
	.ucom_cfg_open = &uftdi_cfg_open,
	.ucom_pre_param = &uftdi_pre_param,
	.ucom_start_read = &uftdi_start_read,
	.ucom_stop_read = &uftdi_stop_read,
	.ucom_start_write = &uftdi_start_write,
	.ucom_stop_write = &uftdi_stop_write,
};

static device_method_t uftdi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, uftdi_probe),
	DEVMETHOD(device_attach, uftdi_attach),
	DEVMETHOD(device_detach, uftdi_detach),

	{0, 0}
};

static devclass_t uftdi_devclass;

static driver_t uftdi_driver = {
	.name = "uftdi",
	.methods = uftdi_methods,
	.size = sizeof(struct uftdi_softc),
};

DRIVER_MODULE(uftdi, uhub, uftdi_driver, uftdi_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uftdi, usb, 1, 1, 1);
MODULE_DEPEND(uftdi, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static int
uftdi_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->iface != NULL) {
		if ((uaa->vendor == USB_VENDOR_FTDI) &&
		    (uaa->product == USB_PRODUCT_FTDI_SERIAL_2232C)) {
			return (UMATCH_VENDOR_IFACESUBCLASS);
		}
		return (UMATCH_NONE);
	}
	if ((uaa->vendor == USB_VENDOR_FTDI) &&
	    ((uaa->product == USB_PRODUCT_FTDI_SERIAL_8U100AX) ||
	    (uaa->product == USB_PRODUCT_FTDI_SERIAL_8U232AM) ||
	    (uaa->product == USB_PRODUCT_FTDI_SEMC_DSS20) ||
	    (uaa->product == USB_PRODUCT_FTDI_CFA_631) ||
	    (uaa->product == USB_PRODUCT_FTDI_CFA_632) ||
	    (uaa->product == USB_PRODUCT_FTDI_CFA_633) ||
	    (uaa->product == USB_PRODUCT_FTDI_CFA_634) ||
	    (uaa->product == USB_PRODUCT_FTDI_CFA_635) ||
	    (uaa->product == USB_PRODUCT_FTDI_USBSERIAL) ||
	    (uaa->product == USB_PRODUCT_FTDI_MX2_3) ||
	    (uaa->product == USB_PRODUCT_FTDI_MX4_5) ||
	    (uaa->product == USB_PRODUCT_FTDI_LK202) ||
	    (uaa->product == USB_PRODUCT_FTDI_LK204) ||
	    (uaa->product == USB_PRODUCT_FTDI_TACTRIX_OPENPORT_13M) ||
	    (uaa->product == USB_PRODUCT_FTDI_TACTRIX_OPENPORT_13S) ||
	    (uaa->product == USB_PRODUCT_FTDI_TACTRIX_OPENPORT_13U) ||
	    (uaa->product == USB_PRODUCT_FTDI_EISCOU) ||
	    (uaa->product == USB_PRODUCT_FTDI_UOPTBR) ||
	    (uaa->product == USB_PRODUCT_FTDI_EMCU2D) ||
	    (uaa->product == USB_PRODUCT_FTDI_PCMSFU) ||
	    (uaa->product == USB_PRODUCT_FTDI_EMCU2H))) {
		return (UMATCH_VENDOR_PRODUCT);
	}
	if ((uaa->vendor == USB_VENDOR_SIIG2) &&
	    (uaa->product == USB_PRODUCT_SIIG2_US2308)) {
		return (UMATCH_VENDOR_PRODUCT);
	}
	if ((uaa->vendor == USB_VENDOR_INTREPIDCS) &&
	    ((uaa->product == USB_PRODUCT_INTREPIDCS_VALUECAN) ||
	    (uaa->product == USB_PRODUCT_INTREPIDCS_NEOVI))) {
		return (UMATCH_VENDOR_PRODUCT);
	}
	if ((uaa->vendor == USB_VENDOR_BBELECTRONICS) &&
	    (uaa->product == USB_PRODUCT_BBELECTRONICS_USOTL4)) {
		return (UMATCH_VENDOR_PRODUCT);
	}
	if (uaa->vendor == USB_VENDOR_MELCO &&
	    (uaa->product == USB_PRODUCT_MELCO_PCOPRS1))
		return (UMATCH_VENDOR_PRODUCT);

	return (UMATCH_NONE);
}

static int
uftdi_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uftdi_softc *sc = device_get_softc(dev);
	usb_interface_descriptor_t *id;
	int32_t error;

	if (sc == NULL) {
		return (ENOMEM);
	}
	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;
	sc->sc_unit = device_get_unit(dev);

	usbd_set_device_desc(dev);

	snprintf(sc->sc_name, sizeof(sc->sc_name),
	    "%s", device_get_nameunit(dev));

	DPRINTF(sc, 0, "\n");

	if (uaa->iface == NULL) {

		error = usbd_set_config_index(uaa->device, UFTDI_CONFIG_INDEX, 1);

		if (error) {
			device_printf(dev, "failed to set configuration, "
			    "error=%s\n", usbd_errstr(error));
			goto detach;
		}
		sc->sc_iface_index = UFTDI_IFACE_INDEX;
	} else {
		sc->sc_iface_index = uaa->iface_index;
	}

	switch (uaa->vendor) {
	case USB_VENDOR_FTDI:
		switch (uaa->product) {
		case USB_PRODUCT_FTDI_SERIAL_8U100AX:
			sc->sc_type = UFTDI_TYPE_SIO;
			sc->sc_hdrlen = 1;
			break;

		case USB_PRODUCT_FTDI_SEMC_DSS20:
		case USB_PRODUCT_FTDI_SERIAL_8U232AM:
		case USB_PRODUCT_FTDI_SERIAL_2232C:
		case USB_PRODUCT_FTDI_CFA_631:
		case USB_PRODUCT_FTDI_CFA_632:
		case USB_PRODUCT_FTDI_CFA_633:
		case USB_PRODUCT_FTDI_CFA_634:
		case USB_PRODUCT_FTDI_CFA_635:
		case USB_PRODUCT_FTDI_USBSERIAL:
		case USB_PRODUCT_FTDI_MX2_3:
		case USB_PRODUCT_FTDI_MX4_5:
		case USB_PRODUCT_FTDI_LK202:
		case USB_PRODUCT_FTDI_LK204:
		case USB_PRODUCT_FTDI_TACTRIX_OPENPORT_13M:
		case USB_PRODUCT_FTDI_TACTRIX_OPENPORT_13S:
		case USB_PRODUCT_FTDI_TACTRIX_OPENPORT_13U:
		case USB_PRODUCT_FTDI_EISCOU:
		case USB_PRODUCT_FTDI_UOPTBR:
		case USB_PRODUCT_FTDI_EMCU2D:
		case USB_PRODUCT_FTDI_PCMSFU:
		case USB_PRODUCT_FTDI_EMCU2H:
			sc->sc_type = UFTDI_TYPE_8U232AM;
			sc->sc_hdrlen = 0;
			break;

		default:		/* Can't happen */
			goto detach;
		}
		break;

	case USB_VENDOR_INTREPIDCS:
		switch (uaa->product) {
		case USB_PRODUCT_INTREPIDCS_VALUECAN:
		case USB_PRODUCT_INTREPIDCS_NEOVI:
			sc->sc_type = UFTDI_TYPE_8U232AM;
			sc->sc_hdrlen = 0;
			break;

		default:		/* Can't happen */
			goto detach;
		}
		break;

	case USB_VENDOR_SIIG2:
		switch (uaa->product) {
		case USB_PRODUCT_SIIG2_US2308:
			sc->sc_type = UFTDI_TYPE_8U232AM;
			sc->sc_hdrlen = 0;
			break;

		default:		/* Can't happen */
			goto detach;
		}
		break;

	case USB_VENDOR_BBELECTRONICS:
		switch (uaa->product) {
		case USB_PRODUCT_BBELECTRONICS_USOTL4:
			sc->sc_type = UFTDI_TYPE_8U232AM;
			sc->sc_hdrlen = 0;
			break;

		default:		/* Can't happen */
			goto detach;
		}
		break;

	case USB_VENDOR_MELCO:
		switch (uaa->product) {
		case USB_PRODUCT_MELCO_PCOPRS1:
			sc->sc_type = UFTDI_TYPE_8U232AM;
			sc->sc_hdrlen = 0;
			break;

		default:		/* Can't happen */
			goto detach;
		}
		break;

	default:			/* Can't happen */
		goto detach;
	}

	error = usbd_transfer_setup(uaa->device,
	    &(sc->sc_iface_index), sc->sc_xfer, uftdi_config,
	    UFTDI_ENDPT_MAX, sc, &Giant);

	if (error) {
		device_printf(dev, "allocating USB "
		    "transfers failed!\n");
		goto detach;
	}
	sc->sc_ucom.sc_portno = FTDI_PIT_SIOA;

	if (uaa->iface) {
		id = usbd_get_interface_descriptor(uaa->iface);

		if (id == NULL) {
			goto detach;
		}
		sc->sc_ucom.sc_portno += id->bInterfaceNumber;
	}
	/* clear stall at first run */

	sc->sc_flag |= (UFTDI_FLAG_WRITE_STALL |
	    UFTDI_FLAG_READ_STALL);

	/* set a valid "lcr" value */

	sc->sc_last_lcr =
	    (FTDI_SIO_SET_DATA_STOP_BITS_2 |
	    FTDI_SIO_SET_DATA_PARITY_NONE |
	    FTDI_SIO_SET_DATA_BITS(8));

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
	    &uftdi_callback, &Giant);
	if (error) {
		goto detach;
	}
	return (0);			/* success */

detach:
	uftdi_detach(dev);
	return (ENXIO);
}

static int
uftdi_detach(device_t dev)
{
	struct uftdi_softc *sc = device_get_softc(dev);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UFTDI_ENDPT_MAX);

	return (0);
}

static void
uftdi_cfg_do_request(struct uftdi_softc *sc, usb_device_request_t *req,
    void *data)
{
	uint16_t length;
	usbd_status_t err;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
		goto error;
	}
	err = usbd_do_request_flags
	    (sc->sc_udev, &Giant, req, data, 0, NULL, 1000);

	if (err) {

		DPRINTF(sc, -1, "device request failed, err=%s "
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
uftdi_cfg_open(struct ucom_softc *ucom)
{
	struct uftdi_softc *sc = ucom->sc_parent;
	uint16_t wIndex = ucom->sc_portno;
	usb_device_request_t req;

	DPRINTF(sc, 0, "");

	/* perform a full reset on the device */

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_RESET;
	USETW(req.wValue, FTDI_SIO_RESET_SIO);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	/* turn on RTS/CTS flow control */

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_FLOW_CTRL;
	USETW(req.wValue, 0);
	USETW2(req.wIndex, FTDI_SIO_RTS_CTS_HS, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	/*
	 * NOTE: with the new UCOM layer there will always be a
	 * "uftdi_cfg_param()" call after "open()", so there is no need for
	 * "open()" to configure anything
	 */
	return;
}

static void
uftdi_write_callback(struct usbd_xfer *xfer)
{
	struct uftdi_softc *sc = xfer->priv_sc;
	uint32_t actlen;
	uint8_t buf[1];

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
	case USBD_ST_TRANSFERRED:
		if (sc->sc_flag & UFTDI_FLAG_WRITE_STALL) {
			usbd_transfer_start(sc->sc_xfer[2]);
			return;
		}
		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers,
		    sc->sc_hdrlen, UFTDI_OBUFSIZE - sc->sc_hdrlen,
		    &actlen)) {

			if (sc->sc_hdrlen > 0) {
				buf[0] =
				    FTDI_OUT_TAG(actlen, sc->sc_ucom.sc_portno);
				usbd_copy_in(xfer->frbuffers, 0, buf, 1);
			}
			xfer->frlengths[0] = actlen + sc->sc_hdrlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= UFTDI_FLAG_WRITE_STALL;
			usbd_transfer_start(sc->sc_xfer[2]);
		}
		return;

	}
}

static void
uftdi_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uftdi_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flag &= ~UFTDI_FLAG_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uftdi_read_callback(struct usbd_xfer *xfer)
{
	struct uftdi_softc *sc = xfer->priv_sc;
	uint8_t buf[2];
	uint8_t msr;
	uint8_t lsr;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		if (xfer->actlen < 2) {
			goto tr_setup;
		}
		usbd_copy_out(xfer->frbuffers, 0, buf, 2);

		msr = FTDI_GET_MSR(buf);
		lsr = FTDI_GET_LSR(buf);

		if ((sc->sc_msr != msr) ||
		    ((sc->sc_lsr & FTDI_LSR_MASK) != (lsr & FTDI_LSR_MASK))) {
			DPRINTF(sc, 0, "status change msr=0x%02x (0x%02x) "
			    "lsr=0x%02x (0x%02x)\n", msr, sc->sc_msr,
			    lsr, sc->sc_lsr);

			sc->sc_msr = msr;
			sc->sc_lsr = lsr;

			ucom_status_change(&(sc->sc_ucom));
		}
		xfer->actlen -= 2;

		if (xfer->actlen > 0) {
			ucom_put_data(&(sc->sc_ucom), xfer->frbuffers, 2,
			    xfer->actlen);
		}
	case USBD_ST_SETUP:
tr_setup:
		if (sc->sc_flag & UFTDI_FLAG_READ_STALL) {
			usbd_transfer_start(sc->sc_xfer[3]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= UFTDI_FLAG_READ_STALL;
			usbd_transfer_start(sc->sc_xfer[3]);
		}
		return;

	}
}

static void
uftdi_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uftdi_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flag &= ~UFTDI_FLAG_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uftdi_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uftdi_softc *sc = ucom->sc_parent;
	uint16_t wIndex = ucom->sc_portno;
	uint16_t wValue;
	usb_device_request_t req;

	wValue = onoff ? FTDI_SIO_SET_DTR_HIGH : FTDI_SIO_SET_DTR_LOW;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_MODEM_CTRL;
	USETW(req.wValue, wValue);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uftdi_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uftdi_softc *sc = ucom->sc_parent;
	uint16_t wIndex = ucom->sc_portno;
	uint16_t wValue;
	usb_device_request_t req;

	wValue = onoff ? FTDI_SIO_SET_RTS_HIGH : FTDI_SIO_SET_RTS_LOW;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_MODEM_CTRL;
	USETW(req.wValue, wValue);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);
	return;
}

static void
uftdi_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uftdi_softc *sc = ucom->sc_parent;
	uint16_t wIndex = ucom->sc_portno;
	uint16_t wValue;
	usb_device_request_t req;

	if (onoff) {
		sc->sc_last_lcr |= FTDI_SIO_SET_BREAK;
	} else {
		sc->sc_last_lcr &= ~FTDI_SIO_SET_BREAK;
	}

	wValue = sc->sc_last_lcr;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_DATA;
	USETW(req.wValue, wValue);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);
	return;
}

static int
uftdi_set_parm_soft(struct termios *t,
    struct uftdi_param_config *cfg, uint8_t type)
{
	bzero(cfg, sizeof(*cfg));

	switch (type) {
	case UFTDI_TYPE_SIO:
		switch (t->c_ospeed) {
		case 300:
			cfg->rate = ftdi_sio_b300;
			break;
		case 600:
			cfg->rate = ftdi_sio_b600;
			break;
		case 1200:
			cfg->rate = ftdi_sio_b1200;
			break;
		case 2400:
			cfg->rate = ftdi_sio_b2400;
			break;
		case 4800:
			cfg->rate = ftdi_sio_b4800;
			break;
		case 9600:
			cfg->rate = ftdi_sio_b9600;
			break;
		case 19200:
			cfg->rate = ftdi_sio_b19200;
			break;
		case 38400:
			cfg->rate = ftdi_sio_b38400;
			break;
		case 57600:
			cfg->rate = ftdi_sio_b57600;
			break;
		case 115200:
			cfg->rate = ftdi_sio_b115200;
			break;
		default:
			return (EINVAL);
		}
		break;

	case UFTDI_TYPE_8U232AM:
		switch (t->c_ospeed) {
		case 300:
			cfg->rate = ftdi_8u232am_b300;
			break;
		case 600:
			cfg->rate = ftdi_8u232am_b600;
			break;
		case 1200:
			cfg->rate = ftdi_8u232am_b1200;
			break;
		case 2400:
			cfg->rate = ftdi_8u232am_b2400;
			break;
		case 4800:
			cfg->rate = ftdi_8u232am_b4800;
			break;
		case 9600:
			cfg->rate = ftdi_8u232am_b9600;
			break;
		case 19200:
			cfg->rate = ftdi_8u232am_b19200;
			break;
		case 38400:
			cfg->rate = ftdi_8u232am_b38400;
			break;
		case 57600:
			cfg->rate = ftdi_8u232am_b57600;
			break;
		case 115200:
			cfg->rate = ftdi_8u232am_b115200;
			break;
		case 230400:
			cfg->rate = ftdi_8u232am_b230400;
			break;
		case 460800:
			cfg->rate = ftdi_8u232am_b460800;
			break;
		case 921600:
			cfg->rate = ftdi_8u232am_b921600;
			break;
		case 2000000:
			cfg->rate = ftdi_8u232am_b2000000;
			break;
		case 3000000:
			cfg->rate = ftdi_8u232am_b3000000;
			break;
		default:
			return (EINVAL);
		}
		break;
	}

	if (t->c_cflag & CSTOPB)
		cfg->lcr = FTDI_SIO_SET_DATA_STOP_BITS_2;
	else
		cfg->lcr = FTDI_SIO_SET_DATA_STOP_BITS_1;

	if (t->c_cflag & PARENB) {
		if (t->c_cflag & PARODD) {
			cfg->lcr |= FTDI_SIO_SET_DATA_PARITY_ODD;
		} else {
			cfg->lcr |= FTDI_SIO_SET_DATA_PARITY_EVEN;
		}
	} else {
		cfg->lcr |= FTDI_SIO_SET_DATA_PARITY_NONE;
	}

	switch (t->c_cflag & CSIZE) {
	case CS5:
		cfg->lcr |= FTDI_SIO_SET_DATA_BITS(5);
		break;

	case CS6:
		cfg->lcr |= FTDI_SIO_SET_DATA_BITS(6);
		break;

	case CS7:
		cfg->lcr |= FTDI_SIO_SET_DATA_BITS(7);
		break;

	case CS8:
		cfg->lcr |= FTDI_SIO_SET_DATA_BITS(8);
		break;
	}

	if (t->c_cflag & CRTSCTS) {
		cfg->v_flow = FTDI_SIO_RTS_CTS_HS;
	} else if (t->c_iflag & (IXON | IXOFF)) {
		cfg->v_flow = FTDI_SIO_XON_XOFF_HS;
		cfg->v_start = t->c_cc[VSTART];
		cfg->v_stop = t->c_cc[VSTOP];
	} else {
		cfg->v_flow = FTDI_SIO_DISABLE_FLOW_CTRL;
	}

	return (0);
}

static int
uftdi_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uftdi_softc *sc = ucom->sc_parent;
	struct uftdi_param_config cfg;

	DPRINTF(sc, 0, "\n");

	return (uftdi_set_parm_soft(t, &cfg, sc->sc_type));
}

static void
uftdi_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uftdi_softc *sc = ucom->sc_parent;
	uint16_t wIndex = ucom->sc_portno;
	struct uftdi_param_config cfg;
	usb_device_request_t req;

	if (uftdi_set_parm_soft(t, &cfg, sc->sc_type)) {
		/* should not happen */
		return;
	}
	sc->sc_last_lcr = cfg.lcr;

	DPRINTF(sc, 0, "\n");

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_BAUD_RATE;
	USETW(req.wValue, cfg.rate);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_DATA;
	USETW(req.wValue, cfg.lcr);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_FLOW_CTRL;
	USETW2(req.wValue, cfg.v_stop, cfg.v_start);
	USETW2(req.wIndex, cfg.v_flow, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	return;
}

static void
uftdi_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	DPRINTF(sc, 0, "msr=0x%02x lsr=0x%02x\n",
	    sc->sc_msr, sc->sc_lsr);

	*msr = sc->sc_msr;
	*lsr = sc->sc_lsr;
	return;
}

static void
uftdi_start_read(struct ucom_softc *ucom)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
uftdi_stop_read(struct ucom_softc *ucom)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
uftdi_start_write(struct ucom_softc *ucom)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
uftdi_stop_write(struct ucom_softc *ucom)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}
