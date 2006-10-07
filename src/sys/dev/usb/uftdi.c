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
__FBSDID("$FreeBSD: src/sys/dev/usb/uftdi.c,v 1.24 2006/09/07 00:06:41 imp Exp $");
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
#include <sys/taskqueue.h>

#define usbd_config_td_cc uftdi_config_copy
#define usbd_config_td_softc uftdi_softc

#include <dev/usb2/usb_port.h>
#include <dev/usb2/usb.h>
#include <dev/usb2/usb_subr.h>
#include <dev/usb2/usb_quirks.h>
#include <dev/usb2/usb_cdc.h>

#include <dev/usb/ucomvar.h>
#include <dev/usb/uftdireg.h>

#include "usbdevs.h"

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)					\
  do { if (uftdi_debug > (n)) {				\
      printf("%s: %s: " fmt, (sc)->sc_name,			\
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uftdi_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uftdi, CTLFLAG_RW, 0, "USB uftdi");
SYSCTL_INT(_hw_usb_uftdi, OID_AUTO, debug, CTLFLAG_RW,
	   &uftdi_debug, 0, "uftdi debug level");
#else
#define DPRINTF(...)
#endif

#define UFTDI_CONFIG_INDEX	0
#define UFTDI_IFACE_INDEX	0
#define UFTDI_ENDPT_MAX		4

/*
 * These are the maximum number of bytes transferred per frame.
 * The output buffer size cannot be increased due to the size encoding.
 */
#define UFTDI_IBUFSIZE 64
#define UFTDI_OBUFSIZE 64

struct uftdi_softc {
	struct ucom_softc	sc_ucom;
	struct usbd_config_td	sc_config_td;

	struct usbd_device *	sc_udev;
	struct usbd_xfer *	sc_xfer[UFTDI_ENDPT_MAX];
	device_t		sc_dev;

	u_int32_t		sc_unit;
	enum uftdi_type		sc_type;

	u_int16_t		sc_last_lcr;
	u_int16_t		sc_rate;

	u_int8_t		sc_iface_index;
	u_int8_t		sc_hdrlen;

	u_int8_t		sc_msr;
	u_int8_t		sc_lsr;

	u_int8_t		sc_dtr_onoff;
	u_int8_t		sc_rts_onoff;
	u_int8_t		sc_break_onoff;

	u_int8_t		sc_flow;
	u_int8_t		sc_flow_v_start;
	u_int8_t		sc_flow_v_stop;

	u_int8_t		sc_flag;
#define UFTDI_FLAG_WRITE_STALL  0x01
#define UFTDI_FLAG_READ_STALL   0x02

	u_int8_t		sc_name[16];
};

struct uftdi_config_copy {
	u_int16_t	last_lcr;
	u_int16_t	rate;

	u_int8_t	dtr_onoff;
	u_int8_t	rts_onoff;
	u_int8_t	break_onoff;

	u_int8_t	flow;
	u_int8_t	v_start;
	u_int8_t	v_stop;
};

/* prototypes */

static device_probe_t uftdi_probe;
static device_attach_t uftdi_attach;
static device_detach_t uftdi_detach;

static void
uftdi_config_copy(struct uftdi_softc *sc, 
		  struct uftdi_config_copy *cc, u_int16_t refcount);
static void
uftdi_cfg_do_request(struct uftdi_softc *sc, usb_device_request_t *req, 
		     void *data);
static int
uftdi_open(struct ucom_softc *ucom);

static void
uftdi_cfg_open(struct uftdi_softc *sc,
	       struct uftdi_config_copy *cc, u_int16_t refcount);
static void
uftdi_write_callback(struct usbd_xfer *xfer);

static void
uftdi_write_clear_stall_callback(struct usbd_xfer *xfer);

static void
uftdi_read_callback(struct usbd_xfer *xfer);

static void
uftdi_read_clear_stall_callback(struct usbd_xfer *xfer);

static void
uftdi_set_dtr(struct ucom_softc *ucom, u_int8_t onoff);

static void
uftdi_cfg_set_dtr(struct uftdi_softc *sc,
		  struct uftdi_config_copy *cc, u_int16_t refcount);
static void
uftdi_set_rts(struct ucom_softc *ucom, u_int8_t onoff);

static void
uftdi_cfg_set_rts(struct uftdi_softc *sc,
		  struct uftdi_config_copy *cc, u_int16_t refcount);
static void
uftdi_set_break(struct ucom_softc *ucom, u_int8_t onoff);

static void
uftdi_cfg_set_break(struct uftdi_softc *sc,
		    struct uftdi_config_copy *cc, u_int16_t refcount);
static int
uftdi_set_parm_soft(struct uftdi_softc *sc, u_int32_t ospeed, 
		    u_int8_t v_start, u_int8_t v_stop, 
		    u_int32_t cflag, u_int32_t iflag);
static int
uftdi_param(struct ucom_softc *ucom, struct termios *t);

static void
uftdi_cfg_parm(struct uftdi_softc *sc,
	       struct uftdi_config_copy *cc, u_int16_t refcount);
static void
uftdi_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr);

static void
uftdi_start_read(struct ucom_softc *ucom);

static void
uftdi_stop_read(struct ucom_softc *ucom);

static void
uftdi_start_write(struct ucom_softc *ucom);

static void
uftdi_stop_write(struct ucom_softc *ucom);

static const struct usbd_config uftdi_config[UFTDI_ENDPT_MAX] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = UFTDI_OBUFSIZE,
      .flags     = 0,
      .callback  = &uftdi_write_callback,
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = UFTDI_IBUFSIZE,
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uftdi_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = (USBD_USE_DMA),
      .callback  = &uftdi_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = (USBD_USE_DMA),
      .callback  = &uftdi_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static const struct ucom_callback uftdi_callback = {
    .ucom_get_status  = &uftdi_get_status,
    .ucom_set_dtr     = &uftdi_set_dtr,
    .ucom_set_rts     = &uftdi_set_rts,
    .ucom_set_break   = &uftdi_set_break,
    .ucom_param       = &uftdi_param,
    .ucom_open        = &uftdi_open,
    .ucom_start_read  = &uftdi_start_read,
    .ucom_stop_read   = &uftdi_stop_read,
    .ucom_start_write = &uftdi_start_write,
    .ucom_stop_write  = &uftdi_stop_write,
};

static device_method_t uftdi_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe, uftdi_probe),
    DEVMETHOD(device_attach, uftdi_attach),
    DEVMETHOD(device_detach, uftdi_detach),

    { 0, 0 }
};

static driver_t uftdi_driver = {
    .name    = "ucom",
    .methods = uftdi_methods,
    .size    = sizeof (struct uftdi_softc),
};

DRIVER_MODULE(uftdi, uhub, uftdi_driver, ucom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uftdi, usb, 1, 1, 1);
MODULE_DEPEND(uftdi, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static int
uftdi_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->iface != NULL) {
	    if ((uaa->vendor == USB_VENDOR_FTDI) &&
		(uaa->product == USB_PRODUCT_FTDI_SERIAL_2232C)) {
	        return UMATCH_VENDOR_IFACESUBCLASS;
	    }
	    return UMATCH_NONE;
	}

	if ((uaa->vendor == USB_VENDOR_FTDI) &&
	    ((uaa->product == USB_PRODUCT_FTDI_SERIAL_8U100AX) ||
	     (uaa->product == USB_PRODUCT_FTDI_SERIAL_8U232AM) ||
	     (uaa->product == USB_PRODUCT_FTDI_SEMC_DSS20) ||
	     (uaa->product == USB_PRODUCT_FTDI_CFA_631) ||
	     (uaa->product == USB_PRODUCT_FTDI_CFA_632) ||
	     (uaa->product == USB_PRODUCT_FTDI_CFA_633) ||
	     (uaa->product == USB_PRODUCT_FTDI_CFA_634) ||
	     (uaa->product == USB_PRODUCT_FTDI_USBSERIAL) ||
	     (uaa->product == USB_PRODUCT_FTDI_MX2_3) ||
	     (uaa->product == USB_PRODUCT_FTDI_MX4_5) ||
	     (uaa->product == USB_PRODUCT_FTDI_LK202) ||
	     (uaa->product == USB_PRODUCT_FTDI_LK204))) {
	    return UMATCH_VENDOR_PRODUCT;
	}

	if ((uaa->vendor == USB_VENDOR_SIIG2) &&
	    (uaa->product == USB_PRODUCT_SIIG2_US2308)) {
	    return UMATCH_VENDOR_PRODUCT;
	}

	if ((uaa->vendor == USB_VENDOR_INTREPIDCS) &&
	    ((uaa->product == USB_PRODUCT_INTREPIDCS_VALUECAN) ||
	     (uaa->product == USB_PRODUCT_INTREPIDCS_NEOVI))) {
	    return UMATCH_VENDOR_PRODUCT;
	}

	if ((uaa->vendor == USB_VENDOR_BBELECTRONICS) &&
	    (uaa->product == USB_PRODUCT_BBELECTRONICS_USOTL4)) {
	    return UMATCH_VENDOR_PRODUCT;
	}

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
	    return ENOMEM;
	}

	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;
	sc->sc_unit = device_get_unit(dev);

	usbd_set_desc(dev, uaa->device);

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

	switch( uaa->vendor ) {
	case USB_VENDOR_FTDI:
	    switch( uaa->product ){
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
	    case USB_PRODUCT_FTDI_USBSERIAL:
	    case USB_PRODUCT_FTDI_MX2_3:
	    case USB_PRODUCT_FTDI_MX4_5:
	    case USB_PRODUCT_FTDI_LK202:
	    case USB_PRODUCT_FTDI_LK204:
	        sc->sc_type = UFTDI_TYPE_8U232AM;
		sc->sc_hdrlen = 0;
		break;

	    default:		/* Can't happen */
	        goto detach;
	    }
	    break;

	case USB_VENDOR_INTREPIDCS:
	    switch( uaa->product ){
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
	    switch( uaa->product ){
	    case USB_PRODUCT_SIIG2_US2308:
	        sc->sc_type = UFTDI_TYPE_8U232AM;
		sc->sc_hdrlen = 0;
		break;

	    default:		/* Can't happen */
	        goto detach;
	    }
	    break;

	case USB_VENDOR_BBELECTRONICS:
	    switch( uaa->product ){
	    case USB_PRODUCT_BBELECTRONICS_USOTL4:
	        sc->sc_type = UFTDI_TYPE_8U232AM;
		sc->sc_hdrlen = 0;
		break;

	    default:		/* Can't happen */
	        goto detach;
	    }
	    break;

	default:		/* Can't happen */
	    goto detach;
	}

	error = usbd_transfer_setup(uaa->device, sc->sc_iface_index, 
				    sc->sc_xfer, uftdi_config, UFTDI_ENDPT_MAX,
				    sc, &Giant);
	if (error) {
	    device_printf(dev, "allocating USB "
			  "transfers failed!\n");
	    goto detach;
	}

	error = usbd_config_td_setup(&(sc->sc_config_td), sc, &Giant,
				     &uftdi_config_copy, NULL,
				     sizeof(struct uftdi_config_copy), 16);
	if (error) {
	    device_printf(dev, "could not setup config "
			  "thread!\n");
	    goto detach;
	}

        sc->sc_ucom.sc_parent = sc;
	sc->sc_ucom.sc_portno = FTDI_PIT_SIOA;
	sc->sc_ucom.sc_callback = &uftdi_callback;

	if (uaa->iface) {
	    id = usbd_get_interface_descriptor(uaa->iface);

	    if (id == NULL) {
	        goto detach;
	    }

	    sc->sc_ucom.sc_portno += id->bInterfaceNumber;
	}

	error = ucom_attach(&(sc->sc_ucom), dev);

	if (error) {
	    goto detach;
	}

	return 0; /* success */

 detach:
	uftdi_detach(dev);
	return ENXIO;
}

static int
uftdi_detach(device_t dev)
{
	struct uftdi_softc *sc = device_get_softc(dev);

	mtx_lock(&Giant);

	usbd_config_td_stop(&(sc->sc_config_td));

	mtx_unlock(&Giant);

	ucom_detach(&(sc->sc_ucom));

	usbd_transfer_unsetup(sc->sc_xfer, UFTDI_ENDPT_MAX);

	usbd_config_td_unsetup(&(sc->sc_config_td));

	return 0;
}

static void
uftdi_config_copy(struct uftdi_softc *sc, 
		  struct uftdi_config_copy *cc, u_int16_t refcount)
{
	bzero(cc, sizeof(*cc));

	cc->dtr_onoff = sc->sc_dtr_onoff;
	cc->rts_onoff = sc->sc_rts_onoff;
	cc->break_onoff = sc->sc_break_onoff;
	cc->last_lcr = sc->sc_last_lcr;
	cc->rate = sc->sc_rate;
	cc->v_stop = sc->sc_flow_v_stop;
	cc->v_start = sc->sc_flow_v_start;
	cc->flow = sc->sc_flow;

	return;
}

static void
uftdi_cfg_do_request(struct uftdi_softc *sc, usb_device_request_t *req, 
		     void *data)
{
	u_int16_t length;
	usbd_status err;

	if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto error;
	}

	err = usbd_do_request_flags_mtx(sc->sc_udev, &Giant, req, 
					data, 0, NULL, 1000);

	if (err) {

	    DPRINTF(sc, 0, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));

	error:
	    length = UGETW(req->wLength);

	    if ((req->bmRequestType & UT_READ) && length) {
		bzero(data, length);
	    }
	}
	return;
}

static int
uftdi_open(struct ucom_softc *ucom)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	sc->sc_flag |= (UFTDI_FLAG_WRITE_STALL|
			UFTDI_FLAG_READ_STALL);

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &uftdi_cfg_open, 0);

	return 0;
}

static void
uftdi_cfg_open(struct uftdi_softc *sc,
	       struct uftdi_config_copy *cc, u_int16_t refcount)
{
	u_int16_t wIndex = sc->sc_ucom.sc_portno;
	usb_device_request_t req;

	if (cc == NULL) {

	    /* set 9600 baud, 2 stop bits, 
	     * no parity, 8 bits
	     */
	    (void) uftdi_set_parm_soft
	      (sc, 9600, 0, 0, CSTOPB | CS8, 0);

	    return;
	}

	DPRINTF(sc, 0, "");

	/* perform a full reset on the device */

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_RESET;
	USETW(req.wValue, FTDI_SIO_RESET_SIO);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	/* write parameters */

	uftdi_cfg_parm(sc, cc, 0);

	/* turn on RTS/CTS flow control */

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_FLOW_CTRL;
	USETW(req.wValue, 0);
	USETW2(req.wIndex, FTDI_SIO_RTS_CTS_HS, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	return;
}

static void
uftdi_write_callback(struct usbd_xfer *xfer)
{
	struct uftdi_softc *sc = xfer->priv_sc;
	u_int32_t actlen;

	USBD_CHECK_STATUS(xfer);

tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UFTDI_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}
	return;

tr_setup:
tr_transferred:
	if (sc->sc_flag & UFTDI_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    return;
	}

	if(ucom_get_data(&(sc->sc_ucom), ((u_int8_t *)(xfer->buffer)) + 
			 sc->sc_hdrlen, UFTDI_OBUFSIZE - sc->sc_hdrlen, 
			 &actlen)) {

	    if (sc->sc_hdrlen > 0) {
	        *(u_int8_t *)(xfer->buffer) =
		    FTDI_OUT_TAG(actlen, sc->sc_ucom.sc_portno);
	    }

	    xfer->length = actlen + sc->sc_hdrlen;

	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uftdi_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uftdi_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[0]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[0]);
	sc->sc_flag &= ~UFTDI_FLAG_WRITE_STALL;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;

 tr_error:
	sc->sc_flag &= ~UFTDI_FLAG_WRITE_STALL;
	DPRINTF(sc, 0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
uftdi_read_callback(struct usbd_xfer *xfer)
{
	struct uftdi_softc *sc = xfer->priv_sc;
	u_int8_t *ptr = xfer->buffer;
	u_int8_t msr;
	u_int8_t lsr;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UFTDI_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;

 tr_transferred:

	if (xfer->actlen < 2) {
	    goto tr_setup;
	}

	msr = FTDI_GET_MSR(ptr);
	lsr = FTDI_GET_LSR(ptr);

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
	ptr += 2;

	if (xfer->actlen) {
	    ucom_put_data(&(sc->sc_ucom), ptr, xfer->actlen);
	}

 tr_setup:
	if (sc->sc_flag & UFTDI_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uftdi_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uftdi_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[1]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[1]);
	sc->sc_flag &= ~UFTDI_FLAG_READ_STALL;
	usbd_transfer_start(sc->sc_xfer[1]);
	return;

 tr_error:
	sc->sc_flag &= ~UFTDI_FLAG_READ_STALL;
	DPRINTF(sc, 0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
uftdi_set_dtr(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	sc->sc_dtr_onoff = onoff;

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &uftdi_cfg_set_dtr, 0);

	return;
}

static void
uftdi_cfg_set_dtr(struct uftdi_softc *sc,
		  struct uftdi_config_copy *cc, u_int16_t refcount)
{
	u_int16_t wIndex = sc->sc_ucom.sc_portno;
	u_int16_t wValue;
	usb_device_request_t req;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	wValue = cc->dtr_onoff ? FTDI_SIO_SET_DTR_HIGH : FTDI_SIO_SET_DTR_LOW;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_MODEM_CTRL;
	USETW(req.wValue, wValue);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	return;
}

static void
uftdi_set_rts(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	sc->sc_rts_onoff = onoff;

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &uftdi_cfg_set_rts, 0);

	return;
}

static void
uftdi_cfg_set_rts(struct uftdi_softc *sc,
		  struct uftdi_config_copy *cc, u_int16_t refcount)
{
	u_int16_t wIndex = sc->sc_ucom.sc_portno;
	u_int16_t wValue;
	usb_device_request_t req;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	wValue = cc->rts_onoff ? FTDI_SIO_SET_RTS_HIGH : FTDI_SIO_SET_RTS_LOW;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_MODEM_CTRL;
	USETW(req.wValue, wValue);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	return;
}

static void
uftdi_set_break(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	sc->sc_break_onoff = onoff;

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &uftdi_cfg_set_break, 0);

	return;
}

static void
uftdi_cfg_set_break(struct uftdi_softc *sc,
		    struct uftdi_config_copy *cc, u_int16_t refcount)
{
	u_int16_t wIndex = sc->sc_ucom.sc_portno;
	u_int16_t wValue;
	usb_device_request_t req;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	if (cc->break_onoff) {
		wValue = cc->last_lcr | FTDI_SIO_SET_BREAK;
	} else {
		wValue = cc->last_lcr;
	}

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_DATA;
	USETW(req.wValue, wValue);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	return;
}

static int
uftdi_set_parm_soft(struct uftdi_softc *sc, u_int32_t ospeed, 
		    u_int8_t v_start, u_int8_t v_stop, 
		    u_int32_t cflag, u_int32_t iflag)
{
	u_int16_t rate = 0;
	u_int16_t data = 0;
	u_int8_t flow = 0;

	switch (sc->sc_type) {
	case UFTDI_TYPE_SIO:
		switch (ospeed) {
		case 300: rate = ftdi_sio_b300; break;
		case 600: rate = ftdi_sio_b600; break;
		case 1200: rate = ftdi_sio_b1200; break;
		case 2400: rate = ftdi_sio_b2400; break;
		case 4800: rate = ftdi_sio_b4800; break;
		case 9600: rate = ftdi_sio_b9600; break;
		case 19200: rate = ftdi_sio_b19200; break;
		case 38400: rate = ftdi_sio_b38400; break;
		case 57600: rate = ftdi_sio_b57600; break;
		case 115200: rate = ftdi_sio_b115200; break;
		default:
		    return (EINVAL);
		}
		break;

	case UFTDI_TYPE_8U232AM:
		switch(ospeed) {
		case 300: rate = ftdi_8u232am_b300; break;
		case 600: rate = ftdi_8u232am_b600; break;
		case 1200: rate = ftdi_8u232am_b1200; break;
		case 2400: rate = ftdi_8u232am_b2400; break;
		case 4800: rate = ftdi_8u232am_b4800; break;
		case 9600: rate = ftdi_8u232am_b9600; break;
		case 19200: rate = ftdi_8u232am_b19200; break;
		case 38400: rate = ftdi_8u232am_b38400; break;
		case 57600: rate = ftdi_8u232am_b57600; break;
		case 115200: rate = ftdi_8u232am_b115200; break;
		case 230400: rate = ftdi_8u232am_b230400; break;
		case 460800: rate = ftdi_8u232am_b460800; break;
		case 921600: rate = ftdi_8u232am_b921600; break;
		case 2000000: rate = ftdi_8u232am_b2000000; break;
		case 3000000: rate = ftdi_8u232am_b3000000; break;
		default:
		    return (EINVAL);
		}
		break;
	}
	sc->sc_rate = rate;

	if (cflag & CSTOPB)
	    data = FTDI_SIO_SET_DATA_STOP_BITS_2;
	else
	    data = FTDI_SIO_SET_DATA_STOP_BITS_1;

	if (cflag & PARENB) {
	    if (cflag & PARODD) {
	        data |= FTDI_SIO_SET_DATA_PARITY_ODD;
	    } else {
	        data |= FTDI_SIO_SET_DATA_PARITY_EVEN;
	    }
	} else {
	    data |= FTDI_SIO_SET_DATA_PARITY_NONE;
	}

	switch (cflag & CSIZE) {
	case CS5:
	    data |= FTDI_SIO_SET_DATA_BITS(5);
	    break;

	case CS6:
	    data |= FTDI_SIO_SET_DATA_BITS(6);
	    break;

	case CS7:
	    data |= FTDI_SIO_SET_DATA_BITS(7);
	    break;

	case CS8:
	    data |= FTDI_SIO_SET_DATA_BITS(8);
	    break;
	}
	sc->sc_last_lcr = data;

	if (cflag & CRTSCTS) {
	    flow = FTDI_SIO_RTS_CTS_HS;
	    v_start = 0;
	    v_stop = 0;
	} else if (iflag & (IXON|IXOFF)) {
	    flow = FTDI_SIO_XON_XOFF_HS;
	} else {
	    flow = FTDI_SIO_DISABLE_FLOW_CTRL;
	    v_start = 0;
	    v_stop = 0;
	}

	sc->sc_flow = flow;
	sc->sc_flow_v_start = v_start;
	sc->sc_flow_v_stop = v_stop;
	return 0;
}

static int
uftdi_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	DPRINTF(sc, 0, "\n");

	if (uftdi_set_parm_soft(sc, t->c_ospeed,
				t->c_cc[VSTART], t->c_cc[VSTOP],
				t->c_cflag, t->c_iflag)) {
	    return EIO;
	}

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &uftdi_cfg_parm, 0);

	return 0;
}

static void
uftdi_cfg_parm(struct uftdi_softc *sc,
	       struct uftdi_config_copy *cc, u_int16_t refcount)
{
	u_int16_t wIndex = sc->sc_ucom.sc_portno;
	usb_device_request_t req;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}
			       
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_BAUD_RATE;
	USETW(req.wValue, cc->rate);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_DATA;
	USETW(req.wValue, cc->last_lcr);
	USETW(req.wIndex, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_FLOW_CTRL;
	USETW2(req.wValue, cc->v_stop, cc->v_start);
	USETW2(req.wIndex, cc->flow, wIndex);
	USETW(req.wLength, 0);
	uftdi_cfg_do_request(sc, &req, NULL);

	return;
}

static void
uftdi_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr)
{
	struct uftdi_softc *sc = ucom->sc_parent;

	DPRINTF(sc, 0, "msr=0x%02x lsr=0x%02x\n",
		sc->sc_msr, sc->sc_lsr);

	if (msr) {
		*msr = sc->sc_msr;
	}
	if (lsr) {
		*lsr = sc->sc_lsr;
	}
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
