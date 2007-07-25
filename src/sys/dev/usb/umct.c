/*-
 * Copyright (c) 2003 Scott Long
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
 *
 */

/*
 * Driver for the MCT (Magic Control Technology) USB-RS232 Converter.
 * Based on the superb documentation from the linux mct_u232 driver by
 * Wolfgang Grandeggar <wolfgang@cec.ch>.
 * This device smells a lot like the Belkin F5U103, except that it has
 * suffered some mild brain-damage.  This driver is based off of the ubsa.c
 * driver from Alexander Kabaev <kan@freebsd.org>.  Merging the two together
 * might be useful, though the subtle differences might lead to lots of
 * #ifdef's.
 */

/*
 * NOTE: all function names beginning like "umct_cfg_" can only
 * be called from within the config thread function !
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
#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

__FBSDID("$FreeBSD: src/sys/dev/usb/umct.c,v 1.18 2007/06/18 22:27:57 imp Exp $");

/* The UMCT advertises the standard 8250 UART registers */
#define UMCT_GET_MSR		2	/* Get Modem Status Register */
#define UMCT_GET_MSR_SIZE	1
#define UMCT_GET_LCR		6	/* Get Line Control Register */
#define UMCT_GET_LCR_SIZE	1
#define UMCT_SET_BAUD		5	/* Set the Baud Rate Divisor */
#define UMCT_SET_BAUD_SIZE	4
#define UMCT_SET_LCR		7	/* Set Line Control Register */
#define UMCT_SET_LCR_SIZE	1
#define UMCT_SET_MCR		10	/* Set Modem Control Register */
#define UMCT_SET_MCR_SIZE	1

#define UMCT_INTR_INTERVAL	100
#define UMCT_IFACE_INDEX	0
#define UMCT_CONFIG_INDEX	1

#define UMCT_ENDPT_MAX		6 /* units */

#define DPRINTF(...) do { } while (0)

struct umct_softc {
	struct ucom_super_softc	sc_super_ucom;
	struct ucom_softc	sc_ucom;

	struct usbd_device *	sc_udev;
	struct usbd_xfer *	sc_xfer[UMCT_ENDPT_MAX];

	u_int32_t		sc_unit;

	u_int16_t		sc_obufsize;

	u_int8_t		sc_lsr;
	u_int8_t		sc_msr;
	u_int8_t		sc_lcr;
	u_int8_t		sc_mcr;

	u_int8_t		sc_name[16];
	u_int8_t		sc_flags;
#define UMCT_FLAG_READ_STALL    0x01
#define UMCT_FLAG_WRITE_STALL   0x02
#define UMCT_FLAG_INTR_STALL    0x04
	u_int8_t		sc_iface_no;
};

/* prototypes */

static device_probe_t umct_probe;
static device_attach_t umct_attach;
static device_detach_t umct_detach;

static usbd_callback_t umct_intr_clear_stall_callback;
static usbd_callback_t umct_intr_callback;
static usbd_callback_t umct_write_callback;
static usbd_callback_t umct_write_clear_stall_callback;
static usbd_callback_t umct_read_callback;
static usbd_callback_t umct_read_clear_stall_callback;

static void	umct_cfg_do_request(struct umct_softc *sc, u_int8_t request, u_int16_t len, u_int32_t value);
static void	umct_cfg_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr);
static void	umct_cfg_set_break(struct ucom_softc *ucom, u_int8_t onoff);
static void	umct_cfg_set_dtr(struct ucom_softc *ucom, u_int8_t onoff);
static void	umct_cfg_set_rts(struct ucom_softc *ucom, u_int8_t onoff);
static u_int8_t	umct_calc_baud(u_int32_t baud);
static int	umct_pre_param(struct ucom_softc *ucom, struct termios *ti);
static void	umct_cfg_param(struct ucom_softc *ucom, struct termios *ti);
static void	umct_start_read(struct ucom_softc *ucom);
static void	umct_stop_read(struct ucom_softc *ucom);
static void	umct_start_write(struct ucom_softc *ucom);
static void	umct_stop_write(struct ucom_softc *ucom);

static const struct usbd_config umct_config[UMCT_ENDPT_MAX] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_OUT,
      .bufsize   = 0, /* use wMaxPacketSize */
      .flags     = (USBD_PIPE_BOF),
      .callback  = &umct_write_callback,
    },

    [1] = {
      .type      = UE_INTERRUPT,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_IN,
      .flags     = (USBD_PIPE_BOF|USBD_SHORT_XFER_OK),
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &umct_read_callback,
      .index     = 0, /* first interrupt endpoint */
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = (USBD_USE_DMA),
      .callback  = &umct_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
      .interval  = 50, /* 50ms */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = (USBD_USE_DMA),
      .callback  = &umct_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
      .interval  = 50, /* 50ms */
    },

    [4] = {
      .type      = UE_INTERRUPT,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_IN,
      .flags     = (USBD_PIPE_BOF|USBD_SHORT_XFER_OK),
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &umct_intr_callback,
      .index     = 1, /* second interrupt endpoint */
    },

    [5] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = (USBD_USE_DMA),
      .callback  = &umct_intr_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
      .interval  = 50, /* 50ms */
    },
};

static const struct ucom_callback umct_callback = {
    .ucom_cfg_get_status  = &umct_cfg_get_status,
    .ucom_cfg_set_dtr     = &umct_cfg_set_dtr,
    .ucom_cfg_set_rts     = &umct_cfg_set_rts,
    .ucom_cfg_set_break   = &umct_cfg_set_break,
    .ucom_cfg_param       = &umct_cfg_param,
    .ucom_pre_param       = &umct_pre_param,
    .ucom_start_read      = &umct_start_read,
    .ucom_stop_read       = &umct_stop_read,
    .ucom_start_write     = &umct_start_write,
    .ucom_stop_write      = &umct_stop_write,
};

static const struct umct_product {
	uint16_t	vendor;
	uint16_t	product;
} umct_products[] = {
	{ USB_VENDOR_MCT, USB_PRODUCT_MCT_USB232 },
	{ USB_VENDOR_MCT, USB_PRODUCT_MCT_SITECOM_USB232 },
	{ USB_VENDOR_MCT, USB_PRODUCT_MCT_DU_H3SP_USB232 },
	{ USB_VENDOR_BELKIN, USB_PRODUCT_BELKIN_F5U109 },
	{ USB_VENDOR_BELKIN, USB_PRODUCT_BELKIN_F5U409 },
	{ 0, 0 }
};

static device_method_t umct_methods[] = {
	DEVMETHOD(device_probe, umct_probe),
	DEVMETHOD(device_attach, umct_attach),
	DEVMETHOD(device_detach, umct_detach),
	{ 0, 0 }
};

static devclass_t umct_devclass;

static driver_t umct_driver = {
	.name    = "umct",
	.methods = umct_methods,
	.size    = sizeof(struct umct_softc),
};

DRIVER_MODULE(umct, uhub, umct_driver, umct_devclass, usbd_driver_load, 0);
MODULE_DEPEND(umct, usb, 1, 1, 1);
MODULE_DEPEND(umct, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static int
umct_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	u_int32_t i;

	if (uaa->iface != NULL) {
	    return UMATCH_NONE;
	}

	for (i = 0; umct_products[i].vendor != 0; i++) {
	    if ((umct_products[i].vendor == uaa->vendor) &&
		(umct_products[i].product == uaa->product)) {
	        return UMATCH_VENDOR_PRODUCT;
	    }
	}
	return UMATCH_NONE;
}

static int
umct_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct umct_softc *sc = device_get_softc(dev);
	struct usbd_interface *iface;
	usb_interface_descriptor_t *id;
	int32_t error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	sc->sc_udev = uaa->device;
	sc->sc_unit = device_get_unit(dev);

	usbd_set_desc(dev, uaa->device);

	snprintf(sc->sc_name, sizeof(sc->sc_name),
		 "%s", device_get_nameunit(dev));

	error = usbd_set_config_index(uaa->device, UMCT_CONFIG_INDEX, 1);
	if (error) {
	    device_printf(dev, "failed to set configuration: "
			  "%s\n", usbd_errstr(error));
	    goto detach;
	}

        iface = usbd_get_iface(uaa->device, UMCT_IFACE_INDEX);
	if (iface == NULL) {
	    device_printf(dev, "no interface!\n");
	    goto detach;
	}

        id = usbd_get_interface_descriptor(iface);
	if (id == NULL) {
	    device_printf(dev, "no interface descriptor!\n");
	    goto detach;
	}

        sc->sc_iface_no = id->bInterfaceNumber;

	error = usbd_transfer_setup(uaa->device, UMCT_IFACE_INDEX,
				    sc->sc_xfer, umct_config, UMCT_ENDPT_MAX,
				    sc, &Giant);
	if (error) {
	    device_printf(dev, "allocating USB "
			  "transfers failed!\n");
	    goto detach;
	}

	/*
	 * The real bulk-in endpoint is also marked as an interrupt.
	 * The only way to differentiate it from the real interrupt
	 * endpoint is to look at the wMaxPacketSize field.
	 */
	if (sc->sc_xfer[1]->max_packet_size == 0x2) {

	    /* guessed wrong - switch around endpoints */

	    struct usbd_xfer *temp = sc->sc_xfer[4];
	    sc->sc_xfer[4] = sc->sc_xfer[1];
	    sc->sc_xfer[1] = temp;

	    sc->sc_xfer[1]->callback = &umct_read_callback;
	    sc->sc_xfer[4]->callback = &umct_intr_callback;
	}

	sc->sc_obufsize = sc->sc_xfer[0]->length;

	if (uaa->product == USB_PRODUCT_MCT_SITECOM_USB232) {
	    if (sc->sc_obufsize > 16) {
	        sc->sc_obufsize = 16;
	    }
	}

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
			    &umct_callback, &Giant);
	if (error) {
	    goto detach;
	}

	return 0; /* success */

 detach:
	umct_detach(dev);
	return ENXIO; /* failure */
}

static int
umct_detach(device_t dev)
{
	struct umct_softc *sc = device_get_softc(dev);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UMCT_ENDPT_MAX);

	return 0;
}

static void
umct_cfg_do_request(struct umct_softc *sc, u_int8_t request, 
		    u_int16_t len, u_int32_t value)
{
	usb_device_request_t req;
	usbd_status err;
	u_int8_t temp[4];

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
	    goto done;
	}

	if (len > 4) {
	    len = 4;
	}

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = request;
	USETW(req.wValue, 0);
	req.wIndex[0] = sc->sc_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, len);
	USETDW(temp, value);

	err = usbd_do_request_flags_mtx(sc->sc_udev, &Giant, &req, 
					temp, 0, NULL, 1000);

	if (err) {
	    DPRINTF(sc, -1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));
	}
 done:
	return;
}

static void
umct_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct umct_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
	    DPRINTF(sc, 0, "stall cleared\n");
	    sc->sc_flags &= ~UMCT_FLAG_INTR_STALL;
	    usbd_transfer_start(xfer_other);
	}
	return;
}

static void
umct_intr_callback(struct usbd_xfer *xfer)
{
	struct umct_softc *sc = xfer->priv_sc;
	u_int8_t *ptr = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* start clear stall */
	    sc->sc_flags |= UMCT_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[5]);
	}
	return;

 tr_transferred:
	if (xfer->actlen < 2) {
	    DPRINTF(sc, 0, "too short message\n");
	    goto tr_setup;
	}

	sc->sc_msr = ptr[0];
	sc->sc_lsr = ptr[1];

	ucom_status_change(&(sc->sc_ucom));
    
 tr_setup:
	if (sc->sc_flags & UMCT_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[5]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
umct_cfg_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr)
{
	struct umct_softc *sc = ucom->sc_parent;

	*lsr = sc->sc_lsr;
	*msr = sc->sc_msr;
	return;
}

static void
umct_cfg_set_break(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct umct_softc *sc = ucom->sc_parent;

	if (onoff)
	  sc->sc_lcr |= 0x40;
	else
	  sc->sc_lcr &= ~0x40;

	umct_cfg_do_request(sc, UMCT_SET_LCR, UMCT_SET_LCR_SIZE, sc->sc_lcr);
	return;
}

static void
umct_cfg_set_dtr(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct umct_softc *sc = ucom->sc_parent;

	if (onoff)
	  sc->sc_mcr |= 0x01;
	else
	  sc->sc_mcr &= ~0x01;

	umct_cfg_do_request(sc, UMCT_SET_MCR, UMCT_SET_MCR_SIZE, sc->sc_mcr);
	return;
}

static void
umct_cfg_set_rts(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct umct_softc *sc = ucom->sc_parent;

	if (onoff)
	  sc->sc_mcr |= 0x02;
	else
	  sc->sc_mcr &= ~0x02;

	umct_cfg_do_request(sc, UMCT_SET_MCR, UMCT_SET_MCR_SIZE, sc->sc_mcr);
	return;
}

static u_int8_t
umct_calc_baud(u_int32_t baud)
{
	switch(baud) {
	case B300: return (0x1);
	case B600: return (0x2);
	case B1200: return (0x3);
	case B2400: return (0x4);
	case B4800: return (0x6);
	case B9600: return (0x8);
	case B19200: return (0x9);
	case B38400: return (0xa);
	case B57600: return (0xb);
	case 115200: return (0xc);
	case B0:
	default:
	    break;
	}
	return (0x0);
}

static int
umct_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	return 0; /* we accept anything */
}

static void
umct_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct umct_softc *sc = ucom->sc_parent;
	u_int32_t value;

	value = umct_calc_baud(t->c_ospeed);
	umct_cfg_do_request(sc, UMCT_SET_BAUD, UMCT_SET_BAUD_SIZE, value);

	value = (sc->sc_lcr & 0x40);

	switch (t->c_cflag & CSIZE) {
	case CS5: value |= 0x0; break;
	case CS6: value |= 0x1; break;
	case CS7: value |= 0x2; break;
	default:
	case CS8: value |= 0x3; break;
	}

	value |= (t->c_cflag & CSTOPB) ? 0x4 : 0;
	if (t->c_cflag & PARENB) {
	    value |= 0x8;
	    value |= (t->c_cflag & PARODD) ? 0x0 : 0x10;
	}

	/*
	 * XXX There doesn't seem to be a way to tell the device 
	 * to use flow control.
	 */

	sc->sc_lcr = value;
	umct_cfg_do_request(sc, UMCT_SET_LCR, UMCT_SET_LCR_SIZE, value);
 	return;
}

static void
umct_start_read(struct ucom_softc *ucom)
{
        struct umct_softc *sc = ucom->sc_parent;
	/* start interrupt endpoint */
	usbd_transfer_start(sc->sc_xfer[4]);

	/* start read endpoint */
	usbd_transfer_start(sc->sc_xfer[1]);
        return;
}

static void
umct_stop_read(struct ucom_softc *ucom)
{
        struct umct_softc *sc = ucom->sc_parent;

	/* stop interrupt endpoint */
	usbd_transfer_stop(sc->sc_xfer[5]);
	usbd_transfer_stop(sc->sc_xfer[4]);

	/* stop read endpoint */
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
        return;
}

static void
umct_start_write(struct ucom_softc *ucom)
{
        struct umct_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer[0]);
        return;
}

static void
umct_stop_write(struct ucom_softc *ucom)
{
        struct umct_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
        return;
}

static void
umct_write_callback(struct usbd_xfer *xfer)
{
	struct umct_softc *sc = xfer->priv_sc;
	u_int32_t actlen;

	USBD_CHECK_STATUS(xfer);

tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flags |= UMCT_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}
	return;

tr_setup:
tr_transferred:
	if (sc->sc_flags & UMCT_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    return;
	}

	if(ucom_get_data(&(sc->sc_ucom), xfer->buffer, sc->sc_obufsize,
			 &actlen)) {

	    xfer->length = actlen;
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
umct_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct umct_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
	    DPRINTF(sc, 0, "stall cleared\n");
	    sc->sc_flags &= ~UMCT_FLAG_WRITE_STALL;
	    usbd_transfer_start(xfer_other);
	}
	return;
}

static void
umct_read_callback(struct usbd_xfer *xfer)
{
	struct umct_softc *sc = xfer->priv_sc;
	u_int8_t *ptr = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flags |= UMCT_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;

 tr_transferred:
	if (xfer->actlen) {
	    ucom_put_data(&(sc->sc_ucom), ptr, xfer->actlen);
	}

 tr_setup:
	if (sc->sc_flags & UMCT_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
umct_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct umct_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
	    DPRINTF(sc, 0, "stall cleared\n");
	    sc->sc_flags &= ~UMCT_FLAG_READ_STALL;
	    usbd_transfer_start(xfer_other);
	}
	return;
}
