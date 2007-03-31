/*	$NetBSD: usb/uvscom.c,v 1.1 2002/03/19 15:08:42 augustss Exp $	*/

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uvscom.c,v 1.29 2006/09/07 00:06:42 imp Exp $");

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
 *
 */

/*
 * uvscom: SUNTAC Slipper U VS-10U driver.
 * Slipper U is a PC Card to USB converter for data communication card
 * adapter.  It supports DDI Pocket's Air H" C@rd, C@rd H" 64, NTT's P-in,
 * P-in m@ater and various data communication card adapters.
 */

#include "opt_uvscom.h" /* XXX remove this */

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
#define DPRINTF(n,fmt,...)						\
  do { if (uvscom_debug > (n)) {					\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uvscom_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uvscom, CTLFLAG_RW, 0, "USB uvscom");
SYSCTL_INT(_hw_usb_uvscom, OID_AUTO, debug, CTLFLAG_RW,
	   &uvscom_debug, 0, "uvscom debug level");
#else
#define DPRINTF(...) { }
#endif

#define UVSCOM_MODVER		1	/* module version */

#define	UVSCOM_CONFIG_INDEX	0
#define	UVSCOM_IFACE_INDEX	0

/* Request */
#define UVSCOM_SET_SPEED	0x10
#define UVSCOM_LINE_CTL		0x11
#define UVSCOM_SET_PARAM	0x12
#define UVSCOM_READ_STATUS	0xd0
#define UVSCOM_SHUTDOWN		0xe0

/* UVSCOM_SET_SPEED parameters */
#define UVSCOM_SPEED_150BPS	0x00
#define UVSCOM_SPEED_300BPS	0x01
#define UVSCOM_SPEED_600BPS	0x02
#define UVSCOM_SPEED_1200BPS	0x03
#define UVSCOM_SPEED_2400BPS	0x04
#define UVSCOM_SPEED_4800BPS	0x05
#define UVSCOM_SPEED_9600BPS	0x06
#define UVSCOM_SPEED_19200BPS	0x07
#define UVSCOM_SPEED_38400BPS	0x08
#define UVSCOM_SPEED_57600BPS	0x09
#define UVSCOM_SPEED_115200BPS	0x0a

/* UVSCOM_LINE_CTL parameters */
#define UVSCOM_BREAK		0x40
#define UVSCOM_RTS		0x02
#define UVSCOM_DTR		0x01
#define UVSCOM_LINE_INIT	0x08

/* UVSCOM_SET_PARAM parameters */
#define UVSCOM_DATA_MASK	0x03
#define UVSCOM_DATA_BIT_8	0x03
#define UVSCOM_DATA_BIT_7	0x02
#define UVSCOM_DATA_BIT_6	0x01
#define UVSCOM_DATA_BIT_5	0x00

#define UVSCOM_STOP_MASK	0x04
#define UVSCOM_STOP_BIT_2	0x04
#define UVSCOM_STOP_BIT_1	0x00

#define UVSCOM_PARITY_MASK	0x18
#define UVSCOM_PARITY_EVEN	0x18
#define UVSCOM_PARITY_ODD	0x08
#define UVSCOM_PARITY_NONE	0x00

/* Status bits */
#define UVSCOM_TXRDY		0x04
#define UVSCOM_RXRDY		0x01

#define UVSCOM_DCD		0x08
#define UVSCOM_NOCARD		0x04
#define UVSCOM_DSR		0x02
#define UVSCOM_CTS		0x01
#define UVSCOM_USTAT_MASK	(UVSCOM_NOCARD | UVSCOM_DSR | UVSCOM_CTS)

#define UVSCOM_IBUFSIZE		512 /* bytes */
#define UVSCOM_OBUFSIZE		512 /* bytes */

#define UVSCOM_N_TRANSFER	6 /* units */

struct	uvscom_softc {
	struct ucom_super_softc	sc_super_ucom;
	struct ucom_softc	sc_ucom;

	struct usbd_xfer *	sc_xfer[UVSCOM_N_TRANSFER];
	struct usbd_device	*sc_udev;

	u_int16_t		sc_line; /* line control register */

	u_int8_t		sc_flag;
#define UVSCOM_FLAG_WRITE_STALL    0x0001
#define UVSCOM_FLAG_READ_STALL     0x0002
#define UVSCOM_FLAG_INTR_STALL     0x0004
	u_int8_t		sc_iface_no; /* interface number */
	u_int8_t		sc_iface_index; /* interface index */
	u_int8_t		sc_lsr;	/* local status register */
	u_int8_t		sc_msr; /* uvscom status register */
	u_int8_t		sc_unit_status; /* unit status */
};

/* prototypes */

static device_probe_t uvscom_probe;
static device_attach_t uvscom_attach;
static device_detach_t uvscom_detach;

static usbd_callback_t uvscom_write_callback;
static usbd_callback_t uvscom_write_clear_stall_callback;
static usbd_callback_t uvscom_read_callback;
static usbd_callback_t uvscom_read_clear_stall_callback;
static usbd_callback_t uvscom_intr_callback;
static usbd_callback_t uvscom_intr_clear_stall_callback;

static void	uvscom_cfg_set_dtr(struct ucom_softc *ucom, u_int8_t onoff);
static void	uvscom_cfg_set_rts(struct ucom_softc *ucom, u_int8_t onoff);
static void	uvscom_cfg_set_break(struct ucom_softc *ucom, u_int8_t onoff);
static int	uvscom_pre_param(struct ucom_softc *ucom, struct termios *t);
static void	uvscom_cfg_param(struct ucom_softc *ucom, struct termios *t);
static int 	uvscom_pre_open(struct ucom_softc *ucom);
static void	uvscom_cfg_open(struct ucom_softc *ucom);
static void	uvscom_cfg_close(struct ucom_softc *ucom);
static void	uvscom_start_read(struct ucom_softc *ucom);
static void	uvscom_stop_read(struct ucom_softc *ucom);
static void	uvscom_start_write(struct ucom_softc *ucom);
static void	uvscom_stop_write(struct ucom_softc *ucom);
static void	uvscom_cfg_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr);
static int	uvscom_ioctl(struct ucom_softc *ucom, uint32_t cmd, caddr_t data, int fflag, struct thread *td);
static void	uvscom_cfg_write(struct uvscom_softc *sc, uint8_t index, uint16_t value);
static uint16_t	uvscom_cfg_read_status(struct uvscom_softc *sc);

static const struct usbd_config uvscom_config[UVSCOM_N_TRANSFER] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = UVSCOM_OBUFSIZE,
      .flags     = 0,
      .callback  = &uvscom_write_callback,
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = UVSCOM_IBUFSIZE,
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uvscom_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uvscom_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uvscom_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [4] = {
      .type      = UE_INTERRUPT,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .flags     = USBD_SHORT_XFER_OK,
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &uvscom_intr_callback,
    },

    [5] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uvscom_intr_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static const struct ucom_callback uvscom_callback = {
    .ucom_cfg_get_status  = &uvscom_cfg_get_status,
    .ucom_cfg_set_dtr     = &uvscom_cfg_set_dtr,
    .ucom_cfg_set_rts     = &uvscom_cfg_set_rts,
    .ucom_cfg_set_break   = &uvscom_cfg_set_break,
    .ucom_cfg_param       = &uvscom_cfg_param,
    .ucom_cfg_open        = &uvscom_cfg_open,
    .ucom_cfg_close       = &uvscom_cfg_close,
    .ucom_pre_open        = &uvscom_pre_open,
    .ucom_pre_param       = &uvscom_pre_param,
    .ucom_ioctl           = &uvscom_ioctl,
    .ucom_start_read      = &uvscom_start_read,
    .ucom_stop_read       = &uvscom_stop_read,
    .ucom_start_write     = &uvscom_start_write,
    .ucom_stop_write      = &uvscom_stop_write,
};

static const struct usb_devno uvscom_devs [] = {
    /* SUNTAC U-Cable type A4 */
    { USB_VENDOR_SUNTAC, USB_PRODUCT_SUNTAC_AS144L4 },
    /* SUNTAC U-Cable type D2 */
    { USB_VENDOR_SUNTAC, USB_PRODUCT_SUNTAC_DS96L },
    /* SUNTAC Ir-Trinity */
    { USB_VENDOR_SUNTAC, USB_PRODUCT_SUNTAC_IS96U },
    /* SUNTAC U-Cable type P1 */
    { USB_VENDOR_SUNTAC, USB_PRODUCT_SUNTAC_PS64P1 },
    /* SUNTAC Slipper U */
    { USB_VENDOR_SUNTAC, USB_PRODUCT_SUNTAC_VS10U },
};

static device_method_t uvscom_methods[] = {
    DEVMETHOD(device_probe, uvscom_probe),
    DEVMETHOD(device_attach, uvscom_attach),
    DEVMETHOD(device_detach, uvscom_detach),
    { 0, 0 }
};

static devclass_t uvscom_devclass;

static driver_t uvscom_driver = {
    .name    = "uvscom",
    .methods = uvscom_methods,
    .size    = sizeof (struct uvscom_softc),
};

DRIVER_MODULE(uvscom, uhub, uvscom_driver, uvscom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uvscom, usb, 1, 1, 1);
MODULE_DEPEND(uvscom, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);
MODULE_VERSION(uvscom, UVSCOM_MODVER);

static int
uvscom_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->iface) {
	    return UMATCH_NONE;
	}

	return (usb_lookup(uvscom_devs, uaa->vendor, uaa->product) ?
		UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

static int
uvscom_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uvscom_softc *sc = device_get_softc(dev);
	usb_interface_descriptor_t *id;
	struct usbd_interface *iface;
	int error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	usbd_set_desc(dev, uaa->device);

	sc->sc_udev = uaa->device;

	DPRINTF(0, "sc=%p\n", sc);

	/* configure the device */

	error = usbd_set_config_index(uaa->device, UVSCOM_CONFIG_INDEX, 1);
	if (error) {
	    device_printf(dev, "failed to set configuration, "
			  "error=%s\n", usbd_errstr(error));
	    goto detach;
	}

	iface = usbd_get_iface(uaa->device, UVSCOM_IFACE_INDEX);

	if (iface == NULL) {
	    DPRINTF(0, "no interface\n");
	    goto detach;
	}

	id = usbd_get_interface_descriptor(iface);

	if (id == NULL) {
	    DPRINTF(0, "no interface descriptor\n");
	    goto detach;
	}

	sc->sc_iface_no = id->bInterfaceNumber;
	sc->sc_iface_index = UVSCOM_IFACE_INDEX;

	error = usbd_transfer_setup(uaa->device, sc->sc_iface_index,
				    sc->sc_xfer, uvscom_config,
				    UVSCOM_N_TRANSFER,
				    sc, &Giant);
	if (error) {
	    DPRINTF(0, "could not allocate all USB transfers!\n");
	    goto detach;
	}

	sc->sc_line = UVSCOM_LINE_INIT;

	/* clear stall at first run */
	sc->sc_flag |= (UVSCOM_FLAG_WRITE_STALL|
			UVSCOM_FLAG_READ_STALL);

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
			    &uvscom_callback, &Giant);
	if (error) {
	    goto detach;
	}

	/* start interrupt pipe */

	usbd_transfer_start(sc->sc_xfer[4]);

	return 0;

 detach:
	uvscom_detach(dev);
	return ENXIO;
}

static int
uvscom_detach(device_t dev)
{
	struct uvscom_softc *sc = device_get_softc(dev);

	DPRINTF(0, "sc=%p\n", sc);

	/* stop interrupt pipe */

	if (sc->sc_xfer[4]) {
	    usbd_transfer_stop(sc->sc_xfer[4]);
	}

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UVSCOM_N_TRANSFER);

	return 0;
}

static void
uvscom_write_callback(struct usbd_xfer *xfer)
{
	struct uvscom_softc *sc = xfer->priv_sc;
	u_int32_t actlen;

	USBD_CHECK_STATUS(xfer);

tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UVSCOM_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}
	return;

tr_setup:
tr_transferred:
	if (sc->sc_flag & UVSCOM_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    return;
	}

	if(ucom_get_data(&(sc->sc_ucom), xfer->buffer, UVSCOM_OBUFSIZE, &actlen)) {

	    xfer->length = actlen;

	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uvscom_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uvscom_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[0]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[0]);
	sc->sc_flag &= ~UVSCOM_FLAG_WRITE_STALL;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;

 tr_error:
	sc->sc_flag &= ~UVSCOM_FLAG_WRITE_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
uvscom_read_callback(struct usbd_xfer *xfer)
{
	struct uvscom_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UVSCOM_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;

 tr_transferred:
	ucom_put_data(&(sc->sc_ucom), xfer->buffer, xfer->actlen);

 tr_setup:
	if (sc->sc_flag & UVSCOM_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uvscom_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uvscom_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[1]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[1]);
	sc->sc_flag &= ~UVSCOM_FLAG_READ_STALL;
	usbd_transfer_start(sc->sc_xfer[1]);
	return;

 tr_error:
	sc->sc_flag &= ~UVSCOM_FLAG_READ_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
uvscom_intr_callback(struct usbd_xfer *xfer)
{
	struct uvscom_softc *sc = xfer->priv_sc;
	u_int8_t *buf = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UVSCOM_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[5]);
	}
	return;

 tr_transferred:
	if (xfer->actlen >= 2) {
	    sc->sc_lsr = 0;
	    sc->sc_msr = 0;
	    sc->sc_unit_status = buf[1];

	    if (buf[0] & UVSCOM_TXRDY) {
	        sc->sc_lsr |= ULSR_TXRDY;
	    }
	    if (buf[0] & UVSCOM_RXRDY) {
	        sc->sc_lsr |= ULSR_RXRDY;
	    }
	    if (buf[1] & UVSCOM_CTS) {
	        sc->sc_msr |= SER_CTS;
	    }
	    if (buf[1] & UVSCOM_DSR) {
		sc->sc_msr |= SER_DSR;
	    }
	    if (buf[1] & UVSCOM_DCD) {
		sc->sc_msr |= SER_DCD;
	    }

	    /* the UCOM layer will ignore this
	     * call if the TTY device is closed!
	     */
	    ucom_status_change(&(sc->sc_ucom));
	}

 tr_setup:
	if (sc->sc_flag & UVSCOM_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[5]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uvscom_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uvscom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);
	sc->sc_flag &= ~UVSCOM_FLAG_INTR_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	sc->sc_flag &= ~UVSCOM_FLAG_INTR_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
uvscom_cfg_set_dtr(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (onoff)
	    sc->sc_line |= UVSCOM_DTR;
	else
	    sc->sc_line &= ~UVSCOM_DTR;

	uvscom_cfg_write(sc, UVSCOM_LINE_CTL, sc->sc_line);
	return;
}

static void
uvscom_cfg_set_rts(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (onoff)
	    sc->sc_line |= UVSCOM_RTS;
	else
	    sc->sc_line &= ~UVSCOM_RTS;

	uvscom_cfg_write(sc, UVSCOM_LINE_CTL, sc->sc_line);
	return;
}

static void
uvscom_cfg_set_break(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (onoff)
	    sc->sc_line |= UVSCOM_BREAK;
	else
	    sc->sc_line &= ~UVSCOM_BREAK;

	uvscom_cfg_write(sc, UVSCOM_LINE_CTL, sc->sc_line);
	return;
}

static int
uvscom_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	switch (t->c_ospeed) {
	case B150:
	case B300:
	case B600:
	case B1200:
	case B2400:
	case B4800:
	case B9600:
	case B19200:
	case B38400:
	case B57600:
	case B115200:
	default:
	    return EINVAL;
	}
	return 0;
}

static void
uvscom_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uvscom_softc *sc = ucom->sc_parent;
	uint16_t value;

	DPRINTF(0, "\n");

	switch (t->c_ospeed) {
	case B150:
		value = UVSCOM_SPEED_150BPS;
		break;
	case B300:
		value = UVSCOM_SPEED_300BPS;
		break;
	case B600:
		value = UVSCOM_SPEED_600BPS;
		break;
	case B1200:
		value = UVSCOM_SPEED_1200BPS;
		break;
	case B2400:
		value = UVSCOM_SPEED_2400BPS;
		break;
	case B4800:
		value = UVSCOM_SPEED_4800BPS;
		break;
	case B9600:
		value = UVSCOM_SPEED_9600BPS;
		break;
	case B19200:
		value = UVSCOM_SPEED_19200BPS;
		break;
	case B38400:
		value = UVSCOM_SPEED_38400BPS;
		break;
	case B57600:
		value = UVSCOM_SPEED_57600BPS;
		break;
	case B115200:
		value = UVSCOM_SPEED_115200BPS;
		break;
	default:
		return;
	}

	uvscom_cfg_write(sc, UVSCOM_SET_SPEED, value);

	value = 0;

	if (t->c_cflag & CSTOPB) {
	    value |= UVSCOM_STOP_BIT_2;
	}

	if (t->c_cflag & PARENB) {
	  if (t->c_cflag & PARODD) {
	    value |= UVSCOM_PARITY_ODD;
	  } else {
	    value |= UVSCOM_PARITY_EVEN;
	  }
	} else {
	  value |= UVSCOM_PARITY_NONE;
	}

	switch (t->c_cflag & CSIZE) {
	case CS5:
	    value |= UVSCOM_DATA_BIT_5;
	    break;
	case CS6:
	    value |= UVSCOM_DATA_BIT_6;
	    break;
	case CS7:
	    value |= UVSCOM_DATA_BIT_7;
	    break;
	default:
	case CS8:
	    value |= UVSCOM_DATA_BIT_8;
	    break;
	}

	uvscom_cfg_write(sc, UVSCOM_SET_PARAM, value);
	return;
}

static int
uvscom_pre_open(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "sc = %p\n", sc);

	/* check if PC card was inserted */

	if (sc->sc_unit_status & UVSCOM_NOCARD) {
	    DPRINTF(0, "no PC card!\n");
	    return ENXIO;
	}
	return 0;
}

static void
uvscom_cfg_open(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "sc = %p\n", sc);

	uvscom_cfg_read_status(sc);

	return;
}

static void
uvscom_cfg_close(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "sc=%p\n", sc);

	uvscom_cfg_write(sc, UVSCOM_SHUTDOWN, 0);

	return;
}

static void
uvscom_start_read(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
uvscom_stop_read(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
uvscom_start_write(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
uvscom_stop_write(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

static void
uvscom_cfg_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	*lsr = sc->sc_lsr;
	*msr = sc->sc_msr;
	return;
}

static int
uvscom_ioctl(struct ucom_softc *ucom, uint32_t cmd, caddr_t data, int fflag,
	     struct thread *td)
{
	return ENOTTY;
}

static void
uvscom_cfg_write(struct uvscom_softc *sc, uint8_t index, uint16_t value)
{
	usb_device_request_t req;
	usbd_status err;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
	    return;
	}

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = index;
	USETW(req.wValue, value);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request_flags_mtx(sc->sc_udev, &Giant, &req, 
					NULL, 0, NULL, 1000);
	if (err) {
	    DPRINTF(-1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));
	}
	return;
}

static uint16_t
uvscom_cfg_read_status(struct uvscom_softc *sc)
{
	usb_device_request_t req;
	usbd_status err;
	uint8_t data[2];

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
	    return 0;
	}

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = UVSCOM_READ_STATUS;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 2);

	err = usbd_do_request_flags_mtx(sc->sc_udev, &Giant, &req, 
					data, 0, NULL, 1000);
	if (err) {
	    DPRINTF(-1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));
	    data[0] = 0;
	    data[1] = 0;
	}
	return (data[0] | (data[1] << 8));
}
