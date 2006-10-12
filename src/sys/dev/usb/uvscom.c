/*	$NetBSD: usb/uvscom.c,v 1.1 2002/03/19 15:08:42 augustss Exp $	*/
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

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>
#include <sys/taskqueue.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>
#include <dev/usb/usb_cdc.h>

#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

__FBSDID("$FreeBSD: src/sys/dev/usb/uvscom.c $");

#ifdef USB_DEBUG
#define DPRINTF(n,fmt,...)						\
  do { if (uvscom_debug > (n)) {					\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uvscom_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uvscom, CTLFLAG_RW, 0, "USB uvscom");
SYSCTL_INT(_hw_usb_uvscom, OID_AUTO, debug, CTLFLAG_RW,
	   &uvscom_debug, 0, "uvscom debug level");
#else
#define DPRINTF(...)
#endif

#define UVSCOM_MODVER		1	/* module version */

#define	UVSCOM_CONFIG_INDEX	0
#define	UVSCOM_IFACE_INDEX	0

#ifndef UVSCOM_INTR_INTERVAL
#define UVSCOM_INTR_INTERVAL	0 /* default */
#endif

#define UVSCOM_UNIT_WAIT	5

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
#if 0
#define UVSCOM_PARITY_UNK	0x10
#endif
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

/*
 * These are the maximum number of bytes transferred per frame.
 * The output buffer size cannot be increased due to the size encoding.
 */
#define UVSCOM_IBUFSIZE		512 /* bytes */
#define UVSCOM_OBUFSIZE		 64 /* bytes */

#ifndef UVSCOM_DEFAULT_OPKTSIZE
#define UVSCOM_DEFAULT_OPKTSIZE	8
#endif

#define UVSCOM_N_TRANSFER 10

struct	uvscom_softc {
	struct ucom_softc	sc_ucom;
	struct __callout	sc_watchdog;

	struct usbd_xfer *	sc_xfer[UVSCOM_N_TRANSFER];

	u_int16_t		sc_line_ctrl; /* line control register */
	u_int16_t		sc_line_speed; /* line speed */
	u_int16_t		sc_line_param; /* line parameters */
	u_int16_t		sc_flag;
#define UVSCOM_FLAG_WAIT_USB       0x0001
#define UVSCOM_FLAG_WRITE_STALL    0x0002
#define UVSCOM_FLAG_READ_STALL     0x0004
#define UVSCOM_FLAG_INTR_STALL     0x0008
#define UVSCOM_FLAG_OPEN           0x0010
#define UVSCOM_FLAG_SET_LINE       0x0020
#define UVSCOM_FLAG_SET_LINE_SPEED 0x0040
#define UVSCOM_FLAG_SET_LINE_PARM  0x0080

	u_int8_t		sc_iface_no; /* interface number */
	u_int8_t		sc_iface_index; /* interface index */
	u_int8_t		sc_dtr; /* current DTR state */
	u_int8_t		sc_rts; /* current RTS state */
	u_int8_t		sc_lsr;	/* local status register */
	u_int8_t		sc_msr; /* uvscom status register */
	u_int8_t		sc_unit_status; /* unit status */
};

static device_probe_t uvscom_probe;
static device_attach_t uvscom_attach;
static device_detach_t uvscom_detach;

static void
uvscom_watchdog(void *arg);

static void
uvscom_write_callback(struct usbd_xfer *xfer);

static void
uvscom_write_clear_stall_callback(struct usbd_xfer *xfer);

static void
uvscom_read_callback(struct usbd_xfer *xfer);

static void
uvscom_read_clear_stall_callback(struct usbd_xfer *xfer);

static void
uvscom_intr_callback(struct usbd_xfer *xfer);

static void
uvscom_intr_clear_stall_callback(struct usbd_xfer *xfer);

static void
uvscom_read_status_callback(struct usbd_xfer *xfer);

static void
uvscom_shutdown_callback(struct usbd_xfer *xfer);

static void
uvscom_set_line_callback(struct usbd_xfer *xfer);

static void
uvscom_set_line_coding_callback(struct usbd_xfer *xfer);

static void
uvscom_set_dtr(struct ucom_softc *ucom, u_int8_t onoff);

static void
uvscom_set_rts(struct ucom_softc *ucom, u_int8_t onoff);

static void
uvscom_set_break(struct ucom_softc *ucom, u_int8_t onoff);

static int
uvscom_param(struct ucom_softc *ucom, struct termios *t);

static int
uvscom_open(struct ucom_softc *ucom);

static void
uvscom_close(struct ucom_softc *ucom);

static void
uvscom_start_read(struct ucom_softc *ucom);

static void
uvscom_stop_read(struct ucom_softc *ucom);

static void
uvscom_start_write(struct ucom_softc *ucom);

static void
uvscom_stop_write(struct ucom_softc *ucom);

static void
uvscom_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr);

static int
uvscom_ioctl(struct ucom_softc *ucom, uint32_t cmd, caddr_t data, int fflag,
	     struct thread *td);

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

    [6] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t) + 2,
      .callback  = &uvscom_read_status_callback,
      .timeout   = 1000, /* 1 second */
    },

    [7] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uvscom_shutdown_callback,
      .timeout   = 1000, /* 1 second */
    },

    [8] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uvscom_set_line_callback,
      .timeout   = 1000, /* 1 second */
    },

    [9] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uvscom_set_line_coding_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static const struct ucom_callback uvscom_callback = {
    .ucom_get_status  = &uvscom_get_status,
    .ucom_set_dtr     = &uvscom_set_dtr,
    .ucom_set_rts     = &uvscom_set_rts,
    .ucom_set_break   = &uvscom_set_break,
    .ucom_param       = &uvscom_param,
    .ucom_ioctl       = &uvscom_ioctl,
    .ucom_open        = &uvscom_open,
    .ucom_close       = &uvscom_close,
    .ucom_start_read  = &uvscom_start_read,
    .ucom_stop_read   = &uvscom_stop_read,
    .ucom_start_write = &uvscom_start_write,
    .ucom_stop_write  = &uvscom_stop_write,
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

	DPRINTF(0, "sc=%p\n", sc);

	__callout_init_mtx(&(sc->sc_watchdog), &Giant,
                           CALLOUT_RETURNUNLOCKED);

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

	sc->sc_flag |= UVSCOM_FLAG_WAIT_USB;

	sc->sc_dtr = -1;
	sc->sc_rts = -1;
	sc->sc_line_ctrl = UVSCOM_LINE_INIT;

	error = ucom_attach(&(sc->sc_ucom), 1, sc,
			    &uvscom_callback, &Giant);
	if (error) {
	    goto detach;
	}

	/* start interrupt pipe */

	usbd_transfer_start(sc->sc_xfer[4]);

	/* start watchdog (returns unlocked) */

	mtx_lock(&Giant);

	uvscom_watchdog(sc);

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

	__callout_stop(&(sc->sc_watchdog));

	/* stop interrupt pipe */

	if (sc->sc_xfer[4]) {
	    usbd_transfer_stop(sc->sc_xfer[4]);
	}

	ucom_detach(&(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UVSCOM_N_TRANSFER);

	__callout_drain(&(sc->sc_watchdog));

	return 0;
}

static void
uvscom_watchdog(void *arg)
{
	struct uvscom_softc *sc = arg;

	mtx_assert(&Giant, MA_OWNED);

	usbd_transfer_start(sc->sc_xfer[6]);

	__callout_reset(&(sc->sc_watchdog), hz, 
			&(uvscom_watchdog), sc);

	mtx_unlock(&Giant);

	return;
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

	    if (sc->sc_flag & UVSCOM_FLAG_OPEN) {
	        ucom_status_change(&(sc->sc_ucom));
	    }
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

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[4]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[4]);
	sc->sc_flag &= ~UVSCOM_FLAG_INTR_STALL;
	usbd_transfer_start(sc->sc_xfer[4]);
	return;

 tr_error:
	sc->sc_flag &= ~UVSCOM_FLAG_INTR_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
uvscom_read_status_callback(struct usbd_xfer *xfer)
{
	usb_device_request_t *req = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(0, "error=%s\n", usbd_errstr(xfer->error));

 tr_transferred:
	return;

 tr_setup:
	req->bmRequestType = UT_READ_VENDOR_DEVICE;
	req->bRequest = UVSCOM_READ_STATUS;
	USETW(req->wValue, 0);
	USETW(req->wIndex, 0);
	USETW(req->wLength, 2);
	usbd_start_hardware(xfer);
	return;
}

static void
uvscom_shutdown_callback(struct usbd_xfer *xfer)
{
	usb_device_request_t *req = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(0, "error=%s\n", usbd_errstr(xfer->error));

 tr_transferred:
	return;

 tr_setup:
	req->bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req->bRequest = UVSCOM_SHUTDOWN;
	USETW(req->wValue, 0);
	USETW(req->wIndex, 0);
	USETW(req->wLength, 0);
	usbd_start_hardware(xfer);
	return;
}

static void
uvscom_set_line_callback(struct usbd_xfer *xfer)
{
	usb_device_request_t *req = xfer->buffer;
	struct uvscom_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(0, "error=%s\n", usbd_errstr(xfer->error));

 tr_transferred:
 tr_setup:
	if (sc->sc_flag &   UVSCOM_FLAG_SET_LINE) {
	    sc->sc_flag &= ~UVSCOM_FLAG_SET_LINE;

	    req->bmRequestType = UT_WRITE_VENDOR_DEVICE;
	    req->bRequest = UVSCOM_LINE_CTL;
	    USETW(req->wValue, sc->sc_line_ctrl);
	    USETW(req->wIndex, 0);
	    USETW(req->wLength, 0);

	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uvscom_set_line_coding_callback(struct usbd_xfer *xfer)
{
	usb_device_request_t *req = xfer->buffer;
	struct uvscom_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(0, "error=%s\n", usbd_errstr(xfer->error));

 tr_transferred:
 tr_setup:
	if (sc->sc_flag &   UVSCOM_FLAG_SET_LINE_SPEED) {
	    sc->sc_flag &= ~UVSCOM_FLAG_SET_LINE_SPEED;

	    req->bmRequestType = UT_WRITE_VENDOR_DEVICE;
	    req->bRequest = UVSCOM_SET_SPEED;
	    USETW(req->wValue, sc->sc_line_speed);
	    USETW(req->wIndex, 0);
	    USETW(req->wLength, 0);
	    usbd_start_hardware(xfer);
	    return;
	}
	if (sc->sc_flag &   UVSCOM_FLAG_SET_LINE_PARM) {
	    sc->sc_flag &= ~UVSCOM_FLAG_SET_LINE_PARM;

	    req->bmRequestType = UT_WRITE_VENDOR_DEVICE;
	    req->bRequest = UVSCOM_SET_PARAM;
	    USETW(req->wValue, sc->sc_line_param);
	    USETW(req->wIndex, 0);
	    USETW(req->wLength, 0);
	    usbd_start_hardware(xfer);
	    return;
	}
	return;
}

static void
uvscom_set_dtr(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (sc->sc_dtr != onoff) {
	    sc->sc_dtr = onoff;

	    if (onoff)
	      sc->sc_line_ctrl |= UVSCOM_DTR;
	    else
	      sc->sc_line_ctrl &= ~UVSCOM_DTR;

	    sc->sc_flag |= UVSCOM_FLAG_SET_LINE;
	    usbd_transfer_start(sc->sc_xfer[8]);
	}
	return;
}

static void
uvscom_set_rts(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (sc->sc_rts != onoff) {
	    sc->sc_rts = onoff;

	    if (onoff)
	      sc->sc_line_ctrl |= UVSCOM_RTS;
	    else
	      sc->sc_line_ctrl &= ~UVSCOM_RTS;

	    sc->sc_flag |= UVSCOM_FLAG_SET_LINE;
	    usbd_transfer_start(sc->sc_xfer[8]);
	}
	return;
}

static void
uvscom_set_break(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (onoff)
	    sc->sc_line_ctrl |= UVSCOM_BREAK;
	else
	    sc->sc_line_ctrl &= ~UVSCOM_BREAK;

	sc->sc_flag |= UVSCOM_FLAG_SET_LINE;
	usbd_transfer_start(sc->sc_xfer[8]);
	return;
}

static int
uvscom_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "\n");

	switch (t->c_ospeed) {
	case B150:
		sc->sc_line_speed = UVSCOM_SPEED_150BPS;
		break;
	case B300:
		sc->sc_line_speed = UVSCOM_SPEED_300BPS;
		break;
	case B600:
		sc->sc_line_speed = UVSCOM_SPEED_600BPS;
		break;
	case B1200:
		sc->sc_line_speed = UVSCOM_SPEED_1200BPS;
		break;
	case B2400:
		sc->sc_line_speed = UVSCOM_SPEED_2400BPS;
		break;
	case B4800:
		sc->sc_line_speed = UVSCOM_SPEED_4800BPS;
		break;
	case B9600:
		sc->sc_line_speed = UVSCOM_SPEED_9600BPS;
		break;
	case B19200:
		sc->sc_line_speed = UVSCOM_SPEED_19200BPS;
		break;
	case B38400:
		sc->sc_line_speed = UVSCOM_SPEED_38400BPS;
		break;
	case B57600:
		sc->sc_line_speed = UVSCOM_SPEED_57600BPS;
		break;
	case B115200:
		sc->sc_line_speed = UVSCOM_SPEED_115200BPS;
		break;
	default:
		return (EIO);
	}

	sc->sc_line_param = 0; /* reset */

	sc->sc_line_param |= ((t->c_cflag & CSTOPB) ?
			     UVSCOM_STOP_BIT_2 : UVSCOM_STOP_BIT_1);

	sc->sc_line_param |= ((t->c_cflag & PARENB) ?
			     ((t->c_cflag & PARODD) ? 
			      UVSCOM_PARITY_ODD : 
			      UVSCOM_PARITY_EVEN) :
			     UVSCOM_PARITY_NONE);

	switch (t->c_cflag & CSIZE) {
	case CS5:
	    sc->sc_line_param |= UVSCOM_DATA_BIT_5;
	    break;
	case CS6:
	    sc->sc_line_param |= UVSCOM_DATA_BIT_6;
	    break;
	case CS7:
	    sc->sc_line_param |= UVSCOM_DATA_BIT_7;
	    break;
	case CS8:
	    sc->sc_line_param |= UVSCOM_DATA_BIT_8;
	    break;
	default:
	    return EIO;
	}

	sc->sc_flag |= (UVSCOM_FLAG_SET_LINE_SPEED|
			UVSCOM_FLAG_SET_LINE_PARM);

	usbd_transfer_start(sc->sc_xfer[9]);

	return 0;
}

static int
uvscom_open(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "sc = %p\n", sc);

	/* clear stall first */
	sc->sc_flag |= (UVSCOM_FLAG_OPEN|
			UVSCOM_FLAG_WRITE_STALL|
			UVSCOM_FLAG_READ_STALL);

	/* check if PC card was inserted */

	if (sc->sc_unit_status & UVSCOM_NOCARD) {
	    DPRINTF(0, "no PC card!\n");
	    return ENXIO;
	}

	return 0;
}

static void
uvscom_close(struct ucom_softc *ucom)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	DPRINTF(0, "sc=%p\n", sc);

	usbd_transfer_start(sc->sc_xfer[7]);

	sc->sc_flag &= ~UVSCOM_FLAG_OPEN;

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
uvscom_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr)
{
	struct uvscom_softc *sc = ucom->sc_parent;

	if (lsr) {
	    *lsr = sc->sc_lsr;
	}
	if (msr) {
	    *msr = sc->sc_msr;
	}
	return;
}

static int
uvscom_ioctl(struct ucom_softc *ucom, uint32_t cmd, caddr_t data, int fflag,
	     struct thread *td)
{
	int error = ENOTTY;

	DPRINTF(0, "cmd = 0x%08x\n", cmd);

	switch (cmd) {
	case TIOCNOTTY:
	case TIOCMGET:
	case TIOCMSET:
	    break;

	default:
	    DPRINTF(0, "unknown\n");
	    error = ENOTTY;
	    break;
	}
	return error;
}
