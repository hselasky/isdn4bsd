/*-
 * Copyright (c) 2004 Bernd Walter <ticso@freebsd.org>
 *
 * $URL: https://devel.bwct.de/svn/projects/ubser/ubser.c $
 * $Date: 2004-02-29 01:53:10 +0100 (Sun, 29 Feb 2004) $
 * $Author: ticso $
 * $Rev: 1127 $
 */

/*-
 * Copyright (c) 2001-2002, Shunsuke Akiyama <akiyama@jp.FreeBSD.org>.
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
__FBSDID("$FreeBSD: src/sys/dev/usb/ubser.c,v 1.20 2006/09/07 00:06:41 imp Exp $");

/*
 * BWCT serial adapter driver
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>
#include <sys/taskqueue.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

#define UBSER_UNIT_MAX	32

/* Vendor Interface Requests */
#define VENDOR_GET_NUMSER		0x01
#define VENDOR_SET_BREAK		0x02
#define VENDOR_CLEAR_BREAK		0x03

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)	\
  do { if (ubser_debug > (n)) {	    \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ubser_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, ubser, CTLFLAG_RW, 0, "USB ubser");
SYSCTL_INT(_hw_usb_ubser, OID_AUTO, debug, CTLFLAG_RW,
	   &ubser_debug, 0, "ubser debug level");
#else
#define DPRINTF(...)
#endif

#define UBSER_TR_DT_WRITE 0
#define UBSER_TR_DT_READ  1
#define UBSER_TR_CS_WRITE 2
#define UBSER_TR_CS_READ  3
#define UBSER_TR_DT_BREAK 4
#define UBSER_TR_MAX      5

struct ubser_softc {
	struct ucom_softc	sc_ucom[UBSER_UNIT_MAX];

	struct usbd_xfer	*sc_xfer[UBSER_TR_MAX];

	uint16_t		sc_tx_size;

	uint8_t			sc_numser;
	uint8_t			sc_send_break[(UBSER_UNIT_MAX+7)/8];
	uint8_t			sc_flags;
#define UBSER_FLAG_READ_STALL  0x01
#define UBSER_FLAG_WRITE_STALL 0x02

	uint8_t			sc_iface_no;
	uint8_t			sc_iface_index;
	uint8_t			sc_curr_tx_unit;
	uint8_t			sc_curr_break_unit;
	uint8_t			sc_name[16];
};

static device_probe_t ubser_probe;
static device_attach_t ubser_attach;
static device_detach_t ubser_detach;

static int   ubser_param(struct ucom_softc *ucom, struct termios *t);
static void  ubser_write_clear_stall_callback(struct usbd_xfer *xfer);
static void  ubser_write_callback(struct usbd_xfer *xfer);
static void  ubser_read_clear_stall_callback(struct usbd_xfer *xfer);
static void  ubser_read_callback(struct usbd_xfer *xfer);
static void  ubser_set_break(struct ucom_softc *ucom, u_int8_t onoff);
static void  ubser_send_break_callback(struct usbd_xfer *xfer);
static int   ubser_open(struct ucom_softc *ucom);
static void  ubser_start_write(struct ucom_softc *ucom);

static const struct usbd_config ubser_config[UBSER_TR_MAX] = {

    [UBSER_TR_DT_WRITE] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = 0, /* use wMaxPacketSize */
      .flags     = USBD_FORCE_SHORT_XFER,
      .callback  = &ubser_write_callback,
    },

    [UBSER_TR_DT_READ] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = 0, /* use wMaxPacketSize */
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &ubser_read_callback,
    },

    [UBSER_TR_CS_WRITE] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &ubser_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [UBSER_TR_CS_READ] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &ubser_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [UBSER_TR_DT_BREAK] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = 0,
      .callback  = &ubser_send_break_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static const struct ucom_callback ubser_callback = {
  .ucom_open        = &ubser_open,
  .ucom_set_break   = &ubser_set_break,
  .ucom_param       = &ubser_param,
  .ucom_start_write = &ubser_start_write,
};

static device_method_t ubser_methods[] = {
    DEVMETHOD(device_probe, ubser_probe),
    DEVMETHOD(device_attach, ubser_attach),
    DEVMETHOD(device_detach, ubser_detach),
    { 0, 0 }
};

static devclass_t ubser_devclass;

static driver_t ubser_driver = {
    .name    = "ubser",
    .methods = ubser_methods,
    .size    = sizeof(struct ubser_softc),
};

DRIVER_MODULE(ubser, uhub, ubser_driver, ubser_devclass, usbd_driver_load, 0);
MODULE_DEPEND(ubser, usb, 1, 1, 1);
MODULE_DEPEND(ubser, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static int
ubser_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_interface_descriptor_t *id;
	usb_device_descriptor_t *dd;
	char buf[6];
	usbd_status err;

	if (uaa->iface == NULL)
		return (UMATCH_NONE);

	dd = usbd_get_device_descriptor(uaa->device);
	if (dd == NULL) {
		return (UMATCH_NONE);
	}

	id = usbd_get_interface_descriptor(uaa->iface);
	if (id == NULL) {
		return (UMATCH_NONE);
	}

	err = usbreq_get_string_any(uaa->device, dd->iManufacturer, 
				    buf, sizeof(buf));
	if (err != 0)
		return (UMATCH_NONE);

	/* check if this is a BWCT vendor specific ubser interface */
	if ((strcmp(buf, "BWCT") == 0) &&
	    (id->bInterfaceClass == 0xff) && 
	    (id->bInterfaceSubClass == 0x00))
		return (UMATCH_VENDOR_IFACESUBCLASS);

	return (UMATCH_NONE);
}

static int
ubser_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ubser_softc *sc = device_get_softc(dev);
	usb_interface_descriptor_t *id;
	usb_device_request_t req;
	uint8_t n;
	int error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	usbd_set_desc(dev, uaa->device);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s",
		 device_get_nameunit(dev));

	id = usbd_get_interface_descriptor(uaa->iface);

	sc->sc_iface_no = id->bInterfaceNumber;
	sc->sc_iface_index = uaa->iface_index;

	/* get number of serials */
	req.bmRequestType = UT_READ_VENDOR_INTERFACE;
	req.bRequest = VENDOR_GET_NUMSER;
	USETW(req.wValue, 0);
	USETW(req.wIndex, sc->sc_iface_no);
	USETW(req.wLength, 1);
	error = usbd_do_request_flags(uaa->device, &req, &sc->sc_numser,
				    0, NULL, USBD_DEFAULT_TIMEOUT);
	if (error || (sc->sc_numser == 0)) {
		device_printf(dev, "failed to get number "
			      "of serial ports: %s\n",
			      usbd_errstr(error));
		goto detach;
	}

	if (sc->sc_numser > UBSER_UNIT_MAX)
		sc->sc_numser = UBSER_UNIT_MAX;

	device_printf(dev, "found %i serials\n", sc->sc_numser);

	error = usbd_transfer_setup(uaa->device, sc->sc_iface_index,
				    sc->sc_xfer, ubser_config, 
				    UBSER_TR_MAX, sc, &Giant);
	if (error) {
	    goto detach;
	}

	sc->sc_tx_size = sc->sc_xfer[UBSER_TR_DT_WRITE]->length;

	if (sc->sc_tx_size == 0) {
	    DPRINTF(sc, -1, "invalid tx_size!\n");
	    goto detach;
	}

	/* initialize port numbers */

	for (n = 0; n < sc->sc_numser; n++) {
	    sc->sc_ucom[n].sc_portno = n;
	}

	error = ucom_attach(sc->sc_ucom, sc->sc_numser, sc,
			    &ubser_callback, &Giant);
	if (error) {
	    goto detach;
	}

	mtx_lock(&Giant);

	sc->sc_flags |= (UBSER_FLAG_READ_STALL|
			 UBSER_FLAG_WRITE_STALL);

	usbd_transfer_start(sc->sc_xfer[UBSER_TR_DT_READ]);

	mtx_unlock(&Giant);

	return 0; /* success */

 detach:
	ubser_detach(dev);
	return ENXIO; /* failure */
}

static int
ubser_detach(device_t dev)
{
	struct ubser_softc *sc = device_get_softc(dev);
	uint8_t n;

	DPRINTF(sc, 0, "\n");

	ucom_detach(sc->sc_ucom, sc->sc_numser);

	/* need to stop all transfers atomically,
	 * hence when clear stall completes, it
	 * might start other transfers !
	 */
	mtx_lock(&Giant);
	for (n = 0; n < UBSER_TR_MAX; n++) {
	    if (sc->sc_xfer[n]) {
	        usbd_transfer_stop(sc->sc_xfer[n]);
	    }
	}
	mtx_unlock(&Giant);

	usbd_transfer_unsetup(sc->sc_xfer, UBSER_TR_MAX);

	return 0;
}

static int
ubser_param(struct ucom_softc *ucom, struct termios *t)
{
	struct ubser_softc *sc = ucom->sc_parent;

	DPRINTF(sc, 0, "\n");

	/*
	 * The firmware on our devices can only do 8n1@9600bps
	 * without handshake.
	 * We refuse to accept other configurations.
	 */

	/* enshure 9600bps */
	switch (t->c_ospeed) {
	case 9600:
		break;
	default:
		return (EINVAL);
	}

	/* 2 stop bits not possible */
	if (t->c_cflag & CSTOPB)
		return (EINVAL);

	/* XXX parity handling not possible with current firmware */
	if (t->c_cflag & PARENB)
		return (EINVAL);

	/* we can only do 8 data bits */
	switch (t->c_cflag & CSIZE) {
	case CS8:
		break;
	default:
		return (EINVAL);
	}

	/* we can't do any kind of hardware handshaking */
	if ((t->c_cflag &
	    (CRTS_IFLOW | CDTR_IFLOW |CDSR_OFLOW |CCAR_OFLOW)) != 0)
		return (EINVAL);

	/*
	 * XXX xon/xoff not supported by the firmware!
	 * This is handled within FreeBSD only and may overflow buffers
	 * because of delayed reaction due to device buffering.
	 */

	return (0);
}

static __inline void
ubser_inc_tx_unit(struct ubser_softc *sc)
{
	sc->sc_curr_tx_unit ++;
	if (sc->sc_curr_tx_unit >= sc->sc_numser) {
	    sc->sc_curr_tx_unit = 0;
	}
	return;
}

static void
ubser_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubser_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[UBSER_TR_DT_WRITE];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);
	sc->sc_flags &= ~UBSER_FLAG_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	sc->sc_flags &= ~UBSER_FLAG_WRITE_STALL;
	DPRINTF(sc, -1, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
ubser_write_callback(struct usbd_xfer *xfer)
{
	struct ubser_softc *sc = xfer->priv_sc;
	uint8_t *buf = xfer->buffer;
	uint8_t first_unit = sc->sc_curr_tx_unit;
	uint32_t actlen;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flags |= UBSER_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[UBSER_TR_CS_WRITE]);
	}
	return;

 tr_setup:
 tr_transferred:
	if (sc->sc_flags & UBSER_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[UBSER_TR_CS_WRITE]);
	    return;
	}

	do {
	    if (ucom_get_data(sc->sc_ucom + sc->sc_curr_tx_unit,
			      buf + 1, sc->sc_tx_size - 1, &actlen)) {

	        buf[0] = sc->sc_curr_tx_unit;

		xfer->length = actlen + 1;

		usbd_start_hardware(xfer);

		ubser_inc_tx_unit(sc); /* round robin */

		break;
	    }

	    ubser_inc_tx_unit(sc);
			    
	} while (sc->sc_curr_tx_unit != first_unit);

	return;
}

static void
ubser_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubser_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[UBSER_TR_DT_READ];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);
	sc->sc_flags &= ~UBSER_FLAG_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	sc->sc_flags &= ~UBSER_FLAG_READ_STALL;
	DPRINTF(sc, -1, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
ubser_read_callback(struct usbd_xfer *xfer)
{
	struct ubser_softc *sc = xfer->priv_sc;
	uint8_t *buf = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flags |= UBSER_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[UBSER_TR_CS_READ]);
	}
	return;

 tr_transferred:
	if (xfer->actlen < 1) {
	    DPRINTF(sc, 0, "invalid actlen=0!\n");
	    goto tr_setup;
	}

	if (buf[0] >= sc->sc_numser) {
	    DPRINTF(sc, 0, "invalid serial number!\n");
	    goto tr_setup;
	}

	ucom_put_data(sc->sc_ucom + buf[0], 
		      buf + 1, xfer->actlen -1);

 tr_setup:
	if (sc->sc_flags & UBSER_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[UBSER_TR_CS_READ]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
ubser_set_break(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct ubser_softc *sc = ucom->sc_parent;
	uint8_t x = ucom->sc_portno;

	if (onoff) {
	    sc->sc_send_break[x/8] |= (1<<(x%8));
	    usbd_transfer_start(sc->sc_xfer[UBSER_TR_DT_BREAK]);
	}
	return;
}

static __inline void
ubser_inc_break_unit(struct ubser_softc *sc)
{
	sc->sc_curr_break_unit ++;
	if (sc->sc_curr_break_unit >= sc->sc_numser) {
	    sc->sc_curr_break_unit = 0;
	}
	return;
}

static void
ubser_send_break_callback(struct usbd_xfer *xfer)
{
	struct ubser_softc *sc = xfer->priv_sc;
	usb_device_request_t *req = xfer->buffer;
	uint8_t first_unit = sc->sc_curr_break_unit;
	uint8_t x;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, -1, "transfer error: %s\n",
		usbd_errstr(xfer->error));
 tr_setup:
 tr_transferred:

	do {
	    x = sc->sc_curr_break_unit;

	    if (sc->sc_send_break[x/8] & (1<<(x%8))) {
	        sc->sc_send_break[x/8] &= ~(1<<(x%8));

		req->bmRequestType = UT_READ_VENDOR_INTERFACE;
		req->bRequest = VENDOR_SET_BREAK;
		USETW(req->wValue, x);
		USETW(req->wIndex, sc->sc_iface_no);
		USETW(req->wLength, 0);

		usbd_start_hardware(xfer);

		ubser_inc_break_unit(sc); /* round robin */

		break;
	    }

	    ubser_inc_break_unit(sc);

	} while (sc->sc_curr_break_unit != first_unit);
	return;
}

static int
ubser_open(struct ucom_softc *ucom)
{
	/* fake status bits */
	ucom->sc_mcr |= (SER_DTR | SER_RTS);
	ucom->sc_msr |= (SER_DCD);
	return 0;
}

static void
ubser_start_write(struct ucom_softc *ucom)
{
	struct ubser_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer[UBSER_TR_DT_WRITE]);
	return;
}

