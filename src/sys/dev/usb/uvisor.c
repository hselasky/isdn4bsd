/*	$NetBSD: uvisor.c,v 1.9 2001/01/23 14:04:14 augustss Exp $	*/
/*      $FreeBSD: src/sys/dev/usb/uvisor.c,v 1.25 2005/12/04 07:34:19 bmah Exp $	*/

/* Also already merged from NetBSD:
 *	$NetBSD: uvisor.c,v 1.12 2001/11/13 06:24:57 lukem Exp $
 *	$NetBSD: uvisor.c,v 1.13 2002/02/11 15:11:49 augustss Exp $
 *	$NetBSD: uvisor.c,v 1.14 2002/02/27 23:00:03 augustss Exp $
 *	$NetBSD: uvisor.c,v 1.15 2002/06/16 15:01:31 augustss Exp $
 *	$NetBSD: uvisor.c,v 1.16 2002/07/11 21:14:36 augustss Exp $
 *	$NetBSD: uvisor.c,v 1.17 2002/08/13 11:38:15 augustss Exp $
 *	$NetBSD: uvisor.c,v 1.18 2003/02/05 00:50:14 augustss Exp $
 *	$NetBSD: uvisor.c,v 1.19 2003/02/07 18:12:37 augustss Exp $
 *	$NetBSD: uvisor.c,v 1.20 2003/04/11 01:30:10 simonb Exp $
 */

/*-
 * Copyright (c) 2000 The NetBSD Foundation, Inc.
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
 * Handspring Visor (Palmpilot compatible PDA) driver
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>
#include <sys/taskqueue.h>

#include <dev/usb2/usb_port.h>
#include <dev/usb2/usb.h>
#include <dev/usb2/usb_subr.h>
#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

#ifdef USB_DEBUG
#define DPRINTF(n,fmt,...)	\
  do { if (uvisor_debug > (n)) {	\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uvisor_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uvisor, CTLFLAG_RW, 0, "USB uvisor");
SYSCTL_INT(_hw_usb_uvisor, OID_AUTO, debug, CTLFLAG_RW,
	   &uvisor_debug, 0, "uvisor debug level");
#else
#define	DPRINTF(...)
#endif

#define UVISOR_CONFIG_INDEX	0
#define UVISOR_IFACE_INDEX	0
#define UVISOR_MODVER		1
#define UVISOR_N_TRANSFER       5 /* units */
#define UVISOR_BUFSIZE       1024 /* bytes */

/* From the Linux driver */
/*
 * UVISOR_REQUEST_BYTES_AVAILABLE asks the visor for the number of bytes that
 * are available to be transfered to the host for the specified endpoint.
 * Currently this is not used, and always returns 0x0001
 */
#define UVISOR_REQUEST_BYTES_AVAILABLE		0x01

/*
 * UVISOR_CLOSE_NOTIFICATION is set to the device to notify it that the host
 * is now closing the pipe. An empty packet is sent in response.
 */
#define UVISOR_CLOSE_NOTIFICATION		0x02

/*
 * UVISOR_GET_CONNECTION_INFORMATION is sent by the host during enumeration to
 * get the endpoints used by the connection.
 */
#define UVISOR_GET_CONNECTION_INFORMATION	0x03

/*
 * UVISOR_GET_CONNECTION_INFORMATION returns data in the following format
 */
#define UVISOR_MAX_CONN 8
struct uvisor_connection_info {
	uWord	num_ports;
	struct {
		uByte	port_function_id;
		uByte	port;
	} UPACKED connections[UVISOR_MAX_CONN];
} UPACKED;
#define UVISOR_CONNECTION_INFO_SIZE 18

/* struct uvisor_connection_info.connection[x].port defines: */
#define UVISOR_ENDPOINT_1		0x01
#define UVISOR_ENDPOINT_2		0x02

/* struct uvisor_connection_info.connection[x].port_function_id defines: */
#define UVISOR_FUNCTION_GENERIC		0x00
#define UVISOR_FUNCTION_DEBUGGER	0x01
#define UVISOR_FUNCTION_HOTSYNC		0x02
#define UVISOR_FUNCTION_CONSOLE		0x03
#define UVISOR_FUNCTION_REMOTE_FILE_SYS	0x04

/*
 * Unknown PalmOS stuff.
 */
#define UVISOR_GET_PALM_INFORMATION		0x04
#define UVISOR_GET_PALM_INFORMATION_LEN		0x44

struct uvisor_palm_connection_info {
        uByte   num_ports;
        uByte   endpoint_numbers_different;
	uWord   reserved1;
	struct {
                uDWord  port_function_id;
                uByte   port;
                uByte   end_point_info;
                uWord   reserved;
	} UPACKED connections[UVISOR_MAX_CONN];
} UPACKED;

struct uvisor_softc {
	struct ucom_softc	sc_ucom;

	struct usbd_xfer *	sc_xfer[UVISOR_N_TRANSFER];

	u_int16_t		sc_flag;
#define UVISOR_FLAG_PALM4       0x0001
#define UVISOR_FLAG_VISOR       0x0002
#define UVISOR_FLAG_PALM35      0x0004
#define UVISOR_FLAG_SEND_NOTIFY 0x0008
#define UVISOR_FLAG_WRITE_STALL 0x0010
#define UVISOR_FLAG_READ_STALL  0x0020

	u_int8_t		sc_iface_no;
	u_int8_t		sc_iface_index;
};

struct uvisor_product {
	u_int16_t vendor;
	u_int16_t product;
	u_int16_t flags;
};

static const struct uvisor_product *
uvisor_find_up(struct usb_attach_arg *uaa);

static device_probe_t uvisor_probe;
static device_attach_t uvisor_attach;
static device_detach_t uvisor_detach;

usbd_status
uvisor_init(struct uvisor_softc *sc, struct usbd_device *udev, struct usbd_config *config);

static int
uvisor_open(struct ucom_softc *ucom);

static void
uvisor_close(struct ucom_softc *ucom);

static void
uvisor_start_read(struct ucom_softc *ucom);

static void
uvisor_stop_read(struct ucom_softc *ucom);

static void
uvisor_start_write(struct ucom_softc *ucom);

static void
uvisor_stop_write(struct ucom_softc *ucom);

static void
uvisor_write_callback(struct usbd_xfer *xfer);

static void
uvisor_write_clear_stall_callback(struct usbd_xfer *xfer);

static void
uvisor_read_callback(struct usbd_xfer *xfer);

static void
uvisor_read_clear_stall_callback(struct usbd_xfer *xfer);

static void
usbd_close_notify_callback(struct usbd_xfer *xfer);

static const struct usbd_config uvisor_config[UVISOR_N_TRANSFER] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = UVISOR_BUFSIZE, /* bytes */
      .flags     = 0,
      .callback  = &uvisor_write_callback,
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = UVISOR_BUFSIZE, /* bytes */
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uvisor_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uvisor_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uvisor_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [4] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = (sizeof(usb_device_request_t) + sizeof(struct uvisor_connection_info)),
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &usbd_close_notify_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static const struct ucom_callback uvisor_callback = {
    .ucom_open        = &uvisor_open,
    .ucom_close       = &uvisor_close,
    .ucom_start_read  = &uvisor_start_read,
    .ucom_stop_read   = &uvisor_stop_read,
    .ucom_start_write = &uvisor_start_write,
    .ucom_stop_write  = &uvisor_stop_write,
};

static device_method_t uvisor_methods[] = {
    DEVMETHOD(device_probe, uvisor_probe),
    DEVMETHOD(device_attach, uvisor_attach),
    DEVMETHOD(device_detach, uvisor_detach),
    { 0, 0 }
};

static driver_t uvisor_driver = {
    .name    = "ucom",
    .methods = uvisor_methods,
    .size    = sizeof (struct uvisor_softc),
};

DRIVER_MODULE(uvisor, uhub, uvisor_driver, ucom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uvisor, usb, 1, 1, 1);
MODULE_DEPEND(uvisor, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);
MODULE_VERSION(uvisor, UVISOR_MODVER);

static const struct uvisor_product uvisor_products[] = {
    { USB_VENDOR_HANDSPRING, USB_PRODUCT_HANDSPRING_VISOR, UVISOR_FLAG_VISOR },
    { USB_VENDOR_HANDSPRING, USB_PRODUCT_HANDSPRING_TREO, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_HANDSPRING, USB_PRODUCT_HANDSPRING_TREO600, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_M500, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_M505, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_M515, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_I705, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_M125, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_M130, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_TUNGSTEN_Z, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_TUNGSTEN_T, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_ZIRE, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_PALM, USB_PRODUCT_PALM_ZIRE31, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_SONY, USB_PRODUCT_SONY_CLIE_40, 0 },
    { USB_VENDOR_SONY, USB_PRODUCT_SONY_CLIE_41, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_SONY, USB_PRODUCT_SONY_CLIE_S360, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_SONY, USB_PRODUCT_SONY_CLIE_NX60, UVISOR_FLAG_PALM4 },
    { USB_VENDOR_SONY, USB_PRODUCT_SONY_CLIE_35, UVISOR_FLAG_PALM35 },
/*  { USB_VENDOR_SONY, USB_PRODUCT_SONY_CLIE_25, UVISOR_FLAG_PALM4 }, */
    { USB_VENDOR_SONY, USB_PRODUCT_SONY_CLIE_TJ37, UVISOR_FLAG_PALM4 },
    { 0, 0, 0 }
};

static const struct uvisor_product *
uvisor_find_up(struct usb_attach_arg *uaa)
{
	const struct uvisor_product *up = uvisor_products;

	if (uaa->iface == NULL) {
	    while (up->vendor) {
	        if ((up->vendor == uaa->vendor) &&
		    (up->product == uaa->product)) {
		    return up;
		}
		up++;
	    }
	}
	return NULL;
}

static int
uvisor_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	const struct uvisor_product *up = uvisor_find_up(uaa);
	return (up ? UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

static int
uvisor_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uvisor_softc *sc = device_get_softc(dev);
	const struct uvisor_product *up = uvisor_find_up(uaa);
	struct usbd_interface *iface;
	struct usbd_config uvisor_config_copy[UVISOR_N_TRANSFER];
	usb_interface_descriptor_t *id;
	int error;

	DPRINTF(0, "sc=%p\n", sc);

	bcopy(uvisor_config, uvisor_config_copy, 
	      sizeof(uvisor_config_copy));

	if (sc == NULL) {
	    return ENOMEM;
	}

	usbd_set_desc(dev, uaa->device);

	/* configure the device */

	error = usbd_set_config_index(uaa->device, UVISOR_CONFIG_INDEX, 1);

	if (error) {
	    DPRINTF(0, "failed to set configuration, error=%s\n",
		    usbd_errstr(error));
	    goto detach;
	}

	iface = usbd_get_iface(uaa->device, UVISOR_IFACE_INDEX);

	if (iface == NULL) {
	    DPRINTF(0, "no interface\n");
	    goto detach;
	}

	id = usbd_get_interface_descriptor(iface);

	if (id == NULL) {
	    DPRINTF(0, "no interface descriptor\n");
	    goto detach;
	}

	sc->sc_flag = up->flags;
	sc->sc_iface_no = id->bInterfaceNumber;
	sc->sc_iface_index = UVISOR_IFACE_INDEX;

	error = uvisor_init(sc, uaa->device, uvisor_config_copy);

	if (error) {
	    DPRINTF(0, "init failed, error=%s\n",
		    usbd_errstr(error));
	    goto detach;
	}

	error = usbd_transfer_setup(uaa->device, sc->sc_iface_index,
				    sc->sc_xfer, uvisor_config_copy, UVISOR_N_TRANSFER,
				    sc, &Giant);
	if (error) {
	    DPRINTF(0, "could not allocate all pipes\n");
	    goto detach;
	}

	sc->sc_ucom.sc_parent = sc;
	sc->sc_ucom.sc_portno = 0;
	sc->sc_ucom.sc_callback = &uvisor_callback;

	error = ucom_attach(&(sc->sc_ucom), dev);

	if (error) {
	    DPRINTF(0, "ucom_attach failed\n");
	    goto detach;
	}

	return 0;

 detach:
	uvisor_detach(dev);
	return ENXIO;
}

static int
uvisor_detach(device_t dev)
{
	struct uvisor_softc *sc = device_get_softc(dev);

	DPRINTF(0, "sc=%p\n", sc);

	ucom_detach(&(sc->sc_ucom));

	usbd_transfer_unsetup(sc->sc_xfer, UVISOR_N_TRANSFER);

	return 0;
}

usbd_status
uvisor_init(struct uvisor_softc *sc, struct usbd_device *udev, struct usbd_config *config)
{
	usbd_status err = 0;
	usb_device_request_t req;
	struct uvisor_connection_info coninfo;
	struct uvisor_palm_connection_info pconinfo;
	int actlen;
	uWord wAvail;
	u_int8_t buffer[256];

	if (sc->sc_flag & UVISOR_FLAG_VISOR) {
	    DPRINTF(0, "getting connection info\n");
	    req.bmRequestType = UT_READ_VENDOR_ENDPOINT;
	    req.bRequest = UVISOR_GET_CONNECTION_INFORMATION;
	    USETW(req.wValue, 0);
	    USETW(req.wIndex, 0);
	    USETW(req.wLength, UVISOR_CONNECTION_INFO_SIZE);
	    err = usbd_do_request_flags(udev, &req, &coninfo,
					USBD_SHORT_XFER_OK, &actlen,
					USBD_DEFAULT_TIMEOUT);
	    if (err) {
	        goto done;
	    }
	}
#ifdef USB_DEBUG
	if (sc->sc_flag & UVISOR_FLAG_VISOR) {
	    u_int16_t i, np;
	    const char *desc;

	    np = UGETW(coninfo.num_ports);
	    if (np > UVISOR_MAX_CONN) {
	        np = UVISOR_MAX_CONN;
	    }

	    DPRINTF(0, "Number of ports: %d\n", np);

	    for (i = 0; i < np; ++i) {
	        switch (coninfo.connections[i].port_function_id) {
		case UVISOR_FUNCTION_GENERIC:
		    desc = "Generic";
		    break;
		case UVISOR_FUNCTION_DEBUGGER:
		    desc = "Debugger";
		    break;
		case UVISOR_FUNCTION_HOTSYNC:
		    desc = "HotSync";
		    break;
		case UVISOR_FUNCTION_REMOTE_FILE_SYS:
		    desc = "Remote File System";
		    break;
		default:
		    desc = "unknown";
		    break;
		}
		DPRINTF(0, "Port %d is for %s\n",
			coninfo.connections[i].port, desc);
	    }
	}
#endif

	if (sc->sc_flag & UVISOR_FLAG_PALM4) {
	    u_int8_t port;

	    /* Palm OS 4.0 Hack */
	    req.bmRequestType = UT_READ_VENDOR_ENDPOINT;
	    req.bRequest = UVISOR_GET_PALM_INFORMATION;
	    USETW(req.wValue, 0);
	    USETW(req.wIndex, 0);
	    USETW(req.wLength, UVISOR_GET_PALM_INFORMATION_LEN);

	    err = usbd_do_request_flags(udev, &req, &pconinfo,
					USBD_SHORT_XFER_OK, &actlen,
					USBD_DEFAULT_TIMEOUT);
	    if (err) {
	        goto done;
	    }

	    if (actlen < 12) {
	        DPRINTF(0, "too little data\n");
		err = USBD_INVAL;
		goto done;
	    }

	    if (pconinfo.endpoint_numbers_different) {
	        port = pconinfo.connections[0].end_point_info;
		config[0].endpoint = (port & 0xF); /* output */
		config[1].endpoint = (port >> 4); /* input */
	    } else {
	        port = pconinfo.connections[0].port;
		config[0].endpoint = (port & 0xF); /* output */
		config[1].endpoint = (port & 0xF); /* input */
	    }
#if 0
	    req.bmRequestType = UT_READ_VENDOR_ENDPOINT;
	    req.bRequest = UVISOR_GET_PALM_INFORMATION;
	    USETW(req.wValue, 0);
	    USETW(req.wIndex, 0);
	    USETW(req.wLength, UVISOR_GET_PALM_INFORMATION_LEN);
	    err = usbd_do_request(udev, &req, buffer);
	    if (err) {
	        goto done;
	    }
#endif
	}

	if (sc->sc_flag & UVISOR_FLAG_PALM35) {
	    /* get the config number */
	    DPRINTF(0, "getting config info\n");
	    req.bmRequestType = UT_READ;
	    req.bRequest = UR_GET_CONFIG;
	    USETW(req.wValue, 0);
	    USETW(req.wIndex, 0);
	    USETW(req.wLength, 1);

	    err = usbd_do_request(udev, &req, buffer);
	    if (err) {
	        goto done;
	    }
  
	    /* get the interface number */
	    DPRINTF(0, "get the interface number\n");
	    req.bmRequestType = UT_READ_DEVICE;
	    req.bRequest = UR_GET_INTERFACE;
	    USETW(req.wValue, 0);
	    USETW(req.wIndex, 0);
	    USETW(req.wLength, 1);
	    err = usbd_do_request(udev, &req, buffer);
	    if (err) {
	        goto done;
	    }
	}

	DPRINTF(0, "getting available bytes\n");
	req.bmRequestType = UT_READ_VENDOR_ENDPOINT;
	req.bRequest = UVISOR_REQUEST_BYTES_AVAILABLE;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 5);
	USETW(req.wLength, sizeof(wAvail));
	err = usbd_do_request(udev, &req, &wAvail);
	if (err) {
	    goto done;
	}

	DPRINTF(0, "avail=%d\n", UGETW(wAvail));

	DPRINTF(0, "done\n");
 done:
	return err;
}

static int
uvisor_open(struct ucom_softc *ucom)
{
	struct uvisor_softc *sc = ucom->sc_parent;

	/* clear stall first */
	sc->sc_flag |= (UVISOR_FLAG_WRITE_STALL|
			UVISOR_FLAG_READ_STALL);
	return 0;
}

static void
uvisor_close(struct ucom_softc *ucom)
{
	struct uvisor_softc *sc = ucom->sc_parent;
	sc->sc_flag |= UVISOR_FLAG_SEND_NOTIFY;
	usbd_transfer_start(sc->sc_xfer[4]);
	return;
}

static void
uvisor_start_read(struct ucom_softc *ucom)
{
	struct uvisor_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
uvisor_stop_read(struct ucom_softc *ucom)
{
	struct uvisor_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
uvisor_start_write(struct ucom_softc *ucom)
{
	struct uvisor_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
uvisor_stop_write(struct ucom_softc *ucom)
{
	struct uvisor_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

static void
uvisor_write_callback(struct usbd_xfer *xfer)
{
	struct uvisor_softc *sc = xfer->priv_sc;
	u_int32_t actlen;

	USBD_CHECK_STATUS(xfer);
tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UVISOR_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}
	return;

tr_setup:
tr_transferred:
	if (sc->sc_flag & UVISOR_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    return;
	}

	if(ucom_get_data(&(sc->sc_ucom), xfer->buffer, UVISOR_BUFSIZE, &actlen)) {

	    xfer->length = actlen;

	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uvisor_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uvisor_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[0]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[0]);
	sc->sc_flag &= ~UVISOR_FLAG_WRITE_STALL;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;

 tr_error:
	sc->sc_flag &= ~UVISOR_FLAG_WRITE_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
uvisor_read_callback(struct usbd_xfer *xfer)
{
	struct uvisor_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UVISOR_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;

 tr_transferred:
	ucom_put_data(&(sc->sc_ucom), xfer->buffer, xfer->actlen);

 tr_setup:
	if (sc->sc_flag & UVISOR_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
uvisor_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uvisor_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[1]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[1]);
	sc->sc_flag &= ~UVISOR_FLAG_READ_STALL;
	usbd_transfer_start(sc->sc_xfer[1]);
	return;

 tr_error:
	sc->sc_flag &= ~UVISOR_FLAG_READ_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
usbd_close_notify_callback(struct usbd_xfer *xfer)
{
	usb_device_request_t *req = xfer->buffer;
	struct uvisor_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(0, "error=%s\n", usbd_errstr(xfer->error));
 tr_transferred:
 tr_setup:
	if (sc->sc_flag &   UVISOR_FLAG_SEND_NOTIFY) {
	    sc->sc_flag &= ~UVISOR_FLAG_SEND_NOTIFY;

	    req->bmRequestType = UT_READ_VENDOR_ENDPOINT; /* XXX read? */
	    req->bRequest = UVISOR_CLOSE_NOTIFICATION;
	    USETW(req->wValue, 0);
	    USETW(req->wIndex, 0);
	    USETW(req->wLength, UVISOR_CONNECTION_INFO_SIZE);
	    usbd_start_hardware(xfer);
	}
	return;
}
