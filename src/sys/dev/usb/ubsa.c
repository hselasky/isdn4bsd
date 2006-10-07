/*-
 * Copyright (c) 2002, Alexander Kabaev <kan.FreeBSD.org>.
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ubsa.c,v 1.19 2006/09/07 00:06:41 imp Exp $");
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>
#include <sys/taskqueue.h>

#include <dev/usb2/usb_port.h>
#include <dev/usb2/usb.h>
#include <dev/usb2/usb_subr.h>
#include <dev/usb2/usb_quirks.h>
#include <dev/usb2/usb_cdc.h>

#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

#ifdef USB_DEBUG
#define DPRINTF(n,fmt,...)						\
  do { if (ubsa_debug > (n)) {						\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ubsa_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, ubsa, CTLFLAG_RW, 0, "USB ubsa");
SYSCTL_INT(_hw_usb_ubsa, OID_AUTO, debug, CTLFLAG_RW,
	   &ubsa_debug, 0, "ubsa debug level");
#else
#define	DPRINTF(...)
#endif

#define UBSA_MODVER		1 /* module version */

#define UBSA_N_TRANSFER         14 /* units */
#define UBSA_BSIZE              64 /* bytes */

#define UBSA_CONFIG_INDEX	1
#define UBSA_IFACE_INDEX	0

enum {
  UBSA_REG_BAUDRATE,
  UBSA_REG_STOP_BITS,
  UBSA_REG_DATA_BITS,
  UBSA_REG_PARITY,
  UBSA_REG_DTR,
  UBSA_REG_RTS,
  UBSA_REG_BREAK,
  UBSA_REG_FLOW_CTRL,
  USBA_N_REG
};

#define UBSA_PARITY_NONE	0x00
#define UBSA_PARITY_EVEN	0x01
#define UBSA_PARITY_ODD		0x02
#define UBSA_PARITY_MARK	0x03
#define UBSA_PARITY_SPACE	0x04

#define UBSA_FLOW_NONE		0x0000
#define UBSA_FLOW_OCTS		0x0001
#define UBSA_FLOW_ODSR		0x0002
#define UBSA_FLOW_IDSR		0x0004
#define UBSA_FLOW_IDTR		0x0008
#define UBSA_FLOW_IRTS		0x0010
#define UBSA_FLOW_ORTS		0x0020
#define UBSA_FLOW_UNKNOWN	0x0040
#define UBSA_FLOW_OXON		0x0080
#define UBSA_FLOW_IXON		0x0100

/* line status register */
#define UBSA_LSR_TSRE		0x40	/* Transmitter empty: byte sent */
#define UBSA_LSR_TXRDY		0x20	/* Transmitter buffer empty */
#define UBSA_LSR_BI		0x10	/* Break detected */
#define UBSA_LSR_FE		0x08	/* Framing error: bad stop bit */
#define UBSA_LSR_PE		0x04	/* Parity error */
#define UBSA_LSR_OE		0x02	/* Overrun, lost incoming byte */
#define UBSA_LSR_RXRDY		0x01	/* Byte ready in Receive Buffer */
#define UBSA_LSR_RCV_MASK	0x1f	/* Mask for incoming data or error */

/* modem status register */
/* All deltas are from the last read of the MSR. */
#define UBSA_MSR_DCD		0x80	/* Current Data Carrier Detect */
#define UBSA_MSR_RI		0x40	/* Current Ring Indicator */
#define UBSA_MSR_DSR		0x20	/* Current Data Set Ready */
#define	UBSA_MSR_CTS		0x10	/* Current Clear to Send */
#define UBSA_MSR_DDCD		0x08	/* DCD has changed state */
#define UBSA_MSR_TERI		0x04	/* RI has toggled low to high */
#define UBSA_MSR_DDSR		0x02	/* DSR has changed state */
#define UBSA_MSR_DCTS		0x01	/* CTS has changed state */

struct	ubsa_softc {
	struct ucom_softc	sc_ucom;
	struct usbd_memory_wait sc_mem_wait;

	struct usbd_xfer *	sc_xfer[UBSA_N_TRANSFER];

	u_int16_t		sc_flag;
#define UBSA_FLAG_WRITE_STALL   0x0001
#define UBSA_FLAG_READ_STALL    0x0002
#define UBSA_FLAG_INTR_STALL    0x0004

	u_int16_t		sc_reg_flag;
	u_int16_t		sc_reg[USBA_N_REG];

	u_int8_t		sc_iface_no; /* interface number */
	u_int8_t		sc_iface_index; /* interface index */
	u_int8_t		sc_dtr; /* current DTR state */
	u_int8_t		sc_rts; /* current RTS state */
	u_int8_t		sc_lsr; /* local status register */
	u_int8_t		sc_msr; /* UBSA status register */
};

static device_probe_t ubsa_probe;
static device_attach_t ubsa_attach;
static device_detach_t ubsa_detach;

static void
ubsa_request(struct ubsa_softc *sc, u_int8_t index, u_int16_t value);

static void
ubsa_set_dtr(struct ucom_softc *ucom, u_int8_t onoff);

static void
ubsa_set_rts(struct ucom_softc *ucom, u_int8_t onoff);

static void
ubsa_set_break(struct ucom_softc *ucom, u_int8_t onoff);

static u_int8_t
ubsa_baudrate(struct ubsa_softc *sc, speed_t speed);

static void
ubsa_parity(struct ubsa_softc *sc, tcflag_t cflag);

static void
ubsa_databits(struct ubsa_softc *sc, tcflag_t cflag);

static void
ubsa_stopbits(struct ubsa_softc *sc, tcflag_t cflag);

static void
ubsa_flow(struct ubsa_softc *sc, tcflag_t cflag, tcflag_t iflag);

static int
ubsa_param(struct ucom_softc *ucom, struct termios *ti);

static int
ubsa_open(struct ucom_softc *ucom);

static void
ubsa_close(struct ucom_softc *ucom);

static void
ubsa_start_read(struct ucom_softc *ucom);

static void
ubsa_stop_read(struct ucom_softc *ucom);

static void
ubsa_start_write(struct ucom_softc *ucom);

static void
ubsa_stop_write(struct ucom_softc *ucom);

static void
ubsa_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr);

static void
ubsa_write_callback(struct usbd_xfer *xfer);

static void
ubsa_write_clear_stall_callback(struct usbd_xfer *xfer);

static void
ubsa_read_callback(struct usbd_xfer *xfer);

static void
ubsa_read_clear_stall_callback(struct usbd_xfer *xfer);

static void
ubsa_intr_callback(struct usbd_xfer *xfer);

static void
ubsa_intr_clear_stall_callback(struct usbd_xfer *xfer);

static void
ubsa_request_callback(struct usbd_xfer *xfer, u_int8_t index);

static void
ubsa_set_baudrate_callback(struct usbd_xfer *xfer);

static void
ubsa_set_stop_bits_callback(struct usbd_xfer *xfer);

static void
ubsa_set_data_bits_callback(struct usbd_xfer *xfer);

static void
ubsa_set_parity_callback(struct usbd_xfer *xfer);

static void
ubsa_set_dtr_callback(struct usbd_xfer *xfer);

static void
ubsa_set_rts_callback(struct usbd_xfer *xfer);

static void
ubsa_set_break_callback(struct usbd_xfer *xfer);

static void
ubsa_set_flow_ctrl_callback(struct usbd_xfer *xfer);

static const struct usbd_config ubsa_config[UBSA_N_TRANSFER] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = UBSA_BSIZE, /* bytes */
      .flags     = 0,
      .callback  = &ubsa_write_callback,
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = UBSA_BSIZE, /* bytes */
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &ubsa_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [4] = {
      .type      = UE_INTERRUPT,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .flags     = USBD_SHORT_XFER_OK,
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &ubsa_intr_callback,
    },

    [5] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_intr_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [6+UBSA_REG_BAUDRATE] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_set_baudrate_callback,
      .timeout   = 1000, /* 1 second */
    },

    [6+UBSA_REG_STOP_BITS] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_set_stop_bits_callback,
      .timeout   = 1000, /* 1 second */
    },

    [6+UBSA_REG_DATA_BITS] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_set_data_bits_callback,
      .timeout   = 1000, /* 1 second */
    },

    [6+UBSA_REG_PARITY] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_set_parity_callback,
      .timeout   = 1000, /* 1 second */
    },

    [6+UBSA_REG_DTR] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_set_dtr_callback,
      .timeout   = 1000, /* 1 second */
    },

    [6+UBSA_REG_RTS] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_set_rts_callback,
      .timeout   = 1000, /* 1 second */
    },

    [6+UBSA_REG_BREAK] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_set_break_callback,
      .timeout   = 1000, /* 1 second */
    },

    [6+UBSA_REG_FLOW_CTRL] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ubsa_set_flow_ctrl_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static const struct ucom_callback ubsa_callback = {
    .ucom_get_status  = &ubsa_get_status,
    .ucom_set_dtr     = &ubsa_set_dtr,
    .ucom_set_rts     = &ubsa_set_rts,
    .ucom_set_break   = &ubsa_set_break,
    .ucom_param       = &ubsa_param,
    .ucom_open        = &ubsa_open,
    .ucom_close       = &ubsa_close,
    .ucom_start_read  = &ubsa_start_read,
    .ucom_stop_read   = &ubsa_stop_read,
    .ucom_start_write = &ubsa_start_write,
    .ucom_stop_write  = &ubsa_stop_write,
};

static const struct ubsa_product {
	u_int16_t	vendor;
	u_int16_t	product;
} ubsa_products [] = {
    /* BELKIN F5U103 */
    { USB_VENDOR_BELKIN, USB_PRODUCT_BELKIN_F5U103 },
    /* BELKIN F5U120 */
    { USB_VENDOR_BELKIN, USB_PRODUCT_BELKIN_F5U120 },
    /* GoHubs GO-COM232 */
    { USB_VENDOR_ETEK, USB_PRODUCT_ETEK_1COM },
    /* GoHubs GO-COM232 */
    { USB_VENDOR_GOHUBS, USB_PRODUCT_GOHUBS_GOCOM232 },
    /* Peracom */
    { USB_VENDOR_PERACOM, USB_PRODUCT_PERACOM_SERIAL1 },
    /* Vodafone */
    { USB_VENDOR_VODAFONE, USB_PRODUCT_VODAFONE_MC3G },
    { 0, 0 }
};

static device_method_t ubsa_methods[] = {
    DEVMETHOD(device_probe, ubsa_probe),
    DEVMETHOD(device_attach, ubsa_attach),
    DEVMETHOD(device_detach, ubsa_detach),
    { 0, 0 }
};

static driver_t ubsa_driver = {
    .name    = "ucom",
    .methods = ubsa_methods,
    .size    = sizeof(struct ubsa_softc),
};

DRIVER_MODULE(ubsa, uhub, ubsa_driver, ucom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(ubsa, usb, 1, 1, 1);
MODULE_DEPEND(ubsa, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);
MODULE_VERSION(ubsa, UBSA_MODVER);

static int
ubsa_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	const struct ubsa_product *up = ubsa_products;

	if (uaa->iface) {
	    return UMATCH_NONE;
	}

	while (up->vendor) {
	    if ((up->vendor == uaa->vendor) &&
		(up->product == uaa->product)) {
	        return UMATCH_VENDOR_PRODUCT;
	    }
	    up++;
	}
	return UMATCH_NONE;
}

static int
ubsa_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ubsa_softc *sc = device_get_softc(dev);
	struct usbd_interface *iface;
	usb_interface_descriptor_t *id;
	int error;

	DPRINTF(0, "sc=%p\n", sc);

	if (sc == NULL) {
	    return ENOMEM;
	}

	usbd_set_desc(dev, uaa->device);

	/* configure the device */

	error = usbd_set_config_index(uaa->device, UBSA_CONFIG_INDEX, 1);

	if (error) {
	    DPRINTF(0, "failed to set configuration, error=%s\n",
		    usbd_errstr(error));
	    goto detach;
	}

	iface = usbd_get_iface(uaa->device, UBSA_IFACE_INDEX);

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
	sc->sc_iface_index = UBSA_IFACE_INDEX;

	error = usbd_transfer_setup(uaa->device, sc->sc_iface_index,
				    sc->sc_xfer, ubsa_config, UBSA_N_TRANSFER,
				    sc, &Giant, &(sc->sc_mem_wait));
	if (error) {
	    DPRINTF(0, "could not allocate all pipes\n");
	    goto detach;
	}

	sc->sc_dtr = -1;
	sc->sc_rts = -1;

	sc->sc_ucom.sc_parent = sc;
	sc->sc_ucom.sc_portno = 0;
	sc->sc_ucom.sc_callback = &ubsa_callback;

	error = ucom_attach(&(sc->sc_ucom), dev);

	if (error) {
	    DPRINTF(0, "ucom_attach failed\n");
	    goto detach;
	}

	return 0;

 detach:
	ubsa_detach(dev);
	return ENXIO;
}

static int
ubsa_detach(device_t dev)
{
	struct ubsa_softc *sc = device_get_softc(dev);

	DPRINTF(0, "sc=%p\n", sc);

	ucom_detach(&(sc->sc_ucom));

	usbd_transfer_unsetup(sc->sc_xfer, UBSA_N_TRANSFER);

	usbd_transfer_drain(&(sc->sc_mem_wait), &Giant);

	return 0;
}

static void
ubsa_request(struct ubsa_softc *sc, u_int8_t index, u_int16_t value)
{
	if (index >= USBA_N_REG) {
	    panic("invalid register index!");
	}
	sc->sc_reg_flag |= (1 << index);
	sc->sc_reg[index] = value;
	usbd_transfer_start(sc->sc_xfer[6+index]);
	return;
}

static void
ubsa_set_dtr(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (sc->sc_dtr != onoff) {
	    sc->sc_dtr = onoff;
	    ubsa_request(sc, UBSA_REG_DTR, onoff ? 1 : 0);
	}
	return;
}

static void
ubsa_set_rts(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	if (sc->sc_rts != onoff) {
	    sc->sc_rts = onoff;
	    ubsa_request(sc, UBSA_REG_RTS, onoff ? 1 : 0);
	}
	return;
}

static void
ubsa_set_break(struct ucom_softc *ucom, u_int8_t onoff)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	ubsa_request(sc, UBSA_REG_BREAK, onoff ? 1 : 0);
	return;
}

static u_int8_t
ubsa_baudrate(struct ubsa_softc *sc, speed_t speed)
{
	u_int16_t value = 0;

	DPRINTF(0, "speed = %d\n", speed);

	switch(speed) {
	case B0:
		break;
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
	case B230400:
	    value = B230400 / speed;
	    break;

	default:
	    DPRINTF(0, "unsupported baudrate\n");
	    return 1;
	};

	if (speed == B0) {
	    ubsa_flow(sc, 0, 0);
	    ubsa_set_dtr(&(sc->sc_ucom), 0);
	    ubsa_set_rts(&(sc->sc_ucom), 0);
	} else {
	    ubsa_request(sc, UBSA_REG_BAUDRATE, value);
	}
	return 0;
}

static void
ubsa_parity(struct ubsa_softc *sc, tcflag_t cflag)
{
	u_int16_t value;

	DPRINTF(0, "flag = 0x%x\n", cflag);

	if (cflag & PARENB)
		value = (cflag & PARODD) ? UBSA_PARITY_ODD : UBSA_PARITY_EVEN;
	else
		value = UBSA_PARITY_NONE;

	ubsa_request(sc, UBSA_REG_PARITY, value);
	return;
}

static void
ubsa_databits(struct ubsa_softc *sc, tcflag_t cflag)
{
	u_int16_t value;

	DPRINTF(0, "cflag = 0x%x\n", cflag);

	switch (cflag & CSIZE) {
	case CS5: value = 0; break;
	case CS6: value = 1; break;
	case CS7: value = 2; break;
	case CS8: value = 3; break;
	default:
	    DPRINTF(0, "unsupported databits requested, "
		    "forcing default of 8\n");
	    value = 3;
	}

	ubsa_request(sc, UBSA_REG_DATA_BITS, value);
	return;
}

static void
ubsa_stopbits(struct ubsa_softc *sc, tcflag_t cflag)
{
	u_int16_t value;

	DPRINTF(0, "cflag = 0x%x\n", cflag);

	value = (cflag & CSTOPB) ? 1 : 0;

	ubsa_request(sc, UBSA_REG_STOP_BITS, value);
	return;
}

static void
ubsa_flow(struct ubsa_softc *sc, tcflag_t cflag, tcflag_t iflag)
{
	u_int16_t value;

	DPRINTF(0, "cflag = 0x%x, iflag = 0x%x\n", cflag, iflag);

	value = 0;
	if (cflag & CRTSCTS)
		value |= UBSA_FLOW_OCTS | UBSA_FLOW_IRTS;
	if (iflag & (IXON|IXOFF))
		value |= UBSA_FLOW_OXON | UBSA_FLOW_IXON;

	ubsa_request(sc, UBSA_REG_FLOW_CTRL, value);
	return;
}

static int
ubsa_param(struct ucom_softc *ucom, struct termios *ti)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "sc = %p\n", sc);

	if (ubsa_baudrate(sc, ti->c_ospeed)) {
	    return EIO;
	}
	ubsa_parity(sc, ti->c_cflag);
	ubsa_databits(sc, ti->c_cflag);
	ubsa_stopbits(sc, ti->c_cflag);
	ubsa_flow(sc, ti->c_cflag, ti->c_iflag);

	return (0);
}

static int
ubsa_open(struct ucom_softc *ucom)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	/* clear stall first: */
	sc->sc_flag |= (UBSA_FLAG_WRITE_STALL|
			UBSA_FLAG_READ_STALL);

	usbd_transfer_start(sc->sc_xfer[4]);
	return 0;
}

static void
ubsa_close(struct ucom_softc *ucom)
{
	struct ubsa_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer[5]);
	usbd_transfer_stop(sc->sc_xfer[4]);
	return;
}

static void
ubsa_start_read(struct ucom_softc *ucom)
{
	struct ubsa_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
ubsa_stop_read(struct ucom_softc *ucom)
{
	struct ubsa_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
ubsa_start_write(struct ucom_softc *ucom)
{
	struct ubsa_softc *sc = ucom->sc_parent;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
ubsa_stop_write(struct ucom_softc *ucom)
{
	struct ubsa_softc *sc = ucom->sc_parent;
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

static void
ubsa_get_status(struct ucom_softc *ucom, u_int8_t *lsr, u_int8_t *msr)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "\n");

	if (lsr) {
	    *lsr = sc->sc_lsr;
	}

	if (msr) {
	    *msr = sc->sc_msr;
	}
	return;
}

static void
ubsa_write_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;
	u_int32_t actlen;

	USBD_CHECK_STATUS(xfer);
tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UBSA_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}
	return;

tr_setup:
tr_transferred:
	if (sc->sc_flag & UBSA_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    return;
	}

	if(ucom_get_data(&(sc->sc_ucom), xfer->buffer, UBSA_BSIZE, &actlen)) {

	    xfer->length = actlen;

	    usbd_start_hardware(xfer);
	}
	return;
}

static void
ubsa_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[0]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[0]);
	sc->sc_flag &= ~UBSA_FLAG_WRITE_STALL;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;

 tr_error:
	sc->sc_flag &= ~UBSA_FLAG_WRITE_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
ubsa_read_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UBSA_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;

 tr_transferred:
	ucom_put_data(&(sc->sc_ucom), xfer->buffer, xfer->actlen);

 tr_setup:
	if (sc->sc_flag & UBSA_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
ubsa_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[1]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[1]);
	sc->sc_flag &= ~UBSA_FLAG_READ_STALL;
	usbd_transfer_start(sc->sc_xfer[1]);
	return;

 tr_error:
	sc->sc_flag &= ~UBSA_FLAG_READ_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
ubsa_intr_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;
	u_int8_t *buf = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    sc->sc_flag |= UBSA_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[5]);
	}
	return;

 tr_transferred:

	if (xfer->actlen >= 4) {

	    /* incidentally, Belkin adapter status bits match UART 16550 bits */
	    sc->sc_lsr = buf[2];
	    sc->sc_msr = buf[3];

	    DPRINTF(0, "lsr = 0x%02x, msr = 0x%02x\n",
		    sc->sc_lsr, sc->sc_msr);

	    ucom_status_change(&(sc->sc_ucom));
	} else {
	    DPRINTF(0, "ignoring short packet, %d bytes\n",
		    xfer->actlen);
	}

 tr_setup:
	if (sc->sc_flag & UBSA_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[5]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
ubsa_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[4]);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[4]);
	sc->sc_flag &= ~UBSA_FLAG_INTR_STALL;
	usbd_transfer_start(sc->sc_xfer[4]);
	return;

 tr_error:
	sc->sc_flag &= ~UBSA_FLAG_INTR_STALL;
	DPRINTF(0, "clear stall failed, error=%s\n",
		usbd_errstr(xfer->error));
	return;
}

static void
ubsa_request_callback(struct usbd_xfer *xfer, u_int8_t index)
{
	static const u_int8_t mapping[USBA_N_REG] = {
	  [UBSA_REG_BAUDRATE]  = 0x00,
	  [UBSA_REG_STOP_BITS] = 0x01,
	  [UBSA_REG_DATA_BITS] = 0x02,
	  [UBSA_REG_PARITY]    = 0x03,
	  [UBSA_REG_DTR]       = 0x0A,
	  [UBSA_REG_RTS]       = 0x0B,
	  [UBSA_REG_BREAK]     = 0x0C,
	  [UBSA_REG_FLOW_CTRL] = 0x10,
	};

	usb_device_request_t *req = xfer->buffer;
	struct ubsa_softc *sc = xfer->priv_sc;
	u_int8_t request = mapping[index];
	u_int16_t value = sc->sc_reg[index];
	u_int16_t bitmask = (1 << index);

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(0, "transfer failed, "
		"error=%s\n", usbd_errstr(xfer->error));
 tr_setup:
 tr_transferred:
	if (sc->sc_reg_flag &   bitmask) {
	    sc->sc_reg_flag &= ~bitmask;

	    req->bmRequestType = UT_WRITE_VENDOR_DEVICE;
	    req->bRequest = request;
	    USETW(req->wValue, value);
	    USETW(req->wIndex, sc->sc_iface_no);
	    USETW(req->wLength, 0);

	    usbd_start_hardware(xfer);
	}
	return;
}

static void
ubsa_set_baudrate_callback(struct usbd_xfer *xfer)
{
    ubsa_request_callback(xfer, UBSA_REG_BAUDRATE);
    return;
}

static void
ubsa_set_stop_bits_callback(struct usbd_xfer *xfer)
{
    ubsa_request_callback(xfer, UBSA_REG_STOP_BITS);
    return;
}

static void
ubsa_set_data_bits_callback(struct usbd_xfer *xfer)
{
    ubsa_request_callback(xfer, UBSA_REG_DATA_BITS);
    return;
}

static void
ubsa_set_parity_callback(struct usbd_xfer *xfer)
{
    ubsa_request_callback(xfer, UBSA_REG_PARITY);
    return;
}

static void
ubsa_set_dtr_callback(struct usbd_xfer *xfer)
{
    ubsa_request_callback(xfer, UBSA_REG_DTR);
    return;
}

static void
ubsa_set_rts_callback(struct usbd_xfer *xfer)
{
    ubsa_request_callback(xfer, UBSA_REG_RTS);
    return;
}

static void
ubsa_set_break_callback(struct usbd_xfer *xfer)
{
    ubsa_request_callback(xfer, UBSA_REG_BREAK);
    return;
}

static void
ubsa_set_flow_ctrl_callback(struct usbd_xfer *xfer)
{
    ubsa_request_callback(xfer, UBSA_REG_FLOW_CTRL);
    return;
}
