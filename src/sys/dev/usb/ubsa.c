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
__FBSDID("$FreeBSD: src/sys/dev/usb/ubsa.c,v 1.32 2007/06/22 05:56:05 imp Exp $");

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

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>
#include <dev/usb/usb_cdc.h>

#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

#ifdef USB_DEBUG
#define	DPRINTF(n,fmt,...)						\
  do { if (ubsa_debug > (n)) {						\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ubsa_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, ubsa, CTLFLAG_RW, 0, "USB ubsa");
SYSCTL_INT(_hw_usb_ubsa, OID_AUTO, debug, CTLFLAG_RW,
    &ubsa_debug, 0, "ubsa debug level");
#else
#define	DPRINTF(...) do { } while (0)
#endif

#define	UBSA_N_TRANSFER           6	/* units */
#define	UBSA_BSIZE             1024	/* bytes */

#define	UBSA_CONFIG_INDEX	1
#define	UBSA_IFACE_INDEX	0

#define	UBSA_REG_BAUDRATE	0x00
#define	UBSA_REG_STOP_BITS	0x01
#define	UBSA_REG_DATA_BITS	0x02
#define	UBSA_REG_PARITY		0x03
#define	UBSA_REG_DTR		0x0A
#define	UBSA_REG_RTS		0x0B
#define	UBSA_REG_BREAK		0x0C
#define	UBSA_REG_FLOW_CTRL	0x10

#define	UBSA_PARITY_NONE	0x00
#define	UBSA_PARITY_EVEN	0x01
#define	UBSA_PARITY_ODD		0x02
#define	UBSA_PARITY_MARK	0x03
#define	UBSA_PARITY_SPACE	0x04

#define	UBSA_FLOW_NONE		0x0000
#define	UBSA_FLOW_OCTS		0x0001
#define	UBSA_FLOW_ODSR		0x0002
#define	UBSA_FLOW_IDSR		0x0004
#define	UBSA_FLOW_IDTR		0x0008
#define	UBSA_FLOW_IRTS		0x0010
#define	UBSA_FLOW_ORTS		0x0020
#define	UBSA_FLOW_UNKNOWN	0x0040
#define	UBSA_FLOW_OXON		0x0080
#define	UBSA_FLOW_IXON		0x0100

/* line status register */
#define	UBSA_LSR_TSRE		0x40	/* Transmitter empty: byte sent */
#define	UBSA_LSR_TXRDY		0x20	/* Transmitter buffer empty */
#define	UBSA_LSR_BI		0x10	/* Break detected */
#define	UBSA_LSR_FE		0x08	/* Framing error: bad stop bit */
#define	UBSA_LSR_PE		0x04	/* Parity error */
#define	UBSA_LSR_OE		0x02	/* Overrun, lost incoming byte */
#define	UBSA_LSR_RXRDY		0x01	/* Byte ready in Receive Buffer */
#define	UBSA_LSR_RCV_MASK	0x1f	/* Mask for incoming data or error */

/* modem status register */
/* All deltas are from the last read of the MSR. */
#define	UBSA_MSR_DCD		0x80	/* Current Data Carrier Detect */
#define	UBSA_MSR_RI		0x40	/* Current Ring Indicator */
#define	UBSA_MSR_DSR		0x20	/* Current Data Set Ready */
#define	UBSA_MSR_CTS		0x10	/* Current Clear to Send */
#define	UBSA_MSR_DDCD		0x08	/* DCD has changed state */
#define	UBSA_MSR_TERI		0x04	/* RI has toggled low to high */
#define	UBSA_MSR_DDSR		0x02	/* DSR has changed state */
#define	UBSA_MSR_DCTS		0x01	/* CTS has changed state */

struct ubsa_softc {
	struct ucom_super_softc sc_super_ucom;
	struct ucom_softc sc_ucom;

	struct usbd_xfer *sc_xfer[UBSA_N_TRANSFER];
	struct usbd_device *sc_udev;

	uint16_t sc_flag;
#define	UBSA_FLAG_WRITE_STALL   0x0001
#define	UBSA_FLAG_READ_STALL    0x0002
#define	UBSA_FLAG_INTR_STALL    0x0004

	uint8_t	sc_iface_no;		/* interface number */
	uint8_t	sc_iface_index;		/* interface index */
	uint8_t	sc_lsr;			/* local status register */
	uint8_t	sc_msr;			/* UBSA status register */
};

static device_probe_t ubsa_probe;
static device_attach_t ubsa_attach;
static device_detach_t ubsa_detach;

static usbd_callback_t ubsa_write_callback;
static usbd_callback_t ubsa_write_clear_stall_callback;
static usbd_callback_t ubsa_read_callback;
static usbd_callback_t ubsa_read_clear_stall_callback;
static usbd_callback_t ubsa_intr_callback;
static usbd_callback_t ubsa_intr_clear_stall_callback;

static void ubsa_cfg_request(struct ubsa_softc *sc, uint8_t index, uint16_t value);
static void ubsa_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff);
static void ubsa_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff);
static void ubsa_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff);
static int ubsa_pre_param(struct ucom_softc *ucom, struct termios *t);
static void ubsa_cfg_param(struct ucom_softc *ucom, struct termios *t);
static void ubsa_start_read(struct ucom_softc *ucom);
static void ubsa_stop_read(struct ucom_softc *ucom);
static void ubsa_start_write(struct ucom_softc *ucom);
static void ubsa_stop_write(struct ucom_softc *ucom);
static void ubsa_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr);

static const struct usbd_config ubsa_config[UBSA_N_TRANSFER] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.bufsize = UBSA_BSIZE,	/* bytes */
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &ubsa_write_callback,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = UBSA_BSIZE,	/* bytes */
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &ubsa_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &ubsa_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.interval = 50,		/* 50ms */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &ubsa_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.interval = 50,		/* 50ms */
	},

	[4] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.bufsize = 0,		/* use wMaxPacketSize */
		.mh.callback = &ubsa_intr_callback,
	},

	[5] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &ubsa_intr_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.interval = 50,		/* 50ms */
	},
};

static const struct ucom_callback ubsa_callback = {
	.ucom_cfg_get_status = &ubsa_cfg_get_status,
	.ucom_cfg_set_dtr = &ubsa_cfg_set_dtr,
	.ucom_cfg_set_rts = &ubsa_cfg_set_rts,
	.ucom_cfg_set_break = &ubsa_cfg_set_break,
	.ucom_cfg_param = &ubsa_cfg_param,
	.ucom_pre_param = &ubsa_pre_param,
	.ucom_start_read = &ubsa_start_read,
	.ucom_stop_read = &ubsa_stop_read,
	.ucom_start_write = &ubsa_start_write,
	.ucom_stop_write = &ubsa_stop_write,
};

struct ubsa_product {
	uint16_t vendor;
	uint16_t product;
};

static const struct ubsa_product ubsa_products[] = {
	/* AnyData ADU-E100A/H */
	{USB_VENDOR_ANYDATA, USB_PRODUCT_ANYDATA_ADU_E100X},
	/* BELKIN F5U103 */
	{USB_VENDOR_BELKIN, USB_PRODUCT_BELKIN_F5U103},
	/* BELKIN F5U120 */
	{USB_VENDOR_BELKIN, USB_PRODUCT_BELKIN_F5U120},
	/* GoHubs GO-COM232 */
	{USB_VENDOR_ETEK, USB_PRODUCT_ETEK_1COM},
	/* GoHubs GO-COM232 */
	{USB_VENDOR_GOHUBS, USB_PRODUCT_GOHUBS_GOCOM232},
	/* Peracom */
	{USB_VENDOR_PERACOM, USB_PRODUCT_PERACOM_SERIAL1},
	/* Novatel Wireless Merlin cards */
	{USB_VENDOR_NOVATEL, USB_PRODUCT_NOVATEL_U740},
	/* Option Vodafone MC3G */
	{USB_VENDOR_OPTION, USB_PRODUCT_OPTION_VODAFONEMC3G},
	/* Option GlobeTrotter 3G */
	{USB_VENDOR_OPTION, USB_PRODUCT_OPTION_GT3G},
	/* Option GlobeTrotter 3G+ */
	{USB_VENDOR_OPTION, USB_PRODUCT_OPTION_GT3GPLUS},
	/* Option GlobeTrotter 3G QUAD */
	{USB_VENDOR_OPTION, USB_PRODUCT_OPTION_GT3GQUAD},
	/* Huawei Mobile */
	{USB_VENDOR_HUAWEI, USB_PRODUCT_HUAWEI_MOBILE},
	{0, 0}
};

static device_method_t ubsa_methods[] = {
	DEVMETHOD(device_probe, ubsa_probe),
	DEVMETHOD(device_attach, ubsa_attach),
	DEVMETHOD(device_detach, ubsa_detach),
	{0, 0}
};

static devclass_t ubsa_devclass;

static driver_t ubsa_driver = {
	.name = "ubsa",
	.methods = ubsa_methods,
	.size = sizeof(struct ubsa_softc),
};

DRIVER_MODULE(ubsa, uhub, ubsa_driver, ubsa_devclass, usbd_driver_load, 0);
MODULE_DEPEND(ubsa, usb, 1, 1, 1);
MODULE_DEPEND(ubsa, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static int
ubsa_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	const struct ubsa_product *up = ubsa_products;

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->iface) {
		return (UMATCH_NONE);
	}
	while (up->vendor) {
		if ((up->vendor == uaa->vendor) &&
		    (up->product == uaa->product)) {
			return (UMATCH_VENDOR_PRODUCT);
		}
		up++;
	}
	return (UMATCH_NONE);
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
		return (ENOMEM);
	}
	usbd_set_device_desc(dev);

	sc->sc_udev = uaa->device;

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

	error = usbd_transfer_setup(uaa->device, &(sc->sc_iface_index),
	    sc->sc_xfer, ubsa_config, UBSA_N_TRANSFER, sc, &Giant);

	if (error) {
		DPRINTF(0, "could not allocate all pipes\n");
		goto detach;
	}
	/* clear stall at first run */
	sc->sc_flag |= (UBSA_FLAG_WRITE_STALL |
	    UBSA_FLAG_READ_STALL);

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
	    &ubsa_callback, &Giant);
	if (error) {
		DPRINTF(0, "ucom_attach failed\n");
		goto detach;
	}
	return (0);

detach:
	ubsa_detach(dev);
	return (ENXIO);
}

static int
ubsa_detach(device_t dev)
{
	struct ubsa_softc *sc = device_get_softc(dev);

	DPRINTF(0, "sc=%p\n", sc);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UBSA_N_TRANSFER);

	return (0);
}

static void
ubsa_cfg_request(struct ubsa_softc *sc, uint8_t index, uint16_t value)
{
	usb_device_request_t req;
	usbd_status_t err;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
		return;
	}
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = index;
	USETW(req.wValue, value);
	req.wIndex[0] = sc->sc_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	err = usbd_do_request_flags
	    (sc->sc_udev, &Giant, &req, NULL, 0, NULL, 1000);

	if (err) {
		DPRINTF(-1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));
	}
	return;
}

static void
ubsa_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	ubsa_cfg_request(sc, UBSA_REG_DTR, onoff ? 1 : 0);
	return;
}

static void
ubsa_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	ubsa_cfg_request(sc, UBSA_REG_RTS, onoff ? 1 : 0);
	return;
}

static void
ubsa_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "onoff = %d\n", onoff);

	ubsa_cfg_request(sc, UBSA_REG_BREAK, onoff ? 1 : 0);
	return;
}

static int
ubsa_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "sc = %p\n", sc);

	switch (t->c_ospeed) {
	case B0:
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
		break;
	default:
		return (EINVAL);
	}
	return (0);
}

static void
ubsa_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct ubsa_softc *sc = ucom->sc_parent;
	uint16_t value = 0;

	DPRINTF(0, "sc = %p\n", sc);

	switch (t->c_ospeed) {
	case B0:
		ubsa_cfg_request(sc, UBSA_REG_FLOW_CTRL, 0);
		ubsa_cfg_set_dtr(&(sc->sc_ucom), 0);
		ubsa_cfg_set_rts(&(sc->sc_ucom), 0);
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
		value = B230400 / t->c_ospeed;
		ubsa_cfg_request(sc, UBSA_REG_BAUDRATE, value);
		break;
	default:
		return;
	}

	if (t->c_cflag & PARENB)
		value = (t->c_cflag & PARODD) ? UBSA_PARITY_ODD : UBSA_PARITY_EVEN;
	else
		value = UBSA_PARITY_NONE;

	ubsa_cfg_request(sc, UBSA_REG_PARITY, value);

	switch (t->c_cflag & CSIZE) {
	case CS5:
		value = 0;
		break;
	case CS6:
		value = 1;
		break;
	case CS7:
		value = 2;
		break;
	default:
	case CS8:
		value = 3;
		break;
	}

	ubsa_cfg_request(sc, UBSA_REG_DATA_BITS, value);

	value = (t->c_cflag & CSTOPB) ? 1 : 0;

	ubsa_cfg_request(sc, UBSA_REG_STOP_BITS, value);

	value = 0;
	if (t->c_cflag & CRTSCTS)
		value |= UBSA_FLOW_OCTS | UBSA_FLOW_IRTS;

	if (t->c_iflag & (IXON | IXOFF))
		value |= UBSA_FLOW_OXON | UBSA_FLOW_IXON;

	ubsa_cfg_request(sc, UBSA_REG_FLOW_CTRL, value);
	return;
}

static void
ubsa_start_read(struct ucom_softc *ucom)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	/* start interrupt endpoint */
	usbd_transfer_start(sc->sc_xfer[4]);

	/* start read endpoint */
	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
ubsa_stop_read(struct ucom_softc *ucom)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	/* stop interrupt endpoint */
	usbd_transfer_stop(sc->sc_xfer[5]);
	usbd_transfer_stop(sc->sc_xfer[4]);

	/* stop read endpoint */
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
ubsa_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr)
{
	struct ubsa_softc *sc = ucom->sc_parent;

	DPRINTF(0, "\n");

	*lsr = sc->sc_lsr;
	*msr = sc->sc_msr;
	return;
}

static void
ubsa_write_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
	case USBD_ST_TRANSFERRED:
		if (sc->sc_flag & UBSA_FLAG_WRITE_STALL) {
			usbd_transfer_start(sc->sc_xfer[2]);
			return;
		}
		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers + 0, 0,
		    UBSA_BSIZE, &actlen)) {

			xfer->frlengths[0] = actlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_CANCELLED) {
			sc->sc_flag |= UBSA_FLAG_WRITE_STALL;
			usbd_transfer_start(sc->sc_xfer[2]);
		}
		return;

	}
}

static void
ubsa_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flag &= ~UBSA_FLAG_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ubsa_read_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		ucom_put_data(&(sc->sc_ucom), xfer->frbuffers + 0, 0, xfer->actlen);

	case USBD_ST_SETUP:
		if (sc->sc_flag & UBSA_FLAG_READ_STALL) {
			usbd_transfer_start(sc->sc_xfer[3]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_CANCELLED) {
			sc->sc_flag |= UBSA_FLAG_READ_STALL;
			usbd_transfer_start(sc->sc_xfer[3]);
		}
		return;

	}
}

static void
ubsa_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flag &= ~UBSA_FLAG_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ubsa_intr_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;
	uint8_t buf[4];

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		if (xfer->actlen >= sizeof(buf)) {

			usbd_copy_out(xfer->frbuffers + 0, 0, buf, sizeof(buf));

			/*
			 * incidentally, Belkin adapter status bits match
			 * UART 16550 bits
			 */
			sc->sc_lsr = buf[2];
			sc->sc_msr = buf[3];

			DPRINTF(0, "lsr = 0x%02x, msr = 0x%02x\n",
			    sc->sc_lsr, sc->sc_msr);

			ucom_status_change(&(sc->sc_ucom));
		} else {
			DPRINTF(0, "ignoring short packet, %d bytes\n",
			    xfer->actlen);
		}

	case USBD_ST_SETUP:
		if (sc->sc_flag & UBSA_FLAG_INTR_STALL) {
			usbd_transfer_start(sc->sc_xfer[5]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_CANCELLED) {
			sc->sc_flag |= UBSA_FLAG_INTR_STALL;
			usbd_transfer_start(sc->sc_xfer[5]);
		}
		return;

	}
}

static void
ubsa_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubsa_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flag &= ~UBSA_FLAG_INTR_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}
