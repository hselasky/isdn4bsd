/*	$NetBSD: uchcom.c,v 1.1 2007/09/03 17:57:37 tshiozak Exp $	*/

/*-
 * Copyright (c) 2007, Takanori Watanabe
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

/*
 * Copyright (c) 2007 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Takuya SHIOZAKI (tshiozak@netbsd.org).
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
__FBSDID("$FreeBSD: src/sys/dev/usb/uchcom.c,v 1.1 2007/10/18 10:51:06 takawata Exp $");

/*
 * driver for WinChipHead CH341/340, the worst USB-serial chip in the world.
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
#define	DPRINTFN(n,fmt,...)						\
  do { if (uchcom_debug > (n)) {					\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uchcom_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, uchcom, CTLFLAG_RW, 0, "USB uchcom");
SYSCTL_INT(_hw_usb_uchcom, OID_AUTO, debug, CTLFLAG_RW,
    &uchcom_debug, 0, "uchcom debug level");
#else
#define	DPRINTFN(...) do { } while (0)
#endif

#define	UCHCOM_IFACE_INDEX	0
#define	UCHCOM_CONFIG_INDEX	0

#define	UCHCOM_REV_CH340	0x0250
#define	UCHCOM_INPUT_BUF_SIZE	8

#define	UCHCOM_REQ_GET_VERSION	0x5F
#define	UCHCOM_REQ_READ_REG	0x95
#define	UCHCOM_REQ_WRITE_REG	0x9A
#define	UCHCOM_REQ_RESET	0xA1
#define	UCHCOM_REQ_SET_DTRRTS	0xA4

#define	UCHCOM_REG_STAT1	0x06
#define	UCHCOM_REG_STAT2	0x07
#define	UCHCOM_REG_BPS_PRE	0x12
#define	UCHCOM_REG_BPS_DIV	0x13
#define	UCHCOM_REG_BPS_MOD	0x14
#define	UCHCOM_REG_BPS_PAD	0x0F
#define	UCHCOM_REG_BREAK1	0x05
#define	UCHCOM_REG_BREAK2	0x18
#define	UCHCOM_REG_LCR1		0x18
#define	UCHCOM_REG_LCR2		0x25

#define	UCHCOM_VER_20		0x20

#define	UCHCOM_BASE_UNKNOWN	0
#define	UCHCOM_BPS_MOD_BASE	20000000
#define	UCHCOM_BPS_MOD_BASE_OFS	1100

#define	UCHCOM_DTR_MASK		0x20
#define	UCHCOM_RTS_MASK		0x40

#define	UCHCOM_BRK1_MASK	0x01
#define	UCHCOM_BRK2_MASK	0x40

#define	UCHCOM_LCR1_MASK	0xAF
#define	UCHCOM_LCR2_MASK	0x07
#define	UCHCOM_LCR1_PARENB	0x80
#define	UCHCOM_LCR2_PAREVEN	0x07
#define	UCHCOM_LCR2_PARODD	0x06
#define	UCHCOM_LCR2_PARMARK	0x05
#define	UCHCOM_LCR2_PARSPACE	0x04

#define	UCHCOM_INTR_STAT1	0x02
#define	UCHCOM_INTR_STAT2	0x03
#define	UCHCOM_INTR_LEAST	4

#define	UCHCOM_BULK_BUF_SIZE 1024	/* bytes */
#define	UCHCOM_N_TRANSFER 6		/* units */

struct uchcom_softc {
	struct ucom_super_softc sc_super_ucom;
	struct ucom_softc sc_ucom;

	struct usbd_xfer *sc_xfer[UCHCOM_N_TRANSFER];
	struct usbd_device *sc_udev;

	uint8_t	sc_dtr;			/* local copy */
	uint8_t	sc_rts;			/* local copy */
	uint8_t	sc_version;
	uint8_t	sc_msr;
	uint8_t	sc_lsr;			/* local status register */
	uint8_t	sc_flag;
#define	UCHCOM_FLAG_INTR_STALL  0x01
#define	UCHCOM_FLAG_READ_STALL  0x02
#define	UCHCOM_FLAG_WRITE_STALL 0x04
};

struct uchcom_divider {
	uint8_t	dv_prescaler;
	uint8_t	dv_div;
	uint8_t	dv_mod;
};

struct uchcom_divider_record {
	uint32_t dvr_high;
	uint32_t dvr_low;
	uint32_t dvr_base_clock;
	struct uchcom_divider dvr_divider;
};

static const struct uchcom_divider_record dividers[] =
{
	{307200, 307200, UCHCOM_BASE_UNKNOWN, {7, 0xD9, 0}},
	{921600, 921600, UCHCOM_BASE_UNKNOWN, {7, 0xF3, 0}},
	{2999999, 23530, 6000000, {3, 0, 0}},
	{23529, 2942, 750000, {2, 0, 0}},
	{2941, 368, 93750, {1, 0, 0}},
	{367, 1, 11719, {0, 0, 0}},
};

#define	NUM_DIVIDERS	(sizeof (dividers) / sizeof (dividers[0]))

static const struct usb_devno uchcom_devs[] = {
	{USB_VENDOR_WCH, USB_PRODUCT_WCH_CH341SER},
};

#define	uchcom_lookup(v, p)	usb_lookup(uchcom_devs, v, p)

/* protypes */

static int uchcom_ioctl(struct ucom_softc *ucom, uint32_t cmd, caddr_t data, int flag, struct thread *td);
static int uchcom_pre_param(struct ucom_softc *ucom, struct termios *t);
static void uchcom_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr);
static void uchcom_cfg_param(struct ucom_softc *ucom, struct termios *t);
static void uchcom_cfg_set_break(struct ucom_softc *sc, uint8_t onoff);
static void uchcom_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff);
static void uchcom_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff);
static void uchcom_start_read(struct ucom_softc *ucom);
static void uchcom_start_write(struct ucom_softc *ucom);
static void uchcom_stop_read(struct ucom_softc *ucom);
static void uchcom_stop_write(struct ucom_softc *ucom);

static void uchcom_update_version(struct uchcom_softc *sc);
static void uchcom_convert_status(struct uchcom_softc *sc, uint8_t cur);
static void uchcom_update_status(struct uchcom_softc *sc);
static void uchcom_set_dtrrts(struct uchcom_softc *sc);
static int uchcom_calc_divider_settings(struct uchcom_divider *dp, uint32_t rate);
static void uchcom_set_dte_rate(struct uchcom_softc *sc, uint32_t rate);
static void uchcom_set_line_control(struct uchcom_softc *sc, tcflag_t cflag);
static void uchcom_clear_chip(struct uchcom_softc *sc);
static void uchcom_reset_chip(struct uchcom_softc *sc);

static device_probe_t uchcom_probe;
static device_attach_t uchcom_attach;
static device_detach_t uchcom_detach;

static usbd_callback_t uchcom_intr_callback;
static usbd_callback_t uchcom_intr_clear_stall_callback;
static usbd_callback_t uchcom_write_callback;
static usbd_callback_t uchcom_write_clear_stall_callback;
static usbd_callback_t uchcom_read_callback;
static usbd_callback_t uchcom_read_clear_stall_callback;

static const struct usbd_config uchcom_config_data[UCHCOM_N_TRANSFER] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.mh.bufsize = UCHCOM_BULK_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &uchcom_write_callback,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.bufsize = UCHCOM_BULK_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &uchcom_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uchcom_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uchcom_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[4] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.bufsize = 0,	/* use wMaxPacketSize */
		.mh.callback = &uchcom_intr_callback,
	},

	[5] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uchcom_intr_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},
};

struct ucom_callback uchcom_callback = {
	.ucom_cfg_get_status = &uchcom_cfg_get_status,
	.ucom_cfg_set_dtr = &uchcom_cfg_set_dtr,
	.ucom_cfg_set_rts = &uchcom_cfg_set_rts,
	.ucom_cfg_set_break = &uchcom_cfg_set_break,
	.ucom_cfg_param = &uchcom_cfg_param,
	.ucom_pre_param = &uchcom_pre_param,
	.ucom_ioctl = &uchcom_ioctl,
	.ucom_start_read = &uchcom_start_read,
	.ucom_stop_read = &uchcom_stop_read,
	.ucom_start_write = &uchcom_start_write,
	.ucom_stop_write = &uchcom_stop_write,
};

/* ----------------------------------------------------------------------
 * driver entry points
 */

static int
uchcom_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	DPRINTFN(10, "\n");

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->iface) {
		return (UMATCH_NONE);
	}
	return (uchcom_lookup(uaa->vendor, uaa->product) != NULL ?
	    UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

static int
uchcom_attach(device_t dev)
{
	struct uchcom_softc *sc = device_get_softc(dev);
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	int error;
	uint8_t iface_index;

	DPRINTFN(10, "\n");

	if (sc == NULL) {
		return (ENOMEM);
	}
	usbd_set_device_desc(dev);

	sc->sc_udev = uaa->device;

	/* configure the chip */

	error = usbd_set_config_index(uaa->device, UCHCOM_CONFIG_INDEX, 1);

	if (error) {
		device_printf(dev, "failed to set configuration, "
		    "error=%s\n", usbd_errstr(error));
		goto detach;
	}
	switch (uaa->release) {
	case UCHCOM_REV_CH340:
		device_printf(dev, "CH340 detected\n");
		break;
	default:
		device_printf(dev, "CH341 detected\n");
		break;
	}

	iface_index = UCHCOM_IFACE_INDEX;
	error = usbd_transfer_setup(uaa->device,
	    &iface_index, sc->sc_xfer, uchcom_config_data,
	    UCHCOM_N_TRANSFER, sc, &Giant);

	if (error) {
		DPRINTFN(0, "one or more missing USB endpoints, "
		    "error=%s\n", usbd_errstr(error));
		goto detach;
	}
	/*
	 * Do the initialization during attach so that the system does not
	 * sleep during open:
	 */
	uchcom_update_version(sc);
	uchcom_clear_chip(sc);
	uchcom_reset_chip(sc);
	uchcom_update_status(sc);

	sc->sc_dtr = 1;
	sc->sc_rts = 1;

	/* clear stall at first run */
	sc->sc_flag |= (UCHCOM_FLAG_READ_STALL |
	    UCHCOM_FLAG_WRITE_STALL);

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
	    &uchcom_callback, &Giant);
	if (error) {
		goto detach;
	}
	return (0);

detach:
	uchcom_detach(dev);
	return (ENXIO);
}

static int
uchcom_detach(device_t dev)
{
	struct uchcom_softc *sc = device_get_softc(dev);

	DPRINTFN(10, "\n");

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_xfer, UCHCOM_N_TRANSFER);

	return (0);
}

/* ----------------------------------------------------------------------
 * low level i/o
 */

static void
uchcom_do_request(struct uchcom_softc *sc,
    usb_device_request_t *req, void *data)
{
	uint16_t length;
	uint16_t actlen;
	usbd_status_t err;

	length = UGETW(req->wLength);
	actlen = 0;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
		goto done;
	}
	err = usbd_do_request_flags(sc->sc_udev, &Giant, req,
	    data, USBD_SHORT_XFER_OK, &actlen, 1000);

	if (err) {
		DPRINTFN(-1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));
	}
done:
	if (length != actlen) {
		if (req->bmRequestType & UT_READ) {
			bzero(USBD_ADD_BYTES(data, actlen), length - actlen);
		}
	}
	return;
}

static void
uchcom_ctrl_write(struct uchcom_softc *sc, uint8_t reqno,
    uint16_t value, uint16_t index)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = reqno;
	USETW(req.wValue, value);
	USETW(req.wIndex, index);
	USETW(req.wLength, 0);

	uchcom_do_request(sc, &req, NULL);
	return;
}

static void
uchcom_ctrl_read(struct uchcom_softc *sc, uint8_t reqno,
    uint16_t value, uint16_t index, void *buf, uint16_t buflen)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = reqno;
	USETW(req.wValue, value);
	USETW(req.wIndex, index);
	USETW(req.wLength, buflen);

	uchcom_do_request(sc, &req, buf);
	return;
}

static void
uchcom_write_reg(struct uchcom_softc *sc,
    uint8_t reg1, uint8_t val1, uint8_t reg2, uint8_t val2)
{
	DPRINTFN(0, "0x%02X<-0x%02X, 0x%02X<-0x%02X\n",
	    (unsigned)reg1, (unsigned)val1,
	    (unsigned)reg2, (unsigned)val2);
	uchcom_ctrl_write(
	    sc, UCHCOM_REQ_WRITE_REG,
	    reg1 | ((uint16_t)reg2 << 8), val1 | ((uint16_t)val2 << 8));
	return;
}

static void
uchcom_read_reg(struct uchcom_softc *sc,
    uint8_t reg1, uint8_t *rval1, uint8_t reg2, uint8_t *rval2)
{
	uint8_t buf[UCHCOM_INPUT_BUF_SIZE];

	uchcom_ctrl_read(
	    sc, UCHCOM_REQ_READ_REG,
	    reg1 | ((uint16_t)reg2 << 8), 0, buf, sizeof(buf));

	DPRINTFN(0, "0x%02X->0x%02X, 0x%02X->0x%02X\n",
	    (unsigned)reg1, (unsigned)buf[0],
	    (unsigned)reg2, (unsigned)buf[1]);

	if (rval1)
		*rval1 = buf[0];
	if (rval2)
		*rval2 = buf[1];

	return;
}

static void
uchcom_get_version(struct uchcom_softc *sc, uint8_t *rver)
{
	uint8_t buf[UCHCOM_INPUT_BUF_SIZE];

	uchcom_ctrl_read(
	    sc, UCHCOM_REQ_GET_VERSION, 0, 0, buf, sizeof(buf));

	if (rver)
		*rver = buf[0];

	return;
}

static void
uchcom_get_status(struct uchcom_softc *sc, uint8_t *rval)
{
	uchcom_read_reg(sc, UCHCOM_REG_STAT1, rval, UCHCOM_REG_STAT2, NULL);
	return;
}

static void
uchcom_set_dtrrts_10(struct uchcom_softc *sc, uint8_t val)
{
	uchcom_write_reg(sc, UCHCOM_REG_STAT1, val, UCHCOM_REG_STAT1, val);
	return;
}

static void
uchcom_set_dtrrts_20(struct uchcom_softc *sc, uint8_t val)
{
	uchcom_ctrl_write(sc, UCHCOM_REQ_SET_DTRRTS, val, 0);
	return;
}


/* ----------------------------------------------------------------------
 * middle layer
 */

static void
uchcom_update_version(struct uchcom_softc *sc)
{
	uchcom_get_version(sc, &sc->sc_version);
	return;
}

static void
uchcom_convert_status(struct uchcom_softc *sc, uint8_t cur)
{
	sc->sc_dtr = !(cur & UCHCOM_DTR_MASK);
	sc->sc_rts = !(cur & UCHCOM_RTS_MASK);

	cur = ~cur & 0x0F;
	sc->sc_msr = (cur << 4) | ((sc->sc_msr >> 4) ^ cur);
}

static void
uchcom_update_status(struct uchcom_softc *sc)
{
	uint8_t cur;

	uchcom_get_status(sc, &cur);
	uchcom_convert_status(sc, cur);
	return;
}


static void
uchcom_set_dtrrts(struct uchcom_softc *sc)
{
	uint8_t val = 0;

	if (sc->sc_dtr)
		val |= UCHCOM_DTR_MASK;
	if (sc->sc_rts)
		val |= UCHCOM_RTS_MASK;

	if (sc->sc_version < UCHCOM_VER_20)
		uchcom_set_dtrrts_10(sc, ~val);
	else
		uchcom_set_dtrrts_20(sc, ~val);

	return;
}

static void
uchcom_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uchcom_softc *sc = ucom->sc_parent;
	uint8_t brk1;
	uint8_t brk2;

	uchcom_read_reg(sc, UCHCOM_REG_BREAK1, &brk1, UCHCOM_REG_BREAK2, &brk2);
	if (onoff) {
		/* on - clear bits */
		brk1 &= ~UCHCOM_BRK1_MASK;
		brk2 &= ~UCHCOM_BRK2_MASK;
	} else {
		/* off - set bits */
		brk1 |= UCHCOM_BRK1_MASK;
		brk2 |= UCHCOM_BRK2_MASK;
	}
	uchcom_write_reg(sc, UCHCOM_REG_BREAK1, brk1, UCHCOM_REG_BREAK2, brk2);

	return;
}

static int
uchcom_calc_divider_settings(struct uchcom_divider *dp, uint32_t rate)
{
	const struct uchcom_divider_record *rp;
	uint32_t div;
	uint32_t rem;
	uint32_t mod;
	uint8_t i;

	/* find record */
	for (i = 0; i != NUM_DIVIDERS; i++) {
		if (dividers[i].dvr_high >= rate &&
		    dividers[i].dvr_low <= rate) {
			rp = &dividers[i];
			goto found;
		}
	}
	return (-1);

found:
	dp->dv_prescaler = rp->dvr_divider.dv_prescaler;
	if (rp->dvr_base_clock == UCHCOM_BASE_UNKNOWN)
		dp->dv_div = rp->dvr_divider.dv_div;
	else {
		div = rp->dvr_base_clock / rate;
		rem = rp->dvr_base_clock % rate;
		if (div == 0 || div >= 0xFF)
			return (-1);
		if ((rem << 1) >= rate)
			div += 1;
		dp->dv_div = (uint8_t)-div;
	}

	mod = UCHCOM_BPS_MOD_BASE / rate + UCHCOM_BPS_MOD_BASE_OFS;
	mod = mod + mod / 2;

	dp->dv_mod = mod / 0x100;

	return (0);
}

static void
uchcom_set_dte_rate(struct uchcom_softc *sc, uint32_t rate)
{
	struct uchcom_divider dv;

	if (uchcom_calc_divider_settings(&dv, rate))
		return;

	uchcom_write_reg(sc,
	    UCHCOM_REG_BPS_PRE, dv.dv_prescaler,
	    UCHCOM_REG_BPS_DIV, dv.dv_div);
	uchcom_write_reg(sc,
	    UCHCOM_REG_BPS_MOD, dv.dv_mod,
	    UCHCOM_REG_BPS_PAD, 0);
	return;
}

static void
uchcom_set_line_control(struct uchcom_softc *sc, tcflag_t cflag)
{
	uint8_t lcr1 = 0;
	uint8_t lcr2 = 0;

	uchcom_read_reg(sc, UCHCOM_REG_LCR1, &lcr1, UCHCOM_REG_LCR2, &lcr2);

	lcr1 &= ~UCHCOM_LCR1_MASK;
	lcr2 &= ~UCHCOM_LCR2_MASK;

	/*
	 * XXX: it is difficult to handle the line control appropriately:
	 *   - CS8, !CSTOPB and any parity mode seems ok, but
	 *   - the chip doesn't have the function to calculate parity
	 *     in !CS8 mode.
	 *   - it is unclear that the chip supports CS5,6 mode.
	 *   - it is unclear how to handle stop bits.
	 */

	if (cflag & PARENB) {
		lcr1 |= UCHCOM_LCR1_PARENB;
		if (cflag & PARODD)
			lcr2 |= UCHCOM_LCR2_PARODD;
		else
			lcr2 |= UCHCOM_LCR2_PAREVEN;
	}
	uchcom_write_reg(sc, UCHCOM_REG_LCR1, lcr1, UCHCOM_REG_LCR2, lcr2);

	return;
}

static void
uchcom_clear_chip(struct uchcom_softc *sc)
{
	DPRINTFN(0, "\n");
	uchcom_ctrl_write(sc, UCHCOM_REQ_RESET, 0, 0);
	return;
}

static void
uchcom_reset_chip(struct uchcom_softc *sc)
{
	uint16_t val;
	uint16_t idx;
	uint8_t lcr1;
	uint8_t lcr2;
	uint8_t pre;
	uint8_t div;
	uint8_t mod;

	uchcom_read_reg(sc, UCHCOM_REG_LCR1, &lcr1, UCHCOM_REG_LCR2, &lcr2);
	uchcom_read_reg(sc, UCHCOM_REG_BPS_PRE, &pre, UCHCOM_REG_BPS_DIV, &div);
	uchcom_read_reg(sc, UCHCOM_REG_BPS_MOD, &mod, UCHCOM_REG_BPS_PAD, NULL);

	val = 0;
	idx = 0;
	val |= (uint16_t)(lcr1 & 0xF0) << 8;
	val |= 0x01;
	val |= (uint16_t)(lcr2 & 0x0F) << 8;
	val |= 0x02;
	idx |= pre & 0x07;
	val |= 0x04;
	idx |= (uint16_t)div << 8;
	val |= 0x08;
	idx |= mod & 0xF8;
	val |= 0x10;

	DPRINTFN(0, "reset v=0x%04X, i=0x%04X\n", val, idx);

	uchcom_ctrl_write(sc, UCHCOM_REQ_RESET, val, idx);

	return;
}

/* ----------------------------------------------------------------------
 * methods for ucom
 */
static void
uchcom_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr)
{
	struct uchcom_softc *sc = ucom->sc_parent;

	DPRINTFN(0, "\n");

	*lsr = sc->sc_lsr;
	*msr = sc->sc_msr;
	return;
}

static int
uchcom_ioctl(struct ucom_softc *ucom, uint32_t cmd, caddr_t data, int flag,
    struct thread *td)
{
	return (ENOTTY);
}

static void
uchcom_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uchcom_softc *sc = ucom->sc_parent;

	DPRINTFN(0, "onoff = %d\n", onoff);

	sc->sc_dtr = onoff;
	uchcom_set_dtrrts(sc);
	return;
}

static void
uchcom_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uchcom_softc *sc = ucom->sc_parent;

	DPRINTFN(0, "onoff = %d\n", onoff);

	sc->sc_rts = onoff;
	uchcom_set_dtrrts(sc);
	return;
}

static int
uchcom_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uchcom_divider dv;

	switch (t->c_cflag & CSIZE) {
	case CS5:
	case CS6:
	case CS7:
		return (EIO);
	default:
		break;
	}

	if (uchcom_calc_divider_settings(&dv, t->c_ospeed)) {
		return (EIO);
	}
	return (0);			/* success */
}

static void
uchcom_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct uchcom_softc *sc = ucom->sc_parent;

	uchcom_set_line_control(sc, t->c_cflag);
	uchcom_set_dte_rate(sc, t->c_ospeed);
	return;
}

static void
uchcom_start_read(struct ucom_softc *ucom)
{
	struct uchcom_softc *sc = ucom->sc_parent;

	/* start interrupt endpoint */
	usbd_transfer_start(sc->sc_xfer[4]);

	/* start read endpoint */
	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
uchcom_stop_read(struct ucom_softc *ucom)
{
	struct uchcom_softc *sc = ucom->sc_parent;

	/* stop interrupt endpoint */
	usbd_transfer_stop(sc->sc_xfer[4]);

	/* stop read endpoint */
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
uchcom_start_write(struct ucom_softc *ucom)
{
	struct uchcom_softc *sc = ucom->sc_parent;

	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
uchcom_stop_write(struct ucom_softc *ucom)
{
	struct uchcom_softc *sc = ucom->sc_parent;

	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

/* ----------------------------------------------------------------------
 * callback when the modem status is changed.
 */
static void
uchcom_intr_callback(struct usbd_xfer *xfer)
{
	struct uchcom_softc *sc = xfer->priv_sc;
	uint8_t buf[UCHCOM_INTR_LEAST];

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		DPRINTFN(0, "actlen = %u\n", xfer->actlen);

		if (xfer->actlen >= UCHCOM_INTR_LEAST) {
			usbd_copy_out(xfer->frbuffers, 0, buf,
			    UCHCOM_INTR_LEAST);

			DPRINTFN(0, "data = 0x%02X 0x%02X 0x%02X 0x%02X\n",
			    (unsigned)buf[0], (unsigned)buf[1],
			    (unsigned)buf[2], (unsigned)buf[3]);

			uchcom_convert_status(sc, buf[UCHCOM_INTR_STAT1]);
			ucom_status_change(&sc->sc_ucom);
		}
	case USBD_ST_SETUP:
		if (sc->sc_flag & UCHCOM_FLAG_INTR_STALL) {
			usbd_transfer_start(sc->sc_xfer[5]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		break;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= UCHCOM_FLAG_INTR_STALL;
			usbd_transfer_start(sc->sc_xfer[5]);
		}
		break;
	}
	return;
}

static void
uchcom_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uchcom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTFN(0, "stall cleared\n");
		sc->sc_flag &= ~UCHCOM_FLAG_INTR_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uchcom_write_callback(struct usbd_xfer *xfer)
{
	struct uchcom_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
	case USBD_ST_TRANSFERRED:
		if (sc->sc_flag & UCHCOM_FLAG_WRITE_STALL) {
			usbd_transfer_start(sc->sc_xfer[2]);
			return;
		}
		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers, 0,
		    UCHCOM_BULK_BUF_SIZE, &actlen)) {

			DPRINTFN(0, "actlen = %d\n", actlen);

			xfer->frlengths[0] = actlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= UCHCOM_FLAG_WRITE_STALL;
			usbd_transfer_start(sc->sc_xfer[2]);
		}
		return;

	}
}

static void
uchcom_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uchcom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTFN(0, "stall cleared\n");
		sc->sc_flag &= ~UCHCOM_FLAG_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uchcom_read_callback(struct usbd_xfer *xfer)
{
	struct uchcom_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		ucom_put_data(&(sc->sc_ucom), xfer->frbuffers, 0, xfer->actlen);

	case USBD_ST_SETUP:
		if (sc->sc_flag & UCHCOM_FLAG_READ_STALL) {
			usbd_transfer_start(sc->sc_xfer[3]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flag |= UCHCOM_FLAG_READ_STALL;
			usbd_transfer_start(sc->sc_xfer[3]);
		}
		return;

	}
}

static void
uchcom_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uchcom_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTFN(0, "stall cleared\n");
		sc->sc_flag &= ~UCHCOM_FLAG_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static device_method_t uchcom_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, uchcom_probe),
	DEVMETHOD(device_attach, uchcom_attach),
	DEVMETHOD(device_detach, uchcom_detach),

	{0, 0}
};

static driver_t uchcom_driver = {
	"ucom",
	uchcom_methods,
	sizeof(struct uchcom_softc)
};

static devclass_t uchcom_devclass;

DRIVER_MODULE(uchcom, uhub, uchcom_driver, uchcom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uchcom, usb, 1, 1, 1);
MODULE_DEPEND(uchcom, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);
