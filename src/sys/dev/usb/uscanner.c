/*	$NetBSD: uscanner.c,v 1.30 2002/07/11 21:14:36 augustss Exp$	*/

/* Also already merged from NetBSD:
 *	$NetBSD: uscanner.c,v 1.33 2002/09/23 05:51:24 simonb Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uscanner.c,v 1.76 2006/10/07 18:09:27 flz Exp $");

/*-
 * Copyright (c) 2000 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology
 * and Nick Hibma (n_hibma@qubesoft.com).
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
#include <sys/fcntl.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

/*
 * uscanner debugging statements.
 */
#ifdef	USB_DEBUG
static int uscanner_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uscanner, CTLFLAG_RW, 0, "USB uscanner");
SYSCTL_INT(_hw_usb_uscanner, OID_AUTO, uscanner, CTLFLAG_RW, &uscanner_debug,
	   0, "uscanner debug level");

#define	DPRINTF(n, fmt, ...) do {		\
	if (uscanner_debug > (n)) {		\
	    printf("%s: " fmt, __FUNCTION__	\
		   ,## __VA_ARGS__);		\
	}					\
  } while (0)
#else
#define	DPRINTF(...)
#endif

/*
 * uscanner transfers macros definition.
 */
#define	USCANNER_BSIZE			(1 << 16)
#define	USCANNER_IFQ_MAXLEN		2
#define	USCANNER_N_TRANSFER		4

/*
 * Transfers stallings handling flags definition.
 */
#define	USCANNER_FLAG_READ_STALL	0x01
#define	USCANNER_FLAG_WRITE_STALL	0x02

/*
 * uscanner_info flags definition.
 */
#define	USCANNER_FLAG_KEEP_OPEN		0x04

 /*
 * uscanner driver specific structures.
 */
struct uscanner_info {
	struct usb_devno	devno;
	uint8_t			flags;
};

struct uscanner_softc {
	struct usb_cdev		sc_cdev;
	struct mtx		sc_mtx;

	struct usbd_xfer *	sc_xfer[USCANNER_N_TRANSFER];

	uint8_t			sc_flags;	/* Used to prevent stalls */
};

/*
 * Prototypes for driver handling routines (sorted by use).
 */
static device_probe_t	uscanner_probe;
static device_attach_t	uscanner_attach;
static device_detach_t	uscanner_detach;

/*
 * Prototypes for xfer transfer callbacks.
 */
static usbd_callback_t uscanner_read_callback;
static usbd_callback_t uscanner_read_clear_stall_callback;
static usbd_callback_t uscanner_write_callback;
static usbd_callback_t uscanner_write_clear_stall_callback;

/*
 * Prototypes for the character device handling routines.
 */
static int32_t	uscanner_open(struct usb_cdev *cdev, int32_t fflags,
			      int32_t devtype, struct thread *td);
static void	uscanner_start_read(struct usb_cdev *cdev);
static void	uscanner_start_write(struct usb_cdev *cdev);
static void	uscanner_stop_read(struct usb_cdev *cdev);
static void	uscanner_stop_write(struct usb_cdev *cdev);

/*
 * xfer transfers array.  Resolve-stalling callbacks are marked as control
 * transfers.
 */
static const struct usbd_config uscanner_config [USCANNER_N_TRANSFER] = {
    [0] = { 
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = USCANNER_BSIZE,
      .flags     = 0,
      .callback  = &uscanner_write_callback,
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = USCANNER_BSIZE,
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uscanner_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &uscanner_write_clear_stall_callback,
      .timeout   = 1000,
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00,
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &uscanner_read_clear_stall_callback,
      .timeout   = 1000,
    },
};

static devclass_t uscanner_devclass;

static device_method_t uscanner_methods[] = {
    DEVMETHOD(device_probe, uscanner_probe),
    DEVMETHOD(device_attach, uscanner_attach),
    DEVMETHOD(device_detach, uscanner_detach),
    { 0, 0 }
};

static driver_t uscanner_driver = {
    .name    = "uscanner",
    .methods = uscanner_methods,
    .size    = sizeof(struct uscanner_softc),
};

DRIVER_MODULE(uscanner, uhub, uscanner_driver, uscanner_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uscanner, usb, 1, 1, 1);

/*
 * USB scanners probing array.  It determines flags too.
 */
static const struct uscanner_info uscanner_devs[] = {
		/* Acer */
	{ { USB_VENDOR_ACERP, USB_PRODUCT_ACERP_ACERSCAN_320U }, 0 },
	{ { USB_VENDOR_ACERP, USB_PRODUCT_ACERP_ACERSCAN_640U }, 0 },
	{ { USB_VENDOR_ACERP, USB_PRODUCT_ACERP_ACERSCAN_640BT }, 0 },
	{ { USB_VENDOR_ACERP, USB_PRODUCT_ACERP_ACERSCAN_620U }, 0 },
	{ { USB_VENDOR_ACERP, USB_PRODUCT_ACERP_ACERSCAN_1240U }, 0 },
	{ { USB_VENDOR_ACERP, USB_PRODUCT_ACERP_ACERSCAN_C310U }, 0 },
		/* AGFA */
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCAN1236U }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCAN1212U }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCAN1212U2 }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCANTOUCH }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCANE40 }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCANE50 }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCANE20 }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCANE25 }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCANE26 }, 0 },
	{ { USB_VENDOR_AGFA, USB_PRODUCT_AGFA_SNAPSCANE52 }, 0 },
		/* Avision */
	{ { USB_VENDOR_AVISION, USB_PRODUCT_AVISION_1200U }, 0 },
		/* Canon */
	{ { USB_VENDOR_CANON, USB_PRODUCT_CANON_N656U }, 0 },
	{ { USB_VENDOR_CANON, USB_PRODUCT_CANON_N676U }, 0 },
	{ { USB_VENDOR_CANON, USB_PRODUCT_CANON_N1220U }, 0 },
	{ { USB_VENDOR_CANON, USB_PRODUCT_CANON_D660U }, 0 },
	{ { USB_VENDOR_CANON, USB_PRODUCT_CANON_N1240U }, 0 },
	{ { USB_VENDOR_CANON, USB_PRODUCT_CANON_LIDE25 }, 0 },
		/* Epson */
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_636 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_610 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1200 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1240 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1250 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1270 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1600 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1640 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_640U }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1650 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1660 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1670 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_1260 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_RX425 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_3200 }, USCANNER_FLAG_KEEP_OPEN },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_GT9700F }, USCANNER_FLAG_KEEP_OPEN },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_GT9300UF }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_2480 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_3500 }, USCANNER_FLAG_KEEP_OPEN },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_3590 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_4200 }, 0 },
	{ { USB_VENDOR_EPSON, USB_PRODUCT_EPSON_4990 }, 0 },
		/* HP */
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_2200C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_3300C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_3400CSE }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_4100C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_4200C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_4300C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_4670V }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_S20 }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_5200C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_5300C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_5400C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_6200C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_6300C }, 0 },
	{ { USB_VENDOR_HP, USB_PRODUCT_HP_82x0C }, 0 },
		/* Kye */
	{ { USB_VENDOR_KYE, USB_PRODUCT_KYE_VIVIDPRO }, 0 },
		/* Microtek */
	{ { USB_VENDOR_MICROTEK, USB_PRODUCT_MICROTEK_X6U }, 0 },
	{ { USB_VENDOR_MICROTEK, USB_PRODUCT_MICROTEK_336CX }, 0 },
	{ { USB_VENDOR_MICROTEK, USB_PRODUCT_MICROTEK_336CX2 }, 0 },
	{ { USB_VENDOR_MICROTEK, USB_PRODUCT_MICROTEK_C6 }, 0 },
	{ { USB_VENDOR_MICROTEK, USB_PRODUCT_MICROTEK_V6USL }, 0 },
	{ { USB_VENDOR_MICROTEK, USB_PRODUCT_MICROTEK_V6USL2 }, 0 },
	{ { USB_VENDOR_MICROTEK, USB_PRODUCT_MICROTEK_V6UL }, 0 },
		/* Minolta */
	{ { USB_VENDOR_MINOLTA, USB_PRODUCT_MINOLTA_5400 }, 0 },
		/* Mustek */
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_1200CU }, 0 },
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_BEARPAW1200F }, 0 },
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_BEARPAW1200TA }, 0 },
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_600USB }, 0 },
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_600CU }, 0 },
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_1200USB }, 0 },
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_1200UB }, 0 },
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_1200USBPLUS }, 0 },
	{ { USB_VENDOR_MUSTEK, USB_PRODUCT_MUSTEK_1200CUPLUS }, 0 },
		/* National */
	{ { USB_VENDOR_NATIONAL, USB_PRODUCT_NATIONAL_BEARPAW1200 }, 0 },
	{ { USB_VENDOR_NATIONAL, USB_PRODUCT_NATIONAL_BEARPAW2400 }, 0 },
		/* Nikon */
	{ { USB_VENDOR_NIKON, USB_PRODUCT_NIKON_LS40 }, 0 },
		/* Primax */
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_G2X300 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_G2E300 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_G2300 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_G2E3002 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_9600 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_600U }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_6200 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_19200 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_1200U }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_G600 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_636I }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_G2600 }, 0 },
	{ { USB_VENDOR_PRIMAX, USB_PRODUCT_PRIMAX_G2E600 }, 0 },
		/* Scanlogic */
	{ { USB_VENDOR_SCANLOGIC, USB_PRODUCT_SCANLOGIC_336CX }, 0 },
		/* Ultima */
	{ { USB_VENDOR_ULTIMA, USB_PRODUCT_ULTIMA_1200UBPLUS }, 0 },
		/* UMAX */
	{ { USB_VENDOR_UMAX, USB_PRODUCT_UMAX_ASTRA1220U }, 0 },
	{ { USB_VENDOR_UMAX, USB_PRODUCT_UMAX_ASTRA1236U }, 0 },
	{ { USB_VENDOR_UMAX, USB_PRODUCT_UMAX_ASTRA2000U }, 0 },
	{ { USB_VENDOR_UMAX, USB_PRODUCT_UMAX_ASTRA2100U }, 0 },
	{ { USB_VENDOR_UMAX, USB_PRODUCT_UMAX_ASTRA2200U }, 0 },
	{ { USB_VENDOR_UMAX, USB_PRODUCT_UMAX_ASTRA3400 }, 0 },
		/* Visioneer */
	{ { USB_VENDOR_VISIONEER, USB_PRODUCT_VISIONEER_3000 }, 0 },
	{ { USB_VENDOR_VISIONEER, USB_PRODUCT_VISIONEER_5300 }, 0 },
	{ { USB_VENDOR_VISIONEER, USB_PRODUCT_VISIONEER_7600 }, 0 },
	{ { USB_VENDOR_VISIONEER, USB_PRODUCT_VISIONEER_6100 }, 0 },
	{ { USB_VENDOR_VISIONEER, USB_PRODUCT_VISIONEER_6200 }, 0 },
	{ { USB_VENDOR_VISIONEER, USB_PRODUCT_VISIONEER_8100 }, 0 },
	{ { USB_VENDOR_VISIONEER, USB_PRODUCT_VISIONEER_8600 }, 0 }
};

/*
 * It returns vendor and product ids.
 */
static inline const struct uscanner_info *
uscanner_lookup(uint16_t v, uint16_t p)
{
	return ((const struct uscanner_info *)usb_lookup(uscanner_devs, v, p));
}

/*
 * uscanner device probing method.
 */
static int
uscanner_probe(device_t dev)
{
	struct usb_attach_arg *uaa;

	DPRINTF(10, "\n");

	uaa = device_get_ivars(dev);
	if (uaa->iface != NULL) {
		return (UMATCH_NONE);
	}

	return ((uscanner_lookup(uaa->vendor, uaa->product) != NULL) ?
	    UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

/*
 * uscanner device attaching method.
 */
static int
uscanner_attach(device_t dev)
{
	struct usb_attach_arg *uaa;
	struct uscanner_softc *sc;
	struct usbd_device *udev;
	const char *p_buf[2];
	char buf[32];
	int32_t unit;
	int error;

	uaa = device_get_ivars(dev);
	sc = device_get_softc(dev);
	unit = device_get_unit(dev);

	/*
	 * A first path softc structure filling.  sc_cdev and
	 * sc_xfer are filled later with appropriate functions.
	 */
	sc->sc_flags = uscanner_lookup(uaa->vendor, uaa->product)->flags;
	mtx_init(&(sc->sc_mtx), "uscanner mutex", NULL, MTX_DEF | MTX_RECURSE);

	/*
	 * Announce the device:
	 */
	usbd_set_desc(dev, uaa->device);

	/*
	 * Assume only one interface and check for this.
	 */
	udev = uaa->device;
	if ((error = usbd_set_config_no(udev, 1, 1))) {
		device_printf(dev, "could not set config number\n");
		goto detach;
	}

	/*
	 * Setup the transfer.
	 */
	if ((error = usbd_transfer_setup(udev, uaa->iface_index, sc->sc_xfer,
	    uscanner_config, USCANNER_N_TRANSFER, sc, &(sc->sc_mtx)))) {
		device_printf(dev, "could not setup transfers, "
			      "error=%s\n", usbd_errstr(error));
		goto detach;
	}

	/*
	 * Setup the character device for USB scanner.
	 */
	snprintf(buf, sizeof(buf), "uscanner%u", unit);
	p_buf[0] = buf;
	p_buf[1] = NULL;

	sc->sc_cdev.sc_start_read = &uscanner_start_read;
	sc->sc_cdev.sc_stop_read = &uscanner_stop_read;
	sc->sc_cdev.sc_start_write = &uscanner_start_write;
	sc->sc_cdev.sc_stop_write = &uscanner_stop_write;
	sc->sc_cdev.sc_open = &uscanner_open;
	sc->sc_cdev.sc_flags |= (USB_CDEV_FLAG_FWD_SHORT|
				 USB_CDEV_FLAG_WAKEUP_RD_IMMED |
				 USB_CDEV_FLAG_WAKEUP_WR_IMMED);

	if ((error = usb_cdev_attach(&(sc->sc_cdev), sc, &(sc->sc_mtx), p_buf,
	    UID_ROOT, GID_OPERATOR, 0644, USCANNER_BSIZE, USCANNER_IFQ_MAXLEN,
	    USCANNER_BSIZE, USCANNER_IFQ_MAXLEN))) {
		device_printf(dev, "error setting the "
			      "char device!\n");
		goto detach;
	}

	return (0);

detach:
	uscanner_detach(dev);
	return ENOMEM;
}

/*
 * uscanner device detaching method.
 */
static int
uscanner_detach(device_t dev)
{
	struct uscanner_softc *sc;

	sc = device_get_softc(dev);

	usb_cdev_detach(&(sc->sc_cdev));
	usbd_transfer_unsetup(sc->sc_xfer, USCANNER_N_TRANSFER);
	mtx_destroy(&(sc->sc_mtx));

	return (0);
}

/*
 * Reading callback.  Implemented as an "in" bulk transfer.
 */
static void
uscanner_read_callback(struct usbd_xfer *xfer)
{
	struct uscanner_softc *sc;
	struct usbd_mbuf *m;

	sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

tr_transferred:
	usb_cdev_put_data(&(sc->sc_cdev), xfer->buffer, xfer->actlen, 1);

tr_setup:
	/*
	 * If reading is in stall, just jump to clear stall callback and
	 * solve the situation.
	 */
	if (sc->sc_flags & USCANNER_FLAG_READ_STALL) {
		usbd_transfer_start(sc->sc_xfer[3]);
		return;
	}

	USBD_IF_POLL(&(sc->sc_cdev).sc_rdq_free, m);
	if (m) {
		usbd_start_hardware(xfer);
	}
	return;

tr_error:
	if (xfer->error != USBD_CANCELLED) {
		sc->sc_flags |= USCANNER_FLAG_READ_STALL;
		usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;
}

/*
 * Removing stall on reading callback.
 */
static void
uscanner_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uscanner_softc *sc;
	struct usbd_xfer *xfer_other;

	sc = xfer->priv_sc;
	xfer_other = sc->sc_xfer[1];

	USBD_CHECK_STATUS(xfer);

tr_setup:
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);
	sc->sc_flags &= ~USCANNER_FLAG_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

tr_error:
	sc->sc_flags &= ~USCANNER_FLAG_READ_STALL;
	usb_cdev_put_data_error(&(sc->sc_cdev));
	return;
}

/*
 * Writing callback.  Implemented as an "out" bulk transfer.
 */
static void
uscanner_write_callback(struct usbd_xfer *xfer)
{
	struct uscanner_softc *sc;
	uint32_t actlen;

	sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

tr_setup:
tr_transferred:
	/*
	 * If writing is in stall, just jump to clear stall callback and
	 * solve the situation.
	 */
	if (sc->sc_flags & USCANNER_FLAG_WRITE_STALL) {
		usbd_transfer_start(sc->sc_xfer[2]);
		return;
	}

	/*
	 * Write datas, setup and perform hardware transfer.
	 */
	if (usb_cdev_get_data(&(sc->sc_cdev), xfer->buffer, USCANNER_BSIZE,
	    &actlen, 0)) {
		xfer->length = actlen;
		usbd_start_hardware(xfer);
	}
	return;

tr_error:
	if (xfer->error != USBD_CANCELLED) {
		sc->sc_flags |= USCANNER_FLAG_WRITE_STALL;
		usbd_transfer_start(sc->sc_xfer[2]);
	}
	return;
}

/*
 * Removing stall on writing callback.
 */
static void
uscanner_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uscanner_softc *sc;
	struct usbd_xfer *xfer_other;

	sc = xfer->priv_sc;
	xfer_other = sc->sc_xfer[0];

	USBD_CHECK_STATUS(xfer);

tr_setup:
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);
	sc->sc_flags &= ~USCANNER_FLAG_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

tr_error:
	sc->sc_flags &= ~USCANNER_FLAG_WRITE_STALL;
	usb_cdev_get_data_error(&(sc->sc_cdev));
	return;
}

/*
 * uscanner character device opening method.
 */
static int32_t
uscanner_open(struct usb_cdev *cdev, int32_t fflags, int32_t devtype,
	      struct thread *td)
{
	struct uscanner_softc *sc;

	sc = cdev->sc_priv_ptr;

	if (!(sc->sc_flags & USCANNER_FLAG_KEEP_OPEN)) {
	    if (fflags & FWRITE) {
		sc->sc_flags |= USCANNER_FLAG_WRITE_STALL;
	    }

	    if (fflags & FREAD) {
		sc->sc_flags |= USCANNER_FLAG_READ_STALL;
	    }
	}
	return (0);
}

/*
 * uscanner character device start reading method.
 */
static void
uscanner_start_read(struct usb_cdev *cdev)
{
	struct uscanner_softc *sc;

	sc = cdev->sc_priv_ptr;
	usbd_transfer_start(sc->sc_xfer[1]);
}

/*
 * uscanner character device start writing method.
 */
static void
uscanner_start_write(struct usb_cdev *cdev)
{
	struct uscanner_softc *sc;

	sc = cdev->sc_priv_ptr;
	usbd_transfer_start(sc->sc_xfer[0]);
}

/*
 * uscanner character device stop reading method.
 */
static void
uscanner_stop_read(struct usb_cdev *cdev)
{
	struct uscanner_softc *sc;

	sc = cdev->sc_priv_ptr;
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
}

/*
 * uscanner character device stop writing method.
 */
static void
uscanner_stop_write(struct usb_cdev *cdev)
{
	struct uscanner_softc *sc;

	sc = cdev->sc_priv_ptr;
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
}
