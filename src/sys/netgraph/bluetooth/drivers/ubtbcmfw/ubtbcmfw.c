/*
 * ubtbcmfw.c
 */

/*-
 * Copyright (c) 2003 Maksim Yevmenkin <m_evmenkin@yahoo.com>
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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: ubtbcmfw.c,v 1.3 2003/10/10 19:15:08 max Exp $
 * $FreeBSD: src/sys/netgraph/bluetooth/drivers/ubtbcmfw/ubtbcmfw.c,v 1.10 2005/01/07 01:45:42 imp Exp $
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/fcntl.h>
#include <sys/ioccom.h>
#include <sys/syslog.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

/*
 * Download firmware to BCM2033.
 */

#define UBTBCMFW_CONFIG_NO	1	/* Config number */
#define UBTBCMFW_IFACE_IDX	0 	/* Control interface */
#define UBTBCMFW_T_MAX		4	/* units */

struct ubtbcmfw_softc {
	struct usb_cdev		sc_cdev;
	struct mtx		sc_mtx;

	device_t		sc_dev;
	struct usbd_device	*sc_udev;
	struct usbd_xfer	*sc_xfer[UBTBCMFW_T_MAX];

	u_int8_t		sc_flags;
#define UBTBCMFW_FLAG_WRITE_STALL 0x01
#define UBTBCMFW_FLAG_READ_STALL  0x02
};

#define UBTBCMFW_BSIZE		1024
#define UBTBCMFW_IFQ_MAXLEN	2

/* prototypes */

static device_probe_t ubtbcmfw_probe;
static device_attach_t ubtbcmfw_attach;
static device_detach_t ubtbcmfw_detach;

static void
ubtbcmfw_write_callback(struct usbd_xfer *xfer);

static void
ubtbcmfw_write_clear_stall_callback(struct usbd_xfer *xfer);

static void
ubtbcmfw_read_callback(struct usbd_xfer *xfer);

static void
ubtbcmfw_read_clear_stall_callback(struct usbd_xfer *xfer);

static void
ubtbcmfw_start_read(struct usb_cdev *cdev);

static void
ubtbcmfw_stop_read(struct usb_cdev *cdev);

static void
ubtbcmfw_start_write(struct usb_cdev *cdev);

static void
ubtbcmfw_stop_write(struct usb_cdev *cdev);

static int32_t
ubtbcmfw_open(struct usb_cdev *cdev, int32_t fflags,
	      int32_t devtype, struct thread *td);
static int32_t
ubtbcmfw_ioctl(struct usb_cdev *cdev, u_long cmd, caddr_t data, 
	       int32_t fflags, struct thread *td);

static const struct usbd_config ubtbcmfw_config[UBTBCMFW_T_MAX] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = 0x02, /* fixed */
      .direction = UE_DIR_OUT,
      .bufsize   = UBTBCMFW_BSIZE,
      .flags     = 0,
      .callback  = &ubtbcmfw_write_callback,
    },

    [1] = {
      .type      = UE_INTERRUPT,
      .endpoint  = 0x01, /* fixed */
      .direction = UE_DIR_IN,
      .bufsize   = UBTBCMFW_BSIZE,
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &ubtbcmfw_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &ubtbcmfw_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &ubtbcmfw_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

/*
 * Module
 */

static devclass_t ubtbcmfw_devclass;

static device_method_t ubtbcmfw_methods[] = {
    DEVMETHOD(device_probe, ubtbcmfw_probe),
    DEVMETHOD(device_attach, ubtbcmfw_attach),
    DEVMETHOD(device_detach, ubtbcmfw_detach),
    { 0, 0 }
};

static driver_t ubtbcmfw_driver = {
    .name    = "ubtbcmfw",
    .methods = ubtbcmfw_methods,
    .size    = sizeof(struct ubtbcmfw_softc),
};

DRIVER_MODULE(ubtbcmfw, uhub, ubtbcmfw_driver, ubtbcmfw_devclass, 
	      usbd_driver_load, 0);
MODULE_DEPEND(ubtbcmfw, usb, 1, 1, 1);

/*
 * Probe for a USB Bluetooth device
 */

static int
ubtbcmfw_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->iface != NULL)
		return (UMATCH_NONE);

	/* Match the boot device. */
	if (uaa->vendor == USB_VENDOR_BROADCOM &&
	    uaa->product == USB_PRODUCT_BROADCOM_BCM2033)
		return (UMATCH_VENDOR_PRODUCT);

	return (UMATCH_NONE);
}

/*
 * Attach the device
 */

static int
ubtbcmfw_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ubtbcmfw_softc *sc = device_get_softc(dev);
	int32_t err;
	const char *p_buf[4];
	char buf_1[32];
	char buf_2[32];
	char buf_3[32];

	if (sc == NULL) {
	    return ENOMEM;
	}

	sc->sc_dev = dev;
	sc->sc_udev = uaa->device;

	usbd_set_desc(dev, uaa->device);

	mtx_init(&(sc->sc_mtx), "ubtbcmfw lock", NULL, MTX_DEF|MTX_RECURSE);

	err = usbd_set_config_no(sc->sc_udev, UBTBCMFW_CONFIG_NO, 1);
	if (err) {
		device_printf(dev, "setting config no failed, err=%s\n",
			      usbd_errstr(err));
		goto detach;;
	}

	err = usbd_transfer_setup(uaa->device, UBTBCMFW_IFACE_IDX,
				  sc->sc_xfer, ubtbcmfw_config, UBTBCMFW_T_MAX,
				  sc, &(sc->sc_mtx));
	if (err) {
		device_printf(dev, "allocating USB transfers "
			      "failed, err=%s\n", usbd_errstr(err));
		goto detach;
	}

	snprintf(buf_1, sizeof(buf_1), "%s", 
		 device_get_nameunit(dev));

	snprintf(buf_2, sizeof(buf_2), "%s.1",
		 device_get_nameunit(dev));

	snprintf(buf_3, sizeof(buf_3), "%s.2",
		 device_get_nameunit(dev));

	p_buf[0] = buf_1;
	p_buf[1] = buf_2;
	p_buf[2] = buf_3;
	p_buf[3] = NULL;

	sc->sc_cdev.sc_start_read = &ubtbcmfw_start_read;
	sc->sc_cdev.sc_start_write = &ubtbcmfw_start_write;
	sc->sc_cdev.sc_stop_read = &ubtbcmfw_stop_read;
	sc->sc_cdev.sc_stop_write = &ubtbcmfw_stop_write;
	sc->sc_cdev.sc_open = &ubtbcmfw_open;
	sc->sc_cdev.sc_ioctl = &ubtbcmfw_ioctl;
	sc->sc_cdev.sc_flags |= (USB_CDEV_FLAG_FWD_SHORT|
				 USB_CDEV_FLAG_WAKEUP_RD_IMMED|
				 USB_CDEV_FLAG_WAKEUP_WR_IMMED);

	err = usb_cdev_attach(&(sc->sc_cdev), sc, &(sc->sc_mtx), p_buf,
			      UID_ROOT, GID_OPERATOR, 0644, 
			      UBTBCMFW_BSIZE, UBTBCMFW_IFQ_MAXLEN,
			      UBTBCMFW_BSIZE, UBTBCMFW_IFQ_MAXLEN);
	if (err) {
		goto detach;
	}

	return 0; /* success */

 detach:
	ubtbcmfw_detach(dev);
	return ENOMEM; /* failure */
}

/*
 * Detach the device
 */

static int
ubtbcmfw_detach(device_t dev)
{
	struct ubtbcmfw_softc *sc = device_get_softc(dev);

	usb_cdev_detach(&(sc->sc_cdev));

	usbd_transfer_unsetup(sc->sc_xfer, UBTBCMFW_T_MAX);

	mtx_destroy(&(sc->sc_mtx));

	return (0);
}

static void
ubtbcmfw_write_callback(struct usbd_xfer *xfer)
{
	struct ubtbcmfw_softc *sc = xfer->priv_sc;
	u_int32_t actlen;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:
 tr_setup:
	if (sc->sc_flags & UBTBCMFW_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    return;
	}
	if (usb_cdev_get_data(&(sc->sc_cdev), xfer->buffer, 
			      UBTBCMFW_BSIZE, &actlen, 0)) {

	    xfer->length = actlen;
	    usbd_start_hardware(xfer);
	}
	return;

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= UBTBCMFW_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}
	return;
}

static void
ubtbcmfw_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubtbcmfw_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~UBTBCMFW_FLAG_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~UBTBCMFW_FLAG_WRITE_STALL;
	usb_cdev_get_data_error(&(sc->sc_cdev));
	return;
}

static void
ubtbcmfw_read_callback(struct usbd_xfer *xfer)
{
	struct ubtbcmfw_softc *sc = xfer->priv_sc;
	struct usbd_mbuf *m;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:
	usb_cdev_put_data(&(sc->sc_cdev), xfer->buffer, xfer->actlen, 1);

 tr_setup:
	if (sc->sc_flags & UBTBCMFW_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	    return;
	}

	USBD_IF_POLL(&sc->sc_cdev.sc_rdq_free, m);

	if (m) {
	    usbd_start_hardware(xfer);
	}
	return;

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= UBTBCMFW_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;
}

static void
ubtbcmfw_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ubtbcmfw_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~UBTBCMFW_FLAG_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~UBTBCMFW_FLAG_READ_STALL;
	usb_cdev_put_data_error(&(sc->sc_cdev));
	return;
}

static void
ubtbcmfw_start_read(struct usb_cdev *cdev)
{
	struct ubtbcmfw_softc *sc = cdev->sc_priv_ptr;
	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
ubtbcmfw_stop_read(struct usb_cdev *cdev)
{
	struct ubtbcmfw_softc *sc = cdev->sc_priv_ptr;
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
ubtbcmfw_start_write(struct usb_cdev *cdev)
{
	struct ubtbcmfw_softc *sc = cdev->sc_priv_ptr;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
ubtbcmfw_stop_write(struct usb_cdev *cdev)
{
	struct ubtbcmfw_softc *sc = cdev->sc_priv_ptr;
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

static int32_t
ubtbcmfw_open(struct usb_cdev *cdev, int32_t fflags,
	      int32_t devtype, struct thread *td)
{
	struct ubtbcmfw_softc *sc = cdev->sc_priv_ptr;

	if (fflags & FWRITE) {
	    sc->sc_flags |= UBTBCMFW_FLAG_WRITE_STALL;
	}
	return 0;
}

static int32_t
ubtbcmfw_ioctl(struct usb_cdev *cdev, u_long cmd, caddr_t data, 
	       int32_t fflags, struct thread *td)
{
	struct ubtbcmfw_softc *sc = cdev->sc_priv_ptr;
	int32_t error = 0;

	switch (cmd) {
	case USB_GET_DEVICE_DESC:
		*(usb_device_descriptor_t *) data =
				*usbd_get_device_descriptor(sc->sc_udev);
		break;

	default:
		error = EINVAL;
		break;
	}

	return (error);
}
