/*-
 * Copyright (c) 2000 Iwasa Kazmi
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This code is based on ugen.c and ulpt.c developed by Lennart Augustsson.
 * This code includes software developed by the NetBSD Foundation, Inc. and
 * its contributors.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/urio.c,v 1.48 2007/06/21 14:42:34 imp Exp $");


/*
 * 2000/3/24  added NetBSD/OpenBSD support (from Alex Nemirovsky)
 * 2000/3/07  use two bulk-pipe handles for read and write (Dirk)
 * 2000/3/06  change major number(143), and copyright header
 *            some fix for 4.0 (Dirk)
 * 2000/3/05  codes for FreeBSD 4.x - CURRENT (Thanks to Dirk-Willem van Gulik)
 * 2000/3/01  remove retry code from urioioctl()
 *            change method of bulk transfer (no interrupt)
 * 2000/2/28  small fixes for new rio_usb.h
 * 2000/2/24  first version.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/fcntl.h>
#include <sys/syslog.h>
#include <sys/uio.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

#include <dev/usb/rio500_usb.h>

#ifdef USB_DEBUG
#define	DPRINTF(sc,n,fmt,...) do {		\
    if (urio_debug > (n))			\
        printf("%s:%s: " fmt, (sc)->sc_name,	\
	       __FUNCTION__ ,##__VA_ARGS__);	\
} while (0)

static int urio_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, urio, CTLFLAG_RW, 0, "USB urio");
SYSCTL_INT(_hw_usb_urio, OID_AUTO, debug, CTLFLAG_RW,
    &urio_debug, 0, "urio debug level");
#else
#define	DPRINTF(...) do { } while (0)
#endif

#define	URIO_T_WR     0
#define	URIO_T_RD     1
#define	URIO_T_WR_CS  2
#define	URIO_T_RD_CS  3
#define	URIO_T_MAX    4

#define	URIO_BSIZE	(1<<12)		/* bytes */
#define	URIO_IFQ_MAXLEN      2		/* units */

struct urio_softc {
	struct usb_cdev sc_cdev;
	struct mtx sc_mtx;

	struct usbd_device *sc_udev;
	struct usbd_xfer *sc_xfer[URIO_T_MAX];

	uint8_t	sc_flags;
#define	URIO_FLAG_READ_STALL    0x01	/* read transfer stalled */
#define	URIO_FLAG_WRITE_STALL   0x02	/* write transfer stalled */

	uint8_t	sc_name[16];
};

/* prototypes */

static device_probe_t urio_probe;
static device_attach_t urio_attach;
static device_detach_t urio_detach;

static usbd_callback_t urio_write_callback;
static usbd_callback_t urio_write_clear_stall_callback;
static usbd_callback_t urio_read_callback;
static usbd_callback_t urio_read_clear_stall_callback;

static void
	urio_start_read(struct usb_cdev *cdev);

static void
	urio_stop_read(struct usb_cdev *cdev);

static void
	urio_start_write(struct usb_cdev *cdev);

static void
	urio_stop_write(struct usb_cdev *cdev);

static int32_t
urio_open(struct usb_cdev *cdev, int32_t fflags,
    int32_t devtype, struct thread *td);
static int32_t
urio_ioctl(struct usb_cdev *cdev, u_long cmd, caddr_t addr,
    int32_t fflags, struct thread *td);

static const struct usbd_config urio_config[URIO_T_MAX] = {
	[URIO_T_WR] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.mh.bufsize = URIO_BSIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &urio_write_callback,
	},

	[URIO_T_RD] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.bufsize = URIO_BSIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &urio_read_callback,
	},

	[URIO_T_WR_CS] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &urio_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[URIO_T_RD_CS] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &urio_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},
};

static devclass_t urio_devclass;

static device_method_t urio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, urio_probe),
	DEVMETHOD(device_attach, urio_attach),
	DEVMETHOD(device_detach, urio_detach),
	{0, 0}
};

static driver_t urio_driver = {
	.name = "urio",
	.methods = urio_methods,
	.size = sizeof(struct urio_softc),
};

DRIVER_MODULE(urio, uhub, urio_driver, urio_devclass, usbd_driver_load, 0);

MODULE_DEPEND(urio, usb, 1, 1, 1);

static int
urio_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_device_descriptor_t *dd;

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (!uaa->iface)
		return (UMATCH_NONE);

	dd = usbd_get_device_descriptor(uaa->device);

	if (dd &&
	    (((UGETW(dd->idVendor) == USB_VENDOR_DIAMOND) &&
	    (UGETW(dd->idProduct) == USB_PRODUCT_DIAMOND_RIO500USB)) ||
	    ((UGETW(dd->idVendor) == USB_VENDOR_DIAMOND2) &&
	    ((UGETW(dd->idProduct) == USB_PRODUCT_DIAMOND2_RIO600USB) ||
	    (UGETW(dd->idProduct) == USB_PRODUCT_DIAMOND2_RIO800USB)))))
		return (UMATCH_VENDOR_PRODUCT);
	else
		return (UMATCH_NONE);
}

static int
urio_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct urio_softc *sc = device_get_softc(dev);
	const char *p_buf[2];
	int32_t error;
	char buf_1[16];

	if (sc == NULL) {
		return (ENOMEM);
	}
	usbd_set_device_desc(dev);

	sc->sc_udev = uaa->device;

	mtx_init(&(sc->sc_mtx), "urio lock", NULL, MTX_DEF | MTX_RECURSE);

	snprintf(sc->sc_name, sizeof(sc->sc_name),
	    "%s", device_get_nameunit(dev));

	error = usbd_transfer_setup(uaa->device,
	    &(uaa->iface_index), sc->sc_xfer,
	    urio_config, URIO_T_MAX, sc, &(sc->sc_mtx));

	if (error) {
		DPRINTF(sc, 0, "error=%s\n", usbd_errstr(error));
		goto detach;
	}
	snprintf(buf_1, sizeof(buf_1), "urio%d", device_get_unit(dev));

	p_buf[0] = buf_1;
	p_buf[1] = NULL;

	sc->sc_cdev.sc_start_read = &urio_start_read;
	sc->sc_cdev.sc_start_write = &urio_start_write;
	sc->sc_cdev.sc_stop_read = &urio_stop_read;
	sc->sc_cdev.sc_stop_write = &urio_stop_write;
	sc->sc_cdev.sc_open = &urio_open;
	sc->sc_cdev.sc_ioctl = &urio_ioctl;
	sc->sc_cdev.sc_flags |= (USB_CDEV_FLAG_WAKEUP_RD_IMMED |
	    USB_CDEV_FLAG_WAKEUP_WR_IMMED);

	error = usb_cdev_attach(&(sc->sc_cdev), sc, &(sc->sc_mtx), p_buf,
	    UID_ROOT, GID_OPERATOR, 0644,
	    URIO_BSIZE, URIO_IFQ_MAXLEN,
	    URIO_BSIZE, URIO_IFQ_MAXLEN);
	if (error) {
		goto detach;
	}
	return (0);			/* success */

detach:
	urio_detach(dev);
	return (ENOMEM);		/* failure */
}

static void
urio_write_callback(struct usbd_xfer *xfer)
{
	struct urio_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
	case USBD_ST_SETUP:
		if (sc->sc_flags & URIO_FLAG_WRITE_STALL) {
			usbd_transfer_start(sc->sc_xfer[URIO_T_WR_CS]);
			return;
		}
		if (usb_cdev_get_data(&(sc->sc_cdev), xfer->frbuffers + 0, 0,
		    URIO_BSIZE, &actlen, 0)) {

			xfer->frlengths[0] = actlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			/* try to clear stall first */
			sc->sc_flags |= URIO_FLAG_WRITE_STALL;
			usbd_transfer_start(sc->sc_xfer[URIO_T_WR_CS]);
		}
		return;
	}
}

static void
urio_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct urio_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[URIO_T_WR];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~URIO_FLAG_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
urio_read_callback(struct usbd_xfer *xfer)
{
	struct urio_softc *sc = xfer->priv_sc;
	struct usbd_mbuf *m;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		usb_cdev_put_data(&(sc->sc_cdev), xfer->frbuffers + 0, 0,
		    xfer->actlen, 1);

	case USBD_ST_SETUP:
		if (sc->sc_flags & URIO_FLAG_READ_STALL) {
			usbd_transfer_start(sc->sc_xfer[URIO_T_RD_CS]);
			return;
		}
		USBD_IF_POLL(&sc->sc_cdev.sc_rdq_free, m);

		if (m) {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			/* try to clear stall first */
			sc->sc_flags |= URIO_FLAG_READ_STALL;
			usbd_transfer_start(sc->sc_xfer[URIO_T_RD_CS]);
		}
		return;
	}
}

static void
urio_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct urio_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[URIO_T_RD];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~URIO_FLAG_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
urio_start_read(struct usb_cdev *cdev)
{
	struct urio_softc *sc = cdev->sc_priv_ptr;

	usbd_transfer_start(sc->sc_xfer[URIO_T_RD]);
	return;
}

static void
urio_stop_read(struct usb_cdev *cdev)
{
	struct urio_softc *sc = cdev->sc_priv_ptr;

	usbd_transfer_stop(sc->sc_xfer[URIO_T_RD_CS]);
	usbd_transfer_stop(sc->sc_xfer[URIO_T_RD]);
	return;
}

static void
urio_start_write(struct usb_cdev *cdev)
{
	struct urio_softc *sc = cdev->sc_priv_ptr;

	usbd_transfer_start(sc->sc_xfer[URIO_T_WR]);
	return;
}

static void
urio_stop_write(struct usb_cdev *cdev)
{
	struct urio_softc *sc = cdev->sc_priv_ptr;

	usbd_transfer_stop(sc->sc_xfer[URIO_T_WR_CS]);
	usbd_transfer_stop(sc->sc_xfer[URIO_T_WR]);
	return;
}

static int32_t
urio_open(struct usb_cdev *cdev, int32_t fflags,
    int32_t devtype, struct thread *td)
{
	struct urio_softc *sc = cdev->sc_priv_ptr;

	if ((fflags & (FWRITE | FREAD)) != (FWRITE | FREAD)) {
		return (EACCES);
	}
	if (fflags & FREAD) {
		/* clear stall first */
		sc->sc_flags |= URIO_FLAG_READ_STALL;
	}
	if (fflags & FWRITE) {
		/* clear stall first */
		sc->sc_flags |= URIO_FLAG_WRITE_STALL;
	}
	return (0);			/* success */
}

static int32_t
urio_ioctl(struct usb_cdev *cdev, u_long cmd, caddr_t addr,
    int32_t fflags, struct thread *td)
{
	usb_device_request_t req;
	struct iovec iov;
	struct uio uio;

	struct urio_softc *sc = cdev->sc_priv_ptr;
	struct RioCommand *rio_cmd;
	void *ptr = 0;

	int32_t error = 0;

	uint16_t len;
	uint8_t requesttype;

	usb_cdev_unlock(cdev, fflags);

	switch (cmd) {
	case RIO_RECV_COMMAND:
		if (!(fflags & FWRITE)) {
			error = EPERM;
			goto done;
		}
		rio_cmd = (struct RioCommand *)addr;
		if (rio_cmd == NULL) {
			error = EINVAL;
			goto done;
		}
		len = rio_cmd->length;

		requesttype = rio_cmd->requesttype | UT_READ_VENDOR_DEVICE;
		DPRINTF(sc, 1, "sending command:reqtype=%0x req=%0x value=%0x index=%0x len=%0x\n",
		    requesttype, rio_cmd->request, rio_cmd->value, rio_cmd->index, len);
		break;

	case RIO_SEND_COMMAND:
		if (!(fflags & FWRITE)) {
			error = EPERM;
			goto done;
		}
		rio_cmd = (struct RioCommand *)addr;
		if (rio_cmd == NULL) {
			error = EINVAL;
			goto done;
		}
		len = rio_cmd->length;

		requesttype = rio_cmd->requesttype | UT_WRITE_VENDOR_DEVICE;
		DPRINTF(sc, 1, "sending command:reqtype=%0x req=%0x value=%0x index=%0x len=%0x\n",
		    requesttype, rio_cmd->request, rio_cmd->value, rio_cmd->index, len);
		break;

	default:
		error = EINVAL;
		goto done;
	}

	/* Send rio control message */
	req.bmRequestType = requesttype;
	req.bRequest = rio_cmd->request;
	USETW(req.wValue, rio_cmd->value);
	USETW(req.wIndex, rio_cmd->index);
	USETW(req.wLength, len);

	if (len > 32767) {
		error = EINVAL;
		goto done;
	}
	if (len != 0) {
		iov.iov_base = (caddr_t)rio_cmd->buffer;
		iov.iov_len = len;
		uio.uio_iov = &iov;
		uio.uio_iovcnt = 1;
		uio.uio_resid = len;
		uio.uio_offset = 0;
		uio.uio_segflg = UIO_USERSPACE;
		uio.uio_rw =
		    ((req.bmRequestType & UT_READ) ?
		    UIO_READ : UIO_WRITE);
		uio.uio_procp = td;
		ptr = malloc(len, M_TEMP, M_WAITOK);
		if (ptr == NULL) {
			error = ENOMEM;
			goto done;
		}
		if (uio.uio_rw == UIO_WRITE) {
			error = uiomove(ptr, len, &uio);
			if (error) {
				goto done;
			}
		}
	}
	error = usbd_do_request_flags
	    (sc->sc_udev, NULL,
	    &req, ptr, 0, NULL, USBD_DEFAULT_TIMEOUT);

	if (error == 0) {
		if (len != 0) {
			if (uio.uio_rw == UIO_READ) {
				error = uiomove(ptr, len, &uio);
			}
		}
	} else {
		error = EIO;
	}
done:
	if (ptr) {
		free(ptr, M_TEMP);
	}
	return (usb_cdev_lock(cdev, fflags, error));
}

static int
urio_detach(device_t dev)
{
	struct urio_softc *sc = device_get_softc(dev);

	DPRINTF(sc, 0, "\n");

	usb_cdev_detach(&(sc->sc_cdev));

	usbd_transfer_unsetup(sc->sc_xfer, URIO_T_MAX);

	mtx_destroy(&(sc->sc_mtx));

	return (0);
}
