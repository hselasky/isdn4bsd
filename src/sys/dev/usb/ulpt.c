/*	$NetBSD: ulpt.c,v 1.60 2003/10/04 21:19:50 augustss Exp $	*/

/*-
 * Copyright (c) 1998, 2003 The NetBSD Foundation, Inc.
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
 * Printer Class spec: http://www.usb.org/developers/data/devclass/usbprint109.PDF
 * Printer Class spec: http://www.usb.org/developers/devclass_docs/usbprint11.pdf
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/fcntl.h>
#include <sys/syslog.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>

__FBSDID("$FreeBSD: src/sys/dev/usb/ulpt.c,v 1.80 2007/06/21 14:42:34 imp Exp $");

#ifdef USB_DEBUG
#define	DPRINTF(n,fmt,...)						\
  do { if (ulpt_debug > (n)) {						\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ulpt_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, ulpt, CTLFLAG_RW, 0, "USB ulpt");
SYSCTL_INT(_hw_usb_ulpt, OID_AUTO, debug, CTLFLAG_RW,
    &ulpt_debug, 0, "ulpt debug level");
#else
#define	DPRINTF(...) do { } while (0)
#endif

#define	ULPT_BSIZE		(1<<17)	/* bytes */
#define	ULPT_IFQ_MAXLEN         2	/* units */
#define	ULPT_N_TRANSFER         6	/* units */

#define	UR_GET_DEVICE_ID        0x00
#define	UR_GET_PORT_STATUS      0x01
#define	UR_SOFT_RESET           0x02

#define	LPS_NERR		0x08	/* printer no error */
#define	LPS_SELECT		0x10	/* printer selected */
#define	LPS_NOPAPER		0x20	/* printer out of paper */
#define	LPS_INVERT      (LPS_SELECT|LPS_NERR)
#define	LPS_MASK        (LPS_SELECT|LPS_NERR|LPS_NOPAPER)

struct ulpt_softc {
	struct usb_cdev sc_cdev;
	struct mtx sc_mtx;

	device_t sc_dev;
	struct usbd_xfer *sc_xfer[ULPT_N_TRANSFER];

	uint8_t	sc_flags;
#define	ULPT_FLAG_READ_STALL    0x04	/* read transfer stalled */
#define	ULPT_FLAG_WRITE_STALL   0x08	/* write transfer stalled */
#define	ULPT_FLAG_RESETTING     0x10	/* device is resetting */

	uint8_t	sc_iface_no;
	uint8_t	sc_last_status;
	uint8_t	sc_zlps;		/* number of consequtive zero length
					 * packets received */
};

/* prototypes */

static device_probe_t ulpt_probe;
static device_attach_t ulpt_attach;
static device_detach_t ulpt_detach;

static usbd_callback_t ulpt_write_callback;
static usbd_callback_t ulpt_write_clear_stall_callback;
static usbd_callback_t ulpt_read_callback;
static usbd_callback_t ulpt_read_clear_stall_callback;
static usbd_callback_t ulpt_status_callback;
static usbd_callback_t ulpt_reset_callback;

static void
ulpt_write_callback(struct usbd_xfer *xfer)
{
	struct ulpt_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
	case USBD_ST_SETUP:
		if (sc->sc_flags & ULPT_FLAG_WRITE_STALL) {
			usbd_transfer_start(sc->sc_xfer[4]);
			return;
		}
		if (usb_cdev_get_data(&(sc->sc_cdev), xfer->frbuffers + 0, 0,
		    xfer->max_data_length, &actlen, 0)) {

			xfer->frlengths[0] = actlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_CANCELLED) {
			/* try to clear stall first */
			sc->sc_flags |= ULPT_FLAG_WRITE_STALL;
			usbd_transfer_start(sc->sc_xfer[4]);
		}
		return;
	}
}

static void
ulpt_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ulpt_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flags &= ~ULPT_FLAG_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ulpt_read_callback(struct usbd_xfer *xfer)
{
	struct ulpt_softc *sc = xfer->priv_sc;
	struct usbd_mbuf *m;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		if (xfer->actlen == 0) {

			if (sc->sc_zlps == 4) {
				/* enable BULK throttle */
				xfer->interval = 500;	/* ms */
			} else {
				sc->sc_zlps++;
			}
		} else {
			/* disable BULK throttle */

			xfer->interval = 0;
			sc->sc_zlps = 0;
		}

		usb_cdev_put_data(&(sc->sc_cdev), xfer->frbuffers + 0, 0,
		    xfer->actlen, 1);

	case USBD_ST_SETUP:
		if (sc->sc_flags & ULPT_FLAG_READ_STALL) {
			usbd_transfer_start(sc->sc_xfer[5]);
			return;
		}
		USBD_IF_POLL(&sc->sc_cdev.sc_rdq_free, m);

		if (m) {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		/* disable BULK throttle */
		xfer->interval = 0;
		sc->sc_zlps = 0;

		if (xfer->error != USBD_CANCELLED) {
			/* try to clear stall first */
			sc->sc_flags |= ULPT_FLAG_READ_STALL;
			usbd_transfer_start(sc->sc_xfer[5]);
		}
		return;
	}
}

static void
ulpt_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ulpt_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(0, "stall cleared\n");
		sc->sc_flags &= ~ULPT_FLAG_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ulpt_status_callback(struct usbd_xfer *xfer)
{
	struct ulpt_softc *sc = xfer->priv_sc;
	usb_device_request_t req;
	uint8_t cur_status;
	uint8_t new_status;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		usbd_copy_out(xfer->frbuffers + 1, 0, &cur_status, 1);

		cur_status = (cur_status ^ LPS_INVERT) & LPS_MASK;
		new_status = cur_status & ~sc->sc_last_status;
		sc->sc_last_status = cur_status;

		if (new_status & LPS_SELECT)
			log(LOG_NOTICE, "%s: offline\n",
			    device_get_nameunit(sc->sc_dev));
		else if (new_status & LPS_NOPAPER)
			log(LOG_NOTICE, "%s: out of paper\n",
			    device_get_nameunit(sc->sc_dev));
		else if (new_status & LPS_NERR)
			log(LOG_NOTICE, "%s: output error\n",
			    device_get_nameunit(sc->sc_dev));

	case USBD_ST_SETUP:
tr_setup:
		req.bmRequestType = UT_READ_CLASS_INTERFACE;
		req.bRequest = UR_GET_PORT_STATUS;
		USETW(req.wValue, 0);
		req.wIndex[0] = sc->sc_iface_no;
		req.wIndex[1] = 0;
		USETW(req.wLength, 1);

		usbd_copy_in(xfer->frbuffers + 0, 0, &req, sizeof(req));

		xfer->frlengths[0] = sizeof(req);
		xfer->frlengths[1] = 1;
		xfer->nframes = 2;
		usbd_start_hardware(xfer);

		return;

	default:			/* Error */
		DPRINTF(0, "error=%s\n", usbd_errstr(xfer->error));
		if (xfer->error != USBD_CANCELLED) {
			goto tr_setup;
		}
		return;
	}
}

static void
ulpt_reset_callback(struct usbd_xfer *xfer)
{
	struct ulpt_softc *sc = xfer->priv_sc;
	usb_device_request_t req;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
tr_transferred:
		usb_cdev_wakeup(&(sc->sc_cdev));
		return;

	case USBD_ST_SETUP:
		req.bmRequestType = UT_WRITE_CLASS_OTHER;	/* 1.0 */
		req.bRequest = UR_SOFT_RESET;

tr_setup_sub:
		USETW(req.wValue, 0);
		req.wIndex[0] = sc->sc_iface_no;
		req.wIndex[1] = 0;
		USETW(req.wLength, 0);

		usbd_copy_in(xfer->frbuffers + 0, 0, &req, sizeof(req));

		xfer->frlengths[0] = sizeof(req);
		xfer->nframes = 1;
		usbd_start_hardware(xfer);
		return;

	default:			/* Error */
		if (xfer->error == USBD_CANCELLED) {
			return;
		}
		usbd_copy_out(xfer->frbuffers + 0, 0, &req, sizeof(req));

		if (req.bmRequestType == UT_WRITE_CLASS_OTHER) {
			/*
		         * There was a mistake in the USB printer 1.0 spec that
		         * gave the request type as UT_WRITE_CLASS_OTHER; it
		         * should have been UT_WRITE_CLASS_INTERFACE.  Many
		         * printers use the old one, so try both:
		         */
			req.bmRequestType = UT_WRITE_CLASS_INTERFACE;	/* 1.1 */
			req.bRequest = UR_SOFT_RESET;

			goto tr_setup_sub;
		}
		goto tr_transferred;
	}
}

static const struct usbd_config ulpt_config[ULPT_N_TRANSFER] = {
	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.bufsize = ULPT_BSIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,.proxy_buffer = 1},
		.mh.callback = &ulpt_write_callback,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = ULPT_BSIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,.proxy_buffer = 1},
		.mh.callback = &ulpt_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(usb_device_request_t) + 1,
		.mh.callback = &ulpt_status_callback,
		.mh.timeout = 1000,	/* 1 second */
		.interval = 1000,	/* 1 second */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &ulpt_reset_callback,
		.mh.timeout = 1000,	/* 1 second */
	},

	[4] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &ulpt_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.interval = 50,		/* 50ms */
	},

	[5] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &ulpt_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.interval = 50,		/* 50ms */
	},
};

static void
ulpt_start_read(struct usb_cdev *cdev)
{
	struct ulpt_softc *sc = cdev->sc_priv_ptr;

	usbd_transfer_start(sc->sc_xfer[1]);
	return;
}

static void
ulpt_stop_read(struct usb_cdev *cdev)
{
	struct ulpt_softc *sc = cdev->sc_priv_ptr;

	usbd_transfer_stop(sc->sc_xfer[5]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	return;
}

static void
ulpt_start_write(struct usb_cdev *cdev)
{
	struct ulpt_softc *sc = cdev->sc_priv_ptr;

	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
ulpt_stop_write(struct usb_cdev *cdev)
{
	struct ulpt_softc *sc = cdev->sc_priv_ptr;

	usbd_transfer_stop(sc->sc_xfer[4]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	return;
}

static int32_t
ulpt_open(struct usb_cdev *cdev, int32_t fflags,
    int32_t devtype, struct thread *td)
{
	uint8_t prime = ((cdev->sc_last_cdev == cdev->sc_cdev[0]) &&
	    (cdev->sc_first_open));
	struct ulpt_softc *sc = cdev->sc_priv_ptr;
	int32_t error = 0;

	if (fflags & FREAD) {
		/* clear stall first */
		sc->sc_flags |= ULPT_FLAG_READ_STALL;
	}
	if (fflags & FWRITE) {
		/* clear stall first */
		sc->sc_flags |= ULPT_FLAG_WRITE_STALL;
	}
	if (prime) {
		DPRINTF(0, "opening prime device (reset)\n");

		sc->sc_flags |= ULPT_FLAG_RESETTING;

		usbd_transfer_start(sc->sc_xfer[3]);

		error = usb_cdev_sleep(&(sc->sc_cdev), fflags, 0);

		usbd_transfer_stop(sc->sc_xfer[3]);

		sc->sc_flags &= ~ULPT_FLAG_RESETTING;

		if (error) {
			goto done;
		}
	}
done:
	return (error);
}

static int32_t
ulpt_ioctl(struct usb_cdev *cdev, u_long cmd, caddr_t data,
    int32_t fflags, struct thread *td)
{
	return (ENODEV);
}

static int
ulpt_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_interface_descriptor_t *id;

	DPRINTF(10, "\n");

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->iface == NULL) {
		return (UMATCH_NONE);
	}
	id = usbd_get_interface_descriptor(uaa->iface);

	if ((id != NULL) &&
	    (id->bInterfaceClass == UICLASS_PRINTER) &&
	    (id->bInterfaceSubClass == UISUBCLASS_PRINTER) &&
	    ((id->bInterfaceProtocol == UIPROTO_PRINTER_UNI) ||
	    (id->bInterfaceProtocol == UIPROTO_PRINTER_BI) ||
	    (id->bInterfaceProtocol == UIPROTO_PRINTER_1284))) {
		return (UMATCH_IFACECLASS_IFACESUBCLASS_IFACEPROTO);
	}
	return (UMATCH_NONE);
}

static int
ulpt_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ulpt_softc *sc = device_get_softc(dev);
	struct usbd_interface *iface_ptr = uaa->iface;
	usb_interface_descriptor_t *id;
	const char *p_buf[3];
	int32_t iface_alt_index = 0;
	int32_t unit = device_get_unit(dev);
	int32_t error;
	char buf_1[16];
	char buf_2[16];
	uint8_t iface_index = uaa->iface_index;

	DPRINTF(10, "sc=%p\n", sc);

	sc->sc_dev = dev;

	usbd_set_device_desc(dev);

	mtx_init(&(sc->sc_mtx), "ulpt lock", NULL, MTX_DEF | MTX_RECURSE);

	/* search through all the descriptors looking for bidir mode */

	while (iface_alt_index < 32) {

		error = usbd_fill_iface_data
		    (uaa->device, iface_index, iface_alt_index);

		if (error) {
			DPRINTF(0, "end of alternate settings, "
			    "error=%s\n", usbd_errstr(error));
			goto detach;
		}
		id = usbd_get_interface_descriptor(iface_ptr);

		if ((id->bInterfaceClass == UICLASS_PRINTER) &&
		    (id->bInterfaceSubClass == UISUBCLASS_PRINTER) &&
		    (id->bInterfaceProtocol == UIPROTO_PRINTER_BI)) {
			goto found;
		}
		iface_alt_index++;
	}
	goto detach;

found:

	DPRINTF(0, "setting alternate "
	    "config number: %d\n", iface_alt_index);

	if (iface_alt_index) {

		error = usbd_set_alt_interface_index
		    (uaa->device, iface_index, iface_alt_index);

		if (error) {
			DPRINTF(0, "could not set alternate "
			    "config, error=%s\n", usbd_errstr(error));
			goto detach;
		}
	}
	sc->sc_iface_no = id->bInterfaceNumber;

	error = usbd_transfer_setup(uaa->device, &iface_index,
	    sc->sc_xfer, ulpt_config, ULPT_N_TRANSFER,
	    sc, &(sc->sc_mtx));
	if (error) {
		DPRINTF(0, "error=%s\n", usbd_errstr(error));
		goto detach;
	}
	device_printf(sc->sc_dev, "using bi-directional mode\n");

#if 0
/*
 * This code is disabled because for some mysterious reason it causes
 * printing not to work.  But only sometimes, and mostly with
 * UHCI and less often with OHCI.  *sigh*
 */
	{
		usb_config_descriptor_t *cd = usbd_get_config_descriptor(dev);
		usb_device_request_t req;
		int len, alen;

		req.bmRequestType = UT_READ_CLASS_INTERFACE;
		req.bRequest = UR_GET_DEVICE_ID;
		USETW(req.wValue, cd->bConfigurationValue);
		USETW2(req.wIndex, id->bInterfaceNumber, id->bAlternateSetting);
		USETW(req.wLength, sizeof devinfo - 1);
		error = usbd_do_request_flags(dev, &req, devinfo, USBD_SHORT_XFER_OK,
		    &alen, USBD_DEFAULT_TIMEOUT);
		if (error) {
			device_printf(sc->sc_dev, "cannot get device id\n");
		} else if (alen <= 2) {
			device_printf(sc->sc_dev, "empty device id, no "
			    "printer connected?\n");
		} else {
			/* devinfo now contains an IEEE-1284 device ID */
			len = ((devinfo[0] & 0xff) << 8) | (devinfo[1] & 0xff);
			if (len > sizeof devinfo - 3)
				len = sizeof devinfo - 3;
			devinfo[len] = 0;
			printf("%s: device id <", device_get_nameunit(sc->sc_dev));
			ieee1284_print_id(devinfo + 2);
			printf(">\n");
		}
	}
#endif

	snprintf(buf_1, sizeof(buf_1), "ulpt%d", unit);
	snprintf(buf_2, sizeof(buf_2), "unlpt%d", unit);

	p_buf[0] = buf_1;
	p_buf[1] = buf_2;
	p_buf[2] = NULL;

	sc->sc_cdev.sc_start_read = &ulpt_start_read;
	sc->sc_cdev.sc_start_write = &ulpt_start_write;
	sc->sc_cdev.sc_stop_read = &ulpt_stop_read;
	sc->sc_cdev.sc_stop_write = &ulpt_stop_write;
	sc->sc_cdev.sc_open = &ulpt_open;
	sc->sc_cdev.sc_ioctl = &ulpt_ioctl;
	sc->sc_cdev.sc_flags |= (USB_CDEV_FLAG_WAKEUP_RD_IMMED |
	    USB_CDEV_FLAG_WAKEUP_WR_IMMED);

	error = usb_cdev_attach(&(sc->sc_cdev), sc, &(sc->sc_mtx), p_buf,
	    UID_ROOT, GID_OPERATOR, 0644,
	    sc->sc_xfer[1]->max_data_length, ULPT_IFQ_MAXLEN,
	    sc->sc_xfer[0]->max_data_length, ULPT_IFQ_MAXLEN);
	if (error) {
		goto detach;
	}
	/* start reading of status */

	usbd_transfer_start(sc->sc_xfer[2]);

	return (0);

detach:
	ulpt_detach(dev);
	return (ENOMEM);
}

static int
ulpt_detach(device_t dev)
{
	struct ulpt_softc *sc = device_get_softc(dev);

	DPRINTF(0, "sc=%p\n", sc);

	usb_cdev_detach(&(sc->sc_cdev));

	usbd_transfer_unsetup(sc->sc_xfer, ULPT_N_TRANSFER);

	mtx_destroy(&(sc->sc_mtx));

	return (0);
}

#if 0
/* XXX This does not belong here. */

/*
 * Compare two strings until the second ends.
 */

static uint8_t
ieee1284_compare(const char *a, const char *b)
{
	while (1) {

		if (*b == 0) {
			break;
		}
		if (*a != *b) {
			return 1;
		}
		b++;
		a++;
	}
	return 0;
}

/*
 * Print select parts of an IEEE 1284 device ID.
 */
void
ieee1284_print_id(char *str)
{
	char *p, *q;

	for (p = str - 1; p; p = strchr(p, ';')) {
		p++;			/* skip ';' */
		if (ieee1284_compare(p, "MFG:") == 0 ||
		    ieee1284_compare(p, "MANUFACTURER:") == 0 ||
		    ieee1284_compare(p, "MDL:") == 0 ||
		    ieee1284_compare(p, "MODEL:") == 0) {
			q = strchr(p, ';');
			if (q)
				printf("%.*s", (int)(q - p + 1), p);
		}
	}
}

#endif

static devclass_t ulpt_devclass;

static device_method_t ulpt_methods[] = {
	DEVMETHOD(device_probe, ulpt_probe),
	DEVMETHOD(device_attach, ulpt_attach),
	DEVMETHOD(device_detach, ulpt_detach),
	{0, 0}
};

static driver_t ulpt_driver = {
	.name = "ulpt",
	.methods = ulpt_methods,
	.size = sizeof(struct ulpt_softc),
};

DRIVER_MODULE(ulpt, uhub, ulpt_driver, ulpt_devclass, usbd_driver_load, 0);
MODULE_DEPEND(ulpt, usb, 1, 1, 1);
