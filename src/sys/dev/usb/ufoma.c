/*	$NetBSD: umodem.c,v 1.45 2002/09/23 05:51:23 simonb Exp $	*/

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ufoma.c,v 1.10 2007/07/27 12:00:29 takawata Exp $");

/*-
 * Copyright (c) 2005, Takanori Watanabe
 * Copyright (c) 2003, M. Warner Losh <imp@freebsd.org>.
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
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
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
 * Comm Class spec:  http://www.usb.org/developers/devclass_docs/usbccs10.pdf
 *                   http://www.usb.org/developers/devclass_docs/usbcdc11.pdf
 */

/*
 * TODO:
 * - Implement a Call Device for modems without multiplexed commands.
 */

/*
 * NOTE: all function names beginning like "ufoma_cfg_" can only
 * be called from within the config thread function !
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/termios.h>
#include <sys/serial.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>
#include <dev/usb/usb_cdc.h>

#include <dev/usb/ucomvar.h>

#include "usbdevs.h"

typedef struct ufoma_mobile_acm_descriptor {
	uint8_t	bFunctionLength;
	uint8_t	bDescriptorType;
	uint8_t	bDescriptorSubtype;
	uint8_t	bType;
	uint8_t	bMode[1];
} __packed usb_mcpc_acm_descriptor;

#define	UISUBCLASS_MCPC 0x88

#define	UDESC_VS_INTERFACE 0x44
#define	UDESCSUB_MCPC_ACM  0x11

#define	UMCPC_ACM_TYPE_AB1 0x1
#define	UMCPC_ACM_TYPE_AB2 0x2
#define	UMCPC_ACM_TYPE_AB5 0x5
#define	UMCPC_ACM_TYPE_AB6 0x6

#define	UMCPC_ACM_MODE_DEACTIVATED 0x0
#define	UMCPC_ACM_MODE_MODEM 0x1
#define	UMCPC_ACM_MODE_ATCOMMAND 0x2
#define	UMCPC_ACM_MODE_OBEX 0x60
#define	UMCPC_ACM_MODE_VENDOR1 0xc0
#define	UMCPC_ACM_MODE_VENDOR2 0xfe
#define	UMCPC_ACM_MODE_UNLINKED 0xff

#define	UMCPC_CM_MOBILE_ACM 0x0

#define	UMCPC_ACTIVATE_MODE 0x60
#define	UMCPC_GET_MODETABLE 0x61
#define	UMCPC_SET_LINK 0x62
#define	UMCPC_CLEAR_LINK 0x63

#define	UMCPC_REQUEST_ACKNOWLEDGE 0x31

#define	UFOMA_MAX_TIMEOUT 15		/* standard says 10 seconds */
#define	UFOMA_CMD_BUF_SIZE 64		/* bytes */

#define	UFOMA_BULK_BUF_SIZE 1024	/* bytes */

#define	UFOMA_CTRL_ENDPT_MAX 4		/* units */
#define	UFOMA_BULK_ENDPT_MAX 4		/* units */

#define	DPRINTF(...) do { } while (0)

struct ufoma_softc {
	struct ucom_super_softc sc_super_ucom;
	struct ucom_softc sc_ucom;

	struct usbd_xfer *sc_ctrl_xfer[UFOMA_CTRL_ENDPT_MAX];
	struct usbd_xfer *sc_bulk_xfer[UFOMA_BULK_ENDPT_MAX];
	uint8_t *sc_modetable;
	device_t sc_dev;
	struct usbd_device *sc_udev;

	uint32_t sc_unit;

	uint16_t sc_line;

	uint8_t	sc_num_msg;
	uint8_t	sc_is_pseudo;
	uint8_t	sc_ctrl_iface_no;
	uint8_t	sc_ctrl_iface_index;
	uint8_t	sc_data_iface_no;
	uint8_t	sc_data_iface_index;
	uint8_t	sc_cm_cap;
	uint8_t	sc_acm_cap;
	uint8_t	sc_lsr;
	uint8_t	sc_msr;
	uint8_t	sc_modetoactivate;
	uint8_t	sc_currentmode;
	uint8_t	sc_flags;
#define	UFOMA_FLAG_INTR_STALL        0x01
#define	UFOMA_FLAG_BULK_WRITE_STALL  0x02
#define	UFOMA_FLAG_BULK_READ_STALL   0x04

	uint8_t	sc_name[16];
};

/* prototypes */

static device_probe_t ufoma_probe;
static device_attach_t ufoma_attach;
static device_detach_t ufoma_detach;

static usbd_callback_t ufoma_ctrl_read_callback;
static usbd_callback_t ufoma_ctrl_write_callback;
static usbd_callback_t ufoma_intr_clear_stall_callback;
static usbd_callback_t ufoma_intr_callback;
static usbd_callback_t ufoma_bulk_write_callback;
static usbd_callback_t ufoma_bulk_write_clear_stall_callback;
static usbd_callback_t ufoma_bulk_read_callback;
static usbd_callback_t ufoma_bulk_read_clear_stall_callback;

static void ufoma_cfg_do_request(struct ufoma_softc *sc, usb_device_request_t *req, void *data);
static void *ufoma_get_intconf(usb_config_descriptor_t *cd, usb_interface_descriptor_t *id, uint8_t type, uint8_t subtype);
static void ufoma_cfg_link_state(struct ufoma_softc *sc);
static void ufoma_cfg_activate_state(struct ufoma_softc *sc, uint16_t state);
static void ufoma_cfg_open(struct ucom_softc *ucom);
static void ufoma_cfg_close(struct ucom_softc *ucom);
static void ufoma_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff);
static void ufoma_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr);
static void ufoma_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff);
static void ufoma_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff);
static int ufoma_pre_param(struct ucom_softc *ucom, struct termios *t);
static void ufoma_cfg_param(struct ucom_softc *ucom, struct termios *t);
static int ufoma_modem_setup(device_t dev, struct ufoma_softc *sc, struct usb_attach_arg *uaa);
static void ufoma_start_read(struct ucom_softc *ucom);
static void ufoma_stop_read(struct ucom_softc *ucom);
static void ufoma_start_write(struct ucom_softc *ucom);
static void ufoma_stop_write(struct ucom_softc *ucom);

static const struct usbd_config
	ufoma_ctrl_config[UFOMA_CTRL_ENDPT_MAX] = {

	[0] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.bufsize = sizeof(usb_cdc_notification_t),
		.mh.callback = &ufoma_intr_callback,
	},

	[1] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &ufoma_intr_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = (sizeof(usb_device_request_t) + UFOMA_CMD_BUF_SIZE),
		.mh.flags = {.short_xfer_ok = 1,},
		.mh.callback = &ufoma_ctrl_read_callback,
		.mh.timeout = 1000,	/* 1 second */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = (sizeof(usb_device_request_t) + 1),
		.mh.flags = {},
		.mh.callback = &ufoma_ctrl_write_callback,
		.mh.timeout = 1000,	/* 1 second */
	},
};

static const struct usbd_config
	ufoma_bulk_config[UFOMA_BULK_ENDPT_MAX] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.mh.bufsize = UFOMA_BULK_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.mh.callback = &ufoma_bulk_write_callback,
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.mh.bufsize = UFOMA_BULK_BUF_SIZE,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.mh.callback = &ufoma_bulk_read_callback,
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &ufoma_bulk_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.flags = {},
		.mh.callback = &ufoma_bulk_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
		.mh.interval = 50,	/* 50ms */
	},
};

static const struct ucom_callback ufoma_callback = {
	.ucom_cfg_get_status = &ufoma_cfg_get_status,
	.ucom_cfg_set_dtr = &ufoma_cfg_set_dtr,
	.ucom_cfg_set_rts = &ufoma_cfg_set_rts,
	.ucom_cfg_set_break = &ufoma_cfg_set_break,
	.ucom_cfg_param = &ufoma_cfg_param,
	.ucom_cfg_open = &ufoma_cfg_open,
	.ucom_cfg_close = &ufoma_cfg_close,
	.ucom_pre_param = &ufoma_pre_param,
	.ucom_start_read = &ufoma_start_read,
	.ucom_stop_read = &ufoma_stop_read,
	.ucom_start_write = &ufoma_start_write,
	.ucom_stop_write = &ufoma_stop_write,
};

static device_method_t ufoma_methods[] = {
	/* Device methods */
	DEVMETHOD(device_probe, ufoma_probe),
	DEVMETHOD(device_attach, ufoma_attach),
	DEVMETHOD(device_detach, ufoma_detach),
	{0, 0}
};

static devclass_t ufoma_devclass;

static driver_t ufoma_driver = {
	.name = "ufoma",
	.methods = ufoma_methods,
	.size = sizeof(struct ufoma_softc),
};

DRIVER_MODULE(ufoma, uhub, ufoma_driver, ufoma_devclass, usbd_driver_load, 0);
MODULE_DEPEND(ufoma, usb, 1, 1, 1);
MODULE_DEPEND(ufoma, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);

static int
ufoma_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_interface_descriptor_t *id;
	usb_config_descriptor_t *cd;
	usb_mcpc_acm_descriptor *mad;

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->iface == NULL) {
		return (UMATCH_NONE);
	}
	id = usbd_get_interface_descriptor(uaa->iface);
	cd = usbd_get_config_descriptor(uaa->device);

	if ((id == NULL) ||
	    (cd == NULL) ||
	    (id->bInterfaceClass != UICLASS_CDC) ||
	    (id->bInterfaceSubClass != UISUBCLASS_MCPC)) {
		return (UMATCH_NONE);
	}
	mad = ufoma_get_intconf(cd, id, UDESC_VS_INTERFACE, UDESCSUB_MCPC_ACM);
	if (mad == NULL) {
		return (UMATCH_NONE);
	}
#if 0
	if (mad->bType != UMCPC_ACM_TYPE_AB5) {
		return (UMATCH_NONE);
	}
#endif
	return (UMATCH_IFACECLASS_IFACESUBCLASS);
}

static int
ufoma_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ufoma_softc *sc = device_get_softc(dev);
	usb_config_descriptor_t *cd;
	usb_interface_descriptor_t *id;
	usb_mcpc_acm_descriptor *mad;
	uint8_t elements;
	int32_t error;

	if (sc == NULL) {
		return (ENOMEM);
	}
	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;
	sc->sc_unit = device_get_unit(dev);

	usbd_set_device_desc(dev);

	snprintf(sc->sc_name, sizeof(sc->sc_name),
	    "%s", device_get_nameunit(dev));

	DPRINTF(sc, 0, "\n");

	/* setup control transfers */

	cd = usbd_get_config_descriptor(uaa->device);
	id = usbd_get_interface_descriptor(uaa->iface);
	sc->sc_ctrl_iface_no = id->bInterfaceNumber;
	sc->sc_ctrl_iface_index = uaa->iface_index;

	error = usbd_transfer_setup(uaa->device,
	    &(sc->sc_ctrl_iface_index), sc->sc_ctrl_xfer,
	    ufoma_ctrl_config, UFOMA_CTRL_ENDPT_MAX, sc, &Giant);

	if (error) {
		device_printf(dev, "allocating control USB "
		    "transfers failed!\n");
		goto detach;
	}
	mad = ufoma_get_intconf(cd, id, UDESC_VS_INTERFACE, UDESCSUB_MCPC_ACM);
	if (mad == NULL) {
		goto detach;
	}
	if (mad->bFunctionLength < sizeof(*mad)) {
		device_printf(dev, "invalid MAD descriptor\n");
		goto detach;
	}
	if ((mad->bType == UMCPC_ACM_TYPE_AB5) ||
	    (mad->bType == UMCPC_ACM_TYPE_AB6)) {
		sc->sc_is_pseudo = 1;
	} else {
		sc->sc_is_pseudo = 0;
		if (ufoma_modem_setup(dev, sc, uaa)) {
			goto detach;
		}
	}

	elements = (mad->bFunctionLength - sizeof(*mad) + 1);

	/* initialize mode variables */

	sc->sc_modetable = malloc(elements + 1, M_USBDEV, M_WAITOK);

	if (sc->sc_modetable == NULL) {
		goto detach;
	}
	sc->sc_modetable[0] = (elements + 1);
	bcopy(mad->bMode, &(sc->sc_modetable[1]), elements);

	sc->sc_currentmode = UMCPC_ACM_MODE_UNLINKED;
	sc->sc_modetoactivate = mad->bMode[0];

	/* clear stall at first run */
	sc->sc_flags |= (UFOMA_FLAG_BULK_WRITE_STALL |
	    UFOMA_FLAG_BULK_READ_STALL);

	error = ucom_attach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1, sc,
	    &ufoma_callback, &Giant);
	if (error) {
		DPRINTF(sc, 0, "ucom_attach failed\n");
		goto detach;
	}
	return (0);			/* success */

detach:
	ufoma_detach(dev);
	return (ENXIO);			/* failure */
}

static int
ufoma_detach(device_t dev)
{
	struct ufoma_softc *sc = device_get_softc(dev);

	ucom_detach(&(sc->sc_super_ucom), &(sc->sc_ucom), 1);

	usbd_transfer_unsetup(sc->sc_ctrl_xfer, UFOMA_CTRL_ENDPT_MAX);

	usbd_transfer_unsetup(sc->sc_bulk_xfer, UFOMA_BULK_ENDPT_MAX);

	if (sc->sc_modetable) {
		free(sc->sc_modetable, M_USBDEV);
	}
	return (0);
}

static void
ufoma_cfg_do_request(struct ufoma_softc *sc, usb_device_request_t *req,
    void *data)
{
	uint16_t length;
	usbd_status_t err;

	if (ucom_cfg_is_gone(&(sc->sc_ucom))) {
		goto error;
	}
	err = usbd_do_request_flags
	    (sc->sc_udev, &Giant, req, data, 0, NULL, 1000);

	if (err) {

		DPRINTF(sc, -1, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));

error:
		length = UGETW(req->wLength);

		if ((req->bmRequestType & UT_READ) && length) {
			bzero(data, length);
		}
	}
	return;
}

static void *
ufoma_get_intconf(usb_config_descriptor_t *cd, usb_interface_descriptor_t *id,
    uint8_t type, uint8_t subtype)
{
	usb_descriptor_t *desc = (void *)id;

	while ((desc = usbd_desc_foreach(cd, desc))) {

		if (desc->bDescriptorType == UDESC_INTERFACE) {
			return (NULL);
		}
		if ((desc->bDescriptorType == type) &&
		    (desc->bDescriptorSubtype == subtype)) {
			break;
		}
	}
	return (desc);
}

static void
ufoma_cfg_link_state(struct ufoma_softc *sc)
{
	usb_device_request_t req;
	int32_t error;

	req.bmRequestType = UT_WRITE_VENDOR_INTERFACE;
	req.bRequest = UMCPC_SET_LINK;
	USETW(req.wValue, UMCPC_CM_MOBILE_ACM);
	USETW(req.wIndex, sc->sc_ctrl_iface_no);
	USETW(req.wLength, sc->sc_modetable[0]);

	ufoma_cfg_do_request(sc, &req, sc->sc_modetable);

	error = mtx_sleep(&(sc->sc_currentmode), &Giant, 0, "ufoma_link", hz);

	if (error) {
		DPRINTF(sc, 0, "NO response\n");
	}
	return;
}

static void
ufoma_cfg_activate_state(struct ufoma_softc *sc, uint16_t state)
{
	usb_device_request_t req;
	int32_t error;

	req.bmRequestType = UT_WRITE_VENDOR_INTERFACE;
	req.bRequest = UMCPC_ACTIVATE_MODE;
	USETW(req.wValue, state);
	USETW(req.wIndex, sc->sc_ctrl_iface_no);
	USETW(req.wLength, 0);

	ufoma_cfg_do_request(sc, &req, NULL);

	error = mtx_sleep(&(sc->sc_currentmode), &Giant, 0,
	    "fmaact", (UFOMA_MAX_TIMEOUT * hz));
	if (error) {
		DPRINTF(sc, 0, "No response\n");
	}
	return;
}

static void
ufoma_ctrl_read_callback(struct usbd_xfer *xfer)
{
	struct ufoma_softc *sc = xfer->priv_sc;
	usb_device_request_t req;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
tr_transferred:
		if (xfer->aframes != xfer->nframes) {
			goto tr_setup;
		}
		if (xfer->frlengths[1] > 0) {
			ucom_put_data(&(sc->sc_ucom), xfer->frbuffers + 1,
			    0, xfer->frlengths[1]);
		}
	case USBD_ST_SETUP:
tr_setup:
		if (sc->sc_num_msg) {
			sc->sc_num_msg--;

			req.bmRequestType = UT_READ_CLASS_INTERFACE;
			req.bRequest = UCDC_GET_ENCAPSULATED_RESPONSE;
			USETW(req.wIndex, sc->sc_ctrl_iface_no);
			USETW(req.wValue, 0);
			USETW(req.wLength, UFOMA_CMD_BUF_SIZE);

			usbd_copy_in(xfer->frbuffers, 0, &req, sizeof(req));

			xfer->frlengths[0] = sizeof(req);
			xfer->frlengths[1] = UFOMA_CMD_BUF_SIZE;
			xfer->nframes = 2;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		DPRINTF(sc, 0, "error = %s\n",
		    usbd_errstr(xfer->error));

		if (xfer->error == USBD_ERR_CANCELLED) {
			return;
		} else {
			goto tr_setup;
		}

		goto tr_transferred;
	}
}

static void
ufoma_ctrl_write_callback(struct usbd_xfer *xfer)
{
	struct ufoma_softc *sc = xfer->priv_sc;
	usb_device_request_t req;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
tr_transferred:
	case USBD_ST_SETUP:
tr_setup:
		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers + 1,
		    0, 1, &actlen)) {

			req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
			req.bRequest = UCDC_SEND_ENCAPSULATED_COMMAND;
			USETW(req.wIndex, sc->sc_ctrl_iface_no);
			USETW(req.wValue, 0);
			USETW(req.wLength, 1);

			usbd_copy_in(xfer->frbuffers, 0, &req, sizeof(req));

			xfer->frlengths[0] = sizeof(req);
			xfer->frlengths[1] = 1;
			xfer->nframes = 2;

			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		DPRINTF(sc, 0, "error = %s\n",
		    usbd_errstr(xfer->error));

		if (xfer->error == USBD_ERR_CANCELLED) {
			return;
		} else {
			goto tr_setup;
		}

		goto tr_transferred;
	}
}

static void
ufoma_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ufoma_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_ctrl_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~UFOMA_FLAG_INTR_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ufoma_intr_callback(struct usbd_xfer *xfer)
{
	struct ufoma_softc *sc = xfer->priv_sc;
	usb_cdc_notification_t pkt;
	uint16_t wLen;
	uint16_t temp;
	uint8_t mstatus;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		if (xfer->actlen < 8) {
			DPRINTF(sc, 0, "too short message\n");
			goto tr_setup;
		}
		if (xfer->actlen > sizeof(pkt)) {
			DPRINTF(sc, 0, "truncating message\n");
			xfer->actlen = sizeof(pkt);
		}
		usbd_copy_out(xfer->frbuffers, 0, &pkt, xfer->actlen);

		xfer->actlen -= 8;

		wLen = UGETW(pkt.wLength);
		if (xfer->actlen > wLen) {
			xfer->actlen = wLen;
		}
		if ((pkt.bmRequestType == UT_READ_VENDOR_INTERFACE) &&
		    (pkt.bNotification == UMCPC_REQUEST_ACKNOWLEDGE)) {
			temp = UGETW(pkt.wValue);
			sc->sc_currentmode = (temp >> 8);
			if (!(temp & 0xff)) {
				DPRINTF(sc, 0, "Mode change failed!\n");
			}
			wakeup(&(sc->sc_currentmode));
		}
		if (pkt.bmRequestType != UCDC_NOTIFICATION) {
			goto tr_setup;
		}
		switch (pkt.bNotification) {
		case UCDC_N_RESPONSE_AVAILABLE:
			if (!(sc->sc_is_pseudo)) {
				DPRINTF(sc, 0, "Wrong serial state!\n");
				break;
			}
			if (sc->sc_num_msg != 0xFF) {
				sc->sc_num_msg++;
			}
			usbd_transfer_start(sc->sc_ctrl_xfer[3]);
			break;

		case UCDC_N_SERIAL_STATE:
			if (sc->sc_is_pseudo) {
				DPRINTF(sc, 0, "Wrong serial state!\n");
				break;
			}
			/*
		         * Set the serial state in ucom driver based on
		         * the bits from the notify message
		         */
			if (xfer->actlen < 2) {
				DPRINTF(sc, 0, "invalid notification "
				    "length, %d bytes!\n", xfer->actlen);
				break;
			}
			DPRINTF(sc, 0, "notify bytes = 0x%02x, 0x%02x\n",
			    pkt.data[0], pkt.data[1]);

			/* currently, lsr is always zero. */
			sc->sc_lsr = 0;
			sc->sc_msr = 0;

			mstatus = pkt.data[0];

			if (mstatus & UCDC_N_SERIAL_RI) {
				sc->sc_msr |= SER_RI;
			}
			if (mstatus & UCDC_N_SERIAL_DSR) {
				sc->sc_msr |= SER_DSR;
			}
			if (mstatus & UCDC_N_SERIAL_DCD) {
				sc->sc_msr |= SER_DCD;
			}
			ucom_status_change(&(sc->sc_ucom));
			break;

		default:
			break;
		}

	case USBD_ST_SETUP:
tr_setup:
		if (sc->sc_flags & UFOMA_FLAG_INTR_STALL) {
			usbd_transfer_start(sc->sc_ctrl_xfer[1]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			/* start clear stall */
			sc->sc_flags |= UFOMA_FLAG_INTR_STALL;
			usbd_transfer_start(sc->sc_ctrl_xfer[1]);
		}
		return;

	}
}

static void
ufoma_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct ufoma_softc *sc = xfer->priv_sc;
	uint32_t actlen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
	case USBD_ST_TRANSFERRED:
		if (sc->sc_flags & UFOMA_FLAG_BULK_WRITE_STALL) {
			usbd_transfer_start(sc->sc_bulk_xfer[2]);
			return;
		}
		if (ucom_get_data(&(sc->sc_ucom), xfer->frbuffers, 0,
		    UFOMA_BULK_BUF_SIZE, &actlen)) {
			xfer->frlengths[0] = actlen;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flags |= UFOMA_FLAG_BULK_WRITE_STALL;
			usbd_transfer_start(sc->sc_bulk_xfer[2]);
		}
		return;

	}
}

static void
ufoma_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ufoma_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_bulk_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~UFOMA_FLAG_BULK_WRITE_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ufoma_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct ufoma_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		ucom_put_data(&(sc->sc_ucom), xfer->frbuffers, 0,
		    xfer->actlen);

	case USBD_ST_SETUP:
		if (sc->sc_flags & UFOMA_FLAG_BULK_READ_STALL) {
			usbd_transfer_start(sc->sc_bulk_xfer[3]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sc->sc_flags |= UFOMA_FLAG_BULK_READ_STALL;
			usbd_transfer_start(sc->sc_bulk_xfer[3]);
		}
		return;

	}
}

static void
ufoma_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ufoma_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_bulk_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~UFOMA_FLAG_BULK_READ_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ufoma_cfg_open(struct ucom_softc *ucom)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	/* empty input queue */

	if (sc->sc_num_msg != 0xFF) {
		sc->sc_num_msg++;
	}
	if (sc->sc_currentmode == UMCPC_ACM_MODE_UNLINKED) {
		ufoma_cfg_link_state(sc);
	}
	if (sc->sc_currentmode == UMCPC_ACM_MODE_DEACTIVATED) {
		ufoma_cfg_activate_state(sc, sc->sc_modetoactivate);
	}
	return;
}

static void
ufoma_cfg_close(struct ucom_softc *ucom)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	ufoma_cfg_activate_state(sc, UMCPC_ACM_MODE_DEACTIVATED);
	return;
}

static void
ufoma_cfg_set_break(struct ucom_softc *ucom, uint8_t onoff)
{
	struct ufoma_softc *sc = ucom->sc_parent;
	usb_device_request_t req;
	uint16_t wValue;

	if (sc->sc_is_pseudo) {
		return;
	}
	if (!(sc->sc_acm_cap & USB_CDC_ACM_HAS_BREAK)) {
		return;
	}
	wValue = onoff ? UCDC_BREAK_ON : UCDC_BREAK_OFF;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SEND_BREAK;
	USETW(req.wValue, wValue);
	req.wIndex[0] = sc->sc_ctrl_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	ufoma_cfg_do_request(sc, &req, 0);
	return;
}

static void
ufoma_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	*lsr = sc->sc_lsr;
	*msr = sc->sc_msr;
	return;
}

static void
ufoma_cfg_set_line_state(struct ufoma_softc *sc)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SET_CONTROL_LINE_STATE;
	USETW(req.wValue, sc->sc_line);
	req.wIndex[0] = sc->sc_ctrl_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	ufoma_cfg_do_request(sc, &req, 0);
	return;
}

static void
ufoma_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	if (sc->sc_is_pseudo) {
		return;
	}
	if (onoff)
		sc->sc_line |= UCDC_LINE_DTR;
	else
		sc->sc_line &= ~UCDC_LINE_DTR;

	ufoma_cfg_set_line_state(sc);
	return;
}

static void
ufoma_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	if (sc->sc_is_pseudo) {
		return;
	}
	if (onoff)
		sc->sc_line |= UCDC_LINE_RTS;
	else
		sc->sc_line &= ~UCDC_LINE_RTS;

	ufoma_cfg_set_line_state(sc);
	return;
}

static int
ufoma_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	return (0);			/* we accept anything */
}

static void
ufoma_cfg_param(struct ucom_softc *ucom, struct termios *t)
{
	struct ufoma_softc *sc = ucom->sc_parent;
	usb_device_request_t req;
	usb_cdc_line_state_t ls;

	if (sc->sc_is_pseudo) {
		return;
	}
	DPRINTF(sc, 0, "\n");

	bzero(&ls, sizeof(ls));

	USETDW(ls.dwDTERate, t->c_ospeed);

	if (t->c_cflag & CSTOPB) {
		ls.bCharFormat = UCDC_STOP_BIT_2;
	} else {
		ls.bCharFormat = UCDC_STOP_BIT_1;
	}

	if (t->c_cflag & PARENB) {
		if (t->c_cflag & PARODD) {
			ls.bParityType = UCDC_PARITY_ODD;
		} else {
			ls.bParityType = UCDC_PARITY_EVEN;
		}
	} else {
		ls.bParityType = UCDC_PARITY_NONE;
	}

	switch (t->c_cflag & CSIZE) {
	case CS5:
		ls.bDataBits = 5;
		break;
	case CS6:
		ls.bDataBits = 6;
		break;
	case CS7:
		ls.bDataBits = 7;
		break;
	case CS8:
		ls.bDataBits = 8;
		break;
	}

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SET_LINE_CODING;
	USETW(req.wValue, 0);
	req.wIndex[0] = sc->sc_ctrl_iface_no;
	req.wIndex[1] = 0;
	USETW(req.wLength, UCDC_LINE_STATE_LENGTH);

	ufoma_cfg_do_request(sc, &req, &ls);
	return;
}

static int
ufoma_modem_setup(device_t dev, struct ufoma_softc *sc,
    struct usb_attach_arg *uaa)
{
	usb_config_descriptor_t *cd;
	usb_cdc_acm_descriptor_t *acm;
	usb_cdc_cm_descriptor_t *cmd;
	usb_interface_descriptor_t *id;
	struct usbd_interface *iface;
	uint8_t i;
	int32_t error;

	cd = usbd_get_config_descriptor(uaa->device);
	id = usbd_get_interface_descriptor(uaa->iface);

	cmd = ufoma_get_intconf(cd, id, UDESC_CS_INTERFACE, UDESCSUB_CDC_CM);

	if ((cmd == NULL) ||
	    (cmd->bLength < sizeof(*cmd))) {
		return (EINVAL);
	}
	sc->sc_cm_cap = cmd->bmCapabilities;
	sc->sc_data_iface_no = cmd->bDataInterface;

	acm = ufoma_get_intconf(cd, id, UDESC_CS_INTERFACE, UDESCSUB_CDC_ACM);

	if ((acm == NULL) ||
	    (acm->bLength < sizeof(*acm))) {
		return (EINVAL);
	}
	sc->sc_acm_cap = acm->bmCapabilities;

	device_printf(dev, "data interface %d, has %sCM over data, "
	    "has %sbreak\n",
	    sc->sc_data_iface_no,
	    sc->sc_cm_cap & USB_CDC_CM_OVER_DATA ? "" : "no ",
	    sc->sc_acm_cap & USB_CDC_ACM_HAS_BREAK ? "" : "no ");

	/* get the data interface too */

	for (i = 0;; i++) {

		iface = usbd_get_iface(uaa->device, i);

		if (iface) {

			id = usbd_get_interface_descriptor(iface);

			if (id && (id->bInterfaceNumber == sc->sc_data_iface_no)) {
				sc->sc_data_iface_index = i;
				USBD_SET_IFACE_NO_PROBE(uaa->device, i);
				break;
			}
		} else {
			device_printf(dev, "no data interface!\n");
			return (EINVAL);
		}
	}

	error = usbd_transfer_setup(uaa->device,
	    &(sc->sc_data_iface_index), sc->sc_bulk_xfer,
	    ufoma_bulk_config, UFOMA_BULK_ENDPT_MAX, sc, &Giant);

	if (error) {
		device_printf(dev, "allocating BULK USB "
		    "transfers failed!\n");
		return (EINVAL);
	}
	return (0);
}

static void
ufoma_start_read(struct ucom_softc *ucom)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	/* start interrupt transfer */
	usbd_transfer_start(sc->sc_ctrl_xfer[0]);

	/* start data transfer */
	if (sc->sc_is_pseudo) {
		usbd_transfer_start(sc->sc_ctrl_xfer[2]);
	} else {
		usbd_transfer_start(sc->sc_bulk_xfer[1]);
	}
	return;
}

static void
ufoma_stop_read(struct ucom_softc *ucom)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	/* stop interrupt transfer */
	usbd_transfer_stop(sc->sc_ctrl_xfer[1]);
	usbd_transfer_stop(sc->sc_ctrl_xfer[0]);

	/* stop data transfer */
	if (sc->sc_is_pseudo) {
		usbd_transfer_stop(sc->sc_ctrl_xfer[2]);
	} else {
		usbd_transfer_stop(sc->sc_bulk_xfer[3]);
		usbd_transfer_stop(sc->sc_bulk_xfer[1]);
	}
	return;
}

static void
ufoma_start_write(struct ucom_softc *ucom)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	if (sc->sc_is_pseudo) {
		usbd_transfer_start(sc->sc_ctrl_xfer[3]);
	} else {
		usbd_transfer_start(sc->sc_bulk_xfer[0]);
	}
	return;
}

static void
ufoma_stop_write(struct ucom_softc *ucom)
{
	struct ufoma_softc *sc = ucom->sc_parent;

	if (sc->sc_is_pseudo) {
		usbd_transfer_stop(sc->sc_ctrl_xfer[3]);
	} else {
		usbd_transfer_stop(sc->sc_bulk_xfer[2]);
		usbd_transfer_stop(sc->sc_bulk_xfer[0]);
	}
	return;
}
