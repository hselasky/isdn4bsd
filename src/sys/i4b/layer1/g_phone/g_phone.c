/*-
 * Copyright (c) 2014 Hans Petter Selasky. All rights reserved.
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
 *
 *---------------------------------------------------------------------------
 *
 *      USB phone driver for ISDN4BSD
 *      -----------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/linker_set.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/priv.h>

#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>
#include <dev/usb/usbdi.h>

#define	USB_DEBUG_VAR g_phone_debug

#include <dev/usb/usb_core.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_device.h>
#include <dev/usb/usb_request.h>
#include <dev/usb/usb_debug.h>
#include <dev/usb/usb_hub.h>
#include <dev/usb/usb_util.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_transfer.h>
#include <dev/usb/usb_dynamic.h>

#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>

#include <i4b/include/i4b_compat.h>
#include <i4b/include/i4b_controller.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/dss1/dss1_lite.h>

#include <i4b/layer1/g_phone/g_phone.h>

#include "usb_if.h"

#ifdef USB_DEBUG
static int g_phone_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, g_phone, CTLFLAG_RW, 0, "USB VoIP");
SYSCTL_INT(_hw_usb_g_phone, OID_AUTO, debug, CTLFLAG_RW, &g_phone_debug, 0,
    "Debug level");
#endif

static device_probe_t g_phone_probe;
static device_attach_t g_phone_attach;
static device_detach_t g_phone_detach;
static usb_handle_request_t g_phone_handle_request;

static devclass_t g_phone_devclass;

static device_method_t g_phone_methods[] = {
	DEVMETHOD(usb_handle_request, g_phone_handle_request),

	DEVMETHOD(device_probe, g_phone_probe),
	DEVMETHOD(device_attach, g_phone_attach),
	DEVMETHOD(device_detach, g_phone_detach),

	DEVMETHOD_END
};

static driver_t g_phone_driver = {
	.name = "g_phone",
	.methods = g_phone_methods,
	.size = sizeof(struct g_phone_softc)
};

DRIVER_MODULE(g_phone, uhub, g_phone_driver, g_phone_devclass, NULL, 0);
MODULE_DEPEND(g_phone, usb, 1, 1, 1);
MODULE_DEPEND(g_phone, ihfcpnp, 1, 1, 1);

static dss1_lite_set_protocol_t g_phone_set_protocol;
static dss1_lite_set_hook_on_t g_phone_set_hook_on;
static dss1_lite_set_hook_off_t g_phone_set_hook_off;
static dss1_lite_set_r_key_t g_phone_set_r_key;
static dss1_lite_set_dtmf_t g_phone_set_dtmf;

static const struct dss1_lite_methods g_phone_dl_methods = {
	DSS1_LITE_DEFAULT_METHODS,
	.set_hook_on = g_phone_set_hook_on,
	.set_hook_off = g_phone_set_hook_off,
	.set_r_key = g_phone_set_r_key,
	.set_dtmf = g_phone_set_dtmf,
	.set_protocol = g_phone_set_protocol,
	.support_echo_cancel = 0,
};

static const struct usb_device_id g_phone_devs_table[] = {
	{USB_VPI(0x6993, 0xb001, 0)},
};

static usb_callback_t g_phone_intr_callback;
static usb_callback_t g_phone_isoc_read_callback;
static usb_callback_t g_phone_isoc_write_callback;

static const struct usb_config g_phone_config[G_PHONE_XFER_MAX] = {
	[G_PHONE_XFER_INTR] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = G_PHONE_PKT_LEN,
		.flags = {.ext_buffer = 1,},
		.callback = &g_phone_intr_callback,
		.timeout = 1000,	/* 1 second */
		.if_index = 0,
		.usb_mode = USB_MODE_DEVICE,
	},

	[G_PHONE_XFER_ISOC_IN_0] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.bufsize = G_PHONE_BUFSIZE,
		.frames = G_PHONE_MINFRAMES,
		.flags = {.short_xfer_ok = 1,.ext_buffer = 1,},
		.callback = &g_phone_isoc_read_callback,
		.if_index = 2,
		.usb_mode = USB_MODE_DEVICE,
	},

	[G_PHONE_XFER_ISOC_IN_1] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.bufsize = G_PHONE_BUFSIZE,
		.frames = G_PHONE_MINFRAMES,
		.flags = {.short_xfer_ok = 1,.ext_buffer = 1,},
		.callback = &g_phone_isoc_read_callback,
		.if_index = 2,
		.usb_mode = USB_MODE_DEVICE,
	},

	[G_PHONE_XFER_ISOC_OUT_0] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = G_PHONE_BUFSIZE,
		.frames = G_PHONE_MINFRAMES,
		.flags = {.ext_buffer = 1,},
		.callback = &g_phone_isoc_write_callback,
		.if_index = 1,
		.usb_mode = USB_MODE_DEVICE,
	},

	[G_PHONE_XFER_ISOC_OUT_1] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.bufsize = G_PHONE_BUFSIZE,
		.frames = G_PHONE_MINFRAMES,
		.flags = {.ext_buffer = 1,},
		.callback = &g_phone_isoc_write_callback,
		.if_index = 1,
		.usb_mode = USB_MODE_DEVICE,
	},
};

static void
g_phone_set_protocol(struct dss1_lite *pdl,
    struct dss1_lite_fifo *f, struct i4b_protocol *p)
{
}

static void
g_phone_watchdog(void *arg)
{
	struct g_phone_softc *sc = arg;

	if (sc->sc_ring_timeout != 0) {
		sc->sc_ring_timeout--;
		if (sc->sc_ring_timeout == 0 && sc->sc_hook_off == 0)
			(void)dss1_lite_ring_event(&sc->sc_dl, 0);
	}
	/* make sure the DSS1 code gets a chance to run */
	dss1_lite_process(&sc->sc_dl);

	callout_reset(&sc->sc_callout, hz / 8, &g_phone_watchdog, sc);
}

static void
g_phone_intr_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct g_phone_softc *sc = usbd_xfer_softc(xfer);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		sc->sc_intr_input_pos++;
		sc->sc_intr_input_pos %= G_PHONE_CMD_MAX;

		/* FALLTHROUGH */
	case USB_ST_SETUP:
tr_setup:
		if (sc->sc_intr_input_pos != sc->sc_intr_output_pos) {
			/* setup USB transfer */
			usbd_xfer_set_frame_data(xfer, 0,
			    sc->sc_intr_data + sc->sc_intr_input_pos,
			    G_PHONE_PKT_LEN);
			usbd_xfer_set_frames(xfer, 1);
			usbd_transfer_submit(xfer);
		}
		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED)
			goto tr_setup;
		break;
	}
}

static void
g_phone_isoc_read_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct g_phone_softc *sc = usbd_xfer_softc(xfer);
	struct dss1_lite_fifo *f = &sc->sc_dl.dl_fifo[sc->sc_dl.dl_audio_channel];
	uint8_t *buf;
	uint16_t i;
	uint16_t j;
	uint16_t k;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:

		f->tx_timestamp = (usbd_xfer_get_timestamp(xfer) * 8);

		buf = usbd_xfer_get_priv(xfer);

		for (i = 0; i != G_PHONE_MINFRAMES; i++) {

			k = usbd_xfer_frame_len(xfer, i) & -2UL;

			for (j = 0; j < k; j += 2) {
				dss1_lite_l5_put_sample(&sc->sc_dl, f, (int16_t)UGETW(buf + j));
			}

			for (; j < G_PHONE_BPF; j += 2) {
				dss1_lite_l5_put_sample(&sc->sc_dl, f, (int16_t)f->m_tx_last_sample);
			}

			buf += G_PHONE_BPF;
		}

		dss1_lite_l5_put_sample_complete(&sc->sc_dl, f);

		if (sc->sc_hook_off == 0 && f->m_tx_last_sample != 0)
			(void)dss1_lite_ring_event(&sc->sc_dl, 1);

	case USB_ST_SETUP:
tr_setup:

		buf = usbd_xfer_get_priv(xfer);

		/* setup USB transfer */
		usbd_xfer_set_frame_data(xfer, 0, buf, 0);

		for (i = 0; i != G_PHONE_MINFRAMES; i++) {
			usbd_xfer_set_frame_len(xfer, i, G_PHONE_BPF);
		}
		usbd_xfer_set_frames(xfer, G_PHONE_MINFRAMES);

		usbd_transfer_submit(xfer);
		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED) {
			goto tr_setup;
		}
		break;
	}
}

static void
g_phone_isoc_write_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct g_phone_softc *sc = usbd_xfer_softc(xfer);
	struct dss1_lite_fifo *f = &sc->sc_dl.dl_fifo[sc->sc_dl.dl_audio_channel];
	uint8_t *buf;
	uint16_t i;
	uint16_t temp;
	uint16_t timestamp = (usbd_xfer_get_timestamp(xfer) * 8) + (G_PHONE_MINFRAMES * 8 * 2);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
	case USB_ST_SETUP:
tr_setup:
		buf = usbd_xfer_get_priv(xfer);

		for (i = 0; i != G_PHONE_BUFSIZE; i += 2) {
			temp = dss1_lite_l5_get_sample(&sc->sc_dl, f);
			USETW(buf + i, temp);
		}

		dss1_lite_l5_get_sample_complete(&sc->sc_dl, f);

		/* setup USB transfer */
		usbd_xfer_set_frame_data(xfer, 0, buf, 0);

		for (i = 0; i != G_PHONE_MINFRAMES; i++) {
			usbd_xfer_set_frame_len(xfer, i, G_PHONE_BPF);
		}
		usbd_xfer_set_frames(xfer, G_PHONE_MINFRAMES);

		usbd_transfer_submit(xfer);

		/* set timestamp for end next transfer */

		f->rx_timestamp = timestamp;

		break;

	default:			/* Error */
		DPRINTF("error=%s\n", usbd_errstr(error));

		if (error != USB_ERR_CANCELLED) {
			goto tr_setup;
		}
		break;
	}
}

static int
g_phone_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_DEVICE)
		return (ENXIO);
	if (uaa->info.bConfigIndex != G_PHONE_CONFIG_INDEX)
		return (ENXIO);
	if (uaa->info.bIfaceIndex != G_PHONE_IFACE_INDEX)
		return (ENXIO);
	if (usbd_get_speed(uaa->device) != USB_SPEED_FULL &&
	    usbd_get_speed(uaa->device) != USB_SPEED_HIGH)
		return (ENXIO);
	return (usbd_lookup_id_by_uaa(g_phone_devs_table,
	    sizeof(g_phone_devs_table), uaa));
}

static int
g_phone_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct g_phone_softc *sc = device_get_softc(dev);
	struct usb_interface *iface = NULL;
	struct usb_interface_descriptor *idesc = NULL;
	struct usb_endpoint_descriptor *ed = NULL;
	struct i4b_controller *ctrl;
	uint8_t i;
	uint8_t iface_index[3];

	ctrl = i4b_controller_allocate(1, 1, 4, NULL);
	if (ctrl == NULL) {
		device_printf(dev, "Could not allocate I4B controller.\n");
		return (ENXIO);
	}
	sc->sc_pmtx = CNTL_GET_LOCK(ctrl);

	callout_init_mtx(&sc->sc_callout, sc->sc_pmtx, 0);

	device_set_desc(dev, "USB Phone Adapter");

	sc->sc_udev = uaa->device;

	iface_index[0] = 0;
	iface_index[1] = 0;
	iface_index[2] = 0;

	/* claim all interfaces */

	for (i = 0;; i++) {
		if (i == iface_index[0])
			continue;
		iface = usbd_get_iface(uaa->device, i);
		if (iface == NULL)
			break;
		idesc = usbd_get_interface_descriptor(iface);
		if (idesc == NULL)
			continue;

		usbd_set_parent_iface(uaa->device, i, iface_index[0]);

		if (idesc->bInterfaceClass == UICLASS_AUDIO) {
			if (idesc->bInterfaceSubClass == UISUBCLASS_AUDIOSTREAM) {
				usbd_set_alt_interface_index(uaa->device, i, 1);
				ed = usbd_find_descriptor(uaa->device,
				    NULL, i, UDESC_ENDPOINT, 0xFF, 0, 0);
				if (ed != NULL) {
					if (ed->bEndpointAddress & UE_DIR_IN)
						iface_index[1] = i;
					else
						iface_index[2] = i;
				}
			}
			if (idesc->bInterfaceSubClass == UISUBCLASS_AUDIOCONTROL) {
				/* Nothing to do */
			}
		}
		if (idesc->bInterfaceClass == UICLASS_HID)
			iface_index[0] = i;
	}

	if (usbd_transfer_setup(sc->sc_udev, iface_index, sc->sc_xfer,
	    g_phone_config, G_PHONE_XFER_MAX, sc, sc->sc_pmtx)) {
		DPRINTF("could not allocate USB transfers!\n");
		goto error;
	}
	usbd_xfer_set_priv(sc->sc_xfer[G_PHONE_XFER_ISOC_IN_0],
	    sc->sc_buffer + (G_PHONE_BUFSIZE * 0));
	usbd_xfer_set_priv(sc->sc_xfer[G_PHONE_XFER_ISOC_IN_1],
	    sc->sc_buffer + (G_PHONE_BUFSIZE * 1));
	usbd_xfer_set_priv(sc->sc_xfer[G_PHONE_XFER_ISOC_OUT_0],
	    sc->sc_buffer + (G_PHONE_BUFSIZE * 2));
	usbd_xfer_set_priv(sc->sc_xfer[G_PHONE_XFER_ISOC_OUT_1],
	    sc->sc_buffer + (G_PHONE_BUFSIZE * 3));

	sc->sc_dl.dl_softc = sc;

	if (dss1_lite_attach(&sc->sc_dl, dev,
	    ctrl, &g_phone_dl_methods)) {
		device_printf(dev, "DSS1 lite attach failed\n");
		goto error;
	}
	mtx_lock(sc->sc_pmtx);
	usbd_transfer_start(sc->sc_xfer[G_PHONE_XFER_ISOC_IN_0]);
	usbd_transfer_start(sc->sc_xfer[G_PHONE_XFER_ISOC_IN_1]);
	usbd_transfer_start(sc->sc_xfer[G_PHONE_XFER_ISOC_OUT_0]);
	usbd_transfer_start(sc->sc_xfer[G_PHONE_XFER_ISOC_OUT_1]);

	/* start watchdog */
	g_phone_watchdog(sc);
	mtx_unlock(sc->sc_pmtx);

	return (0);

error:
	i4b_controller_free(ctrl, 1);

	g_phone_detach(dev);

	return (ENXIO);
}

static int
g_phone_detach(device_t dev)
{
	struct g_phone_softc *sc = device_get_softc(dev);

	usbd_transfer_unsetup(sc->sc_xfer, G_PHONE_XFER_MAX);

	callout_drain(&sc->sc_callout);

	dss1_lite_detach(&sc->sc_dl);

	return (0);			/* success */
}

static uint8_t
g_phone_get_command(const struct usb_device_request *req)
{
	/* decode the USB request */
	if ((req->bmRequestType == UT_WRITE_CLASS_INTERFACE) &&
	    (req->wValue[0] == 0) && (req->wIndex[1] == 2) &&
	    (req->wIndex[0] == 3) && (req->wIndex[1] == 0) &&
	    (req->bRequest == UR_SET_CONFIG)) {
		return (G_CMD_SET_CONFIG);
	} else if ((req->bmRequestType == UT_READ_CLASS_INTERFACE) &&
	    (req->wIndex[0] == 0) && (req->bRequest == 0x82)) {
		return (G_CMD_AUDIO_GET_MIN);
	} else if ((req->bmRequestType == UT_READ_CLASS_INTERFACE) &&
	    (req->wIndex[0] == 0) && (req->bRequest == 0x83)) {
		return (G_CMD_AUDIO_GET_MAX);
	} else if ((req->bmRequestType == UT_READ_CLASS_INTERFACE) &&
	    (req->wIndex[0] == 0) && (req->bRequest == 0x84)) {
		return (G_CMD_AUDIO_GET_RES);
	} else if ((req->bmRequestType == UT_READ_CLASS_INTERFACE) &&
	    (req->wIndex[0] == 0) && (req->bRequest == 0x81)) {
		return (G_CMD_AUDIO_GET_VOLUME);
	} else if ((req->bmRequestType == UT_WRITE_CLASS_INTERFACE) &&
	    (req->wIndex[0] == 0) && (req->bRequest == 0x01)) {
		return (G_CMD_AUDIO_SET_VOLUME);
	} else if ((req->bmRequestType == UT_WRITE_CLASS_ENDPOINT) &&
	    (req->wIndex[0] == 0) && (req->bRequest == 0x01)) {
		return (G_CMD_AUDIO_SET_RATE);
	}
	return (G_CMD_MAX);
}

static void
g_phone_reply(struct g_phone_softc *sc, uint8_t cmd, uint8_t value)
{
	uint8_t next_cmd = (sc->sc_intr_output_pos + 1) % G_PHONE_CMD_MAX;
	uint8_t *ptr = sc->sc_intr_data[sc->sc_intr_output_pos];
	uint8_t sum = 0;
	uint8_t i;

	if (next_cmd == sc->sc_intr_input_pos)
		return;

	memset(ptr, 0, G_PHONE_PKT_LEN);

	ptr[0] = cmd;
	ptr[1] = 1;			/* 1-byte */
	ptr[4] = value;

	for (i = 0; i < (G_PHONE_PKT_LEN - 1); i++)
		sum -= ptr[i];

	ptr[G_PHONE_PKT_LEN - 1] = sum;

	sc->sc_intr_output_pos = next_cmd;

	usbd_transfer_start(sc->sc_xfer[G_PHONE_XFER_INTR]);
}

static int
g_phone_handle_request(device_t dev,
    const void *preq, void **pptr, uint16_t *plen,
    uint16_t offset, uint8_t *pstate)
{
	struct g_phone_softc *sc = device_get_softc(dev);
	const struct usb_device_request *req = preq;
	uint8_t is_complete = *pstate;
	uint8_t cmd;

	cmd = g_phone_get_command(req);

	if (!is_complete) {
		switch (cmd) {
		case G_CMD_SET_CONFIG:
			if (offset == 0) {
				memset(sc->sc_command_data, 0, sizeof(sc->sc_command_data));
				*plen = sizeof(sc->sc_command_data);
				*pptr = sc->sc_command_data;
			} else {
				*plen = 0;
			}
			return (0);
		case G_CMD_AUDIO_GET_MIN:
			if (offset == 0) {
				USETW(sc->sc_volume_limit, 0);
				*plen = 2;
				*pptr = &sc->sc_volume_limit;
			} else {
				*plen = 0;
			}
			return (0);
		case G_CMD_AUDIO_GET_MAX:
			if (offset == 0) {
				USETW(sc->sc_volume_limit, 0x2000);
				*plen = 2;
				*pptr = &sc->sc_volume_limit;
			} else {
				*plen = 0;
			}
			return (0);
		case G_CMD_AUDIO_GET_RES:
			if (offset == 0) {
				USETW(sc->sc_volume_limit, 0);
				*plen = 2;
				*pptr = &sc->sc_volume_limit;
			} else {
				*plen = 0;
			}
			return (0);
		case G_CMD_AUDIO_GET_VOLUME:
		case G_CMD_AUDIO_SET_VOLUME:
			if (offset == 0) {
				*plen = sizeof(sc->sc_volume_setting);
				*pptr = &sc->sc_volume_setting;
			} else {
				*plen = 0;
			}
			return (0);
		case G_CMD_AUDIO_SET_RATE:
			if (offset == 0) {
				*plen = sizeof(sc->sc_sample_rate);
				*pptr = &sc->sc_sample_rate;
			} else {
				*plen = 0;
			}
			return (0);
		default:
			break;
		}
	} else {
		switch (cmd) {
		case G_CMD_SET_CONFIG:
			mtx_lock(sc->sc_pmtx);
			switch (sc->sc_command_data[0]) {
			case 0x01:	/* ring on/off */
				if (sc->sc_command_data[4]) {
					sc->sc_ring_timeout = 0;	/* ring on */
					(void)dss1_lite_ring_event(&sc->sc_dl, 1);
				} else {
					sc->sc_ring_timeout = 16; /* ring off */ ;
				}
				break;
			case 0x8d:	/* handset query */
				g_phone_reply(sc, 0x8d, sc->sc_hook_off);
				break;
			case 0x80:	/* key query */
				break;
			default:
				break;
			}
			mtx_unlock(sc->sc_pmtx);
			return (0);
		default:
			break;
		}
	}
	return (ENXIO);			/* use builtin handler */
}

static void
g_phone_set_hook_on(struct dss1_lite *pdl)
{
	struct g_phone_softc *sc = pdl->dl_softc;

	sc->sc_hook_off = 0;
	g_phone_reply(sc, 0x81, 0x13);
}

static void
g_phone_set_hook_off(struct dss1_lite *pdl)
{
	struct g_phone_softc *sc = pdl->dl_softc;

	sc->sc_hook_off = 1;
}

static void
g_phone_set_r_key(struct dss1_lite *pdl)
{
	/* NOP */
}

static void
g_phone_set_dtmf(struct dss1_lite *pdl, const char *digits)
{
	struct g_phone_softc *sc = pdl->dl_softc;

	for (; *digits; digits++) {
		switch (*digits) {
		case '0':
			g_phone_reply(sc, 0x81, 0x31);
			break;
		case '1':
			g_phone_reply(sc, 0x81, 0x00);
			break;
		case '2':
			g_phone_reply(sc, 0x81, 0x01);
			break;
		case '3':
			g_phone_reply(sc, 0x81, 0x02);
			break;
		case '4':
			g_phone_reply(sc, 0x81, 0x10);
			break;
		case '5':
			g_phone_reply(sc, 0x81, 0x11);
			break;
		case '6':
			g_phone_reply(sc, 0x81, 0x12);
			break;
		case '7':
			g_phone_reply(sc, 0x81, 0x20);
			break;
		case '8':
			g_phone_reply(sc, 0x81, 0x21);
			break;
		case '9':
			g_phone_reply(sc, 0x81, 0x22);
			break;
		case '*':
			g_phone_reply(sc, 0x81, 0x30);
			break;
		case '#':
			g_phone_reply(sc, 0x81, 0x32);
			break;
		default:
			break;
		}
	}
}
