/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
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

#include <dev/usb2/include/usb2_defs.h>
#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_standard.h>
#include <dev/usb2/include/usb2_ioctl.h>
#include <dev/usb2/include/usb2_error.h>
#include <dev/usb2/include/usb2_revision.h>

#define	USB_DEBUG_VAR ugen_debug

#include <dev/usb2/core/usb2_core.h>
#include <dev/usb2/core/usb2_mbuf.h>
#include <dev/usb2/core/usb2_dev.h>
#include <dev/usb2/core/usb2_process.h>
#include <dev/usb2/core/usb2_device.h>
#include <dev/usb2/core/usb2_debug.h>
#include <dev/usb2/core/usb2_request.h>
#include <dev/usb2/core/usb2_busdma.h>
#include <dev/usb2/core/usb2_util.h>
#include <dev/usb2/core/usb2_hub.h>
#include <dev/usb2/core/usb2_generic.h>

#include <dev/usb2/controller/usb2_controller.h>
#include <dev/usb2/controller/usb2_bus.h>

/* defines */

#define	UGEN_BULK_FS_BUFFER_SIZE	(64*32)	/* bytes */
#define	UGEN_BULK_HS_BUFFER_SIZE	(1024*32)	/* bytes */
#define	UGEN_HW_FRAMES	50		/* number of milliseconds per transfer */

/* function prototypes */

static usb2_callback_t ugen_read_clear_stall_callback;
static usb2_callback_t ugen_write_clear_stall_callback;
static usb2_callback_t ugen_default_read_callback;
static usb2_callback_t ugen_default_write_callback;
static usb2_callback_t ugen_isoc_read_callback;
static usb2_callback_t ugen_isoc_write_callback;

static usb2_fifo_open_t ugen_open;
static usb2_fifo_close_t ugen_close;
static usb2_fifo_ioctl_t ugen_ioctl;
static usb2_fifo_cmd_t ugen_start_read;
static usb2_fifo_cmd_t ugen_start_write;
static usb2_fifo_cmd_t ugen_stop_io;

static int ugen_transfer_setup(struct usb2_fifo *f, const struct usb2_config *setup, uint8_t n_setup);
static int ugen_open_pipe_write(struct usb2_fifo *f);
static int ugen_open_pipe_read(struct usb2_fifo *f);
static int ugen_set_config(struct usb2_fifo *f, uint8_t index);
static int ugen_set_interface(struct usb2_fifo *f, uint8_t iface_index, uint8_t alt_index);
static int ugen_get_cdesc(struct usb2_fifo *f, struct usb2_gen_descriptor *pgd);
static int ugen_get_sdesc(struct usb2_fifo *f, struct usb2_gen_descriptor *ugd);
static int usb2_gen_fill_deviceinfo(struct usb2_fifo *f, struct usb2_device_info *di);
static int ugen_re_enumerate(struct usb2_fifo *f);
static int ugen_iface_ioctl(struct usb2_fifo *f, u_long cmd, void *addr);
static int ugen_ctrl_ioctl(struct usb2_fifo *f, u_long cmd, void *addr, int fflags);

/* structures */

struct usb2_fifo_methods usb2_ugen_methods = {
	.f_open = &ugen_open,
	.f_close = &ugen_close,
	.f_ioctl = &ugen_ioctl,
	.f_start_read = &ugen_start_read,
	.f_stop_read = &ugen_stop_io,
	.f_start_write = &ugen_start_write,
	.f_stop_write = &ugen_stop_io,
};

#ifdef	USB_DEBUG
static int ugen_debug = 0;

SYSCTL_NODE(_hw_usb2, OID_AUTO, ugen, CTLFLAG_RW, 0, "USB generic");
SYSCTL_INT(_hw_usb2_ugen, OID_AUTO, debug, CTLFLAG_RW, &ugen_debug,
    0, "Debug level");
#endif


/* prototypes */

static int
ugen_transfer_setup(struct usb2_fifo *f,
    const struct usb2_config *setup, uint8_t n_setup)
{
	struct usb2_pipe *pipe = f->priv_sc0;
	struct usb2_device *udev = f->udev;
	uint8_t iface_index = pipe->iface_index;
	int error;

	mtx_unlock(f->priv_mtx);

	/*
	 * "usb2_transfer_setup()" can sleep so one needs to make a wrapper,
	 * exiting the mutex and checking things
	 */
	error = usb2_transfer_setup(udev, &iface_index, f->xfer,
	    setup, n_setup, f, f->priv_mtx);
	if (error == 0) {

		if (f->xfer[0]->nframes == 1) {
			error = usb2_fifo_alloc_buffer(f,
			    f->xfer[0]->max_data_length, 2);
		} else {
			error = usb2_fifo_alloc_buffer(f,
			    f->xfer[0]->max_frame_size,
			    2 * f->xfer[0]->nframes);
		}
		if (error) {
			usb2_transfer_unsetup(f->xfer, n_setup);
		}
	}
	mtx_lock(f->priv_mtx);

	return (error);
}

static int
ugen_open(struct usb2_fifo *f, int fflags, struct thread *td)
{
	struct usb2_pipe *pipe = f->priv_sc0;
	struct usb2_endpoint_descriptor *ed = pipe->edesc;
	uint8_t type;

	DPRINTF(5, "flag=0x%x\n", fflags);

	mtx_lock(f->priv_mtx);
	if (usb2_get_speed(f->udev) == USB_SPEED_HIGH) {
		f->nframes = UGEN_HW_FRAMES * 8;
		f->bufsize = UGEN_BULK_HS_BUFFER_SIZE;
	} else {
		f->nframes = UGEN_HW_FRAMES;
		f->bufsize = UGEN_BULK_FS_BUFFER_SIZE;
	}

	type = ed->bmAttributes & UE_XFERTYPE;
	if (type == UE_INTERRUPT) {
		f->bufsize = 0;		/* use "wMaxPacketSize" */
	}
	f->timeout = USB_NO_TIMEOUT;
	f->flag_short = 0;
	f->fifo_zlp = 0;
	mtx_unlock(f->priv_mtx);

	return (0);
}

static void
ugen_close(struct usb2_fifo *f, int fflags, struct thread *td)
{
	DPRINTF(5, "flag=0x%x\n", fflags);

	/* cleanup */

	mtx_lock(f->priv_mtx);
	usb2_transfer_stop(f->xfer[0]);
	usb2_transfer_stop(f->xfer[1]);
	mtx_unlock(f->priv_mtx);

	usb2_transfer_unsetup(f->xfer, 2);
	usb2_fifo_free_buffer(f);
	return;
}

static int
ugen_open_pipe_write(struct usb2_fifo *f)
{
	struct usb2_config usb2_config[2];
	struct usb2_pipe *pipe = f->priv_sc0;
	struct usb2_endpoint_descriptor *ed = pipe->edesc;

	mtx_assert(f->priv_mtx, MA_OWNED);

	if (f->xfer[0] || f->xfer[1]) {
		/* transfers are already opened */
		return (0);
	}
	bzero(usb2_config, sizeof(usb2_config));

	usb2_config[1].type = UE_CONTROL;
	usb2_config[1].endpoint = 0;
	usb2_config[1].direction = UE_DIR_ANY;
	usb2_config[1].mh.timeout = 1000;	/* 1 second */
	usb2_config[1].mh.interval = 50;/* 50 milliseconds */
	usb2_config[1].mh.bufsize = sizeof(struct usb2_device_request);
	usb2_config[1].mh.callback = &ugen_write_clear_stall_callback;

	usb2_config[0].type = ed->bmAttributes & UE_XFERTYPE;
	usb2_config[0].endpoint = ed->bEndpointAddress & UE_ADDR;
	usb2_config[0].direction = ed->bEndpointAddress & (UE_DIR_OUT | UE_DIR_IN);
	usb2_config[0].mh.interval = USB_DEFAULT_INTERVAL;
	usb2_config[0].mh.flags.proxy_buffer = 1;

	switch (ed->bmAttributes & UE_XFERTYPE) {
	case UE_INTERRUPT:
	case UE_BULK:
		if (f->flag_short) {
			usb2_config[0].mh.flags.force_short_xfer = 1;
		}
		usb2_config[0].mh.callback = &ugen_default_write_callback;
		usb2_config[0].mh.timeout = f->timeout;
		usb2_config[0].mh.frames = 1;
		usb2_config[0].mh.bufsize = f->bufsize;
		usb2_config[0].md = usb2_config[0].mh;	/* symmetric config */
		if (ugen_transfer_setup(f, usb2_config, 2)) {
			return (EIO);
		}
		/* first transfer clears stall */
		f->flag_stall = 1;
		break;

	case UE_ISOCHRONOUS:
		usb2_config[0].mh.flags.short_xfer_ok = 1;
		usb2_config[0].mh.bufsize = 0;	/* use default */
		usb2_config[0].mh.frames = f->nframes;
		usb2_config[0].mh.callback = &ugen_isoc_write_callback;
		usb2_config[0].mh.timeout = 0;
		usb2_config[0].md = usb2_config[0].mh;	/* symmetric config */

		/* clone configuration */
		usb2_config[1] = usb2_config[0];

		if (ugen_transfer_setup(f, usb2_config, 2)) {
			return (EIO);
		}
		break;

	default:
		return (EINVAL);
	}
	return (0);
}

static int
ugen_open_pipe_read(struct usb2_fifo *f)
{
	struct usb2_config usb2_config[2];
	struct usb2_pipe *pipe = f->priv_sc0;
	struct usb2_endpoint_descriptor *ed = pipe->edesc;

	mtx_assert(f->priv_mtx, MA_OWNED);

	if (f->xfer[0] || f->xfer[1]) {
		/* transfers are already opened */
		return (0);
	}
	bzero(usb2_config, sizeof(usb2_config));

	usb2_config[1].type = UE_CONTROL;
	usb2_config[1].endpoint = 0;
	usb2_config[1].direction = UE_DIR_ANY;
	usb2_config[1].mh.timeout = 1000;	/* 1 second */
	usb2_config[1].mh.interval = 50;/* 50 milliseconds */
	usb2_config[1].mh.bufsize = sizeof(struct usb2_device_request);
	usb2_config[1].mh.callback = &ugen_read_clear_stall_callback;

	usb2_config[0].type = ed->bmAttributes & UE_XFERTYPE;
	usb2_config[0].endpoint = ed->bEndpointAddress & UE_ADDR;
	usb2_config[0].direction = UE_DIR_IN;
	usb2_config[0].mh.interval = USB_DEFAULT_INTERVAL;
	usb2_config[0].mh.flags.proxy_buffer = 1;

	switch (ed->bmAttributes & UE_XFERTYPE) {
	case UE_INTERRUPT:
	case UE_BULK:
		if (f->flag_short) {
			usb2_config[0].mh.flags.short_xfer_ok = 1;
		}
		usb2_config[0].mh.timeout = f->timeout;
		usb2_config[0].mh.frames = 1;
		usb2_config[0].mh.callback = &ugen_default_read_callback;
		usb2_config[0].mh.bufsize = f->bufsize;
		usb2_config[0].md = usb2_config[0].mh;	/* symmetric config */

		if (ugen_transfer_setup(f, usb2_config, 2)) {
			return (EIO);
		}
		/* first transfer clears stall */
		f->flag_stall = 1;
		break;

	case UE_ISOCHRONOUS:
		usb2_config[0].mh.flags.short_xfer_ok = 1;
		usb2_config[0].mh.bufsize = 0;	/* use default */
		usb2_config[0].mh.frames = f->nframes;
		usb2_config[0].mh.callback = &ugen_isoc_read_callback;
		usb2_config[0].mh.timeout = 0;
		usb2_config[0].md = usb2_config[0].mh;	/* symmetric config */

		/* clone configuration */
		usb2_config[1] = usb2_config[0];

		if (ugen_transfer_setup(f, usb2_config, 2)) {
			return (EIO);
		}
		break;

	default:
		return (EINVAL);
	}
	return (0);
}

static void
ugen_start_read(struct usb2_fifo *f)
{
	/* check that pipes are open */
	if (ugen_open_pipe_read(f)) {
		/* signal error */
		usb2_fifo_put_data_error(f);
	}
	/* start transfers */
	usb2_transfer_start(f->xfer[0]);
	usb2_transfer_start(f->xfer[1]);
	return;
}

static void
ugen_start_write(struct usb2_fifo *f)
{
	/* check that pipes are open */
	if (ugen_open_pipe_write(f)) {
		/* signal error */
		usb2_fifo_get_data_error(f);
	}
	/* start transfers */
	usb2_transfer_start(f->xfer[0]);
	usb2_transfer_start(f->xfer[1]);
	return;
}

static void
ugen_stop_io(struct usb2_fifo *f)
{
	/* stop transfers */
	usb2_transfer_stop(f->xfer[0]);
	usb2_transfer_stop(f->xfer[1]);
	return;
}

static void
ugen_default_read_callback(struct usb2_xfer *xfer)
{
	struct usb2_fifo *f = xfer->priv_sc;
	struct usb2_mbuf *m;

	DPRINTF(3, "actlen=%u, aframes=%u\n", xfer->actlen, xfer->aframes);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		if (xfer->actlen == 0) {
			if (f->fifo_zlp != 4) {
				f->fifo_zlp++;
			} else {
				/*
				 * Throttle a little bit we have multiple ZLPs
				 * in a row!
				 */
				xfer->interval = 64;	/* ms */
			}
		} else {
			/* clear throttle */
			xfer->interval = 0;
			f->fifo_zlp = 0;
		}
		usb2_fifo_put_data(f, xfer->frbuffers, 0,
		    xfer->actlen, 1);

	case USB_ST_SETUP:
		if (f->flag_stall) {
			usb2_transfer_start(f->xfer[1]);
			break;
		}
		USB_IF_POLL(&(f->free_q), m);
		if (m) {
			xfer->frlengths[0] = xfer->max_data_length;
			usb2_start_hardware(xfer);
		}
		break;

	default:			/* Error */
		if (xfer->error != USB_ERR_CANCELLED) {
			f->flag_stall = 1;
			f->fifo_zlp = 0;
			usb2_transfer_start(f->xfer[1]);
		}
		break;
	}
	return;
}

static void
ugen_default_write_callback(struct usb2_xfer *xfer)
{
	struct usb2_fifo *f = xfer->priv_sc;
	uint32_t actlen;

	DPRINTF(3, "actlen=%u, aframes=%u\n", xfer->actlen, xfer->aframes);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_SETUP:
	case USB_ST_TRANSFERRED:
		/*
		 * If writing is in stall, just jump to clear stall callback and
		 * solve the situation.
		 */
		if (f->flag_stall) {
			usb2_transfer_start(f->xfer[1]);
			break;
		}
		/*
		 * Write data, setup and perform hardware transfer.
		 */
		if (usb2_fifo_get_data(f, xfer->frbuffers, 0,
		    xfer->max_data_length, &actlen, 0)) {
			xfer->frlengths[0] = actlen;
			usb2_start_hardware(xfer);
		}
		break;

	default:			/* Error */
		if (xfer->error != USB_ERR_CANCELLED) {
			f->flag_stall = 1;
			usb2_transfer_start(f->xfer[1]);
		}
		break;
	}
	return;
}

static void
ugen_read_clear_stall_callback(struct usb2_xfer *xfer)
{
	struct usb2_fifo *f = xfer->priv_sc;
	struct usb2_xfer *xfer_other = f->xfer[0];

	if (f->flag_stall == 0) {
		/* nothing to do */
		return;
	}
	if (usb2_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(4, "f=%p: stall cleared\n", f);
		f->flag_stall = 0;
		usb2_transfer_start(xfer_other);
	}
	return;
}

static void
ugen_write_clear_stall_callback(struct usb2_xfer *xfer)
{
	struct usb2_fifo *f = xfer->priv_sc;
	struct usb2_xfer *xfer_other = f->xfer[0];

	if (f->flag_stall == 0) {
		/* nothing to do */
		return;
	}
	if (usb2_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(4, "f=%p: stall cleared\n", f);
		f->flag_stall = 0;
		usb2_transfer_start(xfer_other);
	}
	return;
}

static void
ugen_isoc_read_callback(struct usb2_xfer *xfer)
{
	struct usb2_fifo *f = xfer->priv_sc;
	uint32_t offset;
	uint16_t n;

	DPRINTF(3, "actlen=%u, aframes=%u\n", xfer->actlen, xfer->aframes);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:

		DPRINTF(5, "actlen=%d\n", xfer->actlen);

		offset = 0;

		for (n = 0; n != xfer->aframes; n++) {
			usb2_fifo_put_data(f, xfer->frbuffers, offset,
			    xfer->frlengths[n], 1);
			offset += xfer->max_frame_size;
		}

	case USB_ST_SETUP:
tr_setup:
		for (n = 0; n != xfer->nframes; n++) {
			/* setup size for next transfer */
			xfer->frlengths[n] = xfer->max_frame_size;
		}
		usb2_start_hardware(xfer);
		break;

	default:			/* Error */
		if (xfer->error == USB_ERR_CANCELLED) {
			break;
		}
		goto tr_setup;
	}
	return;
}

static void
ugen_isoc_write_callback(struct usb2_xfer *xfer)
{
	struct usb2_fifo *f = xfer->priv_sc;
	uint32_t actlen;
	uint32_t offset;
	uint16_t n;

	DPRINTF(3, "actlen=%u, aframes=%u\n", xfer->actlen, xfer->aframes);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
	case USB_ST_SETUP:
tr_setup:
		offset = 0;
		for (n = 0; n != xfer->nframes; n++) {
			if (usb2_fifo_get_data(f, xfer->frbuffers, offset,
			    xfer->max_frame_size, &actlen, 1)) {
				xfer->frlengths[n] = actlen;
				offset += actlen;
			} else {
				break;
			}
		}

		for (; n != xfer->nframes; n++) {
			/* fill in zero frames */
			xfer->frlengths[n] = 0;
		}
		usb2_start_hardware(xfer);
		break;

	default:			/* Error */
		if (xfer->error == USB_ERR_CANCELLED) {
			break;
		}
		goto tr_setup;
	}
	return;
}

static int
ugen_set_config(struct usb2_fifo *f, uint8_t index)
{
	DPRINTF(1, "index %u\n", index);

	if (f->fifo_index >= 2) {
		/* not the control endpoint - just forget it */
		return (EINVAL);
	}
	if (f->udev->flags.usb2_mode != USB_MODE_HOST) {
		/* not possible in device side mode */
		return (ENOTTY);
	}
	if (f->udev->curr_config_index == index) {
		/* no change needed */
		return (0);
	}
	/* change setting - will free generic FIFOs, if any */
	if (usb2_set_config_index(f->udev, index)) {
		return (EIO);
	}
	/* probe and attach */
	if (usb2_probe_and_attach(f->udev, USB_IFACE_INDEX_ANY)) {
		return (EIO);
	}
	return (0);
}

static int
ugen_set_interface(struct usb2_fifo *f,
    uint8_t iface_index, uint8_t alt_index)
{
	DPRINTF(1, "%u, %u\n", iface_index, alt_index);

	if (f->fifo_index >= 2) {
		/* not the control endpoint - just forget it */
		return (EINVAL);
	}
	if (f->udev->flags.usb2_mode != USB_MODE_HOST) {
		/* not possible in device side mode */
		return (ENOTTY);
	}
	/* change setting - will free generic FIFOs, if any */
	if (usb2_set_alt_interface_index(f->udev, iface_index, alt_index)) {
		return (EIO);
	}
	/* probe and attach */
	if (usb2_probe_and_attach(f->udev, iface_index)) {
		return (EIO);
	}
	return (0);
}

/*------------------------------------------------------------------------*
 *	ugen_get_cdesc
 *
 * This function will retrieve the complete configuration descriptor
 * at the given index.
 *------------------------------------------------------------------------*/
static int
ugen_get_cdesc(struct usb2_fifo *f, struct usb2_gen_descriptor *ugd)
{
	struct usb2_config_descriptor *cdesc;
	struct usb2_device *udev = f->udev;
	int error;
	uint16_t len;
	uint8_t free_data;

	DPRINTF(5, "\n");

	if (f->fifo_index >= 2) {
		/* control endpoint only */
		return (EINVAL);
	}
	if (ugd->ugd_data == NULL) {
		/* userland pointer should not be zero */
		return (EINVAL);
	}
	if ((ugd->ugd_config_index == USB_UNCONFIG_INDEX) ||
	    (ugd->ugd_config_index == udev->curr_config_index)) {
		cdesc = usb2_get_config_descriptor(udev);
		if (cdesc == NULL) {
			return (ENXIO);
		}
		free_data = 0;

	} else {
		if (usb2_req_get_config_desc_full(udev,
		    &Giant, &cdesc, M_USBDEV,
		    ugd->ugd_config_index)) {
			return (ENXIO);
		}
		free_data = 1;
	}

	len = UGETW(cdesc->wTotalLength);
	if (len > ugd->ugd_maxlen) {
		len = ugd->ugd_maxlen;
	}
	DPRINTF(5, "len=%u\n", len);

	ugd->ugd_actlen = len;
	ugd->ugd_offset = 0;

	error = copyout(cdesc, ugd->ugd_data, len);

	if (free_data) {
		free(cdesc, M_USBDEV);
	}
	return (error);
}

static int
ugen_get_sdesc(struct usb2_fifo *f, struct usb2_gen_descriptor *ugd)
{
	void *ptr = f->udev->bus->scratch[0].data;
	uint16_t size = sizeof(f->udev->bus->scratch[0].data);
	int error;

	if (f->fifo_index >= 2) {
		/* control endpoint only */
		return (EINVAL);
	}
	if (usb2_req_get_string_desc(f->udev, &Giant, ptr,
	    size, ugd->ugd_lang_id, ugd->ugd_string_index)) {
		error = EINVAL;
	} else {

		if (size > ((uint8_t *)ptr)[0]) {
			size = ((uint8_t *)ptr)[0];
		}
		if (size > ugd->ugd_maxlen) {
			size = ugd->ugd_maxlen;
		}
		ugd->ugd_actlen = size;
		ugd->ugd_offset = 0;

		error = copyout(ptr, ugd->ugd_data, size);
	}
	return (error);
}

/*------------------------------------------------------------------------*
 *	usb2_gen_fill_deviceinfo
 *
 * This function dumps information about an USB device to the
 * structure pointed to by the "di" argument.
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static int
usb2_gen_fill_deviceinfo(struct usb2_fifo *f, struct usb2_device_info *di)
{
	enum {
		MAX_PORT = (sizeof(di->udi_ports) / sizeof(di->udi_ports[0])),
	};
	struct usb2_port *p;
	struct usb2_interface *iface;
	struct usb2_device *child;
	struct usb2_device *udev;
	uint8_t i;
	uint8_t max;

	if (f->fifo_index >= 2) {
		/* control endpoint only */
		return (EINVAL);
	}
	udev = f->udev;

	bzero(di, sizeof(di[0]));

	di->udi_bus = device_get_unit(udev->bus->bdev);
	di->udi_addr = udev->address;
	strlcpy(di->udi_vendor, udev->manufacturer,
	    sizeof(di->udi_vendor));
	strlcpy(di->udi_product, udev->product,
	    sizeof(di->udi_product));
	usb2_printBCD(di->udi_release, sizeof(di->udi_release),
	    UGETW(udev->ddesc.bcdDevice));
	di->udi_vendorNo = UGETW(udev->ddesc.idVendor);
	di->udi_productNo = UGETW(udev->ddesc.idProduct);
	di->udi_releaseNo = UGETW(udev->ddesc.bcdDevice);
	di->udi_class = udev->ddesc.bDeviceClass;
	di->udi_subclass = udev->ddesc.bDeviceSubClass;
	di->udi_protocol = udev->ddesc.bDeviceProtocol;
	di->udi_config = udev->curr_config_no;
	di->udi_power = udev->flags.self_powered ? 0 : udev->power;
	di->udi_speed = udev->speed;

	for (i = 0; i != MIN(USB_MAX_DEVNAMES, USB_IFACE_MAX); i++) {
		iface = usb2_get_iface(udev, i);
		if (iface && (iface->subdev != NULL) &&
		    device_is_attached(iface->subdev)) {
			strlcpy(di->udi_devnames[i],
			    device_get_nameunit(iface->subdev),
			    USB_MAX_DEVNAMELEN);
		}
	}

	if (udev->hub) {

		max = udev->hub->nports;
		if (max > MAX_PORT) {
			max = MAX_PORT;
		}
		di->udi_nports = max;
		p = udev->hub->ports;
		for (i = 0; i != max; i++, p++) {

			child = usb2_bus_port_get_device(udev->bus, p);

			if (child) {
				di->udi_ports[i] = p->device_index;
			} else {
				di->udi_ports[i] = USB_PORT_POWERED;
			}
		}
	}
	return (0);
}

int
ugen_do_request(struct usb2_fifo *f, struct usb2_ctl_request *ur)
{
	int error;
	uint16_t len;
	uint16_t actlen;
	uint8_t isread;
	void *data = NULL;

	if (f->fifo_index >= 2) {
		/* control endpoint only */
		return (EINVAL);
	}
	/* avoid requests that would damage the bus integrity */
	if (((ur->ucr_request.bmRequestType == UT_WRITE_DEVICE) &&
	    (ur->ucr_request.bRequest == UR_SET_ADDRESS)) ||
	    ((ur->ucr_request.bmRequestType == UT_WRITE_DEVICE) &&
	    (ur->ucr_request.bRequest == UR_SET_CONFIG)) ||
	    ((ur->ucr_request.bmRequestType == UT_WRITE_INTERFACE) &&
	    (ur->ucr_request.bRequest == UR_SET_INTERFACE))) {
		if (suser(curthread)) {
			return (EPERM);
		}
	}
	/*
	 * Clearing the stall this way is not allowed, hence it does
	 * not update the data toggle value in "struct usb2_pipe" !
	 */
	if (ur->ucr_request.bmRequestType == UT_WRITE_ENDPOINT) {
		if (suser(curthread)) {
			return (EPERM);
		}
	}
	/* TODO: add more checks to verify the interface index */

	len = UGETW(ur->ucr_request.wLength);
	isread = (ur->ucr_request.bmRequestType & UT_READ) ? 1 : 0;

	if (len != 0) {
		if (ur->ucr_data == NULL) {
			return (EINVAL);
		}
		data = malloc(len, M_USBDEV, M_WAITOK);
		if (data == NULL) {
			error = ENOMEM;
			goto done;
		}
		if (!(ur->ucr_request.bmRequestType & UT_READ)) {
			error = copyin(ur->ucr_data, data, len);
			if (error) {
				goto done;
			}
		}
	}
	error = usb2_do_request_flags
	    (f->udev, NULL, &ur->ucr_request, data,
	    (ur->ucr_flags & USB_SHORT_XFER_OK), &actlen,
	    USB_DEFAULT_TIMEOUT);

	ur->ucr_actlen = actlen;

	if (error) {
		error = EIO;
		goto done;
	}
	if ((len != 0) && (ur->ucr_request.bmRequestType & UT_READ)) {
		error = copyout(data, ur->ucr_data, len);
		if (error) {
			goto done;
		}
	}
done:
	if (data) {
		free(M_USBDEV, data);
	}
	return (error);
}

/*------------------------------------------------------------------------
 *	ugen_re_enumerate
 *
 * NOTE: This function will currently not restore the device
 * configuration.
 *------------------------------------------------------------------------*/
static int
ugen_re_enumerate(struct usb2_fifo *f)
{
	struct usb2_device *parent_hub;
	struct usb2_device *udev = f->udev;
	int error;
	uint8_t old_addr;
	uint8_t buf[8];

	if (f->fifo_index >= 2) {
		/* control endpoint only */
		return (EINVAL);
	}
	if (suser(curthread)) {
		return (EPERM);
	}
	old_addr = udev->address;
	parent_hub = udev->parent_hub;
	if (parent_hub == NULL) {
		error = EINVAL;
		goto done;
	}
	error = usb2_req_reset_port(parent_hub, &Giant, udev->port_no);
	if (error) {
		error = ENXIO;
		goto done;
	}
	/*
	 * After that the port has been reset our device should be at
	 * address zero:
	 */
	udev->address = USB_START_ADDR;

	/*
	 * It should be allowed to read some descriptors from address
	 * zero:
	 */
	error = usb2_req_get_desc(udev, &Giant, buf,
	    8, 8, 0, UDESC_DEVICE, 0, 0);
	if (error) {
		error = ENXIO;
		goto done;
	}
	/*
	 * Restore device address:
	 */
	error = usb2_req_set_address(udev, &Giant, old_addr);
	if (error) {
		error = ENXIO;
		goto done;
	}
done:
	/* restore address */
	udev->address = old_addr;

	return (error);
}

static int
ugen_iface_ioctl(struct usb2_fifo *f, u_long cmd, void *addr)
{
	int error = 0;

	switch (cmd) {
	case USB_SET_SHORT_XFER:
		if (f->xfer[0] || f->xfer[1]) {
			/* cannot change this during transfer */
			error = EBUSY;
			break;
		}
		if (*(int *)addr)
			f->flag_short = 1;
		else
			f->flag_short = 0;
		break;

	case USB_SET_TIMEOUT:
		f->timeout = *(int *)addr;
		if (f->timeout > 65535) {
			/* limit user input */
			f->timeout = 65535;
		}
		break;

	case USB_GET_FRAME_SIZE:
		if (f->xfer[0]) {
			*(int *)addr = f->xfer[0]->max_frame_size;
		} else {
			error = EINVAL;
		}
		break;

	case USB_SET_BUFFER_SIZE:
		if (f->xfer[0] || f->xfer[1]) {
			/* cannot change this during transfer */
			error = EBUSY;
			break;
		}
		if (*(int *)addr < 1024)
			f->bufsize = 1024;
		else if (*(int *)addr < (256 * 1024))
			f->bufsize = *(int *)addr;
		else
			f->bufsize = 256 * 1024;
		break;

	case USB_GET_BUFFER_SIZE:
		*(int *)addr = f->bufsize;
		break;

	case USB_GET_INTERFACE_DESC:{
			struct usb2_interface_descriptor *idesc = addr;
			struct usb2_interface *iface;

			iface = usb2_get_iface(f->udev, f->iface_index);
			if (iface && iface->idesc) {
				*idesc = *(iface->idesc);
			} else {
				error = EIO;
				break;
			}
			break;
		}

	case USB_GET_ENDPOINT_DESC:{
			struct usb2_endpoint_descriptor *ed = addr;
			struct usb2_pipe *pipe = f->priv_sc0;

			if (pipe && pipe->edesc) {
				*ed = *(pipe->edesc);
			} else {
				error = EINVAL;
				break;
			}
			break;
		}

	default:
		error = ENOTTY;
		break;
	}
	return (error);
}

static int
ugen_ctrl_ioctl(struct usb2_fifo *f, u_long cmd, void *addr, int fflags)
{
	uint8_t n;
	int error = 0;

	switch (cmd) {
	case USB_DISCOVER:
		usb2_needs_explore_all();
		break;

	case USB_SETDEBUG:
		if (!(fflags & FWRITE)) {
			error = EPERM;
			break;
		}
		usb2_debug = *(int *)addr;
		break;

	case USB_GET_CONFIG:
		*(int *)addr = f->udev->curr_config_index;
		break;

	case USB_SET_CONFIG:
		if (!(fflags & FWRITE)) {
			error = EPERM;
			break;
		}
		error = ugen_set_config(f, *(int *)addr);
		break;

	case USB_GET_ALTINTERFACE:{
			struct usb2_alt_interface *ai = addr;
			struct usb2_interface *iface;

			iface = usb2_get_iface(f->udev, ai->uai_interface_index);
			if (iface && iface->idesc) {
				ai->uai_alt_index = iface->alt_index;
			} else {
				error = EINVAL;
			}
			break;
		}

	case USB_SET_ALTINTERFACE:{
			struct usb2_alt_interface *ai = addr;

			if (!(fflags & FWRITE)) {
				error = EPERM;
				break;
			}
			error = ugen_set_interface(f, ai->uai_interface_index, ai->uai_alt_index);
			break;
		}

	case USB_GET_DEVICE_DESC:{
			struct usb2_device_descriptor *ddesc = addr;
			struct usb2_device_descriptor *temp;

			temp = usb2_get_device_descriptor(f->udev);
			if (!temp) {
				error = EIO;
				break;
			}
			*ddesc = *temp;
			break;
		}

	case USB_GET_CONFIG_DESC:{
			struct usb2_config_descriptor *cdesc = addr;
			struct usb2_config_descriptor *temp;

			temp = usb2_get_config_descriptor(f->udev);
			if (!temp) {
				error = EIO;
				break;
			}
			*cdesc = *temp;
			break;
		}

	case USB_GET_FULL_DESC:
		error = ugen_get_cdesc(f, addr);
		break;

	case USB_GET_STRING_DESC:
		error = ugen_get_sdesc(f, addr);
		break;

	case USB_REQUEST:
	case USB_DO_REQUEST:
		if (!(fflags & FWRITE)) {
			error = EPERM;
			break;
		}
		error = ugen_do_request(f, addr);
		break;

	case USB_DEVICEINFO:
	case USB_GET_DEVICEINFO:
		error = usb2_gen_fill_deviceinfo(f, addr);
		break;

	case USB_DEVICESTATS:{
			struct usb2_device_stats *ustat = addr;

			for (n = 0; n != 4; n++) {

				ustat->uds_requests_fail[n] =
				    f->udev->bus->stats_err.uds_requests[n];

				ustat->uds_requests_ok[n] =
				    f->udev->bus->stats_ok.uds_requests[n];
			}
			break;
		}

	case USB_DEVICEENUMERATE:
		error = ugen_re_enumerate(f);
		break;

		/* ... more IOCTL's to come ! ... --hps */

	default:
		error = EINVAL;
		break;
	}
	return (error);
}

static int
ugen_ioctl(struct usb2_fifo *f, u_long cmd, void *addr, int fflags, struct thread *td)
{
	int error;

	DPRINTF(5, "cmd=%08lx\n", cmd);
	if (f->fifo_index >= 2) {
		mtx_lock(f->priv_mtx);
		error = ugen_iface_ioctl(f, cmd, addr);
		mtx_unlock(f->priv_mtx);
		return (error);
	} else {
		error = ugen_ctrl_ioctl(f, cmd, addr, fflags);
	}
	DPRINTF(5, "error=%d\n", error);
	return (error);
}