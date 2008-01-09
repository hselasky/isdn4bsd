/*	$NetBSD: ugen.c,v 1.79 2006/03/01 12:38:13 yamt Exp $	*/

/* Also already merged from NetBSD:
 *	$NetBSD: ugen.c,v 1.61 2002/09/23 05:51:20 simonb Exp $
 *	$NetBSD: ugen.c,v 1.64 2003/06/28 14:21:46 darrenr Exp $
 *	$NetBSD: ugen.c,v 1.65 2003/06/29 22:30:56 fvdl Exp $
 *	$NetBSD: ugen.c,v 1.68 2004/06/23 02:30:52 mycroft Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ugen.c,v 1.111 2007/06/28 06:22:40 imp Exp $");

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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/ioccom.h>
#include <sys/filio.h>
#include <sys/tty.h>
#include <sys/file.h>
#include <sys/vnode.h>
#include <sys/poll.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#define	UGEN_BULK_BUFFER_SIZE	(1024*64)	/* bytes */

#define	UGEN_HW_FRAMES	50		/* number of milliseconds per transfer */

struct ugen_frame_ring {
	uint16_t input_index;
	uint16_t output_index;
	uint16_t end_index;		/* exclusive */
	uint16_t frame_size;
	uint16_t *frlengths;
	void   *buf;
};

struct ugen_endpoint {
	struct ugen_softc *sc;
	struct cdev *dev;
	struct usbd_pipe *pipe_in;	/* pipe for reading data from USB */
	struct usbd_pipe *pipe_out;	/* pipe for writing data to USB */
	struct usbd_xfer *xfer_in[2];
	struct usbd_xfer *xfer_out[2];

	struct ugen_frame_ring in_queue;/* (isoc/interrupt) */
	struct ugen_frame_ring out_queue;	/* (isoc) */
	uint32_t in_timeout;		/* (bulk/interrupt) */
	uint32_t in_frames;		/* number of frames to use (isoc) */
	uint32_t out_frames;		/* number of frames to use (isoc) */
	uint16_t out_frame_size;	/* maximum frame size (isoc) */

	uint32_t io_buffer_size;	/* (bulk) */

	struct selinfo selinfo;

	uint16_t state;
#define	UGEN_OPEN_IN      0x0001
#define	UGEN_OPEN_OUT     0x0002
#define	UGEN_OPEN_DEV     0x0004
#define	UGEN_CLOSING      0x0008
#define	UGEN_GONE         0x0010
#define	UGEN_SHORT_OK     0x0020	/* short xfers are OK */

	/* context bits */
#define	UGEN_RD_CFG	  0x0040
#define	UGEN_RD_SLP	  0x0080	/* sleep entered */
#define	UGEN_RD_WUP	  0x0100	/* need wakeup */
#define	UGEN_RD_UIO	  0x0200

	/* context bits */
#define	UGEN_WR_CFG	  0x0400
#define	UGEN_WR_SLP	  0x0800
#define	UGEN_WR_WUP	  0x1000
#define	UGEN_WR_UIO	  0x2000

	/* context bit */
#define	UGEN_IOCTL	  0x4000

	uint8_t	read_stall:1;
	uint8_t	write_stall:1;
	uint8_t	unused:6;
};

struct ugen_softc {
	device_t sc_dev;
	struct usbd_device *sc_udev;
	struct ugen_endpoint sc_endpoints[USB_MAX_ENDPOINTS];
	struct ugen_endpoint sc_endpoints_end[0];
	struct mtx sc_mtx;
};

extern cdevsw_t ugen_cdevsw;

/* prototypes */

static device_probe_t ugen_probe;
static device_attach_t ugen_attach;
static device_detach_t ugen_detach;

static usbd_callback_t ugen_interrupt_callback;
static usbd_callback_t ugen_read_clear_stall_callback;
static usbd_callback_t ugen_write_clear_stall_callback;
static usbd_callback_t ugen_default_read_callback;
static usbd_callback_t ugen_default_write_callback;
static usbd_callback_t ugenisoc_read_callback;
static usbd_callback_t ugenisoc_write_callback;

static void
	ugen_make_devnodes(struct ugen_softc *sc);

static void
	ugen_destroy_devnodes(struct ugen_softc *sc, int skip_first);

static int
	ugen_set_config(struct ugen_softc *sc, int configno);

static int
	ugen_set_interface(struct ugen_softc *sc, int ifaceidx, int altno);

static usb_config_descriptor_t *
	ugen_get_cdesc(struct usbd_device *udev, int index, int *lenp);

static int
	ugen_get_alt_index(struct usbd_device *udev, int ifaceidx);

#define	UGENMINOR(unit, endpoint) (((unit) << 4) | (endpoint))

#define	DEV2SC(dev) ((dev)->si_drv1)
#define	DEV2SCE(dev) ((dev)->si_drv2)

static int
ugen_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->usegeneric)
		return (UMATCH_GENERIC);
	else
		return (UMATCH_NONE);
}

static int
ugen_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ugen_softc *sc = device_get_softc(dev);
	struct ugen_endpoint *sce;
	int conf;

	sc->sc_dev = dev;
	sc->sc_udev = uaa->device;

	mtx_init(&sc->sc_mtx, "ugen lock",
	    NULL, MTX_DEF | MTX_RECURSE);

	usbd_set_device_desc(dev);

	/* first set configuration index 0, the default one for ugen */
	if (usbd_set_config_index(sc->sc_udev, 0, 0)) {
		device_printf(dev, "setting configuration index 0 failed\n");
		return (ENXIO);
	}
	conf = usbd_get_config_descriptor(sc->sc_udev)->bConfigurationValue;

	/* set up all the local state for this configuration */
	if (ugen_set_config(sc, conf)) {
		device_printf(dev, "setting configuration "
		    "%d failed\n", conf);
		return (ENXIO);
	}
	sce = &sc->sc_endpoints[0];

	/* make control endpoint */
	sce->dev = make_dev(&ugen_cdevsw, UGENMINOR(device_get_unit(sc->sc_dev), 0),
	    UID_ROOT, GID_OPERATOR, 0644, "%s",
	    device_get_nameunit(sc->sc_dev));
	if (sce->dev) {
		DEV2SC(sce->dev) = sc;
		DEV2SCE(sce->dev) = sce;
	}
	return (0);			/* success */
}

static int
ugen_detach(device_t dev)
{
	struct ugen_softc *sc = device_get_softc(dev);
	struct ugen_endpoint *sce = &sc->sc_endpoints[0];
	struct ugen_endpoint *sce_end = &sc->sc_endpoints_end[0];

	mtx_lock(&sc->sc_mtx);
	while (sce < sce_end) {
		sce->state |= UGEN_GONE;
		sce++;
	}
	mtx_unlock(&sc->sc_mtx);

	/* destroy all devices */
	ugen_destroy_devnodes(sc, 0);

	mtx_destroy(&sc->sc_mtx);

	return (0);
}

#define	ugen_inc_input_index(ufr) \
{\
  (ufr)->input_index++;\
  if ((ufr)->input_index >= (ufr)->end_index)\
	(ufr)->input_index = 0;\
}

#define	ugen_inc_output_index(ufr) \
{\
  (ufr)->output_index++;\
  if ((ufr)->output_index >= (ufr)->end_index)\
	(ufr)->output_index = 0;\
}

#define	ugen_half_empty(ufr) \
((ufr)->end_index ? \
((((ufr)->end_index + \
   (ufr)->input_index - \
   (ufr)->output_index) % (ufr)->end_index) \
 < ((ufr)->end_index / 2)) : 0)

#define	ugen_not_empty(ufr) \
((ufr)->output_index != \
 (ufr)->input_index)

static void
ugen_get_input_block(struct ugen_frame_ring *ufr, void **ptr, uint16_t **len)
{
	uint16_t next;

	next = ufr->input_index + 1;
	if (next >= ufr->end_index) {
		next = 0;
	}
	if (next == ufr->output_index) {
		/* buffer full */
		*ptr = 0;
		*len = 0;
	} else {
		*ptr = USBD_ADD_BYTES
		    (ufr->buf,
		    ufr->frame_size * ufr->input_index);

		*len = &ufr->frlengths[ufr->input_index];

		/* set default block length */
		ufr->frlengths[ufr->input_index] = ufr->frame_size;
	}
	return;
}

static void
ugen_get_output_block(struct ugen_frame_ring *ufr, void **ptr, uint16_t *len)
{
	if (ufr->output_index == ufr->input_index) {
		/* buffer empty */
		*ptr = 0;
		*len = 0;
	} else {
		*ptr = USBD_ADD_BYTES
		    (ufr->buf,
		    ufr->frame_size * ufr->output_index);

		*len = ufr->frlengths[ufr->output_index];
	}
	return;
}

static usbd_status_t
ugen_allocate_blocks(struct ugen_softc *sc,
    struct ugen_endpoint *sce,
    struct ugen_frame_ring *ufr,
    uint16_t frames,
    uint16_t frame_size)
{
	void *ptr;

	mtx_lock(&(sc->sc_mtx));
	bzero(ufr, sizeof(*ufr));
	mtx_unlock(&(sc->sc_mtx));

	if (frames == 0) {
		return (USBD_ERR_INVAL);
	}
	/*
	 * one frame will always be unused to make things simple, so
	 * allocate one extra frame:
	 */
	frames++;

	ptr = malloc(frames * (frame_size + sizeof(uint16_t)),
	    M_USBDEV, M_WAITOK);

	if (ptr == NULL) {
		return (USBD_ERR_NOMEM);
	}
	mtx_lock(&(sc->sc_mtx));
	ufr->end_index = frames;
	ufr->frame_size = frame_size;
	ufr->frlengths = USBD_ADD_BYTES(ptr, frames * frame_size);
	ufr->buf = USBD_ADD_BYTES(ptr, 0);
	mtx_unlock(&(sc->sc_mtx));
	return (0);
}

static void
ugen_free_blocks(struct ugen_frame_ring *ufr)
{
	if (ufr->buf) {
		free(ufr->buf, M_USBDEV);
		ufr->buf = NULL;
	}
	return;
}

static usbd_status_t
ugen_transfer_setup(struct ugen_softc *sc,
    struct ugen_endpoint *sce,
    uint16_t context_bit,
    struct usbd_device *udev,
    uint8_t iface_index,
    struct usbd_xfer **pxfer,
    const struct usbd_config *setup,
    uint8_t n_setup,
    uint16_t n_in_frames,
    uint16_t n_out_frames)
{
	struct usbd_xfer *temp[n_setup];

	usbd_status_t error;

	if ((n_out_frames > 0) && (n_in_frames > 0)) {
		/* should not happen */
		return (USBD_ERR_INVAL);
	}
	sce->state |= context_bit;

	mtx_unlock(&sc->sc_mtx);

	/*
	 * "usbd_transfer_setup()" can sleep so one needs to make a wrapper,
	 * exiting the mutex and checking things
	 */
	error = usbd_transfer_setup(udev, &iface_index, &temp[0],
	    setup, n_setup, sce, &(sc->sc_mtx));
	if (error == 0) {
		if (n_in_frames > 0) {
			error = ugen_allocate_blocks
			    (sc, sce, &(sce->in_queue), n_in_frames,
			    temp[0]->max_frame_size);
		} else if (n_out_frames > 0) {
			error = ugen_allocate_blocks
			    (sc, sce, &(sce->out_queue), n_out_frames,
			    temp[0]->max_frame_size);
		}
		if (error) {
			usbd_transfer_unsetup(temp, n_setup);
		}
	}
	mtx_lock(&sc->sc_mtx);

	if (sce->state & UGEN_CLOSING) {
		mtx_unlock(&(sc->sc_mtx));

		/* "usbd_transfer_unsetup()" will clear "temp[]" */
		usbd_transfer_unsetup(&temp[0], n_setup);

		mtx_lock(&(sc->sc_mtx));

		wakeup(sce);
		error = USBD_ERR_CANCELLED;
	}
	sce->state &= ~context_bit;

	while (n_setup--) {
		pxfer[n_setup] = temp[n_setup];
	}
	return (error);
}

static int
ugen_uiomove(struct ugen_softc *sc, struct ugen_endpoint *sce,
    uint16_t context_bit, void *cp, int n,
    struct uio *uio)
{
	int error;

	sce->state |= context_bit;

	mtx_unlock(&sc->sc_mtx);

	/*
	 * "uiomove()" can sleep so one needs to make a wrapper, exiting the
	 * mutex and checking things
	 */
	error = uiomove(cp, n, uio);

	mtx_lock(&sc->sc_mtx);

	sce->state &= ~context_bit;

	if (sce->state & UGEN_CLOSING) {
		wakeup(sce);
		error = EINTR;
	}
	return (error);
}

static int
ugen_usb_uiomove(struct ugen_softc *sc, struct ugen_endpoint *sce,
    struct usbd_page_cache *pc, struct uio *uio,
    uint32_t pc_offset, uint32_t len, uint16_t context_bit)
{
	int error;

	sce->state |= context_bit;

	mtx_unlock(&sc->sc_mtx);

	error = usbd_uiomove(pc, uio, pc_offset, len);

	mtx_lock(&sc->sc_mtx);

	sce->state &= ~context_bit;

	if (sce->state & UGEN_CLOSING) {
		wakeup(sce);
		error = EINTR;
	}
	return (error);
}

static int
ugenopen(struct cdev *dev, int flag, int mode, struct thread *p)
{
	struct ugen_softc *sc = DEV2SC(dev);
	struct ugen_endpoint *sce = DEV2SCE(dev);
	int error = 0;

	PRINTFN(5, ("flag=%d, mode=%d\n", flag, mode));

	if ((sc == NULL) || (sce == NULL)) {
		return (EIO);
	}
	mtx_lock(&sc->sc_mtx);

	if (sce->state & (UGEN_OPEN_DEV | UGEN_GONE)) {
		error = EBUSY;
		goto done;
	}
	if (sce != &sc->sc_endpoints[0]) {
		/*
		 * non-control endpoint(s) ; make sure that there are pipes
		 * for all directions
		 */
		if (((flag & FWRITE) && !sce->pipe_out) ||
		    ((flag & FREAD) && !sce->pipe_in)) {
			error = ENXIO;
			goto done;
		}
	}
	sce->state |= UGEN_OPEN_DEV;

done:
	mtx_unlock(&sc->sc_mtx);
	return (error);
}

static int
ugenclose(struct cdev *dev, int flag, int mode, struct thread *p)
{
	struct ugen_softc *sc = DEV2SC(dev);
	struct ugen_endpoint *sce = DEV2SCE(dev);
	struct usbd_xfer *temp_xfer[4];
	int32_t error;

	PRINTFN(5, ("flag=%d, mode=%d\n", flag, mode));

	if ((sc == NULL) || (sce == NULL)) {
		return (0);
	}
	mtx_lock(&sc->sc_mtx);

	if (sce->state & (UGEN_OPEN_DEV | UGEN_OPEN_IN | UGEN_OPEN_OUT)) {
		/* control endpoint is also ``closed'' here */

		sce->state |= UGEN_CLOSING;

		usbd_transfer_stop(sce->xfer_in[0]);
		usbd_transfer_stop(sce->xfer_in[1]);
		usbd_transfer_stop(sce->xfer_out[0]);
		usbd_transfer_stop(sce->xfer_out[1]);

		while (sce->state &
		    (UGEN_RD_CFG | UGEN_RD_SLP | UGEN_RD_WUP | UGEN_RD_UIO |
		    UGEN_WR_CFG | UGEN_WR_SLP | UGEN_WR_WUP | UGEN_WR_UIO |
		    UGEN_IOCTL)) {
			if (sce->state & (UGEN_RD_WUP | UGEN_WR_WUP)) {
				sce->state &= ~(UGEN_RD_WUP | UGEN_WR_WUP);
				wakeup(sce);
			}
			/* wait for routine(s) to exit */
			error = mtx_sleep(sce, &sc->sc_mtx, 0, "ugensync", 0);
		}

		/*
		 * free all memory after that one has waited for all context
		 * bits to clear, hence functions accessing memory like
		 * "uiomove", might be sleeping !
		 */

		ugen_free_blocks(&sce->in_queue);
		ugen_free_blocks(&sce->out_queue);

		sce->state &= ~(UGEN_OPEN_DEV | UGEN_OPEN_IN | UGEN_OPEN_OUT | UGEN_CLOSING);

		temp_xfer[0] = sce->xfer_in[0];
		temp_xfer[1] = sce->xfer_in[1];
		temp_xfer[2] = sce->xfer_out[0];
		temp_xfer[3] = sce->xfer_out[1];

		sce->xfer_in[0] = NULL;
		sce->xfer_in[1] = NULL;
		sce->xfer_out[0] = NULL;
		sce->xfer_out[1] = NULL;

		mtx_unlock(&sc->sc_mtx);

		usbd_transfer_unsetup(temp_xfer, 4);
	} else {
		mtx_unlock(&sc->sc_mtx);
	}
	return (0);
}

static int
ugen_open_pipe_write(struct ugen_softc *sc, struct ugen_endpoint *sce)
{
	usbd_status_t err;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	if (!(sce->state & UGEN_OPEN_OUT)) {
		if (sce->pipe_out) {
			usb_endpoint_descriptor_t *ed = sce->pipe_out->edesc;

			struct usbd_config usbd_config[2];

			bzero(usbd_config, sizeof(usbd_config));

			usbd_config[1].type = UE_CONTROL;
			usbd_config[1].endpoint = 0;
			usbd_config[1].direction = UE_DIR_ANY;
			usbd_config[1].mh.timeout = 1000;	/* 1 second */
			usbd_config[1].mh.interval = 50;	/* 50 milliseconds */
			usbd_config[1].mh.bufsize = sizeof(usb_device_request_t);
			usbd_config[1].mh.callback = &ugen_write_clear_stall_callback;

			usbd_config[0].type = ed->bmAttributes & UE_XFERTYPE;
			usbd_config[0].endpoint = ed->bEndpointAddress & UE_ADDR;
			usbd_config[0].direction = UE_DIR_OUT;
			usbd_config[0].mh.callback = &ugen_default_write_callback;
			usbd_config[0].mh.interval = USBD_DEFAULT_INTERVAL;
			usbd_config[0].mh.timeout = sce->in_timeout;
			usbd_config[0].mh.flags.proxy_buffer = 1;

			switch (ed->bmAttributes & UE_XFERTYPE) {
			case UE_INTERRUPT:
			case UE_BULK:
				usbd_config[0].mh.bufsize = UGEN_BULK_BUFFER_SIZE;

				if (ugen_transfer_setup
				    (sc, sce, UGEN_WR_CFG,
				    sc->sc_udev, sce->pipe_out->iface_index,
				    &sce->xfer_out[0], &usbd_config[0], 2, 0, 0)) {
					return (EIO);
				}
				/* first transfer clears stall */
				sce->write_stall = 1;
				break;

			case UE_ISOCHRONOUS:

				if (usbd_get_speed(sc->sc_udev) == USB_SPEED_HIGH) {
					sce->out_frames = UGEN_HW_FRAMES * 8;
				} else {
					sce->out_frames = UGEN_HW_FRAMES;
				}

				usbd_config[0].mh.flags.short_xfer_ok = 1;
				usbd_config[0].mh.bufsize = 0;	/* use default */
				usbd_config[0].mh.frames = sce->out_frames;
				usbd_config[0].mh.callback = &ugenisoc_write_callback;
				usbd_config[0].mh.timeout = 0;

				/* clone configuration */
				usbd_config[1] = usbd_config[0];

				err = ugen_transfer_setup
				    (sc, sce, UGEN_WR_CFG,
				    sc->sc_udev, sce->pipe_out->iface_index,
				    sce->xfer_out, usbd_config, 2,
				    0, 10 * sce->out_frames);

				if (err) {
					return (EIO);
				}
				break;

			default:
				return (EINVAL);
			}
			sce->state |= UGEN_OPEN_OUT;
		} else {
			return (ENXIO);
		}
	}
	return (0);
}

static int
ugen_open_pipe_read(struct ugen_softc *sc, struct ugen_endpoint *sce)
{
	usbd_status_t err;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	if (!(sce->state & UGEN_OPEN_IN)) {
		if (sce->pipe_in) {
			usb_endpoint_descriptor_t *ed = sce->pipe_in->edesc;

			struct usbd_config usbd_config[2];

			bzero(usbd_config, sizeof(usbd_config));

			usbd_config[1].type = UE_CONTROL;
			usbd_config[1].endpoint = 0;
			usbd_config[1].direction = UE_DIR_ANY;
			usbd_config[1].mh.timeout = 1000;	/* 1 second */
			usbd_config[1].mh.interval = 50;	/* 50 milliseconds */
			usbd_config[1].mh.bufsize = sizeof(usb_device_request_t);
			usbd_config[1].mh.callback = &ugen_read_clear_stall_callback;

			usbd_config[0].type = ed->bmAttributes & UE_XFERTYPE;
			usbd_config[0].endpoint = ed->bEndpointAddress & UE_ADDR;
			usbd_config[0].direction = UE_DIR_IN;
			usbd_config[0].mh.timeout = sce->in_timeout;
			usbd_config[0].mh.flags.proxy_buffer = 1;

			switch (ed->bmAttributes & UE_XFERTYPE) {
			case UE_INTERRUPT:
				usbd_config[0].mh.flags.short_xfer_ok = 1;
				usbd_config[0].mh.callback = &ugen_interrupt_callback;
				usbd_config[0].mh.bufsize = 0;	/* use "wMaxPacketSize" */
				usbd_config[0].mh.interval = USBD_DEFAULT_INTERVAL;
				usbd_config[0].mh.timeout = 0;

				if (ugen_transfer_setup
				    (sc, sce, UGEN_RD_CFG,
				    sc->sc_udev, sce->pipe_in->iface_index,
				    &sce->xfer_in[0], &usbd_config[0], 2, 1, 0)) {
					return (EIO);
				}
				/* first transfer clears stall */
				sce->read_stall = 1;

				usbd_transfer_start(sce->xfer_in[0]);
				PRINTFN(5, ("interrupt open done\n"));
				break;

			case UE_BULK:
				if (sce->state & UGEN_SHORT_OK) {
					usbd_config[0].mh.flags.short_xfer_ok = 1;
				}
				usbd_config[0].mh.callback = &ugen_default_read_callback;
				usbd_config[0].mh.bufsize = UGEN_BULK_BUFFER_SIZE;

				if (ugen_transfer_setup
				    (sc, sce, UGEN_RD_CFG,
				    sc->sc_udev, sce->pipe_in->iface_index,
				    &sce->xfer_in[0], &usbd_config[0], 2, 0, 0)) {
					return (EIO);
				}
				/* first transfer clears stall */
				sce->read_stall = 1;
				break;

			case UE_ISOCHRONOUS:

				/*
				 * the maximum frame size is validated by
				 * "usbd_fill_iface_data()"
				 */

				if (usbd_get_speed(sc->sc_udev) == USB_SPEED_HIGH) {
					sce->in_frames = UGEN_HW_FRAMES * 8;
				} else {
					sce->in_frames = UGEN_HW_FRAMES;
				}

				usbd_config[0].mh.flags.short_xfer_ok = 1;
				usbd_config[0].mh.bufsize = 0;	/* use default */
				usbd_config[0].mh.frames = sce->in_frames;
				usbd_config[0].mh.callback = &ugenisoc_read_callback;
				usbd_config[0].mh.timeout = 0;
				usbd_config[0].mh.interval = USBD_DEFAULT_INTERVAL;

				/* clone configuration */
				usbd_config[1] = usbd_config[0];

				err = ugen_transfer_setup
				    (sc, sce, UGEN_RD_CFG,
				    sc->sc_udev, sce->pipe_in->iface_index,
				    sce->xfer_in, usbd_config, 2,
				    sce->in_frames * 10, 0);

				if (err) {
					return (EIO);
				}
				usbd_transfer_start(sce->xfer_in[0]);
				usbd_transfer_start(sce->xfer_in[1]);
				PRINTFN(5, ("isoc open done\n"));
				break;

			default:
				return (EINVAL);
			}
			sce->state |= UGEN_OPEN_IN;
		} else {
			return (ENXIO);
		}
	}
	return (0);
}

static void
ugen_make_devnodes(struct ugen_softc *sc)
{
	struct usbd_pipe *pipe;
	struct usbd_pipe *pipe_end;
	struct ugen_endpoint *sce;
	struct cdev *dev;
	int endpoint;

	mtx_lock(&sc->sc_mtx);

	pipe = &sc->sc_udev->pipes[0];
	pipe_end = &sc->sc_udev->pipes_end[0];

	while (pipe < pipe_end) {
		if (pipe->edesc) {
			endpoint = pipe->edesc->bEndpointAddress & UE_ADDR;
			sce = &sc->sc_endpoints[endpoint];

			if (!sce->dev && (endpoint != 0)) {
				mtx_unlock(&sc->sc_mtx);	/* XXX "make_dev()" can
								 * sleep, XXX caller
								 * should have XXX set a
								 * context bit ! */

				dev = make_dev(&ugen_cdevsw,
				    UGENMINOR(device_get_unit(sc->sc_dev), endpoint),
				    UID_ROOT, GID_OPERATOR, 0644, "%s.%d",
				    device_get_nameunit(sc->sc_dev), endpoint);

				mtx_lock(&sc->sc_mtx);	/* XXX */

				sce->dev = dev;

				if (sce->dev) {
					DEV2SCE(sce->dev) = sce;
					DEV2SC(sce->dev) = sc;
				}
			}
			sce->in_timeout = USBD_NO_TIMEOUT;
			sce->out_frame_size = 0 - 1;	/* set maximum value */
			sce->io_buffer_size = UGEN_BULK_BUFFER_SIZE;	/* set default value */

			if ((pipe->edesc->bEndpointAddress &
			    (UE_DIR_IN | UE_DIR_OUT)) == UE_DIR_IN) {
				sce->pipe_in = pipe;
			} else {
				sce->pipe_out = pipe;
			}
		}
		pipe++;
	}
	mtx_unlock(&sc->sc_mtx);
	return;
}

static void
ugen_destroy_devnodes(struct ugen_softc *sc, int skip_first)
{
	struct ugen_endpoint *sce = &sc->sc_endpoints[0];
	struct ugen_endpoint *sce_end = &sc->sc_endpoints_end[0];

	if (skip_first) {
		sce++;			/* skip control endpoint */
	}
	while (sce < sce_end) {
		if (sce->dev) {
			ugenclose(sce->dev, 0, 0, 0);

			DEV2SCE(sce->dev) = NULL;
			DEV2SC(sce->dev) = NULL;

			destroy_dev(sce->dev);
		}
		sce->pipe_in = NULL;
		sce->pipe_out = NULL;
		sce->dev = NULL;

		sce++;
	}
	return;
}

static int
ugenread(struct cdev *dev, struct uio *uio, int flag)
{
	struct ugen_softc *sc = DEV2SC(dev);
	struct ugen_endpoint *sce = DEV2SCE(dev);
	struct usbd_xfer *xfer;
	void *ptr;
	int error;
	int n;
	uint16_t len;

	PRINTFN(5, ("\n"));

	if ((sc == NULL) || (sce == NULL)) {
		return (EIO);
	}
	/* check for control endpoint */
	if (sce == &sc->sc_endpoints[0]) {
		return (ENODEV);
	}
	mtx_lock(&sc->sc_mtx);

	if (sce->state & (UGEN_CLOSING | UGEN_GONE | UGEN_RD_CFG |
	    UGEN_RD_UIO | UGEN_RD_SLP)) {
		error = EIO;
		goto done;
	}
	error = ugen_open_pipe_read(sc, sce);
	if (error) {
		goto done;
	}
	switch (sce->pipe_in->edesc->bmAttributes & UE_XFERTYPE) {
	case UE_ISOCHRONOUS:
		n = 1;
		goto ue_interrupt;

	case UE_INTERRUPT:
		n = 2;

ue_interrupt:
		while (uio->uio_resid) {
			if ((uio->uio_resid < sce->in_queue.frame_size) &&
			    (n == 0)) {
				/* try to keep data synchronization */
				break;
			}
			/* get one frame from input queue */

			ugen_get_output_block(&sce->in_queue, &ptr, &len);

			if (ptr == NULL) {
				if (n == 0) {
					/* let application process data */
					break;
				}
				if (flag & IO_NDELAY) {
					if (n) {
						error = EWOULDBLOCK;
					}
					break;
				}
				/* wait for data */

				sce->state |= (UGEN_RD_SLP | UGEN_RD_WUP);

				error = mtx_sleep(sce, &sc->sc_mtx, PCATCH,
				    "ugen wait callback",
				    sce->in_timeout ?
				    (((sce->in_timeout * hz) + 999) / 1000) : 0);

				sce->state &= ~(UGEN_RD_SLP | UGEN_RD_WUP);

				if (sce->state & UGEN_CLOSING) {
					wakeup(sce);
					error = EIO;
					break;
				}
				if (error) {
					break;
				}
				continue;
			}
			if (len > uio->uio_resid) {
				PRINTFN(5, ("dumping %d bytes!\n",
				    len - (uint16_t)(uio->uio_resid)));

				/*
				 * rest of this frame will get dumped for
				 * sake of synchronization!
				 */
				len = uio->uio_resid;
			}
			PRINTFN(10, ("transferring %d bytes\n", len));

			/* copy data to user memory */
			error = ugen_uiomove(sc, sce, UGEN_RD_UIO, ptr, len, uio);

			if (error)
				break;

			ugen_inc_output_index(&sce->in_queue);

			/* only transfer one interrupt frame per read ! */

			if (n == 2) {
				/* start interrupt transfer again */
				usbd_transfer_start(sce->xfer_in[0]);
				break;
			}
			n = 0;
		}
		break;

	case UE_BULK:
		while ((n = min(UGEN_BULK_BUFFER_SIZE, uio->uio_resid)) != 0) {
#if 0
			if (flag & IO_NDELAY) {
				error = EWOULDBLOCK;
				break;
			}
#endif
			xfer = sce->xfer_in[0];

			/* update length */
			xfer->frlengths[0] = n;

			/* start transfer */
			usbd_transfer_start(xfer);

			while ((xfer->flags_int.transferring) ||
			    (sce->xfer_in[1]->flags_int.transferring)) {

				/* wait for data */

				sce->state |= (UGEN_RD_SLP | UGEN_RD_WUP);

				error = mtx_sleep(sce, &sc->sc_mtx, PCATCH,
				    "ugen wait callback", 0);

				sce->state &= ~(UGEN_RD_SLP | UGEN_RD_WUP);

				if (error) {
					usbd_transfer_stop(sce->xfer_in[1]);
					usbd_transfer_stop(xfer);
					break;
				}
			}

			if (sce->state & UGEN_CLOSING) {
				error = EIO;
				wakeup(sce);
				break;
			}
			if (error) {
				break;
			}
			if (xfer->error) {
				error = (xfer->error == USBD_ERR_CANCELLED) ?
				    EINTR : EIO;
				break;
			}
			PRINTFN(1, ("got %d of %d bytes\n", xfer->actlen, n));

			error = ugen_usb_uiomove
			    (sc, sce, xfer->frbuffers + 0, uio, 0,
			    xfer->actlen, UGEN_RD_UIO);

			if (error || (xfer->actlen < n)) {
				break;
			}
		}
		break;

	default:
		error = ENXIO;
		break;
	}

done:
	mtx_unlock(&sc->sc_mtx);
	return (error);
}

static int
ugenwrite(struct cdev *dev, struct uio *uio, int flag)
{
	struct ugen_softc *sc = DEV2SC(dev);
	struct ugen_endpoint *sce = DEV2SCE(dev);
	struct usbd_xfer *xfer;
	uint16_t *plen;
	void *ptr;
	int error;
	int n;

	PRINTFN(5, ("\n"));

	if ((sc == NULL) || (sce == NULL)) {
		return (EIO);
	}
	/* check for control endpoint */
	if (sce == &sc->sc_endpoints[0]) {
		return (ENODEV);
	}
	mtx_lock(&sc->sc_mtx);

	if (sce->state & (UGEN_CLOSING | UGEN_GONE | UGEN_WR_CFG |
	    UGEN_WR_SLP | UGEN_WR_UIO)) {
		error = EIO;
		goto done;
	}
	error = ugen_open_pipe_write(sc, sce);
	if (error) {
		goto done;
	}
	switch (sce->pipe_out->edesc->bmAttributes & UE_XFERTYPE) {
	case UE_BULK:
	case UE_INTERRUPT:
		while ((n = min(UGEN_BULK_BUFFER_SIZE, uio->uio_resid)) != 0) {
#if 0
			if (flag & IO_NDELAY) {
				error = EWOULDBLOCK;
				break;
			}
#endif
			xfer = sce->xfer_out[0];

			error = ugen_usb_uiomove
			    (sc, sce, xfer->frbuffers + 0, uio,
			    0, n, UGEN_WR_UIO);

			if (error) {
				break;
			}
			PRINTFN(1, ("transferred %d bytes\n", n));

			/* update length */
			xfer->frlengths[0] = n;

			/* start transfer */
			usbd_transfer_start(xfer);

			while ((xfer->flags_int.transferring) ||
			    (sce->xfer_out[1]->flags_int.transferring)) {

				/* wait for data */

				sce->state |= (UGEN_WR_SLP | UGEN_WR_WUP);

				error = mtx_sleep(sce, &sc->sc_mtx, PCATCH,
				    "ugen wait callback", 0);

				sce->state &= ~(UGEN_WR_SLP | UGEN_WR_WUP);

				if (error) {
					usbd_transfer_stop(sce->xfer_out[1]);
					usbd_transfer_stop(xfer);
					break;
				}
			}

			if (sce->state & UGEN_CLOSING) {
				error = EIO;
				wakeup(sce);
				break;
			}
			if (error) {
				break;
			}
			if (xfer->error) {
				error = (xfer->error == USBD_ERR_CANCELLED) ?
				    EINTR : EIO;
				break;
			}
		}
		break;

	case UE_ISOCHRONOUS:

		n = 1;

		while (uio->uio_resid || n) {
			ugen_get_input_block(&sce->out_queue, &ptr, &plen);

			if (ptr == NULL) {
				/*
				 * make sure that the transfers are started,
				 * if not already started.
				 */
				usbd_transfer_start(sce->xfer_out[0]);
				usbd_transfer_start(sce->xfer_out[1]);

				if (flag & IO_NDELAY) {
					if (n) {
						error = EWOULDBLOCK;
					}
					break;
				}
				sce->state |= (UGEN_WR_SLP | UGEN_WR_WUP);

				error = mtx_sleep(sce, &sc->sc_mtx, PCATCH,
				    "ugen wait callback", 0);

				sce->state &= ~(UGEN_WR_SLP | UGEN_WR_WUP);

				if (sce->state & UGEN_CLOSING) {
					wakeup(sce);
					error = EIO;
					break;
				}
				if (error) {
					break;
				}
				continue;
			}
			if (*plen > sce->out_frame_size) {
				*plen = sce->out_frame_size;
			}
			if (*plen > uio->uio_resid) {
				*plen = uio->uio_resid;
			}
			error = ugen_uiomove(sc, sce, UGEN_WR_UIO, ptr, *plen, uio);

			if (error)
				break;

			ugen_inc_input_index(&sce->out_queue);

			n = 0;
		}
		if (n == 0) {
			/*
			 * make sure that the transfers are started, if not
			 * already started.
			 */
			usbd_transfer_start(sce->xfer_out[0]);
			usbd_transfer_start(sce->xfer_out[1]);
		}
		break;

	default:
		error = ENXIO;
		break;
	}

done:
	mtx_unlock(&sc->sc_mtx);
	return (error);
}

static void
ugen_default_read_callback(struct usbd_xfer *xfer)
{
	struct ugen_endpoint *sce = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
		if (sce->read_stall) {
			usbd_transfer_start(sce->xfer_in[1]);
		} else {
			usbd_start_hardware(xfer);
		}
		return;

	case USBD_ST_TRANSFERRED:
tr_transferred:
		if (sce->state & UGEN_RD_WUP) {
			sce->state &= ~UGEN_RD_WUP;

			PRINTFN(5, ("waking %p\n", sce));
			wakeup(sce);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sce->read_stall = 1;
		} else {
			return;
		}

		goto tr_transferred;
	}
}

static void
ugen_default_write_callback(struct usbd_xfer *xfer)
{
	struct ugen_endpoint *sce = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
		if (sce->write_stall) {
			usbd_transfer_start(sce->xfer_out[1]);
		} else {
			usbd_start_hardware(xfer);
		}
		return;

	case USBD_ST_TRANSFERRED:
tr_transferred:
		if (sce->state & UGEN_WR_WUP) {
			sce->state &= ~UGEN_WR_WUP;

			PRINTFN(5, ("waking %p\n", sce));
			wakeup(sce);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sce->write_stall = 1;
		} else {
			return;
		}

		goto tr_transferred;
	}
}

static void
ugen_interrupt_callback(struct usbd_xfer *xfer)
{
	struct ugen_endpoint *sce = xfer->priv_sc;
	uint16_t *plen;
	void *ptr;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		PRINTFN(5, ("xfer=%p actlen=%d\n",
		    xfer, xfer->actlen));

		ugen_get_input_block(&sce->in_queue, &ptr, &plen);

		if (ptr == NULL) {
			PRINTFN(5, ("dropping one packet, sce=%p\n", sce));
		} else {
			usbd_copy_out(xfer->frbuffers + 0, 0, ptr, xfer->actlen);

			if (xfer->actlen > *plen) {
				xfer->actlen = *plen;
			}
			*plen = xfer->actlen;

			ugen_inc_input_index(&sce->in_queue);
		}

		if (sce->state & UGEN_RD_WUP) {
			sce->state &= ~UGEN_RD_WUP;

			PRINTFN(5, ("waking %p\n", sce));
			wakeup(sce);
		}
		selwakeuppri(&sce->selinfo, PZERO);

		/*
		 * the transfer will be restarted after that the packet has
		 * been read
		 */
		return;

	case USBD_ST_SETUP:
tr_setup:
		if (sce->read_stall) {
			usbd_transfer_start(sce->xfer_in[1]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			sce->read_stall = 1;
		} else {
			return;
		}

		goto tr_setup;
	}
}

static void
ugen_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ugen_endpoint *sce = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sce->xfer_in[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		PRINTFN(4, ("sce=%p: stall cleared\n", sce));
		sce->read_stall = 0;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ugen_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ugen_endpoint *sce = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sce->xfer_out[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		PRINTFN(4, ("sce=%p: stall cleared\n", sce));
		sce->write_stall = 0;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
ugenisoc_read_callback(struct usbd_xfer *xfer)
{
	struct ugen_endpoint *sce = xfer->priv_sc;
	uint32_t *plen1;
	uint16_t *plen2;

	void *ptr2;

	uint32_t offset;
	uint16_t n;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		PRINTFN(5, ("actlen=%d\n", xfer->actlen));

		plen1 = xfer->frlengths;

		n = sce->in_frames;
		offset = 0;
		while (n--) {
			if (*plen1 != 0) {
				ugen_get_input_block(&sce->in_queue, &ptr2, &plen2);

				if (ptr2 == NULL) {
					break;
				}
				if (*plen1 > *plen2) {
					*plen1 = *plen2;
				}
				usbd_copy_out(xfer->frbuffers + 0, offset,
				    ptr2, *plen1);

				*plen2 = *plen1;

				ugen_inc_input_index(&sce->in_queue);
			}
			offset += xfer->max_frame_size;
			plen1++;
		}

		if (sce->state & UGEN_RD_WUP) {
			sce->state &= ~UGEN_RD_WUP;
			wakeup(sce);
		}
		selwakeuppri(&sce->selinfo, PZERO);

	case USBD_ST_SETUP:
tr_setup:
		for (n = 0; n < sce->in_frames; n++) {
			/* setup size for next transfer */
			xfer->frlengths[n] = xfer->max_frame_size;
		}
		usbd_start_hardware(xfer);
		return;

	default:			/* Error */
		if (xfer->error == USBD_ERR_CANCELLED) {
			return;
		}
		goto tr_setup;
	}
}

static void
ugenisoc_write_callback(struct usbd_xfer *xfer)
{
	struct ugen_endpoint *sce = xfer->priv_sc;
	uint32_t *plen1;
	void *ptr2;

	uint32_t offset;
	uint16_t len2;
	uint16_t n;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
tr_transferred:
	case USBD_ST_SETUP:

		plen1 = xfer->frlengths;

		offset = 0;
		n = sce->out_frames;
		while (n--) {
			ugen_get_output_block(&sce->out_queue, &ptr2, &len2);

			if (ptr2 == NULL) {
				break;
			}
			if (len2 > xfer->max_frame_size) {
				len2 = xfer->max_frame_size;
			}
			usbd_copy_in(xfer->frbuffers + 0, offset, ptr2, len2);

			*plen1 = len2;

			ugen_inc_output_index(&sce->out_queue);

			offset += len2;
			plen1++;
		}

		n = plen1 - xfer->frlengths;

		/* update number of frames */
		xfer->nframes = n;

		if (sce->state & UGEN_WR_WUP) {
			sce->state &= ~UGEN_WR_WUP;
			wakeup(sce);
		}
		selwakeuppri(&sce->selinfo, PZERO);

		if (n > 0) {
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error == USBD_ERR_CANCELLED) {
			return;
		}
		goto tr_transferred;
	}
}

static int
ugen_set_config(struct ugen_softc *sc, int configno)
{
	PRINTFN(1, ("configno %d, sc=%p\n", configno, sc));

	/* destroy all but control device */
	ugen_destroy_devnodes(sc, 1);

	/* avoid setting the current value */
	if ((usbd_get_config_descriptor(sc->sc_udev) == NULL) ||
	    (usbd_get_config_descriptor(sc->sc_udev)->bConfigurationValue != configno)) {
		if (usbd_set_config_no(sc->sc_udev, configno, 1)) {
			return (EIO);
		}
	}
	/* make devices */
	ugen_make_devnodes(sc);
	return (0);
}

static int
ugen_set_interface(struct ugen_softc *sc, int ifaceidx, int altno)
{
	PRINTFN(15, ("%d %d\n", ifaceidx, altno));

	/* destroy all but control device */
	ugen_destroy_devnodes(sc, 1);

	/* change setting */
	if (usbd_set_alt_interface_index(sc->sc_udev, ifaceidx, altno)) {
		return (EIO);
	}
	/* make the new devices */
	ugen_make_devnodes(sc);
	return (0);
}

/* retrieve a complete descriptor for a certain device and index */
static usb_config_descriptor_t *
ugen_get_cdesc(struct usbd_device *udev, int index, int *lenp)
{
	usb_config_descriptor_t *cdesc, *tdesc, cdescr;
	int len;

	if (index == USB_CURRENT_CONFIG_INDEX) {
		tdesc = usbd_get_config_descriptor(udev);
		len = UGETW(tdesc->wTotalLength);
		if (lenp) {
			*lenp = len;
		}
		cdesc = malloc(len, M_TEMP, M_WAITOK);
		if (cdesc == NULL) {
			return (0);
		}
		memcpy(cdesc, tdesc, len);
		PRINTFN(5, ("current, len=%d\n", len));
	} else {
		if (usbreq_get_config_desc(udev, NULL, &cdescr, index)) {
			return (0);
		}
		len = UGETW(cdescr.wTotalLength);
		PRINTFN(5, ("index=%d, len=%d\n", index, len));
		if (lenp) {
			*lenp = len;
		}
		cdesc = malloc(len, M_TEMP, M_WAITOK);
		if (cdesc == NULL) {
			return (0);
		}
		if (usbreq_get_config_desc_full(udev, NULL, cdesc, len, index)) {
			free(cdesc, M_TEMP);
			return (0);
		}
	}
	return (cdesc);
}

static int
ugen_get_alt_index(struct usbd_device *udev, int ifaceidx)
{
	struct usbd_interface *iface = usbd_get_iface(udev, ifaceidx);

	return (iface) ? (iface->alt_index) : -1;
}

static int
ugenioctl(struct cdev *dev, u_long cmd, caddr_t addr, int flag, struct thread *p)
{
	struct ugen_softc *sc = DEV2SC(dev);
	struct ugen_endpoint *sce = DEV2SCE(dev);
	struct usbd_interface *iface;
	usb_interface_descriptor_t *idesc;
	usb_endpoint_descriptor_t *edesc;
	void *data = 0;
	int error = 0;
	int len;
	uint16_t actlen;
	uint8_t conf;
	uint8_t alt;

	PRINTFN(5, ("cmd=%08lx\n", cmd));
	if ((sc == NULL) || (sce == NULL)) {
		return (EIO);
	}
	mtx_lock(&sc->sc_mtx);

	if (sce->state & (UGEN_CLOSING | UGEN_GONE | UGEN_IOCTL)) {
		error = EIO;
		goto done;
	}
	switch (cmd) {
	case FIOASYNC:
		if (*(int *)addr)
			error = EINVAL;
	case FIONBIO:
		/* all handled in the upper FS layer */
		goto done;

	case USB_SET_SHORT_XFER:
		/* this flag only affects read */
		/* check for control endpoint */
		if (sce == &sc->sc_endpoints[0]) {
			error = EINVAL;
			goto done;
		}
		if (sce->pipe_in == NULL) {
#ifdef USB_DEBUG
			printf("%s: USB_SET_SHORT_XFER, no pipe\n",
			    __FUNCTION__);
#endif
			error = EIO;
			goto done;
		}
		if (*(int *)addr)
			sce->state |= UGEN_SHORT_OK;
		else
			sce->state &= ~UGEN_SHORT_OK;
		goto done;

	case USB_SET_TIMEOUT:
		sce->in_timeout = *(int *)addr;
		goto done;

	case USB_GET_FRAME_SIZE:
		if (sce->xfer_in[0]) {
			*(int *)addr = sce->xfer_in[0]->max_frame_size;
		} else {
			error = EINVAL;
		}
		goto done;

	case USB_SET_FRAME_SIZE:
		if (!(flag & FWRITE)) {
			error = EPERM;
			goto done;
		}
		if ((*((int *)addr) <= 0) ||
		    (*((int *)addr) >= 65536)) {
			error = EINVAL;
			goto done;
		}
		sce->out_frame_size = *(int *)addr;
		goto done;

	case USB_SET_BUFFER_SIZE:
		if (*(int *)addr < 1024)
			sce->io_buffer_size = 1024;
		else if (*(int *)addr < (256 * 1024))
			sce->io_buffer_size = *(int *)addr;
		else
			sce->io_buffer_size = 256 * 1024;
		break;

	case USB_GET_BUFFER_SIZE:
		*(int *)addr = sce->io_buffer_size;
		break;

	default:
		break;
	}

	/*
	 * the following ioctls will sleep and are only allowed on the
	 * control endpoint
	 */
	if (sce != &sc->sc_endpoints[0]) {
		error = EINVAL;
		goto done;
	}
	sce->state |= UGEN_IOCTL;

	mtx_unlock(&sc->sc_mtx);

	switch (cmd) {
#ifdef USB_DEBUG
	case USB_SETDEBUG:
		if (!(flag & FWRITE)) {
			error = EPERM;
			break;
		}
		usbdebug = *(int *)addr;
		break;
#endif
	case USB_GET_CONFIG:
		error = usbreq_get_config(sc->sc_udev, NULL, &conf);
		if (error) {
			error = EIO;
			break;
		}
		*(int *)addr = conf;
		break;
	case USB_SET_CONFIG:
		if (!(flag & FWRITE)) {
			error = EPERM;
			break;
		}
		error = ugen_set_config(sc, *(int *)addr);
		if (error) {
			break;
		}
		break;
	case USB_GET_ALTINTERFACE:
#define	ai ((struct usb_alt_interface *)addr)
		iface = usbd_get_iface(sc->sc_udev, ai->uai_interface_index);

		if (!iface ||
		    !iface->idesc) {
			error = EINVAL;
			break;
		}
		ai->uai_alt_no = iface->idesc->bAlternateSetting;
		break;
	case USB_SET_ALTINTERFACE:
		if (!(flag & FWRITE)) {
			error = EPERM;
			break;
		}
		error = ugen_set_interface(sc, ai->uai_interface_index, ai->uai_alt_no);
		if (error) {
			break;
		}
		break;
	case USB_GET_NO_ALT:
		data = ugen_get_cdesc(sc->sc_udev, ai->uai_config_index, 0);
		if (data == NULL) {
			error = EINVAL;
			break;
		}
		idesc = usbd_find_idesc(data, ai->uai_interface_index, 0);
		if (idesc == NULL) {
			error = EINVAL;
			break;
		}
		ai->uai_alt_no = usbd_get_no_alts(data, idesc->bInterfaceNumber);
#undef ai
		break;
	case USB_GET_DEVICE_DESC:
		if (!usbd_get_device_descriptor(sc->sc_udev)) {
			error = EIO;
			break;
		}
		*(usb_device_descriptor_t *)addr =
		    *usbd_get_device_descriptor(sc->sc_udev);
		break;
	case USB_GET_CONFIG_DESC:
#define	cd ((struct usb_config_desc *)addr)
		data = ugen_get_cdesc(sc->sc_udev, cd->ucd_config_index, 0);
		if (data == NULL) {
			error = EINVAL;
			break;
		}
		cd->ucd_desc = *(usb_config_descriptor_t *)data;
#undef cd
		break;
	case USB_GET_INTERFACE_DESC:
#define	id  ((struct usb_interface_desc *)addr)
		data = ugen_get_cdesc(sc->sc_udev, id->uid_config_index, 0);
		if (data == NULL) {
			error = EINVAL;
			break;
		}
		if ((id->uid_config_index == USB_CURRENT_CONFIG_INDEX) &&
		    (id->uid_alt_index == USB_CURRENT_ALT_INDEX))
			alt = ugen_get_alt_index(sc->sc_udev, id->uid_interface_index);
		else
			alt = id->uid_alt_index;
		idesc = usbd_find_idesc(data, id->uid_interface_index, alt);
		if (idesc == NULL) {
			error = EINVAL;
			break;
		}
		id->uid_desc = *idesc;
#undef id
		break;
	case USB_GET_ENDPOINT_DESC:
#define	ed ((struct usb_endpoint_desc *)addr)
		data = ugen_get_cdesc(sc->sc_udev, ed->ued_config_index, 0);
		if (data == NULL) {
			error = EINVAL;
			break;
		}
		if ((ed->ued_config_index == USB_CURRENT_CONFIG_INDEX) &&
		    (ed->ued_alt_index == USB_CURRENT_ALT_INDEX))
			alt = ugen_get_alt_index(sc->sc_udev, ed->ued_interface_index);
		else
			alt = ed->ued_alt_index;
		edesc = usbd_find_edesc(data, ed->ued_interface_index,
		    alt, ed->ued_endpoint_index);
		if (edesc == NULL) {
			error = EINVAL;
			break;
		}
		ed->ued_desc = *edesc;
#undef ed
		break;
	case USB_GET_FULL_DESC:
#define	fd ((struct usb_full_desc *)addr)
		data = ugen_get_cdesc(sc->sc_udev, fd->ufd_config_index, &len);
		if (data == NULL) {
			error = EINVAL;
			break;
		}
		if (len > fd->ufd_size) {
			len = fd->ufd_size;
		}
		if (fd->ufd_data == NULL) {
			error = EINVAL;
			break;
		}
		error = copyout(data, fd->ufd_data, len);
#undef fd
		break;
	case USB_GET_STRING_DESC:
#define	si ((struct usb_string_desc *)addr)
		if (usbreq_get_string_desc
		    (sc->sc_udev, NULL, &si->usd_desc,
		    sizeof(si->usd_desc), si->usd_language_id,
		    si->usd_string_index)) {
			error = EINVAL;
			break;
		}
#undef si
		break;
	case USB_DO_REQUEST:
#define	ur ((struct usb_ctl_request *)addr)
		if (!(flag & FWRITE)) {
			error = EPERM;
			break;
		}
		/* avoid requests that would damage the bus integrity */
		if (((ur->ucr_request.bmRequestType == UT_WRITE_DEVICE) &&
		    (ur->ucr_request.bRequest == UR_SET_ADDRESS)) ||
		    ((ur->ucr_request.bmRequestType == UT_WRITE_DEVICE) &&
		    (ur->ucr_request.bRequest == UR_SET_CONFIG)) ||
		    ((ur->ucr_request.bmRequestType == UT_WRITE_INTERFACE) &&
		    (ur->ucr_request.bRequest == UR_SET_INTERFACE))) {
			error = EINVAL;
			break;
		}
		/*
		 * Clearing the stall this way is not allowed, hence it does
		 * not update the data toggle value in "struct usbd_pipe" !
		 */
		if (ur->ucr_request.bmRequestType == UT_WRITE_ENDPOINT) {
			if (ur->ucr_request.wLength[0] ||
			    ur->ucr_request.wLength[1]) {
				error = EINVAL;
			} else {
				error = 0;
				ur->ucr_actlen = 0;
			}
			break;
		}
		len = UGETW(ur->ucr_request.wLength);

		if ((len < 0) || (len > 32767)) {
			error = EINVAL;
			break;
		}
		if (len != 0) {
			if (ur->ucr_data == NULL) {
				error = EINVAL;
				break;
			}
			data = malloc(len, M_TEMP, M_WAITOK);
			if (data == NULL) {
				error = ENOMEM;
				break;
			}
			if (!(ur->ucr_request.bmRequestType & UT_READ)) {
				error = copyin(ur->ucr_data, data, len);
				if (error) {
					break;
				}
			}
		}
		error = usbd_do_request_flags
		    (sc->sc_udev, NULL, &ur->ucr_request, data,
		    (ur->ucr_flags & USBD_ERR_SHORT_XFER_OK), &actlen,
		    USBD_DEFAULT_TIMEOUT);

		ur->ucr_actlen = actlen;

		if (error) {
			error = EIO;
			break;
		}
		if ((len != 0) && (ur->ucr_request.bmRequestType & UT_READ)) {
			error = copyout(data, ur->ucr_data, len);
			if (error) {
				break;
			}
		}
#undef ur
		break;

	case USB_GET_DEVICEINFO:
		usbd_fill_deviceinfo(sc->sc_udev, (void *)addr);
		break;
	default:
		error = EINVAL;
		break;
	}

	if (data) {
		free(data, M_TEMP);
	}
	mtx_lock(&sc->sc_mtx);

	sce->state &= ~UGEN_IOCTL;

	if (sce->state & UGEN_CLOSING) {
		wakeup(sce);
		error = EIO;
	}
done:
	mtx_unlock(&sc->sc_mtx);
	return (error);
}

static int
ugenpoll(struct cdev *dev, int events, struct thread *p)
{
	struct ugen_softc *sc = DEV2SC(dev);
	struct ugen_endpoint *sce = DEV2SCE(dev);
	int revents = 0;

	PRINTFN(5, ("\n"));

	if ((sc == NULL) || (sce == NULL)) {
		return (POLLNVAL);
	}
	if (sce == &sc->sc_endpoints[0]) {
		/* not control endpoint */
		return (0);
	}
	mtx_lock(&sc->sc_mtx);

	if (sce->state & (UGEN_CLOSING | UGEN_GONE)) {
		goto done;
	}
	if (sce->pipe_out && (events & (POLLOUT | POLLWRNORM)) &&
	    (!(sce->state & (UGEN_WR_CFG | UGEN_WR_SLP | UGEN_WR_UIO)))) {
		if (ugen_open_pipe_write(sc, sce)) {
			/*
			 * must return, hence the error might indicate that
			 * the device is about to be detached
			 */
			revents = POLLNVAL;
			goto done;
		}
		switch (sce->pipe_out->edesc->bmAttributes & UE_XFERTYPE) {
		case UE_ISOCHRONOUS:
			if (ugen_half_empty(&sce->out_queue)) {
				revents |= events & (POLLOUT | POLLWRNORM);
			} else {
				selrecord(p, &sce->selinfo);
			}
			break;

		case UE_BULK:
		case UE_INTERRUPT:
			/*
			 * pretend that one can write data, hence no
			 * buffering is done:
			 */
			revents |= events & (POLLOUT | POLLWRNORM);
			break;
		}
	}
	if (sce->pipe_in && (events & (POLLIN | POLLRDNORM)) &&
	    (!(sce->state & (UGEN_RD_CFG | UGEN_RD_SLP | UGEN_RD_UIO)))) {
		/*
		 * check that pipes are open, so that selwakeup is called
		 */
		if (ugen_open_pipe_read(sc, sce)) {
			/*
			 * must return, hence the error might indicate that
			 * the device is about to be detached
			 */
			revents = POLLNVAL;
			goto done;
		}
		switch (sce->pipe_in->edesc->bmAttributes & UE_XFERTYPE) {
		case UE_INTERRUPT:
		case UE_ISOCHRONOUS:
			if (ugen_not_empty(&sce->in_queue)) {
				revents |= events & (POLLIN | POLLRDNORM);
			} else {
				selrecord(p, &sce->selinfo);
			}
			break;

		case UE_BULK:
			/*
			 * pretend that one can read data, hence no
			 * buffering is done:
			 */
			revents |= events & (POLLIN | POLLRDNORM);
			break;
		}
	}
done:
	mtx_unlock(&sc->sc_mtx);
	return (revents);
}

cdevsw_t ugen_cdevsw = {
#ifdef D_VERSION
	.d_version = D_VERSION,
#endif
	.d_open = ugenopen,
	.d_close = ugenclose,
	.d_read = ugenread,
	.d_write = ugenwrite,
	.d_ioctl = ugenioctl,
	.d_poll = ugenpoll,
	.d_name = "ugen",
};

static devclass_t ugen_devclass;

static driver_t ugen_driver =
{
	.name = "ugen",
	.methods = (device_method_t[]){
		DEVMETHOD(device_probe, ugen_probe),
		DEVMETHOD(device_attach, ugen_attach),
		DEVMETHOD(device_detach, ugen_detach),
		{0, 0}
	},
	.size = sizeof(struct ugen_softc),
};

DRIVER_MODULE(ugen, uhub, ugen_driver, ugen_devclass, usbd_driver_load, 0);
MODULE_DEPEND(ugen, usb, 1, 1, 1);
