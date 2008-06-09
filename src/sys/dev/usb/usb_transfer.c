#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/usb_transfer.c $");

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
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/kthread.h>
#include <sys/unistd.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_hid.h>

/* prototypes */

static void usbd_compute_max_frame_size(struct usbd_xfer *xfer);
static uint32_t usbd_get_dma_delay(struct usbd_bus *bus);
static void usbd_transfer_unsetup_sub(struct usbd_memory_info *info, uint8_t needs_delay);
static void usbd_control_transfer_init(struct usbd_xfer *xfer);
static uint8_t usbd_start_hardware_sub(struct usbd_xfer *xfer);
static void usbd_premature_callback(struct usbd_xfer *xfer, usbd_status_t error);
static void usbd_pipe_enter_wrapper(struct usbd_xfer *xfer);
static void usbd_bdma_work_loop(struct usbd_memory_info *info);
static void usbd_bdma_done_event(struct usbd_dma_parent_tag *udpt);
static void usbd_callback_intr_sched(struct usbd_memory_info *info);
static void usbd_callback_intr_td_sub(struct usbd_xfer **xfer, uint8_t dropcount);
static void usbd_callback_intr_td(void *arg);
static void usbd_dma_delay_done_cb(struct usbd_xfer *xfer);
static void usbd_delayed_transfer_start(void *arg);
static void usbd_do_request_callback(struct usbd_xfer *xfer);
static void usbd_handle_request_callback(struct usbd_xfer *xfer);
static usbd_status_t usbd_handle_set_config(struct usbd_xfer *xfer, uint8_t conf_no);
static struct usbd_xfer *usbd_get_curr_xfer(struct usbd_pipe *pipe);
static usbd_status_t usbd_handle_set_stall_sub(struct usbd_device *udev, uint8_t ea_val, uint8_t do_stall);
static usbd_status_t usbd_handle_set_stall(struct usbd_xfer *xfer, uint8_t ep, uint8_t do_stall);
static uint8_t usbd_handle_get_stall(struct usbd_device *udev, uint8_t ea_val);
static usbd_status_t usbd_handle_remote_wakeup(struct usbd_xfer *xfer, uint8_t is_on);
static usbd_status_t usbd_handle_request(struct usbd_xfer *xfer);

#ifdef USB_DEBUG
/*------------------------------------------------------------------------*
 *	usbd_dump_iface
 *
 * This function dumps information about an USB interface.
 *------------------------------------------------------------------------*/
void
usbd_dump_iface(struct usbd_interface *iface)
{
	printf("usbd_dump_iface: iface=%p\n", iface);
	if (iface == NULL) {
		return;
	}
	printf(" iface=%p idesc=%p altindex=%d\n",
	    iface, iface->idesc, iface->alt_index);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dump_device
 *
 * This function dumps information about an USB device.
 *------------------------------------------------------------------------*/
void
usbd_dump_device(struct usbd_device *udev)
{
	printf("usbd_dump_device: dev=%p\n", udev);
	if (udev == NULL) {
		return;
	}
	printf(" bus=%p \n"
	    " address=%d config=%d depth=%d speed=%d self_powered=%d\n"
	    " power=%d langid=%d\n",
	    udev->bus,
	    udev->address, udev->curr_config_no, udev->depth, udev->speed,
	    udev->flags.self_powered, udev->power, udev->langid);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dump_queue
 *
 * This function dumps the USB transfer that are queued up on an USB pipe.
 *------------------------------------------------------------------------*/
void
usbd_dump_queue(struct usbd_pipe *pipe)
{
	struct usbd_xfer *xfer;

	printf("usbd_dump_queue: pipe=%p\n", pipe);
	LIST_FOREACH(xfer, &pipe->list_head, pipe_list) {
		printf("  xfer=%p\n", xfer);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dump_pipe
 *
 * This function dumps information about an USB pipe.
 *------------------------------------------------------------------------*/
void
usbd_dump_pipe(struct usbd_pipe *pipe)
{
	if (pipe) {
		printf("usbd_dump_pipe: pipe=%p", pipe);

		printf(" edesc=%p isoc_next=%d toggle_next=%d",
		    pipe->edesc, pipe->isoc_next, pipe->toggle_next);

		if (pipe->edesc) {
			printf(" bEndpointAddress=0x%02x",
			    pipe->edesc->bEndpointAddress);
		}
		printf("\n");
		usbd_dump_queue(pipe);
	} else {
		printf("usbd_dump_pipe: pipe=NULL\n");
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dump_xfer
 *
 * This function dumps information about an USB transfer.
 *------------------------------------------------------------------------*/
void
usbd_dump_xfer(struct usbd_xfer *xfer)
{
	printf("usbd_dump_xfer: xfer=%p\n", xfer);
	if (xfer == NULL) {
		return;
	}
	if (xfer->pipe == NULL) {
		printf("xfer %p: pipe=NULL\n",
		    xfer);
		return;
	}
	printf("xfer %p: udev=%p vid=0x%04x pid=0x%04x addr=%d "
	    "pipe=%p ep=0x%02x attr=0x%02x\n",
	    xfer, xfer->udev,
	    UGETW(xfer->udev->ddesc.idVendor),
	    UGETW(xfer->udev->ddesc.idProduct),
	    xfer->udev->address, xfer->pipe,
	    xfer->pipe->edesc->bEndpointAddress,
	    xfer->pipe->edesc->bmAttributes);
	return;
}

#endif

/*------------------------------------------------------------------------*
 *	usb_get_devid
 *
 * This function returns the USB Vendor and Product ID like a 32-bit
 * unsigned integer.
 *------------------------------------------------------------------------*/
uint32_t
usb_get_devid(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	return ((uaa->vendor << 16) | (uaa->product));
}

/*------------------------------------------------------------------------*
 *	usbd_get_pipe_by_addr
 *
 * This function searches for an USB pipe by endpoint address and
 * direction.
 *
 * Returns:
 * NULL: Failure
 * Else: Success
 *------------------------------------------------------------------------*/
struct usbd_pipe *
usbd_get_pipe_by_addr(struct usbd_device *udev, uint8_t ea_val)
{
	struct usbd_pipe *pipe = udev->pipes;
	struct usbd_pipe *pipe_end = udev->pipes_end;
	enum {
		EA_MASK = (UE_DIR_IN | UE_DIR_OUT | UE_ADDR),
	};

	/*
	 * According to the USB specification not all bits are used
	 * for the endpoint address. Keep defined bits only:
	 */
	ea_val &= EA_MASK;

	/*
	 * Iterate accross all the USB pipes searching for a match
	 * based on the endpoint address:
	 */
	for (; pipe != pipe_end; pipe++) {

		if (pipe->edesc == NULL) {
			continue;
		}
		/* do the mask and check the value */
		if ((pipe->edesc->bEndpointAddress & EA_MASK) == ea_val) {
			goto found;
		}
	}

	/*
	 * The default pipe is always present and is checked separately:
	 */
	if ((udev->default_pipe.edesc) &&
	    ((udev->default_pipe.edesc->bEndpointAddress & EA_MASK) == ea_val)) {
		pipe = &udev->default_pipe;
		goto found;
	}
	return (NULL);

found:
	return (pipe);
}

/*------------------------------------------------------------------------*
 *	usbd_get_pipe
 *
 * This function searches for an USB pipe based on the information
 * given by the passed "struct usbd_config" pointer.
 *
 * Return values:
 * NULL: No match.
 * Else: Pointer to "struct usbd_pipe".
 *------------------------------------------------------------------------*/
struct usbd_pipe *
usbd_get_pipe(struct usbd_device *udev, uint8_t iface_index,
    const struct usbd_config *setup)
{
	struct usbd_pipe *pipe = udev->pipes;
	struct usbd_pipe *pipe_end = udev->pipes_end;
	uint8_t index = setup->ep_index;
	uint8_t ea_mask;
	uint8_t ea_val;
	uint8_t type_mask;
	uint8_t type_val;

	PRINTFN(8, ("udev=%p iface_index=%d address=0x%x "
	    "type=0x%x dir=0x%x index=%d\n",
	    udev, iface_index, setup->endpoint,
	    setup->type, setup->direction, setup->ep_index));

	/* setup expected endpoint direction mask and value */

	if (setup->direction == UE_DIR_ANY) {
		/* match any endpoint direction */
		ea_mask = 0;
		ea_val = 0;
	} else {
		/* match the given endpoint direction */
		ea_mask = (UE_DIR_IN | UE_DIR_OUT);
		ea_val = (setup->direction & (UE_DIR_IN | UE_DIR_OUT));
	}

	/* setup expected endpoint address */

	if (setup->endpoint == UE_ADDR_ANY) {
		/* match any endpoint address */
	} else {
		/* match the given endpoint address */
		ea_mask |= UE_ADDR;
		ea_val |= (setup->endpoint & UE_ADDR);
	}

	/* setup expected endpoint type */

	if (setup->type == UE_BULK_INTR) {
		/* this will match BULK and INTERRUPT endpoints */
		type_mask = 2;
		type_val = 2;
	} else if (setup->type == UE_TYPE_ANY) {
		/* match any endpoint type */
		type_mask = 0;
		type_val = 0;
	} else {
		/* match the given endpoint type */
		type_mask = UE_XFERTYPE;
		type_val = (setup->type & UE_XFERTYPE);
	}

	/*
	 * Iterate accross all the USB pipes searching for a match
	 * based on the endpoint address. Note that we are searching
	 * the pipes from the beginning of the "udev->pipes" array.
	 */
	for (; pipe != pipe_end; pipe++) {

		if ((pipe->edesc == NULL) ||
		    (pipe->iface_index != iface_index)) {
			continue;
		}
		/* do the masks and check the values */

		if (((pipe->edesc->bEndpointAddress & ea_mask) == ea_val) &&
		    ((pipe->edesc->bmAttributes & type_mask) == type_val)) {
			if (!index--) {
				goto found;
			}
		}
	}

	/*
	 * Match against default pipe last, so that "any pipe", "any
	 * address" and "any direction" returns the first pipe of the
	 * interface. "iface_index" and "direction" is ignored:
	 */
	if ((udev->default_pipe.edesc) &&
	    ((udev->default_pipe.edesc->bEndpointAddress & ea_mask) == ea_val) &&
	    ((udev->default_pipe.edesc->bmAttributes & type_mask) == type_val) &&
	    (!index)) {
		pipe = &udev->default_pipe;
		goto found;
	}
	return (NULL);

found:
	return (pipe);
}

/*------------------------------------------------------------------------*
 *	usbd_interface_count
 *
 * This function stores the number of USB interfaces excluding
 * alternate settings, which the USB config descriptor reports into
 * the unsigned 8-bit integer pointed to by "count".
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_interface_count(struct usbd_device *udev, uint8_t *count)
{
	if (udev->cdesc == NULL) {
		return (USBD_ERR_NOT_CONFIGURED);
	}
	*count = udev->cdesc->bNumInterface;
	return (USBD_ERR_NORMAL_COMPLETION);
}

struct usbd_std_packet_size {
	struct {
		uint16_t min;		/* inclusive */
		uint16_t max;		/* inclusive */
	}	range;

	uint16_t fixed[4];

};

/*
 * This table stores the all the allowed packet sizes based on
 * endpoint type and USB speed:
 */
static const struct usbd_std_packet_size
	usbd_std_packet_size[4][USB_SPEED_MAX] = {

	[UE_INTERRUPT] = {
		[USB_SPEED_LOW] = {.range = {0, 8}},
		[USB_SPEED_FULL] = {.range = {0, 64}},
		[USB_SPEED_HIGH] = {.range = {0, 1024}},
		[USB_SPEED_VARIABLE] = {.range = {0, 1024}},
	},

	[UE_CONTROL] = {
		[USB_SPEED_LOW] = {.fixed = {8, 8, 8, 8}},
		[USB_SPEED_FULL] = {.fixed = {8, 16, 32, 64}},
		[USB_SPEED_HIGH] = {.fixed = {64, 64, 64, 64}},
		[USB_SPEED_VARIABLE] = {.fixed = {512, 512, 512, 512}},
	},

	[UE_BULK] = {
		[USB_SPEED_LOW] = {},	/* invalid (all zero) */
		[USB_SPEED_FULL] = {.fixed = {8, 16, 32, 64}},
		[USB_SPEED_HIGH] = {.fixed = {512, 512, 512, 512}},
		[USB_SPEED_VARIABLE] = {.fixed = {512, 512, 1024, 1536}},
	},

	[UE_ISOCHRONOUS] = {
		[USB_SPEED_LOW] = {},	/* invalid (all zero) */
		[USB_SPEED_FULL] = {.range = {0, 1023}},
		[USB_SPEED_HIGH] = {.range = {0, 1024}},
		[USB_SPEED_VARIABLE] = {.range = {0, 3584}},
	},
};

/*------------------------------------------------------------------------*
 *	usbd_compute_max_frame_size
 *
 * This function computes the maximum frame size, hence high speed USB
 * can transfer multiple consecutive packets.
 *------------------------------------------------------------------------*/
static void
usbd_compute_max_frame_size(struct usbd_xfer *xfer)
{
	/* compute maximum frame size */

	if (xfer->max_packet_count == 2) {
		xfer->max_frame_size = 2 * xfer->max_packet_size;
	} else if (xfer->max_packet_count == 3) {
		xfer->max_frame_size = 3 * xfer->max_packet_size;
	} else {
		xfer->max_frame_size = xfer->max_packet_size;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_setup_sub - transfer setup subroutine
 *
 * This function must be called from the "xfer_setup" callback of the
 * USB Host or Device controller driver when setting up an USB
 * transfer. This function will setup correct packet sizes, buffer
 * sizes, flags and more, that are stored in the "usbd_xfer"
 * structure.
 *------------------------------------------------------------------------*/
void
usbd_transfer_setup_sub(struct usbd_setup_params *parm)
{
	enum {
		REQ_SIZE = 8,
		MIN_PKT = 8,
	};
	struct usbd_xfer *xfer = parm->curr_xfer;
	const struct usbd_config_sub *setup_sub = parm->curr_setup_sub;
	usb_endpoint_descriptor_t *edesc;
	struct usbd_std_packet_size std_size;
	uint32_t n_frlengths;
	uint32_t n_frbuffers;
	uint32_t x;
	uint8_t type;
	uint8_t zmps;

	/*
	 * Sanity check. The following parameters must be initialized before
	 * calling this function.
	 */
	if ((parm->hc_max_packet_size == 0) ||
	    (parm->hc_max_packet_count == 0) ||
	    (parm->hc_max_frame_size == 0)) {
		parm->err = USBD_ERR_INVAL;
		goto done;
	}
	edesc = xfer->pipe->edesc;

	type = (edesc->bmAttributes & UE_XFERTYPE);

	xfer->flags = setup_sub->flags;
	xfer->nframes = setup_sub->frames;
	xfer->timeout = setup_sub->timeout;
	xfer->callback = setup_sub->callback;
	xfer->interval = setup_sub->interval;
	xfer->endpoint = edesc->bEndpointAddress;
	xfer->max_packet_size = UGETW(edesc->wMaxPacketSize);
	xfer->max_packet_count = 1;
	xfer->flags_int.usb_mode = parm->udev->flags.usb_mode;	/* make a shadow copy */

	parm->bufsize = setup_sub->bufsize;

	if (parm->speed == USB_SPEED_HIGH) {
		xfer->max_packet_count += (xfer->max_packet_size >> 11) & 3;
		xfer->max_packet_size &= 0x7FF;
	}
	/* range check "max_packet_count" */

	if (xfer->max_packet_count > parm->hc_max_packet_count) {
		xfer->max_packet_count = parm->hc_max_packet_count;
	}
	/* filter "wMaxPacketSize" according to HC capabilities */

	if (xfer->max_packet_size > parm->hc_max_packet_size) {
		xfer->max_packet_size = parm->hc_max_packet_size;
	}
	/* filter "wMaxPacketSize" according to standard sizes */

	std_size = usbd_std_packet_size[type][parm->speed];

	if (std_size.range.min || std_size.range.max) {

		if (xfer->max_packet_size < std_size.range.min) {
			xfer->max_packet_size = std_size.range.min;
		}
		if (xfer->max_packet_size > std_size.range.max) {
			xfer->max_packet_size = std_size.range.max;
		}
	} else {

		if (xfer->max_packet_size >= std_size.fixed[3]) {
			xfer->max_packet_size = std_size.fixed[3];
		} else if (xfer->max_packet_size >= std_size.fixed[2]) {
			xfer->max_packet_size = std_size.fixed[2];
		} else if (xfer->max_packet_size >= std_size.fixed[1]) {
			xfer->max_packet_size = std_size.fixed[1];
		} else {
			/* only one possibility left */
			xfer->max_packet_size = std_size.fixed[0];
		}
	}

	/* compute "max_frame_size" */

	usbd_compute_max_frame_size(xfer);

	/* check interrupt interval and transfer pre-delay */

	if (type == UE_ISOCHRONOUS) {

		uint32_t frame_limit;

		xfer->interval = 0;	/* not used, must be zero */
		xfer->flags_int.isochronous_xfr = 1;	/* set flag */

		if (xfer->timeout == 0) {
			/*
			 * set a default timeout in
			 * case something goes wrong!
			 */
			xfer->timeout = 1000 / 4;
		}
		if (parm->speed == USB_SPEED_HIGH) {
			frame_limit = USB_MAX_HS_ISOC_FRAMES_PER_XFER;
		} else {
			frame_limit = USB_MAX_FS_ISOC_FRAMES_PER_XFER;
		}

		if (xfer->nframes > frame_limit) {
			/*
			 * this is not going to work
			 * cross hardware
			 */
			parm->err = USBD_ERR_INVAL;
			goto done;
		}
		if (xfer->nframes == 0) {
			/*
			 * this is not a valid value
			 */
			parm->err = USBD_ERR_ZERO_NFRAMES;
			goto done;
		}
	} else {

		/*
		 * if a value is specified use that else check the endpoint
		 * descriptor
		 */
		if (xfer->interval == 0) {

			if (type == UE_INTERRUPT) {

				xfer->interval = edesc->bInterval;

				if (parm->speed == USB_SPEED_HIGH) {
					xfer->interval /= 8;	/* 125us -> 1ms */
				}
				if (xfer->interval == 0) {
					/*
					 * one millisecond is the smallest
					 * interval
					 */
					xfer->interval = 1;
				}
			}
		}
	}

	/*
	 * NOTE: we do not allow "max_packet_size" or "max_frame_size"
	 * to be equal to zero when setting up USB transfers, hence
	 * this leads to alot of extra code in the USB kernel.
	 */

	if ((xfer->max_frame_size == 0) ||
	    (xfer->max_packet_size == 0)) {

		zmps = 1;

		if ((parm->bufsize <= MIN_PKT) &&
		    (type != UE_CONTROL) &&
		    (type != UE_BULK)) {

			/* workaround */
			xfer->max_packet_size = MIN_PKT;
			xfer->max_packet_count = 1;
			parm->bufsize = 0;	/* automatic setup length */
			usbd_compute_max_frame_size(xfer);

		} else {
			parm->err = USBD_ERR_ZERO_MAXP;
			goto done;
		}

	} else {
		zmps = 0;
	}

	/*
	 * check if we should setup a default
	 * length:
	 */

	if (parm->bufsize == 0) {

		parm->bufsize = xfer->max_frame_size;

		if (type == UE_ISOCHRONOUS) {
			parm->bufsize *= xfer->nframes;
		}
	}
	/*
	 * check if we are about to setup a proxy
	 * type of buffer:
	 */

	if (xfer->flags.proxy_buffer) {

		/* round bufsize up */

		parm->bufsize += (xfer->max_frame_size - 1);

		if (parm->bufsize < xfer->max_frame_size) {
			/* length wrapped around */
			parm->err = USBD_ERR_INVAL;
			goto done;
		}
		/* subtract remainder */

		parm->bufsize -= (parm->bufsize % xfer->max_frame_size);

		/* add length of USB device request structure, if any */

		if (type == UE_CONTROL) {
			parm->bufsize += REQ_SIZE;	/* SETUP message */
		}
	}
	xfer->max_data_length = parm->bufsize;

	/* Setup "n_frlengths" and "n_frbuffers" */

	if (type == UE_ISOCHRONOUS) {
		n_frlengths = xfer->nframes;
		n_frbuffers = 1;
	} else {

		if (type == UE_CONTROL) {
			xfer->flags_int.control_xfr = 1;
			if (xfer->nframes == 0) {
				if (parm->bufsize <= REQ_SIZE) {
					/*
					 * there will never be any data
					 * stage
					 */
					xfer->nframes = 1;
				} else {
					xfer->nframes = 2;
				}
			}
		} else {
			if (xfer->nframes == 0) {
				xfer->nframes = 1;
			}
		}

		n_frlengths = xfer->nframes;
		n_frbuffers = xfer->nframes;
	}

	/*
	 * check if we have room for the
	 * USB device request structure:
	 */

	if (type == UE_CONTROL) {

		if (xfer->max_data_length < REQ_SIZE) {
			/* length wrapped around or too small bufsize */
			parm->err = USBD_ERR_INVAL;
			goto done;
		}
		xfer->max_data_length -= REQ_SIZE;
	}
	/* setup "frlengths" */

	xfer->frlengths = parm->xfer_length_ptr;

	parm->xfer_length_ptr += n_frlengths;

	/* setup "frbuffers" */

	xfer->frbuffers = parm->xfer_page_cache_ptr;

	parm->xfer_page_cache_ptr += n_frbuffers;

	/*
	 * check if we need to setup
	 * a local buffer:
	 */

	if (!xfer->flags.ext_buffer) {

		/* align data */
		parm->size[0] += ((-parm->size[0]) & (USB_HOST_ALIGN - 1));

		if (parm->buf) {

			xfer->local_buffer =
			    USBD_ADD_BYTES(parm->buf, parm->size[0]);

			usbd_set_frame_offset(xfer, 0, 0);

			if (type == UE_CONTROL) {
				usbd_set_frame_offset(xfer, REQ_SIZE, 1);
			}
		}
		parm->size[0] += parm->bufsize;

		/* align data again */
		parm->size[0] += ((-parm->size[0]) & (USB_HOST_ALIGN - 1));
	}
	/*
	 * Compute maximum buffer size
	 */

	if (parm->bufsize_max < parm->bufsize) {
		parm->bufsize_max = parm->bufsize;
	}
	if (xfer->flags_int.bdma_enable) {
		/*
		 * Setup "dma_page_ptr".
		 *
		 * Proof for formula below:
		 *
		 * Assume there are three USB frames having length "a", "b" and
		 * "c". These USB frames will at maximum need "z"
		 * "usbd_page" structures. "z" is given by:
		 *
		 * z = ((a / USB_PAGE_SIZE) + 2) + ((b / USB_PAGE_SIZE) + 2) +
		 * ((c / USB_PAGE_SIZE) + 2);
		 *
		 * Constraining "a", "b" and "c" like this:
		 *
		 * (a + b + c) <= parm->bufsize
		 *
		 * We know that:
		 *
		 * z <= ((parm->bufsize / USB_PAGE_SIZE) + (3*2));
		 *
		 * Here is the general formula:
		 */
		xfer->dma_page_ptr = parm->dma_page_ptr;
		parm->dma_page_ptr += (2 * n_frbuffers);
		parm->dma_page_ptr += (parm->bufsize / USB_PAGE_SIZE);
	}
	if (zmps) {
		/* correct maximum data length */
		xfer->max_data_length = 0;
	}
	/* subtract USB frame remainder from "hc_max_frame_size" */

	xfer->max_usb_frame_size =
	    (parm->hc_max_frame_size -
	    (parm->hc_max_frame_size % xfer->max_frame_size));

	if (xfer->max_usb_frame_size == 0) {
		parm->err = USBD_ERR_INVAL;
		goto done;
	}
	/* initialize frame buffers */

	if (parm->buf) {
		for (x = 0; x != n_frbuffers; x++) {
			xfer->frbuffers[x].tag_parent =
			    &(xfer->usb_root->dma_parent_tag);

			if (xfer->flags_int.bdma_enable &&
			    (parm->bufsize_max > 0)) {

				if (usbd_pc_dmamap_create(
				    xfer->frbuffers + x,
				    parm->bufsize_max)) {
					parm->err = USBD_ERR_NOMEM;
					goto done;
				}
			}
		}
	}
done:
	if (parm->err) {
		/*
		 * Set some dummy values so that we avoid division by zero:
		 */
		xfer->max_usb_frame_size = 1;
		xfer->max_frame_size = 1;
		xfer->max_packet_size = 1;
		xfer->max_data_length = 0;
		xfer->nframes = 0;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_setup - setup an array of USB transfers
 *
 * NOTE: You must always call "usbd_transfer_unsetup" after calling
 * "usbd_transfer_setup" if success was returned.
 *
 * The idea is that the USB device driver should pre-allocate all its
 * transfers by one call to this function.
 *
 * Return values:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_transfer_setup(struct usbd_device *udev,
    const uint8_t *ifaces, struct usbd_xfer **ppxfer,
    const struct usbd_config *setup_start, uint16_t n_setup,
    void *priv_sc, struct mtx *priv_mtx)
{
	struct usbd_xfer dummy;
	struct usbd_setup_params parm;
	const struct usbd_config *setup_end = setup_start + n_setup;
	const struct usbd_config *setup;
	struct usbd_pipe *pipe;
	struct usbd_memory_info *info;
	struct usbd_xfer *xfer;
	void *buf = NULL;
	uint16_t n;
	uint16_t refcount;

	parm.err = 0;
	refcount = 0;
	info = NULL;

	WITNESS_WARN(WARN_GIANTOK | WARN_SLEEPOK, NULL,
	    "usbd_transfer_setup can sleep!");

	/* do some checking first */

	if (n_setup == 0) {
		PRINTFN(5, ("setup array has zero length!\n"));
		return (USBD_ERR_INVAL);
	}
	if (ifaces == 0) {
		PRINTFN(5, ("ifaces array is NULL!\n"));
		return (USBD_ERR_INVAL);
	}
	if (priv_mtx == NULL) {
		PRINTFN(5, ("using global lock\n"));
		priv_mtx = &usb_global_lock;
	}
	/* sanity checks */
	for (setup = setup_start, n = 0;
	    setup != setup_end; setup++, n++) {
		if ((setup->mh.bufsize == 0xffffffff) ||
		    (setup->md.bufsize == 0xffffffff)) {
			parm.err = USBD_ERR_BAD_BUFSIZE;
			PRINTF(("invalid bufsize\n"));
		}
		if ((setup->mh.callback == NULL) &&
		    (setup->md.callback == NULL)) {
			parm.err = USBD_ERR_NO_CALLBACK;
			PRINTF(("no callback\n"));
		}
		ppxfer[n] = NULL;
	}

	if (parm.err) {
		goto done;
	}
	bzero(&parm, sizeof(parm));

	parm.udev = udev;
	parm.speed = usbd_get_speed(udev);
	parm.hc_max_packet_count = 1;

	if (parm.speed >= USB_SPEED_MAX) {
		parm.err = USBD_ERR_INVAL;
		goto done;
	}
	/* setup all transfers */

	while (1) {

		if (buf) {
			/*
			 * Initialize the "usbd_memory_info" structure,
			 * which is common for all our USB transfers.
			 */
			info = USBD_ADD_BYTES(buf, 0);

			info->memory_base = buf;
			info->memory_size = parm.size[0];

			info->dma_page_cache_start = USBD_ADD_BYTES(buf, parm.size[4]);
			info->dma_page_cache_end = USBD_ADD_BYTES(buf, parm.size[5]);
			info->xfer_page_cache_start = USBD_ADD_BYTES(buf, parm.size[5]);
			info->xfer_page_cache_end = USBD_ADD_BYTES(buf, parm.size[2]);

			info->usb_mtx = &(udev->bus->mtx);
			info->priv_mtx = priv_mtx;

			usbd_dma_tag_setup(&(info->dma_parent_tag),
			    parm.dma_tag_p, udev->bus->dma_parent_tag[0].tag,
			    priv_mtx, &usbd_bdma_done_event, info, 32, parm.dma_tag_max);

			info->bus = udev->bus;

			LIST_INIT(&(info->done_head));

			/* create a callback thread */

			if (usb_thread_create
			    (&usbd_callback_intr_td, info,
			    &(info->done_thread), "USB interrupt thread")) {
				info->done_thread = NULL;
				parm.err = USBD_ERR_NO_INTR_THREAD;
				goto done;
			}
		}
		/* reset sizes */

		parm.size[0] = 0;
		parm.buf = buf;
		parm.size[0] += sizeof(info[0]);

		for (setup = setup_start, n = 0;
		    setup != setup_end; setup++, n++) {

			/* select mode specific structure */
			if (udev->flags.usb_mode == USB_MODE_HOST) {
				parm.curr_setup_sub = &(setup->mh);
			} else {
				parm.curr_setup_sub = &(setup->md);
			}
			/* skip USB transfers without callbacks: */
			if (parm.curr_setup_sub->callback == NULL) {
				continue;
			}
			/* see if there is a matching endpoint */
			pipe = usbd_get_pipe(udev,
			    ifaces[setup->if_index], setup);

			if (!pipe) {
				if (parm.curr_setup_sub->flags.no_pipe_ok) {
					continue;
				}
				parm.err = USBD_ERR_NO_PIPE;
				goto done;
			}
			/* store current setup pointer */
			parm.curr_setup = setup;

			/* align data properly */
			parm.size[0] += ((-parm.size[0]) & (USB_HOST_ALIGN - 1));

			if (buf) {

				/*
				 * Common initialization of the
				 * "usbd_xfer" structure.
				 */
				xfer = USBD_ADD_BYTES(buf, parm.size[0]);

				ppxfer[n] = xfer;
				xfer->udev = udev;
				xfer->address = udev->address;
				xfer->priv_sc = priv_sc;
				xfer->priv_mtx = priv_mtx;
				xfer->usb_mtx = &(udev->bus->mtx);
				xfer->usb_root = info;
				info->memory_refcount++;
				info->setup_refcount++;

				usb_callout_init_mtx(&xfer->timeout_handle, xfer->usb_mtx,
				    CALLOUT_RETURNUNLOCKED);
			} else {
				/*
				 * Setup a dummy xfer, hence we are
				 * writing to the "usbd_xfer"
				 * structure pointed to by "xfer"
				 * before we have allocated any
				 * memory:
				 */
				xfer = &dummy;
				bzero(&dummy, sizeof(dummy));
				refcount++;
			}

			parm.size[0] += sizeof(xfer[0]);

			xfer->pipe = pipe;

			if (buf) {
				/*
				 * Increment the pipe refcount. This
				 * basically prevents setting a new
				 * configuration and alternate setting
				 * when USB transfers are in use on
				 * the given interface. Search the USB
				 * code for "pipe->refcount" if you
				 * want more information.
				 */
				xfer->pipe->refcount++;
			}
			parm.methods = xfer->pipe->methods;
			parm.curr_xfer = xfer;

			/*
			 * Call the Host or Device controller transfer setup
			 * routine:
			 */
			(udev->bus->methods->xfer_setup) (&parm);

			if (parm.err) {
				goto done;
			}
		}

		if (buf || parm.err) {
			goto done;
		}
		if (refcount == 0) {
			/* no transfers - nothing to do ! */
			goto done;
		}
		/* align data properly */
		parm.size[0] += ((-parm.size[0]) & (USB_HOST_ALIGN - 1));

		/* store offset temporarily */
		parm.size[1] = parm.size[0];

		/*
		 * UHCI need DMA tags for fixup buffers. There
		 * is a maximum of one tag for each endpoint.
		 */
		parm.dma_tag_max += MIN(n_setup, USB_MAX_ENDPOINTS);

		/*
		 * DMA tags for QH, TD, Data and more.
		 */
		parm.dma_tag_max += 8;

		parm.dma_tag_p += parm.dma_tag_max;

		parm.size[0] += ((uint8_t *)parm.dma_tag_p) -
		    ((uint8_t *)0);

		/* align data properly */
		parm.size[0] += ((-parm.size[0]) & (USB_HOST_ALIGN - 1));

		/* store offset temporarily */
		parm.size[3] = parm.size[0];

		parm.size[0] += ((uint8_t *)parm.dma_page_ptr) -
		    ((uint8_t *)0);

		/* align data properly */
		parm.size[0] += ((-parm.size[0]) & (USB_HOST_ALIGN - 1));

		/* store offset temporarily */
		parm.size[4] = parm.size[0];

		parm.size[0] += ((uint8_t *)parm.dma_page_cache_ptr) -
		    ((uint8_t *)0);

		/* store end offset temporarily */
		parm.size[5] = parm.size[0];

		parm.size[0] += ((uint8_t *)parm.xfer_page_cache_ptr) -
		    ((uint8_t *)0);

		/* store end offset temporarily */

		parm.size[2] = parm.size[0];

		/* align data properly */
		parm.size[0] += ((-parm.size[0]) & (USB_HOST_ALIGN - 1));

		parm.size[6] = parm.size[0];

		parm.size[0] += ((uint8_t *)parm.xfer_length_ptr) -
		    ((uint8_t *)0);

		/* align data properly */
		parm.size[0] += ((-parm.size[0]) & (USB_HOST_ALIGN - 1));

		/* allocate zeroed memory */
		buf = malloc(parm.size[0], M_USB, M_WAITOK | M_ZERO);

		if (buf == NULL) {
			parm.err = USBD_ERR_NOMEM;
			PRINTFN(-1, ("cannot allocate memory block for "
			    "configuration (%d bytes)\n",
			    parm.size[0]));
			goto done;
		}
		parm.dma_tag_p = USBD_ADD_BYTES(buf, parm.size[1]);
		parm.dma_page_ptr = USBD_ADD_BYTES(buf, parm.size[3]);
		parm.dma_page_cache_ptr = USBD_ADD_BYTES(buf, parm.size[4]);
		parm.xfer_page_cache_ptr = USBD_ADD_BYTES(buf, parm.size[5]);
		parm.xfer_length_ptr = USBD_ADD_BYTES(buf, parm.size[6]);
	}

done:
	if (buf) {
		if (info->setup_refcount == 0) {
			/*
			 * "usbd_transfer_unsetup_sub" will unlock
			 * "usb_mtx" before returning !
			 */
			mtx_lock(info->usb_mtx);

			/* something went wrong */
			usbd_transfer_unsetup_sub(info, 0);
		}
	}
	if (parm.err) {
		usbd_transfer_unsetup(ppxfer, n_setup);
	}
	return (parm.err);
}

/*------------------------------------------------------------------------*
 *	usbd_get_dma_delay
 *
 * The following function is called when we need to
 * synchronize with DMA hardware.
 *
 * Returns:
 *    0: no DMA delay required
 * Else: milliseconds of DMA delay
 *------------------------------------------------------------------------*/
static uint32_t
usbd_get_dma_delay(struct usbd_bus *bus)
{
	uint32_t temp = 0;

	if (bus->methods->get_dma_delay) {
		(bus->methods->get_dma_delay) (bus, &temp);
		/*
		 * Round up and convert to milliseconds. Note that we use
		 * 1024 milliseconds per second. to save a division.
		 */
		temp += 0x3FF;
		temp /= 0x400;
	}
	return (temp);
}

static void
usbd_transfer_unsetup_sub(struct usbd_memory_info *info, uint8_t needs_delay)
{
	struct usbd_page_cache *pc;
	uint32_t temp;

	mtx_assert(info->usb_mtx, MA_OWNED);

	/* wait for any outstanding DMA operations */

	if (needs_delay) {
		temp = usbd_get_dma_delay(info->bus);
		usbd_pause_mtx(info->usb_mtx, temp);
	}
	/* wait for interrupt thread to exit */

	while (info->done_thread) {

		usbd_callback_intr_sched(info);

		if (mtx_sleep(&(info->done_thread), info->usb_mtx,
		    0, "usbdwait", 0)) {
		}
	}

	mtx_unlock(info->usb_mtx);

	/* free DMA'able memory, if any */
	pc = info->dma_page_cache_start;
	while (pc != info->dma_page_cache_end) {
		usbd_pc_free_mem(pc);
		pc++;
	}

	/* free DMA maps in all "xfer->frbuffers" */
	pc = info->xfer_page_cache_start;
	while (pc != info->xfer_page_cache_end) {
		usbd_pc_dmamap_destroy(pc);
		pc++;
	}

	/* free all DMA tags */
	usbd_dma_tag_unsetup(&(info->dma_parent_tag));

	/*
	 * free the "memory_base" last, hence the "info" structure is
	 * contained within the "memory_base"!
	 */
	free(info->memory_base, M_USB);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_unsetup - unsetup/free an array of USB transfers
 *
 * NOTE: All USB transfers in progress will get called back passing
 * the error code "USBD_ERR_CANCELLED" before this function
 * returns.
 *------------------------------------------------------------------------*/
void
usbd_transfer_unsetup(struct usbd_xfer **pxfer, uint16_t n_setup)
{
	struct usbd_xfer *xfer;
	struct usbd_memory_info *info;
	uint8_t needs_delay = 0;

	WITNESS_WARN(WARN_GIANTOK | WARN_SLEEPOK, NULL,
	    "usbd_transfer_unsetup can sleep!");

	while (n_setup--) {
		xfer = pxfer[n_setup];

		if (xfer) {
			if (xfer->pipe) {
				mtx_lock(xfer->priv_mtx);

				/*
				 * HINT: when you start/stop a transfer, it
				 * might be a good idea to directly use the
				 * "pxfer[]" structure:
				 *
				 * usbd_transfer_start(sc->pxfer[0]);
				 * usbd_transfer_stop(sc->pxfer[0]);
				 *
				 * That way, if your code has many parts that
				 * will not stop running under the same
				 * lock, in other words "priv_mtx", the
				 * usbd_transfer_start and
				 * usbd_transfer_stop functions will simply
				 * return when they detect a NULL pointer
				 * argument.
				 *
				 * To avoid any races we clear the "pxfer[]"
				 * pointer while holding the private mutex
				 * of the driver:
				 */
				pxfer[n_setup] = NULL;

				mtx_unlock(xfer->priv_mtx);

				usbd_transfer_drain(xfer);

				if (xfer->flags_int.bdma_enable) {
					needs_delay = 1;
				}
				/*
				 * NOTE: default pipe does not have an
				 * interface, even if pipe->iface_index == 0
				 */
				xfer->pipe->refcount--;

			} else {
				/* clear the transfer pointer */
				pxfer[n_setup] = NULL;
			}

			usb_callout_drain(&(xfer->timeout_handle));

			if (xfer->usb_root) {
				info = xfer->usb_root;

				mtx_lock(info->usb_mtx);

				__KASSERT(info->memory_refcount != 0,
				    ("Invalid memory "
				    "reference count!\n"));

				__KASSERT(info->setup_refcount != 0,
				    ("Invalid setup "
				    "reference count!\n"));

				info->memory_refcount--;
				info->setup_refcount--;

				if (info->setup_refcount == 0) {
					usbd_transfer_unsetup_sub(info,
					    needs_delay);
				} else {
					mtx_unlock(info->usb_mtx);
				}
			}
		}
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_std_root_transfer - factored out code
 *
 * This function is basically used for the Virtual Root HUB, and can
 * emulate control, bulk and interrupt endpoints. Data is exchanged
 * using the "std->ptr" and "std->len" fields, that allows kernel
 * virtual memory to be transferred. All state is kept in the
 * structure pointed to by the "std" argument passed to this
 * function. The "func" argument points to a function that is called
 * back in the various states, so that the application using this
 * function can get a chance to select the outcome. The "func"
 * function is allowed to sleep, exiting all mutexes.
 *------------------------------------------------------------------------*/
void
usbd_std_root_transfer(struct usbd_std_root_transfer *std,
    usbd_std_root_transfer_func_t *func)
{
	struct usbd_xfer *xfer;
	struct thread *td;
	uint32_t len;
	uint8_t shortpkt = 0;

	xfer = std->xfer;
	if (xfer == NULL) {
		/* the transfer is gone */
		return;
	}
	mtx_assert(xfer->usb_mtx, MA_OWNED);

	std->xfer = NULL;

	/* get current thread */
	td = curthread;

	/* signal that we plan to do the callback */
	xfer->usb_thread = td;

	/* check for control transfer */
	if (xfer->flags_int.control_xfr) {
		/* check if we are transferring the SETUP packet */
		if (xfer->flags_int.control_hdr) {

			/* copy out the USB request */

			if (xfer->frlengths[0] == sizeof(std->req)) {
				usbd_copy_out(xfer->frbuffers, 0,
				    &(std->req), sizeof(std->req));
			} else {
				std->err = USBD_ERR_INVAL;
				goto done;
			}

			xfer->aframes = 1;

			std->err = 0;
			std->state = USBD_STD_ROOT_TR_SETUP;

			(func) (xfer, std);

			if (xfer->usb_thread != td) {
				/* transfer cancelled */
				goto done;
			}
			if (std->err) {
				goto done;
			}
		} else {
			/* skip the first frame in this case */
			xfer->aframes = 1;
		}
	}
	std->err = 0;
	std->state = USBD_STD_ROOT_TR_PRE_DATA;

	(func) (xfer, std);

	if (xfer->usb_thread != td) {
		/* transfer cancelled */
		goto done;
	}
	if (std->err) {
		goto done;
	}
	/* Transfer data. Iterate accross all frames. */
	while (xfer->aframes != xfer->nframes) {

		len = xfer->frlengths[xfer->aframes];

		if (len > std->len) {
			len = std->len;
			shortpkt = 1;
		}
		if (len > 0) {
			if ((xfer->endpoint & (UE_DIR_IN | UE_DIR_OUT)) == UE_DIR_IN) {
				usbd_copy_in(xfer->frbuffers + xfer->aframes, 0,
				    std->ptr, len);
			} else {
				usbd_copy_out(xfer->frbuffers + xfer->aframes, 0,
				    std->ptr, len);
			}
		}
		std->ptr += len;
		std->len -= len;
		xfer->frlengths[xfer->aframes] = len;
		xfer->aframes++;

		if (shortpkt) {
			break;
		}
	}

	std->err = 0;
	std->state = USBD_STD_ROOT_TR_POST_DATA;

	(func) (xfer, std);

	if (xfer->usb_thread != td) {
		/* transfer cancelled */
		goto done;
	}
	if (std->err) {
		goto done;
	}
	/* check if the control transfer is complete */
	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		std->err = 0;
		std->state = USBD_STD_ROOT_TR_STATUS;

		(func) (xfer, std);

		if (xfer->usb_thread != td) {
			/* transfer cancelled */
			goto done;
		}
		if (std->err) {
			goto done;
		}
	}
done:
	/* check if we are still handling this USB transfer */

	if (xfer->usb_thread == td) {
		std->state = USBD_STD_ROOT_TR_PRE_CALLBACK;
		(func) (xfer, std);

		/* queue callback for execution */
		usbd_callback_wrapper(xfer, NULL, USBD_CONTEXT_CALLBACK);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_control_transfer_init - factored out code
 *
 * In USB Device Mode we have to wait for the SETUP packet which
 * containst the "usb_device_request_t" structure, before we can
 * transfer any data. In USB Host Mode we already have the SETUP
 * packet at the moment the USB transfer is started. This leads us to
 * having to setup the USB transfer at two different places in
 * time. This function just contains factored out control transfer
 * initialisation code, so that we don't duplicate the code.
 *------------------------------------------------------------------------*/
static void
usbd_control_transfer_init(struct usbd_xfer *xfer)
{
	usb_device_request_t req;

	/* copy out the USB request header */

	usbd_copy_out(xfer->frbuffers, 0, &req, sizeof(req));

	/* setup remainder */

	xfer->flags_int.control_rem = UGETW(req.wLength);

	/* copy direction to endpoint variable */

	xfer->endpoint &= ~(UE_DIR_IN | UE_DIR_OUT);
	xfer->endpoint |=
	    (req.bmRequestType & UT_READ) ? UE_DIR_IN : UE_DIR_OUT;

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_start_hardware_sub
 *
 * This function handles initialisation of control transfers. Control
 * transfers are special in that regard that they can both transmit
 * and receive data.
 *
 * Return values:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static uint8_t
usbd_start_hardware_sub(struct usbd_xfer *xfer)
{
	uint32_t len;

	/* Check for control endpoint stall */
	if (xfer->flags.stall_pipe) {
		/* no longer active */
		xfer->flags_int.control_act = 0;
	}
	/*
         * Check if there is a control
         * transfer in progress:
         */
	if (xfer->flags_int.control_act) {

		if (xfer->flags_int.control_hdr) {

			/* clear send header flag */

			xfer->flags_int.control_hdr = 0;

			/* setup control transfer */
			if (xfer->flags_int.usb_mode == USB_MODE_DEVICE) {
				usbd_control_transfer_init(xfer);
			}
		}
		/* get data length */

		len = xfer->sumlen;

	} else {

		/* the size of the SETUP structure is hardcoded ! */

		if (xfer->frlengths[0] != sizeof(usb_device_request_t)) {
			goto error;
		}
		/* check USB mode */
		if (xfer->flags_int.usb_mode == USB_MODE_DEVICE) {

			/* check number of frames */
			if (xfer->nframes != 1) {
				/*
			         * We need to receive the setup
			         * message first so that we know the
			         * data direction!
			         */
				PRINTFN(0, ("Misconfigured transfer\n"));
				goto error;
			}
			/*
			 * Set a dummy "control_rem" value.  This
			 * variable will be overwritten later by a
			 * call to "usbd_control_transfer_init()" !
			 */
			xfer->flags_int.control_rem = 0xFFFF;
		} else {

			/* setup "endpoint" and "control_rem" */

			usbd_control_transfer_init(xfer);
		}

		/* set transfer-header flag */

		xfer->flags_int.control_hdr = 1;

		/* get data length */

		len = (xfer->sumlen - sizeof(usb_device_request_t));
	}

	/* check if there is a length mismatch */

	if (len > xfer->flags_int.control_rem) {
		PRINTFN(-1, ("Length greater than remaining length!\n"));
		goto error;
	}
	/* check if we are doing a short transfer */

	if (xfer->flags.force_short_xfer) {
		xfer->flags_int.control_rem = 0;
	} else {
		if ((len != xfer->max_data_length) &&
		    (len != xfer->flags_int.control_rem) &&
		    (xfer->nframes != 1)) {
			PRINTFN(-1, ("Short control transfer without "
			    "force_short_xfer set!\n"));
			goto error;
		}
		xfer->flags_int.control_rem -= len;
	}

	/* the status part is executed when "control_act" is 0 */

	if ((xfer->flags_int.control_rem > 0) ||
	    (xfer->flags.manual_status)) {
		/* don't execute the STATUS stage yet */
		xfer->flags_int.control_act = 1;

		/* sanity check */
		if ((!xfer->flags_int.control_hdr) &&
		    (xfer->nframes == 1)) {
			/*
		         * This is not a valid operation!
		         */
			PRINTFN(-1, ("Invalid parameter "
			    "combination\n"));
			goto error;
		}
	} else {
		/* time to execute the STATUS stage */
		xfer->flags_int.control_act = 0;
	}
	return (0);			/* success */

error:
	return (1);			/* failure */
}

/*------------------------------------------------------------------------*
 *	usbd_premature_callback
 *
 * This function is used to do a completion callback passing the given
 * error code to the USB transfer, before the "pipe->methods->enter"
 * method has been called. It can optionally be called with "priv_mtx"
 * locked.
 *------------------------------------------------------------------------*/
static void
usbd_premature_callback(struct usbd_xfer *xfer, usbd_status_t error)
{
	mtx_lock(xfer->usb_mtx);
	usbd_transfer_dequeue(xfer, error);
	usbd_callback_wrapper(xfer, NULL, USBD_CONTEXT_CALLBACK);
	mtx_unlock(xfer->usb_mtx);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_start_hardware - start USB hardware for the given transfer
 *
 * This function should only be called from the USB callback.
 *------------------------------------------------------------------------*/
void
usbd_start_hardware(struct usbd_xfer *xfer)
{
	struct usbd_memory_info *info;
	uint32_t x;

	PRINTFN(0, ("xfer=%p, pipe=%p, nframes=%d, dir=%s\n",
	    xfer, xfer->pipe, xfer->nframes, USBD_GET_DATA_ISREAD(xfer) ?
	    "read" : "write"));

#ifdef USB_DEBUG
	if (usbdebug > 0) {
		mtx_lock(xfer->usb_mtx);

		usbd_dump_pipe(xfer->pipe);

		mtx_unlock(xfer->usb_mtx);
	}
#endif

	mtx_assert(xfer->priv_mtx, MA_OWNED);
	mtx_assert(xfer->usb_mtx, MA_NOTOWNED);

	/* sanity checks */

	if (!xfer->flags_int.started || xfer->flags_int.transferring) {
		PRINTFN(-1, ("sanity checks failed\n"));
		return;
	}
	/* Only open the USB transfer once! */
	if (!xfer->flags_int.open) {
		xfer->flags_int.open = 1;

		mtx_lock(xfer->usb_mtx);
		(xfer->pipe->methods->open) (xfer);
		mtx_unlock(xfer->usb_mtx);
	}
	/* set "transferring" and "recursed_2" flags */
	xfer->flags_int.transferring = 1;
	xfer->flags_int.recursed_2 = 1;

	/* clear "did_dma_delay" flag */
	xfer->flags_int.did_dma_delay = 0;

	/* clear lengths and frame counts by default */
	xfer->sumlen = 0;
	xfer->actlen = 0;
	xfer->aframes = 0;

	/* sanity check */

	if (xfer->nframes == 0) {
		usbd_premature_callback(xfer, USBD_ERR_INVAL);
		return;
	}
	/* compute total transfer length */

	for (x = 0; x != xfer->nframes; x++) {
		xfer->sumlen += xfer->frlengths[x];
		if (xfer->sumlen < xfer->frlengths[x]) {
			/* length wrapped around */
			usbd_premature_callback(xfer, USBD_ERR_INVAL);
			return;
		}
	}

	/* clear some internal flags */

	xfer->flags_int.short_xfer_ok = 0;
	xfer->flags_int.short_frames_ok = 0;

	/* check if this is a control transfer */

	if (xfer->flags_int.control_xfr) {

		if (usbd_start_hardware_sub(xfer)) {
			usbd_premature_callback(xfer, USBD_ERR_STALLED);
			return;
		}
	}
	/*
	 * Setup filtered version of some transfer flags,
	 * in case of data read direction
	 */
	if (USBD_GET_DATA_ISREAD(xfer)) {

		if (xfer->flags_int.control_xfr) {

			/*
			 * Control transfers do not support reception
			 * of multiple short USB frames !
			 */

			if (xfer->flags.short_xfer_ok) {
				xfer->flags_int.short_xfer_ok = 1;
			}
		} else {

			if (xfer->flags.short_frames_ok) {
				xfer->flags_int.short_xfer_ok = 1;
				xfer->flags_int.short_frames_ok = 1;
			} else if (xfer->flags.short_xfer_ok) {
				xfer->flags_int.short_xfer_ok = 1;
			}
		}
	}
	/*
	 * Check if BUS-DMA support is enabled and try to load virtual
	 * buffers into DMA, if any:
	 */

	if (xfer->flags_int.bdma_enable) {

		/*
	         * If the transfer is not inserted, insert
	         * the transfer into the DMA queue
	         */
		if (xfer->dma_list.le_prev == NULL) {
			LIST_INSERT_HEAD(&(xfer->usb_root->dma_head),
			    xfer, dma_list);
		}
		info = xfer->usb_root;

		/*
	         * Only call the BUS-DMA work loop when it is not busy:
	         */
		if (info->dma_refcount == 0) {
			usbd_bdma_work_loop(info);
		}
		return;
	}
	/*
	 * Enter the USB transfer into the Host Controller or
	 * Device Controller schedule:
	 */
	usbd_pipe_enter_wrapper(xfer);

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pipe_enter_wrapper - factored out code
 *------------------------------------------------------------------------*/
static void
usbd_pipe_enter_wrapper(struct usbd_xfer *xfer)
{
	mtx_assert(xfer->priv_mtx, MA_OWNED);

	mtx_lock(xfer->usb_mtx);

	/*
	 * Setup "usb_thread"
	 */
	xfer->usb_thread = (xfer->flags.use_polling) ?
	    curthread : NULL;

	/* enter the transfer */
	(xfer->pipe->methods->enter) (xfer);

	mtx_unlock(xfer->usb_mtx);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bdma_get_next_xfer
 *
 * This function will advance the "dma_curr_xfer" pointer to the next
 * USB transfer in the queue.
 *------------------------------------------------------------------------*/
static void
usbd_bdma_get_next_xfer(struct usbd_memory_info *info)
{
	struct usbd_xfer *xfer;

	xfer = info->dma_curr_xfer;

	/* prepare next USB transfer to load, if any */
	info->dma_curr_xfer =
	    LIST_PREV(&(info->dma_head), xfer, dma_list);
	LIST_REMOVE(xfer, dma_list);
	xfer->dma_list.le_prev = NULL;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bdma_work_loop
 *
 * This function handles loading of virtual buffers into DMA and is
 * only called when "dma_refcount" is zero.
 *------------------------------------------------------------------------*/
static void
usbd_bdma_work_loop(struct usbd_memory_info *info)
{
	struct usbd_xfer *xfer;
	struct usbd_page *pg;
	uint32_t nframes;
	uint32_t frlength_0;
	uint8_t isread;

	mtx_assert(info->priv_mtx, MA_OWNED);

load_complete:

	xfer = info->dma_curr_xfer;
	if (xfer) {

		/* prevent recursion by increasing the DMA refcount */

		info->dma_refcount = 2;

		/* check for errors */

		if (!xfer->flags_int.open) {

			/* get next xfer */
			usbd_bdma_get_next_xfer(info);

			/* we got cancelled */
			usbd_premature_callback(xfer,
			    USBD_ERR_CANCELLED);

		} else if (info->dma_error) {

			/* get next xfer */
			usbd_bdma_get_next_xfer(info);

			/* report error */
			usbd_premature_callback(xfer,
			    USBD_ERR_DMA_LOAD_FAILED);

		} else if (info->dma_currframe != info->dma_nframes) {

			if (info->dma_currframe == 0) {
				/* special case */
				usbd_pc_load_mem(xfer->frbuffers,
				    info->dma_frlength_0);
			} else {
				/* default case */
				nframes = info->dma_currframe;
				usbd_pc_load_mem(xfer->frbuffers + nframes,
				    xfer->frlengths[nframes]);
			}

			/* advance frame index */
			info->dma_currframe++;

			/* check if callback has decremented refcount */
			if (--(info->dma_refcount) == 0) {
				/* we are complete */
				goto load_complete;
			} else {
				/* wait for callback */
				return;
			}
		} else {

			/* get next xfer */
			usbd_bdma_get_next_xfer(info);

			/* go ahead */
			usbd_bdma_pre_sync(xfer);

			/* finally start the hardware */
			usbd_pipe_enter_wrapper(xfer);
		}

		info->dma_refcount = 0;

	} else {
		/* get first USB transfer */
		info->dma_curr_xfer =
		    LIST_FIRST(&(info->dma_head));
	}

	xfer = info->dma_curr_xfer;
	if (xfer == NULL) {
		/* nothing more to do */
		return;
	}
	if (!xfer->flags_int.open) {
		/* we got cancelled */
		goto load_complete;
	}
	/* reset BUS-DMA load state */

	info->dma_error = 0;

	if (xfer->flags_int.isochronous_xfr) {
		/* only one frame buffer */
		nframes = 1;
		frlength_0 = xfer->sumlen;
	} else {
		/* can be multiple frame buffers */
		nframes = xfer->nframes;
		frlength_0 = xfer->frlengths[0];
	}

	/*
	 * Set DMA direction first. This is needed to
	 * select the correct cache invalidate and cache
	 * flush operations.
	 */
	isread = USBD_GET_DATA_ISREAD(xfer);
	pg = xfer->dma_page_ptr;

	if (xfer->flags_int.control_xfr &&
	    xfer->flags_int.control_hdr) {
		/* special case */
		if (xfer->flags_int.usb_mode == USB_MODE_DEVICE) {
			/* The device controller writes to memory */
			xfer->frbuffers[0].isread = 1;
		} else {
			/* The host controller reads from memory */
			xfer->frbuffers[0].isread = 0;
		}
	} else {
		/* default case */
		xfer->frbuffers[0].isread = isread;
	}

	/*
	 * Setup the "page_start" pointer which points to an array of
	 * USB pages where information about the physical address of a
	 * page will be stored. Also initialise the "isread" field of
	 * the USB page caches.
	 */
	xfer->frbuffers[0].page_start = pg;

	info->dma_nframes = nframes;
	info->dma_currframe = 0;
	info->dma_frlength_0 = frlength_0;

	pg += (frlength_0 / USB_PAGE_SIZE);
	pg += 2;

	while (--nframes > 0) {
		xfer->frbuffers[nframes].isread = isread;
		xfer->frbuffers[nframes].page_start = pg;

		pg += (xfer->frlengths[nframes] / USB_PAGE_SIZE);
		pg += 2;
	}

	goto load_complete;
}

/*------------------------------------------------------------------------*
 *	usbd_bdma_done_event
 *
 * This function is called when the BUS-DMA has loaded virtual memory
 * into DMA, if any.
 *------------------------------------------------------------------------*/
static void
usbd_bdma_done_event(struct usbd_dma_parent_tag *udpt)
{
	struct usbd_memory_info *info;

	info = udpt->info;

	mtx_assert(info->priv_mtx, MA_OWNED);

	/* copy error */
	info->dma_error = udpt->dma_error;

	/* check refcount */
	if (--(info->dma_refcount) == 0) {
		/* call work loop */
		usbd_bdma_work_loop(info);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bdma_pre_sync
 *
 * This function handles DMA synchronisation that must be done before
 * an USB transfer is started.
 *------------------------------------------------------------------------*/
void
usbd_bdma_pre_sync(struct usbd_xfer *xfer)
{
	struct usbd_page_cache *pc;
	uint32_t nframes;

	if (xfer->flags_int.isochronous_xfr) {
		/* only one frame buffer */
		nframes = 1;
	} else {
		/* can be multiple frame buffers */
		nframes = xfer->nframes;
	}

	pc = xfer->frbuffers;

	while (nframes--) {

		if (pc->page_offset_buf != pc->page_offset_end) {
			if (pc->isread) {
				usbd_pc_cpu_invalidate(pc);
			} else {
				usbd_pc_cpu_flush(pc);
			}
		}
		pc++;
	}

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bdma_post_sync
 *
 * This function handles DMA synchronisation that must be done after
 * an USB transfer is complete.
 *------------------------------------------------------------------------*/
void
usbd_bdma_post_sync(struct usbd_xfer *xfer)
{
	struct usbd_page_cache *pc;
	uint32_t nframes;

	if (xfer->flags_int.isochronous_xfr) {
		/* only one frame buffer */
		nframes = 1;
	} else {
		/* can be multiple frame buffers */
		nframes = xfer->nframes;
	}

	pc = xfer->frbuffers;

	while (nframes--) {

		if (pc->page_offset_buf != pc->page_offset_end) {
			if (pc->isread) {
				usbd_pc_cpu_invalidate(pc);
			}
		}
		pc++;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_start - start an USB transfer
 *
 * NOTE: Calling this function more than one time will only
 *       result in a single transfer start, until the USB transfer
 *       completes.
 * NOTE: If "use_polling" is set in "xfer->flags", then this
 *       function will spin until transfer is completed
 *------------------------------------------------------------------------*/
void
usbd_transfer_start(struct usbd_xfer *xfer)
{

	if (xfer == NULL) {
		/* transfer is gone */
		return;
	}
	mtx_assert(xfer->priv_mtx, MA_OWNED);

	/* mark the USB transfer started */

	if (!xfer->flags_int.started) {
		xfer->flags_int.started = 1;
	}
	/* check if the USB transfer callback is already transferring */

	if (xfer->flags_int.transferring) {
		return;
	}
	/* call callback */

	usbd_callback_wrapper(xfer, NULL, USBD_CONTEXT_START);

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_stop - stop an USB transfer
 *
 * NOTE: Calling this function more than one time will only
 *       result in a single transfer stop.
 * NOTE: When this function returns it is not safe to free nor
 *       reuse any DMA buffers. See "usbd_transfer_drain()".
 *------------------------------------------------------------------------*/
void
usbd_transfer_stop(struct usbd_xfer *xfer)
{
	if (xfer == NULL) {
		/* transfer is gone */
		return;
	}
	mtx_assert(xfer->priv_mtx, MA_OWNED);

	/* check if the USB transfer was ever opened */

	if (!xfer->flags_int.open) {
		/* nothing to do except clearing the "started" flag */
		xfer->flags_int.started = 0;
		return;
	}
	/*
	 * close transfer
	 */
	mtx_lock(xfer->usb_mtx);
	(xfer->pipe->methods->close) (xfer);
	mtx_unlock(xfer->usb_mtx);

	/*
	 * Clear "open" and "started" after closing the USB transfer
	 * so that we don't get a race updating "flags_int" !
	 */
	xfer->flags_int.open = 0;
	xfer->flags_int.started = 0;

#ifdef USB_DEBUG
	/* check error value */
	if (xfer->error != USBD_ERR_CANCELLED) {
		PRINTFN(-1, ("wrong error code: %s\n",
		    usbd_errstr(xfer->error)));
	}
#endif

	/*
	 * Check if we are doing a transfer and if so
	 * do a Cancel Callback
	 */
	if (xfer->flags_int.transferring) {
		/*
		 * call callback, which will clear
		 * "flags_int.transferring"
		 */
		usbd_callback_wrapper(xfer, NULL, USBD_CONTEXT_STOP);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_drain
 *
 * This function will stop the USB transfer and wait for any
 * additional BUS-DMA and HW-DMA operations to complete. Buffers that
 * are loaded into DMA can safely be freed or reused after that this
 * function has returned.
 *------------------------------------------------------------------------*/
void
usbd_transfer_drain(struct usbd_xfer *xfer)
{
	WITNESS_WARN(WARN_GIANTOK | WARN_SLEEPOK, NULL,
	    "usbd_transfer_drain can sleep!");

	if (xfer == NULL) {
		/* transfer is gone */
		return;
	}
	if (xfer->priv_mtx != &Giant) {
		mtx_assert(xfer->priv_mtx, MA_NOTOWNED);
	}
	mtx_lock(xfer->priv_mtx);

	usbd_transfer_stop(xfer);

	while (xfer->flags_int.transferring) {
		xfer->flags_int.draining = 1;
		/*
		 * Wait until the current outstanding USB
		 * transfer is complete !
		 */
		if (mtx_sleep(&(xfer->flags_int), xfer->priv_mtx,
		    0, "usbdrain", 0)) {
			/* should not happen */
		}
	}
	mtx_unlock(xfer->priv_mtx);

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_set_frame_data
 *
 * This function sets the pointer of the buffer that should
 * loaded directly into DMA for the given USB frame. Passing "ptr"
 * equal to NULL while the corresponding "frlength" is greater
 * than zero gives undefined results!
 *------------------------------------------------------------------------*/
void
usbd_set_frame_data(struct usbd_xfer *xfer, void *ptr, uint32_t frindex)
{
	/* set virtual address to load and length */
	xfer->frbuffers[frindex].buffer = ptr;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_set_frame_offset
 *
 * This function sets the frame data buffer offset relative to the beginning
 * of the USB DMA buffer allocated for this USB transfer.
 *------------------------------------------------------------------------*/
void
usbd_set_frame_offset(struct usbd_xfer *xfer, uint32_t offset,
    uint32_t frindex)
{
	__KASSERT(!xfer->flags.ext_buffer, ("Cannot offset data frame "
	    "when the USB buffer is external!\n"));

	/* set virtual address to load */
	xfer->frbuffers[frindex].buffer =
	    USBD_ADD_BYTES(xfer->local_buffer, offset);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_callback_intr_sched
 *
 * This function gets the USB callback thread running if it is
 * sleeping.
 *------------------------------------------------------------------------*/
static void
usbd_callback_intr_sched(struct usbd_memory_info *info)
{
	mtx_assert(info->usb_mtx, MA_OWNED);

	if (info->done_sleep) {
		info->done_sleep = 0;
		wakeup(info);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_callback_intr_td_sub - factored out code
 *
 * This function performs USB callbacks.
 *------------------------------------------------------------------------*/
static void
usbd_callback_intr_td_sub(struct usbd_xfer **xfer, uint8_t dropcount)
{
	struct usbd_memory_info *info = xfer[0]->usb_root;

	mtx_unlock(info->usb_mtx);

	/*
	 * We exploit the fact that the mutex is the same for
	 * all callbacks:
	 */
	mtx_lock(info->priv_mtx);

	/* call callback(s) */
	switch (dropcount) {
	case 4:
		usbd_callback_wrapper(xfer[3], info, USBD_CONTEXT_CALLBACK);
	case 3:
		usbd_callback_wrapper(xfer[2], info, USBD_CONTEXT_CALLBACK);
	case 2:
		usbd_callback_wrapper(xfer[1], info, USBD_CONTEXT_CALLBACK);
	case 1:
		usbd_callback_wrapper(xfer[0], info, USBD_CONTEXT_CALLBACK);
	default:
		break;
	}
	mtx_unlock(info->priv_mtx);
	mtx_lock(info->usb_mtx);
	info->memory_refcount -= dropcount;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_callback_intr_td
 *
 * This is the USB callback interrupt thread. Every time a callback
 * cannot be called directly we call the callback from the callback
 * interrupt thread instead.
 *------------------------------------------------------------------------*/
static void
usbd_callback_intr_td(void *arg)
{
	struct usbd_memory_info *info = arg;
	struct usbd_xfer *xfer[4];
	struct thread *td;

	/* adjust priority */
	td = curthread;
	thread_lock(td);
	sched_prio(td, PI_NET);
	thread_unlock(td);

	mtx_lock(info->usb_mtx);

	/*
	 * The order of the callbacks should not be important.
	 * Optimize by doing 4 callbacks at a time.
	 */
repeat:
	xfer[0] = LIST_FIRST(&(info->done_head));
	if (xfer[0]) {
		LIST_REMOVE(xfer[0], done_list);
		xfer[0]->done_list.le_prev = NULL;
		xfer[1] = LIST_FIRST(&(info->done_head));
		if (xfer[1] == NULL) {
			usbd_callback_intr_td_sub(xfer, 1);
			goto repeat;
		}
		LIST_REMOVE(xfer[1], done_list);
		xfer[1]->done_list.le_prev = NULL;
		xfer[2] = LIST_FIRST(&(info->done_head));
		if (xfer[2] == NULL) {
			usbd_callback_intr_td_sub(xfer, 2);
			goto repeat;
		}
		LIST_REMOVE(xfer[2], done_list);
		xfer[2]->done_list.le_prev = NULL;
		xfer[3] = LIST_FIRST(&(info->done_head));
		if (xfer[3] == NULL) {
			usbd_callback_intr_td_sub(xfer, 3);
			goto repeat;
		}
		LIST_REMOVE(xfer[3], done_list);
		xfer[3]->done_list.le_prev = NULL;
		usbd_callback_intr_td_sub(xfer, 4);
		goto repeat;
	} else {
		if (info->memory_refcount != 0) {
			info->done_sleep = 1;
			if (mtx_sleep(info, info->usb_mtx, 0, "usbdone", 0)) {
				/* should not happen */
			}
			goto repeat;
		}
	}
	wakeup(&(info->done_thread));
	info->done_thread = NULL;
	mtx_unlock(info->usb_mtx);
	usb_thread_exit(0);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dma_delay_done_cb
 *
 * This function is called when the DMA delay has been exectuded, and
 * will make sure that the callback is called to complete the USB
 * transfer. This code path is ususally only used when there is an USB
 * error like USBD_ERR_CANCELLED.
 *------------------------------------------------------------------------*/
static void
usbd_dma_delay_done_cb(struct usbd_xfer *xfer)
{
	mtx_assert(xfer->usb_mtx, MA_OWNED);

	PRINTFN(2, ("Completed %p\n", xfer));

	/* queue callback for execution */
	usbd_callback_wrapper(xfer, NULL, USBD_CONTEXT_CALLBACK);

	mtx_unlock(xfer->usb_mtx);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_callback_wrapper
 *
 * This is a wrapper for USB callbacks. This wrapper does some
 * auto-magic things like figuring out if we can call the callback
 * directly from the current context or if we need to wakeup the
 * interrupt thread. It also handles recursation of callbacks.
 *
 * The "ctd" input parameter is only used in case of "context" equal
 * to "USBD_CONTEXT_CALLBACK", "priv_mtx" locked and "usb_mtx" not
 * locked.
 *------------------------------------------------------------------------*/
void
usbd_callback_wrapper(struct usbd_xfer *xfer, void *ctd, uint8_t context)
{
	struct usbd_bus *bus;
	uint32_t timeout;
	uint32_t temp;
	uint8_t dropped_usb_mtx = 0;

	if (mtx_owned(xfer->priv_mtx)) {
		/*
		 * Cases that end up here:
		 *
		 * 1) We are starting a transfer
		 * 2) We are prematurely calling back a transfer or polling
		 * 3) We are stopping a transfer
		 * 4) We are doing an ordinary callback
		 */
		if (mtx_owned(xfer->usb_mtx)) {
			/* case 2 */
			/* check first recurse flag */
			if (!xfer->flags_int.recursed_1) {

				PRINTFN(2, ("case 2 not recursed\n"));

				mtx_unlock(xfer->usb_mtx);
				mtx_assert(xfer->usb_mtx, MA_NOTOWNED);

				/*
			         * The locking order is:
			         * 1. priv_mtx
			         * 2. usb_mtx
			         *
			         * This leads us to dropping "usb_mtx"
			         * else we end up having a Locking Order
			         * Reversal, LOR
			         */
				dropped_usb_mtx = 1;
			} else {
				PRINTFN(2, ("case 2 recursed\n"));
			}
		} else {
			if (context == USBD_CONTEXT_CALLBACK) {
				/* case 4 */

				PRINTFN(2, ("case 4\n"));

				/*
				 * During possible unlocked periods,
				 * the USB transfer can have been
				 * restarted by another thread. Check
				 * that it is still this thread that
				 * is calling back the callback by
				 * verifying "xfer->usb_thread".
				 */
				mtx_lock(xfer->usb_mtx);
				temp = (xfer->usb_thread == ctd);
				mtx_unlock(xfer->usb_mtx);

				if (!temp) {
					/* already called back */
					return;
				}
			} else {
				/* case 1 or case 3 */

				PRINTFN(2, ("case 1\n"));

				/*
				 * By clearing the "usb_thread"
				 * variable we are signalling that a
				 * different thread has handled the
				 * callback. This prevents other
				 * threads that are about to call the
				 * callback, to actually call the
				 * callback.
				 */
				mtx_lock(xfer->usb_mtx);
				xfer->usb_thread = NULL;
				mtx_unlock(xfer->usb_mtx);

			}
		}
	} else if (mtx_owned(xfer->usb_mtx)) {
		/*
		 * Cases that end up here:
		 *
		 * 5) HW interrupt done callback or other source.
		 */

		PRINTFN(2, ("case 5\n"));

		/*
	         * We have to postpone the callback due to the fact we
	         * will have a Lock Order Reversal, LOR, if we try to
	         * proceed !
	         */

		/* Set callback thread sort of */
		xfer->usb_thread = (void *)(xfer->usb_root);

		/* Check if the callback is not on the done list */
		if (xfer->done_list.le_prev == NULL) {
			struct usbd_memory_info *info;

			info = xfer->usb_root;
			/*
			 * we need to increment the memory count in
			 * case of a "usbd_transfer_unsetup" call
			 */
			info->memory_refcount++;

			LIST_INSERT_HEAD(&(info->done_head), xfer, done_list);
			usbd_callback_intr_sched(info);
		}
		return;
	} else {
		panic("%s: called unlocked!\n", __FUNCTION__);
	}

	/* check first recurse flag */
	if (!xfer->flags_int.recursed_1) {
		do {
			/*
			 * If we have a non-hardware induced error we
			 * need to do the DMA delay!
			 */
			if (xfer->flags_int.transferring &&
			    xfer->flags_int.bdma_enable &&
			    ((xfer->error == USBD_ERR_CANCELLED) ||
			    (xfer->error == USBD_ERR_TIMEOUT)) &&
			    (!xfer->flags_int.did_dma_delay)) {

				/* only do this one time */
				xfer->flags_int.did_dma_delay = 1;

				temp = usbd_get_dma_delay(xfer->udev->bus);

				PRINTFN(2, ("DMA delay, %u ms, "
				    "on %p\n", temp, xfer));

				if (xfer->flags.use_polling) {
					DELAY(temp * 1024);
				} else {
					mtx_lock(xfer->usb_mtx);
					usb_callout_reset(&(xfer->timeout_handle),
					    USBD_MS_TO_TICKS(temp) + 1,
					    (void *)&usbd_dma_delay_done_cb, xfer);
					mtx_unlock(xfer->usb_mtx);
					break;	/* wait for callback */
				}
			}
			/* set both recurse flags */
			xfer->flags_int.recursed_1 = 1;
			xfer->flags_int.recursed_2 = 1;

			/* set which context we are in */
			xfer->flags_int.context = context;

			/* set correct USB state for callback */
			if (!xfer->flags_int.transferring) {
				xfer->usb_state = USBD_ST_SETUP;
				goto callback;
			}
			xfer->flags_int.transferring = 0;

			if (xfer->error) {
				/*
				 * Check if we got started after that
				 * we got cancelled, but before we
				 * managed to deliver the
				 * USBD_ERR_CANCELLED message!
				 */
				if ((xfer->error == USBD_ERR_CANCELLED) &&
				    (xfer->flags_int.started)) {
					/* restart by doing a second loop */
					xfer->flags_int.recursed_2 = 0;
				}
				xfer->usb_state = USBD_ST_ERROR;
				goto callback;
			}
			/* set transferred state */
			xfer->usb_state = USBD_ST_TRANSFERRED;

			/* sync DMA memory, if any */
			if (xfer->flags_int.bdma_enable &&
			    (!xfer->flags_int.bdma_no_post_sync)) {
				usbd_bdma_post_sync(xfer);
			}
	callback:
			/* call processing routine */
			(xfer->callback) (xfer);

			/* check for polling mode */
			if (!xfer->flags.use_polling) {
				continue;
			}
			/* check if there is a transfer pending */
			if (!xfer->flags_int.transferring) {
				continue;
			}
			/* setup timeout and bus */
			timeout = xfer->timeout + 1;
			bus = xfer->udev->bus;

			/* get real current thread - we are polling */
			ctd = curthread;

			while (xfer->flags_int.recursed_2) {

				mtx_lock(xfer->usb_mtx);
				temp = (xfer->usb_thread == ctd);
				if (!temp) {
					/*
					 * Case 6 has happened on this
					 * transfer. Another thread, but the
					 * polling thread signalled that the
					 * USB transfer is complete. We need
					 * to take back the right to do the
					 * callback:
					 */
					xfer->usb_thread = ctd;
				}
				mtx_unlock(xfer->usb_mtx);

				/* check if complete */
				if (!temp) {
					/* fake a callback */
					xfer->flags_int.recursed_2 = 0;
					break;
				}
				/* check timeout */
				if (!timeout--) {
					/* stop the transfer */
					usbd_transfer_stop(xfer);
					break;
				}
				/* delay one millisecond */
				DELAY(1000);

				/*
				 * Do polling which will call
				 * "usbd_callback_wrapper()" when the
				 * transfer is complete !
				 */
				(bus->methods->do_poll) (bus);
			}

			/* check second recurse flag */
		} while (!xfer->flags_int.recursed_2);

		if (xfer->flags_int.draining &&
		    (!xfer->flags_int.transferring)) {
			/*
			 * "usbd_transfer_drain()" is waiting for us
			 * to stop transferring.
			 */
			xfer->flags_int.draining = 0;
			wakeup(&(xfer->flags_int));
		}
		/* clear first recurse flag */
		xfer->flags_int.recursed_1 = 0;
	} else {
		/* clear second recurse flag */
		xfer->flags_int.recursed_2 = 0;
	}

	if (dropped_usb_mtx) {
		/* pickup the mutex again */
		mtx_lock(xfer->usb_mtx);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_delayed_transfer_start
 *
 * This function is called to start the USB transfer when
 * "xfer->interval" is greater than zero, and and the endpoint type is
 * BULK or CONTROL.
 *------------------------------------------------------------------------*/
static void
usbd_delayed_transfer_start(void *arg)
{
	struct usbd_xfer *xfer = arg;

	mtx_assert(xfer->usb_mtx, MA_OWNED);

	/* start the transfer */
	(xfer->pipe->methods->start) (xfer);

	mtx_unlock(xfer->usb_mtx);

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_set_stall
 *
 * This function is used to set the stall flag outside the
 * callback. This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_transfer_set_stall(struct usbd_xfer *xfer)
{
	if (xfer == NULL) {
		/* tearing down */
		return;
	}
	mtx_assert(xfer->priv_mtx, MA_OWNED);

	/* avoid any races by locking the USB mutex */
	mtx_lock(xfer->usb_mtx);

	xfer->flags.stall_pipe = 1;

	mtx_unlock(xfer->usb_mtx);

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_clear_stall
 *
 * This function is used to clear the stall flag outside the
 * callback. This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_transfer_clear_stall(struct usbd_xfer *xfer)
{
	if (xfer == NULL) {
		/* tearing down */
		return;
	}
	mtx_assert(xfer->priv_mtx, MA_OWNED);

	/* avoid any races by locking the USB mutex */
	mtx_lock(xfer->usb_mtx);

	xfer->flags.stall_pipe = 0;

	mtx_unlock(xfer->usb_mtx);

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_intr_enqueue
 *
 * This function is used to add an USB transfer to the interrupt
 * transfer list.
 *------------------------------------------------------------------------*/
void
usbd_transfer_intr_enqueue(struct usbd_xfer *xfer)
{
	if (xfer->interrupt_list.le_prev == NULL) {
		LIST_INSERT_HEAD(&(xfer->udev->bus->intr_list_head),
		    xfer, interrupt_list);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_enqueue
 *
 * This function is used to add an USB transfer to the pipe transfer list.
 *------------------------------------------------------------------------*/
void
usbd_transfer_enqueue(struct usbd_xfer *xfer)
{
	uint8_t type;

	mtx_assert(xfer->usb_mtx, MA_OWNED);

	/*
	 * If the transfer is not inserted, insert the transfer into the
	 * transfer queue:
	 */
	if (xfer->pipe_list.le_prev == NULL) {
		LIST_INSERT_HEAD(&xfer->pipe->list_head, xfer, pipe_list);
	}
	/*
	 * If the pipe is already stalled we do nothing except putting
	 * the transfer on the queue !
	 */
	if (xfer->pipe->is_stalled) {
		goto done;
	}
	/*
	 * Check if we are supposed to stall the pipe:
	 */
	if (xfer->flags.stall_pipe) {
		/* clear stall command */
		xfer->flags.stall_pipe = 0;

		if (xfer->flags_int.usb_mode == USB_MODE_DEVICE) {
			/*
			 * Only stall BULK and INTERRUPT endpoints.
			 */
			type = (xfer->pipe->edesc->bmAttributes & UE_XFERTYPE);
			if ((type == UE_BULK) ||
			    (type == UE_INTERRUPT)) {
				xfer->pipe->is_stalled = 1;
				(xfer->udev->bus->methods->set_stall) (
				    xfer->udev, NULL, xfer->pipe);
				goto done;
			}
		}
	}
	/*
	 * Handled cases:
	 *
	 * 1) Start the first transfer queued. This transfer is always last on
	 * the pipe transfer list! "le_next" is NULL. "le_prev" is also
	 * NULL.
	 *
	 * 2) Start the last transfer if it is already queued. We are most
	 * likely resuming a control transfer. "le_next" is NULL. "le_prev"
	 * is not NULL.
	 */
	if (xfer->pipe_list.le_next == NULL) {
		/*
		 * Check if there should be any
		 * pre transfer start delay:
		 */
		if (xfer->interval > 0) {
			type = (xfer->pipe->edesc->bmAttributes & UE_XFERTYPE);
			if ((type == UE_BULK) ||
			    (type == UE_CONTROL)) {
				if (xfer->flags.use_polling) {
					DELAY(((uint32_t)(xfer->interval)) * 1000);
				} else {
					usb_callout_reset(&(xfer->timeout_handle),
					    USBD_MS_TO_TICKS(xfer->interval),
					    &usbd_delayed_transfer_start,
					    xfer);
					goto done;
				}
			}
		}
		/* start USB transfer */
		(xfer->pipe->methods->start) (xfer);
	}
done:
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_dequeue
 *
 * This function:
 *  - is used to remove an USB transfer from the pipe transfer list.
 *  - is also used to start the next USB transfer on the pipe
 *    transfer list, if any.
 *  - can be called multiple times in a row.
 *
 * NOTE: In some special cases the USB transfer will not be removed from
 * the pipe queue, but remain first. To enforce USB transfer removal call
 * this function passing the error code "USBD_ERR_CANCELLED".
 *------------------------------------------------------------------------*/
void
usbd_transfer_dequeue(struct usbd_xfer *xfer, usbd_status_t error)
{
	struct usbd_xfer *xfer_prev;
	uint32_t x;

	mtx_assert(xfer->usb_mtx, MA_OWNED);

	/* count completed transfers */
	++(xfer->udev->bus->stats.uds_requests
	    [xfer->pipe->edesc->bmAttributes & UE_XFERTYPE]);

	/* check actual number of frames */
	if (xfer->aframes > xfer->nframes) {
		if (error == 0) {
			panic("%s: actual number of frames, %d, is "
			    "greater than initial number of frames, %d!\n",
			    __FUNCTION__, xfer->aframes, xfer->nframes);
		} else {
			/* just set some valid value */
			xfer->aframes = xfer->nframes;
		}
	}
	/* compute actual length */
	xfer->actlen = 0;

	for (x = 0; x != xfer->aframes; x++) {
		xfer->actlen += xfer->frlengths[x];
	}

	/*
	 * Frames that were not transferred get zero actual length in
	 * case the USB device driver does not check the actual number
	 * of frames transferred, "xfer->aframes":
	 */
	for (; x < xfer->nframes; x++) {
		xfer->frlengths[x] = 0;
	}

	/* check actual length */
	if (xfer->actlen > xfer->sumlen) {
		if (error == 0) {
			panic("%s: actual length, %d, is greater than "
			    "initial length, %d!\n",
			    __FUNCTION__, xfer->actlen, xfer->sumlen);
		} else {
			/* just set some valid value */
			xfer->actlen = xfer->sumlen;
		}
	}
	PRINTFN(5, ("xfer=%p pipe=%p sts=%d alen=%d, slen=%d, afrm=%d, nfrm=%d\n",
	    xfer, xfer->pipe, error, xfer->actlen, xfer->sumlen,
	    xfer->aframes, xfer->nframes));

	if (error) {
		/* end of control transfer, if any */
		xfer->flags_int.control_act = 0;
	} else {
		/* check for short transfers */
		if (xfer->actlen < xfer->sumlen) {

			/* end of control transfer, if any */
			xfer->flags_int.control_act = 0;

			if (!xfer->flags_int.short_xfer_ok) {
				error = USBD_ERR_SHORT_XFER;
			}
		}
	}

	xfer->error = error;

	/*
	 * Check if we are in the middle of a
	 * control transfer:
	 */
	if (xfer->flags_int.control_act) {
		PRINTFN(4, ("xfer=%p: Control transfer "
		    "active on pipe=%p\n", xfer, xfer->pipe));
		goto done;
	}
	/*
	 * Check if we should block the
	 * execution queue:
	 */
	if ((xfer->error) &&
	    (xfer->error != USBD_ERR_CANCELLED) &&
	    (xfer->flags.pipe_bof)) {
		PRINTFN(4, ("xfer=%p: Block On Failure "
		    "on pipe=%p\n", xfer, xfer->pipe));
		goto done;
	}
	/* check if we are queued on any pipe transfer list */

	if (xfer->pipe_list.le_prev) {

		/* get the previous transfer, if any */
		xfer_prev = LIST_PREV(&(xfer->pipe->list_head), xfer, pipe_list);

		/* remove the transfer from pipe transfer list */
		LIST_REMOVE(xfer, pipe_list);
		xfer->pipe_list.le_prev = 0;

		/* start "next" transfer, if any */
		if (xfer_prev) {
			usbd_transfer_enqueue(xfer_prev);
		}
	}
done:
	/* check if we are queued on any interrupt transfer list */
	if (xfer->interrupt_list.le_prev) {
		LIST_REMOVE(xfer, interrupt_list);
		xfer->interrupt_list.le_prev = NULL;
	}
	/* stop any callouts */
	usb_callout_stop(&(xfer->timeout_handle));
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_do_request_callback
 *
 * This function is the USB callback for generic USB Host control
 * transfers.
 *------------------------------------------------------------------------*/
static void
usbd_do_request_callback(struct usbd_xfer *xfer)
{
	;				/* workaround for a bug in "indent" */

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
		usbd_start_hardware(xfer);
		break;
	default:
		if (!xfer->flags.use_polling) {
			wakeup(xfer);
		}
		break;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_handle_request_callback
 *
 * This function is the USB callback for generic USB Device control
 * transfers.
 *------------------------------------------------------------------------*/
static void
usbd_handle_request_callback(struct usbd_xfer *xfer)
{
	usbd_status_t err;

	/* check the current transfer state */

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
	case USBD_ST_TRANSFERRED:

		/* handle the request */
		err = usbd_handle_request(xfer);

		if (err) {
			if (err == USBD_ERR_BAD_CONTEXT) {
				/*
				 * Currently we get a "start" context by
				 * waking up the explore thread.
				 */
				usb_needs_explore(xfer->udev->bus,
				    USB_BUS_EXPLORE_TREE);
				return;
			}
			/*
		         * If no control transfer is active,
		         * receive the next SETUP message:
		         */
			goto tr_restart;
		}
		usbd_start_hardware(xfer);
		return;

	default:
		if (xfer->error != USBD_ERR_CANCELLED) {
			/* should not happen - try stalling */
			goto tr_restart;
		}
		break;
	}
	return;

tr_restart:
	xfer->frlengths[0] = sizeof(usb_device_request_t);
	xfer->nframes = 1;
	xfer->flags.manual_status = 1;
	xfer->flags.force_short_xfer = 0;
	xfer->flags.stall_pipe = 1;	/* cancel previous transfer, if any */
	usbd_start_hardware(xfer);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_handle_set_config
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static usbd_status_t
usbd_handle_set_config(struct usbd_xfer *xfer, uint8_t conf_no)
{
	mtx_unlock(xfer->priv_mtx);

	usbd_detach_device(xfer->udev, USB_IFACE_INDEX_ANY, 1);

	if (conf_no == USB_UNCONFIG_NO) {
		conf_no = USB_UNCONFIG_INDEX;
	} else {
		/*
		 * The relationship between config number and config index
		 * is very simple in our case:
		 */
		conf_no--;
	}

	if (usbd_set_config_index(xfer->udev, conf_no, 0)) {
		PRINTFN(0, ("set config %d failed\n", conf_no));
		mtx_lock(xfer->priv_mtx);
		return (USBD_ERR_STALLED);
	}
	if (usbd_probe_and_attach(xfer->udev, USB_IFACE_INDEX_ANY)) {
		PRINTFN(0, ("probe and attach failed\n"));
		mtx_lock(xfer->priv_mtx);
		return (USBD_ERR_STALLED);
	}
	mtx_lock(xfer->priv_mtx);
	return (0);
}

/*------------------------------------------------------------------------*
 *	usbd_get_curr_xfer - get current transfer on a pipe
 *
 * Returns:
 * NULL: No transfer
 * Else: Current USB transfer
 *------------------------------------------------------------------------*/
static struct usbd_xfer *
usbd_get_curr_xfer(struct usbd_pipe *pipe)
{
	struct usbd_xfer *xfer;

	xfer = LIST_FIRST(&(pipe->list_head));
	if (xfer) {

		/* search to the end of the LIST */
		while (LIST_NEXT(xfer, pipe_list)) {
			xfer = LIST_NEXT(xfer, pipe_list);
		}

		/*
		 * If "priv_mtx" is locked the "udev->bus->mtx" mutex
		 * can get dropped in "usbd_callback_wrapper()" !
		 */
		mtx_assert(xfer->priv_mtx, MA_NOTOWNED);
	}
	return (xfer);
}

/*------------------------------------------------------------------------*
 *	usbd_handle_set_stall_sub
 *
 * This function is used to make a BULK or INTERRUPT endpoint
 * send STALL tokens.
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static usbd_status_t
usbd_handle_set_stall_sub(struct usbd_device *udev, uint8_t ea_val,
    uint8_t do_stall)
{
	struct usbd_pipe *pipe;
	struct usbd_xfer *xfer;
	uint8_t et;
	uint8_t was_stalled;

	pipe = usbd_get_pipe_by_addr(udev, ea_val);
	if (pipe == NULL) {
		/* nothing to do */
		PRINTFN(0, ("Cannot find endpoint\n"));
		/*
		 * Pretend that the clear or set stall request is
		 * successful else some USB host stacks can do
		 * strange things, especially when a control endpoint
		 * stalls.
		 */
		return (0);
	}
	et = (pipe->edesc->bmAttributes & UE_XFERTYPE);

	if ((et != UE_BULK) &&
	    (et != UE_INTERRUPT)) {
		/*
	         * Should not stall control
	         * nor isochronous endpoints.
	         */
		PRINTFN(0, ("Invalid endpoint\n"));
		return (0);
	}
	mtx_lock(&(udev->bus->mtx));

	/* store current stall state */
	was_stalled = pipe->is_stalled;

	/* check for no change */
	if (was_stalled && do_stall) {
		/* if the pipe is already stalled do nothing */
		mtx_unlock(&(udev->bus->mtx));
		PRINTFN(0, ("No change\n"));
		return (0);
	}
	/* set stalled state */
	pipe->is_stalled = 1;

	if (do_stall || (!was_stalled)) {
		if (!was_stalled) {
			/* lookup the current USB transfer */
			xfer = usbd_get_curr_xfer(pipe);
		} else {
			xfer = NULL;
		}

		/*
		 * If "xfer" is non-NULL the "set_stall" method will
		 * complete the USB transfer like in case of a timeout
		 * setting the error code "USBD_ERR_STALLED".
		 */
		(udev->bus->methods->set_stall) (udev, xfer, pipe);
	}
	if (!do_stall) {
		pipe->toggle_next = 0;	/* reset data toggle */
		pipe->is_stalled = 0;	/* clear stalled state */

		(udev->bus->methods->clear_stall) (udev, pipe);

		/* lookup the current USB transfer */
		xfer = usbd_get_curr_xfer(pipe);

		/* start up the current transfer, if any */
		if (xfer) {
			usbd_transfer_enqueue(xfer);
		}
	}
	mtx_unlock(&(udev->bus->mtx));
	return (0);
}

/*------------------------------------------------------------------------*
 *	usbd_handle_stall
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static usbd_status_t
usbd_handle_set_stall(struct usbd_xfer *xfer, uint8_t ep, uint8_t do_stall)
{
	usbd_status_t err;

	mtx_unlock(xfer->priv_mtx);
	err = usbd_handle_set_stall_sub(xfer->udev, ep, do_stall);
	mtx_lock(xfer->priv_mtx);
	return (err);
}

/*------------------------------------------------------------------------*
 *	usbd_handle_get_stall
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static uint8_t
usbd_handle_get_stall(struct usbd_device *udev, uint8_t ea_val)
{
	struct usbd_pipe *pipe;
	uint8_t halted;

	pipe = usbd_get_pipe_by_addr(udev, ea_val);
	if (pipe == NULL) {
		/* nothing to do */
		return (0);
	}
	mtx_lock(&(udev->bus->mtx));

	halted = pipe->is_stalled;
	mtx_unlock(&(udev->bus->mtx));

	return (halted);
}

/*------------------------------------------------------------------------*
 *	usbd_handle_remote_wakeup
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static usbd_status_t
usbd_handle_remote_wakeup(struct usbd_xfer *xfer, uint8_t is_on)
{
	struct usbd_device *udev;
	struct usbd_bus *bus;

	udev = xfer->udev;
	bus = udev->bus;

	mtx_lock(&(bus->mtx));

	if (is_on) {
		udev->flags.remote_wakeup = 1;
	} else {
		udev->flags.remote_wakeup = 0;
	}

	(bus->methods->rem_wakeup_set) (xfer->udev, is_on);

	mtx_unlock(&(bus->mtx));

	return (0);			/* success */
}

/*------------------------------------------------------------------------*
 *	usbd_handle_request
 *
 * Returns:
 * 0: Ready to start hardware
 * USBD_ERR_BAD_CONTEXT: Need to switch context
 * Else: Stall current transfer, if any
 *------------------------------------------------------------------------*/
static usbd_status_t
usbd_handle_request(struct usbd_xfer *xfer)
{
/*
 * USB handle request states
 *
 * Typical state sequence:
 *
 * ST_DATA [ -> ST_CONTEXT_START ] -> ST_POST_STATUS
 */
	enum {
		ST_DATA,
		ST_CONTEXT_START,
		ST_POST_STATUS,
	};
	usb_device_request_t req;
	struct usbd_device *udev;
	struct usbd_interface *iface;
	const void *src_zcopy;		/* zero-copy source pointer */
	const void *src_mcopy;		/* non zero-copy source pointer */
	int error;
	uint16_t off;			/* data offset */
	uint16_t rem;			/* data remainder */
	uint16_t max_len;		/* max fragment length */
	uint16_t wValue;
	uint16_t wIndex;
	uint8_t state;
	uint8_t iface_index;
	union {
		uWord	wStatus;
		uint8_t	buf[2];
	}     temp;

	/*
	 * Filter the USB transfer state into
	 * something which we understand:
	 */

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_SETUP:
		state = ST_CONTEXT_START;

		if (!xfer->flags_int.control_act) {
			/* nothing to do */
			goto tr_stalled;
		}
		if (xfer->flags_int.context != USBD_CONTEXT_START) {
			/* wrong context - should not happen */
			goto tr_bad_context;
		}
		break;

	default:			/* USBD_ST_TRANSFERRED */
		if (!xfer->flags_int.control_act) {
			state = ST_POST_STATUS;
		} else {
			state = ST_DATA;
		}
		break;
	}

	/* reset frame stuff */

	xfer->frlengths[0] = 0;

	usbd_set_frame_offset(xfer, 0, 0);
	usbd_set_frame_offset(xfer, sizeof(req), 1);

	/* get the current request, if any */

	usbd_copy_out(xfer->frbuffers, 0, &req, sizeof(req));

	if (xfer->flags_int.control_rem == 0xFFFF) {
		/* first time - not initialised */
		rem = UGETW(req.wLength);
		off = 0;
	} else {
		/* not first time - initialised */
		rem = xfer->flags_int.control_rem;
		off = UGETW(req.wLength) - rem;
	}

	/* set some defaults */

	max_len = 0;
	src_zcopy = NULL;
	src_mcopy = NULL;
	udev = xfer->udev;

	/* get some request fields decoded */

	wValue = UGETW(req.wValue);
	wIndex = UGETW(req.wIndex);

	PRINTFN(0, ("req 0x%02x 0x%02x 0x%04x 0x%04x "
	    "off=0x%x rem=0x%x, state=%d\n", req.bmRequestType,
	    req.bRequest, wValue, wIndex, off, rem, state));

	/* demultiplex the control request */

	switch (req.bmRequestType) {
	case UT_READ_DEVICE:
		if (state != ST_DATA) {
			break;
		}
		switch (req.bRequest) {
		case UR_GET_DESCRIPTOR:
			goto tr_handle_get_descriptor;
		case UR_GET_CONFIG:
			goto tr_handle_get_config;
		case UR_GET_STATUS:
			goto tr_handle_get_status;
		default:
			goto tr_stalled;
		}
		break;

	case UT_WRITE_DEVICE:
		switch (req.bRequest) {
		case UR_SET_ADDRESS:
			goto tr_handle_set_address;
		case UR_SET_CONFIG:
			goto tr_handle_set_config;
		case UR_CLEAR_FEATURE:
			switch (wValue) {
			case UF_DEVICE_REMOTE_WAKEUP:
				goto tr_handle_clear_wakeup;
			default:
				goto tr_stalled;
			}
			break;
		case UR_SET_FEATURE:
			switch (wValue) {
			case UF_DEVICE_REMOTE_WAKEUP:
				goto tr_handle_set_wakeup;
			default:
				goto tr_stalled;
			}
			break;
		default:
			goto tr_stalled;
		}
		break;

	case UT_WRITE_ENDPOINT:
		switch (req.bRequest) {
		case UR_CLEAR_FEATURE:
			switch (wValue) {
			case UF_ENDPOINT_HALT:
				goto tr_handle_clear_halt;
			default:
				goto tr_stalled;
			}
			break;
		case UR_SET_FEATURE:
			switch (wValue) {
			case UF_ENDPOINT_HALT:
				goto tr_handle_set_halt;
			default:
				goto tr_stalled;
			}
			break;
		default:
			goto tr_stalled;
		}
		break;

	case UT_READ_ENDPOINT:
		switch (req.bRequest) {
		case UR_GET_STATUS:
			goto tr_handle_get_ep_status;
		default:
			goto tr_stalled;
		}
		break;
	default:
		if (state == ST_DATA) {
			goto tr_bad_context;
		}
		if ((req.bmRequestType & 0x1F) == UT_INTERFACE) {
			iface_index = req.wIndex[0];	/* unicast */
		} else {
			iface_index = 0;/* broadcast */
		}
		goto tr_handle_iface_request;
	}
	goto tr_valid;

tr_handle_get_descriptor:
	usbd_temp_get_desc(udev, &req, &src_zcopy, &max_len);
	if (src_zcopy == NULL) {
		goto tr_stalled;
	}
	goto tr_valid;

tr_handle_get_config:
	temp.buf[0] = udev->curr_config_no;
	src_mcopy = temp.buf;
	max_len = 1;
	goto tr_valid;

tr_handle_get_status:

	wValue = 0;

	mtx_lock(&(udev->bus->mtx));
	if (udev->flags.remote_wakeup) {
		wValue |= UDS_REMOTE_WAKEUP;
	}
	if (udev->flags.self_powered) {
		wValue |= UDS_SELF_POWERED;
	}
	mtx_unlock(&(udev->bus->mtx));

	USETW(temp.wStatus, wValue);
	src_mcopy = temp.wStatus;
	max_len = sizeof(temp.wStatus);
	goto tr_valid;

tr_handle_set_address:
	if (state == ST_DATA) {
		if (wValue >= 0x80) {
			/* invalid value */
			goto tr_stalled;
		} else if (udev->curr_config_no != 0) {
			/* we are configured ! */
			goto tr_stalled;
		}
	} else if (state == ST_POST_STATUS) {
		udev->address = (wValue & 0x7F);
		goto tr_bad_context;
	}
	goto tr_valid;

tr_handle_set_config:
	if (state == ST_DATA) {
		goto tr_bad_context;
	} else if (state == ST_CONTEXT_START) {
		if (usbd_handle_set_config(xfer, req.wValue[0])) {
			goto tr_stalled;
		}
	}
	goto tr_valid;

tr_handle_clear_halt:
	if (state == ST_DATA) {
		if (usbd_handle_set_stall(xfer, req.wIndex[0], 0)) {
			goto tr_stalled;
		}
	}
	goto tr_valid;

tr_handle_clear_wakeup:
	if (state == ST_DATA) {
		goto tr_bad_context;
	} else if (state == ST_CONTEXT_START) {
		if (usbd_handle_remote_wakeup(xfer, 0)) {
			goto tr_stalled;
		}
	}
	goto tr_valid;

tr_handle_set_halt:
	if (state == ST_DATA) {
		if (usbd_handle_set_stall(xfer, req.wIndex[0], 1)) {
			goto tr_stalled;
		}
	}
	goto tr_valid;

tr_handle_set_wakeup:
	if (state == ST_DATA) {
		goto tr_bad_context;
	} else if (state == ST_CONTEXT_START) {
		if (usbd_handle_remote_wakeup(xfer, 1)) {
			goto tr_stalled;
		}
	}
	goto tr_valid;

tr_handle_get_ep_status:
	if (state == ST_DATA) {
		temp.wStatus[0] =
		    usbd_handle_get_stall(udev, req.wIndex[0]);
		temp.wStatus[1] = 0;
		src_mcopy = temp.wStatus;
		max_len = sizeof(temp.wStatus);
	}
	goto tr_valid;

tr_handle_iface_request:
	iface = usbd_get_iface(udev, iface_index);
	if (iface == NULL) {
		goto tr_stalled;
	}
	if (iface->subdev == NULL) {
		goto tr_handle_iface_request_builtin;
	}
	if (device_is_attached(iface->subdev) == 0) {
		goto tr_handle_iface_request_builtin;
	}
	mtx_unlock(xfer->priv_mtx);

	/* We use "USBD_ADD_BYTES" to de-const the src_zcopy */

	error = USB_HANDLE_REQUEST(iface->subdev,
	    &req, USBD_ADD_BYTES(&src_zcopy, 0), &max_len,
	    off, (state == ST_POST_STATUS));

	mtx_lock(xfer->priv_mtx);

	if (error == 0) {
		/* negativly adjust pointer and length */
		src_zcopy = ((const uint8_t *)src_zcopy) - off;
		max_len += off;
		goto tr_valid;
	} else if (error == ENOTTY) {
		goto tr_stalled;
	}
	goto tr_handle_iface_request_builtin;

tr_handle_iface_request_builtin:
	if ((req.bmRequestType & 0x1F) != UT_INTERFACE) {
		iface_index++;		/* iterate */
		goto tr_handle_iface_request;
	}
	switch (req.bmRequestType) {
	case UT_WRITE_INTERFACE:
		switch (req.bRequest) {
		case UR_SET_INTERFACE:
			if (state == ST_POST_STATUS) {
				/* we are complete */
				break;
			}
			if (iface->alt_index == req.wValue[0]) {
				/* no change - nothing to do */
				PRINTFN(0, ("alt setting no change\n"));
				break;
			}
			mtx_unlock(xfer->priv_mtx);

			usbd_detach_device(udev, req.wIndex[0], 1);

			error = usbd_set_alt_interface_index(udev,
			    req.wIndex[0], req.wValue[0]);
			if (error) {
				PRINTFN(0, ("alt setting failed %s\n",
				    usbd_errstr(error)));
				mtx_lock(xfer->priv_mtx);
				goto tr_stalled;
			}
			error = usbd_probe_and_attach(udev,
			    req.wIndex[0]);
			if (error) {
				PRINTFN(0, ("alt setting probe failed\n"));
				mtx_lock(xfer->priv_mtx);
				goto tr_stalled;
			}
			mtx_lock(xfer->priv_mtx);
			break;
		default:
			goto tr_stalled;
		}
		break;

	case UT_READ_INTERFACE:
		switch (req.bRequest) {
		case UR_GET_INTERFACE:
			src_mcopy = &(iface->alt_index);
			max_len = 1;
			break;

		default:
			goto tr_stalled;
		}
		break;
	default:
		goto tr_stalled;
	}
	goto tr_valid;

tr_valid:
	if (state == ST_POST_STATUS) {
		goto tr_stalled;
	}
	/* subtract offset from length */

	max_len -= off;

	/* Compute the real maximum data length */

	if (max_len > xfer->max_data_length) {
		max_len = xfer->max_data_length;
	}
	if (max_len > rem) {
		max_len = rem;
	}
	/*
	 * If the remainder is greater than the maximum data length,
	 * we need to truncate the value for the sake of the
	 * comparison below:
	 */
	if (rem > xfer->max_data_length) {
		rem = xfer->max_data_length;
	}
	if (rem != max_len) {
		/*
	         * If we don't transfer the data we can transfer, then
	         * the transfer is short !
	         */
		xfer->flags.force_short_xfer = 1;
		xfer->nframes = 2;
	} else {
		/*
		 * Default case
		 */
		xfer->flags.force_short_xfer = 0;
		xfer->nframes = max_len ? 2 : 1;
	}
	if (max_len > 0) {
		if (src_mcopy) {
			src_mcopy = USBD_ADD_BYTES(src_mcopy, off);
			usbd_copy_in(xfer->frbuffers + 1, 0,
			    src_mcopy, max_len);
		} else {
			usbd_set_frame_data(xfer,
			    USBD_ADD_BYTES(src_zcopy, off), 1);
		}
		xfer->frlengths[1] = max_len;
	} else {
		/* the end is reached, send status */
		xfer->flags.manual_status = 0;
		xfer->frlengths[1] = 0;
	}
	PRINTFN(0, ("success\n"));
	return (0);			/* success */

tr_stalled:
	PRINTFN(0, ("%s\n", (state == ST_POST_STATUS) ?
	    "complete" : "stalled"));
	return (USBD_ERR_STALLED);

tr_bad_context:
	PRINTFN(0, ("bad context\n"));
	return (USBD_ERR_BAD_CONTEXT);
}

static const struct usbd_config usbd_control_ep_cfg[1] = {
	[0] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control endpoint */
		.direction = UE_DIR_ANY,
		.mh.bufsize = 1024,	/* bytes */
		.mh.flags = {.proxy_buffer = 1,.short_xfer_ok = 1,},
		.mh.callback = &usbd_do_request_callback,
		.md.bufsize = 1024,	/* bytes */
		.md.flags = {.proxy_buffer = 1,.short_xfer_ok = 0,},
		.md.callback = &usbd_handle_request_callback,
	},
};

/*------------------------------------------------------------------------*
 *	usbd_default_transfer_setup
 *
 * This function is used to setup the default USB control endpoint
 * transfer.
 *------------------------------------------------------------------------*/
void
usbd_default_transfer_setup(struct usbd_device *udev)
{
	struct usbd_xfer *xfer;
	uint8_t no_resetup;
	uint8_t iface_index;

repeat:

	xfer = udev->default_xfer[0];
	if (xfer) {
		mtx_lock(xfer->priv_mtx);
		no_resetup =
		    ((xfer->address == udev->address) &&
		    (udev->default_ep_desc.wMaxPacketSize[0] ==
		    udev->ddesc.bMaxPacketSize));
		if (udev->flags.usb_mode == USB_MODE_DEVICE) {
			if (no_resetup) {
				/*
				 * NOTE: checking "xfer->address" and
				 * starting the USB transfer must be
				 * atomic!
				 */
				usbd_transfer_start(xfer);
			}
		}
		mtx_unlock(xfer->priv_mtx);
	} else {
		no_resetup = 0;
	}

	if (no_resetup) {
		/*
	         * All parameters are exactly the same like before.
	         * Just return.
	         */
		return;
	}
	/*
	 * Update wMaxPacketSize for the default control endpoint:
	 */
	udev->default_ep_desc.wMaxPacketSize[0] =
	    udev->ddesc.bMaxPacketSize;

	/*
	 * Unsetup any existing USB transfer:
	 */
	usbd_transfer_unsetup(udev->default_xfer, 1);

	/*
	 * Try to setup a new USB transfer for the
	 * default control endpoint:
	 */
	iface_index = 0;
	if (usbd_transfer_setup(udev, &iface_index,
	    udev->default_xfer, usbd_control_ep_cfg, 1, NULL,
	    udev->default_mtx)) {
		PRINTFN(-1, ("could not setup default "
		    "USB transfer!\n"));
	} else {
		goto repeat;
	}
	return;
}

#ifdef USB_DEBUG
static int usb_ss_delay = 0;

SYSCTL_INT(_hw_usb, OID_AUTO, ss_delay, CTLFLAG_RW,
    &usb_ss_delay, 0, "USB status stage delay in ms");
#endif

/*------------------------------------------------------------------------*
 *	usbd_do_request_flags and usbd_do_request
 *
 * Description of arguments passed to these functions:
 *
 * "udev" - this is the "usb_device" structure pointer on which the
 * request should be performed. It is possible to call this function
 * in both Host Side mode and Device Side mode.
 *
 * "mtx" - if this argument is non-NULL the mutex pointed to by it
 * will get dropped and picked up during the execution of this
 * function, hence this function sometimes needs to sleep. If this
 * argument is NULL it has no effect.
 *
 * "req" - this argument must always be non-NULL and points to an
 * 8-byte structure holding the USB request to be done. The USB
 * request structure has a bit telling the direction of the USB
 * request, if it is a read or a write.
 *
 * "data" - if the "wLength" part of the structure pointed to by "req"
 * is non-zero this argument must point to a valid kernel buffer which
 * can hold at least "wLength" bytes. If "wLength" is zero "data" can
 * be NULL.
 *
 * "flags" - here is a list of valid flags:
 *
 *  o USBD_SHORT_XFER_OK: allows the data transfer to be shorter than
 *  specified
 *
 *  o USBD_USE_POLLING: forces the transfer to complete from the
 *  current context by polling the interrupt handler. This flag can be
 *  used to perform USB transfers after that the kernel has crashed.
 *
 *  o USBD_DELAY_STATUS_STAGE: allows the status stage to be performed
 *  at a later point in time. This is tunable by the "hw.usb.ss_delay"
 *  sysctl. This flag is mostly useful for debugging.
 *
 * "actlen" - if non-NULL the actual transfer length will be stored in
 * the 16-bit unsigned integer pointed to by "actlen". This
 * information is mostly useful when the "USBD_SHORT_XFER_OK" flag is
 * used.
 *
 * "timeout" - gives the timeout for the control transfer in
 * milliseconds. A "timeout" value less than 50 milliseconds is
 * treated like a 50 millisecond timeout. A "timeout" value greater
 * than 30 seconds is treated like a 30 second timeout. This USB stack
 * does not allow control requests without a timeout.
 *
 * NOTE: This function is thread safe. All calls to
 * "usbd_do_request_flags" will be serialised by the use of an
 * internal "sx_lock".
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_do_request_flags(struct usbd_device *udev, struct mtx *mtx,
    usb_device_request_t *req, void *data,
    uint32_t flags, uint16_t *actlen, uint32_t timeout)
{
	struct usbd_xfer *xfer;
	const void *desc;
	uint32_t level = 0;
	uint32_t start_ticks;
	uint32_t delta_ticks;
	uint32_t max_ticks;
	uint16_t length;
	uint16_t temp;
	usbd_status_t err = 0;

	if (timeout < 50) {
		/* timeout is too small */
		timeout = 50;
	}
	if (timeout > 30000) {
		/* timeout is too big */
		timeout = 30000;
	}
	length = UGETW(req->wLength);

	PRINTFN(4, ("udev=%p bmRequestType=0x%02x bRequest=0x%02x "
	    "wValue=0x%02x%02x wIndex=0x%02x%02x wLength=0x%02x%02x\n",
	    udev, req->bmRequestType, req->bRequest,
	    req->wValue[1], req->wValue[0],
	    req->wIndex[1], req->wIndex[0],
	    req->wLength[1], req->wLength[0]));

	/*
	 * Set "actlen" to a known value in case the caller does not
	 * check the return value:
	 */
	if (actlen) {
		*actlen = 0;
	}
	if (udev->flags.usb_mode == USB_MODE_DEVICE) {
		PRINTFN(0, ("USB device mode\n"));
		usbd_temp_get_desc(udev, req, &desc, &temp);
		if (length > temp) {
			if (!(flags & USBD_SHORT_XFER_OK)) {
				return (USBD_ERR_SHORT_XFER);
			}
			length = temp;
		}
		if (actlen) {
			*actlen = length;
		}
		if (length > 0) {
			bcopy(desc, data, length);
		}
		return (0);		/* success */
	}
	/*
	 * Drop any mutex:
	 */

	if (mtx) {
		/*
	         * XXX there is some code out there
	         * where recursive locking is really
	         * hard to get rid of. Until further
	         * use "mtx_drop_recurse()" instead
	         * of a single unlock! --hps
	         */
		level = mtx_drop_recurse(mtx);
		mtx_unlock(mtx);
	}
	/*
	 * Grab the default sx-lock so that serialisation
	 * is achieved when multiple threads are involved:
	 */

	sx_xlock(udev->default_sx);

	/*
	 * Setup a new USB transfer or use the existing one, if any:
	 */
	usbd_default_transfer_setup(udev);

	xfer = udev->default_xfer[0];
	if (xfer == NULL) {
		/* most likely out of memory */
		err = USBD_ERR_NOMEM;
		goto done;
	}
	mtx_lock(xfer->priv_mtx);

	if ((flags & USBD_USE_POLLING) || cold) {
		xfer->flags.use_polling = 1;
	} else {
		xfer->flags.use_polling = 0;
	}

	if (flags & USBD_DELAY_STATUS_STAGE) {
		xfer->flags.manual_status = 1;
	} else {
		xfer->flags.manual_status = 0;
	}

	xfer->timeout = timeout;

	start_ticks = ticks;

	max_ticks = USBD_MS_TO_TICKS(timeout);

	usbd_copy_in(xfer->frbuffers, 0, req, sizeof(*req));

	xfer->frlengths[0] = sizeof(*req);
	xfer->nframes = 2;

	while (1) {
		temp = length;
		if (temp > xfer->max_data_length) {
			temp = xfer->max_data_length;
		}
		xfer->frlengths[1] = temp;

		if (temp > 0) {
			if (!(req->bmRequestType & UT_READ)) {
				usbd_copy_in(xfer->frbuffers + 1, 0, data, temp);
			}
			xfer->nframes = 2;
		} else {
			if (xfer->frlengths[0] == 0) {
				if (xfer->flags.manual_status) {
#ifdef USB_DEBUG
					int temp;

					temp = usb_ss_delay;
					if (temp > 5000) {
						temp = 5000;
					}
					if (temp > 0) {
						usbd_pause_mtx(
						    xfer->priv_mtx, temp);
					}
#endif
					xfer->flags.manual_status = 0;
				} else {
					break;
				}
			}
			xfer->nframes = 1;
		}

		usbd_transfer_start(xfer);

		while (xfer->flags_int.transferring) {

			if (mtx_sleep(xfer, xfer->priv_mtx, 0, "usbctrl", 0)) {
				/* should not happen */
			}
		}

		err = xfer->error;

		if (err) {
			break;
		}
		/* subtract length of SETUP packet, if any */

		if (xfer->aframes > 0) {
			xfer->actlen -= xfer->frlengths[0];
		} else {
			xfer->actlen = 0;
		}

		/* check for short packet */

		if (temp > xfer->actlen) {
			temp = xfer->actlen;
			if (!(flags & USBD_SHORT_XFER_OK)) {
				err = USBD_ERR_SHORT_XFER;
			}
			length = temp;
		}
		if (temp > 0) {
			if (req->bmRequestType & UT_READ) {
				usbd_copy_out(xfer->frbuffers + 1, 0, data, temp);
			}
		}
		/*
		 * Clear "frlengths[0]" so that we don't send the setup
		 * packet again:
		 */
		xfer->frlengths[0] = 0;

		/* update length and data pointer */
		length -= temp;
		data = USBD_ADD_BYTES(data, temp);

		if (actlen) {
			(*actlen) += temp;
		}
		/* check for timeout */

		delta_ticks = ticks - start_ticks;
		if (delta_ticks > max_ticks) {
			if (!err) {
				err = USBD_ERR_TIMEOUT;
			}
		}
		if (err) {
			break;
		}
	}

	mtx_unlock(xfer->priv_mtx);

done:
	sx_xunlock(udev->default_sx);

	if (mtx) {
		mtx_lock(mtx);
		mtx_pickup_recurse(mtx, level);
	}
	return (err);
}

/*------------------------------------------------------------------------*
 *	usbd_fill_get_report - factored out code
 *------------------------------------------------------------------------*/
void
usbd_fill_get_report(usb_device_request_t *req, uint8_t iface_no,
    uint8_t type, uint8_t id, uint16_t size)
{
	req->bmRequestType = UT_READ_CLASS_INTERFACE;
	req->bRequest = UR_GET_REPORT;
	USETW2(req->wValue, type, id);
	req->wIndex[0] = iface_no;
	req->wIndex[1] = 0;
	USETW(req->wLength, size);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_fill_set_report - factored out code
 *------------------------------------------------------------------------*/
void
usbd_fill_set_report(usb_device_request_t *req, uint8_t iface_no,
    uint8_t type, uint8_t id, uint16_t size)
{
	req->bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req->bRequest = UR_SET_REPORT;
	USETW2(req->wValue, type, id);
	req->wIndex[0] = iface_no;
	req->wIndex[1] = 0;
	USETW(req->wLength, size);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_clear_data_toggle - factored out code
 *
 * NOTE: the intention of this function is not to reset the hardware data toggle.
 *------------------------------------------------------------------------*/
void
usbd_clear_data_toggle(struct usbd_device *udev, struct usbd_pipe *pipe)
{
	PRINTFN(4, ("udev=%p pipe=%p\n", udev, pipe));

	mtx_lock(&(udev->bus->mtx));
	pipe->toggle_next = 0;
	mtx_unlock(&(udev->bus->mtx));
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_clear_stall_callback - factored out clear stall callback
 *
 * Input parameters:
 *  xfer1: Clear Stall Control Transfer
 *  xfer2: Stalled USB Transfer
 *
 * This function is NULL safe.
 *
 * Return values:
 *   0: In progress
 *   Else: Finished
 *
 * Clear stall config example:
 *
 * static const struct usbd_config my_clearstall =  {
 *	.type = UE_CONTROL,
 *	.endpoint = 0,
 *	.direction = UE_DIR_ANY,
 *	.interval = 50, //50 milliseconds
 *	.bufsize = sizeof(usb_device_request_t),
 *	.mh.timeout = 1000, //1.000 seconds
 *	.mh.flags = { },
 *	.mh.callback = &my_clear_stall_callback, // **
 * };
 *
 * ** "my_clear_stall_callback" calls "usbd_clear_stall_callback"
 * passing the correct parameters.
 *------------------------------------------------------------------------*/
uint8_t
usbd_clear_stall_callback(struct usbd_xfer *xfer1,
    struct usbd_xfer *xfer2)
{
	usb_device_request_t req;

	if (xfer2 == NULL) {
		/* looks like we are tearing down */
		PRINTFN(0, ("NULL input parameter\n"));
		return (0);
	}
	mtx_assert(xfer1->priv_mtx, MA_OWNED);
	mtx_assert(xfer2->priv_mtx, MA_OWNED);

	switch (USBD_GET_STATE(xfer1)) {
	case USBD_ST_SETUP:

		/*
		 * pre-clear the data toggle to DATA0 ("umass.c" and
		 * "ata-usb.c" depends on this)
		 */

		usbd_clear_data_toggle(xfer2->udev, xfer2->pipe);

		/* setup a clear-stall packet */

		req.bmRequestType = UT_WRITE_ENDPOINT;
		req.bRequest = UR_CLEAR_FEATURE;
		USETW(req.wValue, UF_ENDPOINT_HALT);
		req.wIndex[0] = xfer2->pipe->edesc->bEndpointAddress;
		req.wIndex[1] = 0;
		USETW(req.wLength, 0);

		/*
		 * "usbd_transfer_setup_sub()" will ensure that
		 * we have sufficient room in the buffer for
		 * the request structure!
		 */

		/* copy in the transfer */

		usbd_copy_in(xfer1->frbuffers, 0, &req, sizeof(req));

		/* set length */
		xfer1->frlengths[0] = sizeof(req);
		xfer1->nframes = 1;

		usbd_start_hardware(xfer1);
		return (0);

	case USBD_ST_TRANSFERRED:
		break;

	default:			/* Error */
		if (xfer1->error == USBD_ERR_CANCELLED) {
			return (0);
		}
		break;
	}

	usbd_transfer_clear_stall(xfer2);

	return (1);			/* Clear Stall Finished */
}

/*------------------------------------------------------------------------*
 *	usbd_do_poll
 *
 * called from keyboard driver when in polling mode
 *------------------------------------------------------------------------*/
void
usbd_do_poll(struct usbd_device *udev)
{
	(udev->bus->methods->do_poll) (udev->bus);
	return;
}

/*------------------------------------------------------------------------*
 *	usb_match_device
 *
 * Search for a vendor/product pair in an array.  The item size is
 * given as an argument.
 *------------------------------------------------------------------------*/
const struct usb_devno *
usb_match_device(const struct usb_devno *tbl, u_int nentries, u_int size,
    uint16_t vendor, uint16_t product)
{
	while (nentries-- > 0) {
		if ((tbl->ud_vendor == vendor) &&
		    ((tbl->ud_product == product) ||
		    (tbl->ud_product == USB_PRODUCT_ANY))) {
			return (tbl);
		}
		tbl = (const struct usb_devno *)
		    (((const uint8_t *)tbl) + size);
	}
	return (NULL);
}

/*------------------------------------------------------------------------*
 *	usbd_driver_load
 *------------------------------------------------------------------------*/
int
usbd_driver_load(struct module *mod, int what, void *arg)
{
	/*
	 * XXX should implement something like a function that removes all
	 * generic devices
	 */

	return (0);
}
