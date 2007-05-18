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

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_hid.h>

__FBSDID("$FreeBSD: src/sys/dev/usb2/usb_transfer.c $");

#ifdef USB_DEBUG
void
usbd_dump_iface(struct usbd_interface *iface)
{
	printf("usbd_dump_iface: iface=%p\n", iface);
	if(iface == NULL)
	{
		return;
	}
	printf(" iface=%p idesc=%p altindex=%d\n",
	       iface, iface->idesc, iface->alt_index);
	return;
}

void
usbd_dump_device(struct usbd_device *udev)
{
	printf("usbd_dump_device: dev=%p\n", udev);
	if(udev == NULL)
	{
		return;
	}
	printf(" bus=%p \n"
	       " address=%d config=%d depth=%d speed=%d self_powered=%d\n"
	       " power=%d langid=%d\n",
	       udev->bus, 
	       udev->address, udev->config, udev->depth, udev->speed,
	       udev->self_powered, udev->power, udev->langid);
	return;
}

void
usbd_dump_queue(struct usbd_pipe *pipe)
{
	struct usbd_xfer *xfer;

	printf("usbd_dump_queue: pipe=%p\n", pipe);
	LIST_FOREACH(xfer, &pipe->list_head, pipe_list)
	{
		printf("  xfer=%p\n", xfer);
	}
	return;
}

void
usbd_dump_pipe(struct usbd_pipe *pipe)
{
	if(pipe)
	{
		printf("usbd_dump_pipe: pipe=%p", pipe);

		printf(" edesc=%p isoc_next=%d toggle_next=%d",
		       pipe->edesc, pipe->isoc_next, pipe->toggle_next);

		if(pipe->edesc)
		{
			printf(" bEndpointAddress=0x%02x",
			       pipe->edesc->bEndpointAddress);
		}
		printf("\n");
		usbd_dump_queue(pipe);
	}
	else
	{
		printf("usbd_dump_pipe: pipe=NULL\n");
	}
	return;
}

void
usbd_dump_xfer(struct usbd_xfer *xfer)
{
	printf("usbd_dump_xfer: xfer=%p\n", xfer);
	if(xfer == NULL)
	{
		return;
	}
        if(xfer->pipe == NULL)
	{
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

u_int32_t
usb_get_devid(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	return ((uaa->vendor << 16) | (uaa->product));
}

struct usbd_pipe *
usbd_get_pipe(struct usbd_device *udev, u_int8_t iface_index,
	      const struct usbd_config *setup)
{
	struct usbd_pipe *pipe;
	uint8_t index = setup->index;
	uint8_t ea;
	uint8_t at;

	PRINTFN(8,("udev=%p iface_index=%d address=0x%x "
		    "type=0x%x dir=0x%x index=%d\n",
		    udev, iface_index, setup->endpoint,
		    setup->type, setup->direction, setup->index));

	/* NOTE: pipes are searched from the beginning */

	for (pipe = udev->pipes;
	     ((pipe >= udev->pipes) && 
	      (pipe < udev->pipes_end));
	     pipe++) {

	    if ((pipe->edesc == NULL) ||
		(pipe->iface_index != iface_index)) {
	        continue;
	    }

	    ea = pipe->edesc->bEndpointAddress;
	    at = pipe->edesc->bmAttributes;

	    if (((setup->direction == (ea & (UE_DIR_IN|UE_DIR_OUT))) ||
		 (setup->direction == UE_DIR_ANY)) &&
		((setup->endpoint == (ea & UE_ADDR)) ||
		 (setup->endpoint == UE_ADDR_ANY)) &&
		((setup->type == (at & UE_XFERTYPE)) ||
		 (setup->type == UE_TYPE_ANY) ||
		 ((setup->type == UE_BULK_INTR) && (at & 2)))) {

	        if(!index--) {
		    goto found;
		}
	    }
	}

	/* Match against default pipe last, so that "any pipe", 
	 * "any address" and "any direction" returns the first 
	 * pipe of the interface. "iface_index" and "direction"
	 * is ignored:
	 */
	if((setup->endpoint == 0) &&
	   (setup->type == 0)) {
		pipe = &udev->default_pipe;
		goto found;
	}
	return NULL;

 found:
	return pipe;
}

usbd_status
usbd_interface_count(struct usbd_device *udev, u_int8_t *count)
{
	if(udev->cdesc == NULL)
	{
		return (USBD_NOT_CONFIGURED);
	}
	*count = udev->cdesc->bNumInterface;
	return (USBD_NORMAL_COMPLETION);
}

/*---------------------------------------------------------------------------*
 *	usbd_transfer_setup - setup an array of USB transfers
 *
 * NOTE: must always call unsetup after setup 
 * NOTE: the parameter "iface_index" is ignored in
 *       case of control pipes
 *
 * The idea is that the USB device driver should pre-allocate all
 * its transfers by one call to this function.
 *---------------------------------------------------------------------------*/
usbd_status
usbd_transfer_setup(struct usbd_device *udev, 
		    u_int8_t iface_index, 
		    struct usbd_xfer **pxfer,
		    const struct usbd_config *setup_start, 
		    u_int16_t n_setup, 
		    void *priv_sc,
		    struct mtx *priv_mtx)
{
	const struct usbd_config *setup_end = setup_start + n_setup;
	const struct usbd_config *setup;
	struct usbd_memory_info *info;
	struct usbd_xfer *xfer;
	usbd_status error = 0;
	u_int16_t n;

	WITNESS_WARN(WARN_GIANTOK | WARN_SLEEPOK, NULL, 
		     "usbd_transfer_setup can sleep!");

	/* do some checking first */

	if(n_setup == 0)
	{
		PRINTFN(5, ("setup array has zero length!\n"));
		return USBD_INVAL;
	}

	if(priv_mtx == NULL)
	{
		PRINTFN(5, ("using global lock\n"));
		priv_mtx = &usb_global_lock;
	}

	for(setup = setup_start, n = n_setup; n--; setup++)
	{
		if(setup->bufsize == 0xffffffff)
		{
		    error = USBD_BAD_BUFSIZE;
		    PRINTF(("invalid bufsize\n"));
		}
		if(setup->flags & USBD_SYNCHRONOUS)
		{
		    if(setup->callback != &usbd_default_callback)
		    {
		        error = USBD_SYNC_TRANSFER_MUST_USE_DEFAULT_CALLBACK;
			PRINTF(("synchronous transfers "
				"must use default callback\n"));
		    }
		}
		if(setup->flags & 
		   (~(USBD_SYNCHRONOUS|
		      USBD_FORCE_SHORT_XFER|
		      USBD_SHORT_XFER_OK|
		      USBD_USE_POLLING|
		      USBD_USE_DMA)))
		{
		    error = USBD_BAD_FLAG;
		    PRINTF(("invalid flag(s) specified: "
			    "0x%08x\n", setup->flags));
		}
		if(setup->callback == NULL)
		{
		    error = USBD_NO_CALLBACK;
		    PRINTF(("no callback\n"));
		}
		pxfer[n] = NULL;
	}

	if(error)
	{
		goto done;
	}

	error = (udev->bus->methods->xfer_setup)
	  (udev,iface_index,pxfer,setup_start,setup_end);


	/* common setup */

	for(setup = setup_start, n = n_setup; n--; setup++)
	{
		xfer = pxfer[n];

		if(xfer)
		{
		    xfer->priv_sc = priv_sc;
		    xfer->priv_mtx = priv_mtx;
		    xfer->udev = udev;

		    if(xfer->pipe)
		    {
		        xfer->pipe->refcount++;
		    }

		    info = xfer->usb_root;
		    info->memory_refcount++;
		    info->setup_refcount++;
		}
	}

 done:
	if(error)
	{
		usbd_transfer_unsetup(pxfer, n_setup);
	}
	return (error);
}

/*---------------------------------------------------------------------------*
 *	usbd_drop_refcount 
 *
 * This function is called from various places, and its job is to
 * wakeup "usbd_transfer_unsetup", when is safe to free the memory.
 *---------------------------------------------------------------------------*/
static void
usbd_drop_refcount(struct usbd_memory_info *info)
{
    mtx_lock(info->usb_mtx);

    __KASSERT(info->memory_refcount != 0, ("Invalid memory "
					   "reference count!\n"));

    if ((--(info->memory_refcount)) == 0) {
        wakeup(info);
    }

    mtx_unlock(info->usb_mtx);

    return;
}

/*---------------------------------------------------------------------------*
 *	usbd_transfer_unsetup - unsetup/free an array of USB transfers
 *
 * NOTE: if the transfer was in progress, the callback will 
 * called with "xfer->error=USBD_CANCELLED", before this
 * function returns
 *---------------------------------------------------------------------------*/
void
usbd_transfer_unsetup(struct usbd_xfer **pxfer, u_int16_t n_setup)
{
	struct usbd_xfer *xfer;
	struct usbd_memory_info *info;
	int error;

	WITNESS_WARN(WARN_GIANTOK | WARN_SLEEPOK, NULL, 
		     "usbd_transfer_unsetup can sleep!");

	while(n_setup--)
	{
	    xfer = pxfer[n_setup];

	    if(xfer)
	    {
		if(xfer->pipe)
		{
		    mtx_lock(xfer->priv_mtx);

		    /* HINT: when you start/stop a transfer,
		     * it might be a good idea to directly
		     * use the "pxfer[]" structure:
		     *
		     * usbd_transfer_start(sc->pxfer[0]);
		     * usbd_transfer_stop(sc->pxfer[0]);
		     *
		     * That way, if your code has many parts
		     * that will not stop running under the
		     * same lock, in other words "priv_mtx",
		     * the usbd_transfer_start and
		     * usbd_transfer_stop functions will
		     * simply return when they detect a NULL
		     * pointer argument.
		     *
		     * To avoid any races we clear the "pxfer[]"
		     * pointer while holding the private mutex
		     * of the driver:
		     */
		    pxfer[n_setup] = NULL;

		    usbd_transfer_stop(xfer);

		    /* NOTE: default pipe does not
		     * have an interface, even if
		     * pipe->iface_index == 0
		     */
		    xfer->pipe->refcount--;

		    mtx_unlock(xfer->priv_mtx);
		} else {
		    /* clear the transfer pointer */
		    pxfer[n_setup] = NULL;
		}

		__callout_drain(&(xfer->timeout_handle));

		if(xfer->usb_root)
		{
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

		        /* wait for any USB callbacks to return */

		        while (info->memory_refcount > 0) {
			    error = mtx_sleep(info, info->usb_mtx, 0, 
					   "usb_mem_wait", 0);
			}

			mtx_unlock(info->usb_mtx);

			/* free DMA'able memory, if any */

			usbd_page_free(info->page_base, info->page_size);

			/* free the "memory_base" last, hence the
			 * "info" structure is contained within
			 * the "memory_base"!
			 */
			free(info->memory_base, M_USB);

		    } else {
		        mtx_unlock(info->usb_mtx);
		    }

		}
	    }
	}
	return;
}

void
usbd_std_isoc_copy_in(struct usbd_xfer *xfer)
{
    u_int32_t x;
    u_int32_t length;

    if (UE_GET_DIR(xfer->endpoint) == UE_DIR_OUT) {

        length = 0;

	for (x = 0; x < xfer->nframes; x++) {
	    length += xfer->frlengths[x];
	}

	/* only one copy is needed, hence 
	 * the frames are back to back:
	 */
	usbd_copy_in(&(xfer->buf_data), 0, xfer->buffer, length);

    } else {

        for (x = 0; x < xfer->nframes; x++) {
	    xfer->frlengths_old[x] = xfer->frlengths[x];
	}
    }
    return;
}

void
usbd_std_isoc_copy_out(struct usbd_xfer *xfer)
{
    u_int8_t *ptr;
    u_int32_t x;
    u_int32_t offset;

    if (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN) {

        ptr = xfer->buffer;
	offset = 0;

        for (x = 0; x < xfer->nframes; x++) {

	    usbd_copy_out(&(xfer->buf_data), offset,
			  ptr, xfer->frlengths[x]);

	    ptr += xfer->frlengths_old[x];
	    offset += xfer->frlengths_old[x];
	}
    }
    return;
}

void
usbd_std_bulk_intr_copy_in(struct usbd_xfer *xfer)
{
    if (UE_GET_DIR(xfer->endpoint) == UE_DIR_OUT) {

        usbd_copy_in(&(xfer->buf_data), 0, 
		     xfer->buffer, xfer->length);
    }
    return;
}

void
usbd_std_bulk_intr_copy_out(struct usbd_xfer *xfer)
{
    if (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN) {

        usbd_copy_out(&(xfer->buf_data), 0, 
		      xfer->buffer, xfer->actlen);
    }
    return;
}

void
usbd_std_ctrl_copy_in(struct usbd_xfer *xfer)
{
    u_int32_t len = xfer->length;
    usb_device_request_t *req;

    if (xfer->control_remainder == 0) {
        req = xfer->buffer;
	if ((len >= sizeof(*req)) &&
	    (req->bmRequestType & UT_READ)) {
	    len = sizeof(*req);
	}
    } else {
        if (xfer->control_isread) {
	    return;
	}
    }

    usbd_copy_in(&(xfer->buf_data), 0, 
		 xfer->buffer, len);
    return;
}

void
usbd_std_ctrl_copy_out(struct usbd_xfer *xfer)
{
    u_int32_t len = xfer->actlen;
    usb_device_request_t *req;

    if (xfer->flags & USBD_DEV_CONTROL_HEADER) {
        req = xfer->buffer;
        if ((len >= sizeof(*req)) &&
	    (req->bmRequestType & UT_READ)) {
	    /* copy out everything, but the header */
	    usbd_copy_out(&(xfer->buf_data), sizeof(*req), 
			  (req+1), len - sizeof(*req));
	}
    } else {
        if (xfer->control_isread) {
	    /* copy out everything */
	    usbd_copy_out(&(xfer->buf_data), 0,
			  xfer->buffer, len);
	}
    }
    return;
}

uint8_t
usbd_std_ctrl_enter(struct usbd_xfer *xfer)
{
    usb_device_request_t req;
    uint32_t len;

    /* check lengths and stuff */

    if (xfer->control_remainder > 0) {

        xfer->flags &= ~USBD_DEV_CONTROL_HEADER;

	if (xfer->length > xfer->control_remainder) {
	    PRINTFN(-1, ("transfer length overflow, %d bytes\n",
			 xfer->length));
	    goto error;
	}

	xfer->control_remainder -= xfer->length;

    } else {

        xfer->flags |= USBD_DEV_CONTROL_HEADER;

	if (xfer->length < sizeof(req)) {
	    PRINTFN(-1, ("transfer length underflow, %d bytes\n",
			 xfer->length));
	    goto error;
	}

	/* Copy out the header from the USB request */

	usbd_copy_out(&(xfer->buf_data), 0, &req, sizeof(req));

	xfer->control_remainder = UGETW(req.wLength);
	xfer->control_isread = (req.bmRequestType & UT_READ) ? 1 : 0;

	len = (xfer->length - sizeof(req));

	if (len > xfer->control_remainder) {
	    PRINTFN(-1, ("transfer length invalid, %d bytes\n",
			 xfer->length));
	    goto error;
	}

	xfer->control_remainder -= len;
    }
    return 0;

 error:
    /* set error */
    usbd_transfer_done(xfer, USBD_STALLED);
    /* call callback */
    __usbd_callback(xfer);
    /* return error */
    return 1;
}

/* CALLBACK EXAMPLES:
 * ==================
 *
 * USBD_CHECK_STATUS() overview of possible program paths:
 * =======================================================
 *
 *       +->-----------------------+
 *       |                         |    
 *   +-<-+-------[tr_setup]--------+-<-+-<-[start/restart]
 *   |                                 |
 *   |                                 |
 *   |                                 |
 *   +------>-[tr_transferred]---------+
 *   |                                 |
 *   +--------->-[tr_error]------------+
 *
 * NOTE: the USB-driver automatically
 * recovers from errors if 
 * "xfer->clearstall_xfer" is set
 *
 * Host-transmit callback example (bulk/interrupt/isochronous):
 * ============================================================
 * static void
 * usb_callback_tx(struct usbd_xfer *xfer)
 * {
 *   USBD_CHECK_STATUS(xfer);
 *
 * tr_transferred:
 * tr_setup:
 *
 *   ... setup "xfer->length" ...
 *
 *   ... write data to buffer ...
 *
 * tr_error:
 *
 *   ... [re-]transfer "xfer->buffer" ...
 *
 *   usbd_start_hardware(xfer);
 *   return;
 * }
 *
 * Host-receive callback example (bulk/interrupt/isochronous):
 * ===========================================================
 * static void
 * usb_callback_rx(struct usbd_xfer *xfer)
 * {
 *   USBD_CHECK_STATUS(xfer);
 *
 * tr_transferred:
 *
 *   ... process data in buffer ...
 *
 * tr_setup:
 *
 *   ... setup "xfer->length" ...
 *
 * tr_error:
 *
 *   ... [re-]transfer "xfer->buffer" ...
 *
 *   usbd_start_hardware(xfer);
 *   return;
 * }
 *
 *
 * "usbd_start_hardware()" is called when 
 * "xfer->buffer" is ready for transfer
 *
 * "usbd_start_hardware()" should only be called
 *  from callback
 */

/*---------------------------------------------------------------------------*
 *	usbd_start_hardware - start USB hardware for the given transfer
 *---------------------------------------------------------------------------*/
void
usbd_start_hardware(struct usbd_xfer *xfer)
{
	PRINTFN(0,("xfer=%p, pipe=%p len=%d dir=%s\n",
		    xfer, xfer->pipe, xfer->length, 
		    ((xfer->pipe->edesc->bEndpointAddress & 
		      (UE_DIR_IN|UE_DIR_OUT)) == UE_DIR_IN) ? "in" : "out"));

#ifdef USB_DEBUG
	if(usbdebug > 0)
	{
		mtx_lock(xfer->usb_mtx);

		usbd_dump_pipe(xfer->pipe);

		mtx_unlock(xfer->usb_mtx);
	}
#endif

	mtx_assert(xfer->priv_mtx, MA_OWNED);

	if(xfer->flags & USBD_DEV_OPEN)
	{
		/* set USBD_DEV_TRANSFERRING and USBD_DEV_RECURSED_2 */
		xfer->flags |= (USBD_DEV_TRANSFERRING|USBD_DEV_RECURSED_2);

		if(xfer->pipe->clearstall &&
		   xfer->clearstall_xfer)
		{
#ifdef USB_DEBUG
			if(xfer->clearstall_xfer->flags & USBD_DEV_TRANSFERRING)
			{
				PRINTF(("clearstall_xfer is transferrring!\n"));
			}
#endif
			/* the polling flag is inherited */

			if(xfer->flags & USBD_USE_POLLING)
			  xfer->clearstall_xfer->flags |= USBD_USE_POLLING;
			else
			  xfer->clearstall_xfer->flags &= ~USBD_USE_POLLING;

			/* store pointer to transfer */
			xfer->clearstall_xfer->priv_sc = xfer;

			usbd_transfer_start(xfer->clearstall_xfer);
		}
		else
		{
		    if(!(xfer->flags & USBD_USE_DMA)) {
		        /* copy in data */
		        (xfer->pipe->methods->copy_in)(xfer);
		    }

		    mtx_lock(xfer->usb_mtx);

		    /* enter the transfer */
		    (xfer->pipe->methods->enter)(xfer);

		    xfer->usb_thread = (xfer->flags & USBD_USE_POLLING) ? 
		      curthread : NULL;

		    mtx_unlock(xfer->usb_mtx);
 		}
	}
	return;
}

void
usbd_transfer_start_safe(struct usbd_xfer *xfer)
{
	mtx_lock(xfer->priv_mtx);
	usbd_transfer_start(xfer);
	mtx_unlock(xfer->priv_mtx);
	return;
}

/*---------------------------------------------------------------------------*
 *	usbd_transfer_start - start an USB transfer
 *
 * NOTE: Calling this function more than one time will only
 *       result in a single transfer start, until the USB transfer
 *       completes.
 * NOTE: if USBD_SYNCHRONOUS is set in "xfer->flags", then this
 *       function will sleep for transfer completion
 * NOTE: if USBD_USE_POLLING is set in "xfer->flags", then this
 *       function will spin until transfer is completed
 *---------------------------------------------------------------------------*/
void
usbd_transfer_start(struct usbd_xfer *xfer)
{
	if (xfer == NULL) {
	    /* transfer is gone */
	    return;
	}

	mtx_assert(xfer->priv_mtx, MA_OWNED);

	if(!(xfer->flags & USBD_DEV_OPEN))
	{
		xfer->flags |= USBD_DEV_OPEN;

		/*
		 * open transfer
		 */
		mtx_lock(xfer->usb_mtx);
		(xfer->pipe->methods->open)(xfer);
		mtx_unlock(xfer->usb_mtx);
	}

	/* "USBD_DEV_TRANSFERRING" is only changed
	 * when "priv_mtx" is locked;
	 * check first recurse flag
	 */
	if(!(xfer->flags & (USBD_DEV_TRANSFERRING)))
	{
		/* call callback */
		__usbd_callback(xfer);

		/* wait for completion
		 * if the transfer is synchronous
		 */
		if(xfer->flags & (USBD_SYNCHRONOUS|USBD_USE_POLLING))
		{
			u_int timeout = xfer->timeout +1;
			struct usbd_bus *bus = xfer->udev->bus;

			while(xfer->flags & USBD_DEV_TRANSFERRING)
			{
				if(bus->use_polling ||
				   (xfer->flags & USBD_USE_POLLING))
				{
					if(!timeout--)
					{
						/* stop the transfer */
						usbd_transfer_stop(xfer);
						break;
					}

					/* delay one millisecond */
					DELAY(1000);

					/* call the interrupt handler,
					 * which will call __usbd_callback():
					 */
					(bus->methods->do_poll)(bus);
				}
				else
				{
					u_int32_t level;

					level = mtx_drop_recurse(xfer->priv_mtx);

					if(mtx_sleep(xfer, xfer->priv_mtx,
						  (PRIBIO|PCATCH), "usbsync", 0))
					{
						/* stop the transfer */
						usbd_transfer_stop(xfer);
					}

					mtx_pickup_recurse(xfer->priv_mtx, level);
					break;
				}
			}
		}
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	usbd_transfer_stop - stop an USB transfer
 *
 * NOTE: Calling this function more than one time will only
 *       result in a single transfer stop.
 *---------------------------------------------------------------------------*/
void
usbd_transfer_stop(struct usbd_xfer *xfer)
{
	if (xfer == NULL) {
	    /* transfer is gone */
	    return;
	}

	mtx_assert(xfer->priv_mtx, MA_OWNED);

	if(xfer->flags & USBD_DEV_OPEN)
	{
		xfer->flags &= ~USBD_DEV_OPEN;

		/*
		 * stop clearstall first
		 */
		if(xfer->clearstall_xfer)
		{
			usbd_transfer_stop(xfer->clearstall_xfer);
		}

		/*
		 * close transfer (should not call callback)
		 */
		mtx_lock(xfer->usb_mtx);
		(xfer->pipe->methods->close)(xfer);

		/* always set error */
		xfer->error = USBD_CANCELLED;

		if(xfer->flags & USBD_DEV_TRANSFERRING)
		{
		    /* By clearing the "usb_thread" variable
		     * we are signalling that a different
		     * thread has handled the callback.
		     * This prevents other threads that are
		     * about to call the callback, to actually
		     * call the callback.
		     */
		    xfer->usb_thread = NULL;

		    /* call callback, which 
		     * will clear USBD_DEV_TRANSFERRING
		     */
		    __usbd_callback(xfer);
		}
		mtx_unlock(xfer->usb_mtx);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	__usbd_callback
 *
 * This is a wrapper for USB callbacks, which handles
 * recursation, which can happen during boot.
 *---------------------------------------------------------------------------*/
void
__usbd_callback(struct usbd_xfer *xfer)
{
	mtx_assert(xfer->priv_mtx, MA_OWNED);

	/* check first recurse flag */
	if(!(xfer->flags & USBD_DEV_RECURSED_1))
	{
		do {
			/* set both recurse flags */
			xfer->flags |= (USBD_DEV_RECURSED_2|
					USBD_DEV_RECURSED_1);

			/* check if any data must be copied out */
			if(xfer->flags & USBD_DEV_TRANSFERRING) {
			    if(!(xfer->flags & USBD_USE_DMA)) {
			        /* copy out data */
			        (xfer->pipe->methods->copy_out)(xfer);
			    }
			}

			/* call processing routine */
			(xfer->callback)(xfer);

		/* check second recurse flag */
		} while(!(xfer->flags & USBD_DEV_RECURSED_2));

		/* clear first recurse flag */
		xfer->flags &= ~USBD_DEV_RECURSED_1;
	}
	else
	{
		/* clear second recurse flag */
		xfer->flags &= ~USBD_DEV_RECURSED_2;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	usbd_do_callback
 *
 * This function is used to call back a list of USB callbacks. 
 *---------------------------------------------------------------------------*/
void
usbd_do_callback(struct usbd_xfer **pp_xfer, struct thread *td)
{
    struct usbd_xfer *xfer;
    uint8_t tmp;

    while (1)
    {
	xfer = *(pp_xfer++);
	if (xfer == NULL) {
	    break;
	}

	/*
	 * During the unlocked period, the transfer can be restarted
	 * by another thread. Check that it is this thread that is
	 * calling back the callback:
	 */
	mtx_lock(xfer->priv_mtx);
	mtx_lock(xfer->usb_mtx);
	tmp = (xfer->usb_thread == td);
	mtx_unlock(xfer->usb_mtx);

	if (tmp) {
	    /* call callback */
	    __usbd_callback(xfer);
	}
	/* 
	 * else already called back !
	 */
	mtx_unlock(xfer->priv_mtx);

	usbd_drop_refcount(xfer->usb_root);
    }
    return;
}

/*---------------------------------------------------------------------------*
 *	usbd_transfer_done
 *
 * NOTE: this function does not call the callback!
 *---------------------------------------------------------------------------*/
void
usbd_transfer_done(struct usbd_xfer *xfer, usbd_status error)
{
	mtx_assert(xfer->usb_mtx, MA_OWNED);

	PRINTFN(5,("xfer=%p pipe=%p status=%d "
		    "actlen=%d\n", xfer, xfer->pipe, error, xfer->actlen));

       	/* count completed transfers */
	++(xfer->udev->bus->stats.uds_requests
		[xfer->pipe->edesc->bmAttributes & UE_XFERTYPE]);

	/* check for short transfers */
	if (error) {
		/* end of control transfer, if any */
		xfer->control_remainder = 0;
	} else {
		if (xfer->actlen > xfer->length) {

		    PRINTFN(-1,("overrun actlen(%d) > len(%d)\n",
				xfer->actlen, xfer->length));
		    xfer->actlen = xfer->length;
		}

		if (xfer->actlen < xfer->length) {

		    /* end of control transfer, if any */
		    xfer->control_remainder = 0;

		    if (!(xfer->flags & USBD_SHORT_XFER_OK)) {
			PRINTFN(-1,("short transfer actlen(%d) < len(%d)\n",
				    xfer->actlen, xfer->length));
			error = USBD_SHORT_XFER;
		    }
		}
	}
	xfer->error = error;
	return;
}

/*---------------------------------------------------------------------------*
 *	usbd_transfer_enqueue
 *
 * This function is used to add an USB transfer to the pipe transfer list.
 *---------------------------------------------------------------------------*/
void
usbd_transfer_enqueue(struct usbd_xfer *xfer)
{
	mtx_assert(xfer->usb_mtx, MA_OWNED);

	/* if the transfer is not inserted, 
	 * insert the transfer into the transfer queue
	 */
	if(xfer->pipe_list.le_prev == NULL)
	{
		LIST_INSERT_HEAD(&xfer->pipe->list_head, xfer, pipe_list);
	}

	/* Handled cases:
	 *
	 * 1) Start the first transfer queued.
	 *    This transfer is always last on the pipe transfer list!
	 *    "le_next" is NULL. "le_prev" is also NULL.
	 *
	 * 2) Start the last transfer if it is
	 *    already queued. We are most likely
	 *    resuming a control transfer.
	 *    "le_next" is NULL. "le_prev" is not NULL.
	 */
	if (xfer->pipe_list.le_next == NULL)
	{
		(xfer->pipe->methods->start)(xfer);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	usbd_transfer_dequeue
 *
 * This function is used to remove an USB transfer from the pipe transfer list.
 * This function is also used to start the next USB transfer on the pipe 
 * transfer list, if any.
 *
 * IMPLEMENTATION NOTE:
 *       The transfer will not be removed from the pipe transfer list
 *       when "xfer->control_remainder" is non-zero. To ensure that a
 *       transfer is always removed from the pipe transfer list, call
 *       the "usbd_transfer_done()" function one extra time passing an
 *       error code from the pipe close method. Then call
 *       "usbd_transfer_dequeue()".
 *---------------------------------------------------------------------------*/
#undef LIST_PREV
#define LIST_PREV(head,elm,field) \
  (((elm) == LIST_FIRST(head)) ? ((__typeof(elm))0) : \
   ((__typeof(elm))(((uint8_t *)((elm)->field.le_prev)) - \
		    ((uint8_t *)&LIST_NEXT((__typeof(elm))0,field)))))

void
usbd_transfer_dequeue(struct usbd_xfer *xfer)
{
	struct usbd_xfer *xfer_prev;

	mtx_assert(xfer->usb_mtx, MA_OWNED);

	/* check if we are in the middle of a transfer first */

	if (xfer->control_remainder == 0)
	{
	    /* check if we are queued on any pipe transfer list */

	    if (xfer->pipe_list.le_prev)
	    {
		/* get the previous transfer, if any */
		xfer_prev = LIST_PREV(&(xfer->pipe->list_head), xfer, pipe_list);

		/* remove the transfer from pipe transfer list */
		LIST_REMOVE(xfer,pipe_list);

		/* start "next" transfer, if any */
		if (xfer_prev &&
		    (xfer_prev->pipe_list.le_next == NULL))
		{
			(xfer_prev->pipe->methods->start)(xfer_prev);
		}
		xfer->pipe_list.le_prev = 0;
	    }
	}
	return;
}

void
usbd_default_callback(struct usbd_xfer *xfer)
{
	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/**/
	usbd_start_hardware(xfer);
	return;

 tr_transferred:
 tr_error:
	if((xfer->flags & USBD_SYNCHRONOUS) && 
	   (!(xfer->udev->bus->use_polling || (xfer->flags & USBD_USE_POLLING))))
	{
		wakeup(xfer);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	usbd_do_request_flags / usbd_do_request
 *
 * NOTE: the caller should hold "Giant" while calling this function
 *---------------------------------------------------------------------------*/
usbd_status
usbd_do_request(struct usbd_device *udev, usb_device_request_t *req, void *data)
{
	return usbd_do_request_flags_mtx
	  (udev, NULL, req, data, 0, NULL, USBD_DEFAULT_TIMEOUT);
}

usbd_status
usbd_do_request_flags(struct usbd_device *udev, usb_device_request_t *req,
		      void *data, u_int32_t flags, int *actlen,
		      u_int32_t timeout)
{
	return usbd_do_request_flags_mtx
	  (udev, NULL, req, data, flags, actlen, timeout);
}

/*---------------------------------------------------------------------------*
 *	usbd_do_request_flags_mtx / usbd_do_request_mtx
 *
 * NOTE: the caller should hold "mtx" while calling this function
 *---------------------------------------------------------------------------*/
usbd_status
usbd_do_request_mtx(struct usbd_device *udev, struct mtx *mtx, 
		    usb_device_request_t *req, void *data)
{
	return usbd_do_request_flags_mtx
	  (udev, mtx, req, data, 0, 0, USBD_DEFAULT_TIMEOUT);
}

usbd_status
usbd_do_request_flags_mtx(struct usbd_device *udev, struct mtx *mtx,
			  usb_device_request_t *req, void *data, 
			  u_int32_t flags, int *actlen,
			  u_int32_t timeout)
{
	struct usbd_config usbd_config[1] = { /* zero */ };
	struct usbd_xfer *xfer = NULL;
	u_int16_t length = UGETW(req->wLength);
	u_int32_t level = 0;
	usbd_status err;

	PRINTFN(4,("udev=%p bmRequestType=0x%02x bRequest=0x%02x "
		   "wValue=0x%02x%02x wIndex=0x%02x%02x wLength=0x%02x%02x\n",
		   udev, req->bmRequestType, req->bRequest,
		   req->wValue[1], req->wValue[0],
		   req->wIndex[1], req->wIndex[0],
		   req->wLength[1], req->wLength[0]));

	usbd_config[0].type = UE_CONTROL;
	usbd_config[0].endpoint = 0; /* control pipe */
	usbd_config[0].direction = -1;
	usbd_config[0].timeout = timeout;
	usbd_config[0].flags = (flags|USBD_SYNCHRONOUS|USBD_USE_DMA);
	usbd_config[0].bufsize = sizeof(*req) + length;
	usbd_config[0].callback = &usbd_default_callback;

	if (mtx) {
	    level = mtx_drop_recurse(mtx);
	    mtx_unlock(mtx);
	}

	/* setup transfer */
	err = usbd_transfer_setup(udev, 0, &xfer, &usbd_config[0], 1,
				  NULL, mtx);

	if (mtx) {
	    mtx_lock(mtx);
	    mtx_pickup_recurse(mtx, level);
	}

	if(err)
	{
	    goto done;
	}

	usbd_copy_in(&(xfer->buf_data), 0, req, sizeof(*req));

	if(!(req->bmRequestType & UT_READ))
	{
	    usbd_copy_in(&(xfer->buf_data), sizeof(*req), data, length);
	}

	usbd_transfer_start_safe(xfer);

	if(req->bmRequestType & UT_READ)
	{
	    usbd_copy_out(&(xfer->buf_data), sizeof(*req), data, length);
	}

	err = xfer->error;

	if(actlen != NULL)
	{
	    actlen[0] = (xfer->actlen < sizeof(req[0])) ?
	      0 : (xfer->actlen - sizeof(req[0]));
	}

 done:
	if (mtx) {
	    level = mtx_drop_recurse(mtx);
	    mtx_unlock(mtx);
	}

	usbd_transfer_unsetup(&xfer, 1);

	if (mtx) {
	    mtx_lock(mtx);
	    mtx_pickup_recurse(mtx, level);
	}
	return (err);
}

void
usbd_fill_get_report(usb_device_request_t *req, u_int8_t iface_no, 
		     u_int8_t type, u_int8_t id, u_int16_t size)
{
        req->bmRequestType = UT_READ_CLASS_INTERFACE;
        req->bRequest = UR_GET_REPORT;
        USETW2(req->wValue, type, id);
        USETW(req->wIndex, iface_no);
        USETW(req->wLength, size);
	return;
}

void
usbd_fill_set_report(usb_device_request_t *req, u_int8_t iface_no,
		     u_int8_t type, u_int8_t id, u_int16_t size)
{
        req->bmRequestType = UT_WRITE_CLASS_INTERFACE;
        req->bRequest = UR_SET_REPORT;
        USETW2(req->wValue, type, id);
	req->wIndex[0] = iface_no;
	req->wIndex[1] = 0;
        USETW(req->wLength, size);
	return;
}

void
usbd_clear_stall_tr_setup(struct usbd_xfer *xfer1, 
			  struct usbd_xfer *xfer2)
{
	usb_device_request_t req;

	mtx_assert(xfer1->priv_mtx, MA_OWNED);
	mtx_assert(xfer2->priv_mtx, MA_OWNED);

	/* setup a clear-stall packet */

	req.bmRequestType = UT_WRITE_ENDPOINT;
	req.bRequest = UR_CLEAR_FEATURE;
	USETW(req.wValue, UF_ENDPOINT_HALT);
	req.wIndex[0] = xfer2->pipe->edesc->bEndpointAddress;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);

	/* double check the length */

	if (xfer1->length != sizeof(req)) {
	    printf("%s:%d: invalid transfer length, %d bytes!\n",
		   __FUNCTION__, __LINE__, xfer1->length);
	    return;
	}

	/* copy in the transfer */

	if (xfer1->flags & USBD_USE_DMA) {
	    usbd_copy_in(&(xfer1->buf_data), 0, &req, sizeof(req));
	} else {
	    bcopy(&req, xfer1->buffer, sizeof(req));
	}

	usbd_start_hardware(xfer1);
	return;
}

void
usbd_clear_stall_tr_transferred(struct usbd_xfer *xfer1, 
				struct usbd_xfer *xfer2)
{
	mtx_assert(xfer1->priv_mtx, MA_OWNED);
	mtx_assert(xfer2->priv_mtx, MA_OWNED);

	mtx_lock(xfer2->usb_mtx);

	/* 
	 * clear any stall and make sure 
	 * that DATA0 toggle will be 
	 * used next:
	 */

	xfer2->pipe->clearstall = 0;
	xfer2->pipe->toggle_next = 0;

	mtx_unlock(xfer2->usb_mtx);

	return;
}

void
usbd_clearstall_callback(struct usbd_xfer *xfer)
{
	USBD_CHECK_STATUS(xfer);

 tr_setup:
	usbd_clear_stall_tr_setup(xfer, xfer->priv_sc);
	return;

 tr_transferred:
 tr_error:
	PRINTFN(3,("xfer=%p\n", xfer));

	/* NOTE: some devices reject this command,
	 * so ignore a STALL
	 */
	usbd_clear_stall_tr_transferred(xfer, xfer->priv_sc);

	usbd_start_hardware(xfer->priv_sc);
	return;
}

/* clearstall config:
 *
 *	.type = UE_CONTROL,
 *	.endpoint = 0,
 *	.direction = -1,
 *	.timeout = USBD_DEFAULT_TIMEOUT,
 *	.flags = 0,
 *	.bufsize = sizeof(usb_device_request_t),
 *	.callback = &usbd_clearstall_callback,
 */

/*
 * called from keyboard driver when in polling mode
 */
void
usbd_do_poll(struct usbd_device *udev)
{
	(udev->bus->methods->do_poll)(udev->bus);
	return;
}

void
usbd_set_polling(struct usbd_device *udev, int on)
{
	if(on)
	{
		udev->bus->use_polling++;
	}
	else
	{
		udev->bus->use_polling--;
	}

	/* make sure there is nothing pending to do */
	if(udev->bus->use_polling)
	{
		(udev->bus->methods->do_poll)(udev->bus);
	}
	return;
}

/*
 * usbd_ratecheck() can limit the number of error messages that occurs.
 * When a device is unplugged it may take up to 0.25s for the hub driver
 * to notice it.  If the driver continuosly tries to do I/O operations
 * this can generate a large number of messages.
 */
int
usbd_ratecheck(struct timeval *last)
{
	if(last->tv_sec == time_second)
	{
		return (0);
	}
	last->tv_sec = time_second;
	return (1);
}

/*
 * Search for a vendor/product pair in an array.  The item size is
 * given as an argument.
 */
const struct usb_devno *
usb_match_device(const struct usb_devno *tbl, u_int nentries, u_int size,
		 u_int16_t vendor, u_int16_t product)
{
	while(nentries-- > 0)
	{
		if((tbl->ud_vendor == vendor) &&
		   ((tbl->ud_product == product) ||
		    (tbl->ud_product == USB_PRODUCT_ANY)))
		{
			return (tbl);
		}
		tbl = (const struct usb_devno *)
		  (((const u_int8_t *)tbl) + size);
	}
	return (NULL);
}

int
usbd_driver_load(struct module *mod, int what, void *arg)
{
	/* XXX should implement something like a 
	 * function that removes all generic devices
	 */

 	return (0);
}
