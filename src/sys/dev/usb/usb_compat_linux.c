/*
 * Copyright (c) 2007 Luigi Rizzo - Universita` di Pisa
 * Copyright (c) 2007 Hans Petter Selasky. All rights reserved.
 *
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/usb_compat_linux.c $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/endian.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include <dev/usb/usb_compat_linux.h>

struct usb_linux_driver_list usb_linux_driver_list;

struct usb_linux_softc {
    LIST_ENTRY(usb_linux_softc) sc_attached_list;

    device_t sc_fbsd_dev;
    struct usbd_device *sc_fbsd_udev;
    struct usb_interface *sc_ui;
    struct usb_driver *sc_udrv;
};

/* prototypes */
static device_probe_t usb_linux_probe;
static device_attach_t usb_linux_attach;
static device_detach_t usb_linux_detach;
static device_suspend_t usb_linux_suspend;
static device_resume_t usb_linux_resume;
static device_shutdown_t usb_linux_shutdown;

static usbd_callback_t usb_linux_isoc_callback;
static usbd_callback_t usb_linux_non_isoc_callback;

static usb_complete_t usb_linux_wait_complete;

static uint16_t usb_max_isoc_frames(struct usb_device *dev);
static int32_t usb_start_wait_urb(struct urb *urb, uint32_t timeout, uint32_t *p_actlen);
static const struct usb_device_id * usb_linux_lookup_id(struct usb_driver *udrv, struct usb_attach_arg *uaa);
static struct usb_driver * usb_linux_get_usb_driver(struct usb_linux_softc *sc);
static struct usb_device * usb_linux_create_usb_device(struct usbd_device *udev, device_t dev);
static void usb_linux_cleanup_interface(struct usb_device *dev, struct usb_interface *iface);
static void usb_linux_complete(struct usbd_xfer *xfer);

/*------------------------------------------------------------------------*
 * FreeBSD USB interface
 *------------------------------------------------------------------------*/

static LIST_HEAD(,usb_linux_softc) usb_linux_attached_list;

static device_method_t usb_linux_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, usb_linux_probe),
	DEVMETHOD(device_attach, usb_linux_attach),
	DEVMETHOD(device_detach, usb_linux_detach),
	DEVMETHOD(device_suspend, usb_linux_suspend),
	DEVMETHOD(device_resume, usb_linux_resume),
	DEVMETHOD(device_shutdown, usb_linux_shutdown),

	{ 0, 0 }
};

static driver_t usb_linux_driver = {
	.name	 = "usb_linux",
	.methods = usb_linux_methods,
	.size	 = sizeof(struct usb_linux_softc),
};

static devclass_t usb_linux_devclass;

DRIVER_MODULE(usb_linux, uhub, usb_linux_driver, usb_linux_devclass, usbd_driver_load, 0);
MODULE_DEPEND(usb_linux, usb, 1, 1, 1);

static const struct usb_device_id *
usb_linux_lookup_id(struct usb_driver *udrv, struct usb_attach_arg *uaa)
{
	const struct usb_device_id *id;
	usb_interface_descriptor_t *idesc;
	usb_device_descriptor_t *dd;

	id = udrv->id_table;
	if (id == NULL) {
		goto done;
	}

	idesc = usbd_get_interface_descriptor(uaa->iface);
	if (idesc == NULL) {
		goto done;
	}

	dd = &(uaa->device->ddesc);

	for ( ; id->match_flags; id++) {

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
	      (id->idVendor != uaa->vendor)) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_PRODUCT) &&
	      (id->idProduct != uaa->product)) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_LO) &&
	      (id->bcdDevice_lo > uaa->release)) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_HI) &&
	      (id->bcdDevice_hi < uaa->release)) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_CLASS) &&
	      (id->bDeviceClass != dd->bDeviceClass)) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_SUBCLASS) &&
	      (id->bDeviceSubClass!= dd->bDeviceSubClass)) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_PROTOCOL) &&
	      (id->bDeviceProtocol != dd->bDeviceProtocol)) {
		continue;
	  }

	  if ((dd->bDeviceClass == 0xFF) &&
	      !(id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
	      (id->match_flags & (USB_DEVICE_ID_MATCH_INT_CLASS |
				  USB_DEVICE_ID_MATCH_INT_SUBCLASS |
				  USB_DEVICE_ID_MATCH_INT_PROTOCOL))) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_CLASS) &&
	      (id->bInterfaceClass != idesc->bInterfaceClass)) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_SUBCLASS) &&
	      (id->bInterfaceSubClass != idesc->bInterfaceSubClass)) {
		continue;
	  }

	  if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_PROTOCOL) &&
	      (id->bInterfaceProtocol != idesc->bInterfaceProtocol)) {
		continue;
	  }

	  /* we found a match! */
	  return id;
	}

 done:
	return NULL;
}

static int
usb_linux_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct usb_driver *udrv;
	int err = UMATCH_NONE;

	if (uaa->iface == NULL) {
		return UMATCH_NONE;
	}

	mtx_lock(&usb_global_lock);
	LIST_FOREACH(udrv, &usb_linux_driver_list, linux_driver_list)
	{
		if (usb_linux_lookup_id(udrv, uaa))
		{
			err = 0;
			break;
		}
	}
	mtx_unlock(&usb_global_lock);

	return err;
}

static struct usb_driver *
usb_linux_get_usb_driver(struct usb_linux_softc *sc)
{
	struct usb_driver *udrv;
	mtx_lock(&usb_global_lock);
	udrv = sc->sc_udrv;
	mtx_unlock(&usb_global_lock);
	return udrv;
}

static int
usb_linux_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct usb_linux_softc *sc = device_get_softc(dev);
	struct usb_driver *udrv;
	struct usb_device *p_dev;
	const struct usb_device_id *id = NULL;

	if (sc == NULL) {
		return ENOMEM;
	}

	mtx_lock(&usb_global_lock);
	LIST_FOREACH(udrv, &usb_linux_driver_list, linux_driver_list)
	{
		id = usb_linux_lookup_id(udrv, uaa);
		if (id) break;
	}
	mtx_unlock(&usb_global_lock);

	if (id == NULL) {
		return ENXIO;
	}

	/* Save some memory and only create 
	 * the Linux compat structure when needed:
	 */
	p_dev = uaa->device->linux_dev;
	if (p_dev == NULL) {
	    p_dev = usb_linux_create_usb_device(uaa->device, dev);
	    if (p_dev == NULL) {
		return ENOMEM;
	    }
	    uaa->device->linux_dev = p_dev;
	}

	usbd_set_desc(dev, uaa->device);

	sc->sc_fbsd_udev = uaa->device;
	sc->sc_fbsd_dev = dev;
	sc->sc_udrv = udrv;
	sc->sc_ui = usb_ifnum_to_if(p_dev, uaa->iface->idesc->bInterfaceNumber);
	if (sc->sc_ui == NULL) {
	    return EINVAL;
	}

	if (udrv->probe) {
	    if ((udrv->probe)(sc->sc_ui, id)) {
		return ENXIO;
	    }
	}

	mtx_lock(&usb_global_lock);
	LIST_INSERT_HEAD(&usb_linux_attached_list, sc, sc_attached_list);
	mtx_unlock(&usb_global_lock);

	/* success */
	return 0;
}

static int 
usb_linux_detach(device_t dev)
{
	struct usb_linux_softc *sc = device_get_softc(dev);
	struct usb_driver *udrv = NULL;

	mtx_lock(&usb_global_lock);
	if (sc->sc_attached_list.le_prev) {
	    LIST_REMOVE(sc, sc_attached_list);
	    sc->sc_attached_list.le_prev = NULL;
	    udrv = sc->sc_udrv;
	    sc->sc_udrv = NULL;
	}
	mtx_unlock(&usb_global_lock);

	if (udrv && udrv->disconnect) {
	    (udrv->disconnect)(sc->sc_ui);
	}

	usb_linux_cleanup_interface(sc->sc_fbsd_udev->linux_dev, sc->sc_ui);
	return 0;
}

static int
usb_linux_suspend(device_t dev)
{
	struct usb_linux_softc *sc = device_get_softc(dev);
	struct usb_driver *udrv = usb_linux_get_usb_driver(sc);
	int err;

	if (udrv && udrv->suspend) {
	    err = (udrv->suspend)(sc->sc_ui, 0);
	}
	return 0;
}

static int
usb_linux_resume(device_t dev)
{
	struct usb_linux_softc *sc = device_get_softc(dev);
	struct usb_driver *udrv = usb_linux_get_usb_driver(sc);
	int err;

	if (udrv && udrv->resume) {
	    err = (udrv->resume)(sc->sc_ui);
	}
	return 0;
}

static int
usb_linux_shutdown(device_t dev)
{
	struct usb_linux_softc *sc = device_get_softc(dev);
	struct usb_driver *udrv = usb_linux_get_usb_driver(sc);

	if (udrv && udrv->shutdown) {
		(udrv->shutdown)(sc->sc_ui);
	}
	return 0;
}

/*------------------------------------------------------------------------*
 * Linux emulation layer
 *------------------------------------------------------------------------*/

static uint16_t
usb_max_isoc_frames(struct usb_device *dev)
{
	return ((usbd_get_speed(dev->bsd_udev) == USB_SPEED_HIGH) ?
		USB_MAX_HIGH_SPEED_ISOC_FRAMES : USB_MAX_FULL_SPEED_ISOC_FRAMES);
}

int32_t
usb_submit_urb(struct urb *urb, gfp_t mem_flags)
{
	struct usb_host_endpoint *uhe;

	if (urb == NULL) {
	    return -EINVAL;
	}

	mtx_assert(&usb_global_lock, MA_OWNED);

	if (urb->pipe == NULL) {
	    return -EINVAL;
	}

	uhe = urb->pipe;

	if (uhe->bsd_xfer[0] ||
	    uhe->bsd_xfer[1])
	{
	    /* we are ready! */

	    TAILQ_INSERT_HEAD(&(uhe->bsd_urb_list), urb, bsd_urb_list);

	    urb->status = -EINPROGRESS;

	    usbd_transfer_start(uhe->bsd_xfer[0]);
	    usbd_transfer_start(uhe->bsd_xfer[1]);
	} else {
	    /* no pipes have been setup yet! */
	    urb->status = -EINVAL;
	    return -EINVAL;
	}
	return 0;
}

int32_t
usb_unlink_urb(struct urb *urb)
{
	struct usb_host_endpoint *uhe;
	uint16_t x;

	if (urb == NULL) {
	    return -EINVAL;
	}

	mtx_assert(&usb_global_lock, MA_OWNED);

	if (urb->pipe == NULL) {
	    return -EINVAL;
	}

	uhe = urb->pipe;

	if (urb->bsd_urb_list.tqe_prev) {

	    /* not started yet, just remove it from the queue */
	    TAILQ_REMOVE(&(uhe->bsd_urb_list), urb, bsd_urb_list);
	    urb->bsd_urb_list.tqe_prev = NULL;
	    urb->status = -ECONNRESET;
	    urb->actual_length = 0;

	    for (x = 0; x < urb->number_of_packets; x++) {
		urb->iso_frame_desc[x].actual_length = 0;
	    }

	    if (urb->complete) {
		(urb->complete)(urb, NULL);
	    }

	} else {

	    if (uhe->bsd_xfer[0] &&
		(uhe->bsd_xfer[0]->priv_fifo == (void *)urb))
	    {
		usbd_transfer_stop(uhe->bsd_xfer[0]);
		usbd_transfer_start(uhe->bsd_xfer[0]);
	    }

	    if (uhe->bsd_xfer[1] &&
		(uhe->bsd_xfer[1]->priv_fifo == (void *)urb))
	    {
		usbd_transfer_stop(uhe->bsd_xfer[1]);
		usbd_transfer_start(uhe->bsd_xfer[1]);
	    }
	}
	return 0;
}

int32_t
usb_clear_halt(struct usb_device *dev, struct usb_host_endpoint *uhe)
{
	struct usbd_config cfg[1];
	struct usbd_pipe *pipe;
	uint8_t type;
	uint8_t addr;

	if (uhe == NULL) return -EINVAL;

	type = uhe->desc.bmAttributes & UE_XFERTYPE;
	addr = uhe->desc.bEndpointAddress;

	bzero(cfg, sizeof(cfg));

	cfg[0].type = type;
	cfg[0].endpoint = addr & UE_ADDR;
	cfg[0].direction = addr & (UE_DIR_OUT|UE_DIR_IN);

	pipe = usbd_get_pipe(dev->bsd_udev, uhe->bsd_iface_index, cfg);
	if (pipe == NULL) return -EINVAL;

	mtx_lock(&(dev->bsd_udev->bus->mtx));
	pipe->clearstall = 0;
	pipe->toggle_next = 0;
	mtx_unlock(&(dev->bsd_udev->bus->mtx));

	return usb_control_msg(dev, &(dev->ep0), 
			       UR_CLEAR_FEATURE, UT_WRITE_ENDPOINT,
			       UF_ENDPOINT_HALT, addr, NULL, 0, 1000);
}

static int32_t
usb_start_wait_urb(struct urb *urb, uint32_t timeout, uint32_t *p_actlen)
{
	int32_t err;

	/* you must have a timeout! */
	if (timeout == 0) {
	    timeout = 1;
	}

	urb->complete = &usb_linux_wait_complete;
	urb->timeout = timeout;
	urb->transfer_flags |= URB_WAIT_WAKEUP;
	urb->transfer_flags &= ~URB_IS_SLEEPING;

	err = usb_submit_urb(urb, 0);
	if (err) goto done;

	/* the URB might have completed before we get
	 * here, so check that by using some flags!
	 */
	while (urb->transfer_flags & URB_WAIT_WAKEUP) {
	    urb->transfer_flags |= URB_IS_SLEEPING;
	    err = mtx_sleep(urb, &usb_global_lock, 0, "USB Linux Wait", 0);
	    urb->transfer_flags &= ~URB_IS_SLEEPING;
	    if (err) goto done;
	}

	err = urb->status;

 done:
	if (err) {
	  *p_actlen = 0;
	} else {
	  *p_actlen = urb->actual_length;
	}
	return err;
}

int32_t
usb_control_msg(struct usb_device *dev, struct usb_host_endpoint *uhe, 
		uint8_t request, uint8_t requesttype,
		uint16_t value, uint16_t index, void *data, uint16_t size, uint32_t timeout)
{
	struct urb *urb;
	usb_device_request_t *req;
	struct usb_host_endpoint *uhe_write;
	struct usb_host_endpoint *uhe_read;
	uint32_t actlen;
	int32_t err;
	uint8_t type;
	uint8_t addr;

	if (uhe == NULL) {
	    return -EINVAL;
	}

	type = (uhe->desc.bmAttributes & UE_XFERTYPE);
	addr = (uhe->desc.bEndpointAddress & UE_ADDR);

	if (type == UE_CONTROL) {
	    uhe_write = NULL;
	    uhe_read = NULL;
	} else {
	    if (type == UE_ISOCHRONOUS) {
		return -EINVAL;
	    }
	    uhe_write = usb_find_host_endpoint(dev, type, addr | UE_DIR_OUT);
	    if (uhe_write == NULL) {
		return -EINVAL;
	    }
	    if (requesttype & UT_READ) {
		uhe_read = usb_find_host_endpoint(dev, type, addr | UE_DIR_IN);
		if (uhe_read == NULL) {
		    return -EINVAL;
		}
	    } else {
		uhe_read = NULL;
	    }
	}

	/* NOTE: we need to allocate real memory
	 * here so that we don't transfer data
	 * to/from the stack!
	 *
	 * 0xFFFF is a magic value.
	 */
	urb = usb_alloc_urb(0xFFFF, size);
	if (urb == NULL) return -ENOMEM;

	urb->dev = dev;

	req = (void *)(urb->setup_packet);

	req->bmRequestType = requesttype;
	req->bRequest = request;
	USETW(req->wValue, value);
	USETW(req->wIndex, index);
	USETW(req->wLength, size);

	if (size && (req->bmRequestType & UT_WRITE)) {
	      /* move the data to a real buffer */
	      bcopy(data, req->bData, size);
	}

	if (type == UE_CONTROL) {
	    urb->pipe = uhe;
	    err = usb_start_wait_urb(urb, timeout, &actlen);
	} else {
	    urb->pipe = uhe_write;
	    urb->transfer_buffer = urb->setup_packet;
	    urb->transfer_buffer_length = sizeof(*req);

	    err = usb_start_wait_urb(urb, 1000, &actlen);
	    if (err) {
		goto done;
	    }

	    if (actlen < sizeof(*req)) {
		err = -EPIPE;
		actlen = 0;
		goto done;
	    }

	    if (size) {
		if (req->bmRequestType & UT_READ) {
		    urb->pipe = uhe_read;
		}
		urb->transfer_buffer = req->bData;
		urb->transfer_buffer_length = size;

		err = usb_start_wait_urb(urb, timeout, &actlen);
		if (err) {
		    goto done;
		}
	    }
	}

 done:
	if (req->bmRequestType & UT_READ) {
	    if (actlen < size) {
		/* we don't like returning random data */
		bzero(((uint8_t *)data) + actlen, size - actlen);
	    }
	    if (actlen) {
		bcopy(req->bData, data, actlen);
	    }
	}

	usb_free_urb(urb);

	return err;
}

int32_t
usb_set_interface(struct usb_device *dev, uint8_t iface_no, uint8_t alt_index)
{
	struct usb_interface *p_ui = usb_ifnum_to_if(dev, iface_no);
	int32_t err;
	if (p_ui == NULL) return -EINVAL;
	if (alt_index >= p_ui->num_altsetting) return -EINVAL;
	usb_linux_cleanup_interface(dev, p_ui);
	err = -usbd_set_config_index(dev->bsd_udev, p_ui->bsd_iface_index, alt_index);
	if (err == 0) {
	    p_ui->cur_altsetting = p_ui->altsetting + alt_index;
	}
	return err;
}

int32_t
usb_setup_endpoint(struct usb_device *dev, struct usb_host_endpoint *uhe, uint32_t bufsize)
{
	struct usbd_config cfg[2];
	uint16_t mfs = usbd_get_max_frame_size((usb_endpoint_descriptor_t *)&(uhe->desc));
	uint8_t type = uhe->desc.bmAttributes & UE_XFERTYPE;
	uint8_t addr = uhe->desc.bEndpointAddress;

	if (uhe->fbsd_buf_size == bufsize) {
	    /* optimize */
	    return 0;
	}

	usbd_transfer_unsetup(uhe->bsd_xfer, 2);

	uhe->fbsd_buf_size = bufsize;

	if (bufsize == 0) {
	    return 0;
	}

	bzero(cfg, sizeof(cfg));

	if (type == UE_ISOCHRONOUS) {

	    /* Isochronous is special */

	    cfg[0].type = type;
	    cfg[0].endpoint = addr & UE_ADDR;
	    cfg[0].direction = addr & (UE_DIR_OUT|UE_DIR_IN);
	    cfg[0].callback = &usb_linux_isoc_callback;
	    cfg[0].bufsize = 0; /* use wMaxPacketSize */
	    cfg[0].frames = usb_max_isoc_frames(dev);
	    cfg[0].flags = (USBD_USE_DMA|USBD_SHORT_XFER_OK);

	    bcopy(cfg + 0, cfg + 1, sizeof(*cfg));

	    if (usbd_transfer_setup(dev->bsd_udev, uhe->bsd_iface_index,
				    uhe->bsd_xfer, cfg, 2, uhe, 
				    &usb_global_lock))
	    {
		return -EINVAL;
	    }
	} else {
	    if (bufsize > (1 << 22)) {
		/* limit buffer size */
		bufsize = (1 << 22);
	    }

	    /* we need enough room for the control header */
	    if (bufsize < sizeof(usb_device_request_t)) {
		bufsize = sizeof(usb_device_request_t);
	    }

	    if (bufsize < mfs) {
		/* we need to be able to hold at least one frame! */
		bufsize = mfs;
	    }

	    cfg[0].type = type;
	    cfg[0].endpoint = addr & UE_ADDR;
	    cfg[0].direction = addr & (UE_DIR_OUT|UE_DIR_IN);
	    cfg[0].callback = &usb_linux_non_isoc_callback;
	    cfg[0].bufsize = bufsize;
	    cfg[0].flags = (USBD_USE_DMA|USBD_SHORT_XFER_OK);

	    if (usbd_transfer_setup(dev->bsd_udev, uhe->bsd_iface_index,
				    uhe->bsd_xfer, cfg, 1, uhe, 
				    &usb_global_lock))
	    {
		return -EINVAL;
	    }
	}
	return 0;
}

static struct usb_device *
usb_linux_create_usb_device(struct usbd_device *udev, device_t dev)
{
	usb_config_descriptor_t *cd = usbd_get_config_descriptor(udev);
	usb_descriptor_t *desc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	usb_device_descriptor_t *dd;
	struct usb_device *p_ud = NULL;
	struct usb_interface *p_ui = NULL;
	struct usb_host_interface *p_uhi = NULL;
	struct usb_host_endpoint *p_uhe = NULL;
	uint32_t size;
	uint16_t niface_total;
	uint16_t nedesc;
	uint16_t iface_no_curr;
	uint16_t iface_index;
	uint8_t pass;
	uint8_t iface_no;

	for (pass = 0; pass < 2; pass++) {

	    iface_no_curr = 0-1;
	    niface_total = 0;
	    iface_index = 0;
	    nedesc = 0;
	    desc = NULL;

	    while ((desc = usbd_desc_foreach(cd, desc))) {

		switch (desc->bDescriptorType) {
		case UDESC_DEVICE:
		    dd = (void *)desc;
		    if (dd->bLength < sizeof(*dd)) break;
		    if (p_ud) {
			bcopy(dd, &(p_ud->descriptor), sizeof(p_ud->descriptor));
		    }
		    break;

		case UDESC_ENDPOINT:
		    ed = (void *)desc;
		    if ((ed->bLength < sizeof(*ed)) || 
			(iface_index == 0)) break;
		    if (p_uhe) {
			bcopy(ed, &(p_uhe->desc), sizeof(p_uhe->desc));
			p_uhe->bsd_iface_index = iface_index-1;
			p_uhe++;
		    }
		    if (p_uhi) {
			(p_uhi-1)->desc.bNumEndpoints++;
		    }
		    nedesc ++;
		    break;

		case UDESC_INTERFACE:
		    id = (void *)desc;
		    if (id->bLength < sizeof(*id)) break;
		    if (p_uhi) {
			bcopy(id, &(p_uhi->desc), sizeof(p_uhi->desc));
			p_uhi->desc.bNumEndpoints = 0;
			p_uhi->endpoint = p_uhe;
			p_uhi->string = "";
			p_uhi++;
		    }
		    iface_no = id->bInterfaceNumber;
		    niface_total ++;
		    if (iface_no_curr != iface_no) {
			if (p_ui) {
			    p_ui->altsetting = p_uhi-1;
			    p_ui->cur_altsetting = p_uhi-1;
			    p_ui->num_altsetting = 1;
			    p_ui->bsd_iface_index = iface_index;
			    p_ui->linux_udev = p_ud;
			    p_ui++;
			}
			iface_no_curr = iface_no;
			iface_index ++;
		    } else {
			if (p_ui) {
			    (p_ui-1)->num_altsetting ++;
			}
		    }
		    break;

		default:
		    break;
		}
	    }

	    if (pass == 0) {

		size = ((sizeof(*p_ud) * 1) +
			(sizeof(*p_uhe) * nedesc) +
			(sizeof(*p_ui) * iface_index) +
			(sizeof(*p_uhi) * niface_total));

		p_ud = malloc(size, M_USBDEV, M_WAITOK | M_ZERO);
		if (p_ud == NULL) {
		    goto done;
		}
		p_uhe = (void *)(p_ud + 1);
		p_ui = (void *)(p_uhe + nedesc);
		p_uhi = (void *)(p_ui + iface_index);

		p_ud->product = "";
		p_ud->manufacturer = "";
		p_ud->serial = "";
		p_ud->speed = usbd_get_speed(udev);
		p_ud->bsd_udev = udev;
		p_ud->bsd_iface_start = p_ui;
		p_ud->bsd_iface_end = p_ui + iface_index;
		p_ud->bsd_endpoint_start = p_uhe;
		p_ud->bsd_endpoint_end = p_uhe + nedesc;
		p_ud->devnum = device_get_unit(dev);

		bcopy(udev->default_pipe.edesc, &(p_ud->ep0.desc),
		      sizeof(p_ud->ep0.desc));

		if (usb_setup_endpoint(p_ud, &(p_ud->ep0), 1024 /* bytes */)) {
		    usb_linux_free_usb_device(p_ud);
		    p_ud = NULL;
		    goto done;
		}
	    }
	}
  done:
	return p_ud;
}

struct urb *
usb_alloc_urb(uint16_t iso_packets, gfp_t mem_flags)
{
	struct urb *urb;
	uint32_t size;

	if (iso_packets == 0xFFFF) {
	    size = sizeof(*urb) + sizeof(usb_device_request_t) + mem_flags;
	} else {
	    size = sizeof(*urb) + (iso_packets * sizeof(urb->iso_frame_desc[0]));
	}

	urb = malloc(size, M_USBDEV, M_WAITOK | M_ZERO);
	if (urb) {
	    if (iso_packets == 0xFFFF) {
		urb->setup_packet = (void *)(urb + 1);
		urb->transfer_buffer = (void *)(urb->setup_packet + 
						sizeof(usb_device_request_t));
		urb->transfer_buffer_length = mem_flags;
	    } else {
		urb->number_of_packets = iso_packets;
	    }
	}
	return urb;
}

struct usb_host_endpoint *
usb_find_host_endpoint(struct usb_device *dev, uint8_t type, uint8_t ep)
{
	struct usb_host_endpoint *uhe;
	struct usb_host_endpoint *uhe_end;
	struct usb_host_interface *uhi;
	struct usb_interface *ui;
	uint8_t ea;
	uint8_t at;
	uint8_t mask;

	if (dev == NULL) {
	    return NULL;
	}

	if (type == UE_CONTROL) {
	  mask = UE_ADDR;
	} else {
	  mask = (UE_DIR_IN|UE_DIR_OUT|UE_ADDR);
	}

	ep &= mask;

	for (ui = dev->bsd_iface_start;
	     ui != dev->bsd_iface_end;
	     ui++) {
	    uhi = ui->cur_altsetting;
	    if (uhi) {
		uhe_end = uhi->endpoint + uhi->desc.bNumEndpoints;
		for (uhe = uhi->endpoint;
		     uhe != uhe_end;
		     uhe++) {

		    ea = uhe->desc.bEndpointAddress;
		    at = uhe->desc.bmAttributes;

		    if (((ea & mask) == ep) &&
			((at & UE_XFERTYPE) == type)) {
			return uhe;
		    }
		}
	    }
	}

	if ((type == UE_CONTROL) && ((ep & UE_ADDR) == 0)) {
	    return (&(dev->ep0));
	}
	return NULL;
}

struct usb_host_interface *
usb_altnum_to_altsetting(const struct usb_interface *intf, uint8_t alt_index)
{
	if (alt_index >= intf->num_altsetting) {
		return NULL;
	}
	return (intf->altsetting + alt_index);
}

struct usb_interface *
usb_ifnum_to_if(struct usb_device *dev, uint8_t iface_no)
{
	struct usb_interface * p_ui;

	for (p_ui = dev->bsd_iface_start;
	     p_ui != dev->bsd_iface_end;
	     p_ui++) {
	  if ((p_ui->num_altsetting > 0) &&
	      (p_ui->altsetting->desc.bInterfaceNumber == iface_no)) {
	      return p_ui;
	  }
	}
	return NULL;
}

void *
usb_buffer_alloc(struct usb_device *dev, uint32_t size, gfp_t mem_flags, uint8_t *dma_addr)
{
	return malloc(size, M_USBDEV, M_WAITOK | M_ZERO);
}

void *
usb_get_intfdata(struct usb_interface *intf)
{
	return intf->bsd_priv_sc;
}

void
usb_linux_register(void *arg)
{
	struct usb_driver *drv = arg;
	mtx_lock(&usb_global_lock);
	LIST_INSERT_HEAD(&usb_linux_driver_list, drv, linux_driver_list);
	mtx_unlock(&usb_global_lock);

	usb_needs_probe_and_attach();
	return;
}

void
usb_linux_deregister(void *arg)
{
	struct usb_driver *drv = arg;
	struct usb_linux_softc *sc;

 repeat:
	mtx_lock(&usb_global_lock);
	LIST_FOREACH(sc, &usb_linux_attached_list, sc_attached_list)
	{
	    if (sc->sc_udrv == drv)
	    {
		mtx_unlock(&usb_global_lock);
		device_detach(sc->sc_fbsd_dev);
		goto repeat;
	    }
	}
	LIST_REMOVE(drv, linux_driver_list);
	mtx_unlock(&usb_global_lock);
	return;
}

void
usb_linux_free_usb_device(struct usb_device *dev)
{
	struct usb_host_endpoint *uhe;
	struct usb_host_endpoint *uhe_end;
	int32_t err;

	uhe = dev->bsd_endpoint_start;
	uhe_end = dev->bsd_endpoint_end;
	while (uhe != uhe_end) {
	    err = usb_setup_endpoint(dev, uhe, 0);
	    uhe++;
	}
	err = usb_setup_endpoint(dev, &(dev->ep0), 0);
	free(dev, M_USBDEV);
	return;
}

void
usb_buffer_free(struct usb_device *dev, uint32_t size, 
		void *addr, uint8_t dma_addr)
{
	free(addr, M_USBDEV);
	return;
}

void
usb_free_urb(struct urb *urb)
{
	if (urb == NULL) {
	    return;
	}

	/* make sure that the current URB is not active */
	usb_kill_urb(urb);

	/* just free it */
	free(urb, M_USBDEV);
	return;
}

void
usb_init_urb(struct urb *urb)
{
	if (urb == NULL) {
	    return;
	}
	bzero(urb, sizeof(*urb));
	return;
}

void
usb_kill_urb(struct urb *urb)
{
	int32_t err;
	err = usb_unlink_urb(urb);
	return;
}

void
usb_set_intfdata(struct usb_interface *intf, void *data)
{
	intf->bsd_priv_sc = data;
	return;
}

static void
usb_linux_cleanup_interface(struct usb_device *dev, struct usb_interface *iface)
{
	struct usb_host_interface *uhi;
	struct usb_host_interface *uhi_end;
	struct usb_host_endpoint *uhe;
	struct usb_host_endpoint *uhe_end;
	int32_t err;

	uhi = iface->altsetting;
	uhi_end = iface->altsetting + iface->num_altsetting;
	while (uhi != uhi_end) {
	    uhe = uhi->endpoint;
	    uhe_end = uhi->endpoint + uhi->desc.bNumEndpoints;
	    while (uhe != uhe_end) {
		err = usb_setup_endpoint(dev, uhe, 0);
		uhe++;
	    }
	    uhi++;
	}
	return;
}

static void
usb_linux_wait_complete(struct urb *urb, struct pt_regs *pt_regs)
{
	if (urb->transfer_flags & URB_IS_SLEEPING)
	{
		wakeup(urb);
	}
	urb->transfer_flags &= ~URB_WAIT_WAKEUP;
	return;
}

static void
usb_linux_complete(struct usbd_xfer *xfer)
{
	struct urb *urb;
	urb = xfer->priv_fifo;
	xfer->priv_fifo = NULL;
	if (urb->complete) {
	    (urb->complete)(urb, NULL);
	}
	return;
}

static void
usb_linux_isoc_callback(struct usbd_xfer *xfer)
{
	uint32_t max_frame = xfer->max_frame_size;
	uint32_t offset;
	uint16_t x;
	struct urb *urb = xfer->priv_fifo;
	struct usb_host_endpoint *uhe = xfer->priv_sc;
	struct usb_iso_packet_descriptor *uipd;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
	    urb->status = -ECONNRESET;
	} else {
	    urb->status = -EPIPE; /* stalled */
	}

	/* just in case: */
	urb->actual_length = 0;

	for (x = 0; x < urb->number_of_packets; x++) {
	    urb->iso_frame_desc[x].actual_length = 0;
	}

	/* call callback */
	usb_linux_complete(xfer);

	if (xfer->error == USBD_CANCELLED) {
	    /* we need to return in this case */
	    return;
	}

	goto tr_setup;

 tr_transferred:

	if (urb->bsd_isread) {

	    offset = 0;

	    for (x = 0; x < urb->number_of_packets; x++) {
		uipd = urb->iso_frame_desc + x;
		uipd->actual_length = xfer->frlengths[x];
		uipd->status = 0;
		usbd_copy_out(&(xfer->buf_data), offset, 
			      ((uint8_t *)(urb->transfer_buffer)) + uipd->offset,
			      uipd->actual_length);
		offset += max_frame;
	    }
	} else {
	    for (x = 0; x < urb->number_of_packets; x++) {
		uipd = urb->iso_frame_desc + x;
		uipd->actual_length = xfer->frlengths[x];
		uipd->status = 0;
	    }
	}

	urb->actual_length = xfer->actlen;

	/* check for short transfer */
	if (xfer->actlen < xfer->length) {
	    /* short transfer */
	    if (urb->transfer_flags & URB_SHORT_NOT_OK) {
		urb->status = -EPIPE; /* XXX should be EREMOTEIO */
	    } else {
		urb->status = 0;
	    }
	} else {
	    /* success */
	    urb->status = 0;
	}

	/* call callback */
	usb_linux_complete(xfer);

 tr_setup:

	if (xfer->priv_fifo == NULL) {

	    /* get next transfer */
	    urb = TAILQ_FIRST(&(uhe->bsd_urb_list));
	    if (urb == NULL) {
		/* nothing to do */
		return;
	    }

	    TAILQ_REMOVE(&(uhe->bsd_urb_list), urb, bsd_urb_list);
	    urb->bsd_urb_list.tqe_prev = NULL;

	    x = usb_max_isoc_frames(urb->dev);
	    if (urb->number_of_packets > x) {
		/* XXX simply truncate the transfer */
		urb->number_of_packets = x;
	    }

	} else {

	    /* already got a transfer (should not happen) */
	    urb = xfer->priv_fifo;
	}

	urb->bsd_isread = (uhe->desc.bEndpointAddress & UE_DIR_IN) ? 1 : 0;

	if (!(urb->bsd_isread)) {

	    offset = 0;

	    for (x = 0; x < urb->number_of_packets; x++) {
		uipd = urb->iso_frame_desc + x;
		xfer->frlengths[x] = uipd->length;
		usbd_copy_in(&(xfer->buf_data), offset,
			     ((uint8_t *)(urb->transfer_buffer)) + uipd->offset,
			     uipd->length);
		offset += uipd->length;
	    }
	} else {

	    offset = urb->number_of_packets * max_frame;

	    for (x = 0; x < urb->number_of_packets; x++) {
		uipd = urb->iso_frame_desc + x;
		xfer->frlengths[x] = max_frame;
	    }
	}

	xfer->priv_fifo = urb;
	xfer->flags &= ~USBD_FORCE_SHORT_XFER;
	xfer->timeout = urb->timeout;
	xfer->nframes = urb->number_of_packets;
	xfer->length = offset; /* not really used */
	usbd_start_hardware(xfer);
	return;
}

static void
usb_linux_non_isoc_callback(struct usbd_xfer *xfer)
{
	struct urb *urb = xfer->priv_fifo;
	struct usb_host_endpoint *uhe = xfer->priv_sc;
	uint32_t max_bulk = (uhe->fbsd_buf_size - 
			     (uhe->fbsd_buf_size % xfer->max_packet_size));

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
	    urb->status = -ECONNRESET;
	} else {
	    urb->status = -EPIPE;
	}

	/* just in case: */
	urb->actual_length = 0;

	/* call callback */
	usb_linux_complete(xfer);

	if (xfer->error == USBD_CANCELLED) {
	    /* we need to return in this case */
	    return;
	}
	goto tr_setup;

 tr_transferred:

	if (urb->bsd_isread) {
	    usbd_copy_out(&(xfer->buf_data), 0,
			  urb->bsd_data_ptr, xfer->actlen);
	}

	urb->bsd_length_rem -= xfer->actlen;
	urb->bsd_data_ptr += xfer->actlen;
	urb->actual_length += xfer->actlen;

	/* check for short packet */
	if (xfer->actlen < xfer->length) {
	    urb->bsd_length_rem = 0;

	    /* short transfer */
	    if (urb->transfer_flags & URB_SHORT_NOT_OK) {
		urb->status = -EPIPE;
	    } else {
		urb->status = 0;
	    }
	} else {

	    /* check remainder */
	    if (urb->bsd_length_rem) {
		goto setup_bulk;
	    }

	    /* success */
	    urb->status = 0;
	}

	/* check actual length */
	if (urb->actual_length > urb->transfer_buffer_length) {
	    /* premature end of a control transfer */
	    urb->actual_length = 0;
	}

	/* call callback */
	usb_linux_complete(xfer);

 tr_setup:

	/* get next transfer */
	urb = TAILQ_FIRST(&(uhe->bsd_urb_list));
	if (urb == NULL) {
	    /* nothing to do */
	    return;
	}

	TAILQ_REMOVE(&(uhe->bsd_urb_list), urb, bsd_urb_list);
	urb->bsd_urb_list.tqe_prev = NULL;

	xfer->priv_fifo = urb;
	xfer->flags &= ~USBD_FORCE_SHORT_XFER;
	xfer->timeout = urb->timeout;

	if ((uhe->desc.bmAttributes & UE_XFERTYPE) == UE_CONTROL) {
	    /* we transfer the header first, then the data */
	    usbd_copy_in(&(xfer->buf_data), 0, 
			 urb->setup_packet, sizeof(usb_device_request_t));
	    xfer->length = sizeof(usb_device_request_t);

	    urb->bsd_length_rem = xfer->length + urb->transfer_buffer_length;
	    urb->bsd_data_ptr = ((uint8_t *)(urb->transfer_buffer)) - xfer->length;
	    urb->actual_length = -xfer->length;
	    urb->bsd_isread = (((uint8_t *)(urb->setup_packet))[0] & UT_READ) ? 1 : 0;

	    usbd_start_hardware(xfer);
	    return;
	}

	urb->bsd_length_rem = urb->transfer_buffer_length;
	urb->bsd_data_ptr = urb->transfer_buffer;
	urb->actual_length = 0;
	urb->bsd_isread = (uhe->desc.bEndpointAddress & UE_DIR_IN) ? 1 : 0;

 setup_bulk:
	if (max_bulk > urb->bsd_length_rem) {
	    max_bulk = urb->bsd_length_rem;
	}

	if (max_bulk == urb->bsd_length_rem) {
	    if (urb->transfer_flags & URB_ZERO_PACKET) {
		xfer->flags |= USBD_FORCE_SHORT_XFER;
	    }
	}

	if (!(urb->bsd_isread)) {
	    usbd_copy_in(&(xfer->buf_data), 0,
			 urb->bsd_data_ptr, max_bulk);
	}

	xfer->length = max_bulk;
	usbd_start_hardware(xfer);
	return;
}