/*      $NetBSD: usb_subr.c,v 1.99 2002/07/11 21:14:34 augustss Exp $   */

/* Also already have from NetBSD:
 *      $NetBSD: usb_subr.c,v 1.102 2003/01/01 16:21:50 augustss Exp $
 *      $NetBSD: usb_subr.c,v 1.103 2003/01/10 11:19:13 augustss Exp $
 *      $NetBSD: usb_subr.c,v 1.111 2004/03/15 10:35:04 augustss Exp $
 *      $NetBSD: usb_subr.c,v 1.114 2004/06/23 02:30:52 mycroft Exp $
 *      $NetBSD: usb_subr.c,v 1.115 2004/06/23 05:23:19 mycroft Exp $
 *      $NetBSD: usb_subr.c,v 1.116 2004/06/23 06:27:54 mycroft Exp $
 *      $NetBSD: usb_subr.c,v 1.119 2004/10/23 13:26:33 augustss Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/usb_subr.c,v 1.90 2007/05/08 03:25:05 kevlo Exp $");

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
#include <sys/endian.h>
#include <sys/queue.h> /* LIST_XXX() */
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/kthread.h>
#include <sys/unistd.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_hid.h>
#include "usbdevs.h"
#include <dev/usb/usb_quirks.h>

#ifdef USBVERBOSE
/*
 * Descriptions of of known vendors and devices ("products").
 */
struct usb_knowndev {
	u_int16_t		vendor;
	u_int16_t		product;
	int			flags;
	char			*vendorname, *productname;
};
#define	USB_KNOWNDEV_NOPROD	0x01		/* match on vendor only */

#include "usbdevs_data.h"
#endif /* USBVERBOSE */

static void
usbd_trim_spaces(char *p)
{
	char *q;
	char *e;

	if(p == NULL)
		return;
	q = e = p;
	while (*q == ' ')	/* skip leading spaces */
		q++;
	while ((*p = *q++))	/* copy string */
		if (*p++ != ' ') /* remember last non-space */
			e = p;
	*e = 0;			/* kill trailing spaces */
	return;
}

static void
usbd_devinfo_vp(struct usbd_device *udev, char *v, char *p, uint16_t v_len, 
		uint16_t p_len, uint8_t usedev)
{
	usb_device_descriptor_t *udd = &udev->ddesc;
	char *vendor;
	char *product;
#ifdef USBVERBOSE
	const struct usb_knowndev *kdp;
#endif
	uint16_t vendor_id;
	uint16_t product_id;
	usbd_status err;

	v[0] = 0;
	p[0] = 0;

	if(udev == NULL) {
		return;
	}

	if (usedev)
	{
		err = usbreq_get_string_any
		  (udev, udd->iManufacturer, v, v_len);

		usbd_trim_spaces(v);

		err = usbreq_get_string_any
		  (udev, udd->iProduct, p, p_len);

		usbd_trim_spaces(p);
	}

	vendor = (*v) ? v : NULL;
	product = (*p) ? p : NULL;

	vendor_id = UGETW(udd->idVendor);
	product_id = UGETW(udd->idProduct);

#ifdef USBVERBOSE
	if (vendor == NULL || product == NULL) {

		for(kdp = usb_knowndevs;
		    kdp->vendorname != NULL;
		    kdp++) {
			if (kdp->vendor == vendor_id &&
			   (kdp->product == product_id ||
			    (kdp->flags & USB_KNOWNDEV_NOPROD) != 0))
				break;
		}
		if (kdp->vendorname != NULL) {
			if (vendor == NULL)
			    vendor = kdp->vendorname;
			if (product == NULL)
			    product = (kdp->flags & USB_KNOWNDEV_NOPROD) == 0 ?
			        kdp->productname : NULL;
		}
	}
#endif
	if (vendor && *vendor) {
	    if (v != vendor) {
	        strlcpy(v, vendor, v_len);
	    }
	} else {
	    snprintf(v, v_len, "vendor 0x%04x", vendor_id);
	}

	if (product && *product) {
	    if (p != product) {
	        strlcpy(p, product, p_len);
	    }
	} else {
	    snprintf(p, p_len, "product 0x%04x", product_id);
	}
	return;
}

static void
usbd_printBCD(char *p, uint16_t p_len, uint16_t bcd)
{
	int len;
	len = snprintf(p, p_len, "%x.%02x", bcd >> 8, bcd & 0xff);
	return;
}

void
usbd_devinfo(struct usbd_device *udev, int showclass, 
	     char *dst_ptr, u_int16_t dst_len)
{
	usb_device_descriptor_t *udd = &udev->ddesc;
	char vendor[USB_MAX_STRING_LEN];
	char product[USB_MAX_STRING_LEN];
	u_int16_t bcdDevice, bcdUSB;

	usbd_devinfo_vp(udev, vendor, product, 
			sizeof(vendor), sizeof(product), 1);

	bcdUSB = UGETW(udd->bcdUSB);
	bcdDevice = UGETW(udd->bcdDevice);

	if(showclass)
	{
	    snprintf(dst_ptr, dst_len, "%s %s, class %d/%d, rev %x.%02x/"
		     "%x.%02x, addr %d", vendor, product,
		     udd->bDeviceClass, udd->bDeviceSubClass,
		     (bcdUSB >> 8), bcdUSB & 0xFF,
		     (bcdDevice >> 8), bcdDevice & 0xFF,
		     udev->address);
	}
	else
	{
	    snprintf(dst_ptr, dst_len, "%s %s, rev %x.%02x/"
		     "%x.%02x, addr %d", vendor, product,
		     (bcdUSB >> 8), bcdUSB & 0xFF,
		     (bcdDevice >> 8), bcdDevice & 0xFF,
		     udev->address);
	}
	return;
}

const char * 
usbd_errstr(usbd_status err)
{
	static const char * const
	  MAKE_TABLE(USBD_STATUS,DESC,[]);

	return (err < N_USBD_STATUS) ?
	  USBD_STATUS_DESC[err] : "unknown error!";
}

/* Delay for a certain number of ms */
void
usb_delay_ms(struct usbd_bus *bus, uint32_t ms)
{
	/* Wait at least two clock ticks so we know the time has passed. */
	if (cold)
		DELAY((ms+1) * 1000);
	else
#if (__FreeBSD_version >= 700031)
		pause("usbdly", (((ms*hz)+999)/1000) + 1);
#else
		tsleep(&ms, PRIBIO, "usbdly", (((ms*hz)+999)/1000) + 1);
#endif
}

/* Delay given a device handle. */
void
usbd_delay_ms(struct usbd_device *udev, uint32_t ms)
{
	usb_delay_ms(udev->bus, ms);
}

#define ADD_BYTES(ptr,len) ((void *)(((u_int8_t *)(ptr)) + (len)))

usb_descriptor_t *
usbd_desc_foreach(usb_config_descriptor_t *cd, usb_descriptor_t *desc)
{
	void *end;

	if (cd == NULL) {
	    return NULL;
	}

	end = ADD_BYTES(cd, UGETW(cd->wTotalLength));

	if (desc == NULL) {
	    desc = ADD_BYTES(cd, 0);
	} else {
	    desc = ADD_BYTES(desc, desc->bLength);
	}
	return (((((void *)desc) >= ((void *)cd)) &&
		 (((void *)desc) < end) &&
		 (ADD_BYTES(desc,desc->bLength) >= ((void *)cd)) &&
		 (ADD_BYTES(desc,desc->bLength) <= end) &&
		 (desc->bLength >= sizeof(*desc))) ? desc : NULL);
}

usb_hid_descriptor_t *
usbd_get_hdesc(usb_config_descriptor_t *cd, usb_interface_descriptor_t *id)
{
	usb_descriptor_t *desc = (void *)id;

	if(desc == NULL) {
	    return NULL;
	}

	while ((desc = usbd_desc_foreach(cd, desc)))
	{
		if ((desc->bDescriptorType == UDESC_HID) &&
		    (desc->bLength >= USB_HID_DESCRIPTOR_SIZE(0)))
		{
			return (void *)desc;
		}

		if (desc->bDescriptorType == UDESC_INTERFACE)
		{
			break;
		}
	}
	return NULL;
}

usb_interface_descriptor_t *
usbd_find_idesc(usb_config_descriptor_t *cd, u_int16_t iface_index, 
		u_int16_t alt_index)
{
	usb_descriptor_t *desc = NULL;
	usb_interface_descriptor_t *id;
	u_int16_t curidx = 0xFFFF;
	u_int16_t lastidx = 0xFFFF;
	u_int16_t curaidx = 0;

	while ((desc = usbd_desc_foreach(cd, desc)))
	{
	    if ((desc->bDescriptorType == UDESC_INTERFACE) &&
		(desc->bLength >= sizeof(*id)))
	    {
	        id = (void *)desc;

		if(id->bInterfaceNumber != lastidx)
		{
		    lastidx = id->bInterfaceNumber;
		    curidx++;
		    curaidx = 0;
		}
		else
		{
		    curaidx++;
		}
		if((iface_index == curidx) && (alt_index == curaidx))
		{
		    return (id);
		}
	    }
	}
	return (NULL);
}

usb_endpoint_descriptor_t *
usbd_find_edesc(usb_config_descriptor_t *cd, u_int16_t iface_index, 
		u_int16_t alt_index, u_int16_t endptidx)
{
	usb_descriptor_t *desc = NULL;
	usb_interface_descriptor_t *d;
	u_int16_t curidx = 0;

	d = usbd_find_idesc(cd, iface_index, alt_index);
	if (d == NULL)
	    return NULL;

	if (endptidx >= d->bNumEndpoints) /* quick exit */
	    return NULL;

	desc = ((void *)d);

	while ((desc = usbd_desc_foreach(cd, desc))) {

	    if(desc->bDescriptorType == UDESC_INTERFACE) {
	        break;
	    }

	    if (desc->bDescriptorType == UDESC_ENDPOINT) {

	        if (curidx == endptidx) {
		    return ((desc->bLength >= USB_ENDPOINT_DESCRIPTOR_SIZE) ? 
			    ((void *)desc) : NULL);
		}
		curidx++;
	    }
	}
	return (NULL);
}

/*------------------------------------------------------------------------*
 *	usbd_find_descriptor
 *
 * This function will lookup the first descriptor that matches
 * the criteria given by the arguments "type" and "subtype". Descriptors
 * will only be searched within the interface having the index "iface_index".
 * It is possible to specify the last descriptor returned by this function
 * as the "id" argument. That way one can search for multiple descriptors
 * matching the same criteria.
 *------------------------------------------------------------------------*/
void *
usbd_find_descriptor(struct usbd_device *udev, void *id, uint16_t iface_index,
		     int16_t type, int16_t subtype)
{
	usb_descriptor_t *desc;
	usb_config_descriptor_t *cd;
	struct usbd_interface *iface;

	cd = usbd_get_config_descriptor(udev);
	if (cd == NULL) {
		return NULL;
	}

	if (id == NULL) {
	    iface = usbd_get_iface(udev, iface_index);
	    if (iface == NULL) {
		return NULL;
	    }

	    id = usbd_get_interface_descriptor(iface);
	    if (id == NULL) {
		return NULL;
	    }
	}

	desc = (void *)id;

	while ((desc = usbd_desc_foreach(cd, desc))) {

		if (desc->bDescriptorType == UDESC_INTERFACE) {
			break;
		}

		if (((type == USBD_TYPE_ANY) ||
		     (type == desc->bDescriptorType)) &&
		    ((subtype == USBD_SUBTYPE_ANY) ||
		     (subtype == desc->bDescriptorSubtype)))
		{
			return desc;
		}
	}
	return NULL;
}

uint16_t
usbd_get_no_alts(usb_config_descriptor_t *cd, u_int8_t ifaceno)
{
	usb_descriptor_t *desc = NULL;
	usb_interface_descriptor_t *id;
	uint16_t n = 0;

	while ((desc = usbd_desc_foreach(cd, desc))) {

	    if ((desc->bDescriptorType == UDESC_INTERFACE) &&
		(desc->bLength >= sizeof(*id))) {
	        id = (void *)desc;
		if (id->bInterfaceNumber == ifaceno) {
		    n++;
		}
	    }
	}
	return n;
}

static void
usbd_fill_pipe_data(struct usbd_device *udev, u_int8_t iface_index,
		    usb_endpoint_descriptor_t *edesc, struct usbd_pipe *pipe)
{
	bzero(pipe, sizeof(*pipe));

	(udev->bus->methods->pipe_init)(udev,edesc,pipe);

	if (pipe->methods == NULL) {
	    /* the pipe is invalid: just return */
	    return;
	}

	pipe->edesc = edesc;
	pipe->iface_index = iface_index;
	LIST_INIT(&pipe->list_head);
	return;
}

/*
 * The USB Transaction Translator:
 * ===============================
 *
 * When doing LOW- and FULL-speed USB transfers accross a HIGH-speed
 * USB HUB, bandwidth must be allocated for ISOCHRONOUS and INTERRUPT
 * USB transfers. To utilize bandwidth dynamically the "scatter and
 * gather" principle must be applied. This means that bandwidth must
 * be divided into equal parts of bandwidth. With regard to USB all
 * data is transferred in smaller packets with length
 * "wMaxPacketSize". The problem however is that "wMaxPacketSize" is
 * not a constant!
 *
 * The bandwidth scheduler which I have implemented will simply pack
 * the USB transfers back to back until there is no more space in the
 * schedule. Out of the 8 microframes which the USB 2.0 standard
 * provides, only 6 are available for non-HIGH-speed devices. I have
 * reserved the first 4 microframes for ISOCHRONOUS transfers. The
 * last 2 microframes I have reserved for INTERRUPT transfers. Without
 * this division, it is very difficult to allocate and free bandwidth
 * dynamically.
 *
 * NOTE about the Transaction Translator in USB HUBs:
 *
 * USB HUBs have a very simple Transaction Translator, that will
 * simply pipeline all the SPLIT transactions. That means that the
 * first one queued will be executed!
 *
 */

static uint8_t
usbd_find_best_slot(uint32_t *ptr, uint8_t start, uint8_t end)
{
	uint32_t max = 0xffffffff;
	uint8_t x;
	uint8_t y;

	y = 0;

	/* find the last slot with lesser used bandwidth */

	for (x = start; x < end; x++) {
	    if (max >= ptr[x]) {
	        max = ptr[x];
		y = x;
	    }
	}
	return y;
}

uint8_t
usbd_intr_schedule_adjust(struct usbd_device *udev, int16_t len, uint8_t slot)
{
	struct usbd_bus *bus = udev->bus;
	struct usbd_hub *hub;

	if (usbd_get_speed(udev) == USB_SPEED_HIGH) {
	    if (slot >= USB_HS_MICRO_FRAMES_MAX) {
	        slot = usbd_find_best_slot(bus->uframe_usage, 0, 
					   USB_HS_MICRO_FRAMES_MAX);
	    }
	    bus->uframe_usage[slot] += len;
	} else {
	    if (usbd_get_speed(udev) == USB_SPEED_LOW) {
	        len *= 8;
	    }
	    hub = udev->myhsport->parent->hub;
	    if (slot >= USB_HS_MICRO_FRAMES_MAX) {
	        slot = usbd_find_best_slot(hub->uframe_usage, 
					   USB_FS_ISOC_UFRAME_MAX, 6);
	    }
	    hub->uframe_usage[slot] += len;
	    bus->uframe_usage[slot] += len;
	}
	return slot;
}

static void
usbd_fs_isoc_schedule_init_sub(struct usbd_fs_isoc_schedule *fss)
{
	fss->total_bytes = (USB_FS_ISOC_UFRAME_MAX * 
			    USB_FS_BYTES_PER_HS_UFRAME);
	fss->frame_bytes = (USB_FS_BYTES_PER_HS_UFRAME);
	fss->frame_slot = 0;
	return;
}

void
usbd_fs_isoc_schedule_init_all(struct usbd_fs_isoc_schedule *fss)
{
	struct usbd_fs_isoc_schedule *fss_end = fss + USB_ISOC_TIME_MAX;

	while (fss != fss_end) {
	    usbd_fs_isoc_schedule_init_sub(fss);
	    fss ++;
	}
	return;
}

uint16_t
usbd_fs_isoc_schedule_isoc_time_expand(struct usbd_device *udev,
				       struct usbd_fs_isoc_schedule **pp_start,
				       struct usbd_fs_isoc_schedule **pp_end,
				       uint16_t isoc_time)
{
	struct usbd_fs_isoc_schedule *fss_end;
	struct usbd_fs_isoc_schedule *fss_a;
	struct usbd_fs_isoc_schedule *fss_b;
	struct usbd_hub *hs_hub;

	isoc_time = usbd_isoc_time_expand(udev->bus, isoc_time);

	if (udev->myhsport &&
	    udev->myhsport->parent->hub) {

	    hs_hub = udev->myhsport->parent->hub;

	    fss_a = hs_hub->fs_isoc_schedule + 
	      (hs_hub->isoc_last_time % USB_ISOC_TIME_MAX);

	    hs_hub->isoc_last_time = isoc_time;

	    fss_b = hs_hub->fs_isoc_schedule +
	      (isoc_time % USB_ISOC_TIME_MAX);

	    fss_end = hs_hub->fs_isoc_schedule + USB_ISOC_TIME_MAX;

	    *pp_start = hs_hub->fs_isoc_schedule;
	    *pp_end = fss_end;

	    while (fss_a != fss_b) {
	        if (fss_a == fss_end) {
		    fss_a = hs_hub->fs_isoc_schedule;
		    continue;
		}
		usbd_fs_isoc_schedule_init_sub(fss_a);
		fss_a++;
	    }

	} else {

	    *pp_start = NULL;
	    *pp_end = NULL;
	}
	return isoc_time;
}

uint8_t
usbd_fs_isoc_schedule_alloc(struct usbd_fs_isoc_schedule *fss, uint16_t len)
{
	uint8_t slot = fss->frame_slot;

	/* Compute overhead and bit-stuffing */

	len += 8;

	len *= 7;
	len /= 6;

	if (len > fss->total_bytes) {
	    len = fss->total_bytes;
	}

	if (len > 0) {

	    fss->total_bytes -= len;

	    while (len >= fss->frame_bytes) {
	        len -= fss->frame_bytes;
		fss->frame_bytes = USB_FS_BYTES_PER_HS_UFRAME;
		fss->frame_slot ++;
	    }

	    fss->frame_bytes -= len;
	}
	return slot;
}

/* NOTE: pipes should not be in use when
 * ``usbd_free_pipe_data()'' is called
 */
static void
usbd_free_pipe_data(struct usbd_device *udev, int iface_index)
{
	struct usbd_pipe *pipe = udev->pipes;
	struct usbd_pipe *pipe_end = udev->pipes_end;

	while (pipe != pipe_end)
	{
		if((iface_index == pipe->iface_index) ||
		   (iface_index == -1))
		{
			/* free pipe */
			pipe->edesc = NULL;
		}
		pipe++;
	}
	return;
}

usbd_status
usbd_fill_iface_data(struct usbd_device *udev, int iface_index, int alt_index)
{
	struct usbd_interface *iface = usbd_get_iface(udev,iface_index);
	struct usbd_pipe *pipe = udev->pipes;
	struct usbd_pipe *pipe_end = udev->pipes_end;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed = NULL;
	usb_descriptor_t *desc;
	u_int8_t nendpt;

	if(iface == NULL)
	{
		return (USBD_INVAL);
	}

	PRINTFN(4,("iface_index=%d alt_index=%d\n",
		    iface_index, alt_index));

	/* mtx_assert() */

	while (pipe != pipe_end)
	{
		if(pipe->iface_index == iface_index)
		{
			if(pipe->refcount)
			{
				return(USBD_IN_USE);
			}
		}
		pipe++;
	}

	pipe = &udev->pipes[0];

	/* free old pipes if any */
	usbd_free_pipe_data(udev, iface_index);

	id = usbd_find_idesc(udev->cdesc, iface_index, alt_index);
	if(id == NULL)
	{
		return (USBD_INVAL);
	}
	iface->idesc = id;
	iface->alt_index = alt_index;

	USBD_CLR_IFACE_NO_PROBE(udev, iface_index);

	nendpt = id->bNumEndpoints;
	PRINTFN(4,("found idesc nendpt=%d\n", nendpt));

	desc = (void *)id;

	while(nendpt--)
	{
		PRINTFN(10,("endpt=%d\n", nendpt));

		while ((desc = usbd_desc_foreach(udev->cdesc, desc)))
		{
			if ((desc->bDescriptorType == UDESC_ENDPOINT) &&
			    (desc->bLength >= USB_ENDPOINT_DESCRIPTOR_SIZE))
			{
				goto found;
			}
			if (desc->bDescriptorType == UDESC_INTERFACE)
			{
				break;
			}
		}
		goto error;

	found:
		ed = (void *)desc;

		if(udev->speed == USB_SPEED_HIGH)
		{
			/* control and bulk endpoints have fixed max packet sizes */
			switch (UE_GET_XFERTYPE(ed->bmAttributes)) {
			case UE_CONTROL:
				USETW(ed->wMaxPacketSize, USB_2_MAX_CTRL_PACKET);
				break;
			case UE_BULK:
				USETW(ed->wMaxPacketSize, USB_2_MAX_BULK_PACKET);
				break;
			}
		}

		if (usbd_get_max_frame_size(ed) == 0) {
#ifdef USB_DEBUG
			printf("%s: invalid wMaxPacketSize=0x%04x, addr=%d!\n",
			       __FUNCTION__, UGETW(ed->wMaxPacketSize),
			       udev->address);
#endif
			/* avoid division by zero 
			 * (in EHCI/UHCI/OHCI drivers) 
			 */
			usbd_set_max_packet_size_count(ed, USB_MAX_IPACKET, 1);
		}

		/* find a free pipe */
		while (pipe != pipe_end)
		{
			if(pipe->edesc == NULL)
			{
				/* pipe is free */
				usbd_fill_pipe_data(udev,iface_index,ed,pipe);
				break;
			}
			pipe++;
		}
	}
	return (USBD_NORMAL_COMPLETION);

 error:
	/* passed end, or bad desc */
	printf("%s: bad descriptor(s), addr=%d!\n",
	       __FUNCTION__, udev->address);

	/* free old pipes if any */
	usbd_free_pipe_data(udev, iface_index);
	return (USBD_INVAL);
}

static void
usbd_free_iface_data(struct usbd_device *udev)
{
	struct usbd_interface *iface = udev->ifaces;
	struct usbd_interface *iface_end = udev->ifaces_end;

	/* mtx_assert() */

	/* free all pipes, if any */
	usbd_free_pipe_data(udev, -1);

	/* free all interfaces, if any */
	while (iface != iface_end)
	{
		iface->idesc = NULL;
		iface++;
	}

	if(udev->cdesc != NULL)
	{
		/* free "cdesc" after "ifaces" */
		free(udev->cdesc, M_USB);
	}
	udev->cdesc = NULL;
	udev->config = USB_UNCONFIG_NO;
	return;
}

/* - USB config 0
 *   - USB interfaces
 *     - USB alternative interfaces
 *       - USB pipes
 *
 * - USB config 1
 *   - USB interfaces
 *     - USB alternative interfaces
 *       - USB pipes
 */
usbd_status
usbd_search_and_set_config(struct usbd_device *udev, int no, int msg)
{
	usb_config_descriptor_t cd;
	usbd_status err;
	int index;

	if(no == USB_UNCONFIG_NO)
	{
		return (usbd_set_config_index(udev, USB_UNCONFIG_INDEX, msg));
	}

	PRINTFN(5,("%d\n", no));

	/* figure out what config index to use */
	for(index = 0; 
	    index < udev->ddesc.bNumConfigurations;
	    index++)
	{
		err = usbreq_get_config_desc(udev, index, &cd);
		if(err)
		{
			return (err);
		}
		if(cd.bConfigurationValue == no)
		{
			return (usbd_set_config_index(udev, index, msg));
		}
	}
	return (USBD_INVAL);
}

usbd_status
usbd_set_config_index(struct usbd_device *udev, int index, int msg)
{
	usb_status_t ds;
	usb_hub_descriptor_t hd;
	usb_config_descriptor_t cd, *cdp;
	usbd_status err;
	int nifc, len, selfpowered, power;

	PRINTFN(5,("udev=%p index=%d\n", udev, index));

	if(index == USB_UNCONFIG_INDEX)
	{
		/* leave unallocated when
		 * unconfiguring the device
		 */
		err = usbreq_set_config(udev, USB_UNCONFIG_NO);
		goto error;
	}

	/* get the short descriptor */
	err = usbreq_get_config_desc(udev, index, &cd);
	if(err)
	{
		goto error;
	}

	/* free all configuration data structures */
	usbd_free_iface_data(udev);

	/* get full descriptor */
	len = UGETW(cd.wTotalLength);
	udev->cdesc = malloc(len, M_USB, M_WAITOK|M_ZERO);
	if(udev->cdesc == NULL)
	{
		return (USBD_NOMEM);
	}

	cdp = udev->cdesc;

	/* Get the full descriptor. Try a few times for slow devices. */
	for (nifc = 0; nifc < 3; nifc++) {

	    err = usbreq_get_desc(udev, UDESC_CONFIG, index, len, cdp, 3);

	    if (!err) break;

	    usbd_delay_ms(udev, 200);
	}

	if (err)
		goto error;
	if(cdp->bDescriptorType != UDESC_CONFIG)
	{
		PRINTF(("bad desc %d\n",
			     cdp->bDescriptorType));
		err = USBD_INVAL;
		goto error;
	}
	if(cdp->bNumInterface > (sizeof(udev->ifaces)/sizeof(udev->ifaces[0])))
	{
		PRINTF(("too many interfaces: %d\n", cdp->bNumInterface));
		cdp->bNumInterface = (sizeof(udev->ifaces)/sizeof(udev->ifaces[0]));
	}

	/* Figure out if the device is self or bus powered. */
	selfpowered = 0;
	if (!(udev->quirks->uq_flags & UQ_BUS_POWERED) &&
	    (cdp->bmAttributes & UC_SELF_POWERED)) {
		/* May be self powered. */
		if (cdp->bmAttributes & UC_BUS_POWERED) {
			/* Must ask device. */
			if (udev->quirks->uq_flags & UQ_POWER_CLAIM) {
				/*
				 * Hub claims to be self powered, but isn't.
				 * It seems that the power status can be
				 * determined by the hub characteristics.
				 */
				err = usbreq_get_hub_descriptor(udev, &hd);

				if(!err &&
				   (UGETW(hd.wHubCharacteristics) &
				    UHD_PWR_INDIVIDUAL))
				{
					selfpowered = 1;
				}
				PRINTF(("characteristics=0x%04x, error=%s\n",
					 UGETW(hd.wHubCharacteristics),
					 usbd_errstr(err)));
			} else {
				err = usbreq_get_device_status(udev, &ds);
				if(!err &&
				   (UGETW(ds.wStatus) & UDS_SELF_POWERED))
				{
					selfpowered = 1;
				}
				PRINTF(("status=0x%04x, error=%s\n",
					 UGETW(ds.wStatus), usbd_errstr(err)));
			}
		} else
			selfpowered = 1;
	}
	PRINTF(("udev=%p cdesc=%p (addr %d) cno=%d attr=0x%02x, "
		 "selfpowered=%d, power=%d\n",
		 udev, cdp,
		 cdp->bConfigurationValue, udev->address, cdp->bmAttributes,
		 selfpowered, cdp->bMaxPower * 2));

	/* Check if we have enough power. */
	power = cdp->bMaxPower * 2;
	if(power > udev->powersrc->power)
	{
		PRINTF(("power exceeded %d %d\n", power, udev->powersrc->power));
		/* XXX print nicer message */
		if(msg)
		{
			device_printf(udev->bus->bdev,
				      "device addr %d (config %d) exceeds power "
				      "budget, %d mA > %d mA\n",
				      udev->address,
				      cdp->bConfigurationValue,
				      power, udev->powersrc->power);
		}
		err = USBD_NO_POWER;
		goto error;
	}

	udev->power = power;
	udev->self_powered = selfpowered;
	udev->config = cdp->bConfigurationValue;

	/* Set the actual configuration value. */
	err = usbreq_set_config(udev, cdp->bConfigurationValue);
	if(err)
	{
		goto error;
	}

	/* Allocate and fill interface data. */
	nifc = cdp->bNumInterface;
	while(nifc--)
	{
		err = usbd_fill_iface_data(udev, nifc, 0);
		if (err)
		{
			goto error;
		}
	}

	return (USBD_NORMAL_COMPLETION);

 error:
	PRINTF(("error=%s\n", usbd_errstr(err)));
	usbd_free_iface_data(udev);
	return (err);
}

int
usbd_fill_deviceinfo(struct usbd_device *udev, struct usb_device_info *di,
		     int usedev)
{
	struct usbd_port *p;
	uint16_t s;
	uint8_t i;
	uint8_t err;

	if((udev == NULL) || (di == NULL))
	{
		return (ENXIO);
	}

	bzero(di, sizeof(di[0]));

	di->udi_bus = device_get_unit(udev->bus->bdev);
	di->udi_addr = udev->address;
	usbd_devinfo_vp(udev, di->udi_vendor, di->udi_product, 
			sizeof(di->udi_vendor), sizeof(di->udi_product), usedev);
	usbd_printBCD(di->udi_release, sizeof(di->udi_release), UGETW(udev->ddesc.bcdDevice));
	di->udi_vendorNo = UGETW(udev->ddesc.idVendor);
	di->udi_productNo = UGETW(udev->ddesc.idProduct);
	di->udi_releaseNo = UGETW(udev->ddesc.bcdDevice);
	di->udi_class = udev->ddesc.bDeviceClass;
	di->udi_subclass = udev->ddesc.bDeviceSubClass;
	di->udi_protocol = udev->ddesc.bDeviceProtocol;
	di->udi_config = udev->config;
	di->udi_power = udev->self_powered ? 0 : udev->power;
	di->udi_speed = udev->speed;

	for(i = 0;
	    (i < (sizeof(udev->subdevs)/sizeof(udev->subdevs[0]))) &&
	      (i < USB_MAX_DEVNAMES);
	    i++)
	{
		if(udev->subdevs[i] &&
		   device_is_attached(udev->subdevs[i]))
		{
			strlcpy(di->udi_devnames[i],
				device_get_nameunit(udev->subdevs[i]),
				USB_MAX_DEVNAMELEN);
		}
	}

	if(udev->hub)
	{
		for(i = 0;
		    (i < (sizeof(di->udi_ports)/sizeof(di->udi_ports[0]))) &&
		      (i < udev->hub->hubdesc.bNbrPorts);
		    i++)
		{
			p = &udev->hub->ports[i];
			if(p->device)
			{
				err = p->device->address;
			}
			else
			{
				s = UGETW(p->status.wPortStatus);
				if (s & UPS_PORT_ENABLED)
				{
					err = USB_PORT_ENABLED;
				}
				else if (s & UPS_SUSPEND)
				{
					err = USB_PORT_SUSPENDED;
				}
				else if (s & UPS_PORT_POWER)
				{
					err = USB_PORT_POWERED;
				}
				else
				{
					err = USB_PORT_DISABLED;
				}
			}
			di->udi_ports[i] = err;
		}
		di->udi_nports = udev->hub->hubdesc.bNbrPorts;
	}
	return 0;
}

/* The following function will remove detached 
 * devices from the interface list. This can
 * happen during USB device module unload.
 */
static void
usbd_remove_detached_devices(struct usbd_device *udev)
{
	device_t *subdev = udev->subdevs;
	device_t *subdev_end = udev->subdevs_end;
	uint8_t detached_first = 0;

	PRINTFN(3,("udev=%p\n", udev));

	while (subdev != subdev_end) {
	    if (subdev[0]) {
	        if (device_is_attached(subdev[0]) == 0) {
		    if (device_delete_child(device_get_parent(subdev[0]), 
					    subdev[0]) == 0) {
		        subdev[0] = NULL;
			if (subdev == udev->subdevs) {
			    detached_first = 1;
			}

		    } else {
		        /* Panic here, else one can get a double call to 
			 * device_detach(). USB devices should never fail
			 * on detach!
			 */
		        panic("device_delete_child() failed!\n");
		    }
		}
	    }
	    subdev++;
	}

	if (detached_first) {
	  if ((udev->probed == USBD_PROBED_SPECIFIC_AND_FOUND) ||
	      (udev->probed == USBD_PROBED_GENERIC_AND_FOUND)) {
	      /* The first and only device is gone. 
	       * Reset the "probed" variable.
	       */
	      udev->probed = USBD_PROBED_NOTHING;
	  }
	}
	return;
}

/* "usbd_probe_and_attach()" is called 
 * from "usbd_new_device()" and "uhub_explore()"
 */
usbd_status
usbd_probe_and_attach(device_t parent, int port, struct usbd_port *up)
{
	struct usb_attach_arg uaa;
	struct usbd_device *udev = up->device;
	device_t bdev = NULL;
	usbd_status err = 0;
	u_int8_t config;
	u_int8_t i;

	up->last_refcount = usb_driver_added_refcount;

	if(udev == NULL)
	{
		PRINTF(("%s: port %d has no device\n", 
			device_get_nameunit(parent), port));
		return (USBD_INVAL);
	}

	usbd_remove_detached_devices(udev);

	bzero(&uaa, sizeof(uaa));

	/* probe and attach */

	uaa.device = udev;
	uaa.port = port;
	uaa.configno = -1;
	uaa.vendor = UGETW(udev->ddesc.idVendor);
	uaa.product = UGETW(udev->ddesc.idProduct);
	uaa.release = UGETW(udev->ddesc.bcdDevice);

	if((udev->probed == USBD_PROBED_SPECIFIC_AND_FOUND) ||
	   (udev->probed == USBD_PROBED_GENERIC_AND_FOUND))
	{
		/* nothing more to probe */
		goto done;
	}

	bdev = device_add_child(parent, NULL, -1);
	if(!bdev)
	{
		device_printf(udev->bus->bdev,
			      "Device creation failed\n");
		err = USBD_INVAL;
		goto done;
	}

	device_set_ivars(bdev, &uaa);
	device_quiet(bdev);

	if(udev->probed == USBD_PROBED_NOTHING)
	{
		/* first try device specific drivers */
		PRINTF(("trying device specific drivers\n"));

		if(device_probe_and_attach(bdev) == 0)
		{
			device_set_ivars(bdev, NULL); /* no longer accessible */
			udev->subdevs[0] = bdev;
			udev->probed = USBD_PROBED_SPECIFIC_AND_FOUND;
			bdev = 0;
			goto done;
		}

		PRINTF(("no device specific driver found; "
			"looping over %d configurations\n",
			udev->ddesc.bNumConfigurations));
	}

	/* next try interface drivers */

	if((udev->probed == USBD_PROBED_NOTHING) ||
	   (udev->probed == USBD_PROBED_IFACE_AND_FOUND))
	{
	  for(config = 0; config < udev->ddesc.bNumConfigurations; config++)
	  {
		struct usbd_interface *iface;

		/* only set config index the first 
		 * time the devices are probed
		 */
		if(udev->probed == USBD_PROBED_NOTHING)
		{
			err = usbd_set_config_index(udev, config, 1);
			if(err)
			{
			    device_printf(parent,
					  "port %d, set config at addr %d "
					  "failed, error=%s\n",
					  port, udev->address, 
					  usbd_errstr(err));
			    goto done;
			}

			/* ``bNumInterface'' is checked 
			 * by ``usbd_set_config_index()''
			 *
			 * ``USBD_CLR_IFACE_NO_PROBE()'' is run
			 * by ``usbd_fill_iface_data()'', which
			 * is called by ``usbd_set_config_index()''
			 */
		}

		/*
		 * else the configuration is already set
		 */

		uaa.configno = udev->cdesc->bConfigurationValue;
		uaa.ifaces_start = udev->ifaces;
		uaa.ifaces_end = udev->ifaces + udev->cdesc->bNumInterface;

		for(iface = uaa.ifaces_start;
		    iface != uaa.ifaces_end;
		    iface++)
		{
			uaa.iface = iface;
			uaa.iface_index = (i = (iface - udev->ifaces));

			if(uaa.iface_index >= (sizeof(udev->subdevs)/
					       sizeof(udev->subdevs[0])))
			{
				device_printf(udev->bus->bdev,
					      "Too many subdevices\n");
				break;
			}

			if((USBD_GET_IFACE_NO_PROBE(udev, i) == 0) &&
			   (udev->subdevs[i] == NULL) &&
			   (device_probe_and_attach(bdev) == 0))
			{
				/* "ivars" are no longer accessible: */
				device_set_ivars(bdev, NULL); 
				udev->subdevs[i] = bdev;
				udev->probed = USBD_PROBED_IFACE_AND_FOUND;
				bdev = 0;

				/* create another child for the next iface [if any] */
				bdev = device_add_child(parent, NULL, -1);
				if(!bdev)
				{
					device_printf(udev->bus->bdev,
						      "Device creation failed\n");

					/* need to update "IFACE_NO_PROBE": */
					break; 
				}
				device_set_ivars(bdev, &uaa);
				device_quiet(bdev);
			}
		}

		if(udev->probed == USBD_PROBED_IFACE_AND_FOUND)
		{
			break;
		}
	  }
	}

	if(udev->probed == USBD_PROBED_NOTHING)
	{
		/* set config index 0 */

		err = usbd_set_config_index(udev, 0, 1);
		if(err)
		{
		    device_printf(parent,
				  "port %d, set config at addr %d "
				  "failed, error=%s\n",
				  port, udev->address, 
				  usbd_errstr(err));
		    goto done;
		}

		PRINTF(("no interface drivers found\n"));

		/* finally try the generic driver */
		uaa.iface = NULL;
		uaa.iface_index = 0;
		uaa.ifaces_start = NULL;
		uaa.ifaces_end = NULL;
		uaa.usegeneric = 1;
		uaa.configno = -1;

		if(device_probe_and_attach(bdev) == 0)
		{
			device_set_ivars(bdev, NULL); /* no longer accessible */
			udev->subdevs[0] = bdev;
			udev->probed = USBD_PROBED_GENERIC_AND_FOUND;
			bdev = 0;
			goto done;
		}

		/*
		 * Generic attach failed. 
		 * The device is left as it is.
		 * It has no driver, but is fully operational.
		 */
		PRINTF(("generic attach failed\n"));
	}
 done:
	if(bdev)
	{
		/* remove the last created child; it is unused */
		device_delete_child(parent, bdev);
	}
	return err;
}

/*
 * Called when a new device has been put in the powered state,
 * but not yet in the addressed state.
 * Get initial descriptor, set the address, get full descriptor,
 * and attach a driver.
 */
usbd_status
usbd_new_device(device_t parent, struct usbd_bus *bus, int depth,
		int speed, int port, struct usbd_port *up)
{
	struct usbd_device *adev;
	struct usbd_device *udev;
	struct usbd_device *hub;
	usbd_status err = 0;
	int addr;
	int i;

	PRINTF(("bus=%p port=%d depth=%d speed=%d\n",
		 bus, port, depth, speed));

	/* find unused address */
	addr = USB_MAX_DEVICES;
#if (USB_MAX_DEVICES == 0)
#error "(USB_MAX_DEVICES == 0)"
#endif
	while(addr--)
	{
		if(addr == 0)
		{
			/* address 0 is always unused */
			device_printf(bus->bdev,
				      "No free USB addresses, "
				      "new device ignored.\n");
			return (USBD_NO_ADDR);
		}
		if(bus->devices[addr] == 0)
		{
			break;
		}
	}

	udev = malloc(sizeof(udev[0]), M_USB, M_WAITOK|M_ZERO);
	if(udev == NULL)
	{
		return (USBD_NOMEM);
	}

	up->device = udev;

	/* set up default endpoint descriptor */
	udev->default_ep_desc.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE;
	udev->default_ep_desc.bDescriptorType = UDESC_ENDPOINT;
	udev->default_ep_desc.bEndpointAddress = USB_CONTROL_ENDPOINT;
	udev->default_ep_desc.bmAttributes = UE_CONTROL;
	USETW(udev->default_ep_desc.wMaxPacketSize, USB_MAX_IPACKET);
	udev->default_ep_desc.bInterval = 0;

	udev->bus = bus;
	udev->quirks = &usbd_no_quirk;
	udev->address = USB_START_ADDR;
	udev->ddesc.bMaxPacketSize = 0;
	udev->depth = depth;
	udev->powersrc = up;
	udev->myhub = up->parent;

	hub = up->parent;

	if(hub)
	{
		if(speed > hub->speed)
		{
#ifdef USB_DEBUG
			printf("%s: maxium speed of attached "
			       "device, %d, is higher than speed "
			       "of parent HUB, %d.\n",
			       __FUNCTION__, speed, hub->speed);
#endif
			/* speed down 
			 * (else there is trouble setting 
			 *  up the right transfer methods)
			 */
			speed = hub->speed;
		}
	}

	adev = udev;
	while(hub && (hub->speed != USB_SPEED_HIGH))
	{
		adev = hub;
		hub = hub->myhub;
	}

	if(hub)
	{
		for(i = 0; i < hub->hub->hubdesc.bNbrPorts; i++)
		{
			if(hub->hub->ports[i].device == adev)
			{
				udev->myhsport = &hub->hub->ports[i];
				break;
			}
		}
	}

	udev->speed = speed;
	udev->langid = USBD_NOLANG;

	/* init the default pipe */
	usbd_fill_pipe_data(udev, 0,
			    &udev->default_ep_desc,
			    &udev->default_pipe);

	err = usbreq_set_address(udev, addr);
	if(err)
	{
		PRINTF(("set address %d failed\n", addr));
		err = USBD_SET_ADDR_FAILED;
		goto done;
	}

	/* allow device time to set new address */
	usbd_delay_ms(udev, USB_SET_ADDRESS_SETTLE);
	udev->address = addr;	/* new device address now */

	mtx_lock(&(bus->mtx));
	bus->devices[addr] = udev;
	mtx_unlock(&(bus->mtx));

	/* get the first 8 bytes of the device descriptor */
	err = usbreq_get_desc(udev, UDESC_DEVICE, 0, USB_MAX_IPACKET, &udev->ddesc, 0);
	if(err)
	{
		PRINTF(("addr=%d, getting first desc failed\n",
			 udev->address));
		goto done;
	}

	if(speed == USB_SPEED_HIGH)
	{
		/* max packet size must be 64 (sec 5.5.3) */
		udev->ddesc.bMaxPacketSize = USB_2_MAX_CTRL_PACKET;
	}

	if(udev->ddesc.bMaxPacketSize == 0)
	{
#ifdef USB_DEBUG
		printf("%s: addr=%d invalid bMaxPacketSize=0x00!\n",
		       __FUNCTION__, udev->address);
#endif
		/* avoid division by zero */
		udev->ddesc.bMaxPacketSize = USB_MAX_IPACKET;
	}

	PRINTF(("adding unit addr=%d, rev=%02x, class=%d, "
		 "subclass=%d, protocol=%d, maxpacket=%d, len=%d, speed=%d\n",
		 udev->address, UGETW(udev->ddesc.bcdUSB),
		 udev->ddesc.bDeviceClass,
		 udev->ddesc.bDeviceSubClass,
		 udev->ddesc.bDeviceProtocol,
		 udev->ddesc.bMaxPacketSize,
		 udev->ddesc.bLength,
		 udev->speed));

	if(udev->ddesc.bDescriptorType != UDESC_DEVICE)
	{
		/* illegal device descriptor */
		PRINTF(("illegal descriptor %d\n",
			 udev->ddesc.bDescriptorType));
		err = USBD_INVAL;
		goto done;
	}
	if(udev->ddesc.bLength < USB_DEVICE_DESCRIPTOR_SIZE)
	{
		PRINTF(("bad length %d\n",
			 udev->ddesc.bLength));
		err = USBD_INVAL;
		goto done;
	}

	USETW(udev->default_ep_desc.wMaxPacketSize, udev->ddesc.bMaxPacketSize);

	/* get the full device descriptor */
	err = usbreq_get_device_desc(udev, &udev->ddesc);
	if(err)
	{
		PRINTF(("addr=%d, getting full desc failed\n",
			 udev->address));
		goto done;
	}

	/* figure out what's wrong with this device */
	udev->quirks = usbd_find_quirk(&udev->ddesc);

	/* assume 100mA bus powered for now. Changed when configured. */
	udev->power = USB_MIN_POWER;
	udev->self_powered = 0;

	/* buffer serial number */
	(void) usbreq_get_string_any
	  (udev, udev->ddesc.iSerialNumber, 
	   &udev->serial[0], sizeof(udev->serial));

	/* check serial number format */

	for(i = 0;;i++)
	{
		if(udev->serial[i] == '\0') break;
		if(udev->serial[i] == '\"') udev->serial[i] = ' ';
		if(udev->serial[i] == '\n') udev->serial[i] = ' ';
	}

	PRINTF(("new dev (addr %d), udev=%p, parent=%p\n",
		 udev->address, udev, parent));

	err = usbd_probe_and_attach(parent, port, up);

 done:
	if(err)
	{
		/* remove device and sub-devices */
		usbd_free_device(up, 1);
	}
	return(err);
}

/* called when a port has been disconnected
 *
 * The general mechanism for detaching:
 *
 * The drivers should use a static softc or a softc which is not freed
 * immediately, so that calls to routines which are about to access
 * the softc, does not access freed memory.
 *
 * The drivers mutex should also be available for some time after
 * detach.
 *
 * The drivers should have a detach flag which is set when the driver
 * is detached. The detach flag is checked after locking drivers mutex
 * and after waking up from sleep. When the detach flag is set, the
 * driver must unlock drivers mutex and exit.
 */
void
usbd_free_device(struct usbd_port *up, u_int8_t free_subdev)
{
	struct usbd_device *udev = up->device;
	device_t *subdev = udev->subdevs;
	device_t *subdev_end = udev->subdevs_end;
	int error = 0;

	/* mtx_assert() */

	if(udev == NULL)
	{
		/* already freed */
		return;
	}

	PRINTFN(3,("up=%p udev=%p port=%d; "
		    "disconnect subdevs\n",
		    up, udev, up->portno));

	while (subdev != subdev_end)
	{
		if(subdev[0] && free_subdev)
		{
			device_printf(subdev[0], "at %s ", device_get_nameunit
				(device_get_parent(subdev[0])));

			if(up->portno != 0)
			{
				printf("port %d ", up->portno);
			}

			printf("(addr %d) disconnected\n", udev->address);

			/* first detach the child to give the child's detach routine
			 * a chance to detach the sub-devices in the correct order.
			 * Then delete the child using "device_delete_child()" which
			 * will detach all sub-devices from the bottom and upwards!
			 */
			if (device_detach(subdev[0]) || device_delete_child
				(device_get_parent(subdev[0]), subdev[0]))
			{
				/* if detach fails sub-devices will still
				 * be referring to the udev structure
				 * which cannot be freed
				 */
				device_printf(subdev[0], "detach failed "
					"(please ensure that this "
					"device driver supports detach)\n");

				error = ENXIO;
			}
		}

		/* always clear subdev[0], 
		 * because it might be used by
		 * usbd_add_dev_event()
		 */
		subdev[0] = NULL;
		subdev++;
	}

	/* issue detach event and free address */
	if(udev->bus != NULL)
	{
		struct usbd_bus *bus = udev->bus;

		/* NOTE: address 0 is always unused */

		/* Wait until all references to our
		 * device is gone:
		 */
		mtx_lock(&(bus->mtx));
		udev->detaching = 1;
		while (udev->refcount > 0) {
		    int dummy;
		    dummy = mtx_sleep(&(udev->detaching), &(bus->mtx), 0, 
				      "USB detach wait", 0);
		}
		bus->devices[udev->address] = 0;

		mtx_unlock(&(bus->mtx));
	}

	usbd_free_iface_data(udev);

	if(error)
	{
		panic("%s: some USB devices would not detach\n",
			__FUNCTION__);
	}

	/* free Linux compat device if any */
	if (udev->linux_dev) {
	    usb_linux_free_usb_device(udev->linux_dev);
	    udev->linux_dev = NULL;
	}

	/* free device */
	free(udev, M_USB);
	up->device = 0;
	return;
}

struct usbd_device *
usbd_ref_device(struct usbd_bus *bus, uint8_t addr)
{
	struct usbd_device *udev;

	if ((addr < 1) ||
	    (addr >= USB_MAX_DEVICES)) {
	    /* invalid address */
	    return NULL;
	}

	mtx_lock(&(bus->mtx));
	udev = bus->devices[addr];
	if (udev == NULL) {
	    /* do nothing */
	} else if (udev->detaching) {
	    udev = NULL;
	} else if (udev->refcount == USB_DEV_REFCOUNT_MAX) {
	    udev = NULL;
	} else {
	    udev->refcount ++;
	}
	mtx_unlock(&(bus->mtx));
	return udev;
}

void
usbd_unref_device(struct usbd_device *udev)
{
	struct usbd_bus *bus;

	if (udev == NULL) {
	    /* should not happen */
	    return;
	}

	bus = udev->bus;

	if (bus == NULL) {
	    /* should not happen */
	    return;
	}

	mtx_lock(&(bus->mtx));
	if (udev->refcount == 0) {
	    panic("Invalid USB device refcount!\n");
	} else {
	  if (--(udev->refcount) == 0) {
	      if (udev->detaching) {
	          wakeup(&(udev->detaching));
	      }
	  }
	}
	mtx_unlock(&(bus->mtx));
	return;
}

struct usbd_interface *
usbd_get_iface(struct usbd_device *udev, u_int8_t iface_index)
{
	struct usbd_interface *iface = udev->ifaces + iface_index;

	if ((iface < udev->ifaces) ||
	    (iface >= udev->ifaces_end) ||
	    (udev->cdesc == NULL) ||
	    (iface_index >= udev->cdesc->bNumInterface))
	{
		return NULL;
	}
	return iface;
}

void
usbd_set_desc(device_t dev, struct usbd_device *udev)
{
	u_int8_t devinfo[256];

	usbd_devinfo(udev, 1, devinfo, sizeof(devinfo));
	device_set_desc_copy(dev, devinfo);
	device_printf(dev, "<%s>\n", devinfo);
	return;
}

/*------------------------------------------------------------------------------*
 *      allocate mbufs to an usbd interface queue
 *
 * returns a pointer that eventually should be passed to "free()"
 *------------------------------------------------------------------------------*/
void *
usbd_alloc_mbufs(struct malloc_type *type, struct usbd_ifqueue *ifq, 
		 u_int32_t block_size, u_int16_t block_number)
{
	struct usbd_mbuf *m_ptr;
	u_int8_t *data_ptr;
	void *free_ptr = NULL;
	u_int32_t alloc_size;

        /* align data */
        block_size += ((-block_size) & (USB_HOST_ALIGN-1));

	if (block_number && block_size) {

	  alloc_size = (block_size + sizeof(struct usbd_mbuf)) * block_number;

	  free_ptr = malloc(alloc_size, type, M_WAITOK|M_ZERO);

	  if (free_ptr == NULL) {
	      goto done;
	  }

	  m_ptr = free_ptr;
	  data_ptr = (void *)(m_ptr + block_number);

	  while(block_number--) {

	      m_ptr->cur_data_ptr =
		m_ptr->min_data_ptr = data_ptr;

	      m_ptr->cur_data_len =
		m_ptr->max_data_len = block_size;

	      USBD_IF_ENQUEUE(ifq, m_ptr);

	      m_ptr++;
	      data_ptr += block_size;
	  }
	}
 done:
	return free_ptr;
}

/*------------------------------------------------------------------------------*
 *  usbd_get_page - lookup DMA-able memory for the given offset
 *------------------------------------------------------------------------------*/
void
usbd_get_page(struct usbd_page_cache *cache, u_int32_t offset, 
	      struct usbd_page_search *res)
{
	struct usbd_page *page;

	offset += cache->page_offset_buf;

	if ((offset < cache->page_offset_cur) ||
	    (cache->page_cur == NULL)) {

	    /* reset the page search */

	    cache->page_cur = cache->page_start;
	    cache->page_offset_cur = 0;
	}

	offset -= cache->page_offset_cur;
	page = cache->page_cur;

	while (1) {

	    if (page >= cache->page_end) {
	        res->length = 0;
		res->buffer = NULL;
		res->physaddr = 0;
		res->page = NULL;
		break;
	    }

	    if (offset < page->length) {

	        /* found the offset on a page */

	        res->length = page->length - offset;
		res->buffer = ADD_BYTES(page->buffer, offset);
		res->physaddr = page->physaddr + offset;
		res->page = page;
		break;
	    }

	    /* get the next page */

	    offset -= page->length;
	    cache->page_offset_cur += page->length;
	    page++;
	    cache->page_cur = page;
	}
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_copy_in - copy directly to DMA-able memory
 *------------------------------------------------------------------------------*/
void
usbd_copy_in(struct usbd_page_cache *cache, u_int32_t offset, 
	     const void *ptr, u_int32_t len)
{
	struct usbd_page_search res;

	while (len) {

	    usbd_get_page(cache, offset, &res);

	    if (res.length == 0) {
	        panic("%s:%d invalid offset!\n",
		      __FUNCTION__, __LINE__);
	    }

	    if (res.length > len) {
	        res.length = len;
	    }

	    usbd_page_dma_exit(res.page);

	    bcopy(ptr, res.buffer, res.length);

	    usbd_page_dma_enter(res.page);

	    offset += res.length;
	    len -= res.length;
	    ptr = ((const u_int8_t *)ptr) + res.length;
	}
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_m_copy_in - copy a mbuf chain directly into DMA-able memory
 *------------------------------------------------------------------------------*/
struct usbd_m_copy_in_arg {
	struct usbd_page_cache *cache;
	u_int32_t dst_offset;
};

static int32_t
#ifdef __FreeBSD__
usbd_m_copy_in_cb(void *arg, void *src, u_int32_t count)
#else
usbd_m_copy_in_cb(void *arg, caddr_t src, u_int32_t count)
#endif
{
	register struct usbd_m_copy_in_arg *ua = arg;
	usbd_copy_in(ua->cache, ua->dst_offset, src, count);
	ua->dst_offset += count;
	return 0;
}

void
usbd_m_copy_in(struct usbd_page_cache *cache, u_int32_t dst_offset,
	       struct mbuf *m, u_int32_t src_offset, u_int32_t src_len)
{
	struct usbd_m_copy_in_arg arg = { cache, dst_offset };
	register int error;
	error = m_apply(m, src_offset, src_len, &usbd_m_copy_in_cb, &arg);
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_copy_out - copy directly from DMA-able memory
 *------------------------------------------------------------------------------*/
void
usbd_copy_out(struct usbd_page_cache *cache, u_int32_t offset, 
	      void *ptr, u_int32_t len)
{
	struct usbd_page_search res;

	while (len) {

	    usbd_get_page(cache, offset, &res);

	    if (res.length == 0) {
	        panic("%s:%d invalid offset!\n",
		      __FUNCTION__, __LINE__);
	    }

	    if (res.length > len) {
	        res.length = len;
	    }

	    usbd_page_dma_exit(res.page);

	    bcopy(res.buffer, ptr, res.length);

	    usbd_page_dma_enter(res.page);

	    offset += res.length;
	    len -= res.length;
	    ptr = ADD_BYTES(ptr, res.length);
	}
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_bzero - zero DMA-able memory
 *------------------------------------------------------------------------------*/
void
usbd_bzero(struct usbd_page_cache *cache, u_int32_t offset, u_int32_t len)
{
	struct usbd_page_search res;

	while (len) {

	    usbd_get_page(cache, offset, &res);

	    if (res.length == 0) {
	        panic("%s:%d invalid offset!\n",
		      __FUNCTION__, __LINE__);
	    }

	    if (res.length > len) {
	        res.length = len;
	    }

	    usbd_page_dma_exit(res.page);

	    bzero(res.buffer, res.length);

	    usbd_page_dma_enter(res.page);

	    offset += res.length;
	    len -= res.length;
	}
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_page_alloc - allocate multiple DMA-able memory pages
 *
 * return values:
 *   1: failure
 *   0: success
 *------------------------------------------------------------------------------*/
u_int8_t
usbd_page_alloc(bus_dma_tag_t tag, struct usbd_page *page, 
		u_int32_t npages)
{
	u_int32_t x;
	void *ptr;

	for (x = 0; x < npages; x++) {

	repeat:

	    ptr = usbd_mem_alloc_sub(tag, page + x, 
				     USB_PAGE_SIZE, USB_PAGE_SIZE);
	    if (ptr == NULL) {

	        while(x--) {
		    usbd_mem_free_sub(page + x);
		}
		return 1; /* failure */
	    } else {

	      if ((page + x)->physaddr == 0) {

		  /* at least the OHCI controller
		   * gives special meaning to 
		   * physaddr == 0, so discard
		   * that page if it gets here:
		   */
		  printf("%s:%d: Discarded memory "
			 "page with physaddr=0!\n",
			 __FUNCTION__, __LINE__);
		  goto repeat;
	      }
	    }
	}
	return 0; /* success */
}

/*------------------------------------------------------------------------------*
 *  usbd_page_free - free multiple DMA-able memory pages
 *------------------------------------------------------------------------------*/
void
usbd_page_free(struct usbd_page *page, u_int32_t npages)
{
	while(npages--) {
	    usbd_mem_free_sub(page + npages);
	}
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_page_get_info - get USB page info by size offset
 *------------------------------------------------------------------------------*/
void
usbd_page_get_info(struct usbd_page *page, u_int32_t size, 
		   struct usbd_page_info *info)
{
	page += (size / USB_PAGE_SIZE);
	size &= (USB_PAGE_SIZE-1);
	info->buffer = ADD_BYTES(page->buffer,size);
	info->physaddr = (page->physaddr + size);
	info->page = page;
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_page_set_start
 *------------------------------------------------------------------------------*/
void
usbd_page_set_start(struct usbd_page_cache *pc, struct usbd_page *page_ptr,
		    u_int32_t size)
{
	pc->page_start = page_ptr + (size / USB_PAGE_SIZE);
	pc->page_offset_buf = (size % USB_PAGE_SIZE);
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_page_set_end
 *------------------------------------------------------------------------------*/
void
usbd_page_set_end(struct usbd_page_cache *pc, struct usbd_page *page_ptr,
		  u_int32_t size)
{
	pc->page_end = (page_ptr + 
			((size + USB_PAGE_SIZE -1) / USB_PAGE_SIZE));
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_page_fit_obj - fit object function
 *------------------------------------------------------------------------------*/
u_int32_t
usbd_page_fit_obj(u_int32_t size, u_int32_t obj_len)
{
	u_int32_t adj;

	if (obj_len > (USB_PAGE_SIZE - (size & (USB_PAGE_SIZE-1)))) {

	  if (obj_len > USB_PAGE_SIZE) {
	      panic("%s:%d Too large object, %d bytes, will "
		    "not fit on a USB page, %d bytes!\n", 
		    __FUNCTION__, __LINE__, obj_len, 
		    (int32_t)USB_PAGE_SIZE);
	  }

	  /* adjust offset to the beginning 
	   * of the next page:
	   */
	  adj = ((-size) & (USB_PAGE_SIZE-1));
	} else {
	  adj = 0;
	}
	return adj;
}

/*------------------------------------------------------------------------------*
 *  usbd_mem_alloc - allocate DMA-able memory
 *------------------------------------------------------------------------------*/
void *
usbd_mem_alloc(bus_dma_tag_t parent, struct usbd_page *page, uint32_t size, 
	       uint8_t align_power)
{
	bus_dma_tag_t tag;
	u_int32_t alignment = (1 << align_power);
	void *ptr;

	tag = usbd_dma_tag_alloc(parent, size, alignment);

	if (tag == NULL) {
	    return NULL;
	}

	ptr = usbd_mem_alloc_sub(tag, page, size, alignment);

	if (ptr == NULL) {
	    usbd_dma_tag_free(tag);
	    return NULL;
	}

	return ptr;
}

/*------------------------------------------------------------------------------*
 *  usbd_mem_free - free DMA-able memory
 *------------------------------------------------------------------------------*/
void
usbd_mem_free(struct usbd_page *page)
{
	bus_dma_tag_t tag;

	tag = page->tag;

	usbd_mem_free_sub(page);

	usbd_dma_tag_free(tag);

	return;
}

#ifdef __FreeBSD__
/*------------------------------------------------------------------------------*
 *  bus_dmamap_load_callback
 *------------------------------------------------------------------------------*/
static void
bus_dmamap_load_callback(void *arg, bus_dma_segment_t *segs, 
			 int nseg, int error)
{
	*((bus_size_t *)arg) = nseg ? segs->ds_addr : 0;

	if(error)
	{
	    printf("%s: %s: error=%d\n",
		   __FILE__, __FUNCTION__, error);
	}
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_dma_tag_alloc - allocate a bus-DMA tag
 *------------------------------------------------------------------------------*/
bus_dma_tag_t 
usbd_dma_tag_alloc(bus_dma_tag_t parent, u_int32_t size, 
		   u_int32_t alignment)
{
	bus_dma_tag_t tag;

	if(bus_dma_tag_create
	   ( /* parent    */parent,
	     /* alignment */alignment,
	     /* boundary  */0,
	     /* lowaddr   */BUS_SPACE_MAXADDR_32BIT,
	     /* highaddr  */BUS_SPACE_MAXADDR,
	     /* filter    */NULL,
	     /* filterarg */NULL,
	     /* maxsize   */size,
	     /* nsegments */1,
	     /* maxsegsz  */size,
	     /* flags     */0,
	     /* lock      */NULL,
	     /*           */NULL,
	     &tag))
	{
	    tag = NULL;
	}
	return tag;
}

/*------------------------------------------------------------------------------*
 *  usbd_dma_tag_free - free a bus-DMA tag
 *------------------------------------------------------------------------------*/
void
usbd_dma_tag_free(bus_dma_tag_t tag)
{
	bus_dma_tag_destroy(tag);
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_mem_alloc_sub - allocate DMA-able memory
 *------------------------------------------------------------------------------*/
void *
usbd_mem_alloc_sub(bus_dma_tag_t tag, struct usbd_page *page,
		   u_int32_t size, u_int32_t alignment)
{
	bus_dmamap_t map;
	bus_size_t physaddr = 0;
	void *ptr;

	if(bus_dmamem_alloc
	   (tag, &ptr, (BUS_DMA_WAITOK|BUS_DMA_COHERENT), &map))
	{
		return NULL;
	}

	if(bus_dmamap_load
	   (tag, map, ptr, size, &bus_dmamap_load_callback, 
	    &physaddr, (BUS_DMA_WAITOK|BUS_DMA_COHERENT)))
	{
		bus_dmamem_free(tag, ptr, map);
		return NULL;
	}

	page->tag = tag;
	page->map = map;
	page->physaddr = physaddr;
	page->buffer = ptr;
	page->length = size;
	page->exit_level = 0;
	page->intr_temp = 0;

	bzero(ptr, size);

	bus_dmamap_sync(page->tag, page->map, 
			BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);

#ifdef USB_DEBUG
	if(usbdebug > 14)
	{
	    printf("%s: %p, %d bytes, phys=%p\n", 
		   __FUNCTION__, ptr, size, 
		   ((char *)0) + physaddr);
	}
#endif
	return ptr;
}

/*------------------------------------------------------------------------------*
 *  usbd_mem_free_sub - free DMA-able memory
 *------------------------------------------------------------------------------*/
void
usbd_mem_free_sub(struct usbd_page *page)
{
	/* NOTE: make a copy of "tag", "map", 
	 * and "buffer" in case "page" is part 
	 * of the allocated memory:
	 */
	bus_dma_tag_t tag = page->tag;
	bus_dmamap_t map = page->map;
	void *ptr = page->buffer;

	if (page->exit_level == 0) {
	    bus_dmamap_sync(page->tag, page->map, 
			    BUS_DMASYNC_POSTWRITE|BUS_DMASYNC_POSTREAD);
	} else {
	    panic("%s:%d: exit_level is not zero!\n",
		  __FUNCTION__, __LINE__);
	}

	bus_dmamap_unload(tag, map);

	bus_dmamem_free(tag, ptr, map);

#ifdef USB_DEBUG
	if(usbdebug > 14)
	{
	    printf("%s: %p\n", 
		   __FUNCTION__, ptr);
	}
#endif
	return;
}

void
usbd_page_dma_exit(struct usbd_page *page)
{
	if ((page->exit_level)++ == 0) {
		/*
		 * Disable interrupts so that the
		 * PCI controller can continue
		 * using the memory as quick as
		 * possible:
		 */
		page->intr_temp = intr_disable();
		bus_dmamap_sync(page->tag, page->map, 
				BUS_DMASYNC_POSTWRITE|BUS_DMASYNC_POSTREAD);
	}
	return;
}

void
usbd_page_dma_enter(struct usbd_page *page)
{
	if (--(page->exit_level) == 0) {
		bus_dmamap_sync(page->tag, page->map, 
				BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);
		intr_restore(page->intr_temp);
	}
	return;
}
#endif

#ifdef __NetBSD__

bus_dma_tag_t 
usbd_dma_tag_alloc(bus_dma_tag_t parent, u_int32_t size, 
		   u_int32_t alignment)
{
	/* FreeBSD specific */
	return parent;
}

void
usbd_dma_tag_free(bus_dma_tag_t tag)
{
	return;
}

void *
usbd_mem_alloc_sub(bus_dma_tag_t tag, struct usbd_page *page,
		   u_int32_t size, u_int32_t alignment)
{
	caddr_t ptr = NULL;

	page->tag = tag;
	page->seg_count = 1;

	if(bus_dmamem_alloc(page->tag, size, alignment, 0,
			    &page->seg, 1,
			    &page->seg_count, BUS_DMA_WAITOK))
	{
		goto done_4;
	}

	if(bus_dmamem_map(page->tag, &page->seg, page->seg_count, size,
			  &ptr, BUS_DMA_WAITOK|BUS_DMA_COHERENT))
	{
		goto done_3;
	}

	if(bus_dmamap_create(page->tag, size, 1, size,
			     0, BUS_DMA_WAITOK, &page->map))
	{
		goto done_2;
	}

	if(bus_dmamap_load(page->tag, page->map, ptr, size, NULL, 
			   BUS_DMA_WAITOK))
	{
		goto done_1;
	}

	page->physaddr = page->map->dm_segs[0].ds_addr;
	page->buffer = ptr;
	page->length = size;
	page->exit_level = 0;
	page->intr_temp = 0;

	bzero(ptr, size);

	bus_dmamap_sync(page->tag, page->map, 0, page->length, 
			BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);

#ifdef USB_DEBUG
	if(usbdebug > 14)
	{
	    printf("%s: %p, %d bytes, phys=%p\n", 
		   __FUNCTION__, ptr, size, 
		   ((char *)0) + page->physaddr);
	}
#endif
	return ptr;

 done_1:
	bus_dmamap_destroy(page->tag, page->map);

 done_2:
	bus_dmamem_unmap(page->tag, ptr, size);

 done_3:
	bus_dmamem_free(page->tag, &page->seg, page->seg_count);

 done_4:
	return NULL;
}

void
usbd_mem_free_sub(struct usbd_page *page)
{
	/* NOTE: make a copy of "tag", "map", 
	 * and "buffer" in case "page" is part 
	 * of the allocated memory:
	 */
	struct usbd_page temp = *page;

	if (temp.exit_level == 0) {
	    bus_dmamap_sync(temp.tag, temp.map, 0, temp.length, 
			    BUS_DMASYNC_POSTWRITE|BUS_DMASYNC_POSTREAD);
	} else {
	    panic("%s:%d: exit_level is not zero!\n",
		  __FUNCTION__, __LINE__);
	}

	bus_dmamap_unload(temp.tag, temp.map);
	bus_dmamap_destroy(temp.tag, temp.map);
	bus_dmamem_unmap(temp.tag, temp.buffer, temp.length);
	bus_dmamem_free(temp.tag, &temp.seg, temp.seg_count);

#ifdef USB_DEBUG
	if(usbdebug > 14)
	{
	    printf("%s: %p\n", 
		   __FUNCTION__, temp.buffer);
	}
#endif
	return;
}

void
usbd_page_dma_exit(struct usbd_page *page)
{
	if ((page->exit_level)++ == 0) {
		/*
		 * Disable interrupts so that the
		 * PCI controller can continue
		 * using the memory as quick as
		 * possible:
		 */
		page->intr_temp = intr_disable();
		bus_dmamap_sync(page->tag, page->map, 0, page->length,
				BUS_DMASYNC_POSTWRITE|BUS_DMASYNC_POSTREAD);
	}
	return;
}

void
usbd_page_dma_enter(struct usbd_page *page)
{
	if (--(page->exit_level) == 0) {
		bus_dmamap_sync(page->tag, page->map, 0, page->length,
				BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);
		intr_restore(page->intr_temp);
	}
	return;
}
#endif

/*------------------------------------------------------------------------------*
 *  usbd_std_transfer_setup - standard transfer setup
 *------------------------------------------------------------------------------*/
void
usbd_std_transfer_setup(struct usbd_device *udev,
			struct usbd_xfer *xfer, 
			const struct usbd_config *setup, 
			u_int16_t max_packet_size, u_int16_t max_frame_size,
			uint8_t max_packet_count)
{
	usb_endpoint_descriptor_t *edesc = xfer->pipe->edesc;
	uint8_t type;

	__callout_init_mtx(&xfer->timeout_handle, xfer->usb_mtx,
			   CALLOUT_RETURNUNLOCKED);

	xfer->flags = setup->flags;
	xfer->nframes = setup->frames;
	xfer->timeout = setup->timeout;
	xfer->callback = setup->callback;
	xfer->interval = setup->interval;
	xfer->endpoint = edesc->bEndpointAddress;
	xfer->max_packet_size = usbd_get_max_packet_size(edesc);
	xfer->max_packet_count = usbd_get_max_packet_count(edesc);
	xfer->max_frame_size = usbd_get_max_frame_size(edesc);
	xfer->length = setup->bufsize;

	/* check interrupt interval and transfer pre-delay */

	type = (edesc->bmAttributes & UE_XFERTYPE);

	if (type == UE_ISOCHRONOUS) {
	    xfer->interval = 0; /* not used, must be zero */

	    if (xfer->timeout == 0) {
	        /* set a default timeout in 
		 * case something goes wrong!
		 */
	        xfer->timeout = 1000 / 4;
	    }

	} else {
	    /* if a value is specified use that
	     * else check the endpoint descriptor
	     */
	    if (xfer->interval == 0) {

	        if (type == UE_INTERRUPT) {

		    xfer->interval = edesc->bInterval;

		    if (usbd_get_speed(udev) == USB_SPEED_HIGH) {
		        xfer->interval /= 8; /* 125us -> 1ms */
		    }

		    if (xfer->interval == 0) {
		        /* one millisecond is the smallest interval */
		        xfer->interval = 1;
		    }
		}
	    }
	}

	/* wMaxPacketSize is also checked by "usbd_fill_iface_data()" */

	if (xfer->max_packet_size == 0) {
	    xfer->max_packet_size = 8;
	}

	if (xfer->max_packet_size > max_packet_size) {
	    xfer->max_packet_size = max_packet_size;
	}

	/* check the maximum packet count */

	if (xfer->max_packet_count == 0) {
	    xfer->max_packet_count = 1;
	}

	if (xfer->max_packet_count > max_packet_count) {
	    xfer->max_packet_count = max_packet_count;
	}

	/* check the maximum frame size */

	if (xfer->max_frame_size > max_frame_size) {
	    xfer->max_frame_size = max_frame_size;
	}

	if (xfer->length == 0) {
	    xfer->length = xfer->max_frame_size;
	    if (xfer->nframes) {
	        xfer->length *= xfer->nframes;
	    }
	}
	return;
}

/*------------------------------------------------------------------------------*
 *  usbd_make_str_desc - convert an ASCII string into a UNICODE string
 *------------------------------------------------------------------------------*/
u_int8_t
usbd_make_str_desc(void *ptr, u_int16_t max_len, const char *s)
{
	usb_string_descriptor_t *p = ptr;
	u_int8_t totlen;
	int32_t j;

	if (max_len < 2) {
	    /* invalid length */
	    return 0;
	}

	max_len = ((max_len / 2) - 1);

	j = strlen(s);

	if (j < 0) {
	    j = 0;
	}

	if (j > 126) {
	    j = 126;
	}

	if (max_len > j) {
	    max_len = j;
	}

	totlen = (max_len + 1) * 2;

	p->bLength = totlen;
	p->bDescriptorType = UDESC_STRING;

	while (max_len--) {
	    USETW2(p->bString[max_len], 0, s[max_len]);
	}
	return totlen;
}

/*---------------------------------------------------------------------------*
 * mtx_drop_recurse - drop mutex recurse level
 *---------------------------------------------------------------------------*/
u_int32_t
mtx_drop_recurse(struct mtx *mtx)
{
	u_int32_t recurse_level = mtx->mtx_recurse;
	u_int32_t recurse_curr = recurse_level;

	mtx_assert(mtx, MA_OWNED);

	while(recurse_curr--) {
	    mtx_unlock(mtx);
	}

	return recurse_level;
}

/*---------------------------------------------------------------------------*
 * mtx_pickup_recurse - pickup mutex recurse level
 *---------------------------------------------------------------------------*/
void
mtx_pickup_recurse(struct mtx *mtx, u_int32_t recurse_level)
{
	mtx_assert(mtx, MA_OWNED);

	while(recurse_level--) {
	    mtx_lock(mtx);
	}
	return;
}


/*---------------------------------------------------------------------------*
 * usbd_config_thread
 *---------------------------------------------------------------------------*/
static void
usbd_config_td_thread(void *arg)
{
	struct usbd_config_td *ctd = arg;
	struct usbd_config_td_item *item;
	struct usbd_mbuf *m;
	register int error;

	mtx_lock(ctd->p_mtx);

	while(1) {

	    if (ctd->flag_config_td_gone) {
	        break;
	    }

	    /* NOTE to reimplementors: dequeueing a command from the
	     * "used" queue and executing it must be atomic, with
	     * regard to the "p_mtx" mutex. That means any attempt to
	     * queue a command by another thread must be blocked until
	     * either:
	     *
	     * 1) the command sleeps
	     *
	     * 2) the command returns
	     *
	     * Here is a practical example that shows how this
	     * helps solving a problem:
	     *
	     * Assume that you want to set the baud rate on a USB
	     * serial device. During the programming of the device you
	     * don't want to receive nor transmit any data, because it
	     * will be garbage most likely anyway. The programming of
	     * our USB device takes 20 milliseconds and it needs to
	     * call functions that sleep.
	     *
	     * Non-working solution: Before we queue the programming command,
	     * we stop transmission and reception of data. Then we
	     * queue a programming command. At the end of the programming
	     * command we enable transmission and reception of data.
	     *
	     * Problem: If a second programming command is queued
	     * while the first one is sleeping, we end up enabling
	     * transmission and reception of data too early.
	     *
	     * Working solution: Before we queue the programming
	     * command, we stop transmission and reception of
	     * data. Then we queue a programming command. Then we
	     * queue a second command that only enables transmission
	     * and reception of data.
	     *
	     * Why it works: If a second programming command is queued
	     * while the first one is sleeping, then the queueing of a
	     * second command to enable the data transfers, will cause
	     * the previous one, which is still on the queue, to be
	     * removed from the queue, and re-inserted after the last
	     * baud rate programming command, which then gives the
	     * desired result.
	     *
	     * This example assumes that you use a "qcount" of zero.
	     */

	    USBD_IF_DEQUEUE(&(ctd->cmd_used), m);

	    if (m) {

	        item = (void *)(m->cur_data_ptr);

		(item->command_func)
		  (ctd->p_softc, (void *)(item+1), item->command_ref);

		USBD_IF_ENQUEUE(&(ctd->cmd_free), m);

		continue;
	    }

	    if (ctd->p_end_of_commands) {
	        (ctd->p_end_of_commands)(ctd->p_softc);
	    }

	    ctd->flag_config_td_sleep = 1;

	    error = mtx_sleep(&(ctd->wakeup_config_td), ctd->p_mtx,
			      0, "cfg td sleep", 0);

	    ctd->flag_config_td_sleep = 0;
	}

	ctd->config_thread = NULL;

	wakeup(&(ctd->wakeup_config_td_gone));

	mtx_unlock(ctd->p_mtx);

	kthread_exit(0);

	return;
}

/*---------------------------------------------------------------------------*
 * usbd_config_td_setup
 *
 * NOTE: the structure pointed to by "ctd" must be zeroed before calling 
 * this function!
 *
 * Return values:
 *    0: success
 * else: failure
 *---------------------------------------------------------------------------*/
u_int8_t
usbd_config_td_setup(struct usbd_config_td *ctd, void *priv_sc, 
		     struct mtx *priv_mtx, 
		     usbd_config_td_end_of_commands_t *p_func_eoc,
		     u_int16_t item_size, u_int16_t item_count)
{
	ctd->p_mtx = priv_mtx;
	ctd->p_softc = priv_sc;
	ctd->p_end_of_commands = p_func_eoc;

	if (item_count >= 256) {
	    PRINTFN(0,("too many items!\n"));
	    goto error;
	}

	ctd->p_cmd_queue = 
	  usbd_alloc_mbufs(M_DEVBUF, &(ctd->cmd_free), 
			   (sizeof(struct usbd_config_td_item) + item_size), 
			   item_count);

	if (ctd->p_cmd_queue == NULL) {
	    PRINTFN(0,("unable to allocate memory "
		       "for command queue!\n"));
	    goto error;
	}

	if (usb_kthread_create1
	    (&usbd_config_td_thread, ctd, &(ctd->config_thread), 
	     "usbd config thread")) {
	    PRINTFN(0,("unable to create config thread!\n"));
	    ctd->config_thread = NULL;
	    goto error;
	}
	return 0;

 error:
	usbd_config_td_unsetup(ctd);
	return 1;
}

/*---------------------------------------------------------------------------*
 * usbd_config_td_dummy_cmd
 *---------------------------------------------------------------------------*/
static void
usbd_config_td_dummy_cmd(struct usbd_config_td_softc *sc, 
			 struct usbd_config_td_cc *cc, 
			 u_int16_t reference)
{
	return;
}

/*---------------------------------------------------------------------------*
 * usbd_config_td_stop
 *
 * NOTE: If the structure pointed to by "ctd" is all zero,
 * this function does nothing.
 *---------------------------------------------------------------------------*/
void
usbd_config_td_stop(struct usbd_config_td *ctd)
{
	uint32_t level;
	int error;

	if (ctd->p_mtx) {

	  mtx_lock(ctd->p_mtx);

	  while (ctd->config_thread) {

	    usbd_config_td_queue_command(ctd, NULL, &usbd_config_td_dummy_cmd, 
					 0, 0);

	    /* set the gone flag after queueing the
	     * last command:
	     */
	    ctd->flag_config_td_gone = 1;

	    if (cold) {
	        panic("%s:%d: cannot stop config thread!\n",
		      __FUNCTION__, __LINE__);
	    }

	    level = mtx_drop_recurse(ctd->p_mtx);

	    error = mtx_sleep(&(ctd->wakeup_config_td_gone), 
			      ctd->p_mtx, 0, "wait config TD", 0);

	    mtx_pickup_recurse(ctd->p_mtx, level);
	  }

	  mtx_unlock(ctd->p_mtx);
	}
	return;
}

/*---------------------------------------------------------------------------*
 * usbd_config_td_unsetup
 *
 * NOTE: If the structure pointed to by "ctd" is all zero,
 * this function does nothing.
 *---------------------------------------------------------------------------*/
void
usbd_config_td_unsetup(struct usbd_config_td *ctd)
{
	usbd_config_td_stop(ctd);

	if (ctd->p_cmd_queue) {
	    free(ctd->p_cmd_queue, M_DEVBUF);
	    ctd->p_cmd_queue = NULL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 * usbd_config_td_queue_command
 *---------------------------------------------------------------------------*/
void
usbd_config_td_queue_command(struct usbd_config_td *ctd,
			     usbd_config_td_command_t *command_pre_func,
			     usbd_config_td_command_t *command_post_func,
			     u_int16_t command_qcount,
			     u_int16_t command_ref)
{
	struct usbd_config_td_item *item;
	struct usbd_mbuf *m;
	int32_t qlen;

	if (usbd_config_td_is_gone(ctd)) {
	    /* nothing more to do */
	    return;
	}

	/*
	 * first check if the command was
	 * already queued, and if so, remove
	 * it from the queue:
	 */
	qlen = USBD_IF_QLEN(&(ctd->cmd_used));

	while (qlen--) {

	    USBD_IF_DEQUEUE(&(ctd->cmd_used), m);

	    if (m == NULL) {
	        /* should not happen */
	        break;
	    }

	    item = (void *)(m->cur_data_ptr);

	    if ((item->command_func == command_post_func) &&
		(item->command_ref == command_ref)) {
	        if (command_qcount == 0) {
		  USBD_IF_ENQUEUE(&(ctd->cmd_free), m);
		  continue;
		}
		command_qcount--;
	    }
	    USBD_IF_ENQUEUE(&(ctd->cmd_used), m);
	}

	USBD_IF_DEQUEUE(&(ctd->cmd_free), m);

	if (m == NULL) {
	    /* should not happen */
	    panic("%s:%d: out of memory!\n",
		  __FUNCTION__, __LINE__);
	}

	USBD_MBUF_RESET(m);

	item = (void *)(m->cur_data_ptr);

	/* The job of the post-command
	 * function is to finish the command
	 * in a separate context to allow calls
	 * to sleeping functions basically.
	 * Queue the post command before calling
	 * the pre command. That way commands 
	 * queued by the pre command will be
	 * queued after the current command.
	 */
	item->command_func = command_post_func;
	item->command_ref = command_ref;

	USBD_IF_ENQUEUE(&(ctd->cmd_used), m);

	/* The job of the pre-command
	 * function is to copy the needed 
	 * configuration to the provided
	 * structure and to execute other
	 * commands that must happen
	 * immediately
	 */
	if (command_pre_func) {
	    (command_pre_func)(ctd->p_softc, (void *)(item+1), command_ref);
	}

	/* Currently we use a separate thread
	 * to execute the command, but it is not
	 * impossible that we might use
	 * a so called taskqueue in the future:
	 */
	if (ctd->flag_config_td_sleep) {
	    ctd->flag_config_td_sleep = 0;
	    wakeup(&(ctd->wakeup_config_td));
	}
	return;
}

/*---------------------------------------------------------------------------*
 * usbd_config_td_is_gone
 *
 * Return values:
 *    0: config thread is running
 * else: config thread is gone
 *---------------------------------------------------------------------------*/
u_int8_t
usbd_config_td_is_gone(struct usbd_config_td *ctd)
{
	mtx_assert(ctd->p_mtx, MA_OWNED);

	return ctd->flag_config_td_gone ? 1 : 0;
}

/*---------------------------------------------------------------------------*
 * usbd_config_td_sleep
 *
 * NOTE: this function can only be called from the config thread
 *
 * Return values:
 *    0: normal delay
 * else: config thread is gone
 *---------------------------------------------------------------------------*/
u_int8_t
usbd_config_td_sleep(struct usbd_config_td *ctd, u_int32_t timeout)
{
	register int error;
	u_int8_t is_gone = usbd_config_td_is_gone(ctd);
	u_int32_t level;

	if (is_gone) {
	    goto done;
	}

	if (timeout == 0) {
	    /* zero means no timeout, 
	     * so avoid that by setting
	     * timeout to one:
	     */
	    timeout = 1;
	}

	level = mtx_drop_recurse(ctd->p_mtx);

	error = mtx_sleep(ctd, ctd->p_mtx, 0, 
			  "config td sleep", timeout);

	mtx_pickup_recurse(ctd->p_mtx, level);

 done:
	return is_gone;
}

/*---------------------------------------------------------------------------*
 * usbd_ether_get_mbuf - get a new ethernet mbuf
 *---------------------------------------------------------------------------*/
struct mbuf *
usbd_ether_get_mbuf(void)
{
	register struct mbuf *m_new;

	m_new = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR);
	if (m_new) {
	    m_new->m_len = m_new->m_pkthdr.len = MCLBYTES;
	    m_adj(m_new, ETHER_ALIGN);
	}
	return (m_new);
}

/*---------------------------------------------------------------------------*
 * device_delete_all_children - delete all children of a device
 *---------------------------------------------------------------------------*/
int32_t
device_delete_all_children(device_t dev)
{
	device_t *devlist;
	int32_t devcount;
	int32_t error;

	error = device_get_children(dev, &devlist, &devcount);
	if (error == 0) {
	    while (devcount-- > 0) {
	        error = device_delete_child(dev, devlist[devcount]);
		if (error) {
		    break;
		}
	    }
	    free(devlist, M_TEMP);
	}
	return error;
}

/*---------------------------------------------------------------------------*
 * usbd_get_max_packet_size - get maximum packet size
 *---------------------------------------------------------------------------*/
uint16_t
usbd_get_max_packet_size(usb_endpoint_descriptor_t *edesc)
{
	return (UGETW(edesc->wMaxPacketSize) & 0x7FF);
}

/*---------------------------------------------------------------------------*
 * usbd_get_max_packet_count - get maximum packet count
 *---------------------------------------------------------------------------*/
uint16_t
usbd_get_max_packet_count(usb_endpoint_descriptor_t *edesc)
{
	return (1 + ((UGETW(edesc->wMaxPacketSize) >> 11) & 3));
}

/*---------------------------------------------------------------------------*
 * usbd_get_max_frame_size - get maximum frame size
 *---------------------------------------------------------------------------*/
uint16_t
usbd_get_max_frame_size(usb_endpoint_descriptor_t *edesc)
{
	uint16_t n;
	n = UGETW(edesc->wMaxPacketSize);
	return (n & 0x7FF) * (1 + ((n >> 11) & 3));
}

/*---------------------------------------------------------------------------*
 * usbd_set_max_packet_size_count - set maximum packet size and count
 *---------------------------------------------------------------------------*/
void
usbd_set_max_packet_size_count(usb_endpoint_descriptor_t *edesc,
			       uint16_t size, uint16_t count)
{
	uint16_t n;
	n = (size & 0x7FF)|(((count-1) & 3) << 11);
	USETW(edesc->wMaxPacketSize, n);
	return;
}

/*---------------------------------------------------------------------------*
 * usbd_isoc_time_expand - expand time counter from 7-bit to 16-bit
 *---------------------------------------------------------------------------*/
uint16_t
usbd_isoc_time_expand(struct usbd_bus *bus, uint16_t isoc_time_curr)
{
	uint16_t rem;

	mtx_assert(&(bus->mtx), MA_OWNED);

	rem = bus->isoc_time_last & (USB_ISOC_TIME_MAX-1);

	isoc_time_curr &= (USB_ISOC_TIME_MAX-1);

	if (isoc_time_curr < rem) {
	    /* the time counter wrapped around */
	    bus->isoc_time_last += USB_ISOC_TIME_MAX;
	}

	/* update the remainder */

	bus->isoc_time_last &= ~(USB_ISOC_TIME_MAX-1);
	bus->isoc_time_last |= isoc_time_curr;

	return bus->isoc_time_last;
}
