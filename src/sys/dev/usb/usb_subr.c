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
__FBSDID("$FreeBSD: src/sys/dev/usb/usb_subr.c,v 1.95 2007/06/30 20:18:44 imp Exp $");

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
#include <sys/queue.h>			/* LIST_XXX() */
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/kthread.h>
#include <sys/unistd.h>
#include <sys/uio.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_hid.h>
#include "usbdevs.h"
#include <dev/usb/usb_quirks.h>
#include <dev/usb/usb_template.h>

#ifdef USBVERBOSE
/*
 * Descriptions of of known vendors and devices ("products").
 */
struct usb_knowndev {
	uint16_t vendor;
	uint16_t product;
	int	flags;
	char   *vendorname, *productname;
};

#define	USB_KNOWNDEV_NOPROD	0x01	/* match on vendor only */

#include "usbdevs_data.h"
#endif					/* USBVERBOSE */

static void
usbd_trim_spaces(char *p)
{
	char *q;
	char *e;

	if (p == NULL)
		return;
	q = e = p;
	while (*q == ' ')		/* skip leading spaces */
		q++;
	while ((*p = *q++))		/* copy string */
		if (*p++ != ' ')	/* remember last non-space */
			e = p;
	*e = 0;				/* kill trailing spaces */
	return;
}

static void
usbd_finish_vp_info(struct usbd_device *udev)
{
	usb_device_descriptor_t *udd = &udev->ddesc;
	uint8_t *vendor;
	uint8_t *product;

#ifdef USBVERBOSE
	const struct usb_knowndev *kdp;

#endif
	uint16_t vendor_id;
	uint16_t product_id;

	usbd_trim_spaces(udev->manufacturer);
	usbd_trim_spaces(udev->product);

	if (udev->manufacturer[0]) {
		vendor = udev->manufacturer;
	} else {
		vendor = NULL;
	}

	if (udev->product[0]) {
		product = udev->product;
	} else {
		product = NULL;
	}

	vendor_id = UGETW(udd->idVendor);
	product_id = UGETW(udd->idProduct);

#ifdef USBVERBOSE
	if (vendor == NULL || product == NULL) {

		for (kdp = usb_knowndevs;
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
		if (udev->manufacturer != vendor) {
			strlcpy(udev->manufacturer, vendor,
			    sizeof(udev->manufacturer));
		}
	} else {
		snprintf(udev->manufacturer,
		    sizeof(udev->manufacturer), "vendor 0x%04x", vendor_id);
	}

	if (product && *product) {
		if (udev->product != product) {
			strlcpy(udev->product, product,
			    sizeof(udev->product));
		}
	} else {
		snprintf(udev->product,
		    sizeof(udev->product), "product 0x%04x", product_id);
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
usbd_devinfo(struct usbd_device *udev, char *dst_ptr, uint16_t dst_len)
{
	usb_device_descriptor_t *udd = &udev->ddesc;
	uint16_t bcdDevice;
	uint16_t bcdUSB;

	bcdUSB = UGETW(udd->bcdUSB);
	bcdDevice = UGETW(udd->bcdDevice);

	if (udd->bDeviceClass != 0xFF) {
		snprintf(dst_ptr, dst_len, "%s %s, class %d/%d, rev %x.%02x/"
		    "%x.%02x, addr %d", udev->manufacturer, udev->product,
		    udd->bDeviceClass, udd->bDeviceSubClass,
		    (bcdUSB >> 8), bcdUSB & 0xFF,
		    (bcdDevice >> 8), bcdDevice & 0xFF,
		    udev->address);
	} else {
		snprintf(dst_ptr, dst_len, "%s %s, rev %x.%02x/"
		    "%x.%02x, addr %d", udev->manufacturer, udev->product,
		    (bcdUSB >> 8), bcdUSB & 0xFF,
		    (bcdDevice >> 8), bcdDevice & 0xFF,
		    udev->address);
	}
	return;
}

const char *
usbd_errstr(usbd_status_t err)
{
	static const char *const
	      MAKE_TABLE(USBD_STATUS, DESC,[]);

	return (err < N_USBD_STATUS) ?
	    USBD_STATUS_DESC[err] : "unknown error!";
}

/* Delay for a certain number of ms */
void
usb_delay_ms(struct usbd_bus *bus, uint32_t ms)
{
	/* Wait at least two clock ticks so we know the time has passed. */
	if (cold)
		DELAY((ms + 1) * 1000);
	else
#if (__FreeBSD_version >= 700031)
		pause("usbdly", (((ms * hz) + 999) / 1000) + 1);
#else
		tsleep(&ms, PRIBIO, "usbdly", (((ms * hz) + 999) / 1000) + 1);
#endif
}

/* Delay given a device handle. */
void
usbd_delay_ms(struct usbd_device *udev, uint32_t ms)
{
	usb_delay_ms(udev->bus, ms);
}

/*------------------------------------------------------------------------*
 *	 usbd_pause_mtx - factored out code
 *
 * NOTE: number of milliseconds per second is 1024 for sake of optimisation
 *------------------------------------------------------------------------*/
void
usbd_pause_mtx(struct mtx *mtx, uint32_t ms)
{
	if (cold) {
		ms = (ms + 1) * 1024;
		DELAY(ms);

	} else {

		ms = USBD_MS_TO_TICKS(ms);
		ms++;			/* be sure that we don't return too
					 * early */

		if (mtx_sleep(&ms, mtx, 0, "pause_mtx", ms)) {
			/* should not happen */
		}
	}
	return;
}

usb_descriptor_t *
usbd_desc_foreach(usb_config_descriptor_t *cd, usb_descriptor_t *desc)
{
	void *end;

	if (cd == NULL) {
		return (NULL);
	}
	end = USBD_ADD_BYTES(cd, UGETW(cd->wTotalLength));

	if (desc == NULL) {
		desc = USBD_ADD_BYTES(cd, 0);
	} else {
		desc = USBD_ADD_BYTES(desc, desc->bLength);
	}
	return (((((void *)desc) >= ((void *)cd)) &&
	    (((void *)desc) < end) &&
	    (USBD_ADD_BYTES(desc, desc->bLength) >= ((void *)cd)) &&
	    (USBD_ADD_BYTES(desc, desc->bLength) <= end) &&
	    (desc->bLength >= sizeof(*desc))) ? desc : NULL);
}

usb_interface_descriptor_t *
usbd_find_idesc(usb_config_descriptor_t *cd,
    uint8_t iface_index, uint8_t alt_index)
{
	usb_descriptor_t *desc = NULL;
	usb_interface_descriptor_t *id;
	uint8_t curidx = 0;
	uint8_t lastidx = 0;
	uint8_t curaidx = 0;
	uint8_t first = 1;

	while ((desc = usbd_desc_foreach(cd, desc))) {
		if ((desc->bDescriptorType == UDESC_INTERFACE) &&
		    (desc->bLength >= sizeof(*id))) {
			id = (void *)desc;

			if (first) {
				first = 0;
				lastidx = id->bInterfaceNumber;

			} else if (id->bInterfaceNumber != lastidx) {

				lastidx = id->bInterfaceNumber;
				curidx++;
				curaidx = 0;

			} else {
				curaidx++;
			}

			if ((iface_index == curidx) && (alt_index == curaidx)) {
				return (id);
			}
		}
	}
	return (NULL);
}

usb_endpoint_descriptor_t *
usbd_find_edesc(usb_config_descriptor_t *cd,
    uint8_t iface_index, uint8_t alt_index, uint8_t ep_index)
{
	usb_descriptor_t *desc = NULL;
	usb_interface_descriptor_t *d;
	uint8_t curidx = 0;

	d = usbd_find_idesc(cd, iface_index, alt_index);
	if (d == NULL)
		return (NULL);

	if (ep_index >= d->bNumEndpoints)	/* quick exit */
		return (NULL);

	desc = ((void *)d);

	while ((desc = usbd_desc_foreach(cd, desc))) {

		if (desc->bDescriptorType == UDESC_INTERFACE) {
			break;
		}
		if (desc->bDescriptorType == UDESC_ENDPOINT) {

			if (curidx == ep_index) {
				if (desc->bLength < USB_ENDPOINT_DESCRIPTOR_SIZE) {
					/* endpoint index is invalid */
					break;
				}
				return ((void *)desc);
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
void   *
usbd_find_descriptor(struct usbd_device *udev, void *id, uint8_t iface_index,
    uint8_t type, uint8_t type_mask,
    uint8_t subtype, uint8_t subtype_mask)
{
	usb_descriptor_t *desc;
	usb_config_descriptor_t *cd;
	struct usbd_interface *iface;

	cd = usbd_get_config_descriptor(udev);
	if (cd == NULL) {
		return (NULL);
	}
	if (id == NULL) {
		iface = usbd_get_iface(udev, iface_index);
		if (iface == NULL) {
			return (NULL);
		}
		id = usbd_get_interface_descriptor(iface);
		if (id == NULL) {
			return (NULL);
		}
	}
	desc = (void *)id;

	while ((desc = usbd_desc_foreach(cd, desc))) {

		if (desc->bDescriptorType == UDESC_INTERFACE) {
			break;
		}
		if (((desc->bDescriptorType & type_mask) == type) &&
		    ((desc->bDescriptorSubtype & subtype_mask) == subtype)) {
			return (desc);
		}
	}
	return (NULL);
}

uint16_t
usbd_get_no_alts(usb_config_descriptor_t *cd, uint8_t ifaceno)
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
	return (n);
}

static void
usbd_fill_pipe_data(struct usbd_device *udev, uint8_t iface_index,
    usb_endpoint_descriptor_t *edesc, struct usbd_pipe *pipe)
{
	bzero(pipe, sizeof(*pipe));

	(udev->bus->methods->pipe_init) (udev, edesc, pipe);

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
 * transactions will be executed in the order they are queued!
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
	return (y);
}

uint8_t
usbd_intr_schedule_adjust(struct usbd_device *udev, int16_t len, uint8_t slot)
{
	struct usbd_bus *bus = udev->bus;
	struct usbd_hub *hub;

	mtx_assert(&(bus->mtx), MA_OWNED);

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
		/*
	         * The Host Controller Driver should have
	         * performed checks so that the lookup
	         * below does not result in a NULL pointer
	         * access.
	         */

		hub = bus->devices[udev->hs_hub_addr]->hub;
		if (slot >= USB_HS_MICRO_FRAMES_MAX) {
			slot = usbd_find_best_slot(hub->uframe_usage,
			    USB_FS_ISOC_UFRAME_MAX, 6);
		}
		hub->uframe_usage[slot] += len;
		bus->uframe_usage[slot] += len;
	}
	return (slot);
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
		fss++;
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

	hs_hub = udev->bus->devices[udev->hs_hub_addr]->hub;

	if (hs_hub != NULL) {

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
	return (isoc_time);
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
			fss->frame_slot++;
		}

		fss->frame_bytes -= len;
	}
	return (slot);
}

/*------------------------------------------------------------------------*
 *	usbd_free_pipe_data
 *
 * NOTE: The interface pipes should not be in use when
 * this function is called !
 *------------------------------------------------------------------------*/
static void
usbd_free_pipe_data(struct usbd_device *udev,
    uint8_t iface_index, uint8_t iface_mask)
{
	struct usbd_pipe *pipe = udev->pipes;
	struct usbd_pipe *pipe_end = udev->pipes_end;

	while (pipe != pipe_end) {
		if ((pipe->iface_index & iface_mask) == iface_index) {
			/* free pipe */
			pipe->edesc = NULL;
		}
		pipe++;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_fill_iface_data
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_fill_iface_data(struct usbd_device *udev,
    uint8_t iface_index, uint8_t alt_index)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	struct usbd_pipe *pipe = udev->pipes;
	struct usbd_pipe *pipe_end = udev->pipes_end;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed = NULL;
	usb_descriptor_t *desc;
	uint8_t nendpt;

	if (iface == NULL) {
		return (USBD_ERR_INVAL);
	}
	PRINTFN(4, ("iface_index=%d alt_index=%d\n",
	    iface_index, alt_index));

	/* mtx_assert() */

	while (pipe != pipe_end) {
		if (pipe->iface_index == iface_index) {
			if (pipe->refcount) {
				return (USBD_ERR_IN_USE);
			}
		}
		pipe++;
	}

	pipe = &udev->pipes[0];

	/* free old pipes if any */
	usbd_free_pipe_data(udev, iface_index, 0 - 1);

	id = usbd_find_idesc(udev->cdesc, iface_index, alt_index);
	if (id == NULL) {
		return (USBD_ERR_INVAL);
	}
	iface->idesc = id;
	iface->alt_index = alt_index;

	USBD_CLR_IFACE_NO_PROBE(udev, iface_index);

	nendpt = id->bNumEndpoints;
	PRINTFN(4, ("found idesc nendpt=%d\n", nendpt));

	desc = (void *)id;

	while (nendpt--) {
		PRINTFN(10, ("endpt=%d\n", nendpt));

		while ((desc = usbd_desc_foreach(udev->cdesc, desc))) {
			if ((desc->bDescriptorType == UDESC_ENDPOINT) &&
			    (desc->bLength >= USB_ENDPOINT_DESCRIPTOR_SIZE)) {
				goto found;
			}
			if (desc->bDescriptorType == UDESC_INTERFACE) {
				break;
			}
		}
		goto error;

found:
		ed = (void *)desc;

		/* find a free pipe */
		while (pipe != pipe_end) {
			if (pipe->edesc == NULL) {
				/* pipe is free */
				usbd_fill_pipe_data(udev, iface_index, ed, pipe);
				break;
			}
			pipe++;
		}
	}
	return (USBD_ERR_NORMAL_COMPLETION);

error:
	/* passed end, or bad desc */
	PRINTFN(-1, ("%s: bad descriptor(s), addr=%d!\n",
	    __FUNCTION__, udev->address));

	/* free old pipes if any */
	usbd_free_pipe_data(udev, iface_index, 0 - 1);
	return (USBD_ERR_INVAL);
}

static void
usbd_free_iface_data(struct usbd_device *udev)
{
	struct usbd_interface *iface = udev->ifaces;
	struct usbd_interface *iface_end = udev->ifaces_end;

	/* mtx_assert() */

	/* free Linux compat device, if any */
	if (udev->linux_dev) {
		usb_linux_free_usb_device(udev->linux_dev);
		udev->linux_dev = NULL;
	}
	/* free all pipes, if any */
	usbd_free_pipe_data(udev, 0, 0);

	/* free all interfaces, if any */
	while (iface != iface_end) {
		iface->idesc = NULL;
		iface->alt_index = 0;
		iface++;
	}

	/* free "cdesc" after "ifaces", if any */
	if (udev->cdesc) {
		free(udev->cdesc, M_USB);
		udev->cdesc = NULL;
	}
	/* set unconfigured state */
	udev->curr_config_no = USB_UNCONFIG_NO;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_set_config_no
 *
 * This function will search all the configuration descriptors for a
 * matching configuration number. It is recommended to use
 * the function "usbd_set_config_index()" when the configuration
 * number does not matter.
 *
 * - USB config 0
 *   - USB interfaces
 *     - USB alternative interfaces
 *       - USB pipes
 *
 * - USB config 1
 *   - USB interfaces
 *     - USB alternative interfaces
 *       - USB pipes
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_set_config_no(struct usbd_device *udev, uint8_t no, uint8_t msg)
{
	usb_config_descriptor_t cd;
	usbd_status_t err;
	uint8_t index;

	if (no == USB_UNCONFIG_NO) {
		return (usbd_set_config_index(udev, USB_UNCONFIG_INDEX, msg));
	}
	PRINTFN(5, ("%d\n", no));

	/* figure out what config index to use */
	for (index = 0;
	    index < udev->ddesc.bNumConfigurations;
	    index++) {
		err = usbreq_get_config_desc(udev, &usb_global_lock, &cd, index);
		if (err) {
			return (err);
		}
		if (cd.bConfigurationValue == no) {
			return (usbd_set_config_index(udev, index, msg));
		}
	}
	return (USBD_ERR_INVAL);
}

/*------------------------------------------------------------------------*
 *	usbd_set_config_index
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_set_config_index(struct usbd_device *udev, uint8_t index, uint8_t msg)
{
	usb_status_t ds;
	usb_hub_descriptor_t hd;
	usb_config_descriptor_t cd;
	usb_config_descriptor_t *cdp;
	uint16_t len;
	uint16_t power;
	uint16_t max_power;
	uint8_t nifc;
	uint8_t selfpowered;
	usbd_status_t err;

	PRINTFN(5, ("udev=%p index=%d\n", udev, index));

	if (index == USB_UNCONFIG_INDEX) {
		/*
		 * leave unallocated when unconfiguring the device
		 */
		err = usbreq_set_config(udev, &usb_global_lock, USB_UNCONFIG_NO);
		goto error;
	}
	/* get the short descriptor */
	err = usbreq_get_config_desc(udev, &usb_global_lock, &cd, index);
	if (err) {
		goto error;
	}
	/* free all configuration data structures */
	usbd_free_iface_data(udev);

	/* get full descriptor */
	len = UGETW(cd.wTotalLength);
	udev->cdesc = malloc(len, M_USB, M_WAITOK | M_ZERO);
	if (udev->cdesc == NULL) {
		return (USBD_ERR_NOMEM);
	}
	cdp = udev->cdesc;

	/* Get the full descriptor. Try 3 times for slow devices. */

	err = usbreq_get_desc(udev, &usb_global_lock, cdp, len, len,
	    0, UDESC_CONFIG, index, 3);
	if (err)
		goto error;

	if (cdp->bNumInterface > (sizeof(udev->ifaces) / sizeof(udev->ifaces[0]))) {
		PRINTF(("too many interfaces: %d\n", cdp->bNumInterface));
		cdp->bNumInterface = (sizeof(udev->ifaces) / sizeof(udev->ifaces[0]));
	}
	/* Figure out if the device is self or bus powered. */
	selfpowered = 0;
	if (!(udev->quirks->uq_flags & UQ_BUS_POWERED) &&
	    (cdp->bmAttributes & UC_SELF_POWERED) &&
	    (udev->flags.usb_mode == USB_MODE_HOST)) {
		/* May be self powered. */
		if (cdp->bmAttributes & UC_BUS_POWERED) {
			/* Must ask device. */
			if (udev->quirks->uq_flags & UQ_POWER_CLAIM) {
				/*
				 * Hub claims to be self powered, but isn't.
				 * It seems that the power status can be
				 * determined by the hub characteristics.
				 */
				err = usbreq_get_hub_descriptor
				    (udev, &usb_global_lock, &hd);

				if (!err &&
				    (UGETW(hd.wHubCharacteristics) &
				    UHD_PWR_INDIVIDUAL)) {
					selfpowered = 1;
				}
				PRINTF(("characteristics=0x%04x, error=%s\n",
				    UGETW(hd.wHubCharacteristics),
				    usbd_errstr(err)));
			} else {
				err = usbreq_get_device_status
				    (udev, &usb_global_lock, &ds);

				if (!err &&
				    (UGETW(ds.wStatus) & UDS_SELF_POWERED)) {
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

	if (udev->parent_hub) {
		max_power = udev->parent_hub->hub->portpower;
	} else {
		max_power = USB_MAX_POWER;
	}

	if (power > max_power) {
		PRINTF(("power exceeded %d %d\n", power, max_power));

		/* XXX print nicer message */
		if (msg) {
			device_printf(udev->bus->bdev,
			    "device addr %d (config %d) exceeds power "
			    "budget, %d mA > %d mA\n",
			    udev->address,
			    cdp->bConfigurationValue,
			    power, max_power);
		}
		err = USBD_ERR_NO_POWER;
		goto error;
	}
	/* Only update "self_powered" in USB Host Mode */
	if (udev->flags.usb_mode == USB_MODE_HOST) {
		udev->flags.self_powered = selfpowered;
	}
	udev->power = power;
	udev->curr_config_no = cdp->bConfigurationValue;

	/* Set the actual configuration value. */
	err = usbreq_set_config(udev, &usb_global_lock,
	    cdp->bConfigurationValue);
	if (err) {
		goto error;
	}
	/* Allocate and fill interface data. */
	nifc = cdp->bNumInterface;
	while (nifc--) {
		err = usbd_fill_iface_data(udev, nifc, 0);
		if (err) {
			goto error;
		}
	}

	return (USBD_ERR_NORMAL_COMPLETION);

error:
	PRINTF(("error=%s\n", usbd_errstr(err)));
	usbd_free_iface_data(udev);
	return (err);
}

usbd_status_t
usbd_set_alt_interface_index(struct usbd_device *udev,
    uint8_t iface_index, uint8_t alt_index)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	usbd_status_t err;

	if (iface == NULL) {
		err = USBD_ERR_INVAL;
		goto done;
	}
	err = usbd_fill_iface_data(udev, iface_index, alt_index);
	if (err) {
		goto done;
	}
	err = usbreq_set_alt_interface_no
	    (udev, &usb_global_lock, iface_index,
	    iface->idesc->bAlternateSetting);

done:
	return (err);
}

int
usbd_fill_deviceinfo(struct usbd_device *udev, struct usb_device_info *di)
{
	enum {
		MAX_PORT = (sizeof(di->udi_ports) / sizeof(di->udi_ports[0])),
	};
	struct usbd_port *p;
	struct usbd_interface *iface;
	struct usbd_device *child;
	uint8_t i;
	uint8_t max;

	if ((udev == NULL) || (di == NULL)) {
		return (ENXIO);
	}
	bzero(di, sizeof(di[0]));

	mtx_lock(&usb_global_lock);

	di->udi_bus = device_get_unit(udev->bus->bdev);
	di->udi_addr = udev->address;
	strlcpy(di->udi_vendor, udev->manufacturer,
	    sizeof(di->udi_vendor));
	strlcpy(di->udi_product, udev->product,
	    sizeof(di->udi_product));
	usbd_printBCD(di->udi_release, sizeof(di->udi_release),
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

	if (udev->probed == USBD_PROBED_SPECIFIC_AND_FOUND) {
		if (udev->global_dev &&
		    device_is_attached(udev->global_dev)) {
			strlcpy(di->udi_devnames[0],
			    device_get_nameunit(udev->global_dev),
			    USB_MAX_DEVNAMELEN);
		}
	} else if (udev->probed == USBD_PROBED_IFACE_AND_FOUND) {
		for (i = 0; i != MIN(USB_MAX_DEVNAMES,
		    USB_MAX_INTERFACES); i++) {
			iface = usbd_get_iface(udev, i);
			if (iface && iface->subdev &&
			    device_is_attached(iface->subdev)) {
				strlcpy(di->udi_devnames[i],
				    device_get_nameunit(iface->subdev),
				    USB_MAX_DEVNAMELEN);
			}
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

			child = usbd_bus_port_get_device(udev->bus, p);

			if (child) {
				di->udi_ports[i] = p->device_index;
			} else {
				di->udi_ports[i] = USB_PORT_POWERED;
			}
		}
	}
	mtx_unlock(&usb_global_lock);
	return (0);
}

static void
usbd_reset_probed(struct usbd_device *udev)
{
	udev->probed = (udev->flags.usb_mode == USB_MODE_HOST) ?
	USBD_PROBED_NOTHING : USBD_PROBED_IFACE_AND_FOUND;
	return;
}

static void
usbd_detach_device_sub(struct usbd_device *udev, device_t *ppdev,
    uint8_t free_subdev)
{
	device_t dev;
	int err;

	if (!free_subdev) {

		*ppdev = NULL;

	} else if (*ppdev) {

		/*
		 * NOTE: It is important to clear "*ppdev" before deleting
		 * the child due to some device methods being called late
		 * during the delete process !
		 */
		dev = *ppdev;
		*ppdev = NULL;

		device_printf(dev, "at %s, port %d, addr %d "
		    "(disconnected)\n",
		    device_get_nameunit(udev->parent_dev),
		    udev->port_no, udev->address);

		if (device_is_attached(dev)) {
			if (udev->flags.suspended) {
				err = DEVICE_RESUME(dev);
				if (err) {
					device_printf(dev, "Resume failed!\n");
				}
			}
			if (device_detach(dev)) {
				goto error;
			}
		}
		if (device_delete_child(udev->parent_dev, dev)) {
			goto error;
		}
	}
	return;

error:
	/* Detach is not allowed to fail in the USB world */
	panic("An USB driver would not detach!\n");
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_detach_device
 *
 * The following function will detach the matching interfaces.
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_detach_device(struct usbd_device *udev, uint8_t iface_index,
    uint8_t free_subdev)
{
	struct usbd_interface *iface;
	uint8_t i;

	if (udev == NULL) {
		/* nothing to do */
		return;
	}
	PRINTFN(3, ("udev=%p\n", udev));

	/*
	 * First detach the child to give the child's detach routine a
	 * chance to detach the sub-devices in the correct order.
	 * Then delete the child using "device_delete_child()" which
	 * will detach all sub-devices from the bottom and upwards!
	 */
	if (iface_index != USB_IFACE_INDEX_ANY) {
		i = iface_index;
		iface_index = i + 1;
	} else {
		usbd_detach_device_sub(udev, &(udev->global_dev), free_subdev);
		i = 0;
		iface_index = USB_MAX_INTERFACES;
	}

	/* do the detach */

	for (; i != iface_index; i++) {

		iface = usbd_get_iface(udev, i);
		if (iface == NULL) {
			/* looks like the end of the USB interfaces */
			break;
		}
		usbd_detach_device_sub(udev, &(iface->subdev), free_subdev);
	}

	if (iface_index == USB_IFACE_INDEX_ANY) {
		/*
		 * All devices are gone. Reset the "probed" variable.
		 */
		usbd_reset_probed(udev);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_probe_and_attach_sub
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static uint8_t
usbd_probe_and_attach_sub(struct usbd_device *udev,
    struct usb_attach_arg *uaa, device_t *ppdev)
{
	device_t dev;
	int err;

	dev = *ppdev;

	if (dev) {

		/* clean up after module unload */

		if (device_is_attached(dev)) {
			/* already a device there */
			return (0);
		}
		/* XXX clear "*ppdev" as early as possible */

		*ppdev = NULL;

		if (device_delete_child(udev->parent_dev, dev)) {

			/*
			 * Panic here, else one can get a double call
			 * to device_detach().  USB devices should
			 * never fail on detach!
			 */
			panic("device_delete_child() failed!\n");
		}
	}
	if (uaa->temp_dev == NULL) {

		/* create a new child */
		uaa->temp_dev = device_add_child(udev->parent_dev, NULL, -1);
		if (uaa->temp_dev == NULL) {
			device_printf(udev->parent_dev,
			    "Device creation failed!\n");
			return (1);	/* failure */
		}
		device_set_ivars(uaa->temp_dev, uaa);
		device_quiet(uaa->temp_dev);
	}
	if (device_probe_and_attach(uaa->temp_dev) == 0) {
		/*
		 * The USB attach arguments are only available during probe
		 * and attach !
		 */
		*ppdev = uaa->temp_dev;
		uaa->temp_dev = NULL;
		device_set_ivars(*ppdev, NULL);

		if (udev->flags.suspended) {
			err = DEVICE_SUSPEND(*ppdev);
			device_printf(*ppdev, "Suspend failed\n");
		}
		return (0);		/* success */
	}
	return (1);			/* failure */
}

/*------------------------------------------------------------------------*
 *	usbd_probe_and_attach
 *
 * This function is called from "uhub_explore_sub()" and
 * "usbd_serve_request_callback_sub()"
 *
 * Returns:
 *    0: Success
 * Else: A control transfer failed
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_probe_and_attach(struct usbd_device *udev, uint8_t iface_index)
{
	struct usb_attach_arg uaa;
	struct usbd_interface *iface;
	usbd_status_t err;
	uint8_t nconfig;
	uint8_t config;
	uint8_t i;

	if (udev == NULL) {
		PRINTF(("udev == NULL\n"));
		return (USBD_ERR_INVAL);
	}
	if (udev->flags.usb_mode == USB_MODE_DEVICE) {
		if (udev->curr_config_no == USB_UNCONFIG_NO) {
			/* do nothing - no configuration has been set */
			return (0);
		}
	}
	if (udev->probed == USBD_PROBED_SPECIFIC_AND_FOUND) {
		if ((udev->global_dev == NULL) ||
		    (!device_is_attached(udev->global_dev))) {
			/* reset */
			udev->probed = USBD_PROBED_NOTHING;
		}
	}
	err = 0;
	config = 0;

	bzero(&uaa, sizeof(uaa));

	/* probe and attach */

	uaa.device = udev;
	uaa.usb_mode = udev->flags.usb_mode;
	uaa.port = udev->port_no;
	uaa.configno = -1;
	uaa.vendor = UGETW(udev->ddesc.idVendor);
	uaa.product = UGETW(udev->ddesc.idProduct);
	uaa.release = UGETW(udev->ddesc.bcdDevice);

	/* first try device specific drivers */
	if (udev->probed == USBD_PROBED_NOTHING) {
		PRINTF(("trying device specific drivers\n"));

		if (!usbd_probe_and_attach_sub(
		    udev, &uaa, &(udev->global_dev))) {
			udev->probed = USBD_PROBED_SPECIFIC_AND_FOUND;
			goto done;
		}
		PRINTF(("no device specific driver found; "
		    "looping over %d configurations\n",
		    udev->ddesc.bNumConfigurations));
	}
	/* next try the USB interface drivers */

	nconfig = udev->ddesc.bNumConfigurations;

	for (config = 0; config != nconfig; config++) {

		/*
		 * Only set the config index the first time the
		 * devices are probed !
		 */
		if (udev->probed == USBD_PROBED_NOTHING) {

			err = usbd_set_config_index(udev, config, 1);
			if (err) {
				goto done;
			}
		}
		/*
		 * else the configuration is already set
		 */

		if ((udev->probed == USBD_PROBED_NOTHING) ||
		    (udev->probed == USBD_PROBED_IFACE_AND_FOUND)) {

			uaa.configno = udev->cdesc->bConfigurationValue;

			/* check if only one interface should be probed */

			if (iface_index != USB_IFACE_INDEX_ANY) {
				i = iface_index;
				iface_index = i + 1;
			} else {
				i = 0;
				iface_index = USB_MAX_INTERFACES;
			}

			/* do the probe and attach */

			for (; i != iface_index; i++) {

				iface = usbd_get_iface(udev, i);
				if (iface == NULL) {
					/*
					 * Looks like the end of the USB
					 * interfaces !
					 */
					PRINTFN(1, ("end of interfaces "
					    "at %u\n", i));
					break;
				}
				if (USBD_GET_IFACE_NO_PROBE(udev, i)) {
					/* somebody grabbed the interface */
					PRINTFN(1, ("no probe %d\n", i));
					continue;
				}
				uaa.iface_index = i;
				uaa.iface = iface;

				if (!usbd_probe_and_attach_sub(
				    udev, &uaa, &(iface->subdev))) {
					udev->probed = USBD_PROBED_IFACE_AND_FOUND;
				}
			}
		}
		if (udev->probed != USBD_PROBED_NOTHING) {
			/* nothing more to do */
			break;
		}
	}

	if (udev->probed == USBD_PROBED_NOTHING) {
		/* set config index 0 */

		config = 0;
		err = usbd_set_config_index(udev, 0, 1);
		if (err) {
			goto done;
		}
		PRINTF(("no interface drivers found\n"));

		/* finally try the generic driver */
		uaa.iface = NULL;
		uaa.iface_index = 0;
		uaa.usegeneric = 1;
		uaa.configno = -1;

		if (!usbd_probe_and_attach_sub(
		    udev, &uaa, &(udev->global_dev))) {
			udev->probed = USBD_PROBED_SPECIFIC_AND_FOUND;
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
	if (err) {
		device_printf(udev->parent_dev,
		    "port %d, set config %d at addr %d "
		    "failed, error=%s\n",
		    udev->port_no, config, udev->address,
		    usbd_errstr(err));
	}
	if (uaa.temp_dev) {
		/* remove the last created child; it is unused */

		if (device_delete_child(udev->parent_dev, uaa.temp_dev)) {
			PRINTFN(-1, ("device delete child failed!\n"));
		}
	}
	return (err);
}

/*------------------------------------------------------------------------*
 *	usbd_suspend_resume_sub
 *------------------------------------------------------------------------*/
static void
usbd_suspend_resume_sub(struct usbd_device *udev, device_t dev, uint8_t do_suspend)
{
	int err;

	if (dev == NULL) {
		return;
	}
	if (!device_is_attached(dev)) {
		return;
	}
	if (do_suspend) {
		err = DEVICE_SUSPEND(dev);
	} else {
		err = DEVICE_RESUME(dev);
	}
	if (err) {
		device_printf(dev, "%s failed!\n",
		    do_suspend ? "Suspend" : "Resume");
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_suspend_resume_device
 *
 * The following function will suspend or resume the USB device.
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_suspend_resume(struct usbd_device *udev, uint8_t do_suspend)
{
	struct usbd_interface *iface;
	uint8_t i;

	if (udev == NULL) {
		/* nothing to do */
		return (0);
	}
	PRINTFN(3, ("udev=%p do_suspend=%d\n", udev, do_suspend));

	mtx_lock(&(udev->bus->mtx));
	if (udev->flags.suspended == do_suspend) {
		mtx_unlock(&(udev->bus->mtx));
		/* nothing to do */
		return (0);
	}
	udev->flags.suspended = do_suspend;
	mtx_unlock(&(udev->bus->mtx));

	/* do the global_dev first, if any */

	usbd_suspend_resume_sub(udev, udev->global_dev, do_suspend);

	/* do the suspend or resume */

	for (i = 0; i != USB_MAX_INTERFACES; i++) {

		iface = usbd_get_iface(udev, i);
		if (iface == NULL) {
			/* looks like the end of the USB interfaces */
			break;
		}
		usbd_suspend_resume_sub(udev, iface->subdev, do_suspend);
	}
	return (0);
}

static const uint8_t
	usbd_hub_speed_combs[USB_SPEED_MAX][USB_SPEED_MAX] = {
	/* HUB *//* subdevice */
	[USB_SPEED_HIGH][USB_SPEED_HIGH] = 1,
	[USB_SPEED_HIGH][USB_SPEED_FULL] = 1,
	[USB_SPEED_HIGH][USB_SPEED_LOW] = 1,
	[USB_SPEED_FULL][USB_SPEED_FULL] = 1,
	[USB_SPEED_FULL][USB_SPEED_LOW] = 1,
	[USB_SPEED_LOW][USB_SPEED_LOW] = 1,
};

/*------------------------------------------------------------------------*
 *	usbd_alloc_device
 *
 * This function allocates a new USB device. This function is called
 * when a new device has been put in the powered state, but not yet in
 * the addressed state. Get initial descriptor, set the address, get
 * full descriptor and get strings.
 *
 * Return values:
 *    0: Failure
 * Else: Success
 *------------------------------------------------------------------------*/
struct usbd_device *
usbd_alloc_device(device_t parent_dev, struct usbd_bus *bus,
    struct usbd_device *parent_hub, uint8_t depth,
    uint8_t port_index, uint8_t port_no, uint8_t speed, uint8_t usb_mode)
{
	struct usbd_device *udev;
	struct usbd_device *adev;
	struct usbd_device *hub;
	usbd_status_t err;
	uint8_t device_index;

	PRINTFN(0, ("parent_dev=%p, bus=%p, parent_hub=%p, depth=%u, "
	    "port_index=%u, port_no=%u, speed=%u, usb_mode=%u\n",
	    parent_dev, bus, parent_hub, depth, port_index, port_no,
	    speed, usb_mode));

	/*
	 * Find an unused device index. In USB Host mode this is the
	 * same as the device address.
	 *
	 * NOTE: Index 1 is reserved for the Root HUB.
	 */
	for (device_index = USB_ROOT_HUB_ADDR; device_index !=
	    USB_MAX_DEVICES; device_index++) {
		if (bus->devices[device_index] == NULL)
			break;
	}

	if (device_index == USB_MAX_DEVICES) {
		device_printf(bus->bdev,
		    "No free USB device index for new device!\n");
		return (NULL);
	}
	udev = malloc(sizeof(*udev), M_USB, M_WAITOK | M_ZERO);
	if (udev == NULL) {
		return (NULL);
	}
	/* initialize our SX-lock */
	sx_init(udev->default_sx, "USB device SX lock");

	/* initialize our mutex */
	mtx_init(udev->default_mtx, "USB device mutex", NULL, MTX_DEF);

	/* initialize some USB device fields */
	udev->parent_hub = parent_hub;
	udev->parent_dev = parent_dev;
	udev->port_index = port_index;
	udev->port_no = port_no;
	udev->depth = depth;
	udev->bus = bus;
	udev->address = USB_START_ADDR;	/* default value */

	/* we are not ready yet */
	udev->flags.detaching = 1;
	udev->refcount = 1;

	/* register our device */
	usbd_bus_port_set_device(bus, parent_hub ?
	    parent_hub->hub->ports + port_index : NULL, udev, device_index);

	/* set up default endpoint descriptor */
	udev->default_ep_desc.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE;
	udev->default_ep_desc.bDescriptorType = UDESC_ENDPOINT;
	udev->default_ep_desc.bEndpointAddress = USB_CONTROL_ENDPOINT;
	udev->default_ep_desc.bmAttributes = UE_CONTROL;
	udev->default_ep_desc.wMaxPacketSize[0] = USB_MAX_IPACKET;
	udev->default_ep_desc.wMaxPacketSize[1] = 0;
	udev->default_ep_desc.bInterval = 0;
	udev->ddesc.bMaxPacketSize = USB_MAX_IPACKET;

	udev->quirks = &usbd_no_quirk;
	udev->speed = speed;
	udev->flags.usb_mode = usb_mode;

	/* setup probed variable */

	usbd_reset_probed(udev);

	/* check speed combination */

	hub = udev->parent_hub;
	if (hub) {
		if (usbd_hub_speed_combs[hub->speed][speed] == 0) {
#ifdef USB_DEBUG
			printf("%s: the selected subdevice and HUB speed "
			    "combination is not supported %d/%d.\n",
			    __FUNCTION__, speed, hub->speed);
#endif
			/* reject this combination */
			err = USBD_ERR_INVAL;
			goto done;
		}
	}
	/* search for our High Speed USB HUB, if any */

	adev = udev;
	hub = udev->parent_hub;

	while (hub) {
		if (hub->speed == USB_SPEED_HIGH) {
			udev->hs_hub_addr = hub->address;
			udev->hs_port_no = adev->port_no;
			break;
		}
		adev = hub;
		hub = hub->parent_hub;
	}

	/* init the default pipe */
	usbd_fill_pipe_data(udev, 0,
	    &udev->default_ep_desc,
	    &udev->default_pipe);

	if (udev->flags.usb_mode == USB_MODE_HOST) {

		err = usbreq_set_address(udev, &usb_global_lock, device_index);

		/* This is the new USB device address from now on */

		udev->address = device_index;

		/*
		 * We ignore any set-address errors, hence there are
		 * buggy USB devices out there that actually receive
		 * the SETUP PID, but manage to set the address before
		 * the STATUS stage is ACK'ed. If the device responds
		 * to the subsequent get-descriptor at the new
		 * address, then we know that the set-address command
		 * was successful.
		 */
		if (err) {
			PRINTFN(-1, ("set address %d failed "
			    "(ignored)\n", udev->address));
		}
		/* allow device time to set new address */
		usbd_delay_ms(udev, USB_SET_ADDRESS_SETTLE);
	} else {
		/* We are not self powered */
		udev->flags.self_powered = 0;

		/*
	         * TODO: Make some kind of command that lets the user choose
	         * the USB template.
	         */
		err = usbd_temp_setup(udev, &usb_template_cdce);
		if (err) {
			PRINTFN(-1, ("setting up USB template failed\n"));
			goto done;
		}
	}

	/*
	 * Get the first 8 bytes of the device descriptor !
	 *
	 * NOTE: "usbd_do_request" will check the device descriptor
	 * next time we do a request to see if the maximum packet size
	 * changed! The 8 first bytes of the device descriptor
	 * contains the maximum packet size to use on control endpoint
	 * 0. If this value is different from "USB_MAX_IPACKET" a new
	 * USB control request will be setup!
	 */
	err = usbreq_get_desc(udev, &usb_global_lock, &udev->ddesc,
	    USB_MAX_IPACKET, USB_MAX_IPACKET, 0, UDESC_DEVICE, 0, 0);
	if (err) {
		PRINTFN(-1, ("getting device descriptor "
		    "at addr %d failed!\n", udev->address));
		goto done;
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

	/* get the full device descriptor */
	err = usbreq_get_device_desc(udev, &usb_global_lock, &udev->ddesc);
	if (err) {
		PRINTF(("addr=%d, getting full desc failed\n",
		    udev->address));
		goto done;
	}
	/* figure out what's wrong with this device */
	udev->quirks = usbd_find_quirk(&udev->ddesc);
#if 0
	if (udev->quirks->uq_flags & UQ_NO_STRINGS) {
		udev->no_strings = 1;
	}
#endif

	/*
	 * Workaround for buggy USB devices.
	 *
	 * It appears that some string-less USB chips will crash and
	 * disappear if any attempts are made to read any string
	 * descriptors.
	 *
	 * Try to detect such chips by checking the strings in the USB
	 * device descriptor. If no strings are present there we
	 * simply disable all USB strings.
	 */

	if (udev->ddesc.iManufacturer ||
	    udev->ddesc.iProduct ||
	    udev->ddesc.iSerialNumber) {
		/* read out the language ID string */
		err = usbreq_get_string_desc(udev, &usb_global_lock,
		    udev->scratch[0].data, 4, sizeof(udev->scratch),
		    USB_LANGUAGE_TABLE);
	} else {
		err = USBD_ERR_INVAL;
	}

	if (err || (udev->scratch[0].data[0] < 4)) {
		udev->flags.no_strings = 1;
	} else {
		/* pick the first language as the default */
		udev->langid = UGETW(udev->scratch[0].data + 2);
	}

	/* assume 100mA bus powered for now. Changed when configured. */
	udev->power = USB_MIN_POWER;

	/* get serial number string */
	err = usbreq_get_string_any
	    (udev, &usb_global_lock, udev->scratch[0].data,
	    sizeof(udev->scratch), udev->ddesc.iSerialNumber);

	strlcpy(udev->serial, udev->scratch[0].data, sizeof(udev->serial));

	/* get manufacturer string */
	err = usbreq_get_string_any
	    (udev, &usb_global_lock, udev->scratch[0].data,
	    sizeof(udev->scratch), udev->ddesc.iManufacturer);

	strlcpy(udev->manufacturer, udev->scratch[0].data, sizeof(udev->manufacturer));

	/* get product string */
	err = usbreq_get_string_any
	    (udev, &usb_global_lock, udev->scratch[0].data,
	    sizeof(udev->scratch), udev->ddesc.iProduct);

	strlcpy(udev->product, udev->scratch[0].data, sizeof(udev->product));

	/* finish up all the strings */
	usbd_finish_vp_info(udev);

	PRINTF(("new dev (addr %d), udev=%p, parent_hub=%p\n",
	    udev->address, udev, udev->parent_hub));

	mtx_lock(&(bus->mtx));
	udev->flags.detaching = 0;
	mtx_unlock(&(bus->mtx));

	err = 0;			/* force success */

done:
	if (err) {
		/* free device  */
		usbd_free_device(udev);
		udev = NULL;
	}
	return (udev);
}

/*------------------------------------------------------------------------*
 *	usbd_free_device
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_free_device(struct usbd_device *udev)
{
	struct usbd_bus *bus;
	int error;

	/* mtx_assert() */

	if (udev == NULL) {
		/* already freed */
		return;
	}
	PRINTFN(3, ("udev=%p port=%d\n", udev, udev->port_no));

	/*
	 * Wait until all references to our device is gone:
	 */
	bus = udev->bus;
	mtx_lock(&(bus->mtx));
	udev->flags.detaching = 1;
	udev->refcount--;
	while (udev->refcount > 0) {

		error = mtx_sleep(&(udev->refcount), &(bus->mtx), 0,
		    "USB detach wait", 0);
	}
	mtx_unlock(&(bus->mtx));

	usbd_free_iface_data(udev);

	usbd_transfer_unsetup(udev->default_xfer, 1);

	usbd_temp_unsetup(udev);

	sx_destroy(udev->default_sx);

	mtx_destroy(udev->default_mtx);

	/* unregister our device */
	usbd_bus_port_set_device(bus, udev->parent_hub ?
	    udev->parent_hub->hub->ports + udev->port_index : NULL,
	    NULL, USB_ROOT_HUB_ADDR);

	/* free device */
	free(udev, M_USB);

	return;
}

struct usbd_device *
usbd_ref_device(struct usbd_bus *bus, uint8_t index)
{
	struct usbd_device *udev;

	if (index >= USB_MAX_DEVICES) {
		/* invalid index */
		return (NULL);
	}
	mtx_lock(&(bus->mtx));
	udev = bus->devices[index];
	if (udev == NULL) {
		/* do nothing */
	} else if (udev->flags.detaching) {
		udev = NULL;
	} else if (udev->refcount == USB_DEV_REFCOUNT_MAX) {
		udev = NULL;
	} else {
		udev->refcount++;
	}
	mtx_unlock(&(bus->mtx));
	return (udev);
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
			if (udev->flags.detaching) {
				wakeup(&(udev->refcount));
			}
		}
	}
	mtx_unlock(&(bus->mtx));
	return;
}

struct usbd_interface *
usbd_get_iface(struct usbd_device *udev, uint8_t iface_index)
{
	struct usbd_interface *iface = udev->ifaces + iface_index;

	if ((iface < udev->ifaces) ||
	    (iface >= udev->ifaces_end) ||
	    (udev->cdesc == NULL) ||
	    (iface_index >= udev->cdesc->bNumInterface)) {
		return (NULL);
	}
	return (iface);
}

void
usbd_set_device_desc(device_t dev)
{
	struct usb_attach_arg *uaa;
	struct usbd_device *udev;
	struct usbd_interface *iface;
	usbd_status_t err;

	if (dev == NULL) {
		/* should not happen */
		return;
	}
	uaa = device_get_ivars(dev);
	if (uaa == NULL) {
		/* can happend if called at the wrong time */
		return;
	}
	udev = uaa->device;
	iface = uaa->iface;

	if ((iface == NULL) ||
	    (iface->idesc == NULL) ||
	    (iface->idesc->iInterface == 0)) {
		err = USBD_ERR_INVAL;
	} else {
		err = 0;
	}

	if (!err) {
		/* try to get the interface string ! */
		err = usbreq_get_string_any
		    (udev, &usb_global_lock, udev->scratch[0].data,
		    sizeof(udev->scratch), iface->idesc->iInterface);
	}
	if (err) {
		/* use default description */
		usbd_devinfo(udev, udev->scratch[0].data,
		    sizeof(udev->scratch));
	}
	device_set_desc_copy(dev, udev->scratch[0].data);
	device_printf(dev, "<%s> on %s\n", udev->scratch[0].data,
	    device_get_nameunit(udev->bus->bdev));
	return;
}

/*------------------------------------------------------------------------*
 *      allocate mbufs to an usbd interface queue
 *
 * returns a pointer that eventually should be passed to "free()"
 *------------------------------------------------------------------------*/
void   *
usbd_alloc_mbufs(struct malloc_type *type, struct usbd_ifqueue *ifq,
    uint32_t block_size, uint16_t block_number)
{
	struct usbd_mbuf *m_ptr;
	uint8_t *data_ptr;
	void *free_ptr = NULL;
	uint32_t alloc_size;

	/* align data */
	block_size += ((-block_size) & (USB_HOST_ALIGN - 1));

	if (block_number && block_size) {

		alloc_size = (block_size + sizeof(struct usbd_mbuf)) * block_number;

		free_ptr = malloc(alloc_size, type, M_WAITOK | M_ZERO);

		if (free_ptr == NULL) {
			goto done;
		}
		m_ptr = free_ptr;
		data_ptr = (void *)(m_ptr + block_number);

		while (block_number--) {

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
	return (free_ptr);
}

/*------------------------------------------------------------------------*
 *  usbd_get_page - lookup DMA-able memory for the given offset
 *
 * NOTE: Only call this function when the "page_cache" structure has
 * been properly initialized !
 *------------------------------------------------------------------------*/
void
usbd_get_page(struct usbd_page_cache *pc, uint32_t offset,
    struct usbd_page_search *res)
{
	struct usbd_page *page;

	if (pc->page_start) {

		/* Case 1 - something has been loaded into DMA */

		if (pc->buffer) {

			/* Case 1a - Kernel Virtual Address */

			res->buffer = USBD_ADD_BYTES(pc->buffer, offset);
		}
		offset += pc->page_offset_buf;

		/* compute destination page */

		page = pc->page_start;

		page += (offset / USB_PAGE_SIZE);

		offset %= USB_PAGE_SIZE;

		res->length = USB_PAGE_SIZE - offset;
		res->physaddr = page->physaddr + offset;
		if (!pc->buffer) {

			/* Case 1b - Non Kernel Virtual Address */

			res->buffer = USBD_ADD_BYTES(page->buffer, offset);
		}
	} else {

		/* Case 2 - Plain PIO */

		res->buffer = USBD_ADD_BYTES(pc->buffer, offset);
		res->length = 0 - 1;
		res->physaddr = 0;
	}
	return;
}

/*------------------------------------------------------------------------*
 *  usbd_copy_in - copy directly to DMA-able memory
 *------------------------------------------------------------------------*/
void
usbd_copy_in(struct usbd_page_cache *cache, uint32_t offset,
    const void *ptr, uint32_t len)
{
	struct usbd_page_search buf_res;

	while (len != 0) {

		usbd_get_page(cache, offset, &buf_res);

		if (buf_res.length > len) {
			buf_res.length = len;
		}
		bcopy(ptr, buf_res.buffer, buf_res.length);

		offset += buf_res.length;
		len -= buf_res.length;
		ptr = USBD_ADD_BYTES(ptr, buf_res.length);
	}
	return;
}

/*------------------------------------------------------------------------*
 *  usbd_m_copy_in - copy a mbuf chain directly into DMA-able memory
 *------------------------------------------------------------------------*/
struct usbd_m_copy_in_arg {
	struct usbd_page_cache *cache;
	uint32_t dst_offset;
};

static int32_t
#ifdef __FreeBSD__
usbd_m_copy_in_cb(void *arg, void *src, uint32_t count)
#else
usbd_m_copy_in_cb(void *arg, caddr_t src, uint32_t count)
#endif
{
	register struct usbd_m_copy_in_arg *ua = arg;

	usbd_copy_in(ua->cache, ua->dst_offset, src, count);
	ua->dst_offset += count;
	return (0);
}

void
usbd_m_copy_in(struct usbd_page_cache *cache, uint32_t dst_offset,
    struct mbuf *m, uint32_t src_offset, uint32_t src_len)
{
	struct usbd_m_copy_in_arg arg = {cache, dst_offset};
	register int error;

	error = m_apply(m, src_offset, src_len, &usbd_m_copy_in_cb, &arg);
	return;
}

/*------------------------------------------------------------------------*
 *  usbd_uiomove - factored out code
 *------------------------------------------------------------------------*/
int
usbd_uiomove(struct usbd_page_cache *pc, struct uio *uio,
    uint32_t pc_offset, uint32_t len)
{
	struct usbd_page_search res;
	int error = 0;

	while (len != 0) {

		usbd_get_page(pc, pc_offset, &res);

		if (res.length > len) {
			res.length = len;
		}
		/*
		 * "uiomove()" can sleep so one needs to make a wrapper,
		 * exiting the mutex and checking things
		 */
		error = uiomove(res.buffer, res.length, uio);

		if (error) {
			break;
		}
		pc_offset += res.length;
		len -= res.length;
	}
	return (error);
}

/*------------------------------------------------------------------------*
 *  usbd_copy_out - copy directly from DMA-able memory
 *------------------------------------------------------------------------*/
void
usbd_copy_out(struct usbd_page_cache *cache, uint32_t offset,
    void *ptr, uint32_t len)
{
	struct usbd_page_search res;

	while (len != 0) {

		usbd_get_page(cache, offset, &res);

		if (res.length > len) {
			res.length = len;
		}
		bcopy(res.buffer, ptr, res.length);

		offset += res.length;
		len -= res.length;
		ptr = USBD_ADD_BYTES(ptr, res.length);
	}
	return;
}

/*------------------------------------------------------------------------*
 *  usbd_bzero - zero DMA-able memory
 *------------------------------------------------------------------------*/
void
usbd_bzero(struct usbd_page_cache *cache, uint32_t offset, uint32_t len)
{
	struct usbd_page_search res;

	while (len != 0) {

		usbd_get_page(cache, offset, &res);

		if (res.length > len) {
			res.length = len;
		}
		bzero(res.buffer, res.length);

		offset += res.length;
		len -= res.length;
	}
	return;
}

#ifdef __FreeBSD__

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_create - allocate a DMA tag
 *
 * NOTE: If the "align" parameter has a value of 1 the DMA-tag will
 * allow multi-segment mappings. Else all mappings are single-segment.
 *------------------------------------------------------------------------*/
void
usbd_dma_tag_create(bus_dma_tag_t tag_parent, struct usbd_dma_tag *udt,
    uint32_t size, uint32_t align)
{
	bus_dma_tag_t tag;

	if (bus_dma_tag_create
	    ( /* parent    */ tag_parent,
	     /* alignment */ align,
	     /* boundary  */ 0,
	     /* lowaddr   */ BUS_SPACE_MAXADDR_32BIT,
	     /* highaddr  */ BUS_SPACE_MAXADDR,
	     /* filter    */ NULL,
	     /* filterarg */ NULL,
	     /* maxsize   */ size,
	     /* nsegments */ (align == 1) ?
	    (2 + (size / USB_PAGE_SIZE)) : 1,
	     /* maxsegsz  */ USB_PAGE_SIZE,
	     /* flags     */ 0,
	     /* lock      */ NULL,
	     /* */ NULL,
	    &tag)) {
		tag = NULL;
	}
	udt->tag = tag;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_free - free a DMA tag
 *------------------------------------------------------------------------*/
void
usbd_dma_tag_destroy(struct usbd_dma_tag *udt)
{
	bus_dma_tag_destroy(udt->tag);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_alloc_mem_cb
 *------------------------------------------------------------------------*/
static void
usbd_pc_alloc_mem_cb(void *arg, bus_dma_segment_t *segs,
    int nseg, int error)
{
	struct usbd_xfer *xfer;
	struct usbd_page_cache *pc;
	struct usbd_page *pg;
	uint32_t rem;
	uint8_t owned;

	pc = arg;
	xfer = pc->xfer;

	/*
	 * XXX There is sometimes recursive locking here.
	 * XXX We should try to find a better solution.
	 * XXX Until further the "owned" variable does
	 * XXX the trick.
	 */

	if (error) {
		if (xfer) {
			owned = mtx_owned(xfer->priv_mtx);
			if (!owned)
				mtx_lock(xfer->priv_mtx);
			xfer->usb_root->dma_error = 1;
			usbd_bdma_done_event(xfer->usb_root);
			if (!owned)
				mtx_unlock(xfer->priv_mtx);
		}
		return;
	}
	pg = pc->page_start;
	pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	rem = segs->ds_addr & (USB_PAGE_SIZE - 1);
	pc->page_offset_buf = rem;
	pc->page_offset_end += rem;
	nseg--;

	while (nseg > 0) {
		nseg--;
		segs++;
		pg++;
		pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	}

	if (xfer) {
		owned = mtx_owned(xfer->priv_mtx);
		if (!owned)
			mtx_lock(xfer->priv_mtx);
		usbd_bdma_done_event(xfer->usb_root);
		if (!owned)
			mtx_unlock(xfer->priv_mtx);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_alloc_mem - allocate DMA'able memory
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usbd_pc_alloc_mem(bus_dma_tag_t parent_tag, struct usbd_dma_tag *utag,
    struct usbd_page_cache *pc, struct usbd_page *pg, uint32_t size,
    uint32_t align, uint8_t utag_max)
{
	bus_dma_tag_t tag;
	bus_dmamap_t map;
	void *ptr;

	if (align != 1) {
		/*
	         * The alignment must be greater or equal to the "size" else the
	         * object can be split between two memory pages and we get a
	         * problem!
	         */
		while (align < size) {
			align *= 2;
			if (align == 0) {
				goto error;
			}
		}
	}
	/* get the correct DMA tag */
	utag = usbd_dma_tag_setup(parent_tag, utag, size, align, utag_max);
	if (utag == NULL) {
		goto error;
	}
	/* get the DMA tag */
	tag = utag->tag;

	/* allocate memory */
	if (bus_dmamem_alloc
	    (tag, &ptr, (BUS_DMA_WAITOK | BUS_DMA_COHERENT), &map)) {
		goto error;
	}
	/* setup page cache */
	pc->buffer = ptr;
	pc->page_start = pg;
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;
	pc->map = map;
	pc->tag = tag;

	/* load memory into DMA */
	if (bus_dmamap_load
	    (tag, map, ptr, size, &usbd_pc_alloc_mem_cb,
	    pc, (BUS_DMA_WAITOK | BUS_DMA_COHERENT))) {
		bus_dmamem_free(tag, ptr, map);
		goto error;
	}
	bzero(ptr, size);

	usbd_pc_cpu_flush(pc);

	return (0);

error:
	/* reset most of the page cache */
	pc->buffer = NULL;
	pc->page_start = NULL;
	pc->page_offset_buf = 0;
	pc->page_offset_end = 0;
	pc->map = NULL;
	pc->tag = NULL;
	return (1);
}

/*------------------------------------------------------------------------*
 *	usbd_pc_free_mem - free DMA memory
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_pc_free_mem(struct usbd_page_cache *pc)
{
	if (pc && pc->buffer) {

		bus_dmamap_unload(pc->tag, pc->map);

		bus_dmamem_free(pc->tag, pc->buffer, pc->map);

		pc->buffer = NULL;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_load_mem - load virtual memory into DMA
 *------------------------------------------------------------------------*/
void
usbd_pc_load_mem(struct usbd_page_cache *pc, uint32_t size)
{
	/* sanity check */
	if (pc->xfer == NULL) {
		panic("This page cache is not loadable!\n");
		return;
	}
	/* setup page cache */
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;

	if (size > 0) {

		pc->xfer->usb_root->dma_refcount++;

		/* try to load memory into DMA */
		if (bus_dmamap_load(
		    pc->tag, pc->map, pc->buffer, size,
		    &usbd_pc_alloc_mem_cb, pc, 0)) {
		}
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_cpu_invalidate - invalidate CPU cache
 *------------------------------------------------------------------------*/
void
usbd_pc_cpu_invalidate(struct usbd_page_cache *pc)
{
	bus_dmamap_sync(pc->tag, pc->map,
	    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_cpu_flush - flush CPU cache
 *------------------------------------------------------------------------*/
void
usbd_pc_cpu_flush(struct usbd_page_cache *pc)
{
	bus_dmamap_sync(pc->tag, pc->map,
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_dmamap_create
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usbd_pc_dmamap_create(struct usbd_page_cache *pc, uint32_t size)
{
	struct usbd_memory_info *info;
	struct usbd_dma_tag *utag;
	bus_dma_tag_t tag;

	/* sanity check */
	if (pc->xfer == NULL) {
		goto error;
	}
	info = pc->xfer->usb_root;
	tag = pc->xfer->udev->bus->dma_tag_parent;

	utag = usbd_dma_tag_setup(tag, info->dma_tag_p,
	    size, 1, info->dma_tag_max);
	if (utag == NULL) {
		goto error;
	}
	/* get the DMA tag */
	tag = utag->tag;

	/* create DMA map */
	if (bus_dmamap_create(tag, 0, &(pc->map))) {
		goto error;
	}
	pc->tag = tag;
	return 0;			/* success */

error:
	pc->map = NULL;
	pc->tag = NULL;
	return 1;			/* failure */
}

/*------------------------------------------------------------------------*
 *	usbd_pc_dmamap_destroy
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_pc_dmamap_destroy(struct usbd_page_cache *pc)
{
	if (pc && pc->tag) {
		bus_dmamap_destroy(pc->tag, pc->map);
		pc->tag = NULL;
		pc->map = NULL;
	}
	return;
}

#endif

#ifdef __NetBSD__

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_create - allocate a DMA tag
 *
 * NOTE: If the "align" parameter has a value of 1 the DMA-tag will
 * allow multi-segment mappings. Else all mappings are single-segment.
 *------------------------------------------------------------------------*/
void
usbd_dma_tag_create(bus_dma_tag_t tag_parent, struct usbd_dma_tag *udt,
    uint32_t size, uint32_t align)
{
	uint32_t nseg;

	if (align == 1) {
		nseg = (2 + (size / USB_PAGE_SIZE));
	} else {
		nseg = 1;
	}

	udt->p_seg = malloc(nseg * sizeof(*(udt->p_seg)),
	    M_USB, M_WAITOK | M_ZERO);

	if (udt->p_seg == NULL) {
		return;
	}
	udt->tag = tag_parent;
	udt->n_seg = nseg;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_free - free a DMA tag
 *------------------------------------------------------------------------*/
void
usbd_dma_tag_destroy(struct usbd_dma_tag *udt)
{
	free(udt->p_seg, M_USB);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_alloc_mem_cb
 *------------------------------------------------------------------------*/
static void
usbd_pc_alloc_mem_cb(struct usbd_page_cache *pc, bus_dma_segment_t *segs,
    int nseg, int error)
{
	struct usbd_xfer *xfer;
	struct usbd_page *pg;
	uint32_t rem;
	uint8_t owned;

	xfer = pc->xfer;

	/*
	 * XXX There is sometimes recursive locking here.
	 * XXX We should try to find a better solution.
	 * XXX Until further the "owned" variable does
	 * XXX the trick.
	 */

	if (error) {
		if (xfer) {
			owned = mtx_owned(xfer->priv_mtx);
			if (!owned)
				mtx_lock(xfer->priv_mtx);
			xfer->usb_root->dma_error = 1;
			usbd_bdma_done_event(xfer->usb_root);
			if (!owned)
				mtx_unlock(xfer->priv_mtx);
		}
		return;
	}
	pg = pc->page_start;
	pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	rem = segs->ds_addr & (USB_PAGE_SIZE - 1);
	pc->page_offset_buf = rem;
	pc->page_offset_end += rem;
	nseg--;

	while (nseg > 0) {
		nseg--;
		segs++;
		pg++;
		pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	}

	if (xfer) {
		owned = mtx_owned(xfer->priv_mtx);
		if (!owned)
			mtx_lock(xfer->priv_mtx);
		usbd_bdma_done_event(xfer->usb_root);
		if (!owned)
			mtx_unlock(xfer->priv_mtx);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_alloc_mem - allocate DMA'able memory
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usbd_pc_alloc_mem(bus_dma_tag_t parent_tag, struct usbd_dma_tag *utag,
    struct usbd_page_cache *pc, struct usbd_page *pg, uint32_t size,
    uint32_t align, uint8_t utag_max)
{
	caddr_t ptr = NULL;
	bus_dma_tag_t tag;
	bus_dmamap_t map;
	int seg_count;

	if (align != 1) {
		/*
	         * The alignment must be greater or equal to the
	         * "size" else the object can be split between two
	         * memory pages and we get a problem!
	         */
		while (align < size) {
			align *= 2;
			if (align == 0) {
				goto done_5;
			}
		}
	}
	/* get the correct DMA tag */
	utag = usbd_dma_tag_setup(parent_tag, utag, size, align, utag_max);
	if (utag == NULL) {
		goto done_5;
	}
	/* get the DMA tag */
	tag = utag->tag;

	if (bus_dmamem_alloc(tag, size, align, 0, utag->p_seg,
	    utag->n_seg, &seg_count, BUS_DMA_WAITOK)) {
		goto done_4;
	}
	if (bus_dmamem_map(tag, utag->p_seg, seg_count, size,
	    &ptr, BUS_DMA_WAITOK | BUS_DMA_COHERENT)) {
		goto done_3;
	}
	if (bus_dmamap_create(tag, size, utag->n_seg, USB_PAGE_SIZE,
	    0, BUS_DMA_WAITOK, &map)) {
		goto done_2;
	}
	if (bus_dmamap_load(tag, map, ptr, size, NULL,
	    BUS_DMA_WAITOK)) {
		goto done_1;
	}
	pc->p_seg = malloc(seg_count * sizeof(*(pc->p_seg)),
	    M_USB, M_WAITOK | M_ZERO);
	if (pc->p_seg == NULL) {
		goto done_0;
	}
	/* store number if actual segments used */
	pc->n_seg = seg_count;

	/* make a copy of the segments */
	bcopy(utag->p_seg, pc->p_seg,
	    seg_count * sizeof(*(pc->p_seg)));

	/* setup page cache */
	pc->buffer = ptr;
	pc->page_start = pg;
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;
	pc->map = map;
	pc->tag = tag;

	usbd_pc_alloc_mem_cb(pc, utag->p_seg, seg_count, 0);

	bzero(ptr, size);

	usbd_pc_cpu_flush(pc);

	return (0);

done_0:
	bus_dmamap_unload(tag, map);
done_1:
	bus_dmamap_destroy(tag, map);
done_2:
	bus_dmamem_unmap(tag, ptr, size);
done_3:
	bus_dmamem_free(tag, utag->p_seg, seg_count);
done_4:
	/* utag is destroyed later */
done_5:
	/* reset most of the page cache */
	pc->buffer = NULL;
	pc->page_start = NULL;
	pc->page_offset_buf = 0;
	pc->page_offset_end = 0;
	pc->map = NULL;
	pc->tag = NULL;
	pc->n_seg = 0;
	pc->p_seg = NULL;
	return (1);
}

/*------------------------------------------------------------------------*
 *	usbd_pc_free_mem - free DMA memory
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_pc_free_mem(struct usbd_page_cache *pc)
{
	if (pc && pc->buffer) {
		bus_dmamap_unload(pc->tag, pc->map);
		bus_dmamap_destroy(pc->tag, pc->map);
		bus_dmamem_unmap(pc->tag, pc->buffer,
		    pc->page_offset_end - pc->page_offset_buf);
		bus_dmamem_free(pc->tag, pc->p_seg, pc->n_seg);
		free(pc->p_seg, M_USB);
		pc->buffer = NULL;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_load_mem - load virtual memory into DMA
 *------------------------------------------------------------------------*/
void
usbd_pc_load_mem(struct usbd_page_cache *pc, uint32_t size)
{
	int error;

	/* sanity check */
	if (pc->xfer == NULL) {
		panic("This page cache is not loadable!\n");
		return;
	}
	/* setup page cache */
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;

	if (size > 0) {

		pc->xfer->usb_root->dma_refcount++;

		/* try to load memory into DMA */
		if (bus_dmamap_load(pc->tag, pc->map, pc->buffer,
		    size, NULL, BUS_DMA_NOWAIT)) {
			error = ENOMEM;
		} else {
			error = 0;
		}

		usbd_pc_alloc_mem_cb(pc, pc->map->dm_segs,
		    pc->map->dm_nsegs, error);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_cpu_invalidate - invalidate CPU cache
 *------------------------------------------------------------------------*/
void
usbd_pc_cpu_invalidate(struct usbd_page_cache *pc)
{
	uint32_t len;

	len = pc->page_offset_end - pc->page_offset_buf;

	bus_dmamap_sync(pc->tag, pc->map, 0, len,
	    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_cpu_flush - flush CPU cache
 *------------------------------------------------------------------------*/
void
usbd_pc_cpu_flush(struct usbd_page_cache *pc)
{
	uint32_t len;

	len = pc->page_offset_end - pc->page_offset_buf;

	bus_dmamap_sync(pc->tag, pc->map, 0, len,
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_dmamap_create
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usbd_pc_dmamap_create(struct usbd_page_cache *pc, uint32_t size)
{
	struct usbd_memory_info *info;
	struct usbd_dma_tag *utag;
	bus_dma_tag_t tag;

	/* sanity check */
	if (pc->xfer == NULL) {
		goto error;
	}
	info = pc->xfer->usb_root;
	tag = pc->xfer->udev->bus->dma_tag_parent;

	utag = usbd_dma_tag_setup(tag, info->dma_tag_p,
	    size, 1, info->dma_tag_max);
	if (utag == NULL) {
		goto error;
	}
	if (bus_dmamap_create(utag->tag, size, utag->n_seg,
	    USB_PAGE_SIZE, 0, BUS_DMA_WAITOK, &(pc->map))) {
		goto error;
	}
	pc->tag = utag->tag;
	pc->p_seg = utag->p_seg;
	pc->n_seg = utag->n_seg;
	return 0;			/* success */

error:
	pc->map = NULL;
	pc->tag = NULL;
	pc->p_seg = NULL;
	pc->n_seg = 0;
	return 1;			/* failure */
}

/*------------------------------------------------------------------------*
 *	usbd_pc_dmamap_destroy
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_pc_dmamap_destroy(struct usbd_page_cache *pc)
{
	if (pc && pc->tag) {
		bus_dmamap_destroy(pc->tag, pc->map);
		pc->tag = NULL;
		pc->map = NULL;
	}
	return;
}

#endif

/*------------------------------------------------------------------------*
 *  usbd_make_str_desc - convert an ASCII string into a UNICODE string
 *------------------------------------------------------------------------*/
uint8_t
usbd_make_str_desc(void *ptr, uint16_t max_len, const char *s)
{
	usb_string_descriptor_t *p = ptr;
	uint8_t totlen;
	int32_t j;

	if (max_len < 2) {
		/* invalid length */
		return (0);
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
	return (totlen);
}

/*------------------------------------------------------------------------*
 * mtx_drop_recurse - drop mutex recurse level
 *------------------------------------------------------------------------*/
uint32_t
mtx_drop_recurse(struct mtx *mtx)
{
	uint32_t recurse_level = mtx->mtx_recurse;
	uint32_t recurse_curr = recurse_level;

	mtx_assert(mtx, MA_OWNED);

	while (recurse_curr--) {
		mtx_unlock(mtx);
	}

	return (recurse_level);
}

/*------------------------------------------------------------------------*
 * mtx_pickup_recurse - pickup mutex recurse level
 *------------------------------------------------------------------------*/
void
mtx_pickup_recurse(struct mtx *mtx, uint32_t recurse_level)
{
	mtx_assert(mtx, MA_OWNED);

	while (recurse_level--) {
		mtx_lock(mtx);
	}
	return;
}


/*------------------------------------------------------------------------*
 * usbd_config_thread
 *------------------------------------------------------------------------*/
static void
usbd_config_td_thread(void *arg)
{
	struct usbd_config_td *ctd = arg;
	struct usbd_config_td_item *item;
	struct usbd_mbuf *m;
	struct thread *td;
	register int error;

	/* adjust priority */
	td = curthread;
	thread_lock(td);
	sched_prio(td, PI_DISK);
	thread_unlock(td);

	mtx_lock(ctd->p_mtx);

	while (1) {

		if (ctd->flag_config_td_gone) {
			break;
		}
		/*
		 * NOTE to reimplementors: dequeueing a command from the
		 * "used" queue and executing it must be atomic, with regard
		 * to the "p_mtx" mutex. That means any attempt to queue a
		 * command by another thread must be blocked until either:
		 *
		 * 1) the command sleeps
		 *
		 * 2) the command returns
		 *
		 * Here is a practical example that shows how this helps
		 * solving a problem:
		 *
		 * Assume that you want to set the baud rate on a USB serial
		 * device. During the programming of the device you don't
		 * want to receive nor transmit any data, because it will be
		 * garbage most likely anyway. The programming of our USB
		 * device takes 20 milliseconds and it needs to call
		 * functions that sleep.
		 *
		 * Non-working solution: Before we queue the programming
		 * command, we stop transmission and reception of data. Then
		 * we queue a programming command. At the end of the
		 * programming command we enable transmission and reception
		 * of data.
		 *
		 * Problem: If a second programming command is queued while the
		 * first one is sleeping, we end up enabling transmission
		 * and reception of data too early.
		 *
		 * Working solution: Before we queue the programming command,
		 * we stop transmission and reception of data. Then we queue
		 * a programming command. Then we queue a second command
		 * that only enables transmission and reception of data.
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
			    (ctd->p_softc, (void *)(item + 1), item->command_ref);

			USBD_IF_ENQUEUE(&(ctd->cmd_free), m);

			continue;
		}
		if (ctd->p_end_of_commands) {
			(ctd->p_end_of_commands) (ctd->p_softc);
		}
		ctd->flag_config_td_sleep = 1;

		error = mtx_sleep(&(ctd->wakeup_config_td), ctd->p_mtx,
		    0, "cfg td sleep", 0);

		ctd->flag_config_td_sleep = 0;
	}

	ctd->config_thread = NULL;

	wakeup(&(ctd->wakeup_config_td_gone));

	mtx_unlock(ctd->p_mtx);

	usb_thread_exit(0);

	return;
}

/*------------------------------------------------------------------------*
 * usbd_config_td_setup
 *
 * NOTE: the structure pointed to by "ctd" must be zeroed before calling
 * this function!
 *
 * Return values:
 *    0: success
 * else: failure
 *------------------------------------------------------------------------*/
uint8_t
usbd_config_td_setup(struct usbd_config_td *ctd, void *priv_sc,
    struct mtx *priv_mtx,
    usbd_config_td_end_of_commands_t *p_func_eoc,
    uint16_t item_size, uint16_t item_count)
{
	ctd->p_mtx = priv_mtx;
	ctd->p_softc = priv_sc;
	ctd->p_end_of_commands = p_func_eoc;

	if (item_count >= 256) {
		PRINTFN(0, ("too many items!\n"));
		goto error;
	}
	ctd->p_cmd_queue =
	    usbd_alloc_mbufs(M_DEVBUF, &(ctd->cmd_free),
	    (sizeof(struct usbd_config_td_item) + item_size),
	    item_count);

	if (ctd->p_cmd_queue == NULL) {
		PRINTFN(0, ("unable to allocate memory "
		    "for command queue!\n"));
		goto error;
	}
	if (usb_thread_create
	    (&usbd_config_td_thread, ctd, &(ctd->config_thread),
	    "USB config thread")) {
		PRINTFN(0, ("unable to create config thread!\n"));
		ctd->config_thread = NULL;
		goto error;
	}
	return (0);

error:
	usbd_config_td_unsetup(ctd);
	return (1);
}

/*------------------------------------------------------------------------*
 * usbd_config_td_dummy_cmd
 *------------------------------------------------------------------------*/
static void
usbd_config_td_dummy_cmd(struct usbd_config_td_softc *sc,
    struct usbd_config_td_cc *cc,
    uint16_t reference)
{
	return;
}

/*------------------------------------------------------------------------*
 * usbd_config_td_stop
 *
 * NOTE: If the structure pointed to by "ctd" is all zero,
 * this function does nothing.
 *------------------------------------------------------------------------*/
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

			/*
			 * set the gone flag after queueing the last
			 * command:
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

/*------------------------------------------------------------------------*
 * usbd_config_td_unsetup
 *
 * NOTE: If the structure pointed to by "ctd" is all zero,
 * this function does nothing.
 *------------------------------------------------------------------------*/
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

/*------------------------------------------------------------------------*
 * usbd_config_td_queue_command
 *------------------------------------------------------------------------*/
void
usbd_config_td_queue_command(struct usbd_config_td *ctd,
    usbd_config_td_command_t *command_pre_func,
    usbd_config_td_command_t *command_post_func,
    uint16_t command_qcount,
    uint16_t command_ref)
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

	/*
	 * The job of the post-command function is to finish the command in
	 * a separate context to allow calls to sleeping functions
	 * basically. Queue the post command before calling the pre command.
	 * That way commands queued by the pre command will be queued after
	 * the current command.
	 */
	item->command_func = command_post_func;
	item->command_ref = command_ref;

	USBD_IF_ENQUEUE(&(ctd->cmd_used), m);

	/*
	 * The job of the pre-command function is to copy the needed
	 * configuration to the provided structure and to execute other
	 * commands that must happen immediately
	 */
	if (command_pre_func) {
		(command_pre_func) (ctd->p_softc, (void *)(item + 1), command_ref);
	}
	/*
	 * Currently we use a separate thread to execute the command, but it
	 * is not impossible that we might use a so called taskqueue in the
	 * future:
	 */
	if (ctd->flag_config_td_sleep) {
		ctd->flag_config_td_sleep = 0;
		wakeup(&(ctd->wakeup_config_td));
	}
	return;
}

/*------------------------------------------------------------------------*
 * usbd_config_td_is_gone
 *
 * Return values:
 *    0: config thread is running
 * else: config thread is gone
 *------------------------------------------------------------------------*/
uint8_t
usbd_config_td_is_gone(struct usbd_config_td *ctd)
{
	mtx_assert(ctd->p_mtx, MA_OWNED);

	return (ctd->flag_config_td_gone ? 1 : 0);
}

/*------------------------------------------------------------------------*
 * usbd_config_td_sleep
 *
 * NOTE: this function can only be called from the config thread
 *
 * Return values:
 *    0: normal delay
 * else: config thread is gone
 *------------------------------------------------------------------------*/
uint8_t
usbd_config_td_sleep(struct usbd_config_td *ctd, uint32_t timeout)
{
	register int error;
	uint8_t is_gone = usbd_config_td_is_gone(ctd);
	uint32_t level;

	if (is_gone) {
		goto done;
	}
	if (timeout == 0) {
		/*
		 * zero means no timeout, so avoid that by setting timeout
		 * to one:
		 */
		timeout = 1;
	}
	level = mtx_drop_recurse(ctd->p_mtx);

	error = mtx_sleep(ctd, ctd->p_mtx, 0,
	    "config td sleep", timeout);

	mtx_pickup_recurse(ctd->p_mtx, level);

done:
	return (is_gone);
}

/*------------------------------------------------------------------------*
 * usbd_ether_get_mbuf - get a new ethernet mbuf
 *------------------------------------------------------------------------*/
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

/*------------------------------------------------------------------------*
 * device_delete_all_children - delete all children of a device
 *------------------------------------------------------------------------*/
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
	return (error);
}

/*------------------------------------------------------------------------*
 * usbd_isoc_time_expand - expand time counter from 7-bit to 16-bit
 *------------------------------------------------------------------------*/
uint16_t
usbd_isoc_time_expand(struct usbd_bus *bus, uint16_t isoc_time_curr)
{
	uint16_t rem;

	mtx_assert(&(bus->mtx), MA_OWNED);

	rem = bus->isoc_time_last & (USB_ISOC_TIME_MAX - 1);

	isoc_time_curr &= (USB_ISOC_TIME_MAX - 1);

	if (isoc_time_curr < rem) {
		/* the time counter wrapped around */
		bus->isoc_time_last += USB_ISOC_TIME_MAX;
	}
	/* update the remainder */

	bus->isoc_time_last &= ~(USB_ISOC_TIME_MAX - 1);
	bus->isoc_time_last |= isoc_time_curr;

	return (bus->isoc_time_last);
}

/*------------------------------------------------------------------------*
 *	usbd_bus_tag_setup - factored out code
 *------------------------------------------------------------------------*/
struct usbd_dma_tag *
usbd_dma_tag_setup(bus_dma_tag_t tag_parent, struct usbd_dma_tag *udt,
    uint32_t size, uint32_t align, uint8_t nudt)
{
	__KASSERT(align > 0, ("Invalid parameter align = 0!\n"));
	__KASSERT(size > 0, ("Invalid parameter size = 0!\n"));

	while (nudt--) {

		if (udt->align == 0) {
			usbd_dma_tag_create(tag_parent, udt, size, align);
			if (udt->tag == NULL) {
				return (NULL);
			}
			udt->align = align;
			udt->size = size;
			return (udt);
		}
		if ((udt->align == align) && (udt->size == size)) {
			return (udt);
		}
		udt++;
	}
	return (NULL);
}

/*------------------------------------------------------------------------*
 *	usbd_bus_tag_unsetup - factored out code
 *------------------------------------------------------------------------*/
void
usbd_dma_tag_unsetup(struct usbd_dma_tag *udt, uint8_t nudt)
{
	while (nudt--) {

		if (udt->align) {
			usbd_dma_tag_destroy(udt);
			udt->align = 0;
		}
		udt++;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bus_mem_flush_all_cb
 *------------------------------------------------------------------------*/
static void
usbd_bus_mem_flush_all_cb(struct usbd_bus *bus, struct usbd_page_cache *pc,
    struct usbd_page *pg, uint32_t size, uint32_t align)
{
	usbd_pc_cpu_flush(pc);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bus_mem_flush_all - factored out code
 *------------------------------------------------------------------------*/
void
usbd_bus_mem_flush_all(struct usbd_bus *bus, usbd_bus_mem_cb_t *cb)
{
	if (cb) {
		cb(bus, &usbd_bus_mem_flush_all_cb);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bus_mem_alloc_all_cb
 *------------------------------------------------------------------------*/
static void
usbd_bus_mem_alloc_all_cb(struct usbd_bus *bus, struct usbd_page_cache *pc,
    struct usbd_page *pg, uint32_t size, uint32_t align)
{
	if (usbd_pc_alloc_mem(bus->dma_tag_parent,
	    bus->dma_tags, pc, pg, size, align,
	    USB_BUS_DMA_TAG_MAX)) {
		bus->alloc_failed = 1;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bus_mem_alloc_all - factored out code
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usbd_bus_mem_alloc_all(struct usbd_bus *bus, usbd_bus_mem_cb_t *cb)
{
	bus->alloc_failed = 0;

	mtx_init(&(bus->mtx), "USB lock",
	    NULL, MTX_DEF | MTX_RECURSE);

	LIST_INIT(&(bus->intr_list_head));

	if (cb) {
		cb(bus, &usbd_bus_mem_alloc_all_cb);
	}
	if (bus->alloc_failed) {
		usbd_bus_mem_free_all(bus, cb);
	}
	return (bus->alloc_failed);
}

/*------------------------------------------------------------------------*
 *	usbd_bus_mem_free_all_cb
 *------------------------------------------------------------------------*/
static void
usbd_bus_mem_free_all_cb(struct usbd_bus *bus, struct usbd_page_cache *pc,
    struct usbd_page *pg, uint32_t size, uint32_t align)
{
	usbd_pc_free_mem(pc);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_bus_mem_free_all - factored out code
 *------------------------------------------------------------------------*/
void
usbd_bus_mem_free_all(struct usbd_bus *bus, usbd_bus_mem_cb_t *cb)
{
	if (cb) {
		cb(bus, &usbd_bus_mem_free_all_cb);
	}
	usbd_dma_tag_unsetup(bus->dma_tags, USB_BUS_DMA_TAG_MAX);

	mtx_destroy(&(bus->mtx));

	return;
}

/*------------------------------------------------------------------------*
 *	usbd_transfer_setup_sub_malloc
 *
 * This function will allocate DMA'able memory and store the virtual
 * and physical buffer addresses in the structure pointed to by the
 * "info" argument.
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usbd_transfer_setup_sub_malloc(struct usbd_setup_params *parm,
    struct usbd_page_search *info, struct usbd_page_cache **ppc,
    uint32_t size, uint32_t align)
{
	__KASSERT(align > 1, ("Invalid alignment, 0x%08x!\n",
	    align));
	__KASSERT(size > 0, ("Invalid size = 0!\n"));

	if (parm->buf == NULL) {
		/* for the future */
		parm->dma_page_ptr++;
		parm->dma_page_cache_ptr++;
		return (0);
	}
	if (usbd_pc_alloc_mem(parm->udev->bus->dma_tag_parent,
	    parm->dma_tag_p, parm->dma_page_cache_ptr,
	    parm->dma_page_ptr, size, align,
	    parm->dma_tag_max)) {
		return (1);		/* failure */
	}
	if (info) {
		usbd_get_page(parm->dma_page_cache_ptr, 0, info);
	}
	if (ppc) {
		*ppc = parm->dma_page_cache_ptr;
	}
	parm->dma_page_ptr++;
	parm->dma_page_cache_ptr++;

	return (0);
}

/*------------------------------------------------------------------------*
 *	usbd_bus_port_get_device
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
struct usbd_device *
usbd_bus_port_get_device(struct usbd_bus *bus, struct usbd_port *up)
{
	if ((bus == NULL) || (up == NULL)) {
		/* be NULL safe */
		return (NULL);
	}
	if (up->device_index == 0) {
		/* nothing to do */
		return (NULL);
	}
	return (bus->devices[up->device_index]);
}

/*------------------------------------------------------------------------*
 *	usbd_bus_port_set_device
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_bus_port_set_device(struct usbd_bus *bus, struct usbd_port *up,
    struct usbd_device *udev, uint8_t device_index)
{
	if (bus == NULL) {
		/* be NULL safe */
		return;
	}
	/*
	 * There is only one case where we don't
	 * have an USB port, and that is the Root Hub!
         */
	if (up) {
		if (udev) {
			up->device_index = device_index;
		} else {
			device_index = up->device_index;
			up->device_index = 0;
		}
	}
	/*
	 * Make relationships to our new device
	 */
	mtx_lock(&(bus->mtx));
	bus->devices[device_index] = udev;
	mtx_unlock(&(bus->mtx));

	/*
	 * Debug print
	 */
	PRINTFN(1, ("bus %p devices[%u] = %p\n", bus, device_index, udev));

	return;
}
