#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/usb_template.c $");

/*-
 * Copyright (c) 2007 Hans Petter Selasky <hselasky@freebsd.org>
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

/*
 * This file contains sub-routines to build up USB descriptors from
 * USB templates.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/endian.h>
#include <sys/queue.h>
#include <sys/lock.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_template.h>
#include <dev/usb/usb_cdc.h>

/*------------------------------------------------------------------------*
 *	usbd_make_raw_desc
 *
 * This function will insert a raw USB descriptor into the generated
 * USB configuration.
 *------------------------------------------------------------------------*/
static void
usbd_make_raw_desc(struct usbd_temp_setup *temp,
    const uint8_t *raw)
{
	void *dst;
	uint8_t len;

	/*
         * The first byte of any USB descriptor gives the length.
         */
	if (raw) {
		len = raw[0];
		if (temp->buf) {
			dst = USBD_ADD_BYTES(temp->buf, temp->size);
			bcopy(raw, dst, len);

			/* check if we have got a CDC union descriptor */

			if ((raw[0] >= sizeof(usb_cdc_union_descriptor_t)) &&
			    (raw[1] == UDESC_CS_INTERFACE) &&
			    (raw[2] == UDESCSUB_CDC_UNION)) {
				usb_cdc_union_descriptor_t *ud = (void *)dst;

				/* update the interface numbers */

				ud->bMasterInterface +=
				    temp->bInterfaceNumber;
				ud->bSlaveInterface[0] +=
				    temp->bInterfaceNumber;
			}
		}
		temp->size += len;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_make_endpoint_desc
 *
 * This function will generate an USB endpoint descriptor from the
 * given USB template endpoint descriptor, which will be inserted into
 * the USB configuration.
 *------------------------------------------------------------------------*/
static void
usbd_make_endpoint_desc(struct usbd_temp_setup *temp,
    const struct usb_temp_endpoint_desc *ted)
{
	usb_endpoint_descriptor_t *ed;
	const void **rd;
	uint16_t old_size;
	uint16_t mps;
	uint8_t ea = 0;			/* Endpoint Address */
	uint8_t et = 0;			/* Endpiont Type */

	/* Reserve memory */
	old_size = temp->size;
	temp->size += sizeof(*ed);

	/* Scan all Raw Descriptors first */

	rd = ted->ppRawDesc;
	if (rd) {
		while (*rd) {
			usbd_make_raw_desc(temp, *rd);
			rd++;
		}
	}
	if (ted->pPacketSize == NULL) {
		/* not initialized */
		temp->err = USBD_ERR_INVAL;
		return;
	}
	mps = ted->pPacketSize->mps[temp->usb_speed];
	if (mps == 0) {
		/* not initialized */
		temp->err = USBD_ERR_INVAL;
		return;
	} else if (mps == UE_ZERO_MPS) {
		/* escape for Zero Max Packet Size */
		mps = 0;
	}
	ea = (ted->direction & (UE_DIR_IN | UE_DIR_OUT));
	et = (ted->bmAttributes & UE_XFERTYPE);

	/*
	 * Fill out the real USB endpoint descriptor
	 * in case there is a buffer present:
	 */
	if (temp->buf) {
		ed = USBD_ADD_BYTES(temp->buf, old_size);
		ed->bLength = sizeof(*ed);
		ed->bDescriptorType = UDESC_ENDPOINT;
		ed->bEndpointAddress = ea;
		ed->bmAttributes = ted->bmAttributes;
		USETW(ed->wMaxPacketSize, mps);

		/* setup bInterval parameter */

		if (ted->pIntervals &&
		    ted->pIntervals->bInterval[temp->usb_speed]) {
			ed->bInterval =
			    ted->pIntervals->bInterval[temp->usb_speed];
		} else {
			switch (et) {
			case UE_BULK:
			case UE_CONTROL:
				ed->bInterval = 0;	/* not used */
				break;
			case UE_INTERRUPT:
				switch (temp->usb_speed) {
				case USB_SPEED_LOW:
				case USB_SPEED_FULL:
					ed->bInterval = 1;	/* 1 ms */
					break;
				default:
					ed->bInterval = 8;	/* 8*125 us */
					break;
				}
				break;
			default:	/* UE_ISOCHRONOUS */
				switch (temp->usb_speed) {
				case USB_SPEED_LOW:
				case USB_SPEED_FULL:
					ed->bInterval = 1;	/* 1 ms */
					break;
				default:
					ed->bInterval = 1;	/* 125 us */
					break;
				}
				break;
			}
		}
	}
	temp->bNumEndpoints++;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_make_interface_desc
 *
 * This function will generate an USB interface descriptor from the
 * given USB template interface descriptor, which will be inserted
 * into the USB configuration.
 *------------------------------------------------------------------------*/
static void
usbd_make_interface_desc(struct usbd_temp_setup *temp,
    const struct usb_temp_interface_desc *tid)
{
	usb_interface_descriptor_t *id;
	const struct usb_temp_endpoint_desc **ted;
	const void **rd;
	uint16_t old_size;

	/* Reserve memory */

	old_size = temp->size;
	temp->size += sizeof(*id);

	/* Update interface and alternate interface numbers */

	if (tid->isAltInterface == 0) {
		temp->bAlternateSetting = 0;
		temp->bInterfaceNumber++;
	} else {
		temp->bAlternateSetting++;
	}

	/* Scan all Raw Descriptors first */

	rd = tid->ppRawDesc;

	if (rd) {
		while (*rd) {
			usbd_make_raw_desc(temp, *rd);
			rd++;
		}
	}
	/* Reset some counters */

	temp->bNumEndpoints = 0;

	/* Scan all Endpoint Descriptors second */

	ted = tid->ppEndpoints;
	if (ted) {
		while (*ted) {
			usbd_make_endpoint_desc(temp, *ted);
			ted++;
		}
	}
	/*
	 * Fill out the real USB interface descriptor
	 * in case there is a buffer present:
	 */
	if (temp->buf) {
		id = USBD_ADD_BYTES(temp->buf, old_size);
		id->bLength = sizeof(*id);
		id->bDescriptorType = UDESC_INTERFACE;
		id->bInterfaceNumber = temp->bInterfaceNumber;
		id->bAlternateSetting = temp->bAlternateSetting;
		id->bNumEndpoints = temp->bNumEndpoints;
		id->bInterfaceClass = tid->bInterfaceClass;
		id->bInterfaceSubClass = tid->bInterfaceSubClass;
		id->bInterfaceProtocol = tid->bInterfaceProtocol;
		id->iInterface = tid->iInterface;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_make_config_desc
 *
 * This function will generate an USB config descriptor from the given
 * USB template config descriptor, which will be inserted into the USB
 * configuration.
 *------------------------------------------------------------------------*/
static void
usbd_make_config_desc(struct usbd_temp_setup *temp,
    const struct usb_temp_config_desc *tcd)
{
	usb_config_descriptor_t *cd;
	const struct usb_temp_interface_desc **tid;
	uint16_t old_size;

	/* Reserve memory */

	old_size = temp->size;
	temp->size += sizeof(*cd);

	/* Reset some counters */

	temp->bInterfaceNumber = 0 - 1;
	temp->bAlternateSetting = 0;

	/* Scan all the USB interfaces */

	tid = tcd->ppIfaceDesc;
	if (tid) {
		while (*tid) {
			usbd_make_interface_desc(temp, *tid);
			tid++;
		}
	}
	/*
	 * Fill out the real USB config descriptor
	 * in case there is a buffer present:
	 */
	if (temp->buf) {
		cd = USBD_ADD_BYTES(temp->buf, old_size);

		/* compute total size */
		old_size = temp->size - old_size;

		cd->bLength = sizeof(*cd);
		cd->bDescriptorType = UDESC_CONFIG;
		USETW(cd->wTotalLength, old_size);
		cd->bNumInterface = temp->bInterfaceNumber + 1;
		cd->bConfigurationValue = temp->bConfigurationValue;
		cd->iConfiguration = tcd->iConfiguration;
		cd->bmAttributes = tcd->bmAttributes;
		cd->bMaxPower = tcd->bMaxPower;
		cd->bmAttributes |= (UC_REMOTE_WAKEUP | UC_BUS_POWERED);

		if (temp->self_powered) {
			cd->bmAttributes |= UC_SELF_POWERED;
		} else {
			cd->bmAttributes &= ~UC_SELF_POWERED;
		}
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_make_device_desc
 *
 * This function will generate an USB device descriptor from the
 * given USB template device descriptor.
 *------------------------------------------------------------------------*/
static void
usbd_make_device_desc(struct usbd_temp_setup *temp,
    const struct usb_temp_device_desc *tdd)
{
	struct usb_temp_data *utd;
	const struct usb_temp_config_desc **tcd;
	uint16_t old_size;

	/* Reserve memory */

	old_size = temp->size;
	temp->size += sizeof(*utd);

	/* Scan all the USB configs */

	temp->bConfigurationValue = 1;
	tcd = tdd->ppConfigDesc;
	if (tcd) {
		while (*tcd) {
			usbd_make_config_desc(temp, *tcd);
			temp->bConfigurationValue++;
			tcd++;
		}
	}
	/*
	 * Fill out the real USB device descriptor
	 * in case there is a buffer present:
	 */

	if (temp->buf) {
		utd = USBD_ADD_BYTES(temp->buf, old_size);

		/* Store a pointer to our template device descriptor */
		utd->tdd = tdd;

		/* Fill out USB device descriptor */
		utd->udd.bLength = sizeof(utd->udd);
		utd->udd.bDescriptorType = UDESC_DEVICE;
		utd->udd.bDeviceClass = tdd->bDeviceClass;
		utd->udd.bDeviceSubClass = tdd->bDeviceSubClass;
		utd->udd.bDeviceProtocol = tdd->bDeviceProtocol;
		USETW(utd->udd.idVendor, tdd->idVendor);
		USETW(utd->udd.idProduct, tdd->idProduct);
		USETW(utd->udd.bcdDevice, tdd->bcdDevice);
		utd->udd.iManufacturer = tdd->iManufacturer;
		utd->udd.iProduct = tdd->iProduct;
		utd->udd.iSerialNumber = tdd->iSerialNumber;
		utd->udd.bNumConfigurations = temp->bConfigurationValue - 1;

		/*
		 * Fill out the USB device qualifier. Pretend that we
		 * don't support any other speeds by setting
		 * "bNumConfigurations" equal to zero. That saves us
		 * generating an extra set of configuration
		 * descriptors.
		 */
		utd->udq.bLength = sizeof(utd->udq);
		utd->udq.bDescriptorType = UDESC_DEVICE_QUALIFIER;
		utd->udq.bDeviceClass = tdd->bDeviceClass;
		utd->udq.bDeviceSubClass = tdd->bDeviceSubClass;
		utd->udq.bDeviceProtocol = tdd->bDeviceProtocol;
		utd->udq.bNumConfigurations = 0;
		USETW(utd->udq.bcdUSB, 0x0200);
		utd->udq.bMaxPacketSize0 = 0;

		switch (temp->usb_speed) {
		case USB_SPEED_LOW:
			USETW(utd->udd.bcdUSB, 0x0101);
			utd->udd.bMaxPacketSize = 8;
			break;
		case USB_SPEED_FULL:
			USETW(utd->udd.bcdUSB, 0x0101);
			utd->udd.bMaxPacketSize = 32;
			break;
		case USB_SPEED_HIGH:
			USETW(utd->udd.bcdUSB, 0x0200);
			utd->udd.bMaxPacketSize = 64;
			break;
		case USB_SPEED_VARIABLE:
			USETW(utd->udd.bcdUSB, 0x0250);
			utd->udd.bMaxPacketSize = 255;	/* 512 bytes */
			break;
		default:
			temp->err = USBD_ERR_INVAL;
			break;
		}
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_hw_ep_match
 *
 * Return values:
 *    0: The endpoint profile does not match the criterias
 * Else: The endpoint profile matches the criterias
 *------------------------------------------------------------------------*/
static uint8_t
usbd_hw_ep_match(const struct usbd_hw_ep_profile *pf,
    uint8_t ep_type, uint8_t ep_dir_in)
{
	if (ep_type == UE_CONTROL) {
		/* special */
		return (pf->support_control);
	}
	if ((pf->support_in && ep_dir_in) ||
	    (pf->support_out && !ep_dir_in)) {
		if ((pf->support_interrupt && (ep_type == UE_INTERRUPT)) ||
		    (pf->support_isochronous && (ep_type == UE_ISOCHRONOUS)) ||
		    (pf->support_bulk && (ep_type == UE_BULK))) {
			return (1);
		}
	}
	return (0);
}

/*------------------------------------------------------------------------*
 *	usbd_hw_ep_find_match
 *
 * This function is used to find the best matching endpoint profile
 * for and endpoint belonging to an USB descriptor.
 *
 * Return values:
 *    0: Success. Got a match.
 * Else: Failure. No match.
 *------------------------------------------------------------------------*/
static uint8_t
usbd_hw_ep_find_match(struct usbd_hw_ep_scratch *ues,
    struct usbd_sw_ep_scratch *ep, uint8_t is_simplex)
{
	const struct usbd_hw_ep_profile *pf;
	uint16_t distance;
	uint16_t temp;
	uint8_t n;
	uint8_t best_n;
	uint8_t dir_in;
	uint8_t dir_out;

	distance = 0xFFFF;
	best_n = 0;

	if (ep->needs_ep_type == UE_CONTROL) {
		dir_in = 1;
		dir_out = 1;
	} else {
		if (ep->needs_in) {
			ep->needs_in = 0;
			dir_in = 1;
			dir_out = 0;
		} else {
			ep->needs_out = 0;
			dir_in = 0;
			dir_out = 1;
		}
	}

	for (n = 1; n != (USB_MAX_ENDPOINTS / 2); n++) {

		/* check if IN-endpoint is reserved */
		if (dir_in) {
			if (ues->bmInAlloc[n / 8] & (1 << (n % 8))) {
				/* mismatch */
				continue;
			}
		}
		/* check if OUT-endpoint is reserved */
		if (dir_out) {
			if (ues->bmOutAlloc[n / 8] & (1 << (n % 8))) {
				/* mismatch */
				continue;
			}
		}
		/* get HW endpoint profile */
		(ues->methods->get_hw_ep_profile) (ues->udev, &pf, n);
		if (pf == NULL) {
			/* end of profiles */
			break;
		}
		/* check simplex */
		if (pf->is_simplex == is_simplex) {
			/* mismatch */
			continue;
		}
		/* check if HW endpoint matches */
		if (!usbd_hw_ep_match(pf, ep->needs_ep_type, dir_in)) {
			/* mismatch */
			continue;
		}
		if (pf->max_frame_size >= ep->max_frame_size) {
			temp = (pf->max_frame_size - ep->max_frame_size);
			if (distance > temp) {
				distance = temp;
				best_n = n;
				ep->pf = pf;
			}
		} else if ((ep->needs_ep_type == UE_BULK) ||
		    (ep->needs_ep_type == UE_CONTROL)) {
			/* frame size is not so important */
			temp = (ep->max_frame_size - pf->max_frame_size);
			if (distance > temp) {
				distance = temp;
				best_n = n;
				ep->pf = pf;
			}
		}
	}

	/* see if we got a match */
	if (best_n != 0) {
		/* get the correct profile */
		pf = ep->pf;

		/* reserve IN-endpoint */
		if (dir_in || pf->is_simplex) {
			ues->bmInAlloc[best_n / 8] |=
			    (1 << (best_n % 8));
			ep->hw_endpoint_in = best_n | UE_DIR_IN;
		}
		/* reserve OUT-endpoint */
		if (dir_out || pf->is_simplex) {
			ues->bmOutAlloc[best_n / 8] |=
			    (1 << (best_n % 8));
			ep->hw_endpoint_out = best_n | UE_DIR_OUT;
		}
		/*
		 * In case we choose an endpoint having a smaller Maximum
		 * Frame Size than we wanted, we need to update the Maximum
		 * Frame Size !
		 */
		if (ep->max_frame_size > pf->max_frame_size) {
			ep->max_frame_size = pf->max_frame_size;
		}
		return (0);		/* got a match */
	}
	return (1);			/* failure */
}

/*------------------------------------------------------------------------*
 *	usbd_hw_ep_get_needs
 *
 * This function will figure out the type and number of endpoints
 * which are needed for an USB configuration.
 *
 * Return values:
 *    0: Success.
 * Else: Failure.
 *------------------------------------------------------------------------*/
static uint8_t
usbd_hw_ep_get_needs(struct usbd_hw_ep_scratch *ues,
    uint8_t ep_type, uint8_t is_complete)
{
	struct usbd_sw_ep_scratch *ep_iface;
	struct usbd_sw_ep_scratch *ep_curr;
	struct usbd_sw_ep_scratch *ep_max;
	struct usbd_sw_ep_scratch *ep_end;
	usb_descriptor_t *desc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	uint16_t wMaxPacketSize;
	uint16_t temp;
	uint8_t allow_override;
	uint8_t speed;

	ep_iface = ues->ep_max;
	ep_curr = ues->ep_max;
	ep_end = ues->ep + USB_MAX_ENDPOINTS;
	ep_max = ues->ep_max;
	desc = NULL;
	speed = ues->udev->speed;

repeat:

	while ((desc = usbd_desc_foreach(ues->cd, desc))) {

		if ((desc->bDescriptorType == UDESC_INTERFACE) &&
		    (desc->bLength >= sizeof(*id))) {

			id = (void *)desc;

			if (id->bAlternateSetting == 0) {
				/* going forward */
				ep_iface = ep_max;
			} else {
				/* reset */
				ep_curr = ep_iface;
			}
		}
		if ((desc->bDescriptorType == UDESC_ENDPOINT) &&
		    (desc->bLength >= sizeof(*ed))) {

			ed = (void *)desc;

			goto handle_endpoint_desc;
		}
	}
	ues->ep_max = ep_max;
	return (0);

handle_endpoint_desc:
	temp = (ed->bmAttributes & UE_XFERTYPE);

	if (temp == ep_type) {

		if (ep_curr == ep_end) {
			/* too many endpoints */
			return (1);	/* failure */
		}
		wMaxPacketSize = UGETW(ed->wMaxPacketSize);
		if ((wMaxPacketSize & 0xF800) &&
		    (speed == USB_SPEED_HIGH)) {
			/* handle frame multiplier */
			temp = (wMaxPacketSize >> 11) & 3;
			wMaxPacketSize &= 0x7FF;
			if (temp == 2) {
				wMaxPacketSize *= 2;
			} else {
				wMaxPacketSize *= 3;
			}
			allow_override = 0;
		} else {
			if ((ep_type == UE_BULK) ||
			    (ep_type == UE_CONTROL)) {
				allow_override = 1;
			} else {
				allow_override = 0;
			}
		}

		if (is_complete) {

			/*
			 * We assume that
			 * "ep_curr->max_frame_size"
			 * is correct according to the
			 * speed we are connected at !
			 */
			while (1) {

				if (wMaxPacketSize <=
				    ep_curr->max_frame_size) {
					break;
				}
				if (wMaxPacketSize < 8) {
					return (1);	/* failure */
				}
				if (!allow_override) {
					return (1);	/* failure */
				}
				/*
				 * We have a BULK or CONTROL
				 * endpoint having a packet
				 * size that the hardware
				 * cannot handle ! Try to
				 * work it around!
				 */
				wMaxPacketSize /= 2;
				USETW(ed->wMaxPacketSize,
				    wMaxPacketSize);
			}

			if (ed->bEndpointAddress & UE_DIR_IN) {
				ed->bEndpointAddress =
				    ep_curr->hw_endpoint_in;
			} else {
				ed->bEndpointAddress =
				    ep_curr->hw_endpoint_out;
			}

		} else {

			/* compute the maximum frame size */
			if (ep_curr->max_frame_size < wMaxPacketSize) {
				ep_curr->max_frame_size = wMaxPacketSize;
			}
			if (temp == UE_CONTROL) {
				ep_curr->needs_in = 1;
				ep_curr->needs_out = 1;
			} else {
				if (ed->bEndpointAddress & UE_DIR_IN) {
					ep_curr->needs_in = 1;
				} else {
					ep_curr->needs_out = 1;
				}
			}
			ep_curr->needs_ep_type = ep_type;
		}

		ep_curr++;
		if (ep_max < ep_curr) {
			ep_max = ep_curr;
		}
	}
	goto repeat;
}

/*------------------------------------------------------------------------*
 *	usbd_hw_ep_resolve
 *
 * This function will try to resolve endpoint requirements by the
 * given endpoint profiles that the USB hardware reports.
 *
 * Return values:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static usbd_status_t
usbd_hw_ep_resolve(struct usbd_device *udev,
    usb_descriptor_t *desc)
{
	struct usbd_hw_ep_scratch *ues;
	struct usbd_sw_ep_scratch *ep;
	const struct usbd_hw_ep_profile *pf;
	struct usbd_bus_methods *methods;
	usb_device_descriptor_t *dd;
	uint16_t mps;

	if (desc == NULL) {
		return (USBD_ERR_INVAL);
	}
	/* get bus methods */
	methods = udev->bus->methods;

	if (methods->get_hw_ep_profile == NULL) {
		return (USBD_ERR_INVAL);
	}
	if (desc->bDescriptorType == UDESC_DEVICE) {

		if (desc->bLength < sizeof(*dd)) {
			return (USBD_ERR_INVAL);
		}
		dd = (void *)desc;

		/* get HW control endpoint 0 profile */
		(methods->get_hw_ep_profile) (udev, &pf, 0);
		if (pf == NULL) {
			return (USBD_ERR_INVAL);
		}
		if (!usbd_hw_ep_match(pf, UE_CONTROL, 0)) {
			PRINTFN(-1, ("Endpoint 0 does not "
			    "support control\n"));
			return (USBD_ERR_INVAL);
		}
		mps = dd->bMaxPacketSize;

		if (udev->speed == USB_SPEED_FULL) {
			/*
			 * We can optionally choose another packet size !
			 */
			while (1) {
				/* check if "mps" is ok */
				if (pf->max_frame_size >= mps) {
					break;
				}
				/* reduce maximum packet size */
				mps /= 2;

				/* check if "mps" is too small */
				if (mps < 8) {
					return (USBD_ERR_INVAL);
				}
			}

			dd->bMaxPacketSize = mps;

		} else {
			/* We only have one choice */
			if (mps == 255) {
				mps = 512;
			}
			/* Check if we support the specified wMaxPacketSize */
			if (pf->max_frame_size < mps) {
				return (USBD_ERR_INVAL);
			}
		}
		return (0);		/* success */
	}
	if (desc->bDescriptorType != UDESC_CONFIG) {
		return (USBD_ERR_INVAL);
	}
	if (desc->bLength < sizeof(*(ues->cd))) {
		return (USBD_ERR_INVAL);
	}
	ues = udev->scratch[0].hw_ep_scratch;

	bzero(ues, sizeof(*ues));

	ues->ep_max = ues->ep;
	ues->cd = (void *)desc;
	ues->methods = methods;
	ues->udev = udev;

	/* Get all the endpoints we need */

	if (usbd_hw_ep_get_needs(ues, UE_ISOCHRONOUS, 0) ||
	    usbd_hw_ep_get_needs(ues, UE_INTERRUPT, 0) ||
	    usbd_hw_ep_get_needs(ues, UE_CONTROL, 0) ||
	    usbd_hw_ep_get_needs(ues, UE_BULK, 0)) {
		PRINTFN(-1, ("Could not get needs\n"));
		return (USBD_ERR_INVAL);
	}
	for (ep = ues->ep; ep != ues->ep_max; ep++) {

		while (ep->needs_in || ep->needs_out) {

			/*
		         * First try to use a simplex endpoint.
		         * Then try to use a duplex endpoint.
		         */
			if (usbd_hw_ep_find_match(ues, ep, 1) &&
			    usbd_hw_ep_find_match(ues, ep, 0)) {
				PRINTFN(-1, ("Could not find match\n"));
				return (USBD_ERR_INVAL);
			}
		}
	}

	ues->ep_max = ues->ep;

	/* Update all endpoint addresses */

	if (usbd_hw_ep_get_needs(ues, UE_ISOCHRONOUS, 1) ||
	    usbd_hw_ep_get_needs(ues, UE_INTERRUPT, 1) ||
	    usbd_hw_ep_get_needs(ues, UE_CONTROL, 1) ||
	    usbd_hw_ep_get_needs(ues, UE_BULK, 1)) {
		PRINTFN(-1, ("Could not update endpoint address\n"));
		return (USBD_ERR_INVAL);
	}
	return (0);			/* success */
}

/*------------------------------------------------------------------------*
 *	usbd_temp_get_tdd
 *
 * Returns:
 *  NULL: No USB template device descriptor found.
 *  Else: Pointer to the USB template device descriptor.
 *------------------------------------------------------------------------*/
static const struct usb_temp_device_desc *
usbd_temp_get_tdd(struct usbd_device *udev)
{
	if (udev->usb_template_ptr == NULL) {
		return (NULL);
	}
	return (udev->usb_template_ptr->tdd);
}

/*------------------------------------------------------------------------*
 *	usbd_temp_get_device_desc
 *
 * Returns:
 *  NULL: No USB device descriptor found.
 *  Else: Pointer to USB device descriptor.
 *------------------------------------------------------------------------*/
static void *
usbd_temp_get_device_desc(struct usbd_device *udev)
{
	usb_device_descriptor_t *dd;

	if (udev->usb_template_ptr == NULL) {
		return (NULL);
	}
	dd = &(udev->usb_template_ptr->udd);
	if (dd->bDescriptorType != UDESC_DEVICE) {
		/* sanity check failed */
		return (NULL);
	}
	return (dd);
}

/*------------------------------------------------------------------------*
 *	usbd_temp_get_qualifier_desc
 *
 * Returns:
 *  NULL: No USB device_qualifier descriptor found.
 *  Else: Pointer to USB device_qualifier descriptor.
 *------------------------------------------------------------------------*/
static void *
usbd_temp_get_qualifier_desc(struct usbd_device *udev)
{
	usb_device_qualifier_t *dq;

	if (udev->usb_template_ptr == NULL) {
		return (NULL);
	}
	dq = &(udev->usb_template_ptr->udq);
	if (dq->bDescriptorType != UDESC_DEVICE_QUALIFIER) {
		/* sanity check failed */
		return (NULL);
	}
	return (dq);
}

/*------------------------------------------------------------------------*
 *	usbd_temp_get_config_desc
 *
 * Returns:
 *  NULL: No USB config descriptor found.
 *  Else: Pointer to USB config descriptor having index "index".
 *------------------------------------------------------------------------*/
static void *
usbd_temp_get_config_desc(struct usbd_device *udev,
    uint16_t *pLength, uint8_t index)
{
	usb_device_descriptor_t *dd;
	usb_config_descriptor_t *cd;
	uint16_t temp;

	if (udev->usb_template_ptr == NULL) {
		return (NULL);
	}
	dd = &(udev->usb_template_ptr->udd);
	cd = (void *)(udev->usb_template_ptr + 1);

	if (index >= dd->bNumConfigurations) {
		/* out of range */
		return (NULL);
	}
	while (index--) {
		if (cd->bDescriptorType != UDESC_CONFIG) {
			/* sanity check failed */
			return (NULL);
		}
		temp = UGETW(cd->wTotalLength);
		cd = USBD_ADD_BYTES(cd, temp);
	}

	if (pLength) {
		*pLength = UGETW(cd->wTotalLength);
	}
	return (cd);
}

/*------------------------------------------------------------------------*
 *	usbd_temp_get_string_desc
 *
 * Returns:
 *  NULL: No string descriptor found.
 *  Else: Pointer to a string descriptor.
 *------------------------------------------------------------------------*/
static const void *
usbd_temp_get_string_desc(struct usbd_device *udev,
    uint16_t lang_id, uint8_t string_index)
{
	const struct usb_temp_device_desc *tdd;

	tdd = usbd_temp_get_tdd(udev);
	if (tdd == NULL) {
		return (NULL);
	}
	if (tdd->getStringDesc == NULL) {
		return (NULL);
	}
	return ((tdd->getStringDesc) (lang_id, string_index));
}

/*------------------------------------------------------------------------*
 *	usbd_temp_get_hub_desc
 *
 * Returns:
 *  NULL: No USB HUB descriptor found.
 *  Else: Pointer to a USB HUB descriptor.
 *------------------------------------------------------------------------*/
static const void *
usbd_temp_get_hub_desc(struct usbd_device *udev)
{
	return (NULL);			/* needs to be implemented */
}

/*------------------------------------------------------------------------*
 *	usbd_temp_get_desc
 *
 * This function is a demultiplexer for local USB device side control
 * endpoint requests.
 *------------------------------------------------------------------------*/
void
usbd_temp_get_desc(struct usbd_device *udev, usb_device_request_t *req,
    const void **pPtr, uint16_t *pLength)
{
	const uint8_t *buf;
	uint16_t len;

	buf = NULL;
	len = 0;

	switch (req->bmRequestType) {
	case UT_READ_DEVICE:
		switch (req->bRequest) {
		case UR_GET_DESCRIPTOR:
			goto tr_handle_get_descriptor;
		default:
			goto tr_stalled;
		}
		break;
	case UT_READ_CLASS_DEVICE:
		switch (req->bRequest) {
		case UR_GET_DESCRIPTOR:
			goto tr_handle_get_class_descriptor;
		default:
			goto tr_stalled;
		}
		break;
	default:
		goto tr_stalled;
	}

tr_handle_get_descriptor:
	switch (req->wValue[1]) {
	case UDESC_DEVICE:
		if (req->wValue[0]) {
			goto tr_stalled;
		}
		buf = usbd_temp_get_device_desc(udev);
		goto tr_valid;
	case UDESC_DEVICE_QUALIFIER:
		if (udev->speed != USB_SPEED_HIGH) {
			goto tr_stalled;
		}
		if (req->wValue[0]) {
			goto tr_stalled;
		}
		buf = usbd_temp_get_qualifier_desc(udev);
		goto tr_valid;
	case UDESC_OTHER_SPEED_CONFIGURATION:
		if (udev->speed != USB_SPEED_HIGH) {
			goto tr_stalled;
		}
	case UDESC_CONFIG:
		buf = usbd_temp_get_config_desc(udev,
		    &len, req->wValue[0]);
		goto tr_valid;
	case UDESC_STRING:
		buf = usbd_temp_get_string_desc(udev,
		    UGETW(req->wIndex), req->wValue[0]);
		goto tr_valid;
	default:
		goto tr_stalled;
	}
	goto tr_stalled;

tr_handle_get_class_descriptor:
	if (req->wValue[0]) {
		goto tr_stalled;
	}
	buf = usbd_temp_get_hub_desc(udev);
	goto tr_valid;

tr_valid:
	if (buf == NULL) {
		goto tr_stalled;
	}
	if (len == 0) {
		len = buf[0];
	}
	*pPtr = buf;
	*pLength = len;
	return;

tr_stalled:
	*pPtr = NULL;
	*pLength = 0;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_temp_setup
 *
 * This function generates USB descriptors according to the given USB
 * template device descriptor. It will also try to figure out the best
 * matching endpoint addresses using the hardware endpoint profiles.
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbd_temp_setup(struct usbd_device *udev,
    const struct usb_temp_device_desc *tdd)
{
	struct usbd_temp_setup *uts;
	void *buf;
	uint8_t n;

	if (tdd == NULL) {
		/* be NULL safe */
		return (0);
	}
	uts = udev->scratch[0].temp_setup;

	bzero(uts, sizeof(*uts));

	uts->usb_speed = udev->speed;
	uts->self_powered = udev->flags.self_powered;

	/* first pass */

	usbd_make_device_desc(uts, tdd);

	if (uts->err) {
		/* some error happened */
		return (uts->err);
	}
	/* sanity check */
	if (uts->size == 0) {
		return (USBD_ERR_INVAL);
	}
	/* allocate zeroed memory */
	uts->buf = malloc(uts->size, M_USB, M_WAITOK | M_ZERO);
	if (uts->buf == NULL) {
		/* could not allocate memory */
		return (USBD_ERR_NOMEM);
	}
	/* second pass */

	uts->size = 0;

	usbd_make_device_desc(uts, tdd);

	/*
	 * Store a pointer to our descriptors:
	 */
	udev->usb_template_ptr = uts->buf;

	if (uts->err) {
		/* some error happened during second pass */
		goto error;
	}
	/*
	 * Resolve all endpoint addresses !
	 */
	buf = usbd_temp_get_device_desc(udev);
	uts->err = usbd_hw_ep_resolve(udev, buf);
	if (uts->err) {
		PRINTFN(-1, ("Could not resolve endpoints for "
		    "Device Descriptor, error = %s\n",
		    usbd_errstr(uts->err)));
		goto error;
	}
	for (n = 0;; n++) {

		buf = usbd_temp_get_config_desc(udev, NULL, n);
		if (buf == NULL) {
			break;
		}
		uts->err = usbd_hw_ep_resolve(udev, buf);
		if (uts->err) {
			PRINTFN(-1, ("Could not resolve endpoints for "
			    "Config Descriptor %u, error = %s\n", n,
			    usbd_errstr(uts->err)));
			goto error;
		}
	}
	return (uts->err);

error:
	usbd_temp_unsetup(udev);
	return (uts->err);
}

/*------------------------------------------------------------------------*
 *	usbd_temp_unsetup
 *
 * This function frees any memory associated with the currently
 * setup template, if any.
 *------------------------------------------------------------------------*/
void
usbd_temp_unsetup(struct usbd_device *udev)
{
	if (udev->usb_template_ptr) {

		free(udev->usb_template_ptr, M_USB);

		udev->usb_template_ptr = NULL;
	}
	return;
}
