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

/* USB templates are used to build up real USB descriptors */

#ifndef _USB_TEMPLATE_H_
#define	_USB_TEMPLATE_H_

typedef const void *(usb2_temp_get_string_desc_t)(uint16_t lang_id, uint8_t string_index);

struct usb2_temp_packet_size {
	uint16_t mps[USB_SPEED_MAX];
};

struct usb2_temp_interval {
	uint8_t	bInterval[USB_SPEED_MAX];
};

struct usb2_temp_endpoint_desc {
	const void **ppRawDesc;
	const struct usb2_temp_packet_size *pPacketSize;
	const struct usb2_temp_interval *pIntervals;
	uint8_t	direction;		/* UE_DIR_IN or UE_DIR_OUT */
	uint8_t	bmAttributes;
};

struct usb2_temp_interface_desc {
	const void **ppRawDesc;
	const struct usb2_temp_endpoint_desc **ppEndpoints;
	uint8_t	bInterfaceClass;
	uint8_t	bInterfaceSubClass;
	uint8_t	bInterfaceProtocol;
	uint8_t	iInterface;
	uint8_t	isAltInterface;
};

struct usb2_temp_config_desc {
	const struct usb2_temp_interface_desc **ppIfaceDesc;
	uint8_t	bmAttributes;
	uint8_t	bMaxPower;
	uint8_t	iConfiguration;
};

struct usb2_temp_device_desc {
	usb2_temp_get_string_desc_t *getStringDesc;
	const struct usb2_temp_config_desc **ppConfigDesc;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t	bDeviceClass;
	uint8_t	bDeviceSubClass;
	uint8_t	bDeviceProtocol;
	uint8_t	iManufacturer;
	uint8_t	iProduct;
	uint8_t	iSerialNumber;
};

struct usb2_temp_data {
	const struct usb2_temp_device_desc *tdd;
	struct usb2_device_descriptor udd;	/* device descriptor */
	struct usb2_device_qualifier udq;	/* device qualifier */
};

/* prototypes */

extern const struct usb2_temp_device_desc usb2_template_cdce;
extern const struct usb2_temp_device_desc usb2_template_msc;	/* Mass Storage Class */
extern const struct usb2_temp_device_desc usb2_template_mtp;	/* Message Transfer
								 * Protocol */

#endif					/* _USB_TEMPLATE_H_ */