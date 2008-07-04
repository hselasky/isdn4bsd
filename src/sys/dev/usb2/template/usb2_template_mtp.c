#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/usb2_template_mtp.c $");

/*-
 * Copyright (c) 2008 Hans Petter Selasky <hselasky@freebsd.org>
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
 * This file contains the USB templates for an USB Message Transfer
 * Protocol device.
 */

#include <dev/usb2/include/usb2_standard.h>
#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_revision.h>

#include <dev/usb2/core/usb2_core.h>

#include <dev/usb2/template/usb2_template.h>

enum {
	STRING_LANG_INDEX,
	STRING_MTP_DATA_INDEX,
	STRING_MTP_CONFIG_INDEX,
	STRING_MTP_VENDOR_INDEX,
	STRING_MTP_PRODUCT_INDEX,
	STRING_MTP_SERIAL_INDEX,
	STRING_MTP_MAX,
};

#define	STRING_LANG \
  0x09, 0x04,				/* American English */

#define	STRING_MTP_DATA	\
  'U', 0, 'S', 0, 'B', 0, ' ', 0, \
  'M', 0, 'T', 0, 'P', 0, \
  ' ', 0, 'I', 0, 'n', 0, 't', 0, \
  'e', 0, 'r', 0, 'f', 0, 'a', 0, \
  'c', 0, 'e', 0,

#define	STRING_MTP_CONFIG \
  'D', 0, 'e', 0, 'f', 0, 'a', 0, \
  'u', 0, 'l', 0, 't', 0, ' ', 0, \
  'c', 0, 'o', 0, 'n', 0, 'f', 0, \
  'i', 0, 'g', 0,

#define	STRING_MTP_VENDOR \
  'F', 0, 'r', 0, 'e', 0, 'e', 0, \
  'B', 0, 'S', 0, 'D', 0, ' ', 0, \
  'f', 0, 'o', 0, 'u', 0, 'n', 0, \
  'd', 0, 'a', 0, 't', 0, 'i', 0, \
  'o', 0, 'n', 0,

#define	STRING_MTP_PRODUCT \
  'U', 0, 'S', 0, 'B', 0, ' ', 0, \
  'M', 0, 'T', 0, 'P', 0,

#define	STRING_MTP_SERIAL \
  'J', 0, 'u', 0, 'n', 0, 'e', 0, \
  ' ', 0, '2', 0, '0', 0, '0', 0, \
  '8', 0,

/* make the real string descriptors */

USB_MAKE_STRING_DESC(STRING_LANG, string_lang);
USB_MAKE_STRING_DESC(STRING_MTP_DATA, string_mtp_data);
USB_MAKE_STRING_DESC(STRING_MTP_CONFIG, string_mtp_config);
USB_MAKE_STRING_DESC(STRING_MTP_VENDOR, string_mtp_vendor);
USB_MAKE_STRING_DESC(STRING_MTP_PRODUCT, string_mtp_product);
USB_MAKE_STRING_DESC(STRING_MTP_SERIAL, string_mtp_serial);

/* prototypes */

static usb2_temp_get_string_desc_t mtp_get_string_desc;

static const struct usb2_temp_packet_size bulk_mps = {
	.mps[USB_SPEED_FULL] = 64,
	.mps[USB_SPEED_HIGH] = 512,
};

static const struct usb2_temp_endpoint_desc bulk_out_ep = {
	.pPacketSize = &bulk_mps,
	.direction = UE_DIR_OUT,
	.bmAttributes = UE_BULK,
};

static const struct usb2_temp_endpoint_desc intr_in_ep = {
	.pPacketSize = &bulk_mps,
	.direction = UE_DIR_IN,
	.bmAttributes = UE_INTERRUPT,
};

static const struct usb2_temp_endpoint_desc bulk_in_ep = {
	.pPacketSize = &bulk_mps,
	.direction = UE_DIR_IN,
	.bmAttributes = UE_BULK,
};

static const struct usb2_temp_endpoint_desc *mtp_data_endpoints[] = {
	&bulk_in_ep,
	&intr_in_ep,
	&bulk_out_ep,
	NULL,
};

static const struct usb2_temp_interface_desc mtp_data_interface = {
	.ppEndpoints = mtp_data_endpoints,
	.bInterfaceClass = 6,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = STRING_MTP_DATA_INDEX,
};

static const struct usb2_temp_interface_desc *mtp_interfaces[] = {
	&mtp_data_interface,
	NULL,
};

static const struct usb2_temp_config_desc mtp_config_desc = {
	.ppIfaceDesc = mtp_interfaces,
	.bmAttributes = UC_BUS_POWERED,
	.bMaxPower = 25,		/* 50 mA */
	.iConfiguration = STRING_MTP_CONFIG_INDEX,
};

static const struct usb2_temp_config_desc *mtp_configs[] = {
	&mtp_config_desc,
	NULL,
};

const struct usb2_temp_device_desc usb2_template_mtp = {
	.getStringDesc = &mtp_get_string_desc,
	.ppConfigDesc = mtp_configs,
	.idVendor = 0,
	.idProduct = 0,
	.bcdDevice = 0x0100,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.iManufacturer = STRING_MTP_VENDOR_INDEX,
	.iProduct = STRING_MTP_PRODUCT_INDEX,
	.iSerialNumber = STRING_MTP_SERIAL_INDEX,
};

/*------------------------------------------------------------------------*
 *	mtp_get_string_desc
 *
 * Return values:
 * NULL: Failure. No such string.
 * Else: Success. Pointer to string descriptor is returned.
 *------------------------------------------------------------------------*/
static const void *
mtp_get_string_desc(uint16_t lang_id, uint8_t string_index)
{
	static const void *ptr[STRING_MTP_MAX] = {
		[STRING_LANG_INDEX] = &string_lang,
		[STRING_MTP_DATA_INDEX] = &string_mtp_data,
		[STRING_MTP_CONFIG_INDEX] = &string_mtp_config,
		[STRING_MTP_VENDOR_INDEX] = &string_mtp_vendor,
		[STRING_MTP_PRODUCT_INDEX] = &string_mtp_product,
		[STRING_MTP_SERIAL_INDEX] = &string_mtp_serial,
	};

	if (string_index == 0) {
		return (&string_lang);
	}
	if (lang_id != 0x0409) {
		return (NULL);
	}
	if (string_index < STRING_MTP_MAX) {
		return (ptr[string_index]);
	}
	return (NULL);
}
