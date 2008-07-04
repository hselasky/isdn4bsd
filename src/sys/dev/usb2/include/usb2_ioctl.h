/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
 * Copyright (c) 1998 The NetBSD Foundation, Inc. All rights reserved.
 * Copyright (c) 1998 Lennart Augustsson. All rights reserved.
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

#ifndef _USB2_IOCTL_H_
#define	_USB2_IOCTL_H_

#include <sys/ioccom.h>

#define	USB_DEVICE_NAME "usb"

struct usb2_ctl_request {
	void   *ucr_data;
	uint16_t ucr_flags;
#define	USB_USE_POLLING         0x0001	/* internal flag */
#define	USB_SHORT_XFER_OK       0x0004	/* allow short reads */
#define	USB_DELAY_STATUS_STAGE  0x0010	/* insert delay before STATUS stage */
	uint16_t ucr_actlen;		/* actual length transferred */
	uint8_t	ucr_addr;		/* zero - currently not used */
	struct usb2_device_request ucr_request;
};

struct usb2_alt_interface {
	uint8_t	uai_interface_index;
	uint8_t	uai_alt_index;
};

struct usb2_gen_descriptor {
	void   *ugd_data;
	uint16_t ugd_lang_id;
	uint16_t ugd_maxlen;
	uint16_t ugd_actlen;
	uint16_t ugd_offset;
	uint8_t	ugd_config_index;
	uint8_t	ugd_string_index;
	uint8_t	ugd_iface_index;
	uint8_t	ugd_altif_index;
	uint8_t	ugd_endpt_index;
	uint8_t	ugd_report_type;
	uint8_t	reserved[8];
};

#define	USB_MAX_DEVNAMES 4
#define	USB_MAX_DEVNAMELEN 16

struct usb2_device_info {
	uint16_t udi_productNo;
	uint16_t udi_vendorNo;
	uint16_t udi_releaseNo;
	uint16_t udi_power;		/* power consumption in mA, 0 if
					 * selfpowered */
	uint8_t	udi_bus;
	uint8_t	udi_addr;		/* device address */
	uint8_t	udi_class;
	uint8_t	udi_subclass;
	uint8_t	udi_protocol;
	uint8_t	udi_config;
	uint8_t	udi_speed;
	uint8_t	udi_nports;
	uint8_t	udi_ports[16];		/* HUB only: addresses of devices on
					 * ports */
#define	USB_PORT_ENABLED 0xff
#define	USB_PORT_SUSPENDED 0xfe
#define	USB_PORT_POWERED 0xfd
#define	USB_PORT_DISABLED 0xfc

	char	udi_product[128];
	char	udi_vendor[128];
	char	udi_release[8];
	char	udi_devnames[USB_MAX_DEVNAMES][USB_MAX_DEVNAMELEN];
};

struct usb2_device_stats {
	uint32_t uds_requests_ok[4];	/* Indexed by transfer type UE_XXX */
	uint32_t uds_requests_fail[4];	/* Indexed by transfer type UE_XXX */
};

struct usb2_device_enumerate {
	uint8_t	ude_addr;		/* not used */
};

/* USB controller */
#define	USB_REQUEST		_IOWR('U', 1, struct usb2_ctl_request)
#define	USB_SETDEBUG		_IOW ('U', 2, int)
#define	USB_DISCOVER		_IO  ('U', 3)
#define	USB_DEVICEINFO		_IOWR('U', 4, struct usb2_device_info)
#define	USB_DEVICESTATS		_IOR ('U', 5, struct usb2_device_stats)
#define	USB_DEVICEENUMERATE	_IOW ('U', 6, struct usb2_device_enumerate)

/* Generic HID device */
#define	USB_GET_REPORT_DESC	_IOR ('U', 21, struct usb2_gen_descriptor)
#define	USB_SET_IMMED		_IOW ('U', 22, int)
#define	USB_GET_REPORT		_IOWR('U', 23, struct usb2_gen_descriptor)
#define	USB_SET_REPORT		_IOW ('U', 24, struct usb2_gen_descriptor)
#define	USB_GET_REPORT_ID	_IOR ('U', 25, int)

/* Generic USB device */
#define	USB_GET_CONFIG		_IOR ('U', 100, int)
#define	USB_SET_CONFIG		_IOW ('U', 101, int)
#define	USB_GET_ALTINTERFACE	_IOWR('U', 102, struct usb2_alt_interface)
#define	USB_SET_ALTINTERFACE	_IOWR('U', 103, struct usb2_alt_interface)
#define	USB_GET_DEVICE_DESC	_IOR ('U', 105, struct usb2_device_descriptor)
#define	USB_GET_CONFIG_DESC	_IOR ('U', 106, struct usb2_config_descriptor)
#define	USB_GET_INTERFACE_DESC	_IOR ('U', 107, struct usb2_interface_descriptor)
#define	USB_GET_ENDPOINT_DESC	_IOR ('U', 108, struct usb2_endpoint_descriptor)
#define	USB_GET_FULL_DESC	_IOWR('U', 109, struct usb2_gen_descriptor)
#define	USB_GET_STRING_DESC	_IOWR('U', 110, struct usb2_gen_descriptor)
#define	USB_DO_REQUEST		_IOWR('U', 111, struct usb2_ctl_request)
#define	USB_GET_DEVICEINFO	_IOR ('U', 112, struct usb2_device_info)
#define	USB_SET_SHORT_XFER	_IOW ('U', 113, int)
#define	USB_SET_TIMEOUT		_IOW ('U', 114, uint32_t)
#define	USB_GET_FRAME_SIZE	_IOR ('U', 115, uint32_t)
#define	USB_GET_BUFFER_SIZE	_IOR ('U', 117, uint32_t)
#define	USB_SET_BUFFER_SIZE	_IOW ('U', 118, uint32_t)

/* Modem device */
#define	USB_GET_CM_OVER_DATA	_IOR ('U', 130, int)
#define	USB_SET_CM_OVER_DATA	_IOW ('U', 131, int)

#endif					/* _USB2_IOCTL_H_ */
