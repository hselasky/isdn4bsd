/*
 * Copyright (c) 2003-2005 Craig Boston
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Bill Paul.
 * 4. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Bill Paul AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Bill Paul, THE VOICES IN HIS HEAD OR
 * THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/dev/usb/if_cdcereg.h,v 1.8 2007/06/10 07:33:48 imp Exp $
 */

#ifndef _USB_IF_CDCEREG_H_
#define _USB_IF_CDCEREG_H_

#define	CDCE_ENDPT_MAX	4

struct cdce_type {
	struct usb_devno	cdce_dev;
	uint16_t		cdce_flags;
};

struct cdce_softc {
	void			*sc_evilhack; /* XXX this pointer must be first */

	struct ifmedia		sc_ifmedia;
	struct mtx		sc_mtx;

	struct ifnet		*sc_ifp;
	struct usbd_xfer	*sc_xfer[CDCE_ENDPT_MAX];
	struct usbd_device	*sc_udev;
	device_t		sc_dev;

	uint32_t		sc_unit;

	uint16_t		sc_flags;
#define	CDCE_FLAG_ZAURUS	0x0001
#define	CDCE_FLAG_NO_UNION	0x0002
#define	CDCE_FLAG_LL_READY	0x0004
#define	CDCE_FLAG_HL_READY	0x0008
#define	CDCE_FLAG_WRITE_STALL	0x0010
#define	CDCE_FLAG_READ_STALL	0x0020

	uint8_t			sc_name[16];
	uint8_t			sc_data_iface_no;
	uint8_t			sc_data_iface_index;
};

#endif /* _USB_IF_CDCEREG_H_ */
