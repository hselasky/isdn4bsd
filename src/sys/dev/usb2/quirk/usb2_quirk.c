/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc. All rights reserved.
 * Copyright (c) 1998 Lennart Augustsson. All rights reserved.
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
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

#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_devid.h>

#define	USB_DEBUG_VAR usb2_debug

#include <dev/usb2/core/usb2_core.h>
#include <dev/usb2/core/usb2_lookup.h>
#include <dev/usb2/core/usb2_debug.h>
#include <dev/usb2/core/usb2_dynamic.h>

#include <dev/usb2/quirk/usb2_quirk.h>

MODULE_DEPEND(usb2_quirk, usb2_core, 1, 1, 1);
MODULE_VERSION(usb2_quirk, 1);

/*
 * The following macro adds a quirk for any revision of a device:
 */
#define	USB_VPA(v,p,r,...)					\
    USB_VPI(v,p,((const uint16_t []){__VA_ARGS__}))

/*
 * The following macro adds a quirk for a specific revision of a
 * device:
 */
#define	USB_VPR(v,p,r,...)				\
    USB_VPI(v,p, ((const uint16_t []){__VA_ARGS__})),	\
    USB_DEV_BCD_LTEQ(r),				\
    USB_DEV_BCD_GTEQ(r)

/* try to keep the quirks on one line, hence grepping becomes easier */

static const struct usb2_device_id usb2_quirks[] = {
	{USB_VPA(USB_VENDOR_ASUS, USB_PRODUCT_ASUS_LCM, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPR(USB_VENDOR_INSIDEOUT, USB_PRODUCT_INSIDEOUT_EDGEPORT4, 0x094, UQ_SWAP_UNICODE, UQ_NONE)},
	{USB_VPR(USB_VENDOR_DALLAS, USB_PRODUCT_DALLAS_J6502, 0x0a2, UQ_BAD_ADC, UQ_NONE)},
	{USB_VPR(USB_VENDOR_DALLAS, USB_PRODUCT_DALLAS_J6502, 0x0a2, UQ_AU_NO_XU, UQ_NONE)},
	{USB_VPR(USB_VENDOR_ALTEC, USB_PRODUCT_ALTEC_ADA70, 0x103, UQ_BAD_ADC, UQ_NONE)},
	{USB_VPR(USB_VENDOR_ALTEC, USB_PRODUCT_ALTEC_ASC495, 0x000, UQ_BAD_AUDIO, UQ_NONE)},
	{USB_VPR(USB_VENDOR_QTRONIX, USB_PRODUCT_QTRONIX_980N, 0x110, UQ_SPUR_BUT_UP, UQ_NONE)},
	{USB_VPR(USB_VENDOR_ALCOR2, USB_PRODUCT_ALCOR2_KBD_HUB, 0x001, UQ_SPUR_BUT_UP, UQ_NONE)},
	{USB_VPR(USB_VENDOR_MCT, USB_PRODUCT_MCT_HUB0100, 0x102, UQ_BUS_POWERED, UQ_NONE)},
	{USB_VPR(USB_VENDOR_MCT, USB_PRODUCT_MCT_USB232, 0x102, UQ_BUS_POWERED, UQ_NONE)},
	{USB_VPR(USB_VENDOR_TI, USB_PRODUCT_TI_UTUSB41, 0x110, UQ_POWER_CLAIM, UQ_NONE)},
	{USB_VPR(USB_VENDOR_TELEX, USB_PRODUCT_TELEX_MIC1, 0x009, UQ_AU_NO_FRAC, UQ_NONE)},
	{USB_VPR(USB_VENDOR_SILICONPORTALS, USB_PRODUCT_SILICONPORTALS_YAPPHONE, 0x100, UQ_AU_INP_ASYNC, UQ_NONE)},
	{USB_VPA(USB_VENDOR_LOGITECH, USB_PRODUCT_LOGITECH_UN53B, 0x000, UQ_NO_STRINGS, UQ_NONE)},
	/*
	 * XXX These should have a revision number, but I don't know what
	 * they are.
	 */
	{USB_VPA(USB_VENDOR_HP, USB_PRODUCT_HP_895C, 0x000, UQ_BROKEN_BIDIR, UQ_NONE)},
	{USB_VPA(USB_VENDOR_HP, USB_PRODUCT_HP_880C, 0x000, UQ_BROKEN_BIDIR, UQ_NONE)},
	{USB_VPA(USB_VENDOR_HP, USB_PRODUCT_HP_815C, 0x000, UQ_BROKEN_BIDIR, UQ_NONE)},
	{USB_VPA(USB_VENDOR_HP, USB_PRODUCT_HP_810C, 0x000, UQ_BROKEN_BIDIR, UQ_NONE)},
	{USB_VPA(USB_VENDOR_HP, USB_PRODUCT_HP_830C, 0x000, UQ_BROKEN_BIDIR, UQ_NONE)},
	{USB_VPA(USB_VENDOR_HP, USB_PRODUCT_HP_1220C, 0x000, UQ_BROKEN_BIDIR, UQ_NONE)},
	{USB_VPA(USB_VENDOR_XEROX, USB_PRODUCT_XEROX_WCM15, 0x000, UQ_BROKEN_BIDIR, UQ_NONE)},
	/* Devices which should be ignored by uhid */
	{USB_VPA(USB_VENDOR_APC, USB_PRODUCT_APC_UPS, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_BELKIN, USB_PRODUCT_BELKIN_F6C550AVR, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_DELORME, USB_PRODUCT_DELORME_EARTHMATE, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_ITUNERNET, USB_PRODUCT_ITUNERNET_USBLCD2X20, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_MGE, USB_PRODUCT_MGE_UPS1, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_MGE, USB_PRODUCT_MGE_UPS2, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_APPLE, USB_PRODUCT_APPLE_IPHONE, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_APPLE, USB_PRODUCT_APPLE_IPHONE_3G, 0x000, UQ_HID_IGNORE, UQ_NONE)},
	/* Devices which should be ignored by both ukbd and uhid */
	{USB_VPA(USB_VENDOR_CYPRESS, USB_PRODUCT_CYPRESS_WISPY1A, 0x000, UQ_KBD_IGNORE, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_METAGEEK, USB_PRODUCT_METAGEEK_WISPY1B, 0x000, UQ_KBD_IGNORE, UQ_HID_IGNORE, UQ_NONE)},
	{USB_VPR(USB_VENDOR_TENX, USB_PRODUCT_TENX_UAUDIO0, 0x0101, UQ_AUDIO_SWAP_LR, UQ_NONE)},

	/* MS keyboards do weird things */
	{USB_VPA(USB_VENDOR_MICROSOFT, USB_PRODUCT_MICROSOFT_WLNOTEBOOK, 0x000, UQ_MS_BAD_CLASS, UQ_MS_LEADING_BYTE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_MICROSOFT, USB_PRODUCT_MICROSOFT_WLNOTEBOOK2, 0x000, UQ_MS_BAD_CLASS, UQ_MS_LEADING_BYTE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_MICROSOFT, USB_PRODUCT_MICROSOFT_WLINTELLIMOUSE, 0x000, UQ_MS_LEADING_BYTE, UQ_NONE)},
	{USB_VPA(USB_VENDOR_METAGEEK, USB_PRODUCT_METAGEEK_WISPY24X, 0x000, UQ_KBD_IGNORE, UQ_HID_IGNORE, UQ_NONE)},
};

USB_MAKE_DEBUG_TABLE(USB_QUIRK);

/*------------------------------------------------------------------------*
 *	usb2_quirkstr
 *
 * This function converts an USB quirk code into a string.
 *------------------------------------------------------------------------*/
static const char *
usb2_quirkstr(uint16_t quirk)
{
	return ((quirk < USB_QUIRK_MAX) ?
	    USB_QUIRK[quirk] : "USB_QUIRK_UNKNOWN");
}

/*------------------------------------------------------------------------*
 *	usb2_test_quirk_by_info
 *
 * Returns:
 * 0: Quirk not found
 * Else: Quirk found
 *------------------------------------------------------------------------*/
static uint8_t
usb2_test_quirk_by_info(const struct usb2_lookup_info *info, uint16_t quirk)
{
	const struct usb2_device_id *pe;
	const uint16_t *px;

	if (quirk == UQ_NONE) {
		return (0);
	}
	pe = usb2_lookup_id_by_info(usb2_quirks, sizeof(usb2_quirks), info);
	if (pe && pe->driver_info) {
		px = pe->driver_info;
		while (1) {
			if (*px == quirk) {
				DPRINTF("Found quirk '%s'.\n", usb2_quirkstr(quirk));
				return (1);
			}
			if (*px == UQ_NONE) {
				return (0);
			}
			px++;
		}
	}
	return (0);
}

static void
usb2_quirk_init(void *arg)
{
	/* register our function */
	usb2_test_quirk_p = &usb2_test_quirk_by_info;
	return;
}

SYSINIT(usb2_quirk_init, SI_SUB_LOCK, SI_ORDER_FIRST, usb2_quirk_init, NULL);
SYSUNINIT(usb2_quirk_unload, SI_SUB_LOCK, SI_ORDER_ANY, usb2_quirk_unload, NULL);
