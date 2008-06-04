#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/usb_requests.c $");

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
#include <sys/lock.h>
#include <sys/ctype.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>
#include <dev/usb/usb_hid.h>

#ifdef USB_DEBUG
static int usb_pr_poll_delay = USB_PORT_RESET_DELAY;
static int usb_pr_recovery_delay = USB_PORT_RESET_RECOVERY;

SYSCTL_INT(_hw_usb, OID_AUTO, pr_poll_delay, CTLFLAG_RW,
    &usb_pr_poll_delay, 0, "USB port reset poll delay in ms");
SYSCTL_INT(_hw_usb, OID_AUTO, pr_recovery_delay, CTLFLAG_RW,
    &usb_pr_recovery_delay, 0, "USB port reset recovery delay in ms");
#endif

/*------------------------------------------------------------------------*
 *	usbreq_reset_port
 *
 * This function will instruct an USB HUB to perform a reset sequence
 * on the specified port number.
 *
 * Returns:
 *    0: Success. The USB device should now be at address zero.
 * Else: Failure. No USB device is present and the USB port should be
 *       disabled.
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_reset_port(struct usbd_device *udev, struct mtx *mtx, uint8_t port)
{
	usb_port_status_t ps;
	usbd_status_t err;
	uint16_t n;

#ifdef USB_DEBUG
	uint16_t pr_poll_delay;
	uint16_t pr_recovery_delay;

#endif
	err = usbreq_set_port_feature(udev, mtx, port, UHF_PORT_RESET);
	if (err) {
		goto done;
	}
#ifdef USB_DEBUG
	/* range check input parameters */
	pr_poll_delay = usb_pr_poll_delay;
	if (pr_poll_delay < 1) {
		pr_poll_delay = 1;
	} else if (pr_poll_delay > 1000) {
		pr_poll_delay = 1000;
	}
	pr_recovery_delay = usb_pr_recovery_delay;
	if (pr_recovery_delay > 1000) {
		pr_recovery_delay = 1000;
	}
#endif
	n = 0;
	while (1) {
#ifdef USB_DEBUG
		/* wait for the device to recover from reset */
		usbd_pause_mtx(mtx, pr_poll_delay);
		n += pr_poll_delay;
#else
		/* wait for the device to recover from reset */
		usbd_pause_mtx(mtx, USB_PORT_RESET_DELAY);
		n += USB_PORT_RESET_DELAY;
#endif
		err = usbreq_get_port_status(udev, mtx, &ps, port);
		if (err) {
			goto done;
		}
		/* if the device disappeared, just give up */
		if (!(UGETW(ps.wPortStatus) & UPS_CURRENT_CONNECT_STATUS)) {
			goto done;
		}
		/* check if reset is complete */
		if (UGETW(ps.wPortChange) & UPS_C_PORT_RESET) {
			break;
		}
		/* check for timeout */
		if (n > 1000) {
			n = 0;
			break;
		}
	}

	/* clear port reset first */
	err = usbreq_clear_port_feature(
	    udev, mtx, port, UHF_C_PORT_RESET);
	if (err) {
		goto done;
	}
	/* check for timeout */
	if (n == 0) {
		err = USBD_ERR_TIMEOUT;
		goto done;
	}
#ifdef USB_DEBUG
	/* wait for the device to recover from reset */
	usbd_pause_mtx(mtx, pr_recovery_delay);
#else
	/* wait for the device to recover from reset */
	usbd_pause_mtx(mtx, USB_PORT_RESET_RECOVERY);
#endif

done:
	PRINTFN(1, ("port %d reset returning error=%s\n",
	    port, usbd_errstr(err)));
	return (err);
}

/*------------------------------------------------------------------------*
 *	usbreq_get_desc
 *
 * This function can be used to retrieve USB descriptors. It contains
 * some additional logic like zeroing of missing descriptor bytes and
 * retrying an USB descriptor in case of failure. The "min_len"
 * argument specifies the minimum descriptor length. The "max_len"
 * argument specifies the maximum descriptor length. If the real
 * descriptor length is less than the minimum length the missing
 * byte(s) will be zeroed. The length field, first byte, of the USB
 * descriptor will get overwritten in case it indicates a length that
 * is too big. Also the type field, second byte, of the USB descriptor
 * will get forced to the correct type.
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_desc(struct usbd_device *udev, struct mtx *mtx, void *desc,
    uint16_t min_len, uint16_t max_len,
    uint16_t id, uint8_t type, uint8_t index,
    uint8_t retries)
{
	usb_device_request_t req;
	uint8_t *buf;
	usbd_status_t err;

	PRINTFN(3, ("id=%d, type=%d, index=%d, max_len=%d\n",
	    id, type, index, max_len));

	req.bmRequestType = UT_READ_DEVICE;
	req.bRequest = UR_GET_DESCRIPTOR;
	USETW2(req.wValue, type, index);
	USETW(req.wIndex, id);

	while (1) {

		if ((min_len < 2) || (max_len < 2)) {
			err = USBD_ERR_INVAL;
			goto done;
		}
		USETW(req.wLength, min_len);

		err = usbd_do_request(udev, mtx, &req, desc);

		if (err) {
			if (!retries) {
				goto done;
			}
			retries--;

			usbd_pause_mtx(mtx, 200);

			continue;
		}
		buf = desc;

		if (min_len == max_len) {

			/* enforce correct type and length */

			if (buf[0] > min_len) {
				buf[0] = min_len;
			}
			buf[1] = type;

			goto done;
		}
		/* range check */

		if (max_len > buf[0]) {
			max_len = buf[0];
		}
		/* zero minimum data */

		while (min_len > max_len) {
			min_len--;
			buf[min_len] = 0;
		}

		/* set new minimum length */

		min_len = max_len;
	}
done:
	return (err);
}

/*------------------------------------------------------------------------*
 *	usbreq_get_string_any
 *
 * This function will return the string given by "string_index"
 * using the first language ID. The maximum length "len" includes
 * the terminating zero. The "len" argument should be twice as
 * big pluss 2 bytes, compared with the actual maximum string length !
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_string_any(struct usbd_device *udev, struct mtx *mtx, char *buf,
    uint16_t len, uint8_t string_index)
{
	char *s;
	uint8_t *temp;
	uint16_t i;
	uint16_t n;
	uint16_t c;
	uint8_t swap;
	usbd_status_t err;

	if (len == 0) {
		/* should not happen */
		return (USBD_ERR_NORMAL_COMPLETION);
	}
	buf[0] = 0;

	if (string_index == 0) {
		/* this is the language table */
		return (USBD_ERR_INVAL);
	}
	if (udev->flags.no_strings) {
		return (USBD_ERR_STALLED);
	}
	swap = (udev->quirks->uq_flags & UQ_SWAP_UNICODE) ? 1 : 0;

	err = usbreq_get_string_desc
	    (udev, mtx, buf, len, udev->langid, string_index);

	if (err) {
		return (err);
	}
	temp = buf;

	if (temp[0] < 2) {
		/* string length is too short */
		return (USBD_ERR_INVAL);
	}
	/* reserve one byte for terminating zero */
	len--;

	/* find maximum length */
	s = buf;
	n = (temp[0] / 2) - 1;
	if (n > len) {
		n = len;
	}
	/* skip descriptor header */
	temp += 2;

	/* convert and filter */
	for (i = 0; (i != n); i++) {
		c = UGETW(temp + (2 * i));

		/* convert from Unicode, handle buggy strings */
		if ((c & 0xff00) == 0) {
			*s = c;
		} else if (((c & 0x00ff) == 0) && swap) {
			*s = c >> 8;
		} else {
			*s = '?';
		}

		/* filter by default ! */
		if (!isprint(*s)) {
			*s = '?';
		}
		s++;
	}
	*s = 0;
	return (USBD_ERR_NORMAL_COMPLETION);
}

/*------------------------------------------------------------------------*
 *	usbreq_get_string_desc
 *
 * If you don't know the language ID, consider using
 * "usbreq_get_string_any()".
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_string_desc(struct usbd_device *udev, struct mtx *mtx, void *sdesc,
    uint16_t max_len, uint16_t lang_id,
    uint8_t string_index)
{
	return (usbreq_get_desc(udev, mtx, sdesc, 2, max_len, lang_id,
	    UDESC_STRING, string_index, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_config_desc
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_config_desc(struct usbd_device *udev, struct mtx *mtx, usb_config_descriptor_t *d,
    uint8_t conf_index)
{
	usbd_status_t err;

	PRINTFN(3, ("confidx=%d\n", conf_index));

	err = usbreq_get_desc(udev, mtx, d, USB_CONFIG_DESCRIPTOR_SIZE,
	    USB_CONFIG_DESCRIPTOR_SIZE, 0, UDESC_CONFIG, conf_index, 0);
	if (err) {
		goto done;
	}
	if (UGETW(d->wTotalLength) < USB_CONFIG_DESCRIPTOR_SIZE) {
		err = USBD_ERR_INVAL;
	}
done:
	return (err);
}

/*------------------------------------------------------------------------*
 *	usbreq_get_config_desc_full
 *
 * This function gets the complete USB configuration descriptor,
 * limited by the specified "size" which is usually equal to the
 * "wTotalLength" field stored in the USB configuration descriptor.
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_config_desc_full(struct usbd_device *udev, struct mtx *mtx, void *d,
    uint16_t size, uint8_t conf_index)
{
	PRINTFN(3, ("conf_index=%d\n", conf_index));
	return (usbreq_get_desc(udev, mtx, d, size, size, 0,
	    UDESC_CONFIG, conf_index, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_device_desc
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_device_desc(struct usbd_device *udev, struct mtx *mtx,
    usb_device_descriptor_t *d)
{
	PRINTFN(3, ("\n"));
	return (usbreq_get_desc(udev, mtx, d, USB_DEVICE_DESCRIPTOR_SIZE,
	    USB_DEVICE_DESCRIPTOR_SIZE, 0, UDESC_DEVICE, 0, 3));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_alt_interface_no
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_alt_interface_no(struct usbd_device *udev, struct mtx *mtx,
    uint8_t *alt_iface_no, uint8_t iface_index)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	usb_device_request_t req;

	if ((iface == NULL) || (iface->idesc == NULL)) {
		return (USBD_ERR_INVAL);
	}
	req.bmRequestType = UT_READ_INTERFACE;
	req.bRequest = UR_GET_INTERFACE;
	USETW(req.wValue, 0);
	req.wIndex[0] = iface->idesc->bInterfaceNumber;
	req.wIndex[1] = 0;
	USETW(req.wLength, 1);
	return (usbd_do_request(udev, mtx, &req, alt_iface_no));
}

/*------------------------------------------------------------------------*
 *	usbreq_set_alt_interface_no
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_set_alt_interface_no(struct usbd_device *udev, struct mtx *mtx,
    uint8_t iface_index, uint8_t alt_no)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	usb_device_request_t req;

	if ((iface == NULL) || (iface->idesc == NULL)) {
		return (USBD_ERR_INVAL);
	}
	req.bmRequestType = UT_WRITE_INTERFACE;
	req.bRequest = UR_SET_INTERFACE;
	req.wValue[0] = alt_no;
	req.wValue[1] = 0;
	req.wIndex[0] = iface->idesc->bInterfaceNumber;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);
	return (usbd_do_request(udev, mtx, &req, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_device_status
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_device_status(struct usbd_device *udev, struct mtx *mtx,
    usb_status_t *st)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_DEVICE;
	req.bRequest = UR_GET_STATUS;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, sizeof(usb_status_t));
	return (usbd_do_request(udev, mtx, &req, st));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_hub_descriptor
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_hub_descriptor(struct usbd_device *udev, struct mtx *mtx,
    usb_hub_descriptor_t *hd)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_CLASS_DEVICE;
	req.bRequest = UR_GET_DESCRIPTOR;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, USB_HUB_DESCRIPTOR_SIZE);
	return (usbd_do_request(udev, mtx, &req, hd));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_hub_status
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_hub_status(struct usbd_device *udev, struct mtx *mtx,
    usb_hub_status_t *st)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_CLASS_DEVICE;
	req.bRequest = UR_GET_STATUS;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, sizeof(usb_hub_status_t));
	return (usbd_do_request(udev, mtx, &req, st));
}

/*------------------------------------------------------------------------*
 *	usbreq_set_address
 *
 * This function is used to set the address for an USB device. After
 * port reset the USB device will respond at address zero.
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_set_address(struct usbd_device *udev, struct mtx *mtx, uint16_t addr)
{
	usb_device_request_t req;

	PRINTFN(5, ("setting device address=%d\n", addr));

	req.bmRequestType = UT_WRITE_DEVICE;
	req.bRequest = UR_SET_ADDRESS;
	USETW(req.wValue, addr);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	/* Setting the address should not take more than 1 second ! */
	return (usbd_do_request_flags(udev, mtx, &req, NULL,
	    USBD_DELAY_STATUS_STAGE, NULL, 1000));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_port_status
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_port_status(struct usbd_device *udev, struct mtx *mtx,
    usb_port_status_t *ps, uint8_t port)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_CLASS_OTHER;
	req.bRequest = UR_GET_STATUS;
	USETW(req.wValue, 0);
	req.wIndex[0] = port;
	req.wIndex[1] = 0;
	USETW(req.wLength, sizeof *ps);
	return (usbd_do_request(udev, mtx, &req, ps));
}

/*------------------------------------------------------------------------*
 *	usbreq_clear_hub_feature
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_clear_hub_feature(struct usbd_device *udev, struct mtx *mtx,
    uint16_t sel)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_CLASS_DEVICE;
	req.bRequest = UR_CLEAR_FEATURE;
	USETW(req.wValue, sel);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	return (usbd_do_request(udev, mtx, &req, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_set_hub_feature
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_set_hub_feature(struct usbd_device *udev, struct mtx *mtx,
    uint16_t sel)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_CLASS_DEVICE;
	req.bRequest = UR_SET_FEATURE;
	USETW(req.wValue, sel);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	return (usbd_do_request(udev, mtx, &req, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_clear_port_feature
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_clear_port_feature(struct usbd_device *udev, struct mtx *mtx,
    uint8_t port, uint16_t sel)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_CLASS_OTHER;
	req.bRequest = UR_CLEAR_FEATURE;
	USETW(req.wValue, sel);
	req.wIndex[0] = port;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);
	return (usbd_do_request(udev, mtx, &req, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_set_port_feature
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_set_port_feature(struct usbd_device *udev, struct mtx *mtx,
    uint8_t port, uint16_t sel)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_CLASS_OTHER;
	req.bRequest = UR_SET_FEATURE;
	USETW(req.wValue, sel);
	req.wIndex[0] = port;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);
	return (usbd_do_request(udev, mtx, &req, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_set_protocol
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_set_protocol(struct usbd_device *udev, struct mtx *mtx,
    uint8_t iface_index, uint16_t report)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	usb_device_request_t req;

	if ((iface == NULL) || (iface->idesc == NULL)) {
		return (USBD_ERR_INVAL);
	}
	PRINTFN(4, ("iface=%p, report=%d, endpt=%d\n",
	    iface, report, iface->idesc->bInterfaceNumber));

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UR_SET_PROTOCOL;
	USETW(req.wValue, report);
	req.wIndex[0] = iface->idesc->bInterfaceNumber;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);
	return (usbd_do_request(udev, mtx, &req, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_set_report
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_set_report(struct usbd_device *udev, struct mtx *mtx, void *data, uint16_t len,
    uint8_t iface_index, uint8_t type, uint8_t id)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	usb_device_request_t req;

	if ((iface == NULL) || (iface->idesc == NULL)) {
		return (USBD_ERR_INVAL);
	}
	PRINTFN(4, ("len=%d\n", len));

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UR_SET_REPORT;
	USETW2(req.wValue, type, id);
	req.wIndex[0] = iface->idesc->bInterfaceNumber;
	req.wIndex[1] = 0;
	USETW(req.wLength, len);
	return (usbd_do_request(udev, mtx, &req, data));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_report
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_report(struct usbd_device *udev, struct mtx *mtx, void *data,
    uint16_t len, uint8_t iface_index, uint8_t type, uint8_t id)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	usb_device_request_t req;

	if ((iface == NULL) || (iface->idesc == NULL) || (id == 0)) {
		return (USBD_ERR_INVAL);
	}
	PRINTFN(4, ("len=%d\n", len));

	req.bmRequestType = UT_READ_CLASS_INTERFACE;
	req.bRequest = UR_GET_REPORT;
	USETW2(req.wValue, type, id);
	req.wIndex[0] = iface->idesc->bInterfaceNumber;
	req.wIndex[1] = 0;
	USETW(req.wLength, len);
	return (usbd_do_request(udev, mtx, &req, data));
}

/*------------------------------------------------------------------------*
 *	usbreq_set_idle
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_set_idle(struct usbd_device *udev, struct mtx *mtx,
    uint8_t iface_index, uint8_t duration, uint8_t id)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	usb_device_request_t req;

	if ((iface == NULL) || (iface->idesc == NULL)) {
		return (USBD_ERR_INVAL);
	}
	PRINTFN(4, ("%d %d\n", duration, id));

	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UR_SET_IDLE;
	USETW2(req.wValue, duration, id);
	req.wIndex[0] = iface->idesc->bInterfaceNumber;
	req.wIndex[1] = 0;
	USETW(req.wLength, 0);
	return (usbd_do_request(udev, mtx, &req, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_report_descriptor
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_report_descriptor(struct usbd_device *udev, struct mtx *mtx,
    void *d, uint16_t size, uint8_t iface_index)
{
	struct usbd_interface *iface = usbd_get_iface(udev, iface_index);
	usb_device_request_t req;

	if ((iface == NULL) || (iface->idesc == NULL)) {
		return (USBD_ERR_INVAL);
	}
	req.bmRequestType = UT_READ_INTERFACE;
	req.bRequest = UR_GET_DESCRIPTOR;
	USETW2(req.wValue, UDESC_REPORT, 0);	/* report id should be 0 */
	req.wIndex[0] = iface->idesc->bInterfaceNumber;
	req.wIndex[1] = 0;
	USETW(req.wLength, size);
	return (usbd_do_request(udev, mtx, &req, d));
}

/*------------------------------------------------------------------------*
 *	usbreq_set_config
 *
 * This function is used to select the current configuration number in
 * both USB device side mode and USB host side mode. When setting the
 * configuration the function of the interfaces can change.
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_set_config(struct usbd_device *udev, struct mtx *mtx, uint8_t conf)
{
	usb_device_request_t req;

	PRINTF(("setting config %d\n", conf));

	/* do "set configuration" request */

	req.bmRequestType = UT_WRITE_DEVICE;
	req.bRequest = UR_SET_CONFIG;
	req.wValue[0] = conf;
	req.wValue[1] = 0;
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);
	return (usbd_do_request(udev, mtx, &req, 0));
}

/*------------------------------------------------------------------------*
 *	usbreq_get_config
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
usbd_status_t
usbreq_get_config(struct usbd_device *udev, struct mtx *mtx, uint8_t *pconf)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_DEVICE;
	req.bRequest = UR_GET_CONFIG;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 1);
	return (usbd_do_request(udev, mtx, &req, pconf));
}
