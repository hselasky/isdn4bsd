/*	$NetBSD: if_cdce.c,v 1.4 2004/10/24 12:50:54 augustss Exp $ */

/*
 * Copyright (c) 1997, 1998, 1999, 2000-2003 Bill Paul <wpaul@windriver.com>
 * Copyright (c) 2003-2005 Craig Boston
 * Copyright (c) 2004 Daniel Hartmeier
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
 *	This product includes software developed by Bill Paul.
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
 */

/*
 * USB Communication Device Class (Ethernet Networking Control Model)
 * http://www.usb.org/developers/devclass_docs/usbcdc11.pdf
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/if_cdce.c,v 1.25 2007/06/30 20:18:44 imp Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/endian.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_types.h>
#include <net/if_media.h>

#include <net/bpf.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_cdc.h>

#include "usbdevs.h"

#include <dev/usb/if_cdcereg.h>

static device_probe_t cdce_probe;
static device_attach_t cdce_attach;
static device_detach_t cdce_detach;
static device_shutdown_t cdce_shutdown;
static device_suspend_t cdce_suspend;
static device_resume_t cdce_resume;
static usb_handle_request_t cdce_handle_request;

static usbd_callback_t cdce_bulk_write_clear_stall_callback;
static usbd_callback_t cdce_bulk_write_callback;
static usbd_callback_t cdce_bulk_read_clear_stall_callback;
static usbd_callback_t cdce_bulk_read_callback;
static usbd_callback_t cdce_intr_read_clear_stall_callback;
static usbd_callback_t cdce_intr_read_callback;
static usbd_callback_t cdce_intr_write_callback;

static void cdce_start_cb(struct ifnet *ifp);
static void cdce_start_transfers(struct cdce_softc *sc);
static uint32_t cdce_m_crc32(struct mbuf *m, uint32_t src_offset, uint32_t src_len);
static void cdce_stop(struct cdce_softc *sc);
static int cdce_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data);
static void cdce_init_cb(void *arg);
static int cdce_ifmedia_upd_cb(struct ifnet *ifp);
static void cdce_ifmedia_sts_cb(struct ifnet *const ifp, struct ifmediareq *req);

#ifdef USB_DEBUG
#define	DPRINTF(sc,n,fmt,...)	\
  do { if (cdce_debug > (n)) {	     \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int cdce_debug = 0;
static int cdce_force_512x4 = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, cdce, CTLFLAG_RW, 0, "USB cdce");
SYSCTL_INT(_hw_usb_cdce, OID_AUTO, debug, CTLFLAG_RW, &cdce_debug, 0,
    "cdce debug level");
SYSCTL_INT(_hw_usb_cdce, OID_AUTO, force_512x4, CTLFLAG_RW,
    &cdce_force_512x4, 0, "cdce force 512x4 protocol");
#else
#define	DPRINTF(...)
#endif

static const struct usbd_config cdce_config[CDCE_N_TRANSFER] = {

	[0] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.if_index = 0,
		/* Host Mode */
		.mh.frames = CDCE_512X4_FRAGS_MAX + 1,
		.mh.bufsize = (CDCE_512X4_FRAMES_MAX * MCLBYTES) + sizeof(usb_cdc_mf_eth_512x4_header_t),
		.mh.flags = {.pipe_bof = 1,.force_short_xfer = 1,.ext_buffer = 1,},
		.mh.callback = &cdce_bulk_write_callback,
		.mh.timeout = 10000,	/* 10 seconds */
		/* Device Mode */
		.md.frames = CDCE_512X4_FRAGS_MAX + 1,
		.md.bufsize = (CDCE_512X4_FRAMES_MAX * MCLBYTES) + sizeof(usb_cdc_mf_eth_512x4_header_t),
		.md.flags = {.pipe_bof = 1,.short_xfer_ok = 1,.ext_buffer = 1,},
		.md.callback = &cdce_bulk_read_callback,
		.md.timeout = 0,	/* no timeout */
	},

	[1] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.if_index = 0,
		/* Host Mode */
		.mh.frames = CDCE_512X4_FRAGS_MAX + 1,
		.mh.bufsize = (CDCE_512X4_FRAMES_MAX * MCLBYTES) + sizeof(usb_cdc_mf_eth_512x4_header_t),
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,.ext_buffer = 1,},
		.mh.callback = &cdce_bulk_read_callback,
		.mh.timeout = 0,	/* no timeout */
		/* Device Mode */
		.md.frames = CDCE_512X4_FRAGS_MAX + 1,
		.md.bufsize = (CDCE_512X4_FRAMES_MAX * MCLBYTES) + sizeof(usb_cdc_mf_eth_512x4_header_t),
		.md.flags = {.pipe_bof = 1,.force_short_xfer = 1,.ext_buffer = 1,},
		.md.callback = &cdce_bulk_write_callback,
		.md.timeout = 10000,	/* 10 seconds */
	},

	[2] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.if_index = 0,
		/* Host Mode Only */
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.interval = 50,	/* 50ms */
		.mh.flags = {},
		.mh.callback = &cdce_bulk_write_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
	},

	[3] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.if_index = 0,
		/* Host Mode Only */
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.interval = 50,	/* 50ms */
		.mh.flags = {},
		.mh.callback = &cdce_bulk_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
	},

	[4] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.if_index = 1,
		/* Host Mode */
		.mh.bufsize = CDCE_IND_SIZE_MAX,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,.no_pipe_ok = 1,},
		.mh.callback = &cdce_intr_read_callback,
		.mh.timeout = 0,
		/* Device Mode */
		.md.bufsize = CDCE_IND_SIZE_MAX,
		.md.flags = {.pipe_bof = 1,.force_short_xfer = 1,.no_pipe_ok = 1,},
		.md.callback = &cdce_intr_write_callback,
		.md.timeout = 10000,	/* 10 seconds */
	},

	[5] = {
		.type = UE_CONTROL,
		.endpoint = 0x00,	/* Control pipe */
		.direction = UE_DIR_ANY,
		.if_index = 1,
		/* Host Mode Only */
		.mh.bufsize = sizeof(usb_device_request_t),
		.mh.interval = 50,	/* 50ms */
		.mh.flags = {},
		.mh.callback = &cdce_intr_read_clear_stall_callback,
		.mh.timeout = 1000,	/* 1 second */
	},
};

static device_method_t cdce_methods[] = {
	/* USB interface */
	DEVMETHOD(usb_handle_request, cdce_handle_request),

	/* Device interface */
	DEVMETHOD(device_probe, cdce_probe),
	DEVMETHOD(device_attach, cdce_attach),
	DEVMETHOD(device_detach, cdce_detach),
	DEVMETHOD(device_suspend, cdce_suspend),
	DEVMETHOD(device_resume, cdce_resume),
	DEVMETHOD(device_shutdown, cdce_shutdown),

	{0, 0}
};

static driver_t cdce_driver = {
	.name = "cdce",
	.methods = cdce_methods,
	.size = sizeof(struct cdce_softc),
};

static devclass_t cdce_devclass;

DRIVER_MODULE(cdce, uhub, cdce_driver, cdce_devclass, usbd_driver_load, 0);
MODULE_VERSION(cdce, 0);
MODULE_DEPEND(cdce, usb, 1, 1, 1);
MODULE_DEPEND(cdce, ether, 1, 1, 1);

static const struct cdce_type cdce_devs[] = {
	{{USB_VENDOR_ACERLABS, USB_PRODUCT_ACERLABS_M5632}, CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_AMBIT, USB_PRODUCT_AMBIT_NTL_250}, CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_COMPAQ, USB_PRODUCT_COMPAQ_IPAQLINUX}, CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_GMATE, USB_PRODUCT_GMATE_YP3X00}, CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_MOTOROLA2, USB_PRODUCT_MOTOROLA2_USBLAN}, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_MOTOROLA2, USB_PRODUCT_MOTOROLA2_USBLAN2}, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_NETCHIP, USB_PRODUCT_NETCHIP_ETHERNETGADGET}, CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_PROLIFIC, USB_PRODUCT_PROLIFIC_PL2501}, CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SL5500}, CDCE_FLAG_ZAURUS},
	{{USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SL5600}, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SLA300}, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SLC700}, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION},
	{{USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SLC750}, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION},
};

#define	cdce_lookup(v, p) ((const struct cdce_type *)usb_lookup(cdce_devs, v, p))

static int
cdce_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_interface_descriptor_t *id;

	if (uaa->iface == NULL) {
		return (UMATCH_NONE);
	}
	id = usbd_get_interface_descriptor(uaa->iface);
	if (id == NULL) {
		return (UMATCH_NONE);
	}
	if (cdce_lookup(uaa->vendor, uaa->product) != NULL) {
		return (UMATCH_VENDOR_PRODUCT);
	}
	if ((id->bInterfaceClass == UICLASS_CDC) &&
	    (id->bInterfaceSubClass ==
	    UISUBCLASS_ETHERNET_NETWORKING_CONTROL_MODEL)) {
		return (UMATCH_IFACECLASS_GENERIC);
	}
	return (UMATCH_NONE);
}

static int
cdce_attach(device_t dev)
{
	struct cdce_softc *sc = device_get_softc(dev);
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct usbd_interface *iface;
	const usb_cdc_union_descriptor_t *ud;
	const usb_cdc_ethernet_descriptor_t *ue;
	const usb_interface_descriptor_t *id;
	const struct cdce_type *t;
	struct ifnet *ifp;
	int error;
	uint8_t alt_index;
	uint8_t i;
	uint8_t eaddr[ETHER_ADDR_LEN];
	uint8_t eaddr_str[USB_STRING_DESC_LEN(ETHER_ADDR_LEN * 2) + 1];

	if (sc == NULL) {
		return (ENOMEM);
	}
	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;
	sc->sc_unit = device_get_unit(dev);

	t = cdce_lookup(uaa->vendor, uaa->product);
	if (t) {
		sc->sc_flags = t->cdce_flags;
	}
	/* search for alternate settings */
	if (uaa->usb_mode == USB_MODE_HOST) {

		usb_descriptor_t *desc;
		usb_config_descriptor_t *cd;

		cd = usbd_get_config_descriptor(uaa->device);
		desc = (void *)(uaa->iface->idesc);
		id = (void *)desc;
		i = id->bInterfaceNumber;
		alt_index = 0;
		while ((desc = usbd_desc_foreach(cd, desc))) {
			id = (void *)desc;
			if ((id->bDescriptorType == UDESC_INTERFACE) &&
			    (id->bLength >= sizeof(*id))) {
				if (id->bInterfaceNumber != i) {
					alt_index = 0;
					break;
				}
				if ((id->bInterfaceClass == UICLASS_CDC) &&
				    (id->bInterfaceSubClass ==
				    UISUBCLASS_ETHERNET_NETWORKING_CONTROL_MODEL) &&
				    (id->bInterfaceProtocol == UIPROTO_CDC_ETH_512X4)) {

					alt_index = id->bAlternateSetting;
					/*
					 * We want this alt setting hence
					 * the protocol supports multi
					 * sub-framing !
					 */
					break;
				}
			}
		}

		if (alt_index > 0) {

			error = usbd_set_alt_interface_index(uaa->device,
			    uaa->iface_index, alt_index);
			if (error) {
				device_printf(dev, "Could not set alternate "
				    "setting, error = %s\n", usbd_errstr(error));
				return (EINVAL);
			}
		}
	}
	/* get the interface subclass we are using */
	sc->sc_iface_protocol = uaa->iface->idesc->bInterfaceProtocol;
#ifdef USB_DEBUG
	if (cdce_force_512x4) {
		sc->sc_iface_protocol = UIPROTO_CDC_ETH_512X4;
	}
#endif
	usbd_set_device_desc(dev);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s",
	    device_get_nameunit(dev));

	mtx_init(&(sc->sc_mtx), "cdce lock", NULL, MTX_DEF | MTX_RECURSE);

	if (sc->sc_flags & CDCE_FLAG_NO_UNION) {
		sc->sc_ifaces_index[0] = uaa->iface_index;
		sc->sc_ifaces_index[1] = uaa->iface_index;
		sc->sc_data_iface_no = 0;	/* not used */
		goto alloc_transfers;
	}
	ud = usbd_find_descriptor
	    (uaa->device, NULL, uaa->iface_index,
	    UDESC_CS_INTERFACE, 0 - 1, UDESCSUB_CDC_UNION, 0 - 1);

	if ((ud == NULL) || (ud->bLength < sizeof(*ud))) {
		device_printf(dev, "no union descriptor!\n");
		goto detach;
	}
	sc->sc_data_iface_no = ud->bSlaveInterface[0];

	for (i = 0;; i++) {

		iface = usbd_get_iface(uaa->device, i);

		if (iface) {

			id = usbd_get_interface_descriptor(iface);

			if (id && (id->bInterfaceNumber ==
			    sc->sc_data_iface_no)) {
				sc->sc_ifaces_index[0] = i;
				sc->sc_ifaces_index[1] = uaa->iface_index;
				USBD_SET_IFACE_NO_PROBE(uaa->device, i);
				break;
			}
		} else {
			device_printf(dev, "no data interface found!\n");
			goto detach;
		}
	}

	/*
	 * <quote>
	 *
	 *  The Data Class interface of a networking device shall have
	 *  a minimum of two interface settings. The first setting
	 *  (the default interface setting) includes no endpoints and
	 *  therefore no networking traffic is exchanged whenever the
	 *  default interface setting is selected. One or more
	 *  additional interface settings are used for normal
	 *  operation, and therefore each includes a pair of endpoints
	 *  (one IN, and one OUT) to exchange network traffic. Select
	 *  an alternate interface setting to initialize the network
	 *  aspects of the device and to enable the exchange of
	 *  network traffic.
	 *
	 * </quote>
	 *
	 * Some devices, most notably cable modems, include interface
	 * settings that have no IN or OUT endpoint, therefore loop
	 * through the list of all available interface settings
	 * looking for one with both IN and OUT endpoints.
	 */

alloc_transfers:

	for (i = 0; i < 32; i++) {

		error = usbd_set_alt_interface_index
		    (uaa->device, sc->sc_ifaces_index[0], i);

		if (error) {
			device_printf(dev, "no valid alternate "
			    "setting found!\n");
			goto detach;
		}
		error = usbd_transfer_setup
		    (uaa->device, sc->sc_ifaces_index,
		    sc->sc_xfer, cdce_config, CDCE_N_TRANSFER,
		    sc, &(sc->sc_mtx));

		if (error == 0) {
			break;
		}
	}

	ifmedia_init(&(sc->sc_ifmedia), 0,
	    &cdce_ifmedia_upd_cb,
	    &cdce_ifmedia_sts_cb);

	ue = usbd_find_descriptor
	    (uaa->device, NULL, uaa->iface_index,
	    UDESC_CS_INTERFACE, 0 - 1, UDESCSUB_CDC_ENF, 0 - 1);

	if ((ue == NULL) || (ue->bLength < sizeof(*ue))) {
		error = USBD_ERR_INVAL;
	} else {
		error = usbreq_get_string_any
		    (uaa->device, &Giant, eaddr_str,
		    sizeof(eaddr_str), ue->iMacAddress);
	}

	if (error) {

		/* fake MAC address */

		device_printf(dev, "faking MAC address\n");
		eaddr[0] = 0x2a;
		memcpy(&eaddr[1], &ticks, sizeof(uint32_t));
		eaddr[5] = sc->sc_unit;

	} else {

		bzero(eaddr, sizeof(eaddr));

		for (i = 0; i < (ETHER_ADDR_LEN * 2); i++) {

			uint8_t c = eaddr_str[i];

			if ((c >= '0') && (c <= '9')) {
				c -= '0';
			} else {
				c -= 'A' - 10;
			}

			c &= 0xf;

			if ((i & 1) == 0) {
				c <<= 4;
			}
			eaddr[i / 2] |= c;
		}

		if (uaa->usb_mode == USB_MODE_DEVICE) {
			/*
			 * Do not use the same MAC address like the peer !
			 */
			eaddr[5] ^= 0xFF;
		}
	}

	ifp = if_alloc(IFT_ETHER);

	if (ifp == NULL) {
		device_printf(dev, "cannot if_alloc()\n");
		goto detach;
	}
	sc->sc_evilhack = ifp;

	ifp->if_softc = sc;
	if_initname(ifp, "cdce", sc->sc_unit);
	ifp->if_mtu = ETHERMTU;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = cdce_ioctl_cb;
	ifp->if_output = ether_output;
	ifp->if_start = cdce_start_cb;
	ifp->if_init = cdce_init_cb;
	ifp->if_baudrate = 11000000;
	if (sc->sc_iface_protocol == UIPROTO_CDC_ETH_512X4) {
		IFQ_SET_MAXLEN(&ifp->if_snd, CDCE_512X4_IFQ_MAXLEN);
		ifp->if_snd.ifq_drv_maxlen = CDCE_512X4_IFQ_MAXLEN;
	} else {
		IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
		ifp->if_snd.ifq_drv_maxlen = IFQ_MAXLEN;
	}
	IFQ_SET_READY(&ifp->if_snd);

	/* no IFM type for 11Mbps USB, so go with 10baseT */
	ifmedia_add(&sc->sc_ifmedia, IFM_ETHER | IFM_10_T, 0, 0);
	ifmedia_set(&sc->sc_ifmedia, IFM_ETHER | IFM_10_T);

	sc->sc_ifp = ifp;

	ether_ifattach(ifp, eaddr);

	/* start the interrupt transfer, if any */
	mtx_lock(&(sc->sc_mtx));
#ifdef CDCE_DO_BENCHMARK
	usbd_transfer_start(sc->sc_xfer[0]);
	usbd_transfer_start(sc->sc_xfer[1]);
	device_printf(dev, "benchmarking enabled\n");
#endif
	usbd_transfer_start(sc->sc_xfer[4]);
	mtx_unlock(&(sc->sc_mtx));

	return (0);			/* success */

detach:
	cdce_detach(dev);
	return (ENXIO);			/* failure */
}

static int
cdce_detach(device_t dev)
{
	struct cdce_softc *sc = device_get_softc(dev);
	struct ifnet *ifp;

	mtx_lock(&(sc->sc_mtx));

	cdce_stop(sc);

	ifp = sc->sc_ifp;

	mtx_unlock(&(sc->sc_mtx));

	/* stop all USB transfers first */
	usbd_transfer_unsetup(sc->sc_xfer, CDCE_N_TRANSFER);

	/* get rid of any late children */
	bus_generic_detach(dev);

	if (ifp) {
		ether_ifdetach(ifp);
		if_free(ifp);
		ifmedia_removeall(&(sc->sc_ifmedia));
	}
	mtx_destroy(&(sc->sc_mtx));

	return (0);
}

static void
cdce_start_cb(struct ifnet *ifp)
{
	struct cdce_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	cdce_start_transfers(sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
cdce_start_transfers(struct cdce_softc *sc)
{
	if ((sc->sc_flags & CDCE_FLAG_LL_READY) &&
	    (sc->sc_flags & CDCE_FLAG_HL_READY)) {

		/*
		 * start the USB transfers, if not already started:
		 */
		usbd_transfer_start(sc->sc_xfer[1]);
		usbd_transfer_start(sc->sc_xfer[0]);
	}
	return;
}

static uint32_t
cdce_m_frags(struct mbuf *m)
{
	uint32_t temp = 1;

	while ((m = m->m_next)) {
		temp++;
	}
	return (temp);
}

static void
cdce_fwd_mq(struct cdce_softc *sc, struct cdce_mq *mq)
{
	struct mbuf *m;
	struct ifnet *ifp = sc->sc_ifp;

	if (mq->ifq_head) {

		mtx_unlock(&(sc->sc_mtx));

		while (1) {

			_IF_DEQUEUE(mq, m);

			if (m == NULL)
				break;

			(ifp->if_input) (ifp, m);
		}

		mtx_lock(&(sc->sc_mtx));
	}
	return;
}

static void
cdce_free_mq(struct cdce_mq *mq)
{
	struct mbuf *m;

	if (mq->ifq_head) {

		while (1) {

			_IF_DEQUEUE(mq, m);

			if (m == NULL)
				break;

			m_freem(m);
		}
	}
	return;
}

static void
cdce_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
cdce_bulk_write_512x4_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;
	struct mbuf *mt;
	uint16_t x;
	uint16_t y;
	uint16_t flen;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		DPRINTF(sc, 10, "transfer complete: "
		    "%u bytes in %u fragments and %u frames\n",
		    xfer->actlen, xfer->nframes, sc->sc_tx_mq.ifq_len);

		/* update packet counter */
		ifp->if_opackets += sc->sc_tx_mq.ifq_len;

		/* free all previous mbufs */
		cdce_free_mq(&(sc->sc_tx_mq));

	case USBD_ST_SETUP:
tr_setup:
		if (xfer->flags.stall_pipe &&
		    (xfer->flags_int.usb_mode == USB_MODE_HOST)) {
			/* try to clear stall */
			usbd_transfer_start(sc->sc_xfer[2]);
			break;
		}
		x = 0;			/* number of frames */
		y = 1;			/* number of fragments */

		while (x != CDCE_512X4_FRAMES_MAX) {

			IFQ_DRV_DEQUEUE(&(ifp->if_snd), m);

			if (m == NULL) {
				break;
			}
			if (m->m_pkthdr.len > MCLBYTES) {
				m_freem(m);
				ifp->if_oerrors++;
				continue;
			}
			if (cdce_m_frags(m) > CDCE_512X4_FRAME_FRAG_MAX) {
				mt = m_defrag(m, M_DONTWAIT);
				if (mt == NULL) {
					m_freem(m);
					ifp->if_oerrors++;
					continue;
				}
				m = mt;
			}
			_IF_ENQUEUE(&(sc->sc_tx_mq), m);

			/*
			 * if there's a BPF listener, bounce a copy
			 * of this frame to him:
			 */
			BPF_MTAP(ifp, m);

#if (CDCE_512X4_FRAG_LENGTH_MASK < MCLBYTES)
#error "(CDCE_512X4_FRAG_LENGTH_MASK < MCLBYTES)"
#endif
			do {

				flen = m->m_len & CDCE_512X4_FRAG_LENGTH_MASK;
				xfer->frlengths[y] = m->m_len;
				usbd_set_frame_data(xfer, m->m_data, y);

				if (m->m_next == NULL) {
					flen |= CDCE_512X4_FRAG_LAST_MASK;
				}
				USETW(sc->sc_tx.hdr.wFragLength[y - 1], flen);

				y++;

			} while ((m = m->m_next));

			x++;
		}

		if (y == 1) {
			/* no data to transmit */
			break;
		}
		/* fill in Signature */
		sc->sc_tx.hdr.bSig[0] = 'F';
		sc->sc_tx.hdr.bSig[1] = 'L';

		/*
		 * We ensure that the header results in a short packet by
		 * making the length odd !
		 */
		USETW(sc->sc_tx.hdr.wFragLength[y - 1], 0);
		xfer->frlengths[0] = CDCE_512X4_FRAG_LENGTH_OFFSET + ((y - 1) * 2) + 1;
		usbd_set_frame_data(xfer, &(sc->sc_tx.hdr), 0);
		xfer->nframes = y;
		usbd_start_hardware(xfer);
		break;

	default:			/* Error */
		DPRINTF(sc, 10, "transfer error, %s\n",
		    usbd_errstr(xfer->error));

		/* update error counter */
		ifp->if_oerrors += sc->sc_tx_mq.ifq_len;

		/* free all previous mbufs */
		cdce_free_mq(&(sc->sc_tx_mq));

		if (xfer->error != USBD_ERR_CANCELLED) {
			/* try to clear stall first */
			xfer->flags.stall_pipe = 1;
			goto tr_setup;
		}
		break;
	}
	return;
}

static void
cdce_bulk_write_std_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;
	struct mbuf *mt;
	uint32_t crc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		DPRINTF(sc, 10, "transfer complete: "
		    "%u bytes in %u frames\n", xfer->actlen,
		    xfer->aframes);

		ifp->if_opackets++;

		/* free all previous mbufs */
		cdce_free_mq(&(sc->sc_tx_mq));

	case USBD_ST_SETUP:
tr_setup:
		if (xfer->flags.stall_pipe &&
		    (xfer->flags_int.usb_mode == USB_MODE_HOST)) {
			usbd_transfer_start(sc->sc_xfer[2]);
			break;
		}
		IFQ_DRV_DEQUEUE(&(ifp->if_snd), m);

		if (m == NULL) {
			break;
		}
		if (sc->sc_flags & CDCE_FLAG_ZAURUS) {
			/*
			 * Zaurus wants a 32-bit CRC appended to
			 * every frame
			 */

			crc = cdce_m_crc32(m, 0, m->m_pkthdr.len);
			crc = htole32(crc);

			if (!m_append(m, 4, (void *)&crc)) {
				m_freem(m);
				ifp->if_oerrors++;
				goto tr_setup;
			}
		}
		if (m->m_len != m->m_pkthdr.len) {
			mt = m_defrag(m, M_DONTWAIT);
			if (mt == NULL) {
				m_freem(m);
				ifp->if_oerrors++;
				goto tr_setup;
			}
			m = mt;
		}
		if (m->m_pkthdr.len > MCLBYTES) {
			m->m_pkthdr.len = MCLBYTES;
		}
		_IF_ENQUEUE(&(sc->sc_tx_mq), m);

		xfer->frlengths[0] = m->m_len;
		usbd_set_frame_data(xfer, m->m_data, 0);
		xfer->nframes = 1;

		/*
		 * if there's a BPF listener, bounce a copy
		 * of this frame to him:
		 */
		BPF_MTAP(ifp, m);

		usbd_start_hardware(xfer);
		break;

	default:			/* Error */
		DPRINTF(sc, 10, "transfer error, %s\n",
		    usbd_errstr(xfer->error));

		/* free all previous mbufs */
		cdce_free_mq(&(sc->sc_tx_mq));
		ifp->if_oerrors++;

		if (xfer->error != USBD_ERR_CANCELLED) {
			/* try to clear stall first */
			xfer->flags.stall_pipe = 1;
			goto tr_setup;
		}
		break;
	}
	return;
}

static void
cdce_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;

	/* first call - set the correct callback */
	if (sc->sc_iface_protocol == UIPROTO_CDC_ETH_512X4) {
		xfer->flags.force_short_xfer = 0;
		xfer->callback = &cdce_bulk_write_512x4_callback;
	} else {
		xfer->callback = &cdce_bulk_write_std_callback;
	}
	(xfer->callback) (xfer);
	return;
}

static int32_t
#ifdef __FreeBSD__
cdce_m_crc32_cb(void *arg, void *src, uint32_t count)
#else
cdce_m_crc32_cb(void *arg, caddr_t src, uint32_t count)
#endif
{
	register uint32_t *p_crc = arg;

	*p_crc = crc32_raw(src, count, *p_crc);
	return (0);
}

static uint32_t
cdce_m_crc32(struct mbuf *m, uint32_t src_offset, uint32_t src_len)
{
	register int error;
	uint32_t crc = 0xFFFFFFFF;

	error = m_apply(m, src_offset, src_len, &cdce_m_crc32_cb, &crc);
	return (crc ^ 0xFFFFFFFF);
}

static void
cdce_stop(struct cdce_softc *sc)
{
	struct ifnet *ifp = sc->sc_ifp;

	/* immediate configuration */

	if (ifp) {
		/* clear flags */
		ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	}
	sc->sc_flags &= ~(CDCE_FLAG_HL_READY |
	    CDCE_FLAG_LL_READY);

	/*
	 * stop all the transfers, if not already stopped:
	 */
	usbd_transfer_stop(sc->sc_xfer[0]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[3]);
	return;
}

static int
cdce_shutdown(device_t dev)
{
	struct cdce_softc *sc = device_get_softc(dev);

	mtx_lock(&(sc->sc_mtx));

	cdce_stop(sc);

	mtx_unlock(&(sc->sc_mtx));

	return (0);
}

static int
cdce_suspend(device_t dev)
{
	device_printf(dev, "Suspending\n");
	return 0;
}

static int
cdce_resume(device_t dev)
{
	device_printf(dev, "Resuming\n");
	return 0;
}

static int
cdce_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct cdce_softc *sc = ifp->if_softc;
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	switch (command) {
	case SIOCSIFFLAGS:
		if (ifp->if_flags & IFF_UP) {
			if (!(ifp->if_drv_flags & IFF_DRV_RUNNING)) {
				cdce_init_cb(sc);
			}
		} else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				cdce_stop(sc);
			}
		}
		break;

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, (void *)data,
		    &sc->sc_ifmedia, command);
		break;

	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}

	mtx_unlock(&(sc->sc_mtx));

	return (error);
}

static void
cdce_init_cb(void *arg)
{
	struct cdce_softc *sc = arg;
	struct ifnet *ifp;

	mtx_lock(&(sc->sc_mtx));

	ifp = sc->sc_ifp;

	/* immediate configuration */

	cdce_stop(sc);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

	sc->sc_flags |= (
	    CDCE_FLAG_LL_READY |
	    CDCE_FLAG_HL_READY);

	usbd_transfer_set_stall(sc->sc_xfer[0]);
	usbd_transfer_set_stall(sc->sc_xfer[1]);

	cdce_start_transfers(sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
cdce_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
cdce_bulk_read_512x4_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;
	void *data_ptr;
	uint32_t offset;
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint16_t rx_frags;
	uint16_t flen;
	uint8_t fwd_mq;
	uint8_t free_mq;

	fwd_mq = 0;
	free_mq = 0;
	rx_frags = 0;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		DPRINTF(sc, 0, "received %u bytes in %u frames\n",
		    xfer->actlen, xfer->aframes);

		/* check state */
		if (!(sc->sc_flags & CDCE_FLAG_RX_DATA)) {

			/* verify the header */
			if ((xfer->actlen < CDCE_512X4_FRAG_LENGTH_OFFSET) ||
			    (sc->sc_rx.hdr.bSig[0] != 'F') ||
			    (sc->sc_rx.hdr.bSig[1] != 'L')) {
				/* try to clear stall first */
				xfer->flags.stall_pipe = 1;
				goto tr_setup;
			}
			rx_frags = (xfer->actlen -
			    CDCE_512X4_FRAG_LENGTH_OFFSET) / 2;
			if (rx_frags != 0) {
				/* start receiving data */
				sc->sc_flags |= CDCE_FLAG_RX_DATA;
			}
			DPRINTF(sc, 0, "doing %u fragments\n", rx_frags);

		} else {
			/* we are done receiving data */
			sc->sc_flags &= ~CDCE_FLAG_RX_DATA;
			fwd_mq = 1;
		}

	case USBD_ST_SETUP:
tr_setup:
		if (xfer->flags.stall_pipe) {

			/* we are done */
			sc->sc_flags &= ~CDCE_FLAG_RX_DATA;

			if (xfer->flags_int.usb_mode == USB_MODE_HOST) {
				usbd_transfer_start(sc->sc_xfer[3]);
				free_mq = 1;
				break;
			}
		}
		/* we expect a Multi Frame Ethernet Header */
		if (!(sc->sc_flags & CDCE_FLAG_RX_DATA)) {
			DPRINTF(sc, 0, "expecting length header\n");
			usbd_set_frame_data(xfer, &(sc->sc_rx.hdr), 0);
			xfer->frlengths[0] = sizeof(sc->sc_rx.hdr);
			xfer->nframes = 1;
			xfer->flags.short_xfer_ok = 1;
			usbd_start_hardware(xfer);
			free_mq = 1;
			break;
		}
		/* verify number of fragments */
		if (rx_frags > CDCE_512X4_FRAGS_MAX) {
			/* try to clear stall first */
			xfer->flags.stall_pipe = 1;
			goto tr_setup;
		}
		/* check if the last fragment does not complete a frame */
		x = rx_frags - 1;
		flen = UGETW(sc->sc_rx.hdr.wFragLength[x]);
		if (!(flen & CDCE_512X4_FRAG_LAST_MASK)) {
			DPRINTF(sc, 0, "no last frag mask\n");
			/* try to clear stall first */
			xfer->flags.stall_pipe = 1;
			goto tr_setup;
		}
		/*
		 * Setup a new USB transfer chain to receive all the
		 * IP-frame fragments, automagically defragged :
		 */
		x = 0;
		y = 0;
		while (1) {

			z = x;
			offset = 0;

			/* precompute the frame length */
			while (1) {
				flen = UGETW(sc->sc_rx.hdr.wFragLength[z]);
				offset += (flen & CDCE_512X4_FRAG_LENGTH_MASK);
				if (flen & CDCE_512X4_FRAG_LAST_MASK) {
					break;
				}
				z++;
			}

			if (offset >= sizeof(struct ether_header)) {
				/*
				 * allocate a suitable memory buffer, if
				 * possible
				 */
				if (offset > (MCLBYTES - ETHER_ALIGN)) {
					/* try to clear stall first */
					xfer->flags.stall_pipe = 1;
					goto tr_setup;
				} if (offset > (MHLEN - ETHER_ALIGN)) {
					m = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR);
				} else {
					m = m_gethdr(M_DONTWAIT, MT_DATA);
				}
			} else {
				m = NULL;	/* dump it */
			}

			DPRINTF(sc, 16, "frame %u, length = %u \n", y, offset);

			/* check if we have a buffer */
			if (m) {
				m->m_data = USBD_ADD_BYTES(m->m_data, ETHER_ALIGN);
				m->m_pkthdr.rcvif = ifp;
				m->m_pkthdr.len = m->m_len = offset;

				/* enqueue */
				_IF_ENQUEUE(&(sc->sc_rx_mq), m);

				data_ptr = m->m_data;
				ifp->if_ipackets++;
			} else {
				data_ptr = sc->sc_rx.data;
				ifp->if_ierrors++;
			}

			/* setup the RX chain */
			offset = 0;
			while (1) {

				flen = UGETW(sc->sc_rx.hdr.wFragLength[x]);

				usbd_set_frame_data(xfer,
				    USBD_ADD_BYTES(data_ptr, offset), x);

				xfer->frlengths[x] =
				    (flen & CDCE_512X4_FRAG_LENGTH_MASK);

				DPRINTF(sc, 16, "length[%u] = %u\n",
				    x, xfer->frlengths[x]);

				offset += xfer->frlengths[x];

				x++;

				if (flen & CDCE_512X4_FRAG_LAST_MASK) {
					break;
				}
			}

			y++;

			if (x == rx_frags) {
				break;
			}
			if (y == CDCE_512X4_FRAMES_MAX) {
				/* try to clear stall first */
				xfer->flags.stall_pipe = 1;
				goto tr_setup;
			}
		}

		DPRINTF(sc, 0, "nframes = %u\n", x);

		xfer->nframes = x;
		xfer->flags.short_xfer_ok = 0;
		usbd_start_hardware(xfer);
		break;

	default:			/* Error */
		DPRINTF(sc, 0, "error = %s\n",
		    usbd_errstr(xfer->error));

		if (xfer->error != USBD_ERR_CANCELLED) {
			/* try to clear stall first */
			xfer->flags.stall_pipe = 1;
			goto tr_setup;
		}
		free_mq = 1;
		break;
	}

	/*
	 * At the end of a USB callback it is always safe to unlock
	 * the private mutex of a device!
	 *
	 *
	 * By safe we mean that if "usbd_transfer_stop()" is called,
	 * we will get a callback having the error code
	 * USBD_ERR_CANCELLED.
	 */
	if (fwd_mq) {
		cdce_fwd_mq(sc, &(sc->sc_rx_mq));
	}
	if (free_mq) {
		cdce_free_mq(&(sc->sc_rx_mq));
	}
	return;
}

static void
cdce_bulk_read_std_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;
	struct mbuf *m_rx = NULL;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		DPRINTF(sc, 0, "received %u bytes in %u frames\n",
		    xfer->actlen, xfer->aframes);

		if (sc->sc_flags & CDCE_FLAG_ZAURUS) {

			/* Strip off CRC added by Zaurus */
			if (xfer->frlengths[0] >= MAX(4, 14)) {
				xfer->frlengths[0] -= 4;
			}
		}
		_IF_DEQUEUE(&(sc->sc_rx_mq), m);

		if (m) {

			if (xfer->frlengths[0] < sizeof(struct ether_header)) {
				m_freem(m);
				goto tr_setup;
			}
			ifp->if_ipackets++;
			m->m_pkthdr.rcvif = ifp;
			m->m_pkthdr.len = m->m_len = xfer->frlengths[0];
			m_rx = m;
		}
	case USBD_ST_SETUP:
tr_setup:
		if (xfer->flags.stall_pipe) {

			if (xfer->flags_int.usb_mode == USB_MODE_HOST) {
				usbd_transfer_start(sc->sc_xfer[3]);
				break;
			}
		}
		m = usbd_ether_get_mbuf();
		if (m == NULL) {

			/*
			 * We are out of mbufs and need to dump all the
			 * received data !
			 */
			usbd_set_frame_data(xfer, &(sc->sc_rx.data), 0);
			xfer->frlengths[0] = sizeof(sc->sc_rx.data);

		} else {
			usbd_set_frame_data(xfer, m->m_data, 0);
			xfer->frlengths[0] = m->m_len;
			_IF_ENQUEUE(&(sc->sc_rx_mq), m);
		}
		xfer->nframes = 1;
		usbd_start_hardware(xfer);
		break;

	default:			/* Error */
		DPRINTF(sc, 0, "error = %s\n",
		    usbd_errstr(xfer->error));

		/* free all mbufs */
		cdce_free_mq(&(sc->sc_rx_mq));

		if (xfer->error != USBD_ERR_CANCELLED) {
			/* try to clear stall first */
			xfer->flags.stall_pipe = 1;
			goto tr_setup;
		}
		return;
	}

	/*
	 * At the end of a USB callback it is always safe to unlock
	 * the private mutex of a device! That is why we do the
	 * "if_input" here, and not some lines up!
	 *
	 * By safe we mean that if "usbd_transfer_stop()" is called,
	 * we will get a callback having the error code
	 * USBD_ERR_CANCELLED.
	 */
	if (m_rx) {
		mtx_unlock(&(sc->sc_mtx));
		(ifp->if_input) (ifp, m_rx);
		mtx_lock(&(sc->sc_mtx));
	}
	return;
}

static void
cdce_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;

	/* first call - set the correct callback */
	if (sc->sc_iface_protocol == UIPROTO_CDC_ETH_512X4) {
		xfer->callback = &cdce_bulk_read_512x4_callback;
	} else {
		xfer->callback = &cdce_bulk_read_std_callback;
	}
	(xfer->callback) (xfer);
	return;
}

static int
cdce_ifmedia_upd_cb(struct ifnet *ifp)
{
	/* no-op, cdce has only 1 possible media type */
	return (0);
}

static void
cdce_ifmedia_sts_cb(struct ifnet *const ifp, struct ifmediareq *req)
{

	req->ifm_status = IFM_AVALID | IFM_ACTIVE;
	req->ifm_active = IFM_ETHER | IFM_10_T;
}

static void
cdce_intr_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
cdce_intr_read_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		DPRINTF(sc, 0, "Received %d bytes\n",
		    xfer->actlen);

		/* TODO: decode some indications */

	case USBD_ST_SETUP:
tr_setup:
		if (xfer->flags.stall_pipe &&
		    (xfer->flags_int.usb_mode == USB_MODE_HOST)) {
			usbd_transfer_start(sc->sc_xfer[5]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		break;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			/* start clear stall */
			xfer->flags.stall_pipe = 1;
			goto tr_setup;
		}
		break;
	}
	return;
}

static void
cdce_intr_write_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:

		DPRINTF(sc, 0, "Transferred %d bytes\n", xfer->actlen);

	case USBD_ST_SETUP:
tr_setup:
		if (xfer->flags.stall_pipe &&
		    (xfer->flags_int.usb_mode == USB_MODE_HOST)) {
			usbd_transfer_start(sc->sc_xfer[5]);
		} else {
#if 0
			xfer->frlengths[0] = XXX;
			usbd_start_hardware(xfer);
#endif
		}
		break;

	default:			/* Error */
		if (xfer->error != USBD_ERR_CANCELLED) {
			/* start clear stall */
			xfer->flags.stall_pipe = 1;
			goto tr_setup;
		}
		break;
	}
	return;
}

static int
cdce_handle_request(device_t dev,
    const void *req, void **pptr, uint16_t *plen,
    uint16_t offset, uint8_t is_complete)
{
	return (ENXIO);			/* use builtin handler */
}
