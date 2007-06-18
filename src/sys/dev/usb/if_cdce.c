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
__FBSDID("$FreeBSD: src/sys/dev/usb/if_cdce.c,v 1.16 2006/10/07 17:35:37 flz Exp $");

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

static usbd_callback_t cdce_bulk_write_clear_stall_callback;
static usbd_callback_t cdce_bulk_write_callback;
static usbd_callback_t cdce_bulk_read_clear_stall_callback;
static usbd_callback_t cdce_bulk_read_callback;

static void
cdce_start_cb(struct ifnet *ifp);

static void
cdce_start_transfers(struct cdce_softc *sc);

static u_int32_t
cdce_m_crc32(struct mbuf *m, u_int32_t src_offset, u_int32_t src_len);

static void
cdce_stop(struct cdce_softc *sc);

static int
cdce_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data);

static void
cdce_init_cb(void *arg);

static int
cdce_ifmedia_upd_cb(struct ifnet *ifp);

static void
cdce_ifmedia_sts_cb(struct ifnet * const ifp, struct ifmediareq *req);

#define DPRINTF(...)

static const struct usbd_config cdce_config[CDCE_ENDPT_MAX] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_OUT,
      .bufsize   = (MCLBYTES + 4),
      .flags     = (USBD_PIPE_BOF|USBD_USE_DMA|USBD_FORCE_SHORT_XFER),
      .callback  = &cdce_bulk_write_callback,
      .timeout   = 10000, /* 10 seconds */
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_IN,
      .bufsize   = (MCLBYTES + 4),
      .flags     = (USBD_PIPE_BOF|USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &cdce_bulk_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &cdce_bulk_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
      .interval  = 50, /* 50ms */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &cdce_bulk_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
      .interval  = 50, /* 50ms */
    },
};

static device_method_t cdce_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, cdce_probe),
	DEVMETHOD(device_attach, cdce_attach),
	DEVMETHOD(device_detach, cdce_detach),
	DEVMETHOD(device_shutdown, cdce_shutdown),

	{ 0, 0 }
};

static driver_t cdce_driver = {
	.name    = "cdce",
	.methods = cdce_methods,
	.size    = sizeof(struct cdce_softc),
};

static devclass_t cdce_devclass;

DRIVER_MODULE(cdce, uhub, cdce_driver, cdce_devclass, usbd_driver_load, 0);
MODULE_VERSION(cdce, 0);
MODULE_DEPEND(cdce, usb, 1, 1, 1);
MODULE_DEPEND(cdce, ether, 1, 1, 1);

static const struct cdce_type cdce_devs[] = {
  {{ USB_VENDOR_PROLIFIC, USB_PRODUCT_PROLIFIC_PL2501 }, CDCE_FLAG_NO_UNION },
  {{ USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SL5500 }, CDCE_FLAG_ZAURUS },
  {{ USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SLA300 }, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION },
  {{ USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SL5600 }, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION },
  {{ USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SLC700 }, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION },
  {{ USB_VENDOR_SHARP, USB_PRODUCT_SHARP_SLC750 }, CDCE_FLAG_ZAURUS | CDCE_FLAG_NO_UNION },
  {{ USB_VENDOR_GMATE, USB_PRODUCT_GMATE_YP3X00 }, CDCE_FLAG_NO_UNION },
  {{ USB_VENDOR_NETCHIP, USB_PRODUCT_NETCHIP_ETHERNETGADGET }, CDCE_FLAG_NO_UNION },
};
#define cdce_lookup(v, p) ((const struct cdce_type *)usb_lookup(cdce_devs, v, p))

static int
cdce_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_interface_descriptor_t *id;

	if (uaa->iface == NULL) {
	    return UMATCH_NONE;
	}

	id = usbd_get_interface_descriptor(uaa->iface);
	if (id == NULL) {
	    return UMATCH_NONE;
	}

	if (cdce_lookup(uaa->vendor, uaa->product) != NULL) {
	    return UMATCH_VENDOR_PRODUCT;
	}

	if ((id->bInterfaceClass == UICLASS_CDC) && 
	    (id->bInterfaceSubClass == 
	     UISUBCLASS_ETHERNET_NETWORKING_CONTROL_MODEL)) {
	    return UMATCH_IFACECLASS_GENERIC;
	}

	return UMATCH_NONE;
}

static int
cdce_attach(device_t dev)
{
	struct cdce_softc * sc = device_get_softc(dev);
	struct usb_attach_arg * uaa = device_get_ivars(dev);
	struct usbd_interface * iface;
	const usb_cdc_union_descriptor_t * ud;
	const usb_cdc_ethernet_descriptor_t * ue;
	const usb_interface_descriptor_t *id;
	const struct cdce_type * t;
	struct ifnet * ifp;
	int error;
	u_int8_t i;
	u_int8_t eaddr[ETHER_ADDR_LEN];
	u_int8_t eaddr_str[(ETHER_ADDR_LEN * 2) + 1];

	if (sc == NULL) {
	    return ENOMEM;
	}

	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;
	sc->sc_unit = device_get_unit(dev);

	t = cdce_lookup(uaa->vendor, uaa->product);
	if (t) {
	    sc->sc_flags = t->cdce_flags;
	}

	usbd_set_desc(dev, uaa->device);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s",
		 device_get_nameunit(dev));

	mtx_init(&(sc->sc_mtx), "cdce lock", NULL, MTX_DEF | MTX_RECURSE);

	if (sc->sc_flags & CDCE_FLAG_NO_UNION) {
	    sc->sc_data_iface_index = uaa->iface_index;
	    sc->sc_data_iface_no = 0; /* not used */
	    goto alloc_transfers;
	}

	ud = usbd_find_descriptor
	  (uaa->device, NULL, uaa->iface_index,
	   UDESC_CS_INTERFACE, UDESCSUB_CDC_UNION);

	if ((ud == NULL) || (ud->bLength < sizeof(*ud))) {
	    device_printf(dev, "no union descriptor!\n");
	    goto detach;
	}

	sc->sc_data_iface_no = ud->bSlaveInterface[0];

	for (i=0; ; i++) {

	    iface = usbd_get_iface(uaa->device, i);

	    if (iface) {

	        id = usbd_get_interface_descriptor(iface);

                if (id && (id->bInterfaceNumber == sc->sc_data_iface_no)) {
                    sc->sc_data_iface_index = i;
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
	 *  The Data Class interface of a networking device shall have a minimum
	 *  of two interface settings. The first setting (the default interface
	 *  setting) includes no endpoints and therefore no networking traffic is
	 *  exchanged whenever the default interface setting is selected. One or
	 *  more additional interface settings are used for normal operation, and
	 *  therefore each includes a pair of endpoints (one IN, and one OUT) to
	 *  exchange network traffic. Select an alternate interface setting to
	 *  initialize the network aspects of the device and to enable the
	 *  exchange of network traffic.
	 * </quote>
	 *
	 * Some devices, most notably cable modems, include interface settings
	 * that have no IN or OUT endpoint, therefore loop through the list of all
	 * available interface settings looking for one with both IN and OUT
	 * endpoints.
	 */

 alloc_transfers:

	for (i = 0; i < 32; i++) {

	    error = usbreq_set_interface
	      (uaa->device, sc->sc_data_iface_index, i);

	    if (error) {
	        device_printf(dev, "no valid alternate setting found!\n");
		goto detach;
	    }

	    error = usbd_transfer_setup
	      (uaa->device, sc->sc_data_iface_index, 
	       sc->sc_xfer, cdce_config, CDCE_ENDPT_MAX,
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
	   UDESC_CS_INTERFACE, UDESCSUB_CDC_ENF);

	if ((ue == NULL) || (ue->bLength < sizeof(*ue)) || 
	    usbreq_get_string_any(uaa->device, ue->iMacAddress, 
				  eaddr_str, sizeof(eaddr_str))) {

	    /* fake MAC address */

	    device_printf(dev, "faking MAC address\n");
	    eaddr[0]= 0x2a;
	    memcpy(&eaddr[1], &ticks, sizeof(u_int32_t));
	    eaddr[5] = sc->sc_unit;

	} else {

	    bzero(eaddr, sizeof(eaddr));

	    for (i = 0; i < (ETHER_ADDR_LEN * 2); i++) {

	        u_int8_t c = eaddr_str[i];

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
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
	ifp->if_snd.ifq_drv_maxlen = IFQ_MAXLEN;
	IFQ_SET_READY(&ifp->if_snd);

	/* no IFM type for 11Mbps USB, so go with 10baseT */
	ifmedia_add(&sc->sc_ifmedia, IFM_ETHER | IFM_10_T, 0, 0);
	ifmedia_set(&sc->sc_ifmedia, IFM_ETHER | IFM_10_T);

	sc->sc_ifp = ifp;

	ether_ifattach(ifp, eaddr);

	return 0; /* success */

 detach:
	cdce_detach(dev);
	return ENXIO; /* failure */
}

static int
cdce_detach(device_t dev)
{
	struct cdce_softc * sc = device_get_softc(dev);
	struct ifnet * ifp;

	mtx_lock(&(sc->sc_mtx));

	cdce_stop(sc);

	ifp = sc->sc_ifp;

	mtx_unlock(&(sc->sc_mtx));

	/* stop all USB transfers first */
	usbd_transfer_unsetup(sc->sc_xfer, CDCE_ENDPT_MAX);

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

	    /* start the USB transfers, 
	     * if not already started:
	     */
	    usbd_transfer_start(sc->sc_xfer[1]);
	    usbd_transfer_start(sc->sc_xfer[0]);
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
	    sc->sc_flags &= ~CDCE_FLAG_WRITE_STALL;
	    usbd_transfer_start(xfer_other);
	}
	return;
}

static void
cdce_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;
	u_int32_t crc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 10, "transfer error, %s\n",
		 usbd_errstr(xfer->error));

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= CDCE_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}

	ifp->if_oerrors++;
	return;

 tr_transferred:
	DPRINTF(sc, 10, "transfer complete\n");

	ifp->if_opackets++;

 tr_setup:

	if (sc->sc_flags & CDCE_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    goto done;
	}

	IFQ_DRV_DEQUEUE(&(ifp->if_snd), m);

	if (m == NULL) {
	    goto done;
	}

	if (m->m_pkthdr.len > MCLBYTES) {
	    m->m_pkthdr.len = MCLBYTES;
	}

	xfer->length = m->m_pkthdr.len;

	usbd_m_copy_in(&(xfer->buf_data), 0, 
		       m, 0, m->m_pkthdr.len);

	if (sc->sc_flags & CDCE_FLAG_ZAURUS) {
	    /* Zaurus wants a 32-bit CRC appended to every frame */

	    crc = htole32(cdce_m_crc32(m, 0, m->m_pkthdr.len));

	    usbd_copy_in(&(xfer->buf_data), 
			 m->m_pkthdr.len, &crc, 4);

	    xfer->length += 4;
	}

	/*
	 * if there's a BPF listener, bounce a copy 
	 * of this frame to him:
	 */
	BPF_MTAP(ifp, m);

	m_freem(m);

	usbd_start_hardware(xfer);

 done:
	return;
}

static int32_t
#ifdef __FreeBSD__
cdce_m_crc32_cb(void *arg, void *src, u_int32_t count)
#else
cdce_m_crc32_cb(void *arg, caddr_t src, u_int32_t count)
#endif
{
	register u_int32_t *p_crc = arg;
	*p_crc = crc32_raw(src, count, *p_crc);
	return 0;
}

static u_int32_t
cdce_m_crc32(struct mbuf *m, u_int32_t src_offset, u_int32_t src_len)
{
	register int error;
	u_int32_t crc = 0xFFFFFFFF;
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

	sc->sc_flags &= ~(CDCE_FLAG_HL_READY|
			  CDCE_FLAG_LL_READY);

	/* stop all the transfers, 
	 * if not already stopped:
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
cdce_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct cdce_softc * sc = ifp->if_softc;
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	switch(command) {
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
	    error = ifmedia_ioctl(ifp, (void *)data, &sc->sc_ifmedia, command);
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
	struct cdce_softc * sc = arg;
	struct ifnet * ifp;

	mtx_lock(&(sc->sc_mtx));

	ifp = sc->sc_ifp;

	/* immediate configuration */

	cdce_stop(sc);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

	sc->sc_flags |= (CDCE_FLAG_READ_STALL|
			 CDCE_FLAG_WRITE_STALL|
			 CDCE_FLAG_LL_READY|
			 CDCE_FLAG_HL_READY);

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
	    sc->sc_flags &= ~CDCE_FLAG_READ_STALL;
	    usbd_transfer_start(xfer_other);
	}
	return;
}

static void
cdce_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct cdce_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m = NULL;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= CDCE_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	DPRINTF(sc, 0, "bulk read error, %s\n",
		usbd_errstr(xfer->error));
	return;

 tr_transferred:

	if (sc->sc_flags & CDCE_FLAG_ZAURUS) {

	    /* Strip off CRC added by Zaurus */
	    if (xfer->actlen >= 4) {
	        xfer->actlen -= 4;
	    }
	}

	if (xfer->actlen < sizeof(struct ether_header)) {
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	m = usbd_ether_get_mbuf();

	if (m == NULL) {
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	xfer->actlen = min(xfer->actlen, m->m_len);

	usbd_copy_out(&(xfer->buf_data), 0, m->m_data, xfer->actlen);

	ifp->if_ipackets++;
	m->m_pkthdr.rcvif = ifp;
	m->m_pkthdr.len = m->m_len = xfer->actlen;

 tr_setup:

	if (sc->sc_flags & CDCE_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}

	/* At the end of a USB callback it is always safe
	 * to unlock the private mutex of a device! That
	 * is why we do the "if_input" here, and not
	 * some lines up!
	 */
	if (m) {
	    mtx_unlock(&(sc->sc_mtx));
	    (ifp->if_input)(ifp, m);
	    mtx_lock(&(sc->sc_mtx));
	}
	return;
}

static int
cdce_ifmedia_upd_cb(struct ifnet *ifp)
{

	/* no-op, cdce has only 1 possible media type */
	return 0;
}

static void
cdce_ifmedia_sts_cb(struct ifnet * const ifp, struct ifmediareq *req)
{

	req->ifm_status = IFM_AVALID | IFM_ACTIVE;
	req->ifm_active = IFM_ETHER | IFM_10_T;
}
