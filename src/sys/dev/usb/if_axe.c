/*-
 * Copyright (c) 1997, 1998, 1999, 2000-2003
 *	Bill Paul <wpaul@windriver.com>.  All rights reserved.
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
 * ARE DISCLAIMED.  IN NO EVENT SHALL Bill Paul OR THE VOICES IN HIS HEAD
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/if_axe.c,v 1.41 2006/10/19 01:15:58 iedowse Exp $");

/*
 * ASIX Electronics AX88172 USB 2.0 ethernet driver. Used in the
 * LinkSys USB200M and various other adapters.
 *
 * Manuals available from:
 * http://www.asix.com.tw/datasheet/mac/Ax88172.PDF
 * Note: you need the manual for the AX88170 chip (USB 1.x ethernet
 * controller) to find the definitions for the RX control register.
 * http://www.asix.com.tw/datasheet/mac/Ax88170.PDF
 *
 * Written by Bill Paul <wpaul@windriver.com>
 * Senior Engineer
 * Wind River Systems
 */

/*
 * The AX88172 provides USB ethernet supports at 10 and 100Mbps.
 * It uses an external PHY (reference designs use a RealTek chip),
 * and has a 64-bit multicast hash filter. There is some information
 * missing from the manual which one needs to know in order to make
 * the chip function:
 *
 * - You must set bit 7 in the RX control register, otherwise the
 *   chip won't receive any packets.
 * - You must initialize all 3 IPG registers, or you won't be able
 *   to send any packets.
 *
 * Note that this device appears to only support loading the station
 * address via autload from the EEPROM (i.e. there's no way to manaully
 * set it).
 *
 * (Adam Weinberger wanted me to name this driver if_gir.c.)
 */

/*
 * NOTE: all function names beginning like "axe_cfg_" can only
 * be called from within the config thread function !
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/endian.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <net/bpf.h>

#define usbd_config_td_cc axe_config_copy
#define usbd_config_td_softc axe_softc

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

/* "device miibus" required.  See GENERIC if you get errors here. */
#include "miibus_if.h"

#include <dev/usb/if_axereg.h>

MODULE_DEPEND(axe, usb, 1, 1, 1);
MODULE_DEPEND(axe, ether, 1, 1, 1);
MODULE_DEPEND(axe, miibus, 1, 1, 1);

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)	\
  do { if (axe_debug > (n)) {	     \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int axe_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, axe, CTLFLAG_RW, 0, "USB axe");
SYSCTL_INT(_hw_usb_axe, OID_AUTO, debug, CTLFLAG_RW, &axe_debug, 0,
	   "axe debug level");
#else
#define DPRINTF(...)
#endif


/*
 * Various supported device vendors/products.
 */
static struct axe_type axe_devs[] = {
	{ USB_VENDOR_ASIX, USB_PRODUCT_ASIX_AX88172 },
	{ USB_VENDOR_DLINK, USB_PRODUCT_DLINK_DUBE100 },
	{ USB_VENDOR_JVC, USB_PRODUCT_JVC_MP_PRX1 },
	{ USB_VENDOR_LINKSYS2, USB_PRODUCT_LINKSYS2_USB200M },
	{ USB_VENDOR_MELCO, USB_PRODUCT_MELCO_LUAU2KTX },
	{ USB_VENDOR_NETGEAR, USB_PRODUCT_NETGEAR_FA120 },
	{ USB_VENDOR_SYSTEMTALKS, USB_PRODUCT_SYSTEMTALKS_SGCX2UL },
	{ USB_VENDOR_SITECOM, USB_PRODUCT_SITECOM_LN029 },
	{ 0, 0 }
};

static device_probe_t axe_probe;
static device_attach_t axe_attach;
static device_detach_t axe_detach;
static device_shutdown_t axe_shutdown;

static usbd_callback_t axe_intr_clear_stall_callback;
static usbd_callback_t axe_intr_callback;
static usbd_callback_t axe_bulk_read_clear_stall_callback;
static usbd_callback_t axe_bulk_read_callback;
static usbd_callback_t axe_bulk_write_clear_stall_callback;
static usbd_callback_t axe_bulk_write_callback;

static void
axe_cfg_cmd(struct axe_softc *sc, u_int16_t cmd, u_int16_t index, 
	    u_int16_t val, void *buf);

static miibus_readreg_t axe_cfg_miibus_readreg;
static miibus_writereg_t axe_cfg_miibus_writereg;
static miibus_statchg_t axe_cfg_miibus_statchg;

static usbd_config_td_command_t axe_cfg_ifmedia_upd;
static usbd_config_td_command_t axe_config_copy;
static usbd_config_td_command_t axe_cfg_setmulti;
static usbd_config_td_command_t axe_cfg_first_time_setup;
static usbd_config_td_command_t axe_cfg_tick;
static usbd_config_td_command_t axe_cfg_pre_init;
static usbd_config_td_command_t axe_cfg_init;
static usbd_config_td_command_t axe_cfg_promisc_upd;
static usbd_config_td_command_t axe_cfg_pre_stop;
static usbd_config_td_command_t axe_cfg_stop;

static int
axe_ifmedia_upd_cb(struct ifnet *ifp);

static void
axe_ifmedia_sts_cb(struct ifnet *ifp, struct ifmediareq *ifmr);

static void
axe_cfg_reset(struct axe_softc *sc);

static void
axe_start_cb(struct ifnet *ifp);

static void
axe_start_transfers(struct axe_softc *sc);

static void
axe_init_cb(void *arg);

static int
axe_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data);

static void
axe_watchdog(void *arg);

static const struct usbd_config axe_config[AXE_ENDPT_MAX] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = MCLBYTES,
      .flags     = (USBD_USE_DMA|USBD_FORCE_SHORT_XFER),
      .callback  = &axe_bulk_write_callback,
      .timeout   = 10000, /* 10 seconds */
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = MCLBYTES,
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &axe_bulk_read_callback,
      .timeout   = 0, /* no timeout */
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &axe_bulk_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &axe_bulk_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [4] = {
      .type      = UE_INTERRUPT,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .flags     = USBD_SHORT_XFER_OK,
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &axe_intr_callback,
    },

    [5] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &axe_intr_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static device_method_t axe_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		axe_probe),
	DEVMETHOD(device_attach,	axe_attach),
	DEVMETHOD(device_detach,	axe_detach),
	DEVMETHOD(device_shutdown,	axe_shutdown),

	/* bus interface */
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_driver_added,	bus_generic_driver_added),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	axe_cfg_miibus_readreg),
	DEVMETHOD(miibus_writereg,	axe_cfg_miibus_writereg),
	DEVMETHOD(miibus_statchg,	axe_cfg_miibus_statchg),

	{ 0, 0 }
};

static driver_t axe_driver = {
	.name    = "axe",
	.methods = axe_methods,
	.size    = sizeof(struct axe_softc),
};

static devclass_t axe_devclass;

DRIVER_MODULE(axe, uhub, axe_driver, axe_devclass, usbd_driver_load, 0);
DRIVER_MODULE(miibus, axe, miibus_driver, miibus_devclass, 0, 0);

static void
axe_cfg_cmd(struct axe_softc *sc, u_int16_t cmd, u_int16_t index, 
	    u_int16_t val, void *buf)
{
	usb_device_request_t req;
	usbd_status err;
	u_int16_t length = AXE_CMD_LEN(cmd);

	req.bmRequestType = (AXE_CMD_IS_WRITE(cmd) ? 
			     UT_WRITE_VENDOR_DEVICE :
			     UT_READ_VENDOR_DEVICE);
	req.bRequest = AXE_CMD_CMD(cmd);
	USETW(req.wValue, val);
	USETW(req.wIndex, index);
	USETW(req.wLength, length);

	if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto error;
	}

	err = usbd_do_request_flags_mtx(sc->sc_udev, &(sc->sc_mtx), &req, 
					buf, 0, NULL, 1000);

	if (err) {

	    DPRINTF(sc, 0, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));

	error:

	    if ((req.bmRequestType & UT_READ) && length) {
	        bzero(buf, length);
	    }
	}
	return;
}

static int
axe_cfg_miibus_readreg(device_t dev, int phy, int reg)
{
	struct axe_softc * sc = device_get_softc(dev);
	u_int16_t val;

	mtx_lock(&(sc->sc_mtx)); /* XXX */

#if 0
	/*
	 * The chip tells us the MII address of any supported
	 * PHYs attached to the chip, so only read from those.
	 */

	if ((sc->sc_phyaddrs[0] != AXE_NOPHY) && (phy != sc->sc_phyaddrs[0])) {
	    val = 0; 
	    goto done;
	}

	if ((sc->sc_phyaddrs[1] != AXE_NOPHY) && (phy != sc->sc_phyaddrs[1])) {
	    val = 0; 
	    goto done;
	}
#endif
	if ((sc->sc_phyaddrs[0] != 0xFF) && (sc->sc_phyaddrs[0] != phy)) {
	    val = 0; 
	    goto done;
	}

	axe_cfg_cmd(sc, AXE_CMD_MII_OPMODE_SW, 0, 0, NULL);
	axe_cfg_cmd(sc, AXE_CMD_MII_READ_REG, reg, phy, &val);
	axe_cfg_cmd(sc, AXE_CMD_MII_OPMODE_HW, 0, 0, NULL);

	val = le16toh(val);

	if (val) {
	    sc->sc_phyaddrs[0] = phy;
	}

 done:
	mtx_unlock(&(sc->sc_mtx)); /* XXX */

	return (val);
}

static int
axe_cfg_miibus_writereg(device_t dev, int phy, int reg, int val)
{
	struct axe_softc * sc = device_get_softc(dev);

	val = htole16(val);

	mtx_lock(&(sc->sc_mtx)); /* XXX */

	axe_cfg_cmd(sc, AXE_CMD_MII_OPMODE_SW, 0, 0, NULL);
	axe_cfg_cmd(sc, AXE_CMD_MII_WRITE_REG, reg, phy, &val);
	axe_cfg_cmd(sc, AXE_CMD_MII_OPMODE_HW, 0, 0, NULL);

	mtx_unlock(&(sc->sc_mtx)); /* XXX */

	return 0;
}

static void
axe_cfg_miibus_statchg(device_t dev)
{
	/* doesn't seem to be necessary */
	return;
}

/*
 * Set media options.
 */
static int
axe_ifmedia_upd_cb(struct ifnet *ifp)
{
	struct axe_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL, &axe_cfg_ifmedia_upd, 0, 0);
	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

static void
axe_cfg_ifmedia_upd(struct axe_softc *sc,
		    struct axe_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp = sc->sc_ifp;
	struct mii_data * mii = GET_MII(sc);

	if ((ifp == NULL) || 
	    (mii == NULL)) {
	    /* not ready */
	    return;
	}

	sc->sc_flags |= AXE_FLAG_WAIT_LINK;

	if (mii->mii_instance) {
		struct mii_softc *miisc;
		LIST_FOREACH(miisc, &mii->mii_phys, mii_list) {
			 mii_phy_reset(miisc);
		}
	}

	mii_mediachg(mii);

	return;
}

/*
 * Report current media status.
 */
static void
axe_ifmedia_sts_cb(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct axe_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	ifmr->ifm_active = sc->sc_media_active;
	ifmr->ifm_status = sc->sc_media_status;

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
axe_config_copy(struct axe_softc *sc, 
		struct axe_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp = sc->sc_ifp;
	struct ifmultiaddr * ifma;
	u_int8_t h;
	u_int8_t i;

	bzero(cc, sizeof(*cc));

	if (ifp) {
	    for (i = 0; i < ETHER_ADDR_LEN; i++) {
	        cc->if_lladdr[i] = IF_LLADDR(ifp)[i];
	    }

	    cc->if_flags = ifp->if_flags;

	    /* compute hash bits for multicast filter */

	    IF_ADDR_LOCK(ifp);
	    TAILQ_FOREACH(ifma, &ifp->if_multiaddrs, ifma_link)
	    {
	        if (ifma->ifma_addr->sa_family != AF_LINK) {
		    continue;
		}

		h = (ether_crc32_be
		     (LLADDR((struct sockaddr_dl *)(ifma->ifma_addr)), 
		      ETHER_ADDR_LEN) >> 26);

		cc->if_hash[(h >> 3)] |= (1 << (h & 7));
	    }
	    IF_ADDR_UNLOCK(ifp);
	}
	return;
}

static void
axe_cfg_setmulti(struct axe_softc *sc,
		 struct axe_config_copy *cc, u_int16_t refcount)
{
	u_int16_t rxmode;

	axe_cfg_cmd(sc, AXE_CMD_RXCTL_READ, 0, 0, &rxmode);

	rxmode = le16toh(rxmode);

	if (cc->if_flags & (IFF_ALLMULTI|IFF_PROMISC)) {
		rxmode |= AXE_RXCMD_ALLMULTI;
		axe_cfg_cmd(sc, AXE_CMD_RXCTL_WRITE, 0, rxmode, NULL);
		return;
	}

	rxmode &= ~AXE_RXCMD_ALLMULTI;

	axe_cfg_cmd(sc, AXE_CMD_WRITE_MCAST, 0, 0, cc->if_hash);
	axe_cfg_cmd(sc, AXE_CMD_RXCTL_WRITE, 0, rxmode, NULL);

	return;
}

static void
axe_cfg_reset(struct axe_softc *sc)
{
	usbd_status err;

	mtx_unlock(&(sc->sc_mtx));

	mtx_lock(&Giant);

	err = usbreq_set_config(sc->sc_udev, AXE_CONFIG_NO);

	mtx_unlock(&Giant);

	mtx_lock(&(sc->sc_mtx));

	if (err) {
	    DPRINTF(sc, 0, "reset failed (ignored)\n");
	}

	/* wait a little while for the 
	 * chip to get its brains in order:
	 */
	err = usbd_config_td_sleep(&(sc->sc_config_td), hz/100);

	return;
}

/*
 * Probe for a AX88172 chip.
 */
static int
axe_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct axe_type	*t;

	if (uaa->iface != NULL) {
	    return UMATCH_NONE;
	}

	t = axe_devs;
	while(t->axe_vid) {
	    if ((uaa->vendor == t->axe_vid) &&
		(uaa->product == t->axe_did)) {
	        return UMATCH_VENDOR_PRODUCT;
	    }
	    t++;
	}
	return UMATCH_NONE;
}

/*
 * Attach the interface. Allocate softc structures, do ifmedia
 * setup and ethernet/BPF attach.
 */
static int
axe_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct axe_softc *sc = device_get_softc(dev);
	int32_t error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;
	sc->sc_unit = device_get_unit(dev);

	usbd_set_desc(dev, uaa->device);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s",
		 device_get_nameunit(dev));

	mtx_init(&(sc->sc_mtx), "axe lock", NULL, MTX_DEF | MTX_RECURSE);

	__callout_init_mtx(&(sc->sc_watchdog),
			   &(sc->sc_mtx), CALLOUT_RETURNUNLOCKED);

	error = usbd_set_config_no(uaa->device, AXE_CONFIG_NO, 1);

	if (error) {
		device_printf(dev, "setting config "
			      "number failed!\n");
		goto detach;
	}

	error = usbd_transfer_setup(uaa->device, AXE_IFACE_IDX, 
				    sc->sc_xfer, axe_config, AXE_ENDPT_MAX,
				    sc, &(sc->sc_mtx));
	if (error) {
		device_printf(dev, "allocating USB "
			      "transfers failed!\n");
		goto detach;
	}

	error = usbd_config_td_setup(&(sc->sc_config_td), sc, &(sc->sc_mtx),
				     NULL, sizeof(struct axe_config_copy), 16);
	if (error) {
		device_printf(dev, "could not setup config "
			      "thread!\n");
		goto detach;
	}

	mtx_lock(&(sc->sc_mtx));

	sc->sc_flags |= AXE_FLAG_WAIT_LINK;

	/* start setup */

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL, &axe_cfg_first_time_setup, 0, 0);

	/* start watchdog (will exit mutex) */

	axe_watchdog(sc);

	return 0; /* success */

 detach:
	axe_detach(dev);
	return ENXIO; /* failure */
}

static void
axe_cfg_first_time_setup(struct axe_softc *sc,
			 struct axe_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp;
	int error;
	u_int8_t eaddr[min(ETHER_ADDR_LEN,6)];

       	/* set default value */
	bzero(eaddr, sizeof(eaddr));

	/*
	 * Get station address.
	 */
	axe_cfg_cmd(sc, AXE_CMD_READ_NODEID, 0, 0, eaddr);

	/*
	 * Load IPG values and PHY indexes.
	 */
	axe_cfg_cmd(sc, AXE_CMD_READ_IPG012, 0, 0, sc->sc_ipgs);
	axe_cfg_cmd(sc, AXE_CMD_READ_PHYID, 0, 0, sc->sc_phyaddrs);

	/*
	 * Work around broken adapters that appear to lie about
	 * their PHY addresses.
	 */
	sc->sc_phyaddrs[0] = sc->sc_phyaddrs[1] = 0xFF;

	mtx_unlock(&(sc->sc_mtx));

	ifp = if_alloc(IFT_ETHER);

	mtx_lock(&(sc->sc_mtx));

	if (ifp == NULL) {
	    printf("%s: could not if_alloc()\n",
		   sc->sc_name);
	    goto done;
	}

	sc->sc_evilhack = ifp;

	ifp->if_softc = sc;
	if_initname(ifp, "axe", sc->sc_unit);
	ifp->if_mtu = ETHERMTU;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = axe_ioctl_cb;
	ifp->if_start = axe_start_cb;
	ifp->if_watchdog = NULL;
	ifp->if_init = axe_init_cb;
	ifp->if_snd.ifq_maxlen = IFQ_MAXLEN;

	/* XXX need Giant when accessing
	 * the device structures !
	 */

	mtx_unlock(&(sc->sc_mtx));

	mtx_lock(&Giant);

	error = mii_phy_probe(sc->sc_dev, &(sc->sc_miibus),
			      &axe_ifmedia_upd_cb, 
			      &axe_ifmedia_sts_cb);
	mtx_unlock(&Giant);

	mtx_lock(&(sc->sc_mtx));

	if (error) {
	    printf("%s: MII without any PHY!\n", 
		   sc->sc_name);
	    if_free(ifp);
	    goto done;
	}

	sc->sc_ifp = ifp;

	mtx_unlock(&(sc->sc_mtx));

	/*
	 * Call MI attach routine.
	 */

	ether_ifattach(ifp, eaddr);

	mtx_lock(&(sc->sc_mtx));

 done:
	return;
}

static int
axe_detach(device_t dev)
{
	struct axe_softc * sc = device_get_softc(dev);
	struct ifnet * ifp;

	mtx_lock(&(sc->sc_mtx));

	usbd_config_td_stop(&(sc->sc_config_td));

	__callout_stop(&sc->sc_watchdog);

	axe_cfg_pre_stop(sc, NULL, 0);

	ifp = sc->sc_ifp;

	mtx_unlock(&(sc->sc_mtx));

	/* get rid of any late children */
	bus_generic_detach(dev);

	if (ifp) {
	    ether_ifdetach(ifp);
	    if_free(ifp);
	}

	usbd_transfer_unsetup(sc->sc_xfer, AXE_ENDPT_MAX);

	usbd_config_td_unsetup(&(sc->sc_config_td));

	__callout_drain(&(sc->sc_watchdog));

	mtx_destroy(&(sc->sc_mtx));

	return 0;
}

static void
axe_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct axe_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
        return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~AXE_FLAG_INTR_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~AXE_FLAG_INTR_STALL;
	DPRINTF(sc, 0, "interrupt read pipe stopped\n");
	return;
}

static void
axe_intr_callback(struct usbd_xfer *xfer)
{
	struct axe_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:

	/* do nothing */

 tr_setup:
	if (sc->sc_flags & AXE_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[5]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* start clear stall */
	    sc->sc_flags |= AXE_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[5]);
	}
	return;
}

static void
axe_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct axe_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~AXE_FLAG_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~AXE_FLAG_READ_STALL;
	DPRINTF(sc, 0, "bulk read pipe stopped\n");
	return;
}

static void
axe_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct axe_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= AXE_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	DPRINTF(sc, 0, "bulk read error, %s\n",
		usbd_errstr(xfer->error));
	return;

 tr_transferred:

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

	(ifp->if_input)(ifp, m);

 tr_setup:

	if (sc->sc_flags & AXE_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
axe_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct axe_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~AXE_FLAG_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~AXE_FLAG_WRITE_STALL;
	DPRINTF(sc, 0, "bulk write pipe stopped\n");
	return;
}

static void
axe_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct axe_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 10, "transfer error, %s\n",
		 usbd_errstr(xfer->error));

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= AXE_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}

	ifp->if_oerrors++;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	return;

 tr_transferred:
	DPRINTF(sc, 10, "transfer complete\n");

	ifp->if_opackets++;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

 tr_setup:

	if (sc->sc_flags & AXE_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    goto done;
	}

	if (sc->sc_flags & AXE_FLAG_WAIT_LINK) {
	    /* don't send anything 
	     * if there is no link !
	     */
	    goto done;
	}

	IF_DEQUEUE(&(ifp->if_snd), m);

	if (m == NULL) {
	    goto done;
	}

	if (m->m_pkthdr.len > MCLBYTES) {
	    m->m_pkthdr.len = MCLBYTES;
	}

	xfer->length = m->m_pkthdr.len;

	usbd_m_copy_in(&(xfer->buf_data), 0, 
		       m, 0, m->m_pkthdr.len);

	/*
	 * if there's a BPF listener, bounce a copy 
	 * of this frame to him:
	 */
	BPF_MTAP(ifp, m);

	m_freem(m);

	usbd_start_hardware(xfer);

	ifp->if_drv_flags |= IFF_DRV_OACTIVE;

 done:
	return;
}

static void
axe_cfg_tick(struct axe_softc *sc,
	     struct axe_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp = sc->sc_ifp;
	struct mii_data * mii = GET_MII(sc);

	if ((ifp == NULL) || 
	    (mii == NULL)) {
	    /* not ready */
	    return;
	}

	mii_tick(mii);

	mii_pollstat(mii);

	if ((sc->sc_flags & AXE_FLAG_WAIT_LINK) &&
	    (mii->mii_media_status & IFM_ACTIVE) &&
	    (IFM_SUBTYPE(mii->mii_media_active) != IFM_NONE)) {
	    sc->sc_flags &= ~AXE_FLAG_WAIT_LINK;
	}

	sc->sc_media_active = mii->mii_media_active;
	sc->sc_media_status = mii->mii_media_status;

	/* start stopped transfers, if any */

	axe_start_transfers(sc);

	return;
}

static void
axe_start_cb(struct ifnet *ifp)
{
	struct axe_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	axe_start_transfers(sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
axe_start_transfers(struct axe_softc *sc)
{
	if ((sc->sc_flags & AXE_FLAG_LL_READY) &&
	    (sc->sc_flags & AXE_FLAG_HL_READY)) {

	    /* start the USB transfers, 
	     * if not already started:
	     */
	    usbd_transfer_start(sc->sc_xfer[4]);
	    usbd_transfer_start(sc->sc_xfer[1]);
	    usbd_transfer_start(sc->sc_xfer[0]);
	}
	return;
}

static void
axe_init_cb(void *arg)
{
	struct axe_softc *sc = arg;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &axe_cfg_pre_init, &axe_cfg_init, 0, 0);
	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
axe_cfg_pre_init(struct axe_softc *sc,
		 struct axe_config_copy *cc, u_int16_t refcount)
{
	struct ifnet *ifp = sc->sc_ifp;

	/* immediate configuration */

	axe_cfg_pre_stop(sc, cc, 0);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	sc->sc_flags |= AXE_FLAG_HL_READY;
	return;
}

static void
axe_cfg_init(struct axe_softc *sc,
	     struct axe_config_copy *cc, u_int16_t refcount)
{
	struct mii_data *mii = GET_MII(sc);
	u_int16_t rxmode;

	/*
	 * Cancel pending I/O
	 */

	axe_cfg_stop(sc, cc, 0);

#if 0
	/* Set MAC address */
	axe_mac(sc, cc->if_lladdr);
#endif

	/* Set transmitter IPG values */
	axe_cfg_cmd(sc, AXE_CMD_WRITE_IPG0, 0, sc->sc_ipgs[0], NULL);
	axe_cfg_cmd(sc, AXE_CMD_WRITE_IPG1, 0, sc->sc_ipgs[1], NULL);
	axe_cfg_cmd(sc, AXE_CMD_WRITE_IPG2, 0, sc->sc_ipgs[2], NULL);

	/* Enable receiver, set RX mode */
	rxmode = (AXE_RXCMD_UNICAST|AXE_RXCMD_MULTICAST|AXE_RXCMD_ENABLE);

	/* If we want promiscuous mode, set the allframes bit. */
	if (cc->if_flags & IFF_PROMISC) {
	    rxmode |= AXE_RXCMD_PROMISC;
	}

	if (cc->if_flags & IFF_BROADCAST) {
	    rxmode |= AXE_RXCMD_BROADCAST;
	}

	axe_cfg_cmd(sc, AXE_CMD_RXCTL_WRITE, 0, rxmode, NULL);

	/* Load the multicast filter. */
	axe_cfg_setmulti(sc, cc, 0);

	mii_mediachg(mii);

	sc->sc_flags |= (AXE_FLAG_READ_STALL|
			 AXE_FLAG_WRITE_STALL|
			 AXE_FLAG_LL_READY);

	axe_start_transfers(sc);

	return;
}

static void
axe_cfg_promisc_upd(struct axe_softc *sc,
		    struct axe_config_copy *cc, u_int16_t refcount)
{
	u_int16_t rxmode;

	axe_cfg_cmd(sc, AXE_CMD_RXCTL_READ, 0, 0, &rxmode);

	rxmode = le16toh(rxmode);

	if (cc->if_flags & IFF_PROMISC) {
	    rxmode |= AXE_RXCMD_PROMISC;
	} else {
	    rxmode &= ~AXE_RXCMD_PROMISC;
	}

	axe_cfg_cmd(sc, AXE_CMD_RXCTL_WRITE, 0, rxmode, NULL);

	axe_cfg_setmulti(sc, cc, 0);

	return;
}

static int
axe_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct axe_softc * sc = ifp->if_softc;
	struct mii_data * mii;
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	switch(command) {
	case SIOCSIFFLAGS:

	    if (ifp->if_flags & IFF_UP) {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &axe_config_copy, 
		       &axe_cfg_promisc_upd, 0, 0);
		} else {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &axe_cfg_pre_init, 
		       &axe_cfg_init, 0, 0); 
		}
	    } else {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &axe_cfg_pre_stop,
		       &axe_cfg_stop, 0, 0);
		}
	    }
	    break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &axe_config_copy, 
	       &axe_cfg_setmulti, 0, 0);
	    break;

	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
	    mii = GET_MII(sc);
	    if (mii == NULL) {
	        error = EINVAL;
	    } else {
	        error = ifmedia_ioctl
		  (ifp, (void *)data, &(mii->mii_media), command);
	    }
	    break;

	default:
	    error = ether_ioctl(ifp, command, data);
	    break;
	}

	mtx_unlock(&(sc->sc_mtx));

	return error;
}

static void
axe_watchdog(void *arg)
{
	struct axe_softc *sc = arg;

	mtx_assert(&(sc->sc_mtx), MA_OWNED);

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL, &axe_cfg_tick, 0, 0);

	__callout_reset(&(sc->sc_watchdog), 
			hz, &axe_watchdog, sc);

	mtx_unlock(&(sc->sc_mtx));
	return;
}

/*
 * NOTE: can be called when "ifp" is NULL
 */
static void
axe_cfg_pre_stop(struct axe_softc *sc,
		 struct axe_config_copy *cc, u_int16_t refcount)
{
	struct ifnet *ifp = sc->sc_ifp;

	if (cc) {
	    /* copy the needed configuration */
	    axe_config_copy(sc, cc, refcount);
	}

	/* immediate configuration */

	if (ifp) {
	    /* clear flags */
	    ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | 
				   IFF_DRV_OACTIVE);
	}

	sc->sc_flags &= ~(AXE_FLAG_HL_READY|
			  AXE_FLAG_LL_READY);

	sc->sc_flags |= AXE_FLAG_WAIT_LINK;

	/* stop all the transfers, 
	 * if not already stopped:
	 */
	if (sc->sc_xfer[0]) {
	    usbd_transfer_stop(sc->sc_xfer[0]);
	}
	if (sc->sc_xfer[1]) {
	    usbd_transfer_stop(sc->sc_xfer[1]);
	}
	if (sc->sc_xfer[2]) {
	    usbd_transfer_stop(sc->sc_xfer[2]);
	}
	if (sc->sc_xfer[3]) {
	    usbd_transfer_stop(sc->sc_xfer[3]);
	}
	if (sc->sc_xfer[4]) {
	    usbd_transfer_stop(sc->sc_xfer[4]);
	}
	if (sc->sc_xfer[5]) {
	    usbd_transfer_stop(sc->sc_xfer[5]);
	}
	return;
}

static void
axe_cfg_stop(struct axe_softc *sc,
	     struct axe_config_copy *cc, u_int16_t refcount)
{
	axe_cfg_reset(sc);
	return;
}

/*
 * Stop all chip I/O so that the kernel's probe routines don't
 * get confused by errant DMAs when rebooting.
 */
static int
axe_shutdown(device_t dev)
{
	struct axe_softc *sc = device_get_softc(dev);

	mtx_lock(&(sc->sc_mtx));

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &axe_cfg_pre_stop, 
	   &axe_cfg_stop, 0, 0);

	mtx_unlock(&(sc->sc_mtx));

	return 0;
}
