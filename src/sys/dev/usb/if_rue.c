/*-
 * Copyright (c) 2001-2003, Shunsuke Akiyama <akiyama@FreeBSD.org>.
 * Copyright (c) 1997, 1998, 1999, 2000 Bill Paul <wpaul@ee.columbia.edu>.
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
/*-
 * Copyright (c) 1997, 1998, 1999, 2000
 *	Bill Paul <wpaul@ee.columbia.edu>.  All rights reserved.
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
__FBSDID("$FreeBSD: src/sys/dev/usb/if_rue.c,v 1.30 2006/09/07 00:06:41 imp Exp $");

/*
 * RealTek RTL8150 USB to fast ethernet controller driver.
 * Datasheet is available from
 * ftp://ftp.realtek.com.tw/lancard/data_sheet/8150/.
 */

/*
 * NOTE: all function names beginning like "rue_cfg_" can only
 * be called from within the config thread function !
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/endian.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <net/bpf.h>

#define usbd_config_td_cc rue_config_copy
#define usbd_config_td_softc rue_softc

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/usb/if_ruereg.h>

/* "device miibus" required.  See GENERIC if you get errors here. */
#include "miibus_if.h"

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)	\
  do { if (rue_debug > (n)) {	   \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int rue_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, rue, CTLFLAG_RW, 0, "USB rue");
SYSCTL_INT(_hw_usb_rue, OID_AUTO, debug, CTLFLAG_RW,
	   &rue_debug, 0, "rue debug level");
#else
#define DPRINTF(...)
#endif

/*
 * Various supported device vendors/products.
 */

static struct rue_type rue_devs[] = {
	{ USB_VENDOR_MELCO, USB_PRODUCT_MELCO_LUAKTX },
	{ USB_VENDOR_REALTEK, USB_PRODUCT_REALTEK_USBKR100 },
	{ 0, 0 }
};

/* prototypes */

static device_probe_t rue_probe;
static device_attach_t rue_attach;
static device_detach_t rue_detach;
static device_shutdown_t rue_shutdown;

static void
rue_cfg_do_request(struct rue_softc *sc, usb_device_request_t *req, 
		   void *data);
static void
rue_cfg_read_mem(struct rue_softc *sc, u_int16_t addr, void *buf, 
		 u_int16_t len);
static void
rue_cfg_write_mem(struct rue_softc *sc, u_int16_t addr, void *buf, 
		  u_int16_t len);
static u_int8_t
rue_cfg_csr_read_1(struct rue_softc *sc, u_int16_t reg);

static u_int16_t
rue_cfg_csr_read_2(struct rue_softc *sc, u_int16_t reg);

static void
rue_cfg_csr_write_1(struct rue_softc *sc, u_int16_t reg, u_int8_t val);

static void
rue_cfg_csr_write_2(struct rue_softc *sc, u_int16_t reg, u_int16_t val);

static void
rue_cfg_csr_write_4(struct rue_softc *sc, int reg, u_int32_t val);

static int
rue_cfg_miibus_readreg(device_t dev, int phy, int reg);

static int
rue_cfg_miibus_writereg(device_t dev, int phy, int reg, int data);

static void
rue_cfg_miibus_statchg(device_t dev);

static void
rue_config_copy(struct rue_softc *sc, 
		struct rue_config_copy *cc, u_int16_t refcount);
static void
rue_cfg_promisc_upd(struct rue_softc *sc,
		    struct rue_config_copy *cc, u_int16_t refcount);
static void
rue_cfg_reset(struct rue_softc *sc);

static void
rue_cfg_first_time_setup(struct rue_softc *sc,
			 struct rue_config_copy *cc, u_int16_t refcount);
static void
rue_intr_clear_stall_callback(struct usbd_xfer *xfer);

static void
rue_intr_callback(struct usbd_xfer *xfer);

static void
rue_bulk_read_clear_stall_callback(struct usbd_xfer *xfer);

static void
rue_bulk_read_callback(struct usbd_xfer *xfer);

static void
rue_bulk_write_clear_stall_callback(struct usbd_xfer *xfer);

static void
rue_bulk_write_callback(struct usbd_xfer *xfer);

static void
rue_cfg_tick(struct rue_softc *sc,
	     struct rue_config_copy *cc, u_int16_t refcount);
static void
rue_start_cb(struct ifnet *ifp);

static void
rue_start_transfers(struct rue_softc *sc);

static void
rue_init_cb(void *arg);

static void
rue_cfg_init(struct rue_softc *sc,
	     struct rue_config_copy *cc, u_int16_t refcount);
static int
rue_ifmedia_upd_cb(struct ifnet *ifp);

static void
rue_cfg_ifmedia_upd(struct rue_softc *sc,
		    struct rue_config_copy *cc, u_int16_t refcount);
static void
rue_ifmedia_sts_cb(struct ifnet *ifp, struct ifmediareq *ifmr);

static int
rue_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data);

static void
rue_watchdog(void *arg);

static void
rue_cfg_stop(struct rue_softc *sc,
	     struct rue_config_copy *cc, u_int16_t refcount);

static const struct usbd_config rue_config[RUE_ENDPT_MAX] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = MCLBYTES,
      .flags     = (USBD_USE_DMA|USBD_FORCE_SHORT_XFER),
      .callback  = &rue_bulk_write_callback,
      .timeout   = 10000, /* 10 seconds */
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = (MCLBYTES + 4),
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &rue_bulk_read_callback,
      .timeout   = 0, /* no timeout */
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &rue_bulk_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &rue_bulk_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [4] = {
      .type      = UE_INTERRUPT,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .flags     = USBD_SHORT_XFER_OK,
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &rue_intr_callback,
    },

    [5] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &rue_intr_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static device_method_t rue_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, rue_probe),
	DEVMETHOD(device_attach, rue_attach),
	DEVMETHOD(device_detach, rue_detach),
	DEVMETHOD(device_shutdown, rue_shutdown),

	/* Bus interface */
	DEVMETHOD(bus_print_child, bus_generic_print_child),
	DEVMETHOD(bus_driver_added, bus_generic_driver_added),

	/* MII interface */
	DEVMETHOD(miibus_readreg, rue_cfg_miibus_readreg),
	DEVMETHOD(miibus_writereg, rue_cfg_miibus_writereg),
	DEVMETHOD(miibus_statchg, rue_cfg_miibus_statchg),

	{ 0, 0 }
};

static driver_t rue_driver = {
	.name    = "rue",
	.methods = rue_methods,
	.size    = sizeof(struct rue_softc),
};

static devclass_t rue_devclass;

DRIVER_MODULE(rue, uhub, rue_driver, rue_devclass, usbd_driver_load, 0);
DRIVER_MODULE(miibus, rue, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(rue, usb, 1, 1, 1);
MODULE_DEPEND(rue, ether, 1, 1, 1);
MODULE_DEPEND(rue, miibus, 1, 1, 1);

static void
rue_cfg_do_request(struct rue_softc *sc, usb_device_request_t *req, 
		   void *data)
{
	u_int16_t length;
	usbd_status err;

	if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto error;
	}

	err = usbd_do_request_flags_mtx(sc->sc_udev, &(sc->sc_mtx), req, 
					data, 0, NULL, 1000);

	if (err) {

	    DPRINTF(sc, 0, "device request failed, err=%s "
		    "(ignored)\n", usbd_errstr(err));

	error:
	    length = UGETW(req->wLength);

	    if ((req->bmRequestType & UT_READ) && length) {
	        bzero(data, length);
	    }
	}
	return;
}

#define RUE_CFG_SETBIT(sc, reg, x) \
	rue_cfg_csr_write_1(sc, reg, rue_cfg_csr_read_1(sc, reg) | (x))

#define RUE_CFG_CLRBIT(sc, reg, x) \
	rue_cfg_csr_write_1(sc, reg, rue_cfg_csr_read_1(sc, reg) & ~(x))

static void
rue_cfg_read_mem(struct rue_softc *sc, u_int16_t addr, void *buf, 
		 u_int16_t len)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = UR_SET_ADDRESS;
	USETW(req.wValue, addr);
	USETW(req.wIndex, 0);
	USETW(req.wLength, len);

	rue_cfg_do_request(sc, &req, buf);
	return;
}

static void
rue_cfg_write_mem(struct rue_softc *sc, u_int16_t addr, void *buf, 
		  u_int16_t len)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = UR_SET_ADDRESS;
	USETW(req.wValue, addr);
	USETW(req.wIndex, 0);
	USETW(req.wLength, len);

	rue_cfg_do_request(sc, &req, buf);
	return;
}

static u_int8_t
rue_cfg_csr_read_1(struct rue_softc *sc, u_int16_t reg)
{
	u_int8_t val;
	rue_cfg_read_mem(sc, reg, &val, 1);
	return val;
}

static u_int16_t
rue_cfg_csr_read_2(struct rue_softc *sc, u_int16_t reg)
{
	u_int8_t val[2];
	rue_cfg_read_mem(sc, reg, &val, 2);
	return UGETW(val);
}

static void
rue_cfg_csr_write_1(struct rue_softc *sc, u_int16_t reg, u_int8_t val)
{
	rue_cfg_write_mem(sc, reg, &val, 1);
	return;
}

static void
rue_cfg_csr_write_2(struct rue_softc *sc, u_int16_t reg, u_int16_t val)
{
	u_int8_t temp[2];
	USETW(temp, val);
	rue_cfg_write_mem(sc, reg, &temp, 2);
	return;
}

static void
rue_cfg_csr_write_4(struct rue_softc *sc, int reg, u_int32_t val)
{
	u_int8_t temp[4];
	USETDW(temp, val);
	rue_cfg_write_mem(sc, reg, &temp, 4);
	return;
}

static int
rue_cfg_miibus_readreg(device_t dev, int phy, int reg)
{
	struct rue_softc * sc = device_get_softc(dev);
	u_int16_t rval;
	u_int16_t ruereg;

	if (phy != 0) {		/* RTL8150 supports PHY == 0, only */
	    return 0;
	}

	mtx_lock(&(sc->sc_mtx)); /* XXX */

	switch (reg) {
	case MII_BMCR:
		ruereg = RUE_BMCR;
		break;
	case MII_BMSR:
		ruereg = RUE_BMSR;
		break;
	case MII_ANAR:
		ruereg = RUE_ANAR;
		break;
	case MII_ANER:
		ruereg = RUE_AER;
		break;
	case MII_ANLPAR:
		ruereg = RUE_ANLP;
		break;
	case MII_PHYIDR1:
	case MII_PHYIDR2:
		rval = 0;
		goto done;
	default:
		if ((RUE_REG_MIN <= reg) && (reg <= RUE_REG_MAX)) {
		    rval = rue_cfg_csr_read_1(sc, reg);
		    goto done;
		}
		printf("rue%d: bad phy register\n", sc->sc_unit);
		rval = 0;
		goto done;
	}

	rval = rue_cfg_csr_read_2(sc, ruereg);
 done:
	mtx_unlock(&(sc->sc_mtx)); /* XXX */

	return rval;
}

static int
rue_cfg_miibus_writereg(device_t dev, int phy, int reg, int data)
{
	struct rue_softc * sc = device_get_softc(dev);
	u_int16_t ruereg;

	if (phy != 0) {		/* RTL8150 supports PHY == 0, only */
	    return 0;
	}

	mtx_lock(&(sc->sc_mtx)); /* XXX */

	switch (reg) {
	case MII_BMCR:
		ruereg = RUE_BMCR;
		break;
	case MII_BMSR:
		ruereg = RUE_BMSR;
		break;
	case MII_ANAR:
		ruereg = RUE_ANAR;
		break;
	case MII_ANER:
		ruereg = RUE_AER;
		break;
	case MII_ANLPAR:
		ruereg = RUE_ANLP;
		break;
	case MII_PHYIDR1:
	case MII_PHYIDR2:
		goto done;
	default:
		if ((RUE_REG_MIN <= reg) && (reg <= RUE_REG_MAX)) {
		    rue_cfg_csr_write_1(sc, reg, data);
		    goto done;
		}
		printf("%s: bad phy register\n", 
		       sc->sc_name);
		goto done;
	}
	rue_cfg_csr_write_2(sc, ruereg, data);
 done:
	mtx_unlock(&(sc->sc_mtx)); /* XXX */

	return 0;
}

static void
rue_cfg_miibus_statchg(device_t dev)
{
	/*
	 * When the code below is enabled the card starts doing weird
	 * things after link going from UP to DOWN and back UP.
	 *
	 * Looks like some of register writes below messes up PHY
	 * interface.
	 *
	 * No visible regressions were found after commenting this code
	 * out, so that disable it for good.
	 */
#if 0
	struct rue_softc * sc = device_get_softc(dev);
	struct mii_data * mii = GET_MII(sc);
	u_int16_t bmcr;

	mtx_lock(&(sc->sc_mtx)); /* XXX */

	RUE_CFG_CLRBIT(sc, RUE_CR, (RUE_CR_RE | RUE_CR_TE));

	bmcr = rue_cfg_csr_read_2(sc, RUE_BMCR);

	if (IFM_SUBTYPE(mii->mii_media_active) == IFM_100_TX)
		bmcr |= RUE_BMCR_SPD_SET;
	else
		bmcr &= ~RUE_BMCR_SPD_SET;

	if ((mii->mii_media_active & IFM_GMASK) == IFM_FDX)
		bmcr |= RUE_BMCR_DUPLEX;
	else
		bmcr &= ~RUE_BMCR_DUPLEX;

	rue_cfg_csr_write_2(sc, RUE_BMCR, bmcr);

	RUE_CFG_SETBIT(sc, RUE_CR, (RUE_CR_RE | RUE_CR_TE));

	mtx_unlock(&(sc->sc_mtx)); /* XXX */
#endif
	return;
}

static void
rue_config_copy(struct rue_softc *sc, 
		struct rue_config_copy *cc, u_int16_t refcount)
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
	    TAILQ_FOREACH (ifma, &ifp->if_multiaddrs, ifma_link)
	    {
	        if (ifma->ifma_addr->sa_family != AF_LINK) {
		    continue;
		}
		h = ether_crc32_be(LLADDR((struct sockaddr_dl *)
		        ifma->ifma_addr), ETHER_ADDR_LEN) >> 26;
		if (h < 32)
			cc->if_hashes[0] |= (1 << h);
		else
			cc->if_hashes[1] |= (1 << (h - 32));
	    }
	    IF_ADDR_UNLOCK(ifp);
	}
	return;
}

/*
 * Program the 64-bit multicast hash filter.
 */
static void
rue_cfg_promisc_upd(struct rue_softc *sc,
		    struct rue_config_copy *cc, u_int16_t refcount)
{
	u_int16_t rxcfg;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	rxcfg = rue_cfg_csr_read_2(sc, RUE_RCR);

	if ((cc->if_flags & IFF_ALLMULTI) || 
	    (cc->if_flags & IFF_PROMISC)) {
	    rxcfg |= (RUE_RCR_AAM | RUE_RCR_AAP);
	    rxcfg &= ~RUE_RCR_AM;
	    rue_cfg_csr_write_2(sc, RUE_RCR, rxcfg);
	    rue_cfg_csr_write_4(sc, RUE_MAR0, 0xFFFFFFFF);
	    rue_cfg_csr_write_4(sc, RUE_MAR4, 0xFFFFFFFF);
	    return;
	}

	/* first, zero all the existing hash bits */
	rue_cfg_csr_write_4(sc, RUE_MAR0, 0);
	rue_cfg_csr_write_4(sc, RUE_MAR4, 0);

	if (cc->if_hashes[0] || 
	    cc->if_hashes[1])
		rxcfg |= RUE_RCR_AM;
	else
		rxcfg &= ~RUE_RCR_AM;

	rxcfg &= ~(RUE_RCR_AAM | RUE_RCR_AAP);

	rue_cfg_csr_write_2(sc, RUE_RCR, rxcfg);
	rue_cfg_csr_write_4(sc, RUE_MAR0, cc->if_hashes[0]);
	rue_cfg_csr_write_4(sc, RUE_MAR4, cc->if_hashes[1]);
	return;
}

static void
rue_cfg_reset(struct rue_softc *sc)
{
	usbd_status err;
	u_int16_t to;

	rue_cfg_csr_write_1(sc, RUE_CR, RUE_CR_SOFT_RST);

	for (to = 0; ; to++) {

	    if (to < RUE_TIMEOUT) {

	        err = usbd_config_td_sleep(&(sc->sc_config_td), hz/100);

		if (err) {
		    break;
		}

		if (!(rue_cfg_csr_read_1(sc, RUE_CR) & RUE_CR_SOFT_RST)) {
	            break;
		}
	    } else {
	        printf("%s: reset timeout!\n", 
		       sc->sc_name);
		break;
	    }
	}

	err = usbd_config_td_sleep(&(sc->sc_config_td), hz/100);
	return;
}

/*
 * Probe for a RTL8150 chip.
 */
static int
rue_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct rue_type	*t;

	if (uaa->iface != NULL) {
	    return UMATCH_NONE;
	}

	t = rue_devs;
	while (t->rue_vid) {
	    if ((uaa->vendor == t->rue_vid) &&
		(uaa->product == t->rue_did)) {
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
rue_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct rue_softc *sc = device_get_softc(dev);
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

	mtx_init(&(sc->sc_mtx), "rue lock", NULL, MTX_DEF | MTX_RECURSE);

	__callout_init_mtx(&(sc->sc_watchdog),
			   &(sc->sc_mtx), CALLOUT_RETURNUNLOCKED);

	error = usbd_set_config_no(uaa->device, RUE_CONFIG_NO, 0);

	if (error) {
		device_printf(dev, "setting config "
			      "number failed!\n");
		goto detach;
	}

	error = usbd_transfer_setup(uaa->device, RUE_IFACE_IDX, 
				    sc->sc_xfer, rue_config, RUE_ENDPT_MAX,
				    sc, &(sc->sc_mtx));
	if (error) {
		device_printf(dev, "allocating USB "
			      "transfers failed!\n");
		goto detach;
	}

	error = usbd_config_td_setup(&(sc->sc_config_td), sc, &(sc->sc_mtx),
				     &rue_config_copy, NULL,
				     sizeof(struct rue_config_copy), 16);
	if (error) {
		device_printf(dev, "could not setup config "
			      "thread!\n");
		goto detach;
	}

	mtx_lock(&(sc->sc_mtx));

	sc->sc_flags |= RUE_FLAG_WAIT_LINK;

	/* start setup */

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &rue_cfg_first_time_setup, 0);

	/* start watchdog (will exit mutex) */

	rue_watchdog(sc);

	return 0; /* success */

 detach:
	rue_detach(dev);
	return ENXIO; /* failure */
}

static void
rue_cfg_first_time_setup(struct rue_softc *sc,
			 struct rue_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp;
	int error;
	u_int8_t eaddr[min(ETHER_ADDR_LEN,6)];

	if (cc == NULL) {
	    return;
	}

	/* reset the adapter */
	rue_cfg_reset(sc);

	/* get station address from the EEPROM */
	rue_cfg_read_mem(sc, RUE_EEPROM_IDR0,
			 eaddr, ETHER_ADDR_LEN);

	mtx_unlock(&(sc->sc_mtx));

	ifp = if_alloc(IFT_ETHER);

	mtx_lock(&(sc->sc_mtx));

	if (ifp == NULL) {
	    printf("%s: could not if_alloc()\n",
		   sc->sc_name);
	    goto done;
	}

	ifp->if_softc = sc;
	if_initname(ifp, "rue", sc->sc_unit);
	ifp->if_mtu = ETHERMTU;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = rue_ioctl_cb;
	ifp->if_start = rue_start_cb;
	ifp->if_watchdog = NULL;
	ifp->if_init = rue_init_cb;
	ifp->if_snd.ifq_maxlen = IFQ_MAXLEN;

	/* XXX need Giant when accessing
	 * the device structures !
	 */

	mtx_unlock(&(sc->sc_mtx));

	mtx_lock(&Giant);

	/* MII setup */
	error = mii_phy_probe(sc->sc_dev, &(sc->sc_miibus),
			      &rue_ifmedia_upd_cb, 
			      &rue_ifmedia_sts_cb);
	mtx_unlock(&Giant);

	mtx_lock(&(sc->sc_mtx));

	if (error) {
	    printf("%s: MII without any PHY!\n", 
		   sc->sc_name);
	    if_free(ifp);
	    goto done;
	}

	sc->sc_ifp = ifp;

	/*
	 * Call MI attach routine.
	 */

	ether_ifattach(ifp, eaddr);

 done:
	return;
}

static int
rue_detach(device_t dev)
{
	struct rue_softc * sc = device_get_softc(dev);
	struct ifnet * ifp;

	mtx_lock(&(sc->sc_mtx));

	usbd_config_td_stop(&(sc->sc_config_td));

	__callout_stop(&sc->sc_watchdog);

	rue_cfg_stop(sc, NULL, 0);

	ifp = sc->sc_ifp;

	mtx_unlock(&(sc->sc_mtx));

	/* get rid of any late children */
	bus_generic_detach(dev);

	if (ifp) {
	    ether_ifdetach(ifp);
	    if_free(ifp);
	}

	usbd_transfer_unsetup(sc->sc_xfer, RUE_ENDPT_MAX);

	usbd_config_td_unsetup(&(sc->sc_config_td));

	__callout_drain(&(sc->sc_watchdog));

	mtx_destroy(&(sc->sc_mtx));

	return 0;
}

static void
rue_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct rue_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
        return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~RUE_FLAG_INTR_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~RUE_FLAG_INTR_STALL;
	DPRINTF(sc, 0, "interrupt read pipe stopped\n");
	return;
}

static void
rue_intr_callback(struct usbd_xfer *xfer)
{
	struct rue_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct rue_intrpkt *p = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:

        if (ifp && (ifp->if_drv_flags & IFF_DRV_RUNNING) &&
            (xfer->actlen >= sizeof(*p))) {

	    ifp->if_ierrors += p->rue_rxlost_cnt;
	    ifp->if_ierrors += p->rue_crcerr_cnt;
	    ifp->if_collisions += p->rue_col_cnt;
        }

 tr_setup:
	if (sc->sc_flags & RUE_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[5]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* start clear stall */
	    sc->sc_flags |= RUE_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[5]);
	}
	return;
}

static void
rue_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct rue_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~RUE_FLAG_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~RUE_FLAG_READ_STALL;
	DPRINTF(sc, 0, "bulk read pipe stopped\n");
	return;
}

static void
rue_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct rue_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	u_int16_t status;
	struct mbuf *m;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= RUE_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	DPRINTF(sc, 0, "bulk read error, %s\n",
		usbd_errstr(xfer->error));
	return;

 tr_transferred:

	if (xfer->actlen < 4) {
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	usbd_copy_out(&(xfer->buf_data), xfer->actlen - 4, 
		      &status, sizeof(status));

	status = le16toh(status);

	/* check recieve packet was valid or not */

	if ((status & RUE_RXSTAT_VALID) == 0) {
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	xfer->actlen -= 4;

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

	if (sc->sc_flags & RUE_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
rue_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct rue_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~RUE_FLAG_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~RUE_FLAG_WRITE_STALL;
	DPRINTF(sc, 0, "bulk write pipe stopped\n");
	return;
}

static void
rue_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct rue_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 10, "transfer error, %s\n",
		 usbd_errstr(xfer->error));

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= RUE_FLAG_WRITE_STALL;
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

	if (sc->sc_flags & RUE_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    goto done;
	}

	if (sc->sc_flags & RUE_FLAG_WAIT_LINK) {
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
	 * This is an undocumented behavior.
	 * RTL8150 chip doesn't send frame length smaller than
	 * RUE_MIN_FRAMELEN (60) byte packet.
	 */
	if (xfer->length < RUE_MIN_FRAMELEN) {
	    usbd_bzero(&(xfer->buf_data), xfer->length,
		       RUE_MIN_FRAMELEN - xfer->length);
	    xfer->length = RUE_MIN_FRAMELEN;
	}

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
rue_cfg_tick(struct rue_softc *sc,
	     struct rue_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp = sc->sc_ifp;
	struct mii_data * mii = GET_MII(sc);

	if ((cc == NULL) ||
	    (ifp == NULL) || 
	    (mii == NULL)) {
	    /* not ready */
	    return;
	}

	mii_tick(mii);

	mii_pollstat(mii);

	if ((sc->sc_flags & RUE_FLAG_WAIT_LINK) &&
	    (mii->mii_media_status & IFM_ACTIVE) &&
	    (IFM_SUBTYPE(mii->mii_media_active) != IFM_NONE)) {
	    sc->sc_flags &= ~RUE_FLAG_WAIT_LINK;
	}

	sc->sc_media_active = mii->mii_media_active;
	sc->sc_media_status = mii->mii_media_status;

	/* start stopped transfers, if any */

	rue_start_transfers(sc);

	return;
}

static void
rue_start_cb(struct ifnet *ifp)
{
	struct rue_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	rue_start_transfers(sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
rue_start_transfers(struct rue_softc *sc)
{
	if ((sc->sc_flags & RUE_FLAG_LL_READY) &&
	    (sc->sc_flags & RUE_FLAG_HL_READY)) {

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
rue_init_cb(void *arg)
{
	struct rue_softc *sc = arg;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &rue_cfg_init, 0);
	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
rue_cfg_init(struct rue_softc *sc,
	     struct rue_config_copy *cc, u_int16_t refcount)
{
	struct mii_data *mii = GET_MII(sc);
	u_int16_t rxcfg;

	if (cc == NULL) {

	    /* immediate configuration */

	    struct ifnet *ifp = sc->sc_ifp;

	    rue_cfg_stop(sc, NULL, 0);

	    ifp->if_drv_flags |= IFF_DRV_RUNNING;
	    ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	    sc->sc_flags |= RUE_FLAG_HL_READY;

	    return;
	}

	/*
	 * Cancel pending I/O
	 */

	rue_cfg_stop(sc, cc, 0);

	/* set MAC address */

	rue_cfg_write_mem(sc, RUE_IDR0, cc->if_lladdr, ETHER_ADDR_LEN);

	/*
	 * Set the initial TX and RX configuration.
	 */
	rue_cfg_csr_write_1(sc, RUE_TCR, RUE_TCR_CONFIG);

	rxcfg = RUE_RCR_CONFIG;

	/* Set capture broadcast bit to capture broadcast frames. */
	if (cc->if_flags & IFF_BROADCAST)
		rxcfg |= RUE_RCR_AB;
	else
		rxcfg &= ~RUE_RCR_AB;

	rue_cfg_csr_write_2(sc, RUE_RCR, rxcfg);

	/* Load the multicast filter */
	rue_cfg_promisc_upd(sc, cc, 0);

	/* Enable RX and TX */
	rue_cfg_csr_write_1(sc, RUE_CR, (RUE_CR_TE | RUE_CR_RE | RUE_CR_EP3CLREN));

	mii_mediachg(mii);

	sc->sc_flags |= (RUE_FLAG_READ_STALL|
			 RUE_FLAG_WRITE_STALL|
			 RUE_FLAG_LL_READY);

	rue_start_transfers(sc);

	return;
}

/*
 * Set media options.
 */
static int
rue_ifmedia_upd_cb(struct ifnet *ifp)
{
	struct rue_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &rue_cfg_ifmedia_upd, 0);
	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

static void
rue_cfg_ifmedia_upd(struct rue_softc *sc,
		    struct rue_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp = sc->sc_ifp;
	struct mii_data * mii = GET_MII(sc);

	if ((cc == NULL) ||
	    (ifp == NULL) || 
	    (mii == NULL)) {
	    /* not ready */
	    return;
	}

	sc->sc_flags |= RUE_FLAG_WAIT_LINK;

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
rue_ifmedia_sts_cb(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct rue_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	ifmr->ifm_active = sc->sc_media_active;
	ifmr->ifm_status = sc->sc_media_status;

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static int
rue_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct rue_softc * sc = ifp->if_softc;
	struct mii_data	* mii;
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	switch (command) {
	case SIOCSIFFLAGS:
	    if (ifp->if_flags & IFF_UP) {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &rue_cfg_promisc_upd, 0);
		} else {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &rue_cfg_init, 0); 
		}
	    } else {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &rue_cfg_stop, 0);
		}
	    }
	    break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &rue_cfg_promisc_upd, 0);
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
rue_watchdog(void *arg)
{
	struct rue_softc *sc = arg;

	mtx_assert(&(sc->sc_mtx), MA_OWNED);

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &rue_cfg_tick, 0);

	__callout_reset(&(sc->sc_watchdog), 
			hz, &rue_watchdog, sc);

	mtx_unlock(&(sc->sc_mtx));
	return;
}

/*
 * NOTE: can be called when "ifp" is NULL
 */
static void
rue_cfg_stop(struct rue_softc *sc,
	     struct rue_config_copy *cc, u_int16_t refcount)
{
	if (cc == NULL) {

	    /* immediate configuration */

	    struct ifnet *ifp = sc->sc_ifp;

	    if (ifp) {
	        /* clear flags */
		ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | 
				       IFF_DRV_OACTIVE);
	    }

	    sc->sc_flags &= ~(RUE_FLAG_HL_READY|
			      RUE_FLAG_LL_READY);

	    sc->sc_flags |= RUE_FLAG_WAIT_LINK;

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

	rue_cfg_csr_write_1(sc, RUE_CR, 0x00);

	rue_cfg_reset(sc);
	return;
}

/*
 * Stop all chip I/O so that the kernel's probe routines don't
 * get confused by errant DMAs when rebooting.
 */
static int
rue_shutdown(device_t dev)
{
	struct rue_softc * sc = device_get_softc(dev);

	mtx_lock(&(sc->sc_mtx));

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &rue_cfg_stop, 0);

	mtx_unlock(&(sc->sc_mtx));

	return 0;
}
