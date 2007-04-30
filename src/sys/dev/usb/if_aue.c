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
__FBSDID("$FreeBSD: src/sys/dev/usb/if_aue.c,v 1.107 2007/03/31 23:23:41 jhb Exp $");

/*
 * ADMtek AN986 Pegasus and AN8511 Pegasus II USB to ethernet driver.
 * Datasheet is available from http://www.admtek.com.tw.
 *
 * Written by Bill Paul <wpaul@ee.columbia.edu>
 * Electrical Engineering Department
 * Columbia University, New York City
 */

/*
 * The Pegasus chip uses four USB "endpoints" to provide 10/100 ethernet
 * support: the control endpoint for reading/writing registers, burst
 * read endpoint for packet reception, burst write for packet transmission
 * and one for "interrupts." The chip uses the same RX filter scheme
 * as the other ADMtek ethernet parts: one perfect filter entry for the
 * the station address and a 64-bit multicast hash table. The chip supports
 * both MII and HomePNA attachments.
 *
 * Since the maximum data transfer speed of USB is supposed to be 12Mbps,
 * you're never really going to get 100Mbps speeds from this device. I
 * think the idea is to allow the device to connect to 10 or 100Mbps
 * networks, not necessarily to provide 100Mbps performance. Also, since
 * the controller uses an external PHY chip, it's possible that board
 * designers might simply choose a 10Mbps PHY.
 *
 * Registers are accessed using usbd_do_request(). Packet transfers are
 * done using usbd_transfer() and friends.
 */

/*
 * NOTE: all function names beginning like "aue_cfg_" can only
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

#define usbd_config_td_cc aue_config_copy
#define usbd_config_td_softc aue_softc

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/usb/if_auereg.h>

/* "device miibus" required.  See GENERIC if you get errors here. */
#include "miibus_if.h"

MODULE_DEPEND(aue, usb, 1, 1, 1);
MODULE_DEPEND(aue, ether, 1, 1, 1);
MODULE_DEPEND(aue, miibus, 1, 1, 1);

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)	\
  do { if (aue_debug > (n)) {	     \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int aue_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, aue, CTLFLAG_RW, 0, "USB aue");
SYSCTL_INT(_hw_usb_aue, OID_AUTO, debug, CTLFLAG_RW, &aue_debug, 0,
	   "aue debug level");
#else
#define DPRINTF(...)
#endif

/*
 * Various supported device vendors/products.
 */
struct aue_type {
	struct usb_devno	aue_dev;
	u_int16_t		aue_flags;
};

static const struct aue_type aue_devs[] = {
 {{ USB_VENDOR_3COM,		USB_PRODUCT_3COM_3C460B},         AUE_FLAG_PII },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX1},	  AUE_FLAG_PNA|AUE_FLAG_PII },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX2},	  AUE_FLAG_PII },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_UFE1000},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX4},	  AUE_FLAG_PNA },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX5},	  AUE_FLAG_PNA },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX6},	  AUE_FLAG_PII },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX7},	  AUE_FLAG_PII },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX8},	  AUE_FLAG_PII },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX9},	  AUE_FLAG_PNA },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_XX10},	  0 },
 {{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_DSB650TX_PNA}, 0 },
 {{ USB_VENDOR_ACCTON,		USB_PRODUCT_ACCTON_USB320_EC},	  0 },
 {{ USB_VENDOR_ACCTON,		USB_PRODUCT_ACCTON_SS1001},	  AUE_FLAG_PII },
 {{ USB_VENDOR_ADMTEK,		USB_PRODUCT_ADMTEK_PEGASUS},	  AUE_FLAG_PNA },
 {{ USB_VENDOR_ADMTEK,		USB_PRODUCT_ADMTEK_PEGASUSII},	  AUE_FLAG_PII },
 {{ USB_VENDOR_ADMTEK,		USB_PRODUCT_ADMTEK_PEGASUSII_2},  AUE_FLAG_PII },
 {{ USB_VENDOR_BELKIN,		USB_PRODUCT_BELKIN_USB2LAN},	  AUE_FLAG_PII },
 {{ USB_VENDOR_BILLIONTON,	USB_PRODUCT_BILLIONTON_USB100},	  0 },
 {{ USB_VENDOR_BILLIONTON,	USB_PRODUCT_BILLIONTON_USBLP100}, AUE_FLAG_PNA },
 {{ USB_VENDOR_BILLIONTON,	USB_PRODUCT_BILLIONTON_USBEL100}, 0 },
 {{ USB_VENDOR_BILLIONTON,	USB_PRODUCT_BILLIONTON_USBE100},  AUE_FLAG_PII },
 {{ USB_VENDOR_COREGA,		USB_PRODUCT_COREGA_FETHER_USB_TX}, 0 },
 {{ USB_VENDOR_COREGA,		USB_PRODUCT_COREGA_FETHER_USB_TXS},AUE_FLAG_PII },
 {{ USB_VENDOR_DLINK,		USB_PRODUCT_DLINK_DSB650TX4},	  AUE_FLAG_LSYS|AUE_FLAG_PII },
 {{ USB_VENDOR_DLINK,		USB_PRODUCT_DLINK_DSB650TX1},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_DLINK,		USB_PRODUCT_DLINK_DSB650TX},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_DLINK,		USB_PRODUCT_DLINK_DSB650TX_PNA},  AUE_FLAG_PNA },
 {{ USB_VENDOR_DLINK,		USB_PRODUCT_DLINK_DSB650TX3},	  AUE_FLAG_LSYS|AUE_FLAG_PII },
 {{ USB_VENDOR_DLINK,		USB_PRODUCT_DLINK_DSB650TX2},	  AUE_FLAG_LSYS|AUE_FLAG_PII },
 {{ USB_VENDOR_DLINK,		USB_PRODUCT_DLINK_DSB650},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_ELECOM,		USB_PRODUCT_ELECOM_LDUSBTX0},	  0 },
 {{ USB_VENDOR_ELECOM,		USB_PRODUCT_ELECOM_LDUSBTX1},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_ELECOM,		USB_PRODUCT_ELECOM_LDUSBTX2},	  0 },
 {{ USB_VENDOR_ELECOM,		USB_PRODUCT_ELECOM_LDUSBTX3},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_ELECOM,		USB_PRODUCT_ELECOM_LDUSBLTX},	  AUE_FLAG_PII },
 {{ USB_VENDOR_ELSA,		USB_PRODUCT_ELSA_USB2ETHERNET},	  0 },
 {{ USB_VENDOR_HAWKING,		USB_PRODUCT_HAWKING_UF100},       AUE_FLAG_PII },
 {{ USB_VENDOR_HP,		USB_PRODUCT_HP_HN210E},           AUE_FLAG_PII },
 {{ USB_VENDOR_IODATA,		USB_PRODUCT_IODATA_USBETTX},	  0 },
 {{ USB_VENDOR_IODATA,		USB_PRODUCT_IODATA_USBETTXS},	  AUE_FLAG_PII },
 {{ USB_VENDOR_KINGSTON,	USB_PRODUCT_KINGSTON_KNU101TX},   0 },
 {{ USB_VENDOR_LINKSYS,		USB_PRODUCT_LINKSYS_USB10TX1},	  AUE_FLAG_LSYS|AUE_FLAG_PII },
 {{ USB_VENDOR_LINKSYS,		USB_PRODUCT_LINKSYS_USB10T},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_LINKSYS,		USB_PRODUCT_LINKSYS_USB100TX},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_LINKSYS,		USB_PRODUCT_LINKSYS_USB100H1},	  AUE_FLAG_LSYS|AUE_FLAG_PNA },
 {{ USB_VENDOR_LINKSYS,		USB_PRODUCT_LINKSYS_USB10TA},	  AUE_FLAG_LSYS },
 {{ USB_VENDOR_LINKSYS,		USB_PRODUCT_LINKSYS_USB10TX2},	  AUE_FLAG_LSYS|AUE_FLAG_PII },
 {{ USB_VENDOR_MICROSOFT,	USB_PRODUCT_MICROSOFT_MN110},	  AUE_FLAG_PII },
 {{ USB_VENDOR_MELCO,		USB_PRODUCT_MELCO_LUATX1},	  0 },
 {{ USB_VENDOR_MELCO,		USB_PRODUCT_MELCO_LUATX5},	  0 },
 {{ USB_VENDOR_MELCO,		USB_PRODUCT_MELCO_LUA2TX5},	  AUE_FLAG_PII },
 {{ USB_VENDOR_SIEMENS,		USB_PRODUCT_SIEMENS_SPEEDSTREAM}, AUE_FLAG_PII },
 {{ USB_VENDOR_SMARTBRIDGES,	USB_PRODUCT_SMARTBRIDGES_SMARTNIC},AUE_FLAG_PII },
 {{ USB_VENDOR_SMC,		USB_PRODUCT_SMC_2202USB},	  0 },
 {{ USB_VENDOR_SMC,		USB_PRODUCT_SMC_2206USB},	  AUE_FLAG_PII },
 {{ USB_VENDOR_SOHOWARE,	USB_PRODUCT_SOHOWARE_NUB100},	  0 },
 {{ USB_VENDOR_NETGEAR,		USB_PRODUCT_NETGEAR_FA101},	  AUE_FLAG_PII },
};
#define aue_lookup(v, p) ((const struct aue_type *)usb_lookup(aue_devs, v, p))

/* prototypes */

static device_probe_t aue_probe;
static device_attach_t aue_attach;
static device_detach_t aue_detach;
static device_shutdown_t aue_shutdown;

static usbd_callback_t aue_intr_clear_stall_callback;
static usbd_callback_t aue_intr_callback;
static usbd_callback_t aue_bulk_read_clear_stall_callback;
static usbd_callback_t aue_bulk_read_callback;
static usbd_callback_t aue_bulk_write_clear_stall_callback;
static usbd_callback_t aue_bulk_write_callback;

static void
aue_cfg_do_request(struct aue_softc *sc, usb_device_request_t *req, 
		   void *data);
static u_int8_t
aue_cfg_csr_read_1(struct aue_softc *sc, u_int16_t reg);

static u_int16_t
aue_cfg_csr_read_2(struct aue_softc *sc, u_int16_t reg);

static void
aue_cfg_csr_write_1(struct aue_softc *sc, u_int16_t reg, u_int8_t val);

static void
aue_cfg_csr_write_2(struct aue_softc *sc, u_int16_t reg, u_int16_t val);

static void
aue_cfg_eeprom_getword(struct aue_softc *sc, u_int8_t addr, 
		       u_int8_t *dest);
static void
aue_cfg_read_eeprom(struct aue_softc *sc, u_int8_t *dest, 
		    u_int16_t off, u_int16_t len);

static miibus_readreg_t aue_cfg_miibus_readreg;
static miibus_writereg_t aue_cfg_miibus_writereg;
static miibus_statchg_t aue_cfg_miibus_statchg;

static usbd_config_td_command_t aue_cfg_setmulti;
static usbd_config_td_command_t aue_cfg_first_time_setup;
static usbd_config_td_command_t aue_config_copy;
static usbd_config_td_command_t aue_cfg_tick;
static usbd_config_td_command_t aue_cfg_pre_init;
static usbd_config_td_command_t aue_cfg_init;
static usbd_config_td_command_t aue_cfg_promisc_upd;
static usbd_config_td_command_t aue_cfg_ifmedia_upd;
static usbd_config_td_command_t aue_cfg_pre_stop;
static usbd_config_td_command_t aue_cfg_stop;

static void
aue_cfg_reset_pegasus_II(struct aue_softc *sc);

static void
aue_cfg_reset(struct aue_softc *sc);

static void
aue_start_cb(struct ifnet *ifp);

static void
aue_init_cb(void *arg);

static void
aue_start_transfers(struct aue_softc *sc);

static int
aue_ifmedia_upd_cb(struct ifnet *ifp);

static void
aue_ifmedia_sts_cb(struct ifnet *ifp, struct ifmediareq *ifmr);

static int
aue_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data);

static void
aue_watchdog(void *arg);

static const struct usbd_config aue_config[AUE_ENDPT_MAX] = {

    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = (MCLBYTES + 2),
      .flags     = (USBD_USE_DMA|USBD_FORCE_SHORT_XFER),
      .callback  = &aue_bulk_write_callback,
      .timeout   = 10000, /* 10 seconds */
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = (MCLBYTES + 4 + ETHER_CRC_LEN),
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &aue_bulk_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &aue_bulk_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &aue_bulk_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [4] = {
      .type      = UE_INTERRUPT,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .flags     = USBD_SHORT_XFER_OK,
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &aue_intr_callback,
    },

    [5] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &aue_intr_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static device_method_t aue_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aue_probe),
	DEVMETHOD(device_attach,	aue_attach),
	DEVMETHOD(device_detach,	aue_detach),
	DEVMETHOD(device_shutdown,	aue_shutdown),

	/* bus interface */
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_driver_added,	bus_generic_driver_added),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	aue_cfg_miibus_readreg),
	DEVMETHOD(miibus_writereg,	aue_cfg_miibus_writereg),
	DEVMETHOD(miibus_statchg,	aue_cfg_miibus_statchg),

	{ 0, 0 }
};

static driver_t aue_driver = {
	.name    = "aue",
	.methods = aue_methods,
	.size    = sizeof(struct aue_softc)
};

static devclass_t aue_devclass;

DRIVER_MODULE(aue, uhub, aue_driver, aue_devclass, usbd_driver_load, 0);
DRIVER_MODULE(miibus, aue, miibus_driver, miibus_devclass, 0, 0);

static void
aue_cfg_do_request(struct aue_softc *sc, usb_device_request_t *req, 
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

#define AUE_CFG_SETBIT(sc, reg, x) \
	aue_cfg_csr_write_1(sc, reg, aue_cfg_csr_read_1(sc, reg) | (x))

#define AUE_CFG_CLRBIT(sc, reg, x) \
	aue_cfg_csr_write_1(sc, reg, aue_cfg_csr_read_1(sc, reg) & ~(x))

static u_int8_t
aue_cfg_csr_read_1(struct aue_softc *sc, u_int16_t reg)
{
	usb_device_request_t req;
	u_int8_t val;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = AUE_UR_READREG;
	USETW(req.wValue, 0);
	USETW(req.wIndex, reg);
	USETW(req.wLength, 1);

	aue_cfg_do_request(sc, &req, &val);
	return val;
}

static u_int16_t
aue_cfg_csr_read_2(struct aue_softc *sc, u_int16_t reg)
{
	usb_device_request_t req;
	u_int16_t val;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = AUE_UR_READREG;
	USETW(req.wValue, 0);
	USETW(req.wIndex, reg);
	USETW(req.wLength, 2);

	aue_cfg_do_request(sc, &req, &val);
	return le16toh(val);
}

static void
aue_cfg_csr_write_1(struct aue_softc *sc, u_int16_t reg, u_int8_t val)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AUE_UR_WRITEREG;
	req.wValue[0] = val;
	req.wValue[1] = 0;
	USETW(req.wIndex, reg);
	USETW(req.wLength, 1);

	aue_cfg_do_request(sc, &req, &val);
	return;
}

static void
aue_cfg_csr_write_2(struct aue_softc *sc, u_int16_t reg, u_int16_t val)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AUE_UR_WRITEREG;
	USETW(req.wValue, val);
	USETW(req.wIndex, reg);
	USETW(req.wLength, 2);

	val = htole16(val);

	aue_cfg_do_request(sc, &req, &val);
	return;
}

/*
 * Read a word of data stored in the EEPROM at address 'addr.'
 */
static void
aue_cfg_eeprom_getword(struct aue_softc *sc, u_int8_t addr, 
		       u_int8_t *dest)
{
	u_int16_t i;

	aue_cfg_csr_write_1(sc, AUE_EE_REG, addr);
	aue_cfg_csr_write_1(sc, AUE_EE_CTL, AUE_EECTL_READ);

	for (i = 0; ; i++) {

	    if (i < AUE_TIMEOUT) {

		if (aue_cfg_csr_read_1(sc, AUE_EE_CTL) & AUE_EECTL_DONE) {
		    break;
		}

		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    break;
		}

	    } else {
	        DPRINTF(sc, 0, "EEPROM read timed out!\n");
		break;
	    }
	}

	i = aue_cfg_csr_read_2(sc, AUE_EE_DATA);

	dest[0] = (i & 0xFF);
	dest[1] = (i >> 8);

	return;
}

/*
 * Read a sequence of words from the EEPROM.
 */
static void
aue_cfg_read_eeprom(struct aue_softc *sc, u_int8_t *dest, 
		    u_int16_t off, u_int16_t len)
{
	u_int16_t i;

	for (i = 0; i < len; i++) {
	    aue_cfg_eeprom_getword(sc, off + i, dest + (i * 2));
	}
	return;
}

static int
aue_cfg_miibus_readreg(device_t dev, int phy, int reg)
{
	struct aue_softc *	sc = device_get_softc(dev);
	u_int16_t		i;

	mtx_lock(&(sc->sc_mtx)); /* XXX */

	/*
	 * The Am79C901 HomePNA PHY actually contains
	 * two transceivers: a 1Mbps HomePNA PHY and a
	 * 10Mbps full/half duplex ethernet PHY with
	 * NWAY autoneg. However in the ADMtek adapter,
	 * only the 1Mbps PHY is actually connected to
	 * anything, so we ignore the 10Mbps one. It
	 * happens to be configured for MII address 3,
	 * so we filter that out.
	 */
	if ((sc->sc_vendor == USB_VENDOR_ADMTEK) &&
	    (sc->sc_product == USB_PRODUCT_ADMTEK_PEGASUS)) {

	    if (phy == 3) {
	        i = 0;
		goto done;
	    }

#ifdef notdef
	    if (phy != 1) {
	        i = 0;
		goto done;
	    }
#endif
	}

	aue_cfg_csr_write_1(sc, AUE_PHY_ADDR, phy);
	aue_cfg_csr_write_1(sc, AUE_PHY_CTL, reg | AUE_PHYCTL_READ);

	for (i = 0; ; i++) {

	    if (i < AUE_TIMEOUT) {

	        if (aue_cfg_csr_read_1(sc, AUE_PHY_CTL) & AUE_PHYCTL_DONE) {
		    break;
		}

		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    break;
		}
	    } else {
	        DPRINTF(sc, 0, "MII read timed out\n");
		break;
	    }
	}

	i = aue_cfg_csr_read_2(sc, AUE_PHY_DATA);

 done:
	mtx_unlock(&(sc->sc_mtx)); /* XXX */

	return i;
}

static int
aue_cfg_miibus_writereg(device_t dev, int phy, int reg, int data)
{
	struct aue_softc *	sc = device_get_softc(dev);
	u_int16_t		i;

	if (phy == 3) {
	    return (0);
	}

	mtx_lock(&(sc->sc_mtx)); /* XXX */

	aue_cfg_csr_write_2(sc, AUE_PHY_DATA, data);
	aue_cfg_csr_write_1(sc, AUE_PHY_ADDR, phy);
	aue_cfg_csr_write_1(sc, AUE_PHY_CTL, reg | AUE_PHYCTL_WRITE);

	for (i = 0; ; i++) {

	    if (i < AUE_TIMEOUT) {
	        if (aue_cfg_csr_read_1(sc, AUE_PHY_CTL) & AUE_PHYCTL_DONE) {
		    break;
		}

		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    break;
		}
	    } else {
	        DPRINTF(sc, 0, "MII write timed out\n");
		break;
	    }
	}

	mtx_unlock(&(sc->sc_mtx)); /* XXX */

	return(0);
}

static void
aue_cfg_miibus_statchg(device_t dev)
{
	struct aue_softc * sc = device_get_softc(dev);
	struct mii_data	* mii = GET_MII(sc);

	mtx_lock(&(sc->sc_mtx)); /* XXX */

	AUE_CFG_CLRBIT(sc, AUE_CTL0, AUE_CTL0_RX_ENB | AUE_CTL0_TX_ENB);

	if (IFM_SUBTYPE(mii->mii_media_active) == IFM_100_TX) {
		AUE_CFG_SETBIT(sc, AUE_CTL1, AUE_CTL1_SPEEDSEL);
	} else {
		AUE_CFG_CLRBIT(sc, AUE_CTL1, AUE_CTL1_SPEEDSEL);
	}

	if ((mii->mii_media_active & IFM_GMASK) == IFM_FDX) {
		AUE_CFG_SETBIT(sc, AUE_CTL1, AUE_CTL1_DUPLEX);
	} else {
		AUE_CFG_CLRBIT(sc, AUE_CTL1, AUE_CTL1_DUPLEX);
	}

	AUE_CFG_SETBIT(sc, AUE_CTL0, AUE_CTL0_RX_ENB | AUE_CTL0_TX_ENB);

	/*
	 * Set the LED modes on the LinkSys adapter.
	 * This turns on the 'dual link LED' bin in the auxmode
	 * register of the Broadcom PHY.
	 */
	if (sc->sc_flags & AUE_FLAG_LSYS) {
		u_int16_t auxmode;
		auxmode = aue_cfg_miibus_readreg(dev, 0, 0x1b);
		aue_cfg_miibus_writereg(dev, 0, 0x1b, auxmode | 0x04);
	}

	mtx_unlock(&(sc->sc_mtx)); /* XXX */

	return;
}

static void
aue_cfg_setmulti(struct aue_softc *sc,
		 struct aue_config_copy *cc, u_int16_t refcount)
{
	u_int16_t i;

	if ((cc->if_flags & IFF_ALLMULTI) || 
	    (cc->if_flags & IFF_PROMISC)) {
	    AUE_CFG_SETBIT(sc, AUE_CTL0, AUE_CTL0_ALLMULTI);
	    return;
	}

	AUE_CFG_CLRBIT(sc, AUE_CTL0, AUE_CTL0_ALLMULTI);

	/* clear existing ones */
	for (i = 0; i < 8; i++) {
	    aue_cfg_csr_write_1(sc, AUE_MAR0 + i, 0);
	}

	/* now program new ones */
	for (i = 0; i < 8; i++) {
	    aue_cfg_csr_write_1(sc, AUE_MAR0 + i, cc->if_hash[i]);
	}
	return;
}

static void
aue_cfg_reset_pegasus_II(struct aue_softc *sc)
{
	/* Magic constants taken from Linux driver. */
	aue_cfg_csr_write_1(sc, AUE_REG_1D, 0);
	aue_cfg_csr_write_1(sc, AUE_REG_7B, 2);
#if 0
	if ((sc->sc_flags & HAS_HOME_PNA) && mii_mode)
		aue_cfg_csr_write_1(sc, AUE_REG_81, 6);
	else
#endif
		aue_cfg_csr_write_1(sc, AUE_REG_81, 2);

	return;
}

static void
aue_cfg_reset(struct aue_softc *sc)
{
	u_int16_t i;

	AUE_CFG_SETBIT(sc, AUE_CTL1, AUE_CTL1_RESETMAC);

	for (i = 0; ; i++) {
	  
	    if (i < AUE_TIMEOUT) {

	        if (!(aue_cfg_csr_read_1(sc, AUE_CTL1) & AUE_CTL1_RESETMAC)) {
		    break;
		}

		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    break;
		}

	    } else {
	        DPRINTF(sc, 0, "reset timed out\n");
		break;
	    }
	}

	/*
	 * The PHY(s) attached to the Pegasus chip may be held
	 * in reset until we flip on the GPIO outputs. Make sure
	 * to set the GPIO pins high so that the PHY(s) will
	 * be enabled.
	 *
	 * Note: We force all of the GPIO pins low first, *then*
	 * enable the ones we want.
	 */
	aue_cfg_csr_write_1(sc, AUE_GPIO0, (AUE_GPIO_OUT0|AUE_GPIO_SEL0));
	aue_cfg_csr_write_1(sc, AUE_GPIO0, (AUE_GPIO_OUT0|AUE_GPIO_SEL0|
					    AUE_GPIO_SEL1));

	if (sc->sc_flags & AUE_FLAG_LSYS) {
		/* Grrr. LinkSys has to be different from everyone else. */
		aue_cfg_csr_write_1(sc, AUE_GPIO0,
				    (AUE_GPIO_SEL0 | AUE_GPIO_SEL1));
		aue_cfg_csr_write_1(sc, AUE_GPIO0,
				    (AUE_GPIO_SEL0 | 
				     AUE_GPIO_SEL1 | 
				     AUE_GPIO_OUT0));
	}

	if (sc->sc_flags & AUE_FLAG_PII) {
	    aue_cfg_reset_pegasus_II(sc);
	}

	/* wait a little while for the chip to get its brains in order: */
	usbd_config_td_sleep(&(sc->sc_config_td), hz/100);

	return;
}

/*
 * Probe for a Pegasus chip.
 */
static int
aue_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->iface != NULL) {
	    return UMATCH_NONE;
	}

	return (aue_lookup(uaa->vendor, uaa->product) != NULL ?
		UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

/*
 * Attach the interface. Allocate softc structures, do ifmedia
 * setup and ethernet/BPF attach.
 */
static int
aue_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct aue_softc *sc = device_get_softc(dev);
	int32_t error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	sc->sc_udev = uaa->device;
	sc->sc_dev = dev;
	sc->sc_unit = device_get_unit(dev);
	sc->sc_flags = aue_lookup(uaa->vendor, uaa->product)->aue_flags;
	sc->sc_product = uaa->product;
	sc->sc_vendor = uaa->vendor;

	if (uaa->release >= 0x0201) {
	    sc->sc_flags |= AUE_FLAG_VER_2; /* XXX currently undocumented */
	}

	usbd_set_desc(dev, uaa->device);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s",
		 device_get_nameunit(dev));

	mtx_init(&(sc->sc_mtx), "aue lock", NULL, MTX_DEF | MTX_RECURSE);

	__callout_init_mtx(&(sc->sc_watchdog),
			   &(sc->sc_mtx), CALLOUT_RETURNUNLOCKED);

	error = usbd_set_config_no(uaa->device, AUE_CONFIG_NO, 0);

	if (error) {
		device_printf(dev, "setting config "
			      "number failed!\n");
		goto detach;
	}

	error = usbd_transfer_setup(uaa->device, AUE_IFACE_IDX, 
				    sc->sc_xfer, aue_config, AUE_ENDPT_MAX,
				    sc, &(sc->sc_mtx));
	if (error) {
		device_printf(dev, "allocating USB "
			      "transfers failed!\n");
		goto detach;
	}

	error = usbd_config_td_setup(&(sc->sc_config_td), sc, &(sc->sc_mtx),
				     NULL, sizeof(struct aue_config_copy), 16);
	if (error) {
		device_printf(dev, "could not setup config "
			      "thread!\n");
		goto detach;
	}

	mtx_lock(&(sc->sc_mtx));

	sc->sc_flags |= AUE_FLAG_WAIT_LINK;

	/* start setup */

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL, &aue_cfg_first_time_setup, 0, 0);

	/* start watchdog (will exit mutex) */

	aue_watchdog(sc);

	return 0; /* success */

 detach:
	aue_detach(dev);
	return ENXIO; /* failure */
}

static void
aue_cfg_first_time_setup(struct aue_softc *sc,
			 struct aue_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp;
	int error;
	u_int8_t eaddr[min(ETHER_ADDR_LEN,6)];

	/* reset the adapter */
	aue_cfg_reset(sc);

	/* set default value */
	bzero(eaddr, sizeof(eaddr));

	/* get station address from the EEPROM */
	aue_cfg_read_eeprom(sc, eaddr, 0, 3);

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
	if_initname(ifp, "aue", sc->sc_unit);
	ifp->if_mtu = ETHERMTU;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = aue_ioctl_cb;
	ifp->if_start = aue_start_cb;
	ifp->if_watchdog = NULL;
	ifp->if_init = aue_init_cb;
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
	ifp->if_snd.ifq_drv_maxlen = IFQ_MAXLEN;
	IFQ_SET_READY(&ifp->if_snd);

	/* XXX need Giant when accessing
	 * the device structures !
	 */

	mtx_unlock(&(sc->sc_mtx));

	mtx_lock(&Giant);

	error = mii_phy_probe(sc->sc_dev, &(sc->sc_miibus),
			      &aue_ifmedia_upd_cb, 
			      &aue_ifmedia_sts_cb);

	mtx_unlock(&Giant);

	mtx_lock(&(sc->sc_mtx));

	/*
	 * Do MII setup.
	 * NOTE: Doing this causes child devices to be attached to us,
	 * which we would normally disconnect at in the detach routine
	 * using device_delete_child(). However the USB code is set up
	 * such that when this driver is removed, all children devices
	 * are removed as well. In effect, the USB code ends up detaching
	 * all of our children for us, so we don't have to do is ourselves
	 * in aue_detach(). It's important to point this out since if
	 * we *do* try to detach the child devices ourselves, we will
	 * end up getting the children deleted twice, which will crash
	 * the system.
	 */
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
aue_detach(device_t dev)
{
	struct aue_softc * sc = device_get_softc(dev);
	struct ifnet * ifp;

	usbd_config_td_stop(&(sc->sc_config_td));

	mtx_lock(&(sc->sc_mtx));

	__callout_stop(&(sc->sc_watchdog));

	aue_cfg_pre_stop(sc, NULL, 0);

	ifp = sc->sc_ifp;

	mtx_unlock(&(sc->sc_mtx));

	/* get rid of any late children */
	bus_generic_detach(dev);

	if (ifp) {
	    ether_ifdetach(ifp);
	    if_free(ifp);
	}

	usbd_transfer_unsetup(sc->sc_xfer, AUE_ENDPT_MAX);

	usbd_config_td_unsetup(&(sc->sc_config_td));

	__callout_drain(&(sc->sc_watchdog));

	mtx_destroy(&(sc->sc_mtx));

	return 0;
}

static void
aue_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct aue_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[4];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
        return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~AUE_FLAG_INTR_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~AUE_FLAG_INTR_STALL;
	DPRINTF(sc, 0, "interrupt read pipe stopped\n");
	return;
}

static void
aue_intr_callback(struct usbd_xfer *xfer)
{
	struct aue_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct aue_intrpkt *p = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:

	if (ifp && (ifp->if_drv_flags & IFF_DRV_RUNNING) &&
	    (xfer->actlen >= sizeof(*p))) {

	    if (p->aue_txstat0) {
	        ifp->if_oerrors++;
	    }

	    if (p->aue_txstat0 & (AUE_TXSTAT0_LATECOLL & 
				  AUE_TXSTAT0_EXCESSCOLL)) {
	        ifp->if_collisions++;
	    }
	}

 tr_setup:
	if (sc->sc_flags & AUE_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[5]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* start clear stall */
	    sc->sc_flags |= AUE_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[5]);
	}
	return;
}

static void
aue_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct aue_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~AUE_FLAG_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~AUE_FLAG_READ_STALL;
	DPRINTF(sc, 0, "bulk read pipe stopped\n");
	return;
}

static void
aue_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct aue_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= AUE_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	DPRINTF(sc, 0, "bulk read error, %s\n",
		usbd_errstr(xfer->error));
	return;

 tr_transferred:
	DPRINTF(sc, 10, "received %d bytes\n", xfer->actlen);

	if (sc->sc_flags & AUE_FLAG_VER_2) {

	    if (xfer->actlen == 0) {
		ifp->if_ierrors++;
		goto tr_setup;
	    }

	} else {

	    if (xfer->actlen <= (4 + ETHER_CRC_LEN)) {
		ifp->if_ierrors++;
		goto tr_setup;
	    }

	    usbd_copy_out(&(xfer->buf_data), xfer->actlen - 4, &(sc->sc_rxpkt), 
			  sizeof(sc->sc_rxpkt));

	    /* turn off all the non-error bits 
	     * in the rx status word:
	     */
	    sc->sc_rxpkt.aue_rxstat &= AUE_RXSTAT_MASK;

	    if (sc->sc_rxpkt.aue_rxstat) {
		ifp->if_ierrors++;
		goto tr_setup;
	    }

	    /* No errors; receive the packet. */
	    xfer->actlen -= (4 + ETHER_CRC_LEN);
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

	if (sc->sc_flags & AUE_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;
}

static void
aue_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct aue_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~AUE_FLAG_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~AUE_FLAG_WRITE_STALL;
	DPRINTF(sc, 0, "bulk write pipe stopped\n");
	return;
}

static void
aue_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct aue_softc *sc = xfer->priv_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;
	u_int8_t buf[2];

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 10, "transfer error, %s\n",
		 usbd_errstr(xfer->error));

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= AUE_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}

	ifp->if_oerrors++;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	return;

 tr_transferred:
	DPRINTF(sc, 10, "transfer of %d bytes complete\n", xfer->actlen);

	ifp->if_opackets++;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

 tr_setup:

	if (sc->sc_flags & AUE_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    goto done;
	}

	if (sc->sc_flags & AUE_FLAG_WAIT_LINK) {
	    /* don't send anything 
	     * if there is no link !
	     */
	    goto done;
	}

	IFQ_DRV_DEQUEUE(&(ifp->if_snd), m);

	if (m == NULL) {
	    goto done;
	}

	if (m->m_pkthdr.len > MCLBYTES) {
	    m->m_pkthdr.len = MCLBYTES;
	}

	if (sc->sc_flags & AUE_FLAG_VER_2) {

	    xfer->length = m->m_pkthdr.len;

	    usbd_m_copy_in(&(xfer->buf_data), 0, 
			   m, 0, m->m_pkthdr.len);

	} else {

	    xfer->length = (m->m_pkthdr.len + 2);

	    /*
	     * The ADMtek documentation says that the packet length is
	     * supposed to be specified in the first two bytes of the
	     * transfer, however it actually seems to ignore this info
	     * and base the frame size on the bulk transfer length.
	     */
	    buf[0] = (u_int8_t)(m->m_pkthdr.len);
	    buf[1] = (u_int8_t)(m->m_pkthdr.len >> 8);

	    usbd_copy_in(&(xfer->buf_data), 0, buf, 2);

	    usbd_m_copy_in(&(xfer->buf_data), 2, 
			   m, 0, m->m_pkthdr.len);
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

#define AUE_BITS 6

static void
aue_config_copy(struct aue_softc *sc, 
		struct aue_config_copy *cc, u_int16_t refcount)
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

		h = (ether_crc32_le
		     (LLADDR((struct sockaddr_dl *)(ifma->ifma_addr)), 
		      ETHER_ADDR_LEN)) & ((1 << AUE_BITS) - 1);

		cc->if_hash[(h >> 3)] |= (1 << (h & 7));
	    }
	    IF_ADDR_UNLOCK(ifp);
	}
	return;
}

static void
aue_cfg_tick(struct aue_softc *sc,
	     struct aue_config_copy *cc, u_int16_t refcount)
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

	if ((sc->sc_flags & AUE_FLAG_WAIT_LINK) &&
	    (mii->mii_media_status & IFM_ACTIVE) &&
	    (IFM_SUBTYPE(mii->mii_media_active) != IFM_NONE)) {
	    sc->sc_flags &= ~AUE_FLAG_WAIT_LINK;
	}

	sc->sc_media_active = mii->mii_media_active;
	sc->sc_media_status = mii->mii_media_status;

	/* start stopped transfers, if any */

	aue_start_transfers(sc);

	return;
}

static void
aue_start_cb(struct ifnet *ifp)
{
	struct aue_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	aue_start_transfers(sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
aue_init_cb(void *arg)
{
	struct aue_softc *sc = arg;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &aue_cfg_pre_init, &aue_cfg_init, 0, 0);
	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
aue_start_transfers(struct aue_softc *sc)
{
	if ((sc->sc_flags & AUE_FLAG_LL_READY) &&
	    (sc->sc_flags & AUE_FLAG_HL_READY)) {

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
aue_cfg_pre_init(struct aue_softc *sc,
		 struct aue_config_copy *cc, u_int16_t refcount)
{
	struct ifnet *ifp = sc->sc_ifp;

	/* immediate configuration */

	aue_cfg_pre_stop(sc, cc, 0);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	sc->sc_flags |= AUE_FLAG_HL_READY;
	return;
}

static void
aue_cfg_init(struct aue_softc *sc,
	     struct aue_config_copy *cc, u_int16_t refcount)
{
	struct mii_data *mii = GET_MII(sc);
	u_int8_t i;

	/*
	 * Cancel pending I/O
	 */
	aue_cfg_stop(sc, cc, 0);

	/* Set MAC address */
	for (i = 0; i < ETHER_ADDR_LEN; i++) {
	    aue_cfg_csr_write_1(sc, AUE_PAR0 + i, cc->if_lladdr[i]);
	}

	/* update promiscuous setting */
	aue_cfg_promisc_upd(sc, cc, 0);

	/* load the multicast filter */
	aue_cfg_setmulti(sc, cc, 0);

	/* enable RX and TX */
	aue_cfg_csr_write_1(sc, AUE_CTL0, 
			    (AUE_CTL0_RXSTAT_APPEND | 
			     AUE_CTL0_RX_ENB));

	AUE_CFG_SETBIT(sc, AUE_CTL0, AUE_CTL0_TX_ENB);
	AUE_CFG_SETBIT(sc, AUE_CTL2, AUE_CTL2_EP3_CLR);

	mii_mediachg(mii);

	sc->sc_flags |= (AUE_FLAG_READ_STALL|
			 AUE_FLAG_WRITE_STALL|
			 AUE_FLAG_LL_READY);

	aue_start_transfers(sc);
	return;
}

static void
aue_cfg_promisc_upd(struct aue_softc *sc,
		    struct aue_config_copy *cc, u_int16_t refcount)
{
	/* if we want promiscuous mode, set the allframes bit: */
	if (cc->if_flags & IFF_PROMISC) {
	    AUE_CFG_SETBIT(sc, AUE_CTL2, AUE_CTL2_RX_PROMISC);
	} else {
	    AUE_CFG_CLRBIT(sc, AUE_CTL2, AUE_CTL2_RX_PROMISC);
	}
	return;
}

/*
 * Set media options.
 */
static int
aue_ifmedia_upd_cb(struct ifnet *ifp)
{
	struct aue_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL, &aue_cfg_ifmedia_upd, 0, 0);
	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

static void
aue_cfg_ifmedia_upd(struct aue_softc *sc,
		    struct aue_config_copy *cc, u_int16_t refcount)
{
	struct ifnet * ifp = sc->sc_ifp;
	struct mii_data * mii = GET_MII(sc);

	if ((ifp == NULL) || 
	    (mii == NULL)) {
	    /* not ready */
	    return;
	}

	sc->sc_flags |= AUE_FLAG_WAIT_LINK;

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
aue_ifmedia_sts_cb(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct aue_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	ifmr->ifm_active = sc->sc_media_active;
	ifmr->ifm_status = sc->sc_media_status;

	mtx_unlock(&(sc->sc_mtx));
	return;
}

static int
aue_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct aue_softc * sc = ifp->if_softc;
	struct mii_data * mii;
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	switch(command) {
	case SIOCSIFFLAGS:

	    if (ifp->if_flags & IFF_UP) {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &aue_config_copy, 
		       &aue_cfg_promisc_upd, 0, 0);
		} else {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &aue_cfg_pre_init,
		       &aue_cfg_init, 0, 0); 
		}
	    } else {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &aue_cfg_pre_stop,
		       &aue_cfg_stop, 0, 0);
		}
	    }
	    break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &aue_config_copy,
	       &aue_cfg_setmulti, 0, 0);
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
aue_watchdog(void *arg)
{
	struct aue_softc *sc = arg;

	mtx_assert(&(sc->sc_mtx), MA_OWNED);

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL, &aue_cfg_tick, 0, 0);

	__callout_reset(&(sc->sc_watchdog), 
			hz, &aue_watchdog, sc);

	mtx_unlock(&(sc->sc_mtx));
	return;
}

/*
 * Stop the adapter and free any mbufs allocated to the
 * RX and TX lists.
 *
 * NOTE: can be called when "ifp" is NULL
 */
static void
aue_cfg_pre_stop(struct aue_softc *sc,
		 struct aue_config_copy *cc, uint16_t refcount)
{
	struct ifnet *ifp = sc->sc_ifp;

	if (cc) {
	    /* copy the needed configuration */
	    aue_config_copy(sc, cc, refcount);
	}

	/* immediate configuration */

	if (ifp) {
	    /* clear flags */
	    ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | 
				   IFF_DRV_OACTIVE);
	}

	sc->sc_flags &= ~(AUE_FLAG_HL_READY|
			  AUE_FLAG_LL_READY);

	sc->sc_flags |= AUE_FLAG_WAIT_LINK;

	/* stop all the transfers, 
	 * if not already stopped:
	 */
	usbd_transfer_stop(sc->sc_xfer[0]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[3]);
	usbd_transfer_stop(sc->sc_xfer[4]);
	usbd_transfer_stop(sc->sc_xfer[5]);
	return;
}

static void
aue_cfg_stop(struct aue_softc *sc,
	     struct aue_config_copy *cc, u_int16_t refcount)
{
	aue_cfg_csr_write_1(sc, AUE_CTL0, 0);
	aue_cfg_csr_write_1(sc, AUE_CTL1, 0);
	aue_cfg_reset(sc);
	return;
}

/*
 * Stop all chip I/O so that the kernel's probe routines don't
 * get confused by errant DMAs when rebooting.
 */
static int
aue_shutdown(device_t dev)
{
	struct aue_softc *sc = device_get_softc(dev);

	mtx_lock(&(sc->sc_mtx));

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &aue_cfg_pre_stop, 
	   &aue_cfg_stop, 0, 0);

	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

