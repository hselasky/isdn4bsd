/*	$OpenBSD: if_zyd.c,v 1.27 2006/09/23 22:28:43 mglocker Exp $	*/

/*
 * Copyright (c) 2006 by Florian Stoehr <ich@florian-stoehr.de>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/if_zyd.c $");

/*
 * ZyDAS ZD1211 USB WLAN driver
 *
 * NOTE: all function names beginning like "zyd_cfg_" can only
 * be called from within the config thread function !
 */

#include <sys/param.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/endian.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#include <netinet/if_ether.h>

#define usbd_config_td_cc zyd_config_copy
#define usbd_config_td_softc zyd_softc

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

#include <dev/usb/if_zydreg.h>
#include <dev/usb/if_zydfw.h>

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)	\
  do { if (zyd_debug > (n)) {	     \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int zyd_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, zyd, CTLFLAG_RW, 0, "USB zyd");
SYSCTL_INT(_hw_usb_zyd, OID_AUTO, debug, CTLFLAG_RW, &zyd_debug, 0,
	   "zyd debug level");
#else
#define DPRINTF(...)
#endif

static device_probe_t zyd_probe;
static device_attach_t zyd_attach;
static device_detach_t zyd_detach;

static usbd_callback_t zyd_intr_read_clear_stall_callback;
static usbd_callback_t zyd_intr_read_callback;
static usbd_callback_t zyd_intr_write_clear_stall_callback;
static usbd_callback_t zyd_intr_write_callback;
static usbd_callback_t zyd_bulk_read_clear_stall_callback;
static usbd_callback_t zyd_bulk_read_callback;
static usbd_callback_t zyd_bulk_write_clear_stall_callback;
static usbd_callback_t zyd_bulk_write_callback;

static usbd_config_td_command_t zyd_cfg_first_time_setup;
static usbd_config_td_command_t zyd_cfg_set_run;
static usbd_config_td_command_t zyd_cfg_update_promisc;
static usbd_config_td_command_t zyd_cfg_set_chan;
static usbd_config_td_command_t zyd_cfg_pre_init;
static usbd_config_td_command_t zyd_cfg_init;
static usbd_config_td_command_t zyd_cfg_pre_stop;
static usbd_config_td_command_t zyd_cfg_stop;
static usbd_config_td_command_t zyd_config_copy;

static uint16_t	zyd_getrealaddr(struct zyd_softc *sc, uint32_t mangled_addr);
static void	zyd_cfg_usbrequest(struct zyd_softc *sc, uint8_t type, uint8_t request, uint16_t value, uint16_t index, uint16_t length, uint8_t *data);
static void	zyd_cfg_usbrequestzc(struct zyd_softc *sc, struct zyd_control *zc);
static void	zyd_cfg_reset(struct zyd_softc *sc);
static void	zyd_cfg_usb_intr_read(struct zyd_softc *sc, void *data, uint32_t size);
static void	zyd_cfg_usb_intr_write(struct zyd_softc *sc, void *data, uint32_t size);
static uint32_t	zyd_addrinc(uint32_t addr);
static void	zyd_cfg_read16(struct zyd_softc *sc, uint32_t addr, uint16_t *value);
static void	zyd_cfg_read32(struct zyd_softc *sc, uint32_t addr, uint32_t *value);
static void	zyd_cfg_read16_multi(struct zyd_softc *sc, const uint32_t *addrs, uint16_t *data, uint8_t usecount);
static void	zyd_cfg_read32_multi(struct zyd_softc *sc, const uint32_t *addrs, uint32_t *data, uint8_t usecount);
static void	zyd_cfg_write16(struct zyd_softc *sc, uint32_t addr, uint16_t value);
static void	zyd_cfg_write32(struct zyd_softc *sc, uint32_t addr, uint32_t value);
static void	zyd_cfg_write16_multi(struct zyd_softc *sc, const uint32_t *addrs, uint16_t *data, uint8_t usecount);
static void	zyd_cfg_write32_multi(struct zyd_softc *sc, const uint32_t *addrs, uint32_t *data, uint8_t usecount);
static void	zyd_cfg_write16_batch(struct zyd_softc *sc, const struct zyd_adpairs16 *data, uint32_t count);
static void	zyd_cfg_write32_batch(struct zyd_softc *sc, const struct zyd_adpairs32 *data, uint32_t count);
static void	zyd_cfg_rfwrite(struct zyd_softc *sc, uint32_t value, uint8_t bits);
static void	zyd_cfg_stateoutput(struct zyd_softc *sc) __used;
static void	zyd_rxframeproc(struct usbd_xfer *xfer, uint16_t offset, uint16_t len);
static uint8_t	zyd_cfg_uploadfirmware(struct zyd_softc *sc);
static void	zyd_cfg_lock_phy(struct zyd_softc *sc);
static void	zyd_cfg_unlock_phy(struct zyd_softc *sc);
static void	zyd_cfg_get_aw_pt_bi(struct zyd_softc *sc, struct zyd_aw_pt_bi *s);
static void	zyd_cfg_set_aw_pt_bi(struct zyd_softc *sc, struct zyd_aw_pt_bi *s);
static void	zyd_cfg_set_beacon_interval(struct zyd_softc *sc, uint32_t interval);
static const char *zyd_rf_name(uint8_t type);
static void	zyd_cfg_read_rf_pa_types(struct zyd_softc *sc, uint8_t *rf_type, uint8_t *pa_type);
static void	zyd_cfg_rf_rfmd_init(struct zyd_softc *sc, struct zyd_rf *rf);
static void	zyd_cfg_rf_rfmd_switchradio(struct zyd_softc *sc, uint8_t onoff);
static void	zyd_cfg_rf_rfmd_set_channel(struct zyd_softc *sc, struct zyd_rf *rf, uint8_t channel);
static void	zyd_cfg_rf_al2230_switchradio(struct zyd_softc *sc, uint8_t onoff);
static void	zyd_cfg_rf_al2230_init(struct zyd_softc *sc, struct zyd_rf *rf);
static void	zyd_cfg_rf_al2230_set_channel(struct zyd_softc *sc, struct zyd_rf *rf, uint8_t channel);
static uint8_t	zyd_cfg_rf_init_hw(struct zyd_softc *sc, struct zyd_rf *rf, uint8_t type);
static uint8_t	zyd_cfg_hw_init(struct zyd_softc *sc, struct ieee80211com *ic);
static void	zyd_cfg_get_e2p_mac_addr(struct zyd_softc *sc, struct zyd_macaddr *mac_addr);
static void	zyd_cfg_set_mac_addr(struct zyd_softc *sc, const uint8_t *addr);
static void	zyd_cfg_read_regdomain(struct zyd_softc *sc, uint8_t *regdomain);
static uint8_t	zyd_regdomain_supported(uint8_t regdomain);
static void	zyd_cfg_tblreader(struct zyd_softc *sc, uint8_t *values, size_t count, uint32_t e2p_addr, uint32_t guard);
static void	zyd_cfg_readcaltables(struct zyd_softc *sc);
static uint8_t	zyd_reset_channel(struct zyd_softc *sc) __used;
static void	zyd_cfg_set_encryption_type(struct zyd_softc *sc, uint32_t type);
static void	zyd_cfg_switch_radio(struct zyd_softc *sc, uint8_t onoff);
static void	zyd_cfg_enable_hwint(struct zyd_softc *sc);
static void	zyd_cfg_disable_hwint(struct zyd_softc *sc);
static void	zyd_cfg_set_basic_rates(struct zyd_softc *sc, int mode);
static void	zyd_cfg_set_mandatory_rates(struct zyd_softc *sc, int mode);
static void	zyd_cfg_reset_mode(struct zyd_softc *sc);
static void	zyd_cfg_set_bssid(struct zyd_softc *sc, uint8_t *addr);
static int	zyd_media_change_cb(struct ifnet *ifp);
static int	zyd_newstate_cb(struct ieee80211com *ic, enum ieee80211_state nstate, int arg);
static uint16_t	zyd_txtime(uint16_t len, uint8_t rate, uint32_t flags);
static uint8_t	zyd_plcp_signal(uint8_t rate);
static uint16_t	zyd_calc_useclen(uint8_t rate, uint16_t len, uint8_t *service);
static void	zyd_setup_tx_desc(struct usbd_xfer *xfer, struct mbuf *m, uint16_t rate);
static void	zyd_cfg_dump_fw_registers(struct zyd_softc *sc) __used;
static uint8_t	zyd_tx_frame(struct usbd_xfer *xfer, struct mbuf *m0, struct ieee80211_node *ni, uint8_t rate);
static void	zyd_start_transfers(struct zyd_softc *sc);
static void	zyd_start_cb(struct ifnet *ifp);
static void	zyd_init_cb(void *arg);
static int	zyd_reset_cb(struct ifnet *ifp);
static int	zyd_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data);
static void	zyd_watchdog(void *arg);
static void	zyd_next_scan(void *arg);
static void	zyd_end_of_commands(struct zyd_softc *sc);

static const struct usb_devno zyd_devs[] = {
	{ USB_VENDOR_3COM2,		USB_PRODUCT_3COM2_3CRUSB10075 },
	{ USB_VENDOR_ABOCOM,		USB_PRODUCT_ABOCOM_WL54 },
	{ USB_VENDOR_ASUS,		USB_PRODUCT_ASUS_WL159G },
	{ USB_VENDOR_BELKIN,		USB_PRODUCT_BELKIN_F5D7050C },
	{ USB_VENDOR_CYBERTAN,		USB_PRODUCT_CYBERTAN_TG54USB },
	{ USB_VENDOR_DRAYTEK,		USB_PRODUCT_DRAYTEK_VIGOR550 },
	{ USB_VENDOR_PLANEX2,		USB_PRODUCT_PLANEX2_GWUS54GZL },
	{ USB_VENDOR_PLANEX3,		USB_PRODUCT_PLANEX3_GWUS54MINI },
	{ USB_VENDOR_SAGEM,		USB_PRODUCT_SAGEM_XG760A },
	{ USB_VENDOR_SITECOMEU,		USB_PRODUCT_SITECOMEU_WL113 },
	{ USB_VENDOR_SWEEX,		USB_PRODUCT_SWEEX_ZD1211 },
	{ USB_VENDOR_TEKRAM,		USB_PRODUCT_TEKRAM_QUICKWLAN },
	{ USB_VENDOR_TEKRAM,		USB_PRODUCT_TEKRAM_ZD1211 },
	{ USB_VENDOR_TWINMOS,		USB_PRODUCT_TWINMOS_G240 },
	{ USB_VENDOR_UMEDIA,		USB_PRODUCT_UMEDIA_TEW429UB_A },
	{ USB_VENDOR_UMEDIA,		USB_PRODUCT_UMEDIA_TEW429UB },
	{ USB_VENDOR_WISTRONNEWEB,	USB_PRODUCT_WISTRONNEWEB_UR055G },
	{ USB_VENDOR_ZYDAS,		USB_PRODUCT_ZYDAS_ZD1211 },
	{ USB_VENDOR_ZYXEL,		USB_PRODUCT_ZYXEL_ZYAIRG220 }
};

/* Device, regardless of RF */
static const struct zyd_adpairs16 zyd_def_cr[] = {
	ZYD_DEF_CR
};

static const struct zyd_adpairs32 zyd_def_mac[] = {
	ZYD_DEF_MAC
};

/* RF2959 */
static const struct zyd_adpairs16 zyd_rfmd_cr[] = {
	ZYD_RFMD_CR
};

static const uint32_t zyd_rfmd_rf[] = {
	ZYD_RFMD_RF
};

/* AL2230 */
static const struct zyd_adpairs16 zyd_al2230_cr[] = {
	ZYD_AL2230_CR
};

static const uint32_t zyd_al2230_rf[] = {
	ZYD_AL2230_RF
};

static const struct usbd_config zyd_config[ZYD_TR_MAX] = {
    [ZYD_TR_BULK_DT_WR] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_OUT,
      .bufsize   = (MCLBYTES + sizeof(struct zyd_controlsetformat) + 1),
      .flags     = (USBD_USE_DMA|USBD_FORCE_SHORT_XFER),
      .callback  = &zyd_bulk_write_callback,
      .index     = 0,
      .timeout   = 10000, /* 10 seconds */
    },

    [ZYD_TR_BULK_DT_RD] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_IN,
      .bufsize   = (MAX(MCLBYTES,2312) + sizeof(struct zyd_rxleninfoapp)),
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &zyd_bulk_read_callback,
      .index     = 0,
    },

    [ZYD_TR_BULK_CS_WR] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &zyd_bulk_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [ZYD_TR_BULK_CS_RD] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &zyd_bulk_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [ZYD_TR_INTR_DT_WR] = {
      .type      = UE_BULK_INTR,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_OUT,
      .bufsize   = ZYD_INTR_BUF_SIZE,
      .flags     = (USBD_USE_DMA|USBD_FORCE_SHORT_XFER),
      .callback  = &zyd_intr_write_callback,
      .timeout   = 1000, /* 1 second */
      .index     = 1,
    },

    [ZYD_TR_INTR_DT_RD] = {
      .type      = UE_BULK_INTR,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_IN,
      .bufsize   = ZYD_INTR_BUF_SIZE,
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &zyd_intr_read_callback,
      .timeout   = 1000, /* 1 second */
      .index     = 1,
    },

    [ZYD_TR_INTR_CS_WR] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &zyd_intr_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [ZYD_TR_INTR_CS_RD] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &zyd_intr_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static devclass_t zyd_devclass;

static device_method_t zyd_methods[] = {
    DEVMETHOD(device_probe, zyd_probe),
    DEVMETHOD(device_attach, zyd_attach),
    DEVMETHOD(device_detach, zyd_detach),
    { 0, 0 }
};

static driver_t zyd_driver = {
    .name    = "zyd",
    .methods = zyd_methods,
    .size    = sizeof(struct zyd_softc),
};

DRIVER_MODULE(zyd, uhub, zyd_driver, zyd_devclass, usbd_driver_load, 0);
MODULE_DEPEND(zyd, usb, 1, 1, 1);
MODULE_DEPEND(zyd, wlan, 1, 1, 1);

/*
 * Get the real address from a range-mangled address
 */
static uint16_t
zyd_getrealaddr(struct zyd_softc *sc, uint32_t mangled_addr)
{
	uint16_t add;
	uint16_t res;

	add = 0;

	switch (ZYD_GET_RANGE(mangled_addr)) {
	case ZYD_RANGE_USB:
		break;

	case ZYD_RANGE_CTL:
		add = ZYD_CTRL_START_ADDR;
		break;

	case ZYD_RANGE_E2P:
		add = ZYD_E2P_START_ADDR;
		break;

	case ZYD_RANGE_FW:
		add = sc->sc_firmware_base;
		break;
	}

	res = (add + ZYD_GET_OFFS(mangled_addr));

	return res;
}


/*
 * USB request basic wrapper
 */
static void
zyd_cfg_usbrequest(struct zyd_softc *sc, uint8_t type, uint8_t request,
	uint16_t value, uint16_t index, uint16_t length, uint8_t *data)
{
	usb_device_request_t req;
	usbd_status err;

	req.bmRequestType = type;
	req.bRequest = request;
	USETW(req.wValue, value);
	USETW(req.wIndex, index);
	USETW(req.wLength, length);

	DPRINTF(sc, 20, "req=%02x val=%02x ind=%02x "
	    "len=%02x\n", request,
	    value, index, length);

	if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto error;
	}

	err = usbd_do_request_flags_mtx(sc->sc_udev, &(sc->sc_mtx), &req, 
					data, 0, NULL, 10000);

	if (err) {

	    printf("%s: device request failed, err=%s "
		   "(ignored)\n", sc->sc_name, usbd_errstr(err));

	error:
	    length = UGETW(req.wLength);

	    if ((req.bmRequestType & UT_READ) && length) {
	        bzero(data, length);
	    }
	}
	return;
}

/*
 * Same, higher level
 */
static void
zyd_cfg_usbrequestzc(struct zyd_softc *sc, struct zyd_control *zc)
{
	return zyd_cfg_usbrequest(sc, zc->type, zc->id, zc->value,
	    zc->index, zc->length, zc->data);
}

/*
 * Issue a SET_CONFIGURATION command, which will reset the device.
 */
static void
zyd_cfg_reset(struct zyd_softc *sc)
{
	usbd_status err;

	mtx_unlock(&(sc->sc_mtx));

	mtx_lock(&Giant);

	err = usbreq_set_config(sc->sc_udev, ZYD_CONFIG_NO);

	mtx_unlock(&Giant);

	mtx_lock(&(sc->sc_mtx));

	if (err) {
	    DPRINTF(sc, 0, "reset failed (ignored)\n");
	}

	/* Wait a little while for the chip to get its brains in order. */
	err = usbd_config_td_sleep(&(sc->sc_config_td), hz/10);

	return;
}

static void
zyd_intr_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[ZYD_TR_INTR_DT_RD];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~ZYD_FLAG_INTR_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~ZYD_FLAG_INTR_READ_STALL;
	DPRINTF(sc, -1, "clear stall failed\n");
	return;
}

/*
 * Callback handler for interrupt transfer
 */
static void
zyd_intr_read_callback(struct usbd_xfer *xfer)
{
	struct zyd_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 2, "error=%d\n", xfer->error);

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= ZYD_FLAG_INTR_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_INTR_CS_RD]);
	}
	goto wakeup;

 tr_transferred:

	DPRINTF(sc, 2, "length=%d\n", xfer->actlen);

	if (xfer->actlen > sizeof(sc->sc_intr_ibuf)) {
	    xfer->actlen = sizeof(sc->sc_intr_ibuf);
	}

	usbd_copy_out(&(xfer->buf_data), 0, &(sc->sc_intr_ibuf),
		      xfer->actlen);
	goto wakeup;

 tr_setup:

	if (sc->sc_flags & ZYD_FLAG_INTR_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_INTR_CS_RD]);
	    goto wakeup;
	}

	if (sc->sc_intr_iwakeup) {
	    usbd_start_hardware(xfer);
	}
	return;

 wakeup:
	sc->sc_intr_iwakeup = 0;
	wakeup(&(sc->sc_intr_iwakeup));
	return;
}

/*
 * Interrupt call reply transfer, read
 */
static void
zyd_cfg_usb_intr_read(struct zyd_softc *sc, void *data, uint32_t size)
{
	int error;

	if (size > sizeof(sc->sc_intr_ibuf)) {
	    DPRINTF(sc, 0, "truncating transfer size!\n");
	    size = sizeof(sc->sc_intr_ibuf);
	}

	if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    bzero(data, size);
	    goto done;
	}

	sc->sc_intr_iwakeup = 1;

	bzero(&(sc->sc_intr_ibuf), size);

	usbd_transfer_start(sc->sc_xfer[ZYD_TR_INTR_DT_RD]);

	if (sc->sc_intr_iwakeup) {
	    error = mtx_sleep(&(sc->sc_intr_iwakeup), &(sc->sc_mtx), 0,
			   "zyd isleep", 0);
	}

	bcopy(&(sc->sc_intr_ibuf), data, size);

 done:
	return;
}

static void
zyd_intr_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[ZYD_TR_INTR_DT_WR];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~ZYD_FLAG_INTR_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~ZYD_FLAG_INTR_WRITE_STALL;
	return;
}

static void
zyd_intr_write_callback(struct usbd_xfer *xfer)
{
	struct zyd_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 2, "error=%d\n", xfer->error);

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= ZYD_FLAG_INTR_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_INTR_CS_WR]);
	}
	goto wakeup;

 tr_transferred:
	DPRINTF(sc, 2, "length=%d\n", xfer->actlen);
	goto wakeup;

 tr_setup:

	if (sc->sc_flags & ZYD_FLAG_INTR_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_INTR_CS_WR]);
	    goto wakeup;
	}

	if (sc->sc_intr_owakeup) {
	    usbd_copy_in(&(xfer->buf_data), 0, &(sc->sc_intr_obuf),
			 sc->sc_intr_olen);

	    xfer->length = sc->sc_intr_olen;
	    usbd_start_hardware(xfer);
	}
	return;

 wakeup:
	sc->sc_intr_owakeup = 0;
	wakeup(&(sc->sc_intr_owakeup));
	return;
}

/*
 * Interrupt transfer, write.
 *
 * Not always an "interrupt transfer", as if operating in
 * full speed mode, EP4 is bulk out, not interrupt out.
 */
static void
zyd_cfg_usb_intr_write(struct zyd_softc *sc, void *data, uint32_t size)
{
	int error;

	if (size > sizeof(sc->sc_intr_obuf)) {
	    DPRINTF(sc, 0, "truncating transfer size!\n");
	    size = sizeof(sc->sc_intr_obuf);
	}

	if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto done;
	}

	sc->sc_intr_olen = size;
	sc->sc_intr_owakeup = 1;

	bcopy(data, sc->sc_intr_obuf, size);

	usbd_transfer_start(sc->sc_xfer[ZYD_TR_INTR_DT_WR]);

	if (sc->sc_intr_owakeup) {
	    error = mtx_sleep(&(sc->sc_intr_owakeup), &(sc->sc_mtx), 0,
			   "zyd osleep", 0);
	}

 done:
	return;
}

/*
 * Offset correction (all ranges except CTL use word addressing)
 */
static uint32_t
zyd_addrinc(uint32_t addr)
{
	uint32_t range = ZYD_GET_RANGE(addr);
	uint32_t offs = ZYD_GET_OFFS(addr);

	offs += (range == ZYD_RANGE_CTL) ? 2 : 1;

	return (range | offs);
}

/*
 * Read a single 16-bit register
 */
static void
zyd_cfg_read16(struct zyd_softc *sc, uint32_t addr, uint16_t *value)
{
	zyd_cfg_read16_multi(sc, &addr, value, 1);
	return;
}

/*
 * Read a single 32-bit register
 */
static void
zyd_cfg_read32(struct zyd_softc *sc, uint32_t addr, uint32_t *value)
{
	zyd_cfg_read32_multi(sc, &addr, value, 1);
	return;
}

/*
 * Read up to 15 16-bit registers (newer firmware versions)
 */
static void
zyd_cfg_read16_multi(struct zyd_softc *sc, const uint32_t *addrs, uint16_t *data,
	uint8_t usecount)
{
	struct zyd_intoutmultiread in;
	struct zyd_intinmultioutput op;
	uint8_t i;

	memset(&in, 0, sizeof(struct zyd_intoutmultiread));
	memset(&op, 0, sizeof(struct zyd_intinmultioutput));

	USETW(in.id, ZYD_CMD_IORDREQ);

	for (i = 0; i < usecount; i++)
		USETW(in.addr[i], zyd_getrealaddr(sc, addrs[i]));

	zyd_cfg_usb_intr_write(sc, &in, (2 + (usecount * 2)));
	zyd_cfg_usb_intr_read(sc, &op, (2 + (usecount * 4)));

	for (i = 0; i < usecount; i++) {
		data[i] = UGETW(op.registers[i].data);
	}
	return;
}

/*
 * Read up to 7 32-bit registers (newer firmware versions)
 */
static void
zyd_cfg_read32_multi(struct zyd_softc *sc, const uint32_t *addrs, uint32_t *data,
	uint8_t usecount)
{
	struct zyd_intoutmultiread in;
	struct zyd_intinmultioutput op;
	uint8_t i;
	uint8_t realcount;

	realcount = usecount * 2;

	memset(&in, 0, sizeof(struct zyd_intoutmultiread));
	memset(&op, 0, sizeof(struct zyd_intinmultioutput));

	USETW(in.id, ZYD_CMD_IORDREQ);

	for (i = 0; i < usecount; i++) {
		/* high word is first */
		USETW(in.addr[i * 2], zyd_getrealaddr(sc, zyd_addrinc(addrs[i])));
		USETW(in.addr[(i * 2) + 1], zyd_getrealaddr(sc, addrs[i]));
	}

	zyd_cfg_usb_intr_write(sc, &in, (2 + (realcount * 2)));
	zyd_cfg_usb_intr_read(sc, &op, (2 + (realcount * 4)));

	for (i = 0; i < usecount; i++) {
		data[i] =
		    (UGETW(op.registers[i * 2].data) << 16) |
		    UGETW(op.registers[(i * 2) + 1].data);
	}
	return;
}

/*
 * Write a single 16-bit register
 */
static void
zyd_cfg_write16(struct zyd_softc *sc, uint32_t addr, uint16_t value)
{
	zyd_cfg_write16_multi(sc, &addr, &value, 1);
	return;
}

/*
 * Write a single 32-bit register
 */
static void
zyd_cfg_write32(struct zyd_softc *sc, uint32_t addr, uint32_t value)
{
	zyd_cfg_write32_multi(sc, &addr, &value, 1);
	return;
}

/*
 * Write up to 15 16-bit registers (newer firmware versions)
 */
static void
zyd_cfg_write16_multi(struct zyd_softc *sc, const uint32_t *addrs, uint16_t *data,
	uint8_t usecount)
{
	struct zyd_intoutmultiwrite mw;
	uint8_t i;

	memset(&mw, 0, sizeof(struct zyd_intoutmultiwrite));

	USETW(mw.id, ZYD_CMD_IOWRREQ);

	for (i = 0; i < usecount; i++) {
		USETW(mw.registers[i].addr, zyd_getrealaddr(sc, addrs[i]));
		USETW(mw.registers[i].data, data[i]);
	}

	zyd_cfg_usb_intr_write(sc, &mw, (2 + (usecount * 4)));
	return;
}

/*
 * Write up to 7 32-bit registers (newer firmware versions)
 */
static void
zyd_cfg_write32_multi(struct zyd_softc *sc, const uint32_t *addrs, uint32_t *data,
	uint8_t usecount)
{
	struct zyd_intoutmultiwrite mw;
	uint8_t i;
	uint8_t realcount;

	realcount = usecount * 2;

	memset(&mw, 0, sizeof(struct zyd_intoutmultiwrite));

	USETW(mw.id, ZYD_CMD_IOWRREQ);

	for (i = 0; i < usecount; i++) {
		/* high word is first */
		USETW(mw.registers[i * 2].addr, zyd_getrealaddr(sc, zyd_addrinc(addrs[i])));
		USETW(mw.registers[i * 2].data, (*data >> 16));

		USETW(mw.registers[(i * 2) + 1].addr, zyd_getrealaddr(sc, addrs[i]));
		USETW(mw.registers[(i * 2) + 1].data, (*data));
	}

	zyd_cfg_usb_intr_write(sc, &mw, (2 + (realcount * 4)));
	return;
}

/*
 * Batch write 16-bit data
 */
static void
zyd_cfg_write16_batch(struct zyd_softc *sc, const struct zyd_adpairs16 *data,
	uint32_t count)
{
	/* TODO: Optimize, use multi-writer */
	uint32_t i;

	for (i = 0; i < count; i++) {
		zyd_cfg_write16(sc, data[i].addr, data[i].data);
	}
	return;
}

/*
 * Batch write 32-bit data
 */
static void
zyd_cfg_write32_batch(struct zyd_softc *sc, const struct zyd_adpairs32 *data,
	uint32_t count)
{
	/* TODO: Optimize, use multi-writer */
	uint32_t i;

	for (i = 0; i < count; i++) {
		zyd_cfg_write32(sc, data[i].addr, data[i].data);
	}
	return;
}

/*
 * Write RF registers
 */
static void
zyd_cfg_rfwrite(struct zyd_softc *sc, uint32_t value, uint8_t bits)
{
	struct zyd_req_rfwrite req;
	uint16_t bw_template;
	uint32_t len;
	uint8_t i;

	DPRINTF(sc, 4, "\n");

	if (bits > ZYD_REQ_RFWRITE_BITS_MAX) {
	    DPRINTF(sc, 0, "truncating number of bits!\n");
	    bits = ZYD_REQ_RFWRITE_BITS_MAX;
	}

	zyd_cfg_read16(sc, ZYD_CR203, &bw_template);

	/* Clear template */
	bw_template &= ~(ZYD_RF_IF_LE | ZYD_RF_CLK | ZYD_RF_DATA);

	len = sizeof(req) - sizeof(req.bit_values) + (bits * sizeof(uWord));

	USETW(req.id, ZYD_CMD_RFCFGREQ);
	USETW(req.value, 2);
	USETW(req.bits, bits);

	for (i = 0; i < bits; i++) {
		uint16_t bv = bw_template;

		if (value & (1 << (bits - 1 - i)))
			bv |= ZYD_RF_DATA;

		USETW(req.bit_values[i], bv);
	}

	zyd_cfg_usb_intr_write(sc, &req, len);

	DPRINTF(sc, 4, "wrote %d bits\n", bits);
	return;
}

/*
 * Fetch and print state flags of zydas
 */
static void
zyd_cfg_stateoutput(struct zyd_softc *sc)
{
	uint32_t debug;

	DPRINTF(sc, 0, "In zyd_stateoutput()\n");

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x6D4), &debug);
	DPRINTF(sc, 0, "DEBUG: Tx complete: %x\n", debug);

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x6F4), &debug);
	DPRINTF(sc, 0, "DEBUG: Tx total packet: %x\n", debug);

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x69C), &debug);
	DPRINTF(sc, 0, "DEBUG: Rx timeout count: %x\n", debug);

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x6A0), &debug);
	DPRINTF(sc, 0, "DEBUG: Rx total frame count: %x\n", debug);

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x6A4), &debug);
	DPRINTF(sc, 0, "DEBUG: Rx CRC32: %x\n", debug);

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x6A8), &debug);
	DPRINTF(sc, 0, "DEBUG: Rx CRC16: %x\n", debug);

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x6AC), &debug);
	DPRINTF(sc, 0, "DEBUG: Rx unicast decr error: %x\n", debug);

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x6B0), &debug);
	DPRINTF(sc, 0, "DEBUG: Rx FIFO overrun: %x\n", debug);

	debug = 0;
	zyd_cfg_read32(sc, ZYD_REG_CTL(0x6BC), &debug);
	DPRINTF(sc, 0, "DEBUG: Rx multicast decr error: %x\n", debug);
}

/*
 * RX frame processor.
 *
 * Needed because rxeof might fetch multiple frames
 * inside a single USB transfer.
 */
static void
zyd_rxframeproc(struct usbd_xfer *xfer, uint16_t offset, uint16_t len)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = ic->ic_ifp;
	struct zyd_rxstatusreport desc;
	struct ieee80211_node *ni;
	struct mbuf *m = NULL;

	/* Too small for at least an RX status report? */
	if (len < sizeof(desc)) {
		DPRINTF(sc, -1, "xfer too short, %d bytes\n", len);
		ifp->if_ierrors++;
		goto done;
	}

	len -= sizeof(desc);

	if (len > MCLBYTES) {
		DPRINTF(sc, 0, "invalid length, %d bytes\n", len);
		ifp->if_ierrors++;
		goto done;
	}

	m = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR);

	if (m == NULL) {
		DPRINTF(sc, 0, "could not allocate mbuf\n");
		ifp->if_ierrors++;
		goto done;
	}

	/* copy out data */
	usbd_copy_out(&(xfer->buf_data), offset, m->m_data, len);

	/* copy out status report */
	usbd_copy_out(&(xfer->buf_data), offset + len, &desc, sizeof(desc));

	/*
	 * TODO: Signal strength and quality have to be calculated in
	 * conjunction with the PLCP header! The printed values are RAW!
	 */

	/* Print RX debug info */
	DPRINTF(sc, 3, "Rx status: signalstrength = %d, signalqualitycck = %d, "
	    "signalqualityofdm = %d, decryptiontype = %d, "
	    "modulationtype = %d, rxerrorreason = %d, errorindication = %d\n",
	    desc.signalstrength, desc.signalqualitycck, desc.signalqualityofdm,
	    desc.decryptiontype, desc.modulationtype, desc.rxerrorreason,
	    desc.errorindication);

	/* Bad frame? */
	if (desc.errorindication) {
		DPRINTF(sc, 0, "RX status indicated error\n");
		ifp->if_ierrors++;
		goto done;
	}

	m->m_pkthdr.rcvif = ifp;
	m->m_pkthdr.len = m->m_len = len;
	m->m_flags |= M_HASFCS; /* hardware appends FCS */

	if (sc->sc_drvbpf != NULL) {
	    struct zyd_rx_radiotap_header *tap = &(sc->sc_rxtapu.th);

	    tap->wr_flags = 0;
	    tap->wr_chan_freq = htole16(ic->ic_bss->ni_chan->ic_freq);
	    tap->wr_chan_flags = htole16(ic->ic_bss->ni_chan->ic_flags);
 	    tap->wr_rssi = desc.signalstrength;

	    bpf_mtap2(sc->sc_drvbpf, tap, sc->sc_rxtap_len, m);
	}

	ni = ieee80211_find_rxnode(ic, (void *)(m->m_data));

	/* send the frame to the 802.11 layer */
	ieee80211_input(ic, m, ni, desc.signalstrength, 0);

	/* node is no longer needed */
	ieee80211_free_node(ni);

	DPRINTF(sc, 14, "rx done\n");

	m = NULL;

 done:
	if (m) {
	    m_freem(m);
	}
	return;
}

static void
zyd_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[ZYD_TR_BULK_DT_RD];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~ZYD_FLAG_BULK_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~ZYD_FLAG_BULK_READ_STALL;
	return;
}

static void
zyd_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = ic->ic_ifp;
	struct zyd_rxleninfoapp info;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 0, "frame error: %s\n", usbd_errstr(xfer->error));

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= ZYD_FLAG_BULK_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_BULK_CS_RD]);
	}
	return;

 tr_transferred:
	DPRINTF(sc, 0, "received frame: %d bytes\n", xfer->actlen);

	/*
	 * It must be at least 4 bytes - still broken if it is
	 * 4 bytes, but that's enough to hold the multi-frame
	 * append header
	 */
	if (xfer->actlen < sizeof(info)) {
		DPRINTF(sc, -1, "xfer too short, %d bytes\n", xfer->actlen);
		ifp->if_ierrors++;
		goto tr_setup;
	}

	usbd_copy_out(&(xfer->buf_data), xfer->actlen - sizeof(info),
		      &info, sizeof(info));

	/* See whether this is a multi-frame tansmission */
	if (UGETW(info.marker) == ZYD_MULTIFRAME_MARKER) {
		/* Multiframe received */
		DPRINTF(sc, 3, "Received multi-frame transmission\n");

		/* TODO: Support 'em properly */

		/* Append PLCP header size */
/*		tfs = ZYD_PLCP_HDR_SIZE;

		for (i = 0; i < 3; ++i) {
			uint16_t tfl = UGETW(leninfoapp->len[i]);

			zyd_rxframeproc(data, data->buf + tfs, tfl);
			tfs += tfl;
		}*/

	} else {
		DPRINTF(sc, 3, "Received single-frame transmission\n");

		if (xfer->actlen < ZYD_PLCP_HDR_SIZE) {
		    goto tr_setup;
		}

		zyd_rxframeproc(xfer, ZYD_PLCP_HDR_SIZE, 
				xfer->actlen - ZYD_PLCP_HDR_SIZE);
	}

 tr_setup:
	if (sc->sc_flags & ZYD_FLAG_BULK_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_BULK_CS_RD]);
	    return;
	}

	usbd_start_hardware(xfer);
	return;
}

/*
 * Upload firmware to device.
 *
 * Returns nozero on error.
 *
 * The whole upload procedure was implemented accordingly to
 * what ZyDAS' Linux driver does. It does however *NOT* match
 * what their documentation says (argh...)!
 */
static uint8_t
zyd_cfg_uploadfirmware(struct zyd_softc *sc)
{
	/* ZD1211 uses a proprietary "setup" command to upload the fw */
	struct zyd_control zc;
	uint8_t stsresult;
	uint16_t imgsize;
	const uint8_t *imgptr0;
	uint8_t result = 0;
	union {
	  const uint8_t *imgptr_const;
	  uint8_t *imgptr;
	} u;

	memset(&zc, 0, sizeof(struct zyd_control));
	zc.type = ZYD_FIRMDOWN_REQ;
	zc.id = ZYD_FIRMDOWN_ID;
	zc.value = ZYD_FIRMWARE_START_ADDR; /* TODO: Different on old ones! */

	u.imgptr_const = imgptr0 = zyd_firmware;

	imgsize = sizeof(zyd_firmware);

	DPRINTF(sc, 0, "Firmware upload: imgsize=%d\n", imgsize);

	/* Issue upload command(s) */
	while (imgsize > 0) {
		/* Transfer 4KB max */
		uint16_t tlen = (imgsize > 4096) ? 4096 : imgsize;

		DPRINTF(sc, 0, "Firmware upload: tlen=%d, value=%x\n",
		    tlen, zc.value);

		zc.length = tlen;
		zc.data = u.imgptr;

		zyd_cfg_usbrequestzc(sc, &zc);

		imgsize -= tlen;
		u.imgptr += tlen;

		zc.value += (uint16_t)(tlen / 2); /* Requires word */
	}

	/* See whether the upload succeeded */
	memset(&zc, 0, sizeof(struct zyd_control));
	zc.type = ZYD_FIRMSTAT_REQ;
	zc.id = ZYD_FIRMSTAT_ID;
	zc.value = 0;
	zc.length = 1;
	zc.data = &stsresult;

	zyd_cfg_usbrequestzc(sc, &zc);

	/* Firmware successfully uploaded? */
	if ((stsresult == 0) ||
	    (stsresult == ZYD_FIRMWAREUP_FAILURE)) {
		DPRINTF(sc,-1,"Error: Firmware upload "
		    "failed: 0x%X\n", stsresult);
		result = 1;
	} else {
		DPRINTF(sc, 0, "Firmware successfully "
		    "uploaded\n");
	}
	return result;
}

/*
 * Driver OS interface
 */

/*
 * Probe for a ZD1211-containing product
 */
static int
zyd_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->iface)
		return (UMATCH_NONE);

	return (usb_lookup(zyd_devs, uaa->vendor, uaa->product) != NULL) ?
	    UMATCH_VENDOR_PRODUCT : UMATCH_NONE;
}

/*
 * Attach the interface. Allocate softc structures, do
 * setup and ethernet/BPF attach.
 */
static int
zyd_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct zyd_softc *sc = device_get_softc(dev);
	int error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	if (uaa->release < ZYD_ALLOWED_DEV_VERSION) {
		device_printf(dev, "device version mismatch: 0x%X "
		    "(only >= 43.30 supported)\n",
		    uaa->release);
		return EINVAL;
	}

	usbd_set_desc(dev, uaa->device);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s", 
		 device_get_nameunit(dev));

	sc->sc_unit = device_get_unit(dev);
	sc->sc_udev = uaa->device;

	mtx_init(&sc->sc_mtx, "zyd lock", MTX_NETWORK_LOCK,
		 MTX_DEF | MTX_RECURSE);

	__callout_init_mtx(&(sc->sc_watchdog),
			   &(sc->sc_mtx), CALLOUT_RETURNUNLOCKED);

	__callout_init_mtx(&(sc->sc_scan_callout),
			   &(sc->sc_mtx), CALLOUT_RETURNUNLOCKED);

	error = usbd_set_config_no(uaa->device, ZYD_CONFIG_NO, 1);

	if (error) {
		device_printf(dev, "setting config no failed: %s\n", 
			      usbd_errstr(error));
		goto detach;
	}

	/*
	 * Endpoint 1 = Bulk out (512b @ high speed / 64b @ full speed)
	 * Endpoint 2 = Bulk in  (512b @ high speed / 64b @ full speed)
	 * Endpoint 3 = Intr in (64b)
	 * Endpoint 4 = Intr out @ high speed / bulk out @ full speed (64b)
	 */

	error = usbd_transfer_setup(uaa->device, ZYD_IFACE_IDX,
				    sc->sc_xfer, zyd_config, ZYD_TR_MAX,
				    sc, &(sc->sc_mtx));
	if (error) {
		device_printf(dev, "could not allocate USB "
			      "transfers: %s\n", usbd_errstr(error));
		goto detach;
	}

	error = usbd_config_td_setup(&(sc->sc_config_td), sc, &(sc->sc_mtx),
				     &zyd_end_of_commands,
				     sizeof(struct zyd_config_copy), 16);
	if (error) {
	    device_printf(dev, "could not setup config "
			  "thread!\n");
	    goto detach;
	}

	mtx_lock(&(sc->sc_mtx));

	/* start setup */

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL, &zyd_cfg_first_time_setup, 0, 0);

	/* start watchdog (will exit mutex) */

	zyd_watchdog(sc);

	return 0;

 detach:
	zyd_detach(dev);
	return ENXIO;
}

/*
 * Lock PHY registers
 */
static void
zyd_cfg_lock_phy(struct zyd_softc *sc)
{
	uint32_t temp;

	zyd_cfg_read32(sc, ZYD_MAC_MISC, &temp);
	temp &= ~ZYD_UNLOCK_PHY_REGS;
	zyd_cfg_write32(sc, ZYD_MAC_MISC, temp);
}

/*
 * Unlock PHY registers
 */
static void
zyd_cfg_unlock_phy(struct zyd_softc *sc)
{
	uint32_t temp;

	zyd_cfg_read32(sc, ZYD_MAC_MISC, &temp);
	temp |= ZYD_UNLOCK_PHY_REGS;
	zyd_cfg_write32(sc, ZYD_MAC_MISC, temp);
}

/*
 * Helper beacon (get)
 */
static void
zyd_cfg_get_aw_pt_bi(struct zyd_softc *sc, struct zyd_aw_pt_bi *s)
{
	static const uint32_t addrs[] =
	    { ZYD_CR_ATIM_WND_PERIOD, ZYD_CR_PRE_TBTT, ZYD_CR_BCN_INTERVAL };
	uint32_t values[3];

	zyd_cfg_read32_multi(sc, addrs, values, 3);

	s->atim_wnd_period = values[0];
	s->pre_tbtt = values[1];
	s->beacon_interval = values[2];
	DPRINTF(sc, 0, "aw %u pt %u bi %u\n",
		s->atim_wnd_period, s->pre_tbtt, s->beacon_interval);
	return;
}

/*
 * Helper beacon (set)
 */
static void
zyd_cfg_set_aw_pt_bi(struct zyd_softc *sc, struct zyd_aw_pt_bi *s)
{
	static const uint32_t addrs[] =
	    { ZYD_CR_ATIM_WND_PERIOD, ZYD_CR_PRE_TBTT, ZYD_CR_BCN_INTERVAL };
	uint32_t data[3];

	if (s->beacon_interval <= 5)
		s->beacon_interval = 5;

	if (s->pre_tbtt < 4 || s->pre_tbtt >= s->beacon_interval)
		s->pre_tbtt = s->beacon_interval - 1;

	if (s->atim_wnd_period >= s->pre_tbtt)
		s->atim_wnd_period = s->pre_tbtt - 1;

	data[0] = s->atim_wnd_period;
	data[1] = s->pre_tbtt;
	data[2] = s->beacon_interval;

	zyd_cfg_write32_multi(sc, addrs, data, 3);
	return;
}

/*
 * Set beacon interval
 */
static void
zyd_cfg_set_beacon_interval(struct zyd_softc *sc, uint32_t interval)
{
	struct zyd_aw_pt_bi s;

	zyd_cfg_get_aw_pt_bi(sc, &s);

	s.beacon_interval = interval;

	zyd_cfg_set_aw_pt_bi(sc, &s);

	return;
}

/*
 * Get RF name
 */
static const char *
zyd_rf_name(uint8_t type)
{
	if (type & 0xf0)
		type = 0;

	return zyd_rfs[type];
}

/*
 * Read RF PA types
 */
static void
zyd_cfg_read_rf_pa_types(struct zyd_softc *sc, uint8_t *rf_type,
	uint8_t *pa_type)
{
	uint32_t value;

	zyd_cfg_read32(sc, ZYD_E2P_POD, &value);

	*rf_type = value & 0x0f;
	*pa_type = (value >> 16) & 0x0f;

	return;
}

/*
 * RF driver: Init for RFMD chip
 */
static void
zyd_cfg_rf_rfmd_init(struct zyd_softc *sc, struct zyd_rf *rf)
{
	uint32_t i;

	DPRINTF(sc, 0, "ir1 = %d, ir2 = %d\n",
	    (sizeof(zyd_rfmd_cr) / sizeof(struct zyd_adpairs16)),
	    (sizeof(zyd_rfmd_rf) / sizeof(uint32_t)));

	zyd_cfg_write16_batch(sc, zyd_rfmd_cr, (sizeof(zyd_rfmd_cr) /
	    sizeof(struct zyd_adpairs16)));

	for (i = 0; i < (sizeof(zyd_rfmd_rf) / sizeof(uint32_t)); i++) {
		zyd_cfg_rfwrite(sc, zyd_rfmd_rf[i], ZYD_RF_RV_BITS);
	}
	return;
}

/*
 * RF driver: Switch radio on/off for RFMD chip
 */
static void
zyd_cfg_rf_rfmd_switchradio(struct zyd_softc *sc, uint8_t onoff)
{
	static const struct zyd_adpairs16 ir_on[] = {
		ZYD_RFMD_RADIO_ON
	};

	static const struct zyd_adpairs16 ir_off[] = {
		ZYD_RFMD_RADIO_OFF
	};

	if (onoff) {
		zyd_cfg_write16_batch(sc, ir_on,
		    (sizeof(ir_on) / sizeof(struct zyd_adpairs16)));
		return;
	}

	zyd_cfg_write16_batch(sc, ir_off, (sizeof(ir_off) /
	    sizeof(struct zyd_adpairs16)));
	return;
}

/*
 * RF driver: Channel setting for RFMD chip
 */
static void
zyd_cfg_rf_rfmd_set_channel(struct zyd_softc *sc, struct zyd_rf *rf,
	uint8_t channel)
{
	static const uint32_t rfmd_table[][2] = {
		ZYD_RFMD_CHANTABLE
	};

	const uint32_t *dp;
	uint8_t i;

	dp = rfmd_table[channel - 1];

	for (i = 0; i < 2; i++) {
		zyd_cfg_rfwrite(sc, dp[i], ZYD_RF_RV_BITS);
	}
	return;
}

/*
 * RF driver: Switch radio on/off for AL2230 chip
 */
static void
zyd_cfg_rf_al2230_switchradio(struct zyd_softc *sc, uint8_t onoff)
{
	static const struct zyd_adpairs16 ir_on[] = {
		ZYD_AL2230_RADIO_ON
	};

	static const struct zyd_adpairs16 ir_off[] = {
		ZYD_AL2230_RADIO_OFF
	};

	if (onoff) {
		zyd_cfg_write16_batch(sc, ir_on,
		    (sizeof(ir_on) / sizeof(struct zyd_adpairs16)));
		return;
	}

	zyd_cfg_write16_batch(sc, ir_off, (sizeof(ir_off) /
	    sizeof(struct zyd_adpairs16)));
	return;
}

/*
 * RF driver: Init for AL2230 chip
 */
static void
zyd_cfg_rf_al2230_init(struct zyd_softc *sc, struct zyd_rf *rf)
{
	uint32_t i;

	zyd_cfg_write16_batch(sc, zyd_al2230_cr, (sizeof(zyd_al2230_cr) /
	    sizeof(struct zyd_adpairs16)));

	for (i = 0; i < (sizeof(zyd_al2230_rf) / sizeof(uint32_t)); i++) {
		zyd_cfg_rfwrite(sc, zyd_al2230_rf[i], ZYD_RF_RV_BITS);
	}
	return;
}

/*
 * RF driver: Channel setting for AL2230 chip
 */
static void
zyd_cfg_rf_al2230_set_channel(struct zyd_softc *sc, struct zyd_rf *rf,
	uint8_t channel)
{
	static const struct zyd_adpairs16 sc_cmd[] = {
		ZYD_AL2230_SETCHANNEL
	};

	static const uint32_t al2230_table[][3] = {
		ZYD_AL2230_CHANTABLE
	};
	
	uint8_t i;
	const uint32_t *ptr = al2230_table[channel - 1];
	
	for (i = 0; i < 3; i++) {
		zyd_cfg_rfwrite(sc, *ptr, ZYD_RF_RV_BITS);
		ptr++;
	}
	
	zyd_cfg_write16_batch(sc, sc_cmd, (sizeof(sc_cmd) /
	    sizeof(struct zyd_adpairs16)));
	return;
}

/*
 * Assign drivers and init the RF
 */
static uint8_t
zyd_cfg_rf_init_hw(struct zyd_softc *sc, struct zyd_rf *rf, uint8_t type)
 {
	switch (type) {
	case ZYD_RF_RFMD:
		rf->cfg_init_hw = &zyd_cfg_rf_rfmd_init;
		rf->cfg_switch_radio = &zyd_cfg_rf_rfmd_switchradio;
		rf->cfg_set_channel = &zyd_cfg_rf_rfmd_set_channel;
		break;

	case ZYD_RF_AL2230:
		rf->cfg_init_hw = &zyd_cfg_rf_al2230_init;
		rf->cfg_switch_radio = &zyd_cfg_rf_al2230_switchradio;
		rf->cfg_set_channel = &zyd_cfg_rf_al2230_set_channel;
		break;

	default:
		printf("%s: Sorry, radio %s is not supported yet\n",
		    sc->sc_name, zyd_rf_name(type));
		rf->type = 0;
		return 1;
	}

	rf->flags = 0;
	rf->type = type;

	zyd_cfg_lock_phy(sc);
	(rf->cfg_init_hw)(sc, rf);
	zyd_cfg_unlock_phy(sc);

	return 0; /* success */
}

/*
 * Init the hardware
 */
static uint8_t
zyd_cfg_hw_init(struct zyd_softc *sc, struct ieee80211com *ic)
{
	uint8_t rf;

	zyd_cfg_write32(sc, ZYD_MAC_AFTER_PNP, 1);

	zyd_cfg_read16(sc, ZYD_REG_USB(ZYD_FIRMWARE_BASE_ADDR),
	    &(sc->sc_firmware_base));

	DPRINTF(sc, 0, "firmware_base = 0x%04X\n",
	    sc->sc_firmware_base);

	/* Print the firmware version */
	zyd_cfg_read16(sc, ZYD_FW_FIRMWARE_VER, &sc->sc_fw_ver);

	zyd_cfg_write32(sc, ZYD_CR_GPI_EN, 0);

	zyd_cfg_write32(sc, ZYD_MAC_CONT_WIN_LIMIT, 0x007f043f);

	zyd_cfg_set_mandatory_rates(sc, ic->ic_curmode);

	zyd_cfg_disable_hwint(sc);

	/* PHY init ("reset") */
	zyd_cfg_lock_phy(sc);
	zyd_cfg_write16_batch(sc, zyd_def_cr,
	    (sizeof(zyd_def_cr) / sizeof(struct zyd_adpairs16)));
	zyd_cfg_unlock_phy(sc);

	/* HMAC init */
	zyd_cfg_write32_batch(sc, zyd_def_mac,
	    (sizeof(zyd_def_mac) / sizeof(struct zyd_adpairs32)));

	/* RF/PA types */
	zyd_cfg_read_rf_pa_types(sc, &rf, &sc->sc_pa_ver);

	/* Now init the RF chip */
	if (zyd_cfg_rf_init_hw(sc, &sc->sc_rf, rf)) {
	    return 1; /* failure */
	}

	/* Init beacon to 100 * 1024 µs */
	zyd_cfg_set_beacon_interval(sc, 100);

	return 0; /* success */
}

/*
 * Get MAC address from EEPROM
 */
static void
zyd_cfg_get_e2p_mac_addr(struct zyd_softc *sc, struct zyd_macaddr *mac_addr)
{
	uint32_t mac[2];

	zyd_cfg_read32(sc, ZYD_E2P_MAC_ADDR_P1, &mac[0]);
	zyd_cfg_read32(sc, ZYD_E2P_MAC_ADDR_P2, &mac[1]);

	mac_addr->addr[0] = mac[0];
	mac_addr->addr[1] = mac[0] >>  8;
	mac_addr->addr[2] = mac[0] >> 16;
	mac_addr->addr[3] = mac[0] >> 24;
	mac_addr->addr[4] = mac[1];
	mac_addr->addr[5] = mac[1] >>  8;
	return;
}

/*
 * Set MAC address (will accept ANY address)
 */
static void
zyd_cfg_set_mac_addr(struct zyd_softc *sc, const uint8_t *addr)
{
	uint32_t addrs[2];
	uint32_t trans[2];

	addrs[0] = ZYD_MAC_MACADDRL;
	addrs[1] = ZYD_MAC_MACADDRH;

	trans[0] = (
	    (addr[3] << 24) | (addr[2] << 16) |
	    (addr[1] << 8) | (addr[0]));

	trans[1] = (
	    (addr[5] << 8) | (addr[4]));

	zyd_cfg_write32_multi(sc, addrs, trans, 2);
	return;
}

/*
 * Read regdomain
 */
static void
zyd_cfg_read_regdomain(struct zyd_softc *sc, uint8_t *regdomain)
{
	uint32_t value;

	zyd_cfg_read32(sc, ZYD_E2P_SUBID, &value);

	*regdomain = value >> 16;

	return;
}

/*
 * Check whether a particular regdomain is supported
 */
static uint8_t
zyd_regdomain_supported(uint8_t regdomain)
{
	const struct zyd_channel_range *range;

	range = &zyd_channel_ranges[0];

	for ( ; ; ) {
		if (range->regdomain == regdomain)
			return (range->start != 0);
		else if (range->regdomain == -1)
			break; /* end of list */

		range++;
	}
	return 0;
}

/*
 * Helper used by all table readers
 */
static void
zyd_cfg_tblreader(struct zyd_softc *sc, uint8_t *values, size_t count,
	uint32_t e2p_addr, uint32_t guard)
{
	uint32_t i;
	uint32_t v;

	for (i = 0;;) {
		zyd_cfg_read32(sc, (e2p_addr + (i / 2)), &v);

		v -= guard;

		if ((i+4) < count) {
			values[i++] = v;
			values[i++] = v >>  8;
			values[i++] = v >> 16;
			values[i++] = v >> 24;
			continue;
		}

		for (;i < count; i++)
			values[i] = v >> (8*(i%3));

		return;
	}

	return;
}

/*
 * Read calibration tables
 */
static void
zyd_cfg_readcaltables(struct zyd_softc *sc)
{
	uint8_t i;

	static const uint32_t addresses[] = {
		ZYD_E2P_36M_CAL_VALUE1,
		ZYD_E2P_48M_CAL_VALUE1,
		ZYD_E2P_54M_CAL_VALUE1,
	};

	zyd_cfg_tblreader(sc, sc->sc_pwr_cal_values,
	    ZYD_E2P_CHANNEL_COUNT, ZYD_E2P_PWR_CAL_VALUE1, 0);
		
	zyd_cfg_tblreader(sc, sc->sc_pwr_int_values,
	    ZYD_E2P_CHANNEL_COUNT, ZYD_E2P_PWR_INT_VALUE1, ZYD_E2P_PWR_INT_GUARD);

	for (i = 0; i < 3; i++) {
		zyd_cfg_tblreader(sc, sc->sc_ofdm_cal_values[i],
		    ZYD_E2P_CHANNEL_COUNT, addresses[i], 0);
	}
	return;
}

/*
 * Reset channel
 */
static uint8_t
zyd_reset_channel(struct zyd_softc *sc)
{
	const struct zyd_channel_range *range;

	range = &zyd_channel_ranges[0];

	for ( ; ; ) {
		if (range->regdomain == sc->sc_regdomain)
			if (range->start == 0)
				return 1;
			else
			{
				sc->sc_channel = range->start;
				sc->sc_mac_flags &= ~ZMF_FIXED_CHANNEL;
			}
		else if (range->regdomain == -1)
			return 1; /* end of list */

		range++;
	}
	return 0;
}

/*
 * Set encryption type
 */
static void
zyd_cfg_set_encryption_type(struct zyd_softc *sc, uint32_t type)
{
	zyd_cfg_write32(sc, ZYD_MAC_ENCRYPTION_TYPE, type);
	return;
}

/*
 * Switch radio on/off
 */
static void
zyd_cfg_switch_radio(struct zyd_softc *sc, uint8_t onoff)
{
	zyd_cfg_lock_phy(sc);
	(sc->sc_rf.cfg_switch_radio)(sc, onoff);
	zyd_cfg_unlock_phy(sc);

	return;
}

/*
 * Enable hardware interrupt
 */
static void
zyd_cfg_enable_hwint(struct zyd_softc *sc)
{
	zyd_cfg_write32(sc, ZYD_CR_INTERRUPT, ZYD_HWINT_ENABLED);
	return;
}

/*
 * Disable hardware interrupt
 */
static void
zyd_cfg_disable_hwint(struct zyd_softc *sc)
{
	zyd_cfg_write32(sc, ZYD_CR_INTERRUPT, ZYD_HWINT_DISABLED);
	return;
}

/*
 * Set basic rates
 */
static void
zyd_cfg_set_basic_rates(struct zyd_softc *sc, int mode)
{
	/* Do not request high rates for the basic set */
	uint32_t outf = 0;

	switch (mode) {
	case IEEE80211_MODE_11B:
		/* 11B: 1, 2 MBPS */
		outf = 3;
		break;

	case IEEE80211_MODE_11G:
		/* 11G: 6, 12, 24 MBPS */
		outf = (21 << 8);
		break;

	default:
		return;
	}

	zyd_cfg_write32(sc, ZYD_MAC_BASICRATE, outf);
	return;
}

/*
 * Set mandatory rates. This is the full spectrum of a certain mode.
 */
static void
zyd_cfg_set_mandatory_rates(struct zyd_softc *sc, int mode)
{
	uint32_t outf = 0;

	switch (mode) {
	case IEEE80211_MODE_11B:
		/* 11B: 1, 2, 5.5, 11 */
		outf = CSF_RT_CCK_1 | CSF_RT_CCK_2 | CSF_RT_CCK_5_5 | CSF_RT_CCK_11;
		break;

	case IEEE80211_MODE_11G:
		/* 11G: 6, 9, 12, 18, 24, 36, 48, 54 */
		outf = CSF_RT_OFDM_6 | CSF_RT_OFDM_9 | CSF_RT_OFDM_12 |
		    CSF_RT_OFDM_18 | CSF_RT_OFDM_24 | CSF_RT_OFDM_36 |
		    CSF_RT_OFDM_48 | CSF_RT_OFDM_54;
		break;

	default:
		return;
	}

	zyd_cfg_write32(sc, ZYD_MAC_MANDATORYRATE, outf);
	return;
}

/*
 * Reset mode
 */
static void
zyd_cfg_reset_mode(struct zyd_softc *sc)
{
	static const struct zyd_adpairs32 io[3] = {
		{ ZYD_MAC_STOHOSTSETTING, STH_BCN | STH_PRB_RSP | STH_AUTH | STH_ASS_RSP },
		{ ZYD_MAC_SNIFFER, 0U },
		{ ZYD_MAC_ENCRYPTION_TYPE, 0U }
	};
/*
	if (ieee->iw_mode == IW_MODE_MONITOR) {
		ioreqs[0].value = 0xffffffff;
		ioreqs[1].value = 0x1;
		ioreqs[2].value = ENC_SNIFFER;
	}*/

	DPRINTF(sc, 0, "\n");

	zyd_cfg_write32_batch(sc, io, 3);
	return;
}

/*
 * Set BSSID
 */
static void
zyd_cfg_set_bssid(struct zyd_softc *sc, uint8_t *addr)
{
	uint32_t addrh;
	uint32_t addrl;

	addrh = (UGETDW(addr) >> 16);
	addrl = UGETDW(addr + 2);

	DPRINTF(sc, 0, "Setting BSSID addrh = %x, addrl = %x\n",
	    addrh, addrl);

	zyd_cfg_write32(sc, ZYD_MAC_BSSADRL, addrl);
	zyd_cfg_write32(sc, ZYD_MAC_BSSADRH, addrh);
	return;
}

/*
 * Complete the attach process
 */
static void
zyd_cfg_first_time_setup(struct zyd_softc *sc,
			 struct zyd_config_copy *cc, uint16_t refcount)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp;
	struct zyd_macaddr mac;
	uint32_t i;

	if (zyd_cfg_uploadfirmware(sc)) {
	    printf("%s: could not "
		   "upload firmware!\n", sc->sc_name);
	    return;
	}

	/* Perform a device reset */
	zyd_cfg_reset(sc);

	/* Init hardware */
	if (zyd_cfg_hw_init(sc, ic)) {
	    printf("%s: hw_init() failed!\n",
		   sc->sc_name);
	    return;
	}

	/* Read MAC from EEPROM and copy to interface */
	zyd_cfg_get_e2p_mac_addr(sc, &mac);
	memcpy(&sc->sc_ic.ic_myaddr, &mac, IEEE80211_ADDR_LEN);

	printf("%s: Firmware 0x%04X, Radio %s, PA %#01x, address %02x:%02x:%02x:%02x:%02x:%02x\n",
	       sc->sc_name, sc->sc_fw_ver, zyd_rf_name(sc->sc_rf.type),
	       sc->sc_pa_ver, sc->sc_ic.ic_myaddr[0],
	       sc->sc_ic.ic_myaddr[1], sc->sc_ic.ic_myaddr[2],
	       sc->sc_ic.ic_myaddr[3], sc->sc_ic.ic_myaddr[4],
	       sc->sc_ic.ic_myaddr[5]);

	/* Read calibration tables from EEPROM */
	zyd_cfg_readcaltables(sc);

	DPRINTF(sc, 0, "Loading regdomain\n");

	/* Load the regdomain and see whether it is supported */
	zyd_cfg_read_regdomain(sc, &sc->sc_default_regdomain);

	if (!zyd_regdomain_supported(sc->sc_default_regdomain)) {
		printf("%s: Error: Regulatory Domain %#04x is not supported.",
		    sc->sc_name, sc->sc_default_regdomain);
		return;
	}

	sc->sc_regdomain = sc->sc_default_regdomain;

	sc->sc_encrypt = ENC_NOWEP;

	/* TODO: Is this an allowed channel in the domain? */
	sc->sc_channel = ZYD_DEFAULT_CHANNEL;
	sc->sc_operation = OM_INFRASTRUCTURE;

	mtx_unlock(&(sc->sc_mtx));

	ifp = if_alloc(IFT_ETHER);

	mtx_lock(&(sc->sc_mtx));

	if (ifp == NULL) {
	    printf("%s: could not if_alloc()!\n",
		   sc->sc_name);
	    goto done;
	}

	sc->sc_evilhack = ifp;
	sc->sc_ifp = ifp;

	ifp->if_softc = sc;
	if_initname(ifp, "zyd", sc->sc_unit);
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_init = &zyd_init_cb;
	ifp->if_ioctl = &zyd_ioctl_cb;
	ifp->if_start = &zyd_start_cb;
	ifp->if_watchdog = NULL;
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
	ifp->if_snd.ifq_drv_maxlen = IFQ_MAXLEN;
	IFQ_SET_READY(&ifp->if_snd);
	ifp->if_mtu = ZYD_DEFAULT_MTU;

	/* Network interface setup */
	ic->ic_ifp = ifp;
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_state = IEEE80211_S_INIT;

	/* Set device capabilities */
	ic->ic_caps = IEEE80211_C_MONITOR | IEEE80211_C_IBSS |
	    IEEE80211_C_HOSTAP | IEEE80211_C_SHPREAMBLE | IEEE80211_C_PMGT |
	    IEEE80211_C_TXPMGT | IEEE80211_C_WEP;

	/* Rates are in 0,5 MBps units */
	ic->ic_sup_rates[IEEE80211_MODE_11B] = zyd_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = zyd_rateset_11g;

	/* set supported .11b and .11g channels (1 through 14) */
	for (i = 1; i <= 14; i++) {
		ic->ic_channels[i].ic_freq =
		    ieee80211_ieee2mhz(i, IEEE80211_CHAN_2GHZ);
		ic->ic_channels[i].ic_flags =
		    IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
		    IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ;
	}

	mtx_unlock(&(sc->sc_mtx));

	ieee80211_ifattach(ic);

	mtx_lock(&(sc->sc_mtx));

	ic->ic_reset = &zyd_reset_cb;

	sc->sc_newstate = ic->ic_newstate;
	ic->ic_newstate = &zyd_newstate_cb;

	mtx_unlock(&(sc->sc_mtx));

	/* setup ifmedia interface */
	ieee80211_media_init(ic, &zyd_media_change_cb, &ieee80211_media_status);

	bpfattach2(ifp, DLT_IEEE802_11_RADIO,
		   sizeof(struct ieee80211_frame) + 64, &sc->sc_drvbpf);

	mtx_lock(&(sc->sc_mtx));

	sc->sc_rxtap_len = sizeof(sc->sc_rxtapu);
	sc->sc_rxtapu.th.wr_ihdr.it_len = htole16(sc->sc_rxtap_len);
	sc->sc_rxtapu.th.wr_ihdr.it_present = htole32(ZYD_RX_RADIOTAP_PRESENT);
	
	sc->sc_txtap_len = sizeof(sc->sc_txtapu);
	sc->sc_txtapu.th.wt_ihdr.it_len = htole16(sc->sc_txtap_len);
	sc->sc_txtapu.th.wt_ihdr.it_present = htole32(ZYD_TX_RADIOTAP_PRESENT);

	if (bootverbose) {
	    ieee80211_announce(ic);
	}
 done:
	return;
}

/*
 * Detach device
 */
static int
zyd_detach(device_t dev)
{
	struct zyd_softc *sc = device_get_softc(dev);
	struct ieee80211com *ic;
	struct ifnet *ifp;

	usbd_config_td_stop(&(sc->sc_config_td));

	mtx_lock(&(sc->sc_mtx));

	__callout_stop(&(sc->sc_watchdog));
	__callout_stop(&(sc->sc_scan_callout));

	zyd_cfg_pre_stop(sc, NULL, 0);

	ic = &(sc->sc_ic);
	ifp = ic->ic_ifp;

	mtx_unlock(&(sc->sc_mtx));

	/* get rid of any late children */
	bus_generic_detach(dev);

	if (ifp) {
	    bpfdetach(ifp);
	    ieee80211_ifdetach(ic);
	    if_free(ifp);
	}

	usbd_transfer_unsetup(sc->sc_xfer, ZYD_TR_MAX);

	usbd_config_td_unsetup(&(sc->sc_config_td));

	__callout_drain(&(sc->sc_watchdog));
	__callout_drain(&(sc->sc_scan_callout));

	mtx_destroy(&sc->sc_mtx);

	return 0;
}

static int
zyd_media_change_cb(struct ifnet *ifp)
{
	struct zyd_softc *sc = ifp->if_softc;
	int error;

	mtx_lock(&(sc->sc_mtx));

	error = ieee80211_media_change(ifp);
	if (error != ENETRESET) {
	    goto done;
	} else {
	    error = 0;
	}

	if ((ifp->if_flags & IFF_UP) &&
	    (ifp->if_drv_flags & IFF_DRV_RUNNING)) {
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &zyd_cfg_pre_init, 
	       &zyd_cfg_init, 0, 0);
	}

 done:
	mtx_unlock(&(sc->sc_mtx));

	return error;
}

static int
zyd_newstate_cb(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	struct zyd_softc *sc = ic->ic_ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	DPRINTF(sc,0, "setting new state %d\n", nstate);

	/* stop timers */

	__callout_stop(&(sc->sc_scan_callout));

	if (nstate != IEEE80211_S_INIT) {
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &zyd_config_copy,
	       &zyd_cfg_set_chan, 0, 0);
	}

	switch (nstate) {
	case IEEE80211_S_INIT:
		break;

	case IEEE80211_S_SCAN:
		__callout_reset(&(sc->sc_scan_callout), hz/5, 
				&zyd_next_scan, sc);
		break;

	case IEEE80211_S_AUTH:
	case IEEE80211_S_ASSOC:
		break;

	case IEEE80211_S_RUN:
		usbd_config_td_queue_command
		  (&(sc->sc_config_td), &zyd_config_copy,
		   &zyd_cfg_set_run, 0, 0);
		break;
	}

	sc->sc_newstate(ic, nstate, -1);

	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

static void
zyd_cfg_set_run(struct zyd_softc *sc,
		struct zyd_config_copy *cc, uint16_t refcount)
{
	if (cc->ic_opmode != IEEE80211_M_MONITOR)
	    zyd_cfg_set_bssid(sc, cc->ic_bss.ni_bssid);

	if (cc->ic_opmode == IEEE80211_M_HOSTAP ||
	    cc->ic_opmode == IEEE80211_M_IBSS) {

	    sc->sc_flags |= ZYD_FLAG_TX_BEACON;
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_BULK_DT_WR]);
	}
	return;
}

static void
zyd_cfg_update_promisc(struct zyd_softc *sc,
		       struct zyd_config_copy *cc, uint16_t refcount)
{
	return;
}

/*
 * Compute the duration (in us) needed to transmit `len' bytes at rate `rate'.
 * The function automatically determines the operating mode depending on the
 * given rate. `flags' indicates whether short preamble is in use or not.
 */
static uint16_t
zyd_txtime(uint16_t len, uint8_t rate, uint32_t flags)
{
	uint16_t txtime;
	int ceil, dbps;

	if (rate < 2) {
	    rate = 2; /* avoid division by zero */
	}

	if (ZYD_RATE_IS_OFDM(rate)) {
		/*
		 * OFDM TXTIME calculation.
		 * From IEEE Std 802.11a-1999, pp. 37.
		 */
		dbps = rate * 2; /* data bits per OFDM symbol */

		ceil = (16 + (8 * len) + 6) / dbps;

		if (((16 + (8 * len) + 6) % dbps) != 0)
			ceil++;

		txtime = 16 + 4 + (4 * ceil) + 6;
	} else {
		/*
		 * High Rate TXTIME calculation.
		 * From IEEE Std 802.11b-1999, pp. 28.
		 */
		ceil = (8 * len * 2) / rate;

		if (((8 * len * 2) % rate) != 0)
			ceil++;

		if ((rate != 2) && (flags & IEEE80211_F_SHPREAMBLE))
			txtime =  72 + 24 + ceil;
		else
			txtime = 144 + 48 + ceil;
	}
	return txtime;
}

/*
 * Rate-to-bit-converter (Field "rate" in zyd_controlsetformat)
 */
static uint8_t
zyd_plcp_signal(uint8_t rate)
{
	switch (rate) {
	/* CCK rates */
	case 2:		return 0x0;
	case 4:		return 0x1;
	case 11:	return 0x2;
	case 22:	return 0x3;

	/* OFDM rates (cf IEEE Std 802.11a-1999, pp. 14 Table 80) */
	case 12:	return 0xb;
	case 18:	return 0xf;
	case 24:	return 0xa;
	case 36:	return 0xe;
	case 48:	return 0x9;
	case 72:	return 0xd;
	case 96:	return 0x8;
	case 108:	return 0xc;

	/* unsupported rates (should not get there) */
	default:	return 0xff;
	}
}
/*
static int
zyd_calc_useclen2(uint8_t *service, uint8_t cs_rate, uint16_t tx_length)
{
	static const uint8_t rate_divisor[] = {
		[ZD_CS_CCK_RATE_1M]	=  1,
		[ZD_CS_CCK_RATE_2M]	=  2,
		[ZD_CS_CCK_RATE_5_5M]	= 11, // bits must be doubled
		[ZD_CS_CCK_RATE_11M]	= 11,
		[ZD_OFDM_RATE_6M]	=  6,
		[ZD_OFDM_RATE_9M]	=  9,
		[ZD_OFDM_RATE_12M]	= 12,
		[ZD_OFDM_RATE_18M]	= 18,
		[ZD_OFDM_RATE_24M]	= 24,
		[ZD_OFDM_RATE_36M]	= 36,
		[ZD_OFDM_RATE_48M]	= 48,
		[ZD_OFDM_RATE_54M]	= 54,
	};

	uint32_t bits = (uint32_t)tx_length * 8;
	uint32_t divisor;

	divisor = rate_divisor[cs_rate];
	if (divisor == 0)
		return -EINVAL;

	switch (cs_rate) {
	case ZD_CS_CCK_RATE_5_5M:
		bits = (2*bits) + 10; // round up to the next integer
		break;
	case ZD_CS_CCK_RATE_11M:
		if (service) {
			uint32_t t = bits % 11;
			*service &= ~ZD_PLCP_SERVICE_LENGTH_EXTENSION;
			if (0 < t && t <= 3) {
				*service |= ZD_PLCP_SERVICE_LENGTH_EXTENSION;
			}
		}
		bits += 10; // round up to the next integer
		break;
	}

	return bits/divisor;
}

enum {
	R2M_SHORT_PREAMBLE = 0x01,
	R2M_11A		   = 0x02,
};
*/

/*
 * Calculate frame transmit length in microseconds
 */
static uint16_t
zyd_calc_useclen(uint8_t rate, uint16_t len, uint8_t *service)
{
	uint32_t remainder;
	uint32_t delta;
	uint16_t leninus;

	leninus = 0;
	*(service) = 0;

	switch (rate) {
	case 2:	/* 1M bps */
		leninus = len << 3;
		break;

	case 4:	/* 2M bps */
		leninus = len << 2;
		break;

	case 11: /* 5.5M bps */
		leninus = (uint16_t)(((uint32_t)len << 4) / 11);
		remainder = (((uint32_t)len << 4) % 11);

		if (remainder)
			leninus += 1;
		break;

	case 22: /* 11M bps */
		leninus = (uint16_t)(((uint32_t)len << 3) / 11);
		remainder = (((uint32_t)len << 3) % 11);
		delta = 11 - remainder;

		if (remainder) {
			leninus += 1;
			if (delta >= 8)
				*(service) |= 0x80; /* Bit 7 */
		}
		break;

	case 12:/* 6M */
		leninus = (uint16_t)(((uint32_t)len << 3) / 6);
		break;

	case 18:/* 9M */
		leninus = (uint16_t)(((uint32_t)len << 3) / 9);
		break;

	case 24:/* 12M */
		leninus = (uint16_t)(((uint32_t)len << 3) / 12);
		break;

	case 36:/* 18M */
		leninus = (uint16_t)(((uint32_t)len << 3) / 18);
		break;

	case 48:/* 24M */
		leninus = (uint16_t)(((uint32_t)len << 3) / 24);
		break;

	case 72:/* 36M */
		leninus = (uint16_t)(((uint32_t)len << 3) / 36);
		break;

	case 96:/* 48M */
		leninus = (uint16_t)(((uint32_t)len << 3) / 48);
		break;

	case 108: /* 54M */
		leninus = (uint16_t)(((uint32_t)len << 3) / 54);
		break;
	}

	return leninus;
}

/*
 * Setup the controlsetformat structure
 */
static void
zyd_setup_tx_desc(struct usbd_xfer *xfer, struct mbuf *m, 
	uint16_t rate)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh = mtod(m, struct ieee80211_frame *);
	struct zyd_controlsetformat desc;
	uint8_t more_frag = wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG;
	uint8_t type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	uint8_t subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
	uint16_t txlen;
	uint16_t len = m->m_pkthdr.len;

	DPRINTF(sc, 0, "enter\n");

	bzero(&desc, sizeof(desc));

	/* Rate (CCK and OFDM) */
	desc.rate = zyd_plcp_signal(rate);

	/* Modulation type (CCK/OFDM) */
	if (ZYD_RATE_IS_OFDM(rate))
		desc.modulationtype = CSF_MT_OFDM;
	else
		desc.modulationtype = CSF_MT_CCK;

	/* Preamble/a/g (depending on modtype) */
	if (desc.modulationtype == CSF_MT_CCK) {
		if (ic->ic_flags & IEEE80211_F_SHPREAMBLE)
			desc.preamble = CSF_PM_CCK_SHORT;
	}

	// DEBUG!
	desc.preamble = 0;

	/*
	 * Transmit frame length in bytes:
	 * 802.11 MAC header length + raw data length
	 * + ICV/(MIC) length + FCS length.
	 */
	txlen = len; /* + 4;*/
	desc.txlen = htole16(txlen);

	/*
	 * If no more fragments, enable backoff protection,
	 * 80211-1999 p. 77
	 */
	if (!more_frag)
		desc.needbackoff = CSF_BO_RAND;

	/* Multicast */
	if (IEEE80211_IS_MULTICAST(wh->i_addr1))
		desc.multicast = CSF_MC_MULTICAST;

	/* Frame type */
	switch (type) {
	case IEEE80211_FC0_TYPE_DATA:
		desc.frametype = CSF_FT_DATAFRAME;
		break;

	case IEEE80211_FC0_TYPE_MGT:
		desc.frametype = CSF_FT_MGMTFRAME;
		break;

	case IEEE80211_FC0_TYPE_CTL:
		/* Only subtype PS_POLL has seq control */
		if (subtype == IEEE80211_FC0_SUBTYPE_PS_POLL)
			desc.frametype = CSF_FT_POLLFRAME;
		else
			desc.frametype = CSF_FT_NOSEQCONTROL;
		break;

	/* All other don't have a sequence control field */
	default:
		desc.frametype = CSF_FT_NOSEQCONTROL;
	}

	/* Wake dst. ignored */

	/*
	 * RTS/CTS
	 * If the frame is non-multicast, non-mgt, set "RTS" if
	 * fragment size > RTS threshold in CCK mode. In OFDM, set
	 * self cts instead.
	 */
	if (!IEEE80211_IS_MULTICAST(wh->i_addr1)
		&& (type != IEEE80211_FC0_TYPE_MGT)
		&& (txlen > ic->ic_rtsthreshold)) {

		if (ZYD_RATE_IS_OFDM(rate))
			desc.selfcts = CSF_SC_SCFRAME;
		else
			desc.rts = CSF_RTS_NEEDRTSFRAME;
	}

	/* Encryption */

	/*
	 * TODO: Hmm ... only set this if hardware performs
	 * encryption. Does it???
	 */	

	/* Self cts */
/*	if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
		desc.selfcts = CSF_SC_SCFRAME;*/

	/* Packet length */
	desc.packetlength = (sizeof(desc) + len + 1) & ~1;

	/* Service (PLCP) */
	desc.service = 0;

	/* Current length (usec) */
	desc.currentlength = htole16(
		zyd_calc_useclen(rate, txlen, &desc.service));

	/* Next frame length (usec) */
	if (more_frag)
		desc.nextframelen = desc.currentlength; // DEBUG!

	DPRINTF(sc, 0, "desc: rate=%d, modulationtype=%d, preamble=%d, "
	    "txlen=%d, needbackoff=%d, multicast=%d, frametype=%d, "
	    "wakedst=%d, rts=%d, encryption=%d, selfcts=%d, "
	    "packetlength=%d, currentlength=%d, service=%d, nextframelen=%d\n",
	    desc.rate, desc.modulationtype, desc.preamble,
	    desc.txlen, desc.needbackoff, desc.multicast, desc.frametype,
	    desc.wakedst, desc.rts, desc.encryption, desc.selfcts,
	    desc.packetlength, desc.currentlength, desc.service,
	    desc.nextframelen);

	usbd_copy_in(&(xfer->buf_data), 0, &desc, sizeof(desc));

	return;
}

static void
zyd_cfg_dump_fw_registers(struct zyd_softc *sc)
{
	static const uint32_t addr[4] = {
		ZYD_FW_FIRMWARE_VER,
		ZYD_FW_USB_SPEED,
		ZYD_FW_FIX_TX_RATE,
		ZYD_FW_LINK_STATUS
	};

	uint32_t i;
	uint16_t values[4];

	for (i = 0; i < 4; ++i)
		zyd_cfg_read16(sc, addr[i], &values[i]);

	DPRINTF(sc, 0, "FW_FIRMWARE_VER %#06hx\n", values[0]);
	DPRINTF(sc, 0, "FW_USB_SPEED %#06hx\n", values[1]);
	DPRINTF(sc, 0, "FW_FIX_TX_RATE %#06hx\n", values[2]);
	DPRINTF(sc, 0, "FW_LINK_STATUS %#06hx\n", values[3]);
}

static uint8_t
zyd_tx_frame(struct usbd_xfer *xfer, struct mbuf *m0, 
	     struct ieee80211_node *ni, uint8_t rate)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);	

	if (m0->m_pkthdr.len > MCLBYTES) {
	    DPRINTF(sc, 0, "data overflow, %u bytes\n",
		    m0->m_pkthdr.len);
	    m0->m_pkthdr.len = MCLBYTES;
	}

	if (sc->sc_drvbpf != NULL) {
	    struct zyd_tx_radiotap_header *tap = &(sc->sc_txtapu.th);

	    tap->wt_flags = 0;
	    tap->wt_rate = rate;
	    tap->wt_chan_freq = htole16(ic->ic_curchan->ic_freq);
	    tap->wt_chan_flags = htole16(ic->ic_curchan->ic_flags);

	    bpf_mtap2(sc->sc_drvbpf, tap, sc->sc_txtap_len, m0);
	}

	zyd_setup_tx_desc(xfer, m0, rate);

	usbd_m_copy_in(&(xfer->buf_data), sizeof(struct zyd_controlsetformat),
		       m0, 0, m0->m_pkthdr.len);

	xfer->length = (sizeof(struct zyd_controlsetformat) + m0->m_pkthdr.len);

	/* xfer length needs to be a multiple of two! */
	if (xfer->length & 1) {
	    usbd_bzero(&(xfer->buf_data), xfer->length, 1);
	    xfer->length ++;
	}

	DPRINTF(sc, 0, "len=%u rate=%u xfer len=%u\n",
		m0->m_pkthdr.len, rate, xfer->length);

	usbd_start_hardware(xfer);

	m_freem(m0);

	if (ni) {
	    ieee80211_free_node(ni);
	}

	return 0; /* success */
}

static void
zyd_cfg_set_chan(struct zyd_softc *sc, 
		 struct zyd_config_copy *cc, u_int16_t refcount)
{
	uint32_t chan;

	chan = cc->ic_curchan.chan_to_ieee;

	DPRINTF(sc, 0, "Will try %d\n", chan);

	if (chan == 0 || chan == IEEE80211_CHAN_ANY)
	{
		DPRINTF(sc, 0, "0 or ANY, exiting\n");
		return;
	}

	zyd_cfg_lock_phy(sc);

	(sc->sc_rf.cfg_set_channel)(sc, &sc->sc_rf, chan);

	/* Power integration */
	zyd_cfg_write32(sc, ZYD_CR31, sc->sc_pwr_int_values[chan - 1]);

	/* Power calibration */
	zyd_cfg_write32(sc, ZYD_CR68, sc->sc_pwr_cal_values[chan - 1]);

	zyd_cfg_unlock_phy(sc);
}

/*
 * Interface: init
 */

/* immediate configuration */

static void
zyd_cfg_pre_init(struct zyd_softc *sc,
		 struct zyd_config_copy *cc, u_int16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = sc->sc_ic.ic_ifp;

	zyd_cfg_pre_stop(sc, cc, 0);

	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	ifp->if_drv_flags |= IFF_DRV_RUNNING;

	sc->sc_flags |= ZYD_FLAG_HL_READY;

	IEEE80211_ADDR_COPY(ic->ic_myaddr, IF_LLADDR(ifp));

	if (ic->ic_opmode != IEEE80211_M_MONITOR) {
	    if (ic->ic_roaming != IEEE80211_ROAMING_MANUAL) {
	        ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
	    }
	} else {
	    ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
	}
	return;
}

/* delayed configuration */

static void
zyd_cfg_init(struct zyd_softc *sc,
		struct zyd_config_copy *cc, u_int16_t refcount)
{
	uint32_t statedata;

	zyd_cfg_stop(sc, cc, 0);

	/* Do initial setup */

	zyd_cfg_set_mac_addr(sc, cc->ic_myaddr);

/*	DPRINTF(sc, 0, "Reset channel\n");
	if (zyd_reset_channel(sc) != 0) {
		return USBD_INVAL;
	}*/

	zyd_cfg_set_encryption_type(sc, sc->sc_encrypt);

	/* TODO: Check what we've already initialized in the hw_init section */

	/* Additional init */
	zyd_cfg_reset_mode(sc);
	zyd_cfg_switch_radio(sc, 1);

	/* Set basic rates */
	zyd_cfg_set_basic_rates(sc, cc->ic_curmode);

	/* Set mandatory rates */
/*	zyd_cfg_set_mandatory_rates(sc, cc->ic_curmode);	*/

	/* set default BSS channel */
	zyd_cfg_set_chan(sc, cc, 0);

	zyd_cfg_enable_hwint(sc);

	/* Load the multicast filter. */
	/*zyd_setmulti(sc); */

	zyd_cfg_read32(sc, ZYD_REG_CTL(0x684), &statedata);

	return;
}

/* immediate configuration */

static void
zyd_cfg_pre_stop(struct zyd_softc *sc,
		struct zyd_config_copy *cc, u_int16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = ic->ic_ifp;

	if (cc) {
	    /* copy the needed configuration */
	    zyd_config_copy(sc, cc, refcount);
	}

	if (ifp) {

	    ieee80211_new_state(ic, IEEE80211_S_INIT, -1);

	    /* clear flags */
	    ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | 
				   IFF_DRV_OACTIVE);
	}

	sc->sc_flags &= ~(ZYD_FLAG_HL_READY|
			  ZYD_FLAG_LL_READY|
			  ZYD_FLAG_TX_BEACON);

	/* stop all the transfers, 
	 * if not already stopped:
	 */
	usbd_transfer_stop(sc->sc_xfer[ZYD_TR_BULK_DT_WR]);
	usbd_transfer_stop(sc->sc_xfer[ZYD_TR_BULK_DT_RD]);
	usbd_transfer_stop(sc->sc_xfer[ZYD_TR_BULK_CS_WR]);
	usbd_transfer_stop(sc->sc_xfer[ZYD_TR_BULK_CS_RD]);
	return;
}

/* delayed configuration */

static void
zyd_cfg_stop(struct zyd_softc *sc,
		struct zyd_config_copy *cc, u_int16_t refcount)
{
	zyd_cfg_reset(sc);
	return;
}

static void
zyd_start_transfers(struct zyd_softc *sc)
{
	if ((sc->sc_flags & ZYD_FLAG_LL_READY) &&
	    (sc->sc_flags & ZYD_FLAG_HL_READY)) {

	    /* start the USB transfers, 
	     * if not already started:
	     */
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_BULK_DT_WR]);
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_BULK_DT_RD]);
	}
	return;
}

static void
zyd_start_cb(struct ifnet *ifp)
{
	struct zyd_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	zyd_start_transfers(sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
zyd_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[ZYD_TR_BULK_DT_WR];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~ZYD_FLAG_BULK_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~ZYD_FLAG_BULK_WRITE_STALL;
	return;
}

static void
zyd_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct zyd_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = sc->sc_ic.ic_ifp;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni = NULL;
	struct ieee80211_rateset *rs;
	/*	struct ieee80211_key *k; */
	struct ether_header *eh;
	struct mbuf *m0 = NULL;
	uint8_t rate;

	DPRINTF(sc, 0, "\n");

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 10, "transfer error: %s\n",
		 usbd_errstr(xfer->error));

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= ZYD_FLAG_BULK_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_BULK_CS_WR]);
	}

	ifp->if_oerrors++;
	return;


 tr_transferred:
	DPRINTF(sc, 10, "transfer complete\n");

	ifp->if_opackets++;

 tr_setup:
	if (sc->sc_flags & ZYD_FLAG_BULK_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[ZYD_TR_BULK_CS_WR]);
	    return;
	}

	if (sc->sc_flags & ZYD_FLAG_WAIT_COMMAND) {
	    /* don't send anything while a command
	     * is pending !
	     */
	    return;
	}

	IF_DEQUEUE(&ic->ic_mgtq, m0);

	if (m0) {

	    ni = (void *)m0->m_pkthdr.rcvif;
	    m0->m_pkthdr.rcvif = NULL;

	    if (ic->ic_rawbpf != NULL) {
	        bpf_mtap(ic->ic_rawbpf, m0);
	    }

	    rate = IEEE80211_IS_CHAN_5GHZ(ic->ic_bss->ni_chan) ? 12 : 2;

	    wh = mtod(m0, struct ieee80211_frame *);

	    if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {
	        uint16_t dur;

		dur = zyd_txtime(ZYD_ACK_SIZE, rate, ic->ic_flags) + ZYD_SIFS;
		*(uint16_t *)wh->i_dur = htole16(dur);
#if 0
		// tell hardware to add timestamp for probe responses
		if ((wh->i_fc[0] &
			(IEEE80211_FC0_TYPE_MASK | IEEE80211_FC0_SUBTYPE_MASK)) ==
			(IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_RESP))
			flags |= RAL_TX_TIMESTAMP;
#endif
	    }

	    if (zyd_tx_frame(xfer, m0, ni, rate)) {
	        goto error;
	    }
	    return;
	}

	if (ic->ic_state != IEEE80211_S_RUN) {
	    return;
	}

	if (sc->sc_flags &   ZYD_FLAG_TX_BEACON) {
	    sc->sc_flags &= ~ZYD_FLAG_TX_BEACON;

	    /*
	     * Transmit beacon frame
	     */
	    m0 = ieee80211_beacon_alloc(ic, ic->ic_bss, &(sc->sc_bo));

	    if (m0 == NULL) {
	        DPRINTF(sc, -1, "could not allocate beacon frame!\n");
		goto error;
	    }

	    rate = IEEE80211_IS_CHAN_5GHZ(ic->ic_bss->ni_chan) ? 12 : 4;

	    if (zyd_tx_frame(xfer, m0, NULL, rate)) {
	        goto error;
	    }
	    return;
	}

	IFQ_DEQUEUE(&ifp->if_snd, m0);

	if (m0) {

	    if (m0->m_len < sizeof(struct ether_header)) {
	        m0 = m_pullup(m0, sizeof(struct ether_header));

		if (m0 == NULL) {
		    goto error;
		}
	    }

	    eh = mtod(m0, struct ether_header *);
	    ni = ieee80211_find_txnode(ic, eh->ether_dhost);

	    if (ni == NULL) {
	        goto error;
	    }

	    BPF_MTAP(ifp, m0);

	    m0 = ieee80211_encap(ic, m0, ni);

	    if (m0 == NULL) {
	        goto error;
	    }

	    if (ic->ic_rawbpf != NULL) {
	        bpf_mtap(ic->ic_rawbpf, m0);
	    }

	    /* XXX this should be reworked! */
	    if (ic->ic_fixed_rate != -1) {
		if (ic->ic_curmode != IEEE80211_MODE_AUTO)
			rs = &ic->ic_sup_rates[ic->ic_curmode];
		else
			rs = &ic->ic_sup_rates[IEEE80211_MODE_11G];

		rate = rs->rs_rates[ic->ic_fixed_rate];
	    } else {
		rs = &ni->ni_rates;
		rate = rs->rs_rates[ni->ni_txrate];
	    }

	    rate &= IEEE80211_RATE_VAL;
#if 0
	    if (ic->ic_flags & IEEE80211_F_WEPON) {
		m0 = ieee80211_wep_crypt(ifp, m0, 1);
		if (m0 == NULL)
			return ENOBUFS;
	    }
#endif

#if 0
	    wh = mtod(m0, struct ieee80211_frame *);

	    if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		dur = zyd_txtime(ZYD_ACK_SIZE, zyd_ack_rate(ic, rate),
			ic->ic_flags) + ZYD_SIFS;
		*(uint16_t *)wh->i_dur = htole16(dur);
	    }
#endif

	    if (zyd_tx_frame(xfer, m0, ni, rate)) {
	        goto error;
	    }
	    return;
	}
	return;

 error:
	if (m0) {
	    m_freem(m0);
	    m0 = NULL;
	}

	if (ni) {
	    ieee80211_free_node(ni);
	    ni = NULL;
	}

	ifp->if_oerrors++;

	goto tr_setup;
}

static void
zyd_init_cb(void *arg)
{
	struct zyd_softc *sc = arg;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &zyd_cfg_pre_init,
	   &zyd_cfg_init, 0, 0);
	mtx_unlock(&(sc->sc_mtx));

	return;
}

/*
 * This function allows for fast channel switching in monitor mode (used by
 * net-mgmt/kismet). In IBSS mode, we must explicitly reset the interface to
 * generate a new beacon frame.
 */
static int
zyd_reset_cb(struct ifnet *ifp)
{
	struct zyd_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &(sc->sc_ic);
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	if (ic->ic_opmode != IEEE80211_M_MONITOR) {
		error = ENETRESET;
		goto done;
	}

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &zyd_config_copy,
	   &zyd_cfg_set_chan, 0, 0);

 done:
	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

static int
zyd_ioctl_cb(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct zyd_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &(sc->sc_ic);
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	switch (command) {
	case SIOCSIFFLAGS:

	    if (ifp->if_flags & IFF_UP) {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &zyd_config_copy,
		       &zyd_cfg_update_promisc, 0, 0);
		} else {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &zyd_cfg_pre_init,
		       &zyd_cfg_init, 0, 0); 
		}
	    } else {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &zyd_cfg_pre_stop, 
		       &zyd_cfg_stop, 0, 0);
		}
	    }
	    break;

	default:
	    error = ieee80211_ioctl(ic, command, data);
	    break;
	}

	if (error == ENETRESET) {
	    if ((ifp->if_flags & IFF_UP) &&
		(ifp->if_drv_flags & IFF_DRV_RUNNING) &&
		(ic->ic_roaming != IEEE80211_ROAMING_MANUAL)) {
		usbd_config_td_queue_command
		  (&(sc->sc_config_td), &zyd_cfg_pre_init,
		   &zyd_cfg_init, 0, 0);
	    }
	    error = 0;
	}

	mtx_unlock(&(sc->sc_mtx));

	return error;
}

static void
zyd_watchdog(void *arg)
{
	struct zyd_softc *sc = arg;
	struct ieee80211com *ic = &(sc->sc_ic);

	mtx_assert(&(sc->sc_mtx), MA_OWNED);

	DPRINTF(sc, 0, "\n");

	ieee80211_watchdog(ic);

	__callout_reset(&(sc->sc_watchdog), 
			hz, &zyd_watchdog, sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

/*
 * This function is called periodically (every 200ms) during scanning to
 * switch from one channel to another.
 */
static void
zyd_next_scan(void *arg)
{
	struct zyd_softc *sc = arg;
	struct ieee80211com *ic = &sc->sc_ic;

	mtx_assert(&(sc->sc_mtx), MA_OWNED);

	DPRINTF(sc, 0, "executing next_scan\n");

	if (ic->ic_state == IEEE80211_S_SCAN)
		ieee80211_next_scan(ic);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
zyd_config_copy(struct zyd_softc *sc, 
		struct zyd_config_copy *cc, u_int16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ieee80211_channel *c = ic->ic_curchan;
	struct ifnet *ifp = ic->ic_ifp;
	u_int8_t n;

	bzero(cc, sizeof(*cc));

	if (c) {
	    cc->ic_curchan.chan_to_ieee = ieee80211_chan2ieee(ic, c);
	    if (c != IEEE80211_CHAN_ANYC) {
	        cc->ic_curchan.chan_is_2ghz = IEEE80211_IS_CHAN_2GHZ(c) ? 1 : 0;
	    }
	}

	if (ic->ic_bss) {
	    if ((ic->ic_bss->ni_chan) &&
		(ic->ic_bss->ni_chan != IEEE80211_CHAN_ANYC)) {
	        cc->ic_bss.ni_chan.chan_is_5ghz = 
	          IEEE80211_IS_CHAN_5GHZ(ic->ic_bss->ni_chan) ? 1 : 0;
	    }

	    cc->ic_bss.ni_intval = ic->ic_bss->ni_intval;
	    bcopy(ic->ic_bss->ni_bssid, cc->ic_bss.ni_bssid, 
	          sizeof(cc->ic_bss.ni_bssid));
	}

	for (n = 0; n < IEEE80211_WEP_NKID; n++) {
	    cc->ic_crypto.cs_nw_keys[n].wk_keyix =
	      ic->ic_crypto.cs_nw_keys[n].wk_keyix;

	    bcopy(ic->ic_crypto.cs_nw_keys[n].wk_key,
		  cc->ic_crypto.cs_nw_keys[n].wk_key, 
		  IEEE80211_KEYBUF_SIZE);
	}

	cc->ic_opmode = ic->ic_opmode;
	cc->ic_state = ic->ic_state;
	cc->ic_flags = ic->ic_flags;

	if (ifp) {
	    cc->if_flags = ifp->if_flags;
	}

	cc->ic_txpowlimit = ic->ic_txpowlimit;
	cc->ic_curmode = ic->ic_curmode;

	bcopy(ic->ic_myaddr, cc->ic_myaddr, 
	      sizeof(cc->ic_myaddr));

	sc->sc_flags |= ZYD_FLAG_WAIT_COMMAND;
	return;
}

static void
zyd_end_of_commands(struct zyd_softc *sc)
{
	sc->sc_flags &= ~ZYD_FLAG_WAIT_COMMAND;

	zyd_start_transfers(sc);
	return;
}
