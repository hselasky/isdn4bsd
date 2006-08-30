/*-
 * Copyright (c) 2005, 2006
 *	Damien Bergamini <damien.bergamini@free.fr>
 *
 * Copyright (c) 2006
 *	Hans Petter Selasky <hselasky@freebsd.org>
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
 *
 *
 * NOTE: all function names beginning like "ural_cfg_" can only
 * be called from within the config thread function !
 */

/*-
 * Ralink Technology RT2500USB chipset driver
 * http://www.ralinktech.com/
 */

#include <sys/cdefs.h>
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

#define usbd_config_td_cc ural_config_copy
#define usbd_config_td_softc ural_softc

#include <dev/usb2/usb_port.h>
#include <dev/usb2/usb.h>
#include <dev/usb2/usb_subr.h>

#include "usbdevs.h"

#include <dev/usb/if_uralreg.h>
#include <dev/usb/if_uralvar.h>

__FBSDID("$FreeBSD: src/sys/dev/usb/if_ural.c,v 1.38 2006/05/16 14:36:32 phk Exp $");

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)	\
  do { if (ural_debug > (n)) {	     \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ural_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, ural, CTLFLAG_RW, 0, "USB ural");
SYSCTL_INT(_hw_usb_ural, OID_AUTO, debug, CTLFLAG_RW, &ural_debug, 0,
	   "ural debug level");
#else
#define DPRINTF(...)
#endif

/* prototypes */
static device_probe_t ural_probe;
static device_attach_t ural_attach;
static device_detach_t ural_detach;

static void
ural_cfg_do_request(struct ural_softc *sc, usb_device_request_t *req, 
		    void *data);
static void
ural_cfg_set_testmode(struct ural_softc *sc);

static void
ural_cfg_eeprom_read(struct ural_softc *sc, u_int16_t addr, 
		     void *buf, int len);
static u_int16_t
ural_cfg_read(struct ural_softc *sc, u_int16_t reg);

static void
ural_cfg_read_multi(struct ural_softc *sc, u_int16_t reg, 
		    void *buf, int len);
static void
ural_cfg_write(struct ural_softc *sc, u_int16_t reg, u_int16_t val);

static void
ural_cfg_write_multi(struct ural_softc *sc, u_int16_t reg, 
		     void *buf, int len);
static void
ural_cfg_bbp_write(struct ural_softc *sc, u_int8_t reg, u_int8_t val);

static u_int8_t
ural_cfg_bbp_read(struct ural_softc *sc, u_int8_t reg);

static void
ural_cfg_rf_write(struct ural_softc *sc, u_int8_t reg, u_int32_t val);

static void
ural_cfg_first_time_setup(struct ural_softc *sc,
			  struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_end_of_commands(struct ural_softc *sc);

static void
ural_config_copy(struct ural_softc *sc, 
		 struct ural_config_copy *cc, u_int16_t refcount);
static const char *
ural_get_rf(int rev);

static int
ural_rxrate(struct ural_rx_desc *desc);

static void
ural_bulk_read_callback(struct usbd_xfer *xfer);

static void
ural_bulk_read_clear_stall_callback(struct usbd_xfer *xfer);

static u_int16_t
ural_ack_rate(struct ieee80211com *ic, u_int16_t rate);

static u_int16_t
ural_txtime(struct ural_softc *sc, u_int16_t len, u_int16_t rate, 
	    u_int32_t flags);

static u_int8_t
ural_plcp_signal(u_int16_t rate);

static void
ural_setup_tx_desc(struct ural_softc *sc, u_int32_t flags, u_int16_t len, 
		   u_int16_t rate);
static void
ural_bulk_write_callback(struct usbd_xfer *xfer);

static void
ural_bulk_write_clear_stall_callback(struct usbd_xfer *xfer);

static void
ural_watchdog(void *arg);

static void
ural_init_cb(void *arg);

static int
ural_ioctl_cb(struct ifnet *ifp, u_long cmd, caddr_t data);

static void
ural_start_cb(struct ifnet *ifp);

static int
ural_media_change_cb(struct ifnet *ifp);

static int
ural_reset_cb(struct ifnet *ifp);

static int
ural_newstate_cb(struct ieee80211com *ic, enum ieee80211_state nstate, int arg);

static void
ural_tx_bcn_complete(struct ural_softc *sc);

static void
ural_cfg_tx_bcn(struct ural_softc *sc);

static void
ural_cfg_set_chan(struct ural_softc *sc,
		  struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_set_run(struct ural_softc *sc, 
		 struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_disable_rf_tune(struct ural_softc *sc);

static void
ural_cfg_enable_tsf_sync(struct ural_softc *sc,
			 struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_disable_tsf_sync(struct ural_softc *sc,
			  struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_update_slot(struct ural_softc *sc,
		     struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_set_txpreamble(struct ural_softc *sc,
			struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_set_basicrates(struct ural_softc *sc,
			struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_set_bssid(struct ural_softc *sc, const u_int8_t *bssid);

static void
ural_cfg_set_macaddr(struct ural_softc *sc, const u_int8_t *addr);

static void
ural_cfg_update_promisc(struct ural_softc *sc,
			struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_set_txantenna(struct ural_softc *sc, u_int8_t antenna);

static void
ural_cfg_set_rxantenna(struct ural_softc *sc, u_int8_t antenna);

static void
ural_cfg_read_eeprom(struct ural_softc *sc);

static u_int8_t
ural_cfg_bbp_init(struct ural_softc *sc);

static void
ural_cfg_init(struct ural_softc *sc,
	      struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_stop(struct ural_softc *sc,
	      struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_cfg_amrr_start(struct ural_softc *sc);

static void
ural_cfg_amrr_timeout(struct ural_softc *sc,
		      struct ural_config_copy *cc, u_int16_t refcount);
static void
ural_ratectl(struct ural_amrr *amrr, struct ieee80211_node *ni);

/* various supported device vendors/products */
static const struct usb_devno ural_devs[] = {
	{ USB_VENDOR_ASUS,		USB_PRODUCT_ASUS_WL167G },
	{ USB_VENDOR_ASUS,		USB_PRODUCT_RALINK_RT2570 },
	{ USB_VENDOR_BELKIN,		USB_PRODUCT_BELKIN_F5D7050 },
	{ USB_VENDOR_CONCEPTRONIC,	USB_PRODUCT_CONCEPTRONIC_C54U },
	{ USB_VENDOR_DLINK,		USB_PRODUCT_DLINK_DWLG122 },
	{ USB_VENDOR_GIGABYTE,		USB_PRODUCT_GIGABYTE_GNWBKG },
	{ USB_VENDOR_GUILLEMOT,		USB_PRODUCT_GUILLEMOT_HWGUSB254 },
	{ USB_VENDOR_LINKSYS4,		USB_PRODUCT_LINKSYS4_WUSB54G },
	{ USB_VENDOR_LINKSYS4,		USB_PRODUCT_LINKSYS4_WUSB54GP },
	{ USB_VENDOR_LINKSYS4,		USB_PRODUCT_LINKSYS4_HU200TS },
	{ USB_VENDOR_MELCO,		USB_PRODUCT_MELCO_KG54 },
	{ USB_VENDOR_MELCO,		USB_PRODUCT_MELCO_KG54AI },
	{ USB_VENDOR_MELCO,		USB_PRODUCT_MELCO_KG54YB },
	{ USB_VENDOR_MELCO,		USB_PRODUCT_MELCO_NINWIFI },
	{ USB_VENDOR_MSI,		USB_PRODUCT_MSI_RT2570 },
	{ USB_VENDOR_MSI,		USB_PRODUCT_MSI_RT2570_2 },
	{ USB_VENDOR_MSI,		USB_PRODUCT_MSI_RT2570_3 },
	{ USB_VENDOR_RALINK,		USB_PRODUCT_RALINK_RT2570 },
	{ USB_VENDOR_RALINK,		USB_PRODUCT_RALINK_RT2570_2 },
	{ USB_VENDOR_VTECH,		USB_PRODUCT_VTECH_RT2570 },
	{ USB_VENDOR_ZINWELL,		USB_PRODUCT_ZINWELL_RT2570 }
};

/*
 * Supported rates for 802.11a/b/g modes (in 500Kbps unit).
 */
static const struct ieee80211_rateset ural_rateset_11a =
	{ 8, { 12, 18, 24, 36, 48, 72, 96, 108 } };

static const struct ieee80211_rateset ural_rateset_11b =
	{ 4, { 2, 4, 11, 22 } };

static const struct ieee80211_rateset ural_rateset_11g =
	{ 12, { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 } };

/*
 * Default values for MAC registers; values taken from
 * the reference driver:
 */
static const struct {
	u_int16_t	reg;
	u_int16_t	val;
} ural_def_mac[] = {
	{ RAL_TXRX_CSR5,  0x8c8d },
	{ RAL_TXRX_CSR6,  0x8b8a },
	{ RAL_TXRX_CSR7,  0x8687 },
	{ RAL_TXRX_CSR8,  0x0085 },
	{ RAL_MAC_CSR13,  0x1111 },
	{ RAL_MAC_CSR14,  0x1e11 },
	{ RAL_TXRX_CSR21, 0xe78f },
	{ RAL_MAC_CSR9,   0xff1d },
	{ RAL_MAC_CSR11,  0x0002 },
	{ RAL_MAC_CSR22,  0x0053 },
	{ RAL_MAC_CSR15,  0x0000 },
	{ RAL_MAC_CSR8,   0x0780 },
	{ RAL_TXRX_CSR19, 0x0000 },
	{ RAL_TXRX_CSR18, 0x005a },
	{ RAL_PHY_CSR2,   0x0000 },
	{ RAL_TXRX_CSR0,  0x1ec0 },
	{ RAL_PHY_CSR4,   0x000f }
};

/*
 * Default values for BBP registers; values taken from the reference driver.
 */
static const struct {
	u_int8_t	reg;
	u_int8_t	val;
} ural_def_bbp[] = {
	{  3, 0x02 },
	{  4, 0x19 },
	{ 14, 0x1c },
	{ 15, 0x30 },
	{ 16, 0xac },
	{ 17, 0x48 },
	{ 18, 0x18 },
	{ 19, 0xff },
	{ 20, 0x1e },
	{ 21, 0x08 },
	{ 22, 0x08 },
	{ 23, 0x08 },
	{ 24, 0x80 },
	{ 25, 0x50 },
	{ 26, 0x08 },
	{ 27, 0x23 },
	{ 30, 0x10 },
	{ 31, 0x2b },
	{ 32, 0xb9 },
	{ 34, 0x12 },
	{ 35, 0x50 },
	{ 39, 0xc4 },
	{ 40, 0x02 },
	{ 41, 0x60 },
	{ 53, 0x10 },
	{ 54, 0x18 },
	{ 56, 0x08 },
	{ 57, 0x10 },
	{ 58, 0x08 },
	{ 61, 0x60 },
	{ 62, 0x10 },
	{ 75, 0xff }
};

/*
 * Default values for RF register R2 indexed by channel numbers.
 */
static const u_int32_t ural_rf2522_r2[] = {
	0x307f6, 0x307fb, 0x30800, 0x30805, 0x3080a, 0x3080f, 0x30814,
	0x30819, 0x3081e, 0x30823, 0x30828, 0x3082d, 0x30832, 0x3083e
};

static const u_int32_t ural_rf2523_r2[] = {
	0x00327, 0x00328, 0x00329, 0x0032a, 0x0032b, 0x0032c, 0x0032d,
	0x0032e, 0x0032f, 0x00340, 0x00341, 0x00342, 0x00343, 0x00346
};

static const u_int32_t ural_rf2524_r2[] = {
	0x00327, 0x00328, 0x00329, 0x0032a, 0x0032b, 0x0032c, 0x0032d,
	0x0032e, 0x0032f, 0x00340, 0x00341, 0x00342, 0x00343, 0x00346
};

static const u_int32_t ural_rf2525_r2[] = {
	0x20327, 0x20328, 0x20329, 0x2032a, 0x2032b, 0x2032c, 0x2032d,
	0x2032e, 0x2032f, 0x20340, 0x20341, 0x20342, 0x20343, 0x20346
};

static const u_int32_t ural_rf2525_hi_r2[] = {
	0x2032f, 0x20340, 0x20341, 0x20342, 0x20343, 0x20344, 0x20345,
	0x20346, 0x20347, 0x20348, 0x20349, 0x2034a, 0x2034b, 0x2034e
};

static const u_int32_t ural_rf2525e_r2[] = {
	0x2044d, 0x2044e, 0x2044f, 0x20460, 0x20461, 0x20462, 0x20463,
	0x20464, 0x20465, 0x20466, 0x20467, 0x20468, 0x20469, 0x2046b
};

static const u_int32_t ural_rf2526_hi_r2[] = {
	0x0022a, 0x0022b, 0x0022b, 0x0022c, 0x0022c, 0x0022d, 0x0022d,
	0x0022e, 0x0022e, 0x0022f, 0x0022d, 0x00240, 0x00240, 0x00241
};

static const u_int32_t ural_rf2526_r2[] = {
	0x00226, 0x00227, 0x00227, 0x00228, 0x00228, 0x00229, 0x00229,
	0x0022a, 0x0022a, 0x0022b, 0x0022b, 0x0022c, 0x0022c, 0x0022d
};

/*
 * For dual-band RF, RF registers R1 and R4 also depend on channel number;
 * values taken from the reference driver.
 */
static const struct {
	u_int8_t	chan;
	u_int32_t	r1;
	u_int32_t	r2;
	u_int32_t	r4;
} ural_rf5222[] = {
	{   1, 0x08808, 0x0044d, 0x00282 },
	{   2, 0x08808, 0x0044e, 0x00282 },
	{   3, 0x08808, 0x0044f, 0x00282 },
	{   4, 0x08808, 0x00460, 0x00282 },
	{   5, 0x08808, 0x00461, 0x00282 },
	{   6, 0x08808, 0x00462, 0x00282 },
	{   7, 0x08808, 0x00463, 0x00282 },
	{   8, 0x08808, 0x00464, 0x00282 },
	{   9, 0x08808, 0x00465, 0x00282 },
	{  10, 0x08808, 0x00466, 0x00282 },
	{  11, 0x08808, 0x00467, 0x00282 },
	{  12, 0x08808, 0x00468, 0x00282 },
	{  13, 0x08808, 0x00469, 0x00282 },
	{  14, 0x08808, 0x0046b, 0x00286 },

	{  36, 0x08804, 0x06225, 0x00287 },
	{  40, 0x08804, 0x06226, 0x00287 },
	{  44, 0x08804, 0x06227, 0x00287 },
	{  48, 0x08804, 0x06228, 0x00287 },
	{  52, 0x08804, 0x06229, 0x00287 },
	{  56, 0x08804, 0x0622a, 0x00287 },
	{  60, 0x08804, 0x0622b, 0x00287 },
	{  64, 0x08804, 0x0622c, 0x00287 },

	{ 100, 0x08804, 0x02200, 0x00283 },
	{ 104, 0x08804, 0x02201, 0x00283 },
	{ 108, 0x08804, 0x02202, 0x00283 },
	{ 112, 0x08804, 0x02203, 0x00283 },
	{ 116, 0x08804, 0x02204, 0x00283 },
	{ 120, 0x08804, 0x02205, 0x00283 },
	{ 124, 0x08804, 0x02206, 0x00283 },
	{ 128, 0x08804, 0x02207, 0x00283 },
	{ 132, 0x08804, 0x02208, 0x00283 },
	{ 136, 0x08804, 0x02209, 0x00283 },
	{ 140, 0x08804, 0x0220a, 0x00283 },

	{ 149, 0x08808, 0x02429, 0x00281 },
	{ 153, 0x08808, 0x0242b, 0x00281 },
	{ 157, 0x08808, 0x0242d, 0x00281 },
	{ 161, 0x08808, 0x0242f, 0x00281 }
};

static const struct usbd_config ural_config[URAL_N_TRANSFER] = {
    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = (MCLBYTES + RAL_TX_DESC_SIZE + 4),
      .flags     = (USBD_USE_DMA|USBD_FORCE_SHORT_XFER),
      .callback  = &ural_bulk_write_callback,
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = (MCLBYTES + RAL_RX_DESC_SIZE),
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &ural_bulk_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ural_bulk_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ural_bulk_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static devclass_t ural_devclass;

static device_method_t ural_methods[] = {
    DEVMETHOD(device_probe, ural_probe),
    DEVMETHOD(device_attach, ural_attach),
    DEVMETHOD(device_detach, ural_detach),
    { 0, 0 }
};

static driver_t ural_driver = {
    .name    = "ural",
    .methods = ural_methods,
    .size    = sizeof(struct ural_softc),
};

DRIVER_MODULE(ural, uhub, ural_driver, ural_devclass, usbd_driver_load, 0);
MODULE_DEPEND(ural, usb, 1, 1, 1);
MODULE_DEPEND(ural, wlan, 1, 1, 1);

static int
ural_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->iface != NULL) {
	    return UMATCH_NONE;
	}

	return ((usb_lookup(ural_devs, uaa->vendor, uaa->product) != NULL) ?
		UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

static int
ural_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ural_softc *sc = device_get_softc(dev);
	int error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	usbd_set_desc(dev, uaa->device);

	mtx_init(&sc->sc_mtx, "ural lock", MTX_NETWORK_LOCK,
		 MTX_DEF | MTX_RECURSE);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s", 
		 device_get_nameunit(dev));

	sc->sc_udev = uaa->device; /* XXX */
	sc->sc_unit = device_get_unit(dev);

	__callout_init_mtx(&(sc->sc_watchdog),
			   &(sc->sc_mtx), CALLOUT_RETURNUNLOCKED);

	error = usbd_set_config_no(uaa->device, RAL_CONFIG_NO, 0);

	if (error) {
	    device_printf(dev, "could not set configuration "
			  "number, err=%s!\n", usbd_errstr(error));
	    goto detach;
	}

	error = usbd_transfer_setup(uaa->device, RAL_IFACE_INDEX, 
				    sc->sc_xfer, ural_config, URAL_N_TRANSFER, 
				    sc, &(sc->sc_mtx), &(sc->sc_mem_wait));
	if (error) {
	    device_printf(dev, "could not allocate USB transfers, "
			  "err=%s\n", usbd_errstr(error)) ;
	    goto detach;
	}

	error = usbd_config_td_setup(&(sc->sc_config_td), sc, &(sc->sc_mtx),
				     &ural_config_copy, &ural_end_of_commands,
				     sizeof(struct ural_config_copy), 16);
	if (error) {
	    device_printf(dev, "could not setup config "
			  "thread!\n");
	    goto detach;
	}

	mtx_lock(&(sc->sc_mtx));

	/* start setup */

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &ural_cfg_first_time_setup, 0);

	/* start watchdog (will exit mutex) */

	ural_watchdog(sc);

	return 0; /* success */

 detach:
	ural_detach(dev);
	return ENXIO; /* failure */
}

static int
ural_detach(device_t dev)
{
	struct ural_softc *sc = device_get_softc(dev);
	struct ieee80211com *ic;
	struct ifnet *ifp;

	mtx_lock(&(sc->sc_mtx));

	usbd_config_td_stop(&(sc->sc_config_td));

	__callout_stop(&sc->sc_watchdog);

	ural_cfg_stop(sc, NULL, 0);

	ic = &(sc->sc_ic);
	ifp = ic->ic_ifp;

	mtx_unlock(&(sc->sc_mtx));

	if (ifp) {
	    bpfdetach(ifp);
	    ieee80211_ifdetach(ic);
	    if_free(ifp);
	}

	usbd_transfer_unsetup(sc->sc_xfer, URAL_N_TRANSFER);

	usbd_transfer_drain(&(sc->sc_mem_wait), &(sc->sc_mtx));

	usbd_config_td_unsetup(&(sc->sc_config_td));

	mtx_destroy(&sc->sc_mtx);

	return 0;
}

/*========================================================================*
 * REGISTER READ / WRITE WRAPPER ROUTINES
 *========================================================================*/

static void
ural_cfg_do_request(struct ural_softc *sc, usb_device_request_t *req, 
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

	    printf("%s: device request failed, err=%s "
		   "(ignored)\n", sc->sc_name, usbd_errstr(err));

	error:
	    length = UGETW(req->wLength);

	    if ((req->bmRequestType & UT_READ) && length) {
	        bzero(data, length);
	    }
	}
	return;
}

static void
ural_cfg_set_testmode(struct ural_softc *sc)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RAL_VENDOR_REQUEST;
	USETW(req.wValue, 4);
	USETW(req.wIndex, 1);
	USETW(req.wLength, 0);

	ural_cfg_do_request(sc, &req, NULL);
	return;
}

static void
ural_cfg_eeprom_read(struct ural_softc *sc, u_int16_t addr, 
		     void *buf, int len)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = RAL_READ_EEPROM;
	USETW(req.wValue, 0);
	USETW(req.wIndex, addr);
	USETW(req.wLength, len);

	ural_cfg_do_request(sc, &req, buf);
	return;
}

static u_int16_t
ural_cfg_read(struct ural_softc *sc, u_int16_t reg)
{
	usb_device_request_t req;
	u_int16_t val;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = RAL_READ_MAC;
	USETW(req.wValue, 0);
	USETW(req.wIndex, reg);
	USETW(req.wLength, sizeof(val));

	ural_cfg_do_request(sc, &req, &val);

	return le16toh(val);
}

static void
ural_cfg_read_multi(struct ural_softc *sc, u_int16_t reg, 
		    void *buf, int len)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = RAL_READ_MULTI_MAC;
	USETW(req.wValue, 0);
	USETW(req.wIndex, reg);
	USETW(req.wLength, len);

	ural_cfg_do_request(sc, &req, buf);
	return;
}

static void
ural_cfg_write(struct ural_softc *sc, u_int16_t reg, u_int16_t val)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RAL_WRITE_MAC;
	USETW(req.wValue, val);
	USETW(req.wIndex, reg);
	USETW(req.wLength, 0);

	ural_cfg_do_request(sc, &req, NULL);
	return;
}

static void
ural_cfg_write_multi(struct ural_softc *sc, u_int16_t reg, 
		     void *buf, int len)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RAL_WRITE_MULTI_MAC;
	USETW(req.wValue, 0);
	USETW(req.wIndex, reg);
	USETW(req.wLength, len);

	ural_cfg_do_request(sc, &req, buf);
	return;
}

static void
ural_cfg_bbp_write(struct ural_softc *sc, u_int8_t reg, u_int8_t val)
{
	u_int16_t tmp;
	u_int8_t to;

	for (to = 0; ; to++) {
	    if (to < 5) {
	        tmp = ural_cfg_read(sc, RAL_PHY_CSR8);

		if (!(tmp & RAL_BBP_BUSY)) {
		    break;
		}
	    } else {
		printf("%s: could not write to BBP\n", 
		       sc->sc_name);
		return;
	    }
	}

	tmp = (reg << 8) | val;
	ural_cfg_write(sc, RAL_PHY_CSR7, tmp);
	return;
}

static u_int8_t
ural_cfg_bbp_read(struct ural_softc *sc, u_int8_t reg)
{
	u_int16_t val;
	u_int8_t to;

	val = RAL_BBP_WRITE | (reg << 8);
	ural_cfg_write(sc, RAL_PHY_CSR7, val);

	for (to = 0; ; to++) {
	    if (to < 5) {
	        val = ural_cfg_read(sc, RAL_PHY_CSR8);

		if (!(val & RAL_BBP_BUSY)) {
		    break;
		}
	    } else {
		printf("%s: could not read BBP\n", 
		       sc->sc_name);
		return 0;
	    }
	}
	return (ural_cfg_read(sc, RAL_PHY_CSR7) & 0xff);
}

static void
ural_cfg_rf_write(struct ural_softc *sc, u_int8_t reg, u_int32_t val)
{
	u_int32_t tmp;
	u_int8_t to;

	reg &= 3;

	for (to = 0; ; to++) {
	    if (to < 5) {
	        tmp = ural_cfg_read(sc, RAL_PHY_CSR10);

		if (!(tmp & RAL_RF_LOBUSY)) {
		    break;
		}
	    } else {
		printf("%s: could not write to RF\n", 
		       sc->sc_name);
		return;
	    }
	}

	tmp = RAL_RF_BUSY | RAL_RF_20BIT | ((val & 0xfffff) << 2) | reg;
	ural_cfg_write(sc, RAL_PHY_CSR9,  tmp & 0xffff);
	ural_cfg_write(sc, RAL_PHY_CSR10, tmp >> 16);

	/* remember last written value in sc */
	sc->sc_rf_regs[reg] = val;

	DPRINTF(sc, 15, "RF R[%u] <- 0x%05x\n", reg, val & 0xfffff);
	return;
}

static void
ural_cfg_first_time_setup(struct ural_softc *sc,
			  struct ural_config_copy *cc, u_int16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp;
	register u_int16_t i;

	if (cc == NULL) {
	    return;
	}

	/* retrieve RT2570 rev. no */
	sc->sc_asic_rev = ural_cfg_read(sc, RAL_MAC_CSR0);

	/* retrieve MAC address and various other things from EEPROM */
	ural_cfg_read_eeprom(sc);

	printf("%s: MAC/BBP RT2570 (rev 0x%02x), RF %s\n",
	       sc->sc_name, sc->sc_asic_rev, ural_get_rf(sc->sc_rf_rev));

	mtx_unlock(&(sc->sc_mtx));

	ifp = if_alloc(IFT_ETHER);

	mtx_lock(&(sc->sc_mtx));

	if (ifp == NULL) {
	    printf("%s: could not if_alloc()!\n", 
		   sc->sc_name);
	    goto done;
	}

	sc->sc_ifp = ifp;

	ifp->if_softc = sc;
	if_initname(ifp, "ural", sc->sc_unit);
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_init = &ural_init_cb;
	ifp->if_ioctl = &ural_ioctl_cb;
	ifp->if_start = &ural_start_cb;
	ifp->if_watchdog = NULL;
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
	ifp->if_snd.ifq_drv_maxlen = IFQ_MAXLEN;
	IFQ_SET_READY(&ifp->if_snd);

	ic->ic_ifp = ifp;
	ic->ic_phytype = IEEE80211_T_OFDM; /* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA; /* default to BSS mode */
	ic->ic_state = IEEE80211_S_INIT;

	/* set device capabilities */
	ic->ic_caps =
	    IEEE80211_C_IBSS |		/* IBSS mode supported */
	    IEEE80211_C_MONITOR |	/* monitor mode supported */
	    IEEE80211_C_HOSTAP |	/* HostAp mode supported */
	    IEEE80211_C_TXPMGT |	/* tx power management */
	    IEEE80211_C_SHPREAMBLE |	/* short preamble supported */
	    IEEE80211_C_SHSLOT |	/* short slot time supported */
	    IEEE80211_C_WPA;		/* 802.11i */

	if (sc->sc_rf_rev == RAL_RF_5222) {

		/* set supported .11a rates */
		ic->ic_sup_rates[IEEE80211_MODE_11A] = ural_rateset_11a;

		/* set supported .11a channels */
		for (i = 36; i <= 64; i += 4) {
			ic->ic_channels[i].ic_freq =
			    ieee80211_ieee2mhz(i, IEEE80211_CHAN_5GHZ);
			ic->ic_channels[i].ic_flags = IEEE80211_CHAN_A;
		}
		for (i = 100; i <= 140; i += 4) {
			ic->ic_channels[i].ic_freq =
			    ieee80211_ieee2mhz(i, IEEE80211_CHAN_5GHZ);
			ic->ic_channels[i].ic_flags = IEEE80211_CHAN_A;
		}
		for (i = 149; i <= 161; i += 4) {
			ic->ic_channels[i].ic_freq =
			    ieee80211_ieee2mhz(i, IEEE80211_CHAN_5GHZ);
			ic->ic_channels[i].ic_flags = IEEE80211_CHAN_A;
		}
	}

	/* set supported .11b and .11g rates */
	ic->ic_sup_rates[IEEE80211_MODE_11B] = ural_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = ural_rateset_11g;

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

	ic->ic_reset = &ural_reset_cb;

	/* enable SW bmiss handling in sta mode */
	ic->ic_flags_ext |= IEEE80211_FEXT_SWBMISS;

	/* override state transition machine */
	sc->sc_newstate = ic->ic_newstate;
	ic->ic_newstate = ural_newstate_cb;

	mtx_unlock(&(sc->sc_mtx));

	ieee80211_media_init(ic, ural_media_change_cb, ieee80211_media_status);

	bpfattach2(ifp, DLT_IEEE802_11_RADIO,
		   sizeof(struct ieee80211_frame) + 64, &sc->sc_drvbpf);

	mtx_lock(&(sc->sc_mtx));

	sc->sc_rxtap_len = sizeof(sc->sc_rxtap.h);
	sc->sc_rxtap.h.wr_ihdr.it_len = htole16(sc->sc_rxtap_len);
	sc->sc_rxtap.h.wr_ihdr.it_present = htole32(RAL_RX_RADIOTAP_PRESENT);

	sc->sc_txtap_len = sizeof(sc->sc_txtap.h);
	sc->sc_txtap.h.wt_ihdr.it_len = htole16(sc->sc_txtap_len);
	sc->sc_txtap.h.wt_ihdr.it_present = htole32(RAL_TX_RADIOTAP_PRESENT);

	if (bootverbose) {
	    ieee80211_announce(ic);
	}
 done:
	return;
}

static void
ural_end_of_commands(struct ural_softc *sc)
{
	sc->sc_flags &= ~URAL_FLAG_WAIT_COMMAND;

	if ((sc->sc_flags & URAL_FLAG_LL_READY) &&
	    (sc->sc_flags & URAL_FLAG_HL_READY)) {
	    /* start write transfer, if not started */
	    usbd_transfer_start(sc->sc_xfer[0]);
	}
	return;
}

static void
ural_config_copy(struct ural_softc *sc, 
		 struct ural_config_copy *cc, u_int16_t refcount)
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

	sc->sc_flags |= URAL_FLAG_WAIT_COMMAND;
	return;
}

static const char *
ural_get_rf(int rev)
{
	switch (rev) {
	case RAL_RF_2522:	return "RT2522";
	case RAL_RF_2523:	return "RT2523";
	case RAL_RF_2524:	return "RT2524";
	case RAL_RF_2525:	return "RT2525";
	case RAL_RF_2525E:	return "RT2525e";
	case RAL_RF_2526:	return "RT2526";
	case RAL_RF_5222:	return "RT5222";
	default:		return "unknown";
	}
}

/* quickly determine if a given rate is CCK or OFDM */
#define RAL_RATE_IS_OFDM(rate) (((rate) >= 12) && ((rate) != 22))

#define RAL_ACK_SIZE	14	/* 10 + 4(FCS) */
#define RAL_CTS_SIZE	14	/* 10 + 4(FCS) */

#define RAL_SIFS		10	/* us */

#define RAL_RXTX_TURNAROUND	5	/* us */

/*------------------------------------------------------------------------*
 * ural_rxrate - this function is only used by the Rx radiotap code
 *------------------------------------------------------------------------*/
static int
ural_rxrate(struct ural_rx_desc *desc)
{
	if (le32toh(desc->flags) & RAL_RX_OFDM) {
		/* reverse function of ural_plcp_signal */
		switch (desc->rate) {
		case 0xb:	return 12;
		case 0xf:	return 18;
		case 0xa:	return 24;
		case 0xe:	return 36;
		case 0x9:	return 48;
		case 0xd:	return 72;
		case 0x8:	return 96;
		case 0xc:	return 108;
		}
	} else {
		if (desc->rate == 10)
			return 2;
		if (desc->rate == 20)
			return 4;
		if (desc->rate == 55)
			return 11;
		if (desc->rate == 110)
			return 22;
	}
	return 2;  /* should not get there */
}

/*------------------------------------------------------------------------*
 * ural_bulk_read_callback - data read "thread"
 *------------------------------------------------------------------------*/
static void
ural_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct ural_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = ic->ic_ifp;
	struct ieee80211_node *ni;
	struct mbuf *m = NULL;
	u_int32_t flags;
	u_int32_t max_len;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= URAL_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;

 tr_transferred:
	if (xfer->actlen < (RAL_RX_DESC_SIZE + IEEE80211_MIN_LEN)) {
	    DPRINTF(sc, 0, "too short transfer, "
		    "%d bytes\n", xfer->actlen);
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	m = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR);

	if (m == NULL) {
	    DPRINTF(sc, 0, "could not allocate mbuf\n");
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	max_len = (xfer->actlen - RAL_RX_DESC_SIZE);

	usbd_copy_out(&(xfer->buf_data), 0, m->m_data, max_len);

	usbd_copy_out(&(xfer->buf_data), max_len,
		      &(sc->sc_rx_desc), RAL_RX_DESC_SIZE);

	flags = le32toh(sc->sc_rx_desc.flags);

	if ((flags & RAL_RX_PHY_ERROR) ||
	    (flags & RAL_RX_CRC_ERROR)) {
	    /*
	     * This should not happen since we did not 
	     * request to receive those frames when we 
	     * filled RAL_TXRX_CSR2:
	     */
	    DPRINTF(sc, 5, "PHY or CRC error\n");
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	/* finalize mbuf */
	m->m_pkthdr.rcvif = ifp;
	m->m_pkthdr.len = m->m_len = (flags >> 16) & 0xfff;
	m->m_flags |= M_HASFCS;	/* HW leaves FCS */

	if (m->m_len > max_len) {
	    DPRINTF(sc, 0, "invalid length in RX "
		    "descriptor, %u bytes, received %u bytes\n",
		    m->m_len, max_len);
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	if (sc->sc_drvbpf != NULL) {
	    struct ural_rx_radiotap_header *tap = &(sc->sc_rxtap.h);

	    tap->wr_flags = IEEE80211_RADIOTAP_F_FCS;   
	    tap->wr_rate = ural_rxrate(&sc->sc_rx_desc);
	    tap->wr_chan_freq = htole16(ic->ic_curchan->ic_freq);
	    tap->wr_chan_flags = htole16(ic->ic_curchan->ic_flags);
	    tap->wr_antenna = sc->sc_rx_ant;
	    tap->wr_antsignal = sc->sc_rx_desc.rssi;

	    bpf_mtap2(sc->sc_drvbpf, tap, sc->sc_rxtap_len, m);
	}

	ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)
				   (m->m_data));

	/* send the frame to the 802.11 layer */
	ieee80211_input(ic, m, ni, sc->sc_rx_desc.rssi, 0);

	/* node is no longer needed */
	ieee80211_free_node(ni);

	DPRINTF(sc, 14, "rx done\n");

	m = NULL;

 tr_setup:
	if (m) {
	    m_freem(m);
	}

	if (sc->sc_flags & URAL_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	    return;
	}

	usbd_start_hardware(xfer);
	return;
}

static void
ural_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ural_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~URAL_FLAG_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~URAL_FLAG_READ_STALL;
	printf("%s: bulk read pipe stopped\n",
	       sc->sc_name);
	return;
}

/*------------------------------------------------------------------------*
 * ural_ack_rate - return the expected ack rate for a frame 
 *                 transmitted at rate "rate".
 *
 * XXX: this should depend on the destination node basic rate set.
 *------------------------------------------------------------------------*/
static u_int16_t
ural_ack_rate(struct ieee80211com *ic, u_int16_t rate)
{
	switch (rate) {
	/* CCK rates */
	case 2:
		return 2;
	case 4:
	case 11:
	case 22:
		return (ic->ic_curmode == IEEE80211_MODE_11B) ? 4 : rate;

	/* OFDM rates */
	case 12:
	case 18:
		return 12;
	case 24:
	case 36:
		return 24;
	case 48:
	case 72:
	case 96:
	case 108:
		return 48;
	}

	/* default to 1Mbps */
	return 2;
}

/*------------------------------------------------------------------------*
 * ural_txtime - compute the duration (in us) needed to transmit "len" 
 * bytes at rate "rate". The function automatically determines the 
 * operating mode depending on the given rate. `flags' indicates 
 * whether short preamble is in use or not.
 *------------------------------------------------------------------------*/
static u_int16_t
ural_txtime(struct ural_softc *sc, u_int16_t len, u_int16_t rate, u_int32_t flags)
{
	u_int16_t txtime;

	if (rate < 2) {
	    DPRINTF(sc, 0, "rate < 2!\n");

	    /* avoid division by zero */
	    rate = 2;
	}

	if (RAL_RATE_IS_OFDM(rate)) {
		/* IEEE Std 802.11a-1999, pp. 37 */
		txtime = (8 + (4 * len) + 3 + rate - 1) / rate;
		txtime = 16 + 4 + (4 * txtime) + 6;
	} else {
		/* IEEE Std 802.11b-1999, pp. 28 */
		txtime = ((16 * len) + rate - 1) / rate;
		if ((rate != 2) && (flags & IEEE80211_F_SHPREAMBLE))
			txtime +=  72 + 24;
		else
			txtime += 144 + 48;
	}
	return txtime;
}

static u_int8_t
ural_plcp_signal(u_int16_t rate)
{
	switch (rate) {
	/* CCK rates (returned values are device-dependent) */
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

static void
ural_setup_tx_desc(struct ural_softc *sc, u_int32_t flags, u_int16_t len, 
		   u_int16_t rate)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	u_int16_t plcp_length;
	u_int8_t remainder;

	if (rate < 2) {
	    DPRINTF(sc, 0, "rate < 2!\n");

	    /* avoid division by zero */
	    rate = 2;
	}

	sc->sc_tx_desc.flags = htole32(flags);
	sc->sc_tx_desc.flags |= htole32(RAL_TX_NEWSEQ);
	sc->sc_tx_desc.flags |= htole32(len << 16);

	sc->sc_tx_desc.wme = htole16(RAL_AIFSN(2) | 
				     RAL_LOGCWMIN(3) | 
				     RAL_LOGCWMAX(5));
	sc->sc_tx_desc.wme |= htole16(RAL_IVOFFSET
				      (sizeof(struct ieee80211_frame)));

	/* setup PLCP fields */
	sc->sc_tx_desc.plcp_signal = ural_plcp_signal(rate);
	sc->sc_tx_desc.plcp_service = 4;

	len += IEEE80211_CRC_LEN;

	if (RAL_RATE_IS_OFDM(rate)) {
		sc->sc_tx_desc.flags |= htole32(RAL_TX_OFDM);

		plcp_length = len & 0xfff;
		sc->sc_tx_desc.plcp_length_hi = plcp_length >> 6;
		sc->sc_tx_desc.plcp_length_lo = plcp_length & 0x3f;

	} else {
		plcp_length = ((16 * len) + rate - 1) / rate;
		if (rate == 22) {
			remainder = (16 * len) % 22;
			if ((remainder != 0) && (remainder < 7)) {
				sc->sc_tx_desc.plcp_service |= 
				  RAL_PLCP_LENGEXT;
			}
		}

		sc->sc_tx_desc.plcp_length_hi = plcp_length >> 8;
		sc->sc_tx_desc.plcp_length_lo = plcp_length & 0xff;

		if ((rate != 2) && (ic->ic_flags & IEEE80211_F_SHPREAMBLE)) {
			sc->sc_tx_desc.plcp_signal |= 0x08;
		}
	}

	sc->sc_tx_desc.iv = 0;
	sc->sc_tx_desc.eiv = 0;
	return;
}

#define RAL_TX_TIMEOUT	5000 /* ms */

/*------------------------------------------------------------------------*
 * ural_bulk_write_callback - data write "thread"
 *------------------------------------------------------------------------*/
static void
ural_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct ural_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = sc->sc_ic.ic_ifp;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni = NULL;
	struct ieee80211_key *k;
	struct ether_header *eh;
	struct mbuf *m0 = NULL;
	u_int32_t flags;
	u_int16_t dur;
	u_int16_t rate;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 10, "transfer error, %s\n",
		 usbd_errstr(xfer->error));

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= URAL_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}

	ifp->if_oerrors++;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	return;


 tr_transferred:
	DPRINTF(sc, 10, "transfer complete\n");

	ifp->if_opackets++;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	sc->sc_tx_timer = 0;


 tr_setup:
	if (sc->sc_flags & URAL_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    goto out_active;
	}

	if (sc->sc_flags & URAL_FLAG_WAIT_COMMAND) {
	    /* don't send anything while a command
	     * is pending !
	     */
	    goto done;
	}

	if (sc->sc_flags &   URAL_FLAG_SEND_BYTE_FRAME) {
	    sc->sc_flags &= ~URAL_FLAG_SEND_BYTE_FRAME;

	    usbd_bzero(&(xfer->buf_data), 0, 1);

	    xfer->length = 1; /* bytes */
	    xfer->timeout = 1000; /* ms */

	    usbd_start_hardware(xfer);
	    goto out_active;
	}

	if (sc->sc_flags &   URAL_FLAG_SEND_BCN_FRAME) {
	    sc->sc_flags &= ~URAL_FLAG_SEND_BCN_FRAME;

	    flags = sc->sc_bcn_flags;
	    rate = sc->sc_bcn_rate;
	    m0 = sc->sc_bcn_mbuf;

	    ural_tx_bcn_complete(sc);

	    goto transmit_frame;
	}

	IF_DEQUEUE(&(ic->ic_mgtq), m0);

	if (m0) {

	    ni = (struct ieee80211_node *)(m0->m_pkthdr.rcvif);
	    m0->m_pkthdr.rcvif = NULL;

	    if (ic->ic_rawbpf != NULL) {
	        bpf_mtap(ic->ic_rawbpf, m0);
	    }

	    flags = 0;
	    rate = (IEEE80211_IS_CHAN_5GHZ(ic->ic_curchan) ? 12 : 2);

	    wh = mtod(m0, struct ieee80211_frame *);

	    if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {

	        flags |= RAL_TX_ACK;

		dur = ural_txtime(sc, RAL_ACK_SIZE, rate, ic->ic_flags) + RAL_SIFS;
		*(u_int16_t *)(wh->i_dur) = htole16(dur);

		/* tell hardware to add timestamp for probe responses */
		if (((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == 
		     IEEE80211_FC0_TYPE_MGT) &&
		    ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) ==
		     IEEE80211_FC0_SUBTYPE_PROBE_RESP)) {
			flags |= RAL_TX_TIMESTAMP;
		}
	    }

	transmit_frame:

	    if (m0->m_pkthdr.len > MCLBYTES) {
	        DPRINTF(sc, 0, "data overflow, %u bytes\n",
			m0->m_pkthdr.len);
		m0->m_pkthdr.len = MCLBYTES;
	    }

	    if (sc->sc_drvbpf != NULL) {
	        struct ural_tx_radiotap_header *tap = &(sc->sc_txtap.h);

		tap->wt_flags = 0;
		tap->wt_rate = rate;
		tap->wt_chan_freq = htole16(ic->ic_curchan->ic_freq);
		tap->wt_chan_flags = htole16(ic->ic_curchan->ic_flags);
		tap->wt_antenna = sc->sc_tx_ant;

		bpf_mtap2(sc->sc_drvbpf, tap, sc->sc_txtap_len, m0);
	    }

	    ural_setup_tx_desc(sc, flags, m0->m_pkthdr.len, rate);

	    usbd_copy_in(&(xfer->buf_data), 0, &(sc->sc_tx_desc), 
			 RAL_TX_DESC_SIZE);

	    usbd_m_copy_in(&(xfer->buf_data), RAL_TX_DESC_SIZE, 
			   m0, 0, m0->m_pkthdr.len);

	    /* align end on a 2-bytes boundary */
	    xfer->length = (RAL_TX_DESC_SIZE + m0->m_pkthdr.len + 1) & ~1;

	    /*
	     * No space left in the last URB to store the 
	     * extra 2 bytes, force sending of another URB:
	     */
	    if ((xfer->length % 64) == 0) {
	         xfer->length += 2;
	    }

	    /* zero the extra bytes */

	    usbd_bzero(&(xfer->buf_data), RAL_TX_DESC_SIZE + m0->m_pkthdr.len,
		       xfer->length - (RAL_TX_DESC_SIZE + m0->m_pkthdr.len));
	
	    xfer->timeout = RAL_TX_TIMEOUT;

	    DPRINTF(sc, 10, "sending frame len=%u rate=%u xfer len=%u\n",
		     m0->m_pkthdr.len, rate, xfer->length);

	    m_freem(m0);

	    if (ni) {
	        ieee80211_free_node(ni);
	    }

	    usbd_start_hardware(xfer);
	    goto out_active;
	}

	if (ic->ic_state != IEEE80211_S_RUN) {
	    goto done;
	}

	IFQ_DRV_DEQUEUE(&ifp->if_snd, m0);

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

	    wh = mtod(m0, struct ieee80211_frame *);

	    flags = 0;
	    rate = ((ic->ic_fixed_rate != IEEE80211_FIXED_RATE_NONE) ?
		    ic->ic_bss->ni_rates.rs_rates[ic->ic_fixed_rate] :
		    ni->ni_rates.rs_rates[ni->ni_txrate]);

	    rate &= IEEE80211_RATE_VAL;

	    if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
		k = ieee80211_crypto_encap(ic, ni, m0);
		if (k == NULL) {
		    goto error;
		}

		/* packet header may have moved, reset our local pointer */
		wh = mtod(m0, struct ieee80211_frame *);
	    }

	    if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		flags |= RAL_TX_ACK;
		flags |= RAL_TX_RETRY(7);

		dur = ural_txtime(sc, RAL_ACK_SIZE, ural_ack_rate(ic, rate),
				  ic->ic_flags) + RAL_SIFS;
		*(u_int16_t *)(wh->i_dur) = htole16(dur);
	    }

	    goto transmit_frame;
	}

	goto done;

 out_active:
	ifp->if_drv_flags |= IFF_DRV_OACTIVE;
	sc->sc_tx_timer = (5*8);

 done:
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
ural_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ural_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~URAL_FLAG_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~URAL_FLAG_WRITE_STALL;
	printf("%s: bulk write pipe stopped\n",
	       sc->sc_name);
	return;
}

static void
ural_watchdog(void *arg)
{
	struct ural_softc *sc = arg;
	struct ieee80211com *ic = &(sc->sc_ic);

	mtx_assert(&(sc->sc_mtx), MA_OWNED);

	if ((sc->sc_amrr_timer) &&
	    (--sc->sc_amrr_timer == 0)) {

	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &ural_cfg_amrr_timeout, 0);
	}

	if ((sc->sc_tx_timer) &&
	    (--sc->sc_tx_timer == 0)) {

	    printf("%s: device timeout\n", sc->sc_name);
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &ural_cfg_init, 0);
	}

	if ((sc->sc_if_timer) &&
	    (--sc->sc_if_timer == 0)) {

	    DPRINTF(sc, 0, "watchdog timeout\n");

	    /* restart timer */
	    sc->sc_if_timer = (1*8);

	    ieee80211_watchdog(ic);
	}

	if ((sc->sc_scan_timer) &&
	    (--sc->sc_scan_timer == 0)) {
	    ieee80211_next_scan(ic);
	}

	__callout_reset(&(sc->sc_watchdog), 
			hz / 8, &ural_watchdog, sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

/*========================================================================*
 * IF-net callbacks
 *========================================================================*/

static void
ural_init_cb(void *arg)
{
	struct ural_softc *sc = arg;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &ural_cfg_init, 0);
	mtx_unlock(&(sc->sc_mtx));

	return;
}

static int
ural_ioctl_cb(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct ural_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &(sc->sc_ic);
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	switch (cmd) {
	case SIOCSIFFLAGS:

	    if (ifp->if_flags & IFF_UP) {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &ural_cfg_update_promisc, 0);
		} else {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &ural_cfg_init, 0); 
		}
	    } else {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &ural_cfg_stop, 0);
		}
	    }
	    break;

	default:
		error = ieee80211_ioctl(ic, cmd, data);
	}

	if (error == ENETRESET) {
	    if ((ifp->if_flags & IFF_UP) &&
		(ifp->if_drv_flags & IFF_DRV_RUNNING) &&
		(ic->ic_roaming != IEEE80211_ROAMING_MANUAL)) {
	        usbd_config_td_queue_command
		  (&(sc->sc_config_td), &ural_cfg_init, 0);
	    }
	    error = 0;
	}

	mtx_unlock(&(sc->sc_mtx));

	return error;
}

static void
ural_start_cb(struct ifnet *ifp)
{
	struct ural_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	if ((sc->sc_flags & URAL_FLAG_LL_READY) &&
	    (sc->sc_flags & URAL_FLAG_HL_READY)) {
	    /* start write transfer, if not started */
	    usbd_transfer_start(sc->sc_xfer[0]);
	}

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static int
ural_media_change_cb(struct ifnet *ifp)
{
	struct ural_softc *sc = ifp->if_softc;
	int error;

	mtx_lock(&(sc->sc_mtx));

	error = ieee80211_media_change(ifp);
	if (error != ENETRESET) {
	    goto done;
	}

	if ((ifp->if_flags & IFF_UP) &&
	    (ifp->if_drv_flags & IFF_DRV_RUNNING)) {
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &ural_cfg_init, 0);
	}

 done:
	mtx_unlock(&(sc->sc_mtx));

	return error;
}

/*
 * This function allows for fast channel switching in monitor mode (used by
 * net-mgmt/kismet). In IBSS mode, we must explicitly reset the interface to
 * generate a new beacon frame.
 */
static int
ural_reset_cb(struct ifnet *ifp)
{
	struct ural_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &(sc->sc_ic);

	mtx_lock(&(sc->sc_mtx));

	if (ic->ic_opmode != IEEE80211_M_MONITOR) {
		return ENETRESET;
	}

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &ural_cfg_set_chan, 0);

	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

static int
ural_newstate_cb(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	struct ural_softc *sc = ic->ic_ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	DPRINTF(sc, 0, "setting new state: %d\n", nstate);

	/* stop timers */

	sc->sc_amrr_timer = 0;
	sc->sc_scan_timer = 0;
	sc->sc_if_timer = 8;

	/* set new state */

	switch (nstate) {
	case IEEE80211_S_INIT:
	    if (sc->sc_ic.ic_state == IEEE80211_S_RUN) {
	      usbd_config_td_queue_command
		(&(sc->sc_config_td), &ural_cfg_disable_tsf_sync, 0);
	    }
	    sc->sc_if_timer = 0;
	    break;

	case IEEE80211_S_SCAN:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &ural_cfg_set_chan, 0);
	    sc->sc_scan_timer = 3;
	    break;

	case IEEE80211_S_AUTH:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &ural_cfg_set_chan, 0);
	    break;

	case IEEE80211_S_ASSOC:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &ural_cfg_set_chan, 0);
	    break;

	case IEEE80211_S_RUN:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &ural_cfg_set_run, 0);
	    break;
	}

	(sc->sc_newstate)(ic, nstate, arg);

	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

/*========================================================================*
 * configure sub-routines, ural_cfg_xxx
 *========================================================================*/

static void
ural_tx_bcn_complete(struct ural_softc *sc)
{
	if (sc->sc_bcn_mbuf) {
	    m_freem(sc->sc_bcn_mbuf);
	    sc->sc_bcn_mbuf = NULL;

	    sc->sc_flags &= ~(URAL_FLAG_SEND_BYTE_FRAME|
			      URAL_FLAG_SEND_BCN_FRAME);

	    wakeup(&(sc->sc_wakeup_bcn));
	}
	return;
}

static void
ural_cfg_tx_bcn(struct ural_softc *sc)
{
	register int error;

	if ((sc->sc_flags & URAL_FLAG_LL_READY) &&
	    (sc->sc_flags & URAL_FLAG_HL_READY)) {

	    struct ieee80211com *ic = &(sc->sc_ic);
	    struct ieee80211_node *ni = sc->sc_ic.ic_bss;
	    struct mbuf *m;

	    m = ieee80211_beacon_alloc(ic, ni, &sc->sc_bo);

	    if (m == NULL) {
	        printf("%s: could not allocate "
		       "beacon frame\n", sc->sc_name);
		return;
	    }

	    sc->sc_flags |= (URAL_FLAG_SEND_BYTE_FRAME|
			     URAL_FLAG_SEND_BCN_FRAME);

	    sc->sc_bcn_flags = (RAL_TX_IFS_NEWBACKOFF|RAL_TX_TIMESTAMP);
	    sc->sc_bcn_rate = (IEEE80211_IS_CHAN_5GHZ(ni->ni_chan) ? 12 : 2);
	    sc->sc_bcn_mbuf = m;

	    /* start transfer, if not started */

	    usbd_transfer_start(sc->sc_xfer[0]);

	    while (sc->sc_flags & (URAL_FLAG_SEND_BYTE_FRAME|
				   URAL_FLAG_SEND_BCN_FRAME)) {

		if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
		    break;
		}

	        error = msleep(&(sc->sc_wakeup_bcn), &(sc->sc_mtx), 
			       0, "ural beacon sleep", 0);
	    }
	}
	return;
}

static void
ural_cfg_set_chan(struct ural_softc *sc,
		  struct ural_config_copy *cc, u_int16_t refcount)
{
	u_int32_t i;
	u_int32_t chan;
	u_int8_t power;
	u_int8_t tmp;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	chan = cc->ic_curchan.chan_to_ieee;

	if ((chan == 0) ||
	    (chan == IEEE80211_CHAN_ANY)) {
	    /* nothing to do */
	    return;
	}

	if (cc->ic_curchan.chan_is_2ghz)
		power = min(sc->sc_txpow[chan - 1], 31);
	else
		power = 31;

	/* adjust txpower using ifconfig settings */
	power -= (100 - cc->ic_txpowlimit) / 8;

	DPRINTF(sc, 2, "setting channel to %u, "
		 "tx-power to %u\n", chan, power);

	switch (sc->sc_rf_rev) {
	case RAL_RF_2522:
	    ural_cfg_rf_write(sc, RAL_RF1, 0x00814);
	    ural_cfg_rf_write(sc, RAL_RF2, ural_rf2522_r2[chan - 1]);
	    ural_cfg_rf_write(sc, RAL_RF3, (power << 7) | 0x00040);
	    break;

	case RAL_RF_2523:
	    ural_cfg_rf_write(sc, RAL_RF1, 0x08804);
	    ural_cfg_rf_write(sc, RAL_RF2, ural_rf2523_r2[chan - 1]);
	    ural_cfg_rf_write(sc, RAL_RF3, (power << 7) | 0x38044);
	    ural_cfg_rf_write(sc, RAL_RF4, (chan == 14) ? 0x00280 : 0x00286);
	    break;

	case RAL_RF_2524:
	    ural_cfg_rf_write(sc, RAL_RF1, 0x0c808);
	    ural_cfg_rf_write(sc, RAL_RF2, ural_rf2524_r2[chan - 1]);
	    ural_cfg_rf_write(sc, RAL_RF3, (power << 7) | 0x00040);
	    ural_cfg_rf_write(sc, RAL_RF4, (chan == 14) ? 0x00280 : 0x00286);
	    break;

	case RAL_RF_2525:
	    ural_cfg_rf_write(sc, RAL_RF1, 0x08808);
	    ural_cfg_rf_write(sc, RAL_RF2, ural_rf2525_hi_r2[chan - 1]);
	    ural_cfg_rf_write(sc, RAL_RF3, (power << 7) | 0x18044);
	    ural_cfg_rf_write(sc, RAL_RF4, (chan == 14) ? 0x00280 : 0x00286);

	    ural_cfg_rf_write(sc, RAL_RF1, 0x08808);
	    ural_cfg_rf_write(sc, RAL_RF2, ural_rf2525_r2[chan - 1]);
	    ural_cfg_rf_write(sc, RAL_RF3, (power << 7) | 0x18044);
	    ural_cfg_rf_write(sc, RAL_RF4, (chan == 14) ? 0x00280 : 0x00286);
	    break;

	case RAL_RF_2525E:
	    ural_cfg_rf_write(sc, RAL_RF1, 0x08808);
	    ural_cfg_rf_write(sc, RAL_RF2, ural_rf2525e_r2[chan - 1]);
	    ural_cfg_rf_write(sc, RAL_RF3, (power << 7) | 0x18044);
	    ural_cfg_rf_write(sc, RAL_RF4, (chan == 14) ? 0x00286 : 0x00282);
	    break;

	case RAL_RF_2526:
	    ural_cfg_rf_write(sc, RAL_RF2, ural_rf2526_hi_r2[chan - 1]);
	    ural_cfg_rf_write(sc, RAL_RF4, (chan & 1) ? 0x00386 : 0x00381);
	    ural_cfg_rf_write(sc, RAL_RF1, 0x08804);

	    ural_cfg_rf_write(sc, RAL_RF2, ural_rf2526_r2[chan - 1]);
	    ural_cfg_rf_write(sc, RAL_RF3, (power << 7) | 0x18044);
	    ural_cfg_rf_write(sc, RAL_RF4, (chan & 1) ? 0x00386 : 0x00381);
	    break;

	/* dual-band RF */
	case RAL_RF_5222:
		for (i = 0; i < (sizeof(ural_rf5222)/
				 sizeof(ural_rf5222[0])); i++) {
		  if (ural_rf5222[i].chan == chan) {
		      ural_cfg_rf_write(sc, RAL_RF1, ural_rf5222[i].r1);
		      ural_cfg_rf_write(sc, RAL_RF2, ural_rf5222[i].r2);
		      ural_cfg_rf_write(sc, RAL_RF3, (power << 7) | 0x00040);
		      ural_cfg_rf_write(sc, RAL_RF4, ural_rf5222[i].r4);
		      break;
		  }
		}
	    break;
	}

	if ((cc->ic_opmode != IEEE80211_M_MONITOR) &&
	    (cc->ic_state != IEEE80211_S_SCAN)) {

		/* set Japan filter bit for channel 14 */
		tmp = ural_cfg_bbp_read(sc, 70);

		if (chan == 14) {
		    tmp |=  RAL_JAPAN_FILTER;
		} else {
		    tmp &= ~RAL_JAPAN_FILTER;
		}

		ural_cfg_bbp_write(sc, 70, tmp);

		/* clear CRC errors */
		ural_cfg_read(sc, RAL_STA_CSR0);

		/* wait a little */
		usbd_config_td_sleep(&(sc->sc_config_td), hz/100);

		ural_cfg_disable_rf_tune(sc);
	}
	return;
}

static void
ural_cfg_set_run(struct ural_softc *sc, 
		 struct ural_config_copy *cc, u_int16_t refcount)
{
	if (cc == NULL) {

	    /* immediate configuration */

	    struct ieee80211com *ic = &(sc->sc_ic);

	    /* enable automatic rate adaptation in STA mode */
	    if ((ic->ic_opmode == IEEE80211_M_STA) &&
		(ic->ic_fixed_rate == IEEE80211_FIXED_RATE_NONE)) {
	        ural_cfg_amrr_start(sc);
	    }
	    return;
	}

	/* delayed configuration */

	ural_cfg_set_chan(sc, cc, 0);

	if (cc->ic_opmode != IEEE80211_M_MONITOR) {
	    ural_cfg_update_slot(sc, cc, 0);
	    ural_cfg_set_txpreamble(sc, cc, 0);
	    ural_cfg_set_basicrates(sc, cc, 0);
	    ural_cfg_set_bssid(sc, cc->ic_bss.ni_bssid);
	}

	if ((cc->ic_opmode == IEEE80211_M_HOSTAP) ||
	    (cc->ic_opmode == IEEE80211_M_IBSS)) {
	    ural_cfg_tx_bcn(sc);
	}

	/* make tx led blink on tx (controlled by ASIC) */
	ural_cfg_write(sc, RAL_MAC_CSR20, 1);

	if (cc->ic_opmode != IEEE80211_M_MONITOR) {
	    ural_cfg_enable_tsf_sync(sc, cc , 0);
	}

	/* clear statistic registers (STA_CSR0 to STA_CSR10) */
	ural_cfg_read_multi(sc, RAL_STA_CSR0, sc->sc_sta, sizeof(sc->sc_sta));

	return;
}

/*------------------------------------------------------------------------*
 * ural_cfg_disable_rf_tune - disable RF auto-tuning
 *------------------------------------------------------------------------*/
static void
ural_cfg_disable_rf_tune(struct ural_softc *sc)
{
	u_int32_t tmp;

	if (sc->sc_rf_rev != RAL_RF_2523) {
	    tmp = sc->sc_rf_regs[RAL_RF1] & ~RAL_RF1_AUTOTUNE;
	    ural_cfg_rf_write(sc, RAL_RF1, tmp);
	}

	tmp = sc->sc_rf_regs[RAL_RF3] & ~RAL_RF3_AUTOTUNE;
	ural_cfg_rf_write(sc, RAL_RF3, tmp);

	DPRINTF(sc, 2, "disabling RF autotune\n");

	return;
}

/*------------------------------------------------------------------------*
 * ural_cfg_enable_tsf_sync - refer to IEEE Std 802.11-1999 pp. 123 
 * for more information on TSF synchronization
 *------------------------------------------------------------------------*/
static void
ural_cfg_enable_tsf_sync(struct ural_softc *sc,
			 struct ural_config_copy *cc, u_int16_t refcount)
{
	u_int16_t logcwmin;
	u_int16_t preload;
	u_int16_t tmp;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	/* first, disable TSF synchronization */
	ural_cfg_write(sc, RAL_TXRX_CSR19, 0);

	tmp = (16 * cc->ic_bss.ni_intval) << 4;
	ural_cfg_write(sc, RAL_TXRX_CSR18, tmp);

	logcwmin = (cc->ic_opmode == IEEE80211_M_IBSS) ? 2 : 0;
	preload = (cc->ic_opmode == IEEE80211_M_IBSS) ? 320 : 6;
	tmp = (logcwmin << 12) | preload;
	ural_cfg_write(sc, RAL_TXRX_CSR20, tmp);

	/* finally, enable TSF synchronization */
	tmp = RAL_ENABLE_TSF | RAL_ENABLE_TBCN;
	if (cc->ic_opmode == IEEE80211_M_STA)
		tmp |= RAL_ENABLE_TSF_SYNC(1);
	else
		tmp |= RAL_ENABLE_TSF_SYNC(2) | RAL_ENABLE_BEACON_GENERATOR;

	ural_cfg_write(sc, RAL_TXRX_CSR19, tmp);

	DPRINTF(sc, 0, "enabling TSF synchronization\n");

	return;
}

static void
ural_cfg_disable_tsf_sync(struct ural_softc *sc,
			  struct ural_config_copy *cc, u_int16_t refcount)
{
	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	/* abort TSF synchronization */
	ural_cfg_write(sc, RAL_TXRX_CSR19, 0);

	/* force tx led to stop blinking */
	ural_cfg_write(sc, RAL_MAC_CSR20, 0);

	return;
}

static void
ural_cfg_update_slot(struct ural_softc *sc,
		     struct ural_config_copy *cc, u_int16_t refcount)
{
	u_int16_t slottime;
	u_int16_t sifs;
	u_int16_t eifs;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	slottime = (cc->ic_flags & IEEE80211_F_SHSLOT) ? 9 : 20;

	/*
	 * These settings may sound a bit inconsistent but this is what the
	 * reference driver does.
	 */
	if (cc->ic_curmode == IEEE80211_MODE_11B) {
		sifs = 16 - RAL_RXTX_TURNAROUND;
		eifs = 364;
	} else {
		sifs = 10 - RAL_RXTX_TURNAROUND;
		eifs = 64;
	}

	ural_cfg_write(sc, RAL_MAC_CSR10, slottime);
	ural_cfg_write(sc, RAL_MAC_CSR11, sifs);
	ural_cfg_write(sc, RAL_MAC_CSR12, eifs);
	return;
}

static void
ural_cfg_set_txpreamble(struct ural_softc *sc,
			struct ural_config_copy *cc, u_int16_t refcount)
{
	u_int16_t tmp;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	tmp = ural_cfg_read(sc, RAL_TXRX_CSR10);

	if (cc->ic_flags & IEEE80211_F_SHPREAMBLE) {
		tmp |= RAL_SHORT_PREAMBLE;
	} else {
		tmp &= ~RAL_SHORT_PREAMBLE;
	}

	ural_cfg_write(sc, RAL_TXRX_CSR10, tmp);
	return;
}

static void
ural_cfg_set_basicrates(struct ural_softc *sc,
			struct ural_config_copy *cc, u_int16_t refcount)
{
	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	/* update basic rate set */

	if (cc->ic_curmode == IEEE80211_MODE_11B) {
		/* 11b basic rates: 1, 2Mbps */
		ural_cfg_write(sc, RAL_TXRX_CSR11, 0x3);
	} else if (cc->ic_bss.ni_chan.chan_is_5ghz) {
		/* 11a basic rates: 6, 12, 24Mbps */
		ural_cfg_write(sc, RAL_TXRX_CSR11, 0x150);
	} else {
		/* 11g basic rates: 1, 2, 5.5, 11, 6, 12, 24Mbps */
		ural_cfg_write(sc, RAL_TXRX_CSR11, 0x15f);
	}
	return;
}

static void
ural_cfg_set_bssid(struct ural_softc *sc, const u_int8_t *bssid)
{
	u_int16_t tmp;

	tmp = bssid[0] | (bssid[1] << 8);
	ural_cfg_write(sc, RAL_MAC_CSR5, tmp);

	tmp = bssid[2] | (bssid[3] << 8);
	ural_cfg_write(sc, RAL_MAC_CSR6, tmp);

	tmp = bssid[4] | (bssid[5] << 8);
	ural_cfg_write(sc, RAL_MAC_CSR7, tmp);

	DPRINTF(sc, 0, "setting BSSID to 0x%02x%02x%02x%02x%02x%02x\n", 
		bssid[5], bssid[4], bssid[3], 
		bssid[2], bssid[1], bssid[0]);
	return;
}

static void
ural_cfg_set_macaddr(struct ural_softc *sc, const u_int8_t *addr)
{
	u_int16_t tmp;

	tmp = addr[0] | (addr[1] << 8);
	ural_cfg_write(sc, RAL_MAC_CSR2, tmp);

	tmp = addr[2] | (addr[3] << 8);
 	ural_cfg_write(sc, RAL_MAC_CSR3, tmp);

	tmp = addr[4] | (addr[5] << 8);
	ural_cfg_write(sc, RAL_MAC_CSR4, tmp);

	DPRINTF(sc, 0, "setting MAC to 0x%02x%02x%02x%02x%02x%02x\n", 
		addr[5], addr[4], addr[3], 
		addr[2], addr[1], addr[0]);
	return;
}

static void
ural_cfg_update_promisc(struct ural_softc *sc,
			struct ural_config_copy *cc, u_int16_t refcount)
{
	u_int16_t tmp;

	if (cc == NULL) {
	    /* nothing to do */
	    return;
	}

	tmp = ural_cfg_read(sc, RAL_TXRX_CSR2);

	if (cc->if_flags & IFF_PROMISC) {
	    tmp &= ~RAL_DROP_NOT_TO_ME;
	} else {
	    tmp |= RAL_DROP_NOT_TO_ME;
	}

	ural_cfg_write(sc, RAL_TXRX_CSR2, tmp);

	DPRINTF(sc, 0, "%s promiscuous mode\n", 
		(cc->if_flags & IFF_PROMISC) ? 
		"entering" : "leaving");
	return;
}

static void
ural_cfg_set_txantenna(struct ural_softc *sc, u_int8_t antenna)
{
	u_int16_t tmp;
	u_int8_t tx;

	tx = ural_cfg_bbp_read(sc, RAL_BBP_TX) & ~RAL_BBP_ANTMASK;
	if (antenna == 1)
		tx |= RAL_BBP_ANTA;
	else if (antenna == 2)
		tx |= RAL_BBP_ANTB;
	else
		tx |= RAL_BBP_DIVERSITY;

	/* need to force I/Q flip for RF 2525e, 2526 and 5222 */
	if ((sc->sc_rf_rev == RAL_RF_2525E) ||
	    (sc->sc_rf_rev == RAL_RF_2526) ||
	    (sc->sc_rf_rev == RAL_RF_5222)) {
		tx |= RAL_BBP_FLIPIQ;
	}

	ural_cfg_bbp_write(sc, RAL_BBP_TX, tx);

	/* update values in PHY_CSR5 and PHY_CSR6 */
	tmp = ural_cfg_read(sc, RAL_PHY_CSR5) & ~0x7;
	ural_cfg_write(sc, RAL_PHY_CSR5, tmp | (tx & 0x7));

	tmp = ural_cfg_read(sc, RAL_PHY_CSR6) & ~0x7;
	ural_cfg_write(sc, RAL_PHY_CSR6, tmp | (tx & 0x7));

	return;
}

static void
ural_cfg_set_rxantenna(struct ural_softc *sc, u_int8_t antenna)
{
	u_int8_t rx;

	rx = ural_cfg_bbp_read(sc, RAL_BBP_RX) & ~RAL_BBP_ANTMASK;
	if (antenna == 1)
		rx |= RAL_BBP_ANTA;
	else if (antenna == 2)
		rx |= RAL_BBP_ANTB;
	else
		rx |= RAL_BBP_DIVERSITY;

	/* need to force no I/Q flip for RF 2525e and 2526 */

	if ((sc->sc_rf_rev == RAL_RF_2525E) || 
	    (sc->sc_rf_rev == RAL_RF_2526)) {
		rx &= ~RAL_BBP_FLIPIQ;
	}

	ural_cfg_bbp_write(sc, RAL_BBP_RX, rx);
	return;
}

static void
ural_cfg_read_eeprom(struct ural_softc *sc)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	u_int16_t val;

	ural_cfg_eeprom_read(sc, RAL_EEPROM_CONFIG0, &val, 2);

	val = le16toh(val);

	sc->sc_rf_rev   = (val >> 11) & 0x7;
	sc->sc_hw_radio = (val >> 10) & 0x1;
	sc->sc_led_mode = (val >> 6)  & 0x7;
	sc->sc_rx_ant   = (val >> 4)  & 0x3;
	sc->sc_tx_ant   = (val >> 2)  & 0x3;
	sc->sc_nb_ant   = (val & 0x3);

	DPRINTF(sc, 0, "val = 0x%04x\n", val);

	/* read MAC address */
	ural_cfg_eeprom_read(sc, RAL_EEPROM_ADDRESS, ic->ic_myaddr, 
			     sizeof(ic->ic_myaddr));

	/* read default values for BBP registers */
	ural_cfg_eeprom_read(sc, RAL_EEPROM_BBP_BASE, sc->sc_bbp_prom, 
			     sizeof(sc->sc_bbp_prom));

	/* read Tx power for all b/g channels */
	ural_cfg_eeprom_read(sc, RAL_EEPROM_TXPOWER, sc->sc_txpow, 
			     sizeof(sc->sc_txpow));
	return;
}

static u_int8_t
ural_cfg_bbp_init(struct ural_softc *sc)
{
	u_int16_t i;
	u_int8_t to;

	/* wait for BBP to become ready */
	for (to = 0; ; to++) {
	    if (to < 10) {
	        if (ural_cfg_bbp_read(sc, RAL_BBP_VERSION) != 0) {
		    break;
		}
		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    break;
		}
	    } else {
	        printf("%s: timeout waiting for BBP\n",
		       sc->sc_name);
		return 1; /* failure */
	    }
	}

	/* initialize BBP registers to default values */
	for (i = 0; i < (sizeof(ural_def_bbp)/sizeof(ural_def_bbp[0])); i++) {
		ural_cfg_bbp_write(sc, ural_def_bbp[i].reg, 
			           ural_def_bbp[i].val);
	}

#if 0
	/* initialize BBP registers to values stored in EEPROM */
	for (i = 0; i < 16; i++) {
	    if (sc->sc_bbp_prom[i].reg == 0xff) {
	        continue;
	    }
	    ural_cfg_bbp_write(sc, sc->sc_bbp_prom[i].reg, 
			       sc->sc_bbp_prom[i].val);
	}
#endif
	return 0; /* success */
}

static void
ural_cfg_init(struct ural_softc *sc,
	      struct ural_config_copy *cc, u_int16_t refcount)
{
	u_int16_t tmp;
	u_int16_t i;
	u_int8_t to;

	if (cc == NULL) {

	    /* immediate configuration */

	    struct ieee80211com *ic = &(sc->sc_ic);
	    struct ifnet *ifp = sc->sc_ic.ic_ifp;

	    ural_cfg_stop(sc, NULL, 0);

	    ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	    ifp->if_drv_flags |= IFF_DRV_RUNNING;

	    sc->sc_flags |= URAL_FLAG_HL_READY;

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

	ural_cfg_set_testmode(sc);

	ural_cfg_write(sc, 0x308, 0x00f0);	/* XXX magic */

	ural_cfg_stop(sc, cc, 0);

	/* initialize MAC registers to default values */
	for (i = 0; i < (sizeof(ural_def_mac)/
			 sizeof(ural_def_mac[0])); i++) {
	    ural_cfg_write(sc, ural_def_mac[i].reg, 
			   ural_def_mac[i].val);
	}

	/* wait for BBP and RF to wake up (this can take a long time!) */
	for (to = 0; ; to++) {
	    if (to < 10) {
	        tmp = ural_cfg_read(sc, RAL_MAC_CSR17);
		  if ((tmp & (RAL_BBP_AWAKE | RAL_RF_AWAKE)) ==
		      (RAL_BBP_AWAKE | RAL_RF_AWAKE)) {
		      break;
		  }
		  if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		      break;
		  }
	    } else {
	        printf("%s: timeout waiting for "
		       "BBP/RF to wakeup\n", sc->sc_name);
		goto fail;
	    }
	}

	/* we're ready! */
	ural_cfg_write(sc, RAL_MAC_CSR1, RAL_HOST_READY);

	/* set basic rate set (will be updated later) */
	ural_cfg_write(sc, RAL_TXRX_CSR11, 0x15f);

	if (ural_cfg_bbp_init(sc)) {
	    goto fail;
	}

	/* set default BSS channel */
	ural_cfg_set_chan(sc, cc, 0);

	/* clear statistic registers (STA_CSR0 to STA_CSR10) */
	ural_cfg_read_multi(sc, RAL_STA_CSR0, sc->sc_sta, 
			    sizeof(sc->sc_sta));

	DPRINTF(sc, 0, "rx_ant=%d, tx_ant=%d\n",
		sc->sc_rx_ant, sc->sc_tx_ant);

	ural_cfg_set_txantenna(sc, sc->sc_tx_ant);
	ural_cfg_set_rxantenna(sc, sc->sc_rx_ant);

	ural_cfg_set_macaddr(sc, cc->ic_myaddr);

	/*
	 * Copy WEP keys into adapter's memory (SEC_CSR0 to SEC_CSR31).
	 */
	for (i = 0; i < IEEE80211_WEP_NKID; i++) {
	    ural_cfg_write_multi
	      (sc, ((cc->ic_crypto.cs_nw_keys[i].wk_keyix * 
		     IEEE80211_KEYBUF_SIZE) + RAL_SEC_CSR0),
	       cc->ic_crypto.cs_nw_keys[i].wk_key, IEEE80211_KEYBUF_SIZE);
	}

	/*
	 * make sure that the first transaction
	 * clears the stall:
	 */
	sc->sc_flags |= (URAL_FLAG_READ_STALL|
			 URAL_FLAG_WRITE_STALL|
			 URAL_FLAG_LL_READY);

	if ((sc->sc_flags & URAL_FLAG_LL_READY) &&
	    (sc->sc_flags & URAL_FLAG_HL_READY)) {

	    /* start the USB transfers, 
	     * if not already started:
	     */
	    usbd_transfer_start(sc->sc_xfer[1]);
	    usbd_transfer_start(sc->sc_xfer[0]);
	}

	/* 
	 * start Rx 
	 */
	tmp = RAL_DROP_PHY | RAL_DROP_CRC;
	if (cc->ic_opmode != IEEE80211_M_MONITOR) {

	    tmp |= (RAL_DROP_CTL | RAL_DROP_BAD_VERSION);

	    if (cc->ic_opmode != IEEE80211_M_HOSTAP) {
	        tmp |= RAL_DROP_TODS;
	    }
	    if (!(cc->if_flags & IFF_PROMISC)) {
	        tmp |= RAL_DROP_NOT_TO_ME;
	    }
	}
	ural_cfg_write(sc, RAL_TXRX_CSR2, tmp);

	return;

 fail:
	ural_cfg_stop(sc, NULL, 0);

	if (cc) {
	    ural_cfg_stop(sc, cc, 0);
	}
	return;
}

static void
ural_cfg_stop(struct ural_softc *sc,
	      struct ural_config_copy *cc, u_int16_t refcount)
{
	if (cc == NULL) {

	    /* immediate configuration */

	    struct ieee80211com *ic = &(sc->sc_ic);
	    struct ifnet *ifp = ic->ic_ifp;

	    if (ifp) {

	        ieee80211_new_state(ic, IEEE80211_S_INIT, -1);

		/* clear flags */
		ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | 
				       IFF_DRV_OACTIVE);
	    }

	    /* stop timers */
	    sc->sc_tx_timer = 0;
	    sc->sc_if_timer = 0;

	    sc->sc_flags &= ~(URAL_FLAG_HL_READY|
			      URAL_FLAG_LL_READY);

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

	    /* stop transmission of
	     * beacon frame, if any:
	     */
	    ural_tx_bcn_complete(sc);

	    return;
	}

	/* delayed configuration */

	/* disable Rx */
	ural_cfg_write(sc, RAL_TXRX_CSR2, RAL_DISABLE_RX);

	/* reset ASIC and BBP (but won't reset MAC registers!) */
	ural_cfg_write(sc, RAL_MAC_CSR1, RAL_RESET_ASIC | RAL_RESET_BBP);
	ural_cfg_write(sc, RAL_MAC_CSR1, 0);

	return;
}

#define URAL_AMRR_MIN_SUCCESS_THRESHOLD	 1
#define URAL_AMRR_MAX_SUCCESS_THRESHOLD	10

static void
ural_cfg_amrr_start(struct ural_softc *sc)
{
	struct ieee80211_node *ni = sc->sc_ic.ic_bss;
	struct ural_amrr *amrr = &sc->sc_amrr;
	u_int16_t i;

	amrr->success = 0;
	amrr->recovery = 0;
	amrr->txcnt = amrr->retrycnt = 0;
	amrr->success_threshold = URAL_AMRR_MIN_SUCCESS_THRESHOLD;

	/* set rate to some reasonable initial value */

	i = ni->ni_rates.rs_nrates;
	while(i) {
	    i--;
	    if ((ni->ni_rates.rs_rates[i] & IEEE80211_RATE_VAL) > 72) break;
	}
	ni->ni_txrate = i;

	sc->sc_amrr_timer = (1*8);
	return;
}

static void
ural_cfg_amrr_timeout(struct ural_softc *sc,
		      struct ural_config_copy *cc, u_int16_t refcount)
{
	struct ural_amrr *amrr = &sc->sc_amrr;
	struct ifnet *ifp = sc->sc_ic.ic_ifp;

	if (cc == NULL) {

	    /* restart timeout */
	    sc->sc_amrr_timer = (1*8);

	    return;
	}

	/* read and clear statistic registers (STA_CSR0 to STA_CSR10) */
	ural_cfg_read_multi(sc, RAL_STA_CSR0, sc->sc_sta, sizeof(sc->sc_sta));

	if ((sc->sc_flags & URAL_FLAG_LL_READY) &&
	    (sc->sc_flags & URAL_FLAG_HL_READY)) {

	    /* count TX retry-fail as Tx errors */
	    ifp->if_oerrors += sc->sc_sta[9];

	    amrr->retrycnt =
	      sc->sc_sta[7] +	/* TX one-retry ok count */
	      sc->sc_sta[8] +	/* TX more-retry ok count */
	      sc->sc_sta[9];	/* TX retry-fail count */

	    amrr->txcnt =
	      amrr->retrycnt +
	      sc->sc_sta[6];	/* TX no-retry ok count */
	    
	    ural_ratectl(amrr, sc->sc_ic.ic_bss);
	}
	return;
}

/*-
 * Naive implementation of the Adaptive Multi Rate Retry algorithm:
 *     "IEEE 802.11 Rate Adaptation: A Practical Approach"
 *     Mathieu Lacage, Hossein Manshaei, Thierry Turletti
 *     INRIA Sophia - Projet Planete
 *     http://www-sop.inria.fr/rapports/sophia/RR-5208.html
 *
 * This algorithm is particularly well suited for ural since it does not
 * require per-frame retry statistics.  Note however that since h/w does
 * not provide per-frame stats, we can't do per-node rate adaptation and
 * thus automatic rate adaptation is only enabled in STA operating mode.
 */
#define is_success(amrr)	\
	((amrr)->retrycnt < ((amrr)->txcnt / 10))
#define is_failure(amrr)	\
	((amrr)->retrycnt > ((amrr)->txcnt / 3))
#define is_enough(amrr)		\
	((amrr)->txcnt > 10)
#define is_min_rate(ni)		\
	((ni)->ni_txrate == 0)
#define is_max_rate(ni)		\
	((ni)->ni_rates.rs_nrates <= 0) || \
	((ni)->ni_txrate == ((ni)->ni_rates.rs_nrates - 1))
#define increase_rate(ni)	\
	((ni)->ni_txrate++)
#define decrease_rate(ni)	\
	((ni)->ni_txrate--)
#define reset_cnt(amrr)		\
	do { (amrr)->txcnt = (amrr)->retrycnt = 0; } while (0)

static void
ural_ratectl(struct ural_amrr *amrr, struct ieee80211_node *ni)
{
	u_int8_t need_change = 0;

	if (is_success(amrr) && is_enough(amrr)) {
		amrr->success++;
		if ((amrr->success >= amrr->success_threshold) &&
		    (!is_max_rate(ni))) {
			amrr->recovery = 1;
			amrr->success = 0;
			increase_rate(ni);
			need_change = 1;
		} else {
			amrr->recovery = 0;
		}
	} else if (is_failure(amrr)) {
		amrr->success = 0;
		if (!is_min_rate(ni)) {
			if (amrr->recovery) {
				amrr->success_threshold *= 2;
				if (amrr->success_threshold >
				    URAL_AMRR_MAX_SUCCESS_THRESHOLD) {
					amrr->success_threshold =
					    URAL_AMRR_MAX_SUCCESS_THRESHOLD;
				}
			} else {
				amrr->success_threshold =
				    URAL_AMRR_MIN_SUCCESS_THRESHOLD;
			}
			decrease_rate(ni);
			need_change = 1;
		}
		amrr->recovery = 0;	/* original paper was incorrect */
	}

	if (is_enough(amrr) || need_change) {
		reset_cnt(amrr);
	}
	return;
}

