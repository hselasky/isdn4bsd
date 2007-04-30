/*-
 * Copyright (c) 2005-2007 Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2006 Niall O'Higgins <niallo@openbsd.org>
 * Copyright (c) 2007 Hans Petter Selasky <hselasky@freebsd.org>
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

/*
 * NOTE: all function names beginning like "rum_cfg_" can only
 * be called from within the config thread function !
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*-
 * Ralink Technology RT2501USB/RT2601USB chipset driver
 * http://www.ralinktech.com.tw/
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
#include <net80211/ieee80211_amrr.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#include <netinet/if_ether.h>

#define usbd_config_td_cc rum_config_copy
#define usbd_config_td_softc rum_softc

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

#include <dev/usb/if_rumreg.h>
#include <dev/usb/if_rumvar.h>
#include <dev/usb/if_rumfw.h>

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)	\
  do { if (rum_debug > (n)) {	     \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int rum_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, rum, CTLFLAG_RW, 0, "USB rum");
SYSCTL_INT(_hw_usb_rum, OID_AUTO, debug, CTLFLAG_RW, &rum_debug, 0,
	   "rum debug level");
#else
#define DPRINTF(...)
#endif

/* prototypes */

static device_probe_t rum_probe;
static device_attach_t rum_attach;
static device_detach_t rum_detach;

static usbd_callback_t rum_bulk_read_callback;
static usbd_callback_t rum_bulk_read_clear_stall_callback;
static usbd_callback_t rum_bulk_write_callback;
static usbd_callback_t rum_bulk_write_clear_stall_callback;

static usbd_config_td_command_t rum_cfg_first_time_setup;
static usbd_config_td_command_t rum_config_copy;
static usbd_config_td_command_t rum_cfg_select_band;
static usbd_config_td_command_t rum_cfg_set_chan;
static usbd_config_td_command_t rum_cfg_pre_set_run;
static usbd_config_td_command_t rum_cfg_set_run;
static usbd_config_td_command_t rum_cfg_enable_tsf_sync;
static usbd_config_td_command_t rum_cfg_disable_tsf_sync;
static usbd_config_td_command_t rum_cfg_enable_mrr;
static usbd_config_td_command_t rum_cfg_update_slot;
static usbd_config_td_command_t rum_cfg_select_antenna;
static usbd_config_td_command_t rum_cfg_set_txpreamble;
static usbd_config_td_command_t rum_cfg_set_basicrates;
static usbd_config_td_command_t rum_cfg_update_promisc;
static usbd_config_td_command_t rum_cfg_pre_init;
static usbd_config_td_command_t rum_cfg_init;
static usbd_config_td_command_t rum_cfg_pre_stop;
static usbd_config_td_command_t rum_cfg_stop;
static usbd_config_td_command_t rum_cfg_amrr_timeout;
static usbd_config_td_command_t rum_cfg_prepare_beacon;

static const char * rum_get_rf(uint32_t rev);
static int rum_ioctl_cb(struct ifnet *ifp, u_long cmd, caddr_t data);
static int rum_media_change_cb(struct ifnet *ifp);
static int rum_newstate_cb(struct ieee80211com *ic, enum ieee80211_state nstate, int arg);
static int rum_reset_cb(struct ifnet *ifp);
static uint16_t rum_ack_rate(const struct ieee80211com *ic, uint16_t rate);
static uint16_t rum_cfg_eeprom_read_2(struct rum_softc *sc, uint16_t addr);
static uint16_t rum_rxrate(const struct rum_rx_desc *desc);
static uint16_t rum_txtime(struct rum_softc *sc, uint16_t len, uint16_t rate, uint32_t flags);
static uint32_t rum_cfg_bbp_disbusy(struct rum_softc *sc);
static uint32_t rum_cfg_read(struct rum_softc *sc, uint16_t reg);
static uint8_t rum_cfg_bbp_init(struct rum_softc *sc);
static uint8_t rum_cfg_bbp_read(struct rum_softc *sc, uint8_t reg);
static uint8_t rum_plcp_signal(uint16_t rate);
static void rum_bulk_write_callback_sub(struct usbd_xfer *xfer, struct mbuf *m, struct ieee80211_node *ni, uint32_t flags, uint16_t rate);
static void rum_cfg_amrr_start(struct rum_softc *sc);
static void rum_cfg_bbp_write(struct rum_softc *sc, uint8_t reg, uint8_t val);
static void rum_cfg_do_request(struct rum_softc *sc, usb_device_request_t *req, void *data);
static void rum_cfg_eeprom_read(struct rum_softc *sc, uint16_t addr, void *buf, uint16_t len);
static void rum_cfg_load_microcode(struct rum_softc *sc, const uint8_t *ucode, uint16_t size);
static void rum_cfg_read_eeprom(struct rum_softc *sc);
static void rum_cfg_read_multi(struct rum_softc *sc, uint16_t reg, void *buf, uint16_t len);
static void rum_cfg_rf_write(struct rum_softc *sc, uint8_t reg, uint32_t val);
static void rum_cfg_set_bssid(struct rum_softc *sc, uint8_t *bssid);
static void rum_cfg_set_macaddr(struct rum_softc *sc, uint8_t *addr);
static void rum_cfg_write(struct rum_softc *sc, uint16_t reg, uint32_t val);
static void rum_cfg_write_multi(struct rum_softc *sc, uint16_t reg, void *buf, uint16_t len);
static void rum_end_of_commands(struct rum_softc *sc);
static void rum_init_cb(void *arg);
static void rum_setup_tx_desc(struct rum_softc *sc, uint32_t flags, uint16_t xflags, uint16_t len, uint16_t rate);
static void rum_start_cb(struct ifnet *ifp);
static void rum_watchdog(void *arg);

/* various supported device vendors/products */
static const struct usb_devno rum_devs[] = {
	{ USB_VENDOR_ABOCOM,            USB_PRODUCT_ABOCOM_HWU54DM },
	{ USB_VENDOR_ABOCOM,            USB_PRODUCT_ABOCOM_RT2573_2 },
	{ USB_VENDOR_ABOCOM,            USB_PRODUCT_ABOCOM_RT2573_3 },
	{ USB_VENDOR_ABOCOM,            USB_PRODUCT_ABOCOM_RT2573_4 },
	{ USB_VENDOR_ABOCOM,            USB_PRODUCT_ABOCOM_WUG2700 },
	{ USB_VENDOR_AMIT,              USB_PRODUCT_AMIT_CGWLUSB2GO },
	{ USB_VENDOR_ASUS,              USB_PRODUCT_ASUS_RT2573_1 },
	{ USB_VENDOR_ASUS,              USB_PRODUCT_ASUS_RT2573_2 },
	{ USB_VENDOR_BELKIN,            USB_PRODUCT_BELKIN_F5D7050A },
	{ USB_VENDOR_BELKIN,            USB_PRODUCT_BELKIN_F5D9050V3 },
	{ USB_VENDOR_CISCOLINKSYS,      USB_PRODUCT_CISCOLINKSYS_WUSB54GC },
	{ USB_VENDOR_CISCOLINKSYS,      USB_PRODUCT_CISCOLINKSYS_WUSB54GR },
	{ USB_VENDOR_CONCEPTRONIC2,     USB_PRODUCT_CONCEPTRONIC2_C54RU2 },
	{ USB_VENDOR_DICKSMITH,         USB_PRODUCT_DICKSMITH_CWD854F },
	{ USB_VENDOR_DICKSMITH,         USB_PRODUCT_DICKSMITH_RT2573 },
	{ USB_VENDOR_DLINK2,		USB_PRODUCT_DLINK2_DWLG122C1 },
	{ USB_VENDOR_DLINK2,            USB_PRODUCT_DLINK2_WUA1340 },
	{ USB_VENDOR_GIGABYTE,          USB_PRODUCT_GIGABYTE_GNWB01GS },
	{ USB_VENDOR_GIGABYTE,          USB_PRODUCT_GIGABYTE_GNWI05GS },
	{ USB_VENDOR_GIGASET,           USB_PRODUCT_GIGASET_RT2573 },
	{ USB_VENDOR_GOODWAY,           USB_PRODUCT_GOODWAY_RT2573 },
	{ USB_VENDOR_GUILLEMOT,         USB_PRODUCT_GUILLEMOT_HWGUSB254LB },
	{ USB_VENDOR_GUILLEMOT,         USB_PRODUCT_GUILLEMOT_HWGUSB254V2AP },
	{ USB_VENDOR_HUAWEI3COM,        USB_PRODUCT_HUAWEI3COM_WUB320G },
	{ USB_VENDOR_MELCO,             USB_PRODUCT_MELCO_G54HP },
	{ USB_VENDOR_MELCO,             USB_PRODUCT_MELCO_SG54HP },
	{ USB_VENDOR_MSI,               USB_PRODUCT_MSI_RT2573_1 },
	{ USB_VENDOR_MSI,               USB_PRODUCT_MSI_RT2573_2 },
	{ USB_VENDOR_MSI,               USB_PRODUCT_MSI_RT2573_3 },
	{ USB_VENDOR_MSI,               USB_PRODUCT_MSI_RT2573_4 },
	{ USB_VENDOR_NOVATECH,          USB_PRODUCT_NOVATECH_RT2573 },
	{ USB_VENDOR_PLANEX2,           USB_PRODUCT_PLANEX2_GWUS54HP },
	{ USB_VENDOR_PLANEX2,           USB_PRODUCT_PLANEX2_GWUS54MINI2 },
	{ USB_VENDOR_PLANEX2,           USB_PRODUCT_PLANEX2_GWUSMM },
	{ USB_VENDOR_QCOM,              USB_PRODUCT_QCOM_RT2573 },
	{ USB_VENDOR_QCOM,              USB_PRODUCT_QCOM_RT2573_2 },
	{ USB_VENDOR_RALINK,		USB_PRODUCT_RALINK_RT2573 },
	{ USB_VENDOR_RALINK,            USB_PRODUCT_RALINK_RT2573_2 },
	{ USB_VENDOR_RALINK,            USB_PRODUCT_RALINK_RT2671 },
	{ USB_VENDOR_SITECOMEU,         USB_PRODUCT_SITECOMEU_WL113R2 },
	{ USB_VENDOR_SITECOMEU,         USB_PRODUCT_SITECOMEU_WL172 },
	{ USB_VENDOR_SURECOM,           USB_PRODUCT_SURECOM_RT2573 }
};

static const struct {
	uint32_t	reg;
	uint32_t	val;
} rum_def_mac[] = {
	{ RT2573_TXRX_CSR0,  0x025fb032 },
	{ RT2573_TXRX_CSR1,  0x9eaa9eaf },
	{ RT2573_TXRX_CSR2,  0x8a8b8c8d }, 
	{ RT2573_TXRX_CSR3,  0x00858687 },
	{ RT2573_TXRX_CSR7,  0x2e31353b },
	{ RT2573_TXRX_CSR8,  0x2a2a2a2c },
	{ RT2573_TXRX_CSR15, 0x0000000f },
	{ RT2573_MAC_CSR6,   0x00000fff },
	{ RT2573_MAC_CSR8,   0x016c030a },
	{ RT2573_MAC_CSR10,  0x00000718 },
	{ RT2573_MAC_CSR12,  0x00000004 },
	{ RT2573_MAC_CSR13,  0x00007f00 },
	{ RT2573_SEC_CSR0,   0x00000000 },
	{ RT2573_SEC_CSR1,   0x00000000 },
	{ RT2573_SEC_CSR5,   0x00000000 },
	{ RT2573_PHY_CSR1,   0x000023b0 },
	{ RT2573_PHY_CSR5,   0x00040a06 },
	{ RT2573_PHY_CSR6,   0x00080606 },
	{ RT2573_PHY_CSR7,   0x00000408 },
	{ RT2573_AIFSN_CSR,  0x00002273 },
	{ RT2573_CWMIN_CSR,  0x00002344 },
	{ RT2573_CWMAX_CSR,  0x000034aa }
};

static const struct {
	uint8_t	reg;
	uint8_t	val;
} rum_def_bbp[] = {
	{   3, 0x80 },
	{  15, 0x30 },
	{  17, 0x20 },
	{  21, 0xc8 },
	{  22, 0x38 },
	{  23, 0x06 },
	{  24, 0xfe },
	{  25, 0x0a },
	{  26, 0x0d },
	{  32, 0x0b },
	{  34, 0x12 },
	{  37, 0x07 },
	{  39, 0xf8 },
	{  41, 0x60 },
	{  53, 0x10 },
	{  54, 0x18 },
	{  60, 0x10 },
	{  61, 0x04 },
	{  62, 0x04 },
	{  75, 0xfe },
	{  86, 0xfe },
	{  88, 0xfe },
	{  90, 0x0f },
	{  99, 0x00 },
	{ 102, 0x16 },
	{ 107, 0x04 }
};

struct rfprog {
	uint8_t		chan;
	uint32_t	r1, r2, r3, r4;
};

static const struct rfprog rum_rf5226[] = {
	{   1, 0x00b03, 0x001e1, 0x1a014, 0x30282 },
	{   2, 0x00b03, 0x001e1, 0x1a014, 0x30287 },
	{   3, 0x00b03, 0x001e2, 0x1a014, 0x30282 },
	{   4, 0x00b03, 0x001e2, 0x1a014, 0x30287 },
	{   5, 0x00b03, 0x001e3, 0x1a014, 0x30282 },
	{   6, 0x00b03, 0x001e3, 0x1a014, 0x30287 },
	{   7, 0x00b03, 0x001e4, 0x1a014, 0x30282 },
	{   8, 0x00b03, 0x001e4, 0x1a014, 0x30287 },
	{   9, 0x00b03, 0x001e5, 0x1a014, 0x30282 },
	{  10, 0x00b03, 0x001e5, 0x1a014, 0x30287 },
	{  11, 0x00b03, 0x001e6, 0x1a014, 0x30282 },
	{  12, 0x00b03, 0x001e6, 0x1a014, 0x30287 },
	{  13, 0x00b03, 0x001e7, 0x1a014, 0x30282 },
	{  14, 0x00b03, 0x001e8, 0x1a014, 0x30284 },

	{  34, 0x00b03, 0x20266, 0x36014, 0x30282 },
	{  38, 0x00b03, 0x20267, 0x36014, 0x30284 },
	{  42, 0x00b03, 0x20268, 0x36014, 0x30286 },
	{  46, 0x00b03, 0x20269, 0x36014, 0x30288 },

	{  36, 0x00b03, 0x00266, 0x26014, 0x30288 },
	{  40, 0x00b03, 0x00268, 0x26014, 0x30280 },
	{  44, 0x00b03, 0x00269, 0x26014, 0x30282 },
	{  48, 0x00b03, 0x0026a, 0x26014, 0x30284 },
	{  52, 0x00b03, 0x0026b, 0x26014, 0x30286 },
	{  56, 0x00b03, 0x0026c, 0x26014, 0x30288 },
	{  60, 0x00b03, 0x0026e, 0x26014, 0x30280 },
	{  64, 0x00b03, 0x0026f, 0x26014, 0x30282 },

	{ 100, 0x00b03, 0x0028a, 0x2e014, 0x30280 },
	{ 104, 0x00b03, 0x0028b, 0x2e014, 0x30282 },
	{ 108, 0x00b03, 0x0028c, 0x2e014, 0x30284 },
	{ 112, 0x00b03, 0x0028d, 0x2e014, 0x30286 },
	{ 116, 0x00b03, 0x0028e, 0x2e014, 0x30288 },
	{ 120, 0x00b03, 0x002a0, 0x2e014, 0x30280 },
	{ 124, 0x00b03, 0x002a1, 0x2e014, 0x30282 },
	{ 128, 0x00b03, 0x002a2, 0x2e014, 0x30284 },
	{ 132, 0x00b03, 0x002a3, 0x2e014, 0x30286 },
	{ 136, 0x00b03, 0x002a4, 0x2e014, 0x30288 },
	{ 140, 0x00b03, 0x002a6, 0x2e014, 0x30280 },

	{ 149, 0x00b03, 0x002a8, 0x2e014, 0x30287 },
	{ 153, 0x00b03, 0x002a9, 0x2e014, 0x30289 },
	{ 157, 0x00b03, 0x002ab, 0x2e014, 0x30281 },
	{ 161, 0x00b03, 0x002ac, 0x2e014, 0x30283 },
	{ 165, 0x00b03, 0x002ad, 0x2e014, 0x30285 }
};

static const struct rfprog rum_rf5225[] = {
	{   1, 0x00b33, 0x011e1, 0x1a014, 0x30282 },
	{   2, 0x00b33, 0x011e1, 0x1a014, 0x30287 },
	{   3, 0x00b33, 0x011e2, 0x1a014, 0x30282 },
	{   4, 0x00b33, 0x011e2, 0x1a014, 0x30287 },
	{   5, 0x00b33, 0x011e3, 0x1a014, 0x30282 },
	{   6, 0x00b33, 0x011e3, 0x1a014, 0x30287 },
	{   7, 0x00b33, 0x011e4, 0x1a014, 0x30282 },
	{   8, 0x00b33, 0x011e4, 0x1a014, 0x30287 },
	{   9, 0x00b33, 0x011e5, 0x1a014, 0x30282 },
	{  10, 0x00b33, 0x011e5, 0x1a014, 0x30287 },
	{  11, 0x00b33, 0x011e6, 0x1a014, 0x30282 },
	{  12, 0x00b33, 0x011e6, 0x1a014, 0x30287 },
	{  13, 0x00b33, 0x011e7, 0x1a014, 0x30282 },
	{  14, 0x00b33, 0x011e8, 0x1a014, 0x30284 },

	{  34, 0x00b33, 0x01266, 0x26014, 0x30282 },
	{  38, 0x00b33, 0x01267, 0x26014, 0x30284 },
	{  42, 0x00b33, 0x01268, 0x26014, 0x30286 },
	{  46, 0x00b33, 0x01269, 0x26014, 0x30288 },

	{  36, 0x00b33, 0x01266, 0x26014, 0x30288 },
	{  40, 0x00b33, 0x01268, 0x26014, 0x30280 },
	{  44, 0x00b33, 0x01269, 0x26014, 0x30282 },
	{  48, 0x00b33, 0x0126a, 0x26014, 0x30284 },
	{  52, 0x00b33, 0x0126b, 0x26014, 0x30286 },
	{  56, 0x00b33, 0x0126c, 0x26014, 0x30288 },
	{  60, 0x00b33, 0x0126e, 0x26014, 0x30280 },
	{  64, 0x00b33, 0x0126f, 0x26014, 0x30282 },

	{ 100, 0x00b33, 0x0128a, 0x2e014, 0x30280 },
	{ 104, 0x00b33, 0x0128b, 0x2e014, 0x30282 },
	{ 108, 0x00b33, 0x0128c, 0x2e014, 0x30284 },
	{ 112, 0x00b33, 0x0128d, 0x2e014, 0x30286 },
	{ 116, 0x00b33, 0x0128e, 0x2e014, 0x30288 },
	{ 120, 0x00b33, 0x012a0, 0x2e014, 0x30280 },
	{ 124, 0x00b33, 0x012a1, 0x2e014, 0x30282 },
	{ 128, 0x00b33, 0x012a2, 0x2e014, 0x30284 },
	{ 132, 0x00b33, 0x012a3, 0x2e014, 0x30286 },
	{ 136, 0x00b33, 0x012a4, 0x2e014, 0x30288 },
	{ 140, 0x00b33, 0x012a6, 0x2e014, 0x30280 },

	{ 149, 0x00b33, 0x012a8, 0x2e014, 0x30287 },
	{ 153, 0x00b33, 0x012a9, 0x2e014, 0x30289 },
	{ 157, 0x00b33, 0x012ab, 0x2e014, 0x30281 },
	{ 161, 0x00b33, 0x012ac, 0x2e014, 0x30283 },
	{ 165, 0x00b33, 0x012ad, 0x2e014, 0x30285 }
};

static const struct usbd_config rum_config[RUM_N_TRANSFER] = {
    [0] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = (MCLBYTES + RT2573_TX_DESC_SIZE + 8),
      .flags     = (USBD_USE_DMA|USBD_FORCE_SHORT_XFER),
      .callback  = &rum_bulk_write_callback,
      .timeout   = 5000, /* ms */
    },

    [1] = {
      .type      = UE_BULK,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = (MCLBYTES + RT2573_RX_DESC_SIZE),
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &rum_bulk_read_callback,
    },

    [2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &rum_bulk_write_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },

    [3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &rum_bulk_read_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static devclass_t rum_devclass;

static device_method_t rum_methods[] = {
    DEVMETHOD(device_probe, rum_probe),
    DEVMETHOD(device_attach, rum_attach),
    DEVMETHOD(device_detach, rum_detach),
    { 0, 0 }
};

static driver_t rum_driver = {
    .name    = "rum",
    .methods = rum_methods,
    .size    = sizeof(struct rum_softc),
};

DRIVER_MODULE(rum, uhub, rum_driver, rum_devclass, usbd_driver_load, 0);
MODULE_DEPEND(rum, usb, 1, 1, 1);
MODULE_DEPEND(rum, wlan, 1, 1, 1);
MODULE_DEPEND(rum, wlan_amrr, 1, 1, 1);

static int
rum_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->iface != NULL) {
	    return UMATCH_NONE;
	}

	return ((usb_lookup(rum_devs, uaa->vendor, uaa->product) != NULL) ?
		UMATCH_VENDOR_PRODUCT : UMATCH_NONE);
}

static int
rum_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct rum_softc *sc = device_get_softc(dev);
	int error;

	if (sc == NULL) {
	    return ENOMEM;
	}

	usbd_set_desc(dev, uaa->device);

	mtx_init(&sc->sc_mtx, "rum lock", MTX_NETWORK_LOCK,
		 MTX_DEF | MTX_RECURSE);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s", 
		 device_get_nameunit(dev));

	sc->sc_udev = uaa->device;
	sc->sc_unit = device_get_unit(dev);

	__callout_init_mtx(&(sc->sc_watchdog),
			   &(sc->sc_mtx), CALLOUT_RETURNUNLOCKED);

	error = usbd_set_config_no(uaa->device, RT2573_CONFIG_NO, 0);

	if (error) {
	    device_printf(dev, "could not set configuration "
			  "number, err=%s!\n", usbd_errstr(error));
	    goto detach;
	}

	error = usbd_transfer_setup(uaa->device, RT2573_IFACE_INDEX, 
				    sc->sc_xfer, rum_config, RUM_N_TRANSFER, 
				    sc, &(sc->sc_mtx));
	if (error) {
	    device_printf(dev, "could not allocate USB transfers, "
			  "err=%s\n", usbd_errstr(error)) ;
	    goto detach;
	}

	error = usbd_config_td_setup(&(sc->sc_config_td), sc, &(sc->sc_mtx),
				     &rum_end_of_commands,
				     sizeof(struct rum_config_copy), 24);
	if (error) {
	    device_printf(dev, "could not setup config "
			  "thread!\n");
	    goto detach;
	}

	mtx_lock(&(sc->sc_mtx));

	/* start setup */

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL, &rum_cfg_first_time_setup, 0, 0);

	/* start watchdog (will exit mutex) */

	rum_watchdog(sc);

	return 0; /* success */

 detach:
	rum_detach(dev);
	return ENXIO; /* failure */
}

static int
rum_detach(device_t dev)
{
	struct rum_softc *sc = device_get_softc(dev);
	struct ieee80211com *ic;
	struct ifnet *ifp;

	usbd_config_td_stop(&(sc->sc_config_td));

	mtx_lock(&(sc->sc_mtx));

	__callout_stop(&sc->sc_watchdog);

	rum_cfg_pre_stop(sc, NULL, 0);

	ic = &(sc->sc_ic);
	ifp = ic->ic_ifp;

	mtx_unlock(&(sc->sc_mtx));

	/* XXX make sure that all USB callbacks have exited
	 * before tearing down the network stack:
	 */
	usbd_transfer_unsetup(sc->sc_xfer, RUM_N_TRANSFER);

	/* get rid of any late children */
	bus_generic_detach(dev);

	if (ifp) {
	    bpfdetach(ifp);
	    ieee80211_ifdetach(ic);
	    if_free(ifp);
	}

	usbd_config_td_unsetup(&(sc->sc_config_td));

	__callout_drain(&(sc->sc_watchdog));

	mtx_destroy(&sc->sc_mtx);

	return 0;
}

static void
rum_cfg_do_request(struct rum_softc *sc, usb_device_request_t *req, 
		   void *data)
{
	uint16_t length;
	usbd_status err;

 repeat:

	if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto error;
	}

	err = usbd_do_request_flags_mtx
	  (sc->sc_udev, &(sc->sc_mtx), req, data, 0, NULL, 1000);

	if (err) {

	    DPRINTF(sc, 0, "device request failed, err=%s "
		   "(ignored)\n", usbd_errstr(err));

	    /* wait a little before next try */
	    if (usbd_config_td_sleep(&(sc->sc_config_td), hz/4)) {
		goto error;
	    }

	    /* try until we are detached */
	    goto repeat;

	error:
	    /* the device has been detached */
	    length = UGETW(req->wLength);

	    if ((req->bmRequestType & UT_READ) && length) {
	        bzero(data, length);
	    }
	}
	return;
}

static void
rum_cfg_eeprom_read(struct rum_softc *sc, uint16_t addr, void *buf, uint16_t len)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = RT2573_READ_EEPROM;
	USETW(req.wValue, 0);
	USETW(req.wIndex, addr);
	USETW(req.wLength, len);

	rum_cfg_do_request(sc, &req, buf);
	return;
}

static uint16_t
rum_cfg_eeprom_read_2(struct rum_softc *sc, uint16_t addr)
{
	uint16_t tmp;
	rum_cfg_eeprom_read(sc, addr, &tmp, sizeof(tmp));
	return le16toh(tmp);
}

static uint32_t
rum_cfg_read(struct rum_softc *sc, uint16_t reg)
{
	uint32_t val;
	rum_cfg_read_multi(sc, reg, &val, sizeof(val));
	return le32toh(val);
}

static void
rum_cfg_read_multi(struct rum_softc *sc, uint16_t reg, void *buf, uint16_t len)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = RT2573_READ_MULTI_MAC;
	USETW(req.wValue, 0);
	USETW(req.wIndex, reg);
	USETW(req.wLength, len);

	rum_cfg_do_request(sc, &req, buf);
	return;
}

static void
rum_cfg_write(struct rum_softc *sc, uint16_t reg, uint32_t val)
{
	uint32_t tmp = htole32(val);
	rum_cfg_write_multi(sc, reg, &tmp, sizeof(tmp));
	return;
}

static void
rum_cfg_write_multi(struct rum_softc *sc, uint16_t reg, void *buf, uint16_t len)
{
	usb_device_request_t req;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RT2573_WRITE_MULTI_MAC;
	USETW(req.wValue, 0);
	USETW(req.wIndex, reg);
	USETW(req.wLength, len);

	rum_cfg_do_request(sc, &req, buf);
	return;
}

static uint32_t
rum_cfg_bbp_disbusy(struct rum_softc *sc)
{
	uint32_t tmp;
	uint8_t to;

	for (to = 0; ; to++) {
	    if (to < 100) {
	        tmp = rum_cfg_read(sc, RT2573_PHY_CSR3);

		if ((tmp & RT2573_BBP_BUSY) == 0) {
		    return tmp;
		}

		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    break;
		}
	    } else {
		break;
	    }
	}
	DPRINTF(sc, 0, "could not disbusy BBP\n");
	return RT2573_BBP_BUSY; /* failure */
}

static void
rum_cfg_bbp_write(struct rum_softc *sc, uint8_t reg, uint8_t val)
{
	uint32_t tmp;

	if (rum_cfg_bbp_disbusy(sc) & RT2573_BBP_BUSY) {
	    return;
	}

	tmp = RT2573_BBP_BUSY | ((reg & 0x7f) << 8) | val;
	rum_cfg_write(sc, RT2573_PHY_CSR3, tmp);
	return;
}

static uint8_t
rum_cfg_bbp_read(struct rum_softc *sc, uint8_t reg)
{
	uint32_t val;

	if (rum_cfg_bbp_disbusy(sc) & RT2573_BBP_BUSY) {
	    return 0;
	}

	val = RT2573_BBP_BUSY | RT2573_BBP_READ | (reg << 8);
	rum_cfg_write(sc, RT2573_PHY_CSR3, val);

	val = rum_cfg_bbp_disbusy(sc);
	return (val & 0xff);
}

static void
rum_cfg_rf_write(struct rum_softc *sc, uint8_t reg, uint32_t val)
{
	uint32_t tmp;
	uint8_t to;

	reg &= 3;

	for (to = 0; ; to++) {
	    if (to < 100) {
	        tmp = rum_cfg_read(sc, RT2573_PHY_CSR4);
		if (!(tmp & RT2573_RF_BUSY)) {
		    break;
		}

		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    return;
		}

	    } else {
	        DPRINTF(sc, 0, "could not write to RF\n");
		return;
	    }
	}

	tmp = RT2573_RF_BUSY | RT2573_RF_20BIT | ((val & 0xfffff) << 2) | reg;
	rum_cfg_write(sc, RT2573_PHY_CSR4, tmp);

	DPRINTF(sc, 15, "RF R[%u] <- 0x%05x\n", reg, val & 0xfffff);
	return;
}

static void
rum_cfg_first_time_setup(struct rum_softc *sc,
			 struct rum_config_copy *cc, uint16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp;
	uint32_t tmp;
	uint16_t i;

	/* setup RX tap header */
	sc->sc_rxtap_len = sizeof(sc->sc_rxtap.h);
	sc->sc_rxtap.h.wr_ihdr.it_len = htole16(sc->sc_rxtap_len);
	sc->sc_rxtap.h.wr_ihdr.it_present = htole32(RT2573_RX_RADIOTAP_PRESENT);

	/* setup TX tap header */
	sc->sc_txtap_len = sizeof(sc->sc_txtap.h);
	sc->sc_txtap.h.wt_ihdr.it_len = htole16(sc->sc_txtap_len);
	sc->sc_txtap.h.wt_ihdr.it_present = htole32(RT2573_TX_RADIOTAP_PRESENT);

	/* setup AMRR */
	ieee80211_amrr_init(&sc->sc_amrr, ic, 1, 10);

	/* retrieve RT2573 rev. no */
	for (i = 0; i < 100; i++) {

	    tmp = rum_cfg_read(sc, RT2573_MAC_CSR0);
	    if (tmp != 0)  {
		break;
	    }

	    /* wait a little */
	    if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
	        /* device detached */
	        goto done;
	    }
	}

	if (tmp == 0) {
	    DPRINTF(sc, 0, "chip is maybe not ready\n");
	}

	/* retrieve MAC address and various other things from EEPROM */
	rum_cfg_read_eeprom(sc);

	printf("%s: MAC/BBP RT2573 (rev 0x%05x), RF %s\n",
	       sc->sc_name, tmp, rum_get_rf(sc->sc_rf_rev));

	rum_cfg_load_microcode(sc, rt2573_ucode, sizeof(rt2573_ucode));

	mtx_unlock(&(sc->sc_mtx));

	ifp = if_alloc(IFT_ETHER);

	mtx_lock(&(sc->sc_mtx));

	if (ifp == NULL) {
	    DPRINTF(sc, -1, "could not if_alloc()!\n");
	    goto done;
	}

	sc->sc_evilhack = ifp;
	sc->sc_ifp = ifp;

	ifp->if_softc = sc;
	if_initname(ifp, "rum", sc->sc_unit);
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_init = &rum_init_cb;
	ifp->if_ioctl = &rum_ioctl_cb;
	ifp->if_start = &rum_start_cb;
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

	if ((sc->sc_rf_rev == RT2573_RF_5225) || 
	    (sc->sc_rf_rev == RT2573_RF_5226)) {

		/* set supported .11a channels */
		for (i = 34; i <= 46; i += 4) {
			ic->ic_channels[i].ic_freq =
			    ieee80211_ieee2mhz(i, IEEE80211_CHAN_5GHZ);
			ic->ic_channels[i].ic_flags = IEEE80211_CHAN_A;
		}
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
		for (i = 149; i <= 165; i += 4) {
			ic->ic_channels[i].ic_freq =
			    ieee80211_ieee2mhz(i, IEEE80211_CHAN_5GHZ);
			ic->ic_channels[i].ic_flags = IEEE80211_CHAN_A;
		}
	}

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

	/* enable SW bmiss handling in sta mode */
#if (defined(IEEE80211_FEXT_SWBMISS) || (__FreeBSD_version >= 700022))
	ic->ic_flags_ext |= IEEE80211_FEXT_SWBMISS;
#endif

	/* overrides */
	sc->sc_newstate = ic->ic_newstate;
	ic->ic_newstate = &rum_newstate_cb;
#if 0
	ic->ic_raw_xmit = &rum_raw_xmit_cb;
#endif
	ic->ic_reset = &rum_reset_cb;

	mtx_unlock(&(sc->sc_mtx));

	ieee80211_media_init(ic, rum_media_change_cb, ieee80211_media_status);

	bpfattach2(ifp, DLT_IEEE802_11_RADIO,
		   sizeof(struct ieee80211_frame) + 64, &sc->sc_drvbpf);

	if (bootverbose) {
	    ieee80211_announce(ic);
	}

	mtx_lock(&(sc->sc_mtx));
 done:
	return;
}

static void
rum_end_of_commands(struct rum_softc *sc)
{
	sc->sc_flags &= ~RUM_FLAG_WAIT_COMMAND;

	if ((sc->sc_flags & RUM_FLAG_LL_READY) &&
	    (sc->sc_flags & RUM_FLAG_HL_READY)) {
	    /* start write transfer, if not started */
	    usbd_transfer_start(sc->sc_xfer[0]);
	}
	return;
}

static void
rum_config_copy(struct rum_softc *sc, 
		struct rum_config_copy *cc, uint16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ieee80211_channel *c = ic->ic_curchan;
	struct ifnet *ifp = ic->ic_ifp;

	bzero(cc, sizeof(*cc));

	if (c) {
	    cc->ic_curchan.chan_to_ieee = ieee80211_chan2ieee(ic, c);
	    if (c != IEEE80211_CHAN_ANYC) {
	        cc->ic_curchan.chan_is_2ghz = IEEE80211_IS_CHAN_2GHZ(c) ? 1 : 0;
		cc->ic_curchan.chan_is_5ghz = IEEE80211_IS_CHAN_5GHZ(c) ? 1 : 0;
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

	sc->sc_flags |= RUM_FLAG_WAIT_COMMAND;
	return;
}

static const char *
rum_get_rf(uint32_t rev)
{
	switch (rev) {
	case RT2573_RF_2527:	return "RT2527 (MIMO XR)";
	case RT2573_RF_2528:	return "RT2528";
	case RT2573_RF_5225:	return "RT5225 (MIMO XR)";
	case RT2573_RF_5226:	return "RT5226";
	default:		return "unknown";
	}
}

/* quickly determine if a given rate is CCK or OFDM */
#define RUM_RATE_IS_OFDM(rate)	(((rate) >= 12) && ((rate) != 22))

#define RUM_ACK_SIZE	14	/* 10 + 4(FCS) */
#define RUM_CTS_SIZE	14	/* 10 + 4(FCS) */

static uint16_t
rum_rxrate(const struct rum_rx_desc *desc)
{
	if (le32toh(desc->flags) & RT2573_RX_OFDM) {
		/* reverse function of "rum_plcp_signal()" */
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
	return 2;	/* should not get here */
}

static void
rum_bulk_read_callback(struct usbd_xfer *xfer)
{
	struct rum_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = ic->ic_ifp;
	struct ieee80211_node *ni;
	struct mbuf *m = NULL;
	uint32_t flags;
	uint32_t max_len;
	uint8_t rssi;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= RUM_FLAG_READ_STALL;
	    usbd_transfer_start(sc->sc_xfer[3]);
	}
	return;

 tr_transferred:

	DPRINTF(sc, 14, "rx done, actlen=%d\n", xfer->actlen);

	if (xfer->actlen < (RT2573_RX_DESC_SIZE + IEEE80211_MIN_LEN)) {
	    DPRINTF(sc, 0, "too short transfer, "
		    "%d bytes\n", xfer->actlen);
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	usbd_copy_out(&(xfer->buf_data), 0,
		      &(sc->sc_rx_desc), RT2573_RX_DESC_SIZE);

	flags = le32toh(sc->sc_rx_desc.flags);

	if (flags & RT2573_RX_CRC_ERROR) {
	    /*
	     * This should not happen since we did not 
	     * request to receive those frames when we 
	     * filled RAL_TXRX_CSR2:
	     */
	    DPRINTF(sc, 5, "PHY or CRC error\n");
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	m = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR);

	if (m == NULL) {
	    DPRINTF(sc, 0, "could not allocate mbuf\n");
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	max_len = (xfer->actlen - RT2573_RX_DESC_SIZE);

	usbd_copy_out(&(xfer->buf_data), RT2573_RX_DESC_SIZE,
		      m->m_data, max_len);

	/* finalize mbuf */
	m->m_pkthdr.rcvif = ifp;
	m->m_pkthdr.len = m->m_len = (flags >> 16) & 0xfff;

	if (m->m_len > max_len) {
	    DPRINTF(sc, 0, "invalid length in RX "
		    "descriptor, %u bytes, received %u bytes\n",
		    m->m_len, max_len);
	    ifp->if_ierrors++;
	    goto tr_setup;
	}

	rssi = sc->sc_rx_desc.rssi;

	DPRINTF(sc, 0, "real length=%d bytes, rssi=%d\n", m->m_len, rssi);

	if (bpf_peers_present(sc->sc_drvbpf)) {
	    struct rum_rx_radiotap_header *tap = &(sc->sc_rxtap.h);

	    tap->wr_flags = IEEE80211_RADIOTAP_F_FCS;
	    tap->wr_rate = rum_rxrate(&sc->sc_rx_desc);
	    tap->wr_chan_freq = htole16(ic->ic_curchan->ic_freq);
	    tap->wr_chan_flags = htole16(ic->ic_curchan->ic_flags);
	    tap->wr_antenna = sc->sc_rx_ant;
	    tap->wr_antsignal = rssi;

	    bpf_mtap2(sc->sc_drvbpf, tap, sc->sc_rxtap_len, m);
	}

	ni = ieee80211_find_rxnode(ic, (void *)(m->m_data));

	mtx_unlock(&(sc->sc_mtx));

	/* XXX it is possibly not safe 
	 * to do the following unlocked:
	 * --hps
	 */

	/* send the frame to the 802.11 layer */
	ieee80211_input(ic, m, ni, rssi, 0);

	mtx_lock(&(sc->sc_mtx));

	/* node is no longer needed */
	ieee80211_free_node(ni);

	m = NULL;

 tr_setup:
	if (m) {
	    m_freem(m);
	}

	if (sc->sc_flags & RUM_FLAG_READ_STALL) {
	    usbd_transfer_start(sc->sc_xfer[3]);
	    return;
	}

	usbd_start_hardware(xfer);
	return;
}

static void
rum_bulk_read_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct rum_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[1];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~RUM_FLAG_READ_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~RUM_FLAG_READ_STALL;
	DPRINTF(sc, -1, "bulk read pipe stopped\n");
	return;
}

/*
 * Return the expected ack rate for a frame transmitted at rate "rate".
 */
static uint16_t
rum_ack_rate(const struct ieee80211com *ic, uint16_t rate)
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

/*
 * Compute the duration (in us) needed to transmit "len" bytes at rate "rate".
 * The function automatically determines the operating mode depending on the
 * given rate. "flags" indicates whether short preamble is in use or not.
 */
static uint16_t
rum_txtime(struct rum_softc *sc, uint16_t len, uint16_t rate, uint32_t flags)
{
	uint16_t txtime;

	if (rate < 2) {
	    DPRINTF(sc, 0, "rate < 2!\n");

	    /* avoid division by zero */
	    rate = 2;
	}

	if (RUM_RATE_IS_OFDM(rate)) {
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

static uint8_t
rum_plcp_signal(uint16_t rate)
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
rum_setup_tx_desc(struct rum_softc *sc, uint32_t flags, uint16_t xflags,
		  uint16_t len, uint16_t rate)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	uint16_t plcp_length;
	uint8_t remainder;

	if (rate < 2) {
	    DPRINTF(sc, 0, "rate < 2!\n");

	    /* avoid division by zero */
	    rate = 2;
	}

	flags |= RT2573_TX_VALID;
	flags |= (len << 16);

	sc->sc_tx_desc.flags = htole32(flags);
	sc->sc_tx_desc.xflags = htole16(xflags);

	sc->sc_tx_desc.wme = htole16(RT2573_QID(0) | RT2573_AIFSN(2) | 
	    RT2573_LOGCWMIN(4) | RT2573_LOGCWMAX(10));

	/* setup PLCP fields */
	sc->sc_tx_desc.plcp_signal  = rum_plcp_signal(rate);
	sc->sc_tx_desc.plcp_service = 4;

	len += IEEE80211_CRC_LEN;

	if (RUM_RATE_IS_OFDM(rate)) {
		sc->sc_tx_desc.flags |= htole32(RT2573_TX_OFDM);

		plcp_length = (len & 0xfff);
		sc->sc_tx_desc.plcp_length_hi = plcp_length >> 6;
		sc->sc_tx_desc.plcp_length_lo = plcp_length & 0x3f;
	} else {
		plcp_length = ((16 * len) + rate - 1) / rate;
		if (rate == 22) {
			remainder = (16 * len) % 22;
			if ((remainder != 0) && (remainder < 7)) {
				sc->sc_tx_desc.plcp_service |= 
				  RT2573_PLCP_LENGEXT;
			}
		}
		sc->sc_tx_desc.plcp_length_hi = plcp_length >> 8;
		sc->sc_tx_desc.plcp_length_lo = plcp_length & 0xff;

		if ((rate != 2) && (ic->ic_flags & IEEE80211_F_SHPREAMBLE)) {
			sc->sc_tx_desc.plcp_signal |= 0x08;
		}
	}
	return;
}

static void
rum_bulk_write_callback_sub(struct usbd_xfer *xfer, struct mbuf *m,
			    struct ieee80211_node *ni, uint32_t flags,
			    uint16_t rate)
{
	struct rum_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);
	uint8_t align;

	if (m->m_pkthdr.len > MCLBYTES) {
	    DPRINTF(sc, 0, "data overflow, %u bytes\n",
		    m->m_pkthdr.len);
	    m->m_pkthdr.len = MCLBYTES;
	}

	if (bpf_peers_present(sc->sc_drvbpf)) {
	    struct rum_tx_radiotap_header *tap = &(sc->sc_txtap.h);

	    tap->wt_flags = 0;
	    tap->wt_rate = rate;
	    tap->wt_chan_freq = htole16(ic->ic_curchan->ic_freq);
	    tap->wt_chan_flags = htole16(ic->ic_curchan->ic_flags);
	    tap->wt_antenna = sc->sc_tx_ant;

	    bpf_mtap2(sc->sc_drvbpf, tap, sc->sc_txtap_len, m);
	}

	rum_setup_tx_desc(sc, flags, 0, m->m_pkthdr.len, rate);

	usbd_copy_in(&(xfer->buf_data), 0, &(sc->sc_tx_desc), 
		     RT2573_TX_DESC_SIZE);

	usbd_m_copy_in(&(xfer->buf_data), RT2573_TX_DESC_SIZE,
		       m, 0, m->m_pkthdr.len);

	/* compute transfer length */
	xfer->length = (RT2573_TX_DESC_SIZE + m->m_pkthdr.len);

	/* make transfer length 32-bit aligned */
	if (xfer->length & 3) {
	    align = (-(xfer->length)) & 3;
	    /* zero the extra byte(s) */
	    usbd_bzero(&(xfer->buf_data), xfer->length, align);
	    xfer->length += align;
	}

	/* check if we need to add four extra bytes */
	if ((xfer->length % 64) == 0) {
	    /* zero the extra bytes */
	    usbd_bzero(&(xfer->buf_data), xfer->length, 4);
	    xfer->length += 4;
	}

	DPRINTF(sc, 10, "sending frame len=%u rate=%u xferlen=%u\n",
		m->m_pkthdr.len, rate, xfer->length);

	m_freem(m);

	if (ni) {
	    ieee80211_free_node(ni);
	}

	usbd_start_hardware(xfer);
	return;
}

static void
rum_bulk_write_callback(struct usbd_xfer *xfer)
{
	struct rum_softc *sc = xfer->priv_sc;
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = sc->sc_ic.ic_ifp;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni = NULL;
	struct ieee80211_key *k;
	struct ether_header *eh;
	struct mbuf *m = NULL;
	uint32_t flags;
	uint16_t dur;
	uint16_t rate;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(sc, 10, "transfer error, %s\n",
		 usbd_errstr(xfer->error));

	if (xfer->error != USBD_CANCELLED) {
	    /* try to clear stall first */
	    sc->sc_flags |= RUM_FLAG_WRITE_STALL;
	    usbd_transfer_start(sc->sc_xfer[2]);
	}

	ifp->if_oerrors++;
	return;


 tr_transferred:
	DPRINTF(sc, 10, "transfer complete\n");

	ifp->if_opackets++;

 tr_setup:
	if (sc->sc_flags & RUM_FLAG_WRITE_STALL) {
	    usbd_transfer_start(sc->sc_xfer[2]);
	    goto done;
	}

	if (sc->sc_flags & RUM_FLAG_WAIT_COMMAND) {
	    /* don't send anything while a command
	     * is pending !
	     */
	    goto done;
	}

	flags = 0;

	IF_DEQUEUE(&(ic->ic_mgtq), m);

	if (m) {

	    ni = (void *)(m->m_pkthdr.rcvif);
	    m->m_pkthdr.rcvif = NULL;

	    if (bpf_peers_present(ic->ic_rawbpf)) {
	        bpf_mtap(ic->ic_rawbpf, m);
	    }

	    rate = (IEEE80211_IS_CHAN_5GHZ(ic->ic_curchan) ? 12 : 2);

	    wh = mtod(m, struct ieee80211_frame *);

	    if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {

	        flags |= RT2573_TX_NEED_ACK;

		dur = rum_txtime(sc, RUM_ACK_SIZE, rum_ack_rate(ic, rate),
				 ic->ic_flags) + sc->sc_sifs;
		*(uint16_t *)(wh->i_dur) = htole16(dur);

		/* tell hardware to add timestamp for probe responses */
		if ((wh->i_fc[0] &
		     (IEEE80211_FC0_TYPE_MASK|
		      IEEE80211_FC0_SUBTYPE_MASK)) == 
		    (IEEE80211_FC0_TYPE_MGT|
		     IEEE80211_FC0_SUBTYPE_PROBE_RESP)) {
			flags |= RT2573_TX_TIMESTAMP;
		}
	    }

	    rum_bulk_write_callback_sub(xfer, m, ni, flags, rate);
	    goto done;
	}

	if (ic->ic_state != IEEE80211_S_RUN) {
	    goto done;
	}

	IFQ_DRV_DEQUEUE(&ifp->if_snd, m);

	if (m) {

	    if (m->m_len < sizeof(struct ether_header)) {
	        m = m_pullup(m, sizeof(struct ether_header));

		if (m == NULL) {
		    goto error;
		}
	    }

	    eh = mtod(m, struct ether_header *);
	    ni = ieee80211_find_txnode(ic, eh->ether_dhost);
	    if (ni == NULL) {
	        goto error;
	    }

	    BPF_MTAP(ifp, m);

	    m = ieee80211_encap(ic, m, ni);

	    if (m == NULL) {
	        goto error;
	    }

	    if (bpf_peers_present(ic->ic_rawbpf)) {
	        bpf_mtap(ic->ic_rawbpf, m);
	    }

	    wh = mtod(m, struct ieee80211_frame *);

	    if (ic->ic_fixed_rate != IEEE80211_FIXED_RATE_NONE)
	        rate = ic->ic_bss->ni_rates.rs_rates[ic->ic_fixed_rate];
	    else
	        rate = ni->ni_rates.rs_rates[ni->ni_txrate];

	    rate &= IEEE80211_RATE_VAL;

	    if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
		k = ieee80211_crypto_encap(ic, ni, m);
		if (k == NULL) {
		    goto error;
		}

		/* packet header may have moved, reset our local pointer */
		wh = mtod(m, struct ieee80211_frame *);
	    }

	    if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {
	        flags |= (RT2573_TX_NEED_ACK|
			  RT2573_TX_MORE_FRAG);

		dur = rum_txtime(sc, RUM_ACK_SIZE, rum_ack_rate(ic,rate),
				 ic->ic_flags) + sc->sc_sifs;

		*(uint16_t *)(wh->i_dur) = htole16(dur);
	    }

	    rum_bulk_write_callback_sub(xfer, m, ni, flags, rate);
	    goto done;
	}
 done:
	return;

 error:
	if (m) {
	    m_freem(m);
	    m = NULL;
	}

	if (ni) {
	    ieee80211_free_node(ni);
	    ni = NULL;
	}

	ifp->if_oerrors++;

	goto tr_setup;
}

static void
rum_bulk_write_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct rum_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, xfer_other);
	return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, xfer_other);

	sc->sc_flags &= ~RUM_FLAG_WRITE_STALL;
	usbd_transfer_start(xfer_other);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~RUM_FLAG_WRITE_STALL;
	DPRINTF(sc, -1, "bulk write pipe stopped\n");
	return;
}

static void
rum_watchdog(void *arg)
{
	struct rum_softc *sc = arg;
	struct ieee80211com *ic = &(sc->sc_ic);

	mtx_assert(&(sc->sc_mtx), MA_OWNED);

	if ((sc->sc_amrr_timer) &&
	    (--sc->sc_amrr_timer == 0)) {

	    /* restart timeout */
	    sc->sc_amrr_timer = (1*8);

	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), NULL,
	       &rum_cfg_amrr_timeout, 0, 0);
	}

	if ((sc->sc_if_timer) &&
	    (--(sc->sc_if_timer) == 0)) {

	    DPRINTF(sc, 0, "watchdog timeout\n");

	    /* restart timer */
	    sc->sc_if_timer = (1*8);

	    ieee80211_watchdog(ic);
	}

	if ((sc->sc_scan_timer) &&
	    (--(sc->sc_scan_timer) == 0)) {
	    ieee80211_next_scan(ic);
	}

	__callout_reset(&(sc->sc_watchdog), 
			hz / 8, &rum_watchdog, sc);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
rum_init_cb(void *arg)
{
	struct rum_softc *sc = arg;

	mtx_lock(&(sc->sc_mtx));
	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &rum_cfg_pre_init,
	   &rum_cfg_init, 0, 0);
	mtx_unlock(&(sc->sc_mtx));

	return;
}

static int
rum_ioctl_cb(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct rum_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &(sc->sc_ic);
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	switch (cmd) {
	case SIOCSIFFLAGS:

	    if (ifp->if_flags & IFF_UP) {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &rum_config_copy,
		       &rum_cfg_update_promisc, 0, 0);
		} else {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &rum_cfg_pre_init,
		       &rum_cfg_init, 0, 0); 
		}
	    } else {
	        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
		    usbd_config_td_queue_command
		      (&(sc->sc_config_td), &rum_cfg_pre_stop,
		       &rum_cfg_stop, 0, 0);
		}
	    }
	    break;

	default:
		/* XXX it is possibly not safe 
		 * to do the following unlocked:
		 * --hps
		 */
		mtx_unlock(&(sc->sc_mtx));
		error = ieee80211_ioctl(ic, cmd, data);
		mtx_lock(&(sc->sc_mtx));
	}

	if (error == ENETRESET) {
	    if ((ifp->if_flags & IFF_UP) &&
		(ifp->if_drv_flags & IFF_DRV_RUNNING) &&
		(ic->ic_roaming != IEEE80211_ROAMING_MANUAL)) {
	        usbd_config_td_queue_command
		  (&(sc->sc_config_td), &rum_cfg_pre_init,
		   &rum_cfg_init, 0, 0);
	    }
	    error = 0;
	}

	mtx_unlock(&(sc->sc_mtx));

	return error;
}

static void
rum_start_cb(struct ifnet *ifp)
{
	struct rum_softc *sc = ifp->if_softc;

	mtx_lock(&(sc->sc_mtx));

	if ((sc->sc_flags & RUM_FLAG_LL_READY) &&
	    (sc->sc_flags & RUM_FLAG_HL_READY)) {
	    /* start write transfer, if not started */
	    usbd_transfer_start(sc->sc_xfer[0]);
	}

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static int
rum_media_change_cb(struct ifnet *ifp)
{
	struct rum_softc *sc = ifp->if_softc;
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
	      (&(sc->sc_config_td), &rum_cfg_pre_init, 
	       &rum_cfg_init, 0, 0);
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
rum_reset_cb(struct ifnet *ifp)
{
	struct rum_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = &(sc->sc_ic);
	int error = 0;

	mtx_lock(&(sc->sc_mtx));

	if (ic->ic_opmode != IEEE80211_M_MONITOR) {
	    error = ENETRESET;
	    goto done;
	}

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), &rum_config_copy, 
	   &rum_cfg_set_chan, 0, 0);

 done:
	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

static int
rum_newstate_cb(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	struct rum_softc *sc = ic->ic_ifp->if_softc;
	enum ieee80211_state ostate = sc->sc_ic.ic_state;

	mtx_lock(&(sc->sc_mtx));

	DPRINTF(sc, 0, "setting new state: %d\n", nstate);

	/* force data to wait */
	sc->sc_flags |= RUM_FLAG_WAIT_COMMAND;

	/* set new state first! */
	(sc->sc_newstate)(ic, nstate, arg);

	/* stop timers */

	sc->sc_amrr_timer = 0;
	sc->sc_scan_timer = 0;
	sc->sc_if_timer = 8;

	/* set new state */

	switch (nstate) {
	case IEEE80211_S_INIT:
	    if (ostate == IEEE80211_S_RUN) {
	      usbd_config_td_queue_command
		(&(sc->sc_config_td), &rum_config_copy,
		 &rum_cfg_disable_tsf_sync, 0, 0);
	    }
	    sc->sc_if_timer = 0;
	    break;

	case IEEE80211_S_SCAN:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &rum_config_copy, 
	       &rum_cfg_set_chan, 0, 0);
	    sc->sc_scan_timer = 3;
	    break;

	case IEEE80211_S_AUTH:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &rum_config_copy, 
	       &rum_cfg_set_chan, 0, 0);
	    break;

	case IEEE80211_S_ASSOC:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &rum_config_copy, 
	       &rum_cfg_set_chan, 0, 0);
	    break;

	case IEEE80211_S_RUN:
	    usbd_config_td_queue_command
	      (&(sc->sc_config_td), &rum_cfg_pre_set_run,
	       &rum_cfg_set_run, 0, 0);
	    break;
	}

	mtx_unlock(&(sc->sc_mtx));

	return 0;
}

/*
 * Reprogram MAC/BBP to switch to a new band. Values taken from the reference
 * driver.
 */
static void
rum_cfg_select_band(struct rum_softc *sc, 
		    struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;
	uint8_t bbp17, bbp35, bbp96, bbp97, bbp98, bbp104;

	/* update all BBP registers that depend on the band */
	bbp17 = 0x20; bbp96 = 0x48; bbp104 = 0x2c;
	bbp35 = 0x50; bbp97 = 0x48; bbp98  = 0x48;

	if (cc->ic_curchan.chan_is_5ghz) {
		bbp17 += 0x08; bbp96 += 0x10; bbp104 += 0x0c;
		bbp35 += 0x10; bbp97 += 0x10; bbp98  += 0x10;
	}

	if ((cc->ic_curchan.chan_is_2ghz && sc->sc_ext_2ghz_lna) ||
	    (cc->ic_curchan.chan_is_5ghz && sc->sc_ext_5ghz_lna)) {
		bbp17 += 0x10; bbp96 += 0x10; bbp104 += 0x10;
	}

	sc->sc_bbp17 = bbp17;
	rum_cfg_bbp_write(sc,  17, bbp17);
	rum_cfg_bbp_write(sc,  96, bbp96);
	rum_cfg_bbp_write(sc, 104, bbp104);

	if ((cc->ic_curchan.chan_is_2ghz && sc->sc_ext_2ghz_lna) ||
	    (cc->ic_curchan.chan_is_5ghz && sc->sc_ext_5ghz_lna)) {
		rum_cfg_bbp_write(sc, 75, 0x80);
		rum_cfg_bbp_write(sc, 86, 0x80);
		rum_cfg_bbp_write(sc, 88, 0x80);
	}

	rum_cfg_bbp_write(sc, 35, bbp35);
	rum_cfg_bbp_write(sc, 97, bbp97);
	rum_cfg_bbp_write(sc, 98, bbp98);

	tmp = rum_cfg_read(sc, RT2573_PHY_CSR0);
	tmp &= ~(RT2573_PA_PE_2GHZ | RT2573_PA_PE_5GHZ);
	if (cc->ic_curchan.chan_is_2ghz)
		tmp |= RT2573_PA_PE_2GHZ;
	else
		tmp |= RT2573_PA_PE_5GHZ;
	rum_cfg_write(sc, RT2573_PHY_CSR0, tmp);

	/* 802.11a uses a 16 microseconds short interframe space */
	sc->sc_sifs = cc->ic_curchan.chan_is_5ghz ? 16 : 10;

	return;
}

static void
rum_cfg_set_chan(struct rum_softc *sc,
		 struct rum_config_copy *cc, uint16_t refcount)
{
	enum { N_RF5225 = (sizeof(rum_rf5225)/sizeof(rum_rf5225[0])) };
	const struct rfprog *rfprog;
	uint32_t chan;
	uint16_t i;
	uint8_t bbp3;
	uint8_t bbp94 = RT2573_BBPR94_DEFAULT;
	int8_t power;

	chan = cc->ic_curchan.chan_to_ieee;

	if ((chan == 0) ||
	    (chan == IEEE80211_CHAN_ANY)) {
	    /* nothing to do */
	    return;
	}

	if (chan == sc->sc_last_chan) {
		return;
	}

	sc->sc_last_chan = chan;

	/* select the appropriate RF settings based on what EEPROM says */
	rfprog = ((sc->sc_rf_rev == RT2573_RF_5225) ||
		  (sc->sc_rf_rev == RT2573_RF_2527)) ? rum_rf5225 : rum_rf5226;

	/* find the settings for this channel */
	for (i = 0; ; i++) {
		if (i == (N_RF5225-1)) break;
		if (rfprog[i].chan == chan) break;
	}

	DPRINTF(sc, 0, "chan=%d, i=%d\n", chan, i);

	power = sc->sc_txpow[i];
	if (power < 0) {
		bbp94 += power;
		power = 0;
	} else if (power > 31) {
		bbp94 += power - 31;
		power = 31;
	}

	/*
	 * If we are switching from the 2GHz band to the 5GHz band or
	 * vice-versa, BBP registers need to be reprogrammed.
	 */
	rum_cfg_select_band(sc, cc, 0);
	rum_cfg_select_antenna(sc, cc, 0);

	rum_cfg_rf_write(sc, RT2573_RF1, rfprog[i].r1);
	rum_cfg_rf_write(sc, RT2573_RF2, rfprog[i].r2);
	rum_cfg_rf_write(sc, RT2573_RF3, rfprog[i].r3 | (power << 7));
	rum_cfg_rf_write(sc, RT2573_RF4, rfprog[i].r4 | (sc->sc_rffreq << 10));

	rum_cfg_rf_write(sc, RT2573_RF1, rfprog[i].r1);
	rum_cfg_rf_write(sc, RT2573_RF2, rfprog[i].r2);
	rum_cfg_rf_write(sc, RT2573_RF3, rfprog[i].r3 | (power << 7) | 1);
	rum_cfg_rf_write(sc, RT2573_RF4, rfprog[i].r4 | (sc->sc_rffreq << 10));

	rum_cfg_rf_write(sc, RT2573_RF1, rfprog[i].r1);
	rum_cfg_rf_write(sc, RT2573_RF2, rfprog[i].r2);
	rum_cfg_rf_write(sc, RT2573_RF3, rfprog[i].r3 | (power << 7));
	rum_cfg_rf_write(sc, RT2573_RF4, rfprog[i].r4 | (sc->sc_rffreq << 10));

	if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		return;
	}

	/* enable smart mode for MIMO-capable RFs */
	bbp3 = rum_cfg_bbp_read(sc, 3);

	if ((sc->sc_rf_rev == RT2573_RF_5225) ||
	    (sc->sc_rf_rev == RT2573_RF_2527))
	  	bbp3 &= ~RT2573_SMART_MODE;
	else
		bbp3 |= RT2573_SMART_MODE;

	rum_cfg_bbp_write(sc, 3, bbp3);

	rum_cfg_bbp_write(sc, 94, bbp94);

	if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		return;
	}

	return;
}

static void
rum_cfg_pre_set_run(struct rum_softc *sc, 
		    struct rum_config_copy *cc, uint16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);

	/* immediate configuration */

	rum_config_copy(sc, cc, 0);

	/* enable automatic rate adaptation in STA mode */
	if ((ic->ic_opmode == IEEE80211_M_STA) &&
	    (ic->ic_fixed_rate == IEEE80211_FIXED_RATE_NONE)) {
		rum_cfg_amrr_start(sc);
	}
	return;
}

static void
rum_cfg_set_run(struct rum_softc *sc, 
		struct rum_config_copy *cc, uint16_t refcount)
{
	rum_cfg_set_chan(sc, cc, 0);

	if (cc->ic_opmode != IEEE80211_M_MONITOR) {
	    rum_cfg_update_slot(sc, cc, 0);
	    rum_cfg_enable_mrr(sc, cc, 0);
	    rum_cfg_set_txpreamble(sc, cc, 0);
	    rum_cfg_set_basicrates(sc, cc, 0);
	    rum_cfg_set_bssid(sc, cc->ic_bss.ni_bssid);
	}

	if ((cc->ic_opmode == IEEE80211_M_HOSTAP) ||
	    (cc->ic_opmode == IEEE80211_M_IBSS)) {
	    rum_cfg_prepare_beacon(sc, cc, 0);
	}

	if (cc->ic_opmode != IEEE80211_M_MONITOR) {
	    rum_cfg_enable_tsf_sync(sc, cc, 0);
	}
	return;
}

static void
rum_cfg_enable_tsf_sync(struct rum_softc *sc,
			struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;

	if (cc->ic_opmode != IEEE80211_M_STA) {
		/*
		 * Change default 16ms TBTT adjustment to 8ms.
		 * Must be done before enabling beacon generation.
		 */
		rum_cfg_write(sc, RT2573_TXRX_CSR10, (1 << 12) | 8);
	}

	tmp = rum_cfg_read(sc, RT2573_TXRX_CSR9) & 0xff000000;

	/* set beacon interval (in 1/16ms unit) */
	tmp |= cc->ic_bss.ni_intval * 16;

	tmp |= RT2573_TSF_TICKING | RT2573_ENABLE_TBTT;
	if (cc->ic_opmode == IEEE80211_M_STA)
		tmp |= RT2573_TSF_MODE(1);
	else
		tmp |= RT2573_TSF_MODE(2) | RT2573_GENERATE_BEACON;

	rum_cfg_write(sc, RT2573_TXRX_CSR9, tmp);

	return;
}

static void
rum_cfg_disable_tsf_sync(struct rum_softc *sc,
			 struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;

	/* abort TSF synchronization */
	tmp = rum_cfg_read(sc, RT2573_TXRX_CSR9);
	rum_cfg_write(sc, RT2573_TXRX_CSR9, tmp & ~0x00ffffff);

	return;
}

/*
 * Enable multi-rate retries for frames sent at OFDM rates.
 * In 802.11b/g mode, allow fallback to CCK rates.
 */
static void
rum_cfg_enable_mrr(struct rum_softc *sc, 
		   struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;

	tmp = rum_cfg_read(sc, RT2573_TXRX_CSR4);

	if (cc->ic_curchan.chan_is_5ghz)
	    tmp &= ~RT2573_MRR_CCK_FALLBACK;
	else
	    tmp |= RT2573_MRR_CCK_FALLBACK;

	tmp |= RT2573_MRR_ENABLED;

	rum_cfg_write(sc, RT2573_TXRX_CSR4, tmp);

	return;
}

static void
rum_cfg_update_slot(struct rum_softc *sc,
		    struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;
	uint8_t slottime;

	slottime = (cc->ic_flags & IEEE80211_F_SHSLOT) ? 9 : 20;

	tmp = rum_cfg_read(sc, RT2573_MAC_CSR9);
	tmp = (tmp & ~0xff) | slottime;
	rum_cfg_write(sc, RT2573_MAC_CSR9, tmp);

	DPRINTF(sc, 0, "setting slot time to %u us\n", slottime);

	return;
}

static void
rum_cfg_set_txpreamble(struct rum_softc *sc,
		       struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;

	tmp = rum_cfg_read(sc, RT2573_TXRX_CSR4);

	if (cc->ic_flags & IEEE80211_F_SHPREAMBLE)
		tmp |= RT2573_SHORT_PREAMBLE;
	else
		tmp &= ~RT2573_SHORT_PREAMBLE;

	rum_cfg_write(sc, RT2573_TXRX_CSR4, tmp);

	return;
}

static void
rum_cfg_set_basicrates(struct rum_softc *sc,
		       struct rum_config_copy *cc, uint16_t refcount)
{
	/* update basic rate set */

	if (cc->ic_curmode == IEEE80211_MODE_11B) {
		/* 11b basic rates: 1, 2Mbps */
		rum_cfg_write(sc, RT2573_TXRX_CSR5, 0x3);
	} else if (cc->ic_bss.ni_chan.chan_is_5ghz) {
		/* 11a basic rates: 6, 12, 24Mbps */
		rum_cfg_write(sc, RT2573_TXRX_CSR5, 0x150);
	} else {
		/* 11b/g basic rates: 1, 2, 5.5, 11Mbps */
		rum_cfg_write(sc, RT2573_TXRX_CSR5, 0xf);
	}

	return;
}

static void
rum_cfg_set_bssid(struct rum_softc *sc, uint8_t *bssid)
{
	uint32_t tmp;

	tmp = bssid[0] | (bssid[1] << 8) | (bssid[2] << 16) | (bssid[3] << 24);
	rum_cfg_write(sc, RT2573_MAC_CSR4, tmp);

	tmp = (bssid[4]) | (bssid[5] << 8) | (RT2573_ONE_BSSID << 16);
	rum_cfg_write(sc, RT2573_MAC_CSR5, tmp);

	return;
}

static void
rum_cfg_set_macaddr(struct rum_softc *sc, uint8_t *addr)
{
	uint32_t tmp;

	tmp = addr[0] | (addr[1] << 8) | (addr[2] << 16) | (addr[3] << 24);
	rum_cfg_write(sc, RT2573_MAC_CSR2, tmp);

	tmp = addr[4] | (addr[5] << 8) | (0xff << 16);
	rum_cfg_write(sc, RT2573_MAC_CSR3, tmp);

	return;
}

static void
rum_cfg_update_promisc(struct rum_softc *sc,
		       struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;

	tmp = rum_cfg_read(sc, RT2573_TXRX_CSR0);

	if (cc->if_flags & IFF_PROMISC)
	    tmp &= ~RT2573_DROP_NOT_TO_ME;
	else
	    tmp |= RT2573_DROP_NOT_TO_ME;

	rum_cfg_write(sc, RT2573_TXRX_CSR0, tmp);

	DPRINTF(sc, 0, "%s promiscuous mode\n", 
		(cc->if_flags & IFF_PROMISC) ? 
		"entering" : "leaving");
	return;
}

static void
rum_cfg_select_antenna(struct rum_softc *sc, 
		       struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;
	uint8_t bbp3;
	uint8_t bbp4;
	uint8_t bbp77;
	uint8_t rx_ant;
	uint8_t is_5ghz;

	bbp3  = rum_cfg_bbp_read(sc, 3);
	bbp4  = rum_cfg_bbp_read(sc, 4);
	bbp77 = rum_cfg_bbp_read(sc, 77);

	bbp3 &= ~0x01;
	bbp4 &= ~0x23;

	rx_ant = sc->sc_rx_ant;
	is_5ghz = cc->ic_curchan.chan_is_5ghz;

	if ((sc->sc_rf_rev == RT2573_RF_5226) ||
	    (sc->sc_rf_rev == RT2573_RF_5225)) {

	    if (rx_ant == 0) {
	        /* Diversity */
	        bbp4 |= 0x02;
		if (is_5ghz == 0)
		    bbp4 |= 0x20;
	    } else if (rx_ant == 1) {
	        /* RX: Antenna A */
	        bbp4 |= 0x01;
		if (is_5ghz)
		    bbp77 &= ~0x03;
		else
		    bbp77 |= 0x03;
	    } else if (rx_ant == 2) {
	        /* RX: Antenna B */
	        bbp4 |= 0x01;
		if (is_5ghz)
		    bbp77 |= 0x03;
		else
		    bbp77 &= ~0x03;
	    }

	} else if ((sc->sc_rf_rev == RT2573_RF_2528) ||
		   (sc->sc_rf_rev == RT2573_RF_2527)) {

	    if (rx_ant == 0) {
	        /* Diversity */
	        bbp4 |= 0x22;
	    } else if (rx_ant == 1) {
	        /* RX: Antenna A */
	        bbp4 |= 0x21;
		bbp77 |= 0x03;
	    } else if (rx_ant == 2) {
	        /* RX: Antenna B */
	        bbp4 |= 0x21;
		bbp77 &= ~0x03;
	    }
	}

	bbp4 &= ~(sc->sc_ftype << 5);

	/* make sure Rx is disabled before switching antenna */
	tmp = rum_cfg_read(sc, RT2573_TXRX_CSR0);
	rum_cfg_write(sc, RT2573_TXRX_CSR0, tmp | RT2573_DISABLE_RX);

	rum_cfg_bbp_write(sc,  3, bbp3);
	rum_cfg_bbp_write(sc,  4, bbp4);
	rum_cfg_bbp_write(sc, 77, bbp77);

	rum_cfg_write(sc, RT2573_TXRX_CSR0, tmp);

	return;
}

static void
rum_cfg_read_eeprom(struct rum_softc *sc)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	uint16_t val;

	/* read MAC address */
	rum_cfg_eeprom_read(sc, RT2573_EEPROM_ADDRESS, ic->ic_myaddr, 6);

	val = rum_cfg_eeprom_read_2(sc, RT2573_EEPROM_ANTENNA);
	sc->sc_rf_rev =   (val >> 11) & 0x1f;
	sc->sc_hw_radio = (val >> 10) & 0x1;
	sc->sc_ftype =    (val >> 6)  & 0x1;
	sc->sc_rx_ant =   (val >> 4)  & 0x3;
	sc->sc_tx_ant =   (val >> 2)  & 0x3;
	sc->sc_nb_ant =   (val & 0x3);

	DPRINTF(sc, 0, "RF revision=%d\n", sc->sc_rf_rev);

	val = rum_cfg_eeprom_read_2(sc, RT2573_EEPROM_CONFIG2);
	sc->sc_ext_5ghz_lna = (val >> 6) & 0x1;
	sc->sc_ext_2ghz_lna = (val >> 4) & 0x1;

	DPRINTF(sc, 0, "External 2GHz LNA=%d, External 5GHz LNA=%d\n",
		sc->sc_ext_2ghz_lna, sc->sc_ext_5ghz_lna);

	val = rum_cfg_eeprom_read_2(sc, RT2573_EEPROM_RSSI_2GHZ_OFFSET);
	if ((val & 0xff) != 0xff)
		sc->sc_rssi_2ghz_corr = (int8_t)(val & 0xff);	/* signed */
	else
		sc->sc_rssi_2ghz_corr = 0;

	val = rum_cfg_eeprom_read_2(sc, RT2573_EEPROM_RSSI_5GHZ_OFFSET);
	if ((val & 0xff) != 0xff)
		sc->sc_rssi_5ghz_corr = (int8_t)(val & 0xff);	/* signed */
	else
		sc->sc_rssi_5ghz_corr = 0;

	DPRINTF(sc, 0, "RSSI 2GHz corr=%d, RSSI 5GHz corr=%d\n",
		sc->sc_rssi_2ghz_corr, sc->sc_rssi_5ghz_corr);

	val = rum_cfg_eeprom_read_2(sc, RT2573_EEPROM_FREQ_OFFSET);
	if ((val & 0xff) != 0xff)
		sc->sc_rffreq = (val & 0xff);
	else
		sc->sc_rffreq = 0;

	DPRINTF(sc, 0, "RF freq=%d\n", sc->sc_rffreq);

	/* read Tx power for all a/b/g channels */
	rum_cfg_eeprom_read(sc, RT2573_EEPROM_TXPOWER, sc->sc_txpow, 14);

	/* XXX default Tx power for 802.11a channels */
	memset(sc->sc_txpow + 14, 24, sizeof(sc->sc_txpow) - 14);

	/* read default values for BBP registers */
	rum_cfg_eeprom_read(sc, RT2573_EEPROM_BBP_BASE, sc->sc_bbp_prom, 2 * 16);

	return;
}

static uint8_t
rum_cfg_bbp_init(struct rum_softc *sc)
{
	enum { N_DEF_BBP = (sizeof(rum_def_bbp)/sizeof(rum_def_bbp[0])) };
	uint16_t i;
	uint8_t to;
	uint8_t tmp;

	/* wait for BBP to become ready */
	for (to = 0; ; to++) {
	    if (to < 100) {
	        tmp = rum_cfg_bbp_read(sc, 0);
		if ((tmp != 0x00) &&
		    (tmp != 0xff)) {
		    break;
		}
		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    return 1; /* failure */
		}
	    } else {
	        DPRINTF(sc, 0, "timeout waiting for BBP\n");
		return 1; /* failure */
	    }
	}

	/* initialize BBP registers to default values */
	for (i = 0; i < N_DEF_BBP; i++) {
	    rum_cfg_bbp_write(sc, rum_def_bbp[i].reg, rum_def_bbp[i].val);
	}

	/* write vendor-specific BBP values (from EEPROM) */
	for (i = 0; i < 16; i++) {
	    if ((sc->sc_bbp_prom[i].reg == 0) || 
		(sc->sc_bbp_prom[i].reg == 0xff)) {
	        continue;
	    }
	    rum_cfg_bbp_write(sc, sc->sc_bbp_prom[i].reg, sc->sc_bbp_prom[i].val);
	}
	return 0;
}

static void
rum_cfg_pre_init(struct rum_softc *sc,
		 struct rum_config_copy *cc, uint16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = sc->sc_ic.ic_ifp;

	/* immediate configuration */

	rum_cfg_pre_stop(sc, cc, 0);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

	sc->sc_flags |= RUM_FLAG_HL_READY;

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

static void
rum_cfg_init(struct rum_softc *sc,
	     struct rum_config_copy *cc, uint16_t refcount)
{
	enum { N_DEF_MAC = (sizeof(rum_def_mac)/sizeof(rum_def_mac[0])) };

	uint32_t tmp;
	uint16_t i;
	uint8_t to;

	/* delayed configuration */

	rum_cfg_stop(sc, cc, 0);

	/* initialize MAC registers to default values */
	for (i = 0; i < N_DEF_MAC; i++) {
	    rum_cfg_write(sc, rum_def_mac[i].reg, rum_def_mac[i].val);
	}

	/* set host ready */
	rum_cfg_write(sc, RT2573_MAC_CSR1, 3);
	rum_cfg_write(sc, RT2573_MAC_CSR1, 0);

	/* wait for BBP/RF to wakeup */
	for (to = 0; ; to++) {
	    if (to < 100) {
	        if (rum_cfg_read(sc, RT2573_MAC_CSR12) & 8) {
		    break;
		}

		rum_cfg_write(sc, RT2573_MAC_CSR12, 4); /* force wakeup */

		if (usbd_config_td_sleep(&(sc->sc_config_td), hz/100)) {
		    goto fail;
		}
	    } else {
	        DPRINTF(sc, 0, "timeout waiting for "
			"BBP/RF to wakeup\n");
		goto fail;
	    }
	}

	if (rum_cfg_bbp_init(sc)) {
	    goto fail;
	}

	/* select default channel */

	sc->sc_last_chan = 0;

	rum_cfg_set_chan(sc, cc, 0);

	/* clear STA registers */
	rum_cfg_read_multi(sc, RT2573_STA_CSR0, sc->sc_sta, sizeof(sc->sc_sta));

	/* set MAC address */
	rum_cfg_set_macaddr(sc, cc->ic_myaddr);

	/* initialize ASIC */
	rum_cfg_write(sc, RT2573_MAC_CSR1, 4);

	/*
	 * make sure that the first transaction
	 * clears the stall:
	 */
	sc->sc_flags |= (RUM_FLAG_READ_STALL|
			 RUM_FLAG_WRITE_STALL|
			 RUM_FLAG_LL_READY);

	if ((sc->sc_flags & RUM_FLAG_LL_READY) &&
	    (sc->sc_flags & RUM_FLAG_HL_READY)) {

	    /* start the USB transfers, 
	     * if not already started:
	     */
	    usbd_transfer_start(sc->sc_xfer[1]);
	    usbd_transfer_start(sc->sc_xfer[0]);
	}

	/* update Rx filter */
	tmp = rum_cfg_read(sc, RT2573_TXRX_CSR0) & 0xffff;

	tmp |= RT2573_DROP_PHY_ERROR | RT2573_DROP_CRC_ERROR;

	if (cc->ic_opmode != IEEE80211_M_MONITOR) {
		tmp |= RT2573_DROP_CTL | RT2573_DROP_VER_ERROR |
		       RT2573_DROP_ACKCTS;
		if (cc->ic_opmode != IEEE80211_M_HOSTAP) {
			tmp |= RT2573_DROP_TODS;
		}
		if (!(cc->if_flags & IFF_PROMISC)) {
			tmp |= RT2573_DROP_NOT_TO_ME;
		}
	}
	rum_cfg_write(sc, RT2573_TXRX_CSR0, tmp);

	return;

 fail:
	rum_cfg_pre_stop(sc, NULL, 0);

	if (cc) {
	    rum_cfg_stop(sc, cc, 0);
	}
	return;
}

static void
rum_cfg_pre_stop(struct rum_softc *sc,
		 struct rum_config_copy *cc, uint16_t refcount)
{
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ifnet *ifp = ic->ic_ifp;

	if (cc) {
	    /* copy the needed configuration */
	    rum_config_copy(sc, cc, refcount);
	}

	/* immediate configuration */

	if (ifp) {

	    ieee80211_new_state(ic, IEEE80211_S_INIT, -1);

	    /* clear flags */
	    ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	}

	/* stop timers */
	sc->sc_if_timer = 0;

	sc->sc_flags &= ~(RUM_FLAG_HL_READY|
			  RUM_FLAG_LL_READY);

	/* stop all the transfers, 
	 * if not already stopped:
	 */
	usbd_transfer_stop(sc->sc_xfer[0]);
	usbd_transfer_stop(sc->sc_xfer[1]);
	usbd_transfer_stop(sc->sc_xfer[2]);
	usbd_transfer_stop(sc->sc_xfer[3]);

	return;
}

static void
rum_cfg_stop(struct rum_softc *sc,
	     struct rum_config_copy *cc, uint16_t refcount)
{
	uint32_t tmp;

	/* disable Rx */
	tmp = rum_cfg_read(sc, RT2573_TXRX_CSR0);
	rum_cfg_write(sc, RT2573_TXRX_CSR0, tmp | RT2573_DISABLE_RX);

	/* reset ASIC */
	rum_cfg_write(sc, RT2573_MAC_CSR1, 3);

	/* wait a little */
	usbd_config_td_sleep(&(sc->sc_config_td), hz/10);

	rum_cfg_write(sc, RT2573_MAC_CSR1, 0);

	/* wait a little */
	usbd_config_td_sleep(&(sc->sc_config_td), hz/10);

	return;
}

static void
rum_cfg_amrr_start(struct rum_softc *sc)
{
	struct ieee80211_node *ni = sc->sc_ic.ic_bss;
	uint16_t i;

	/* init AMRR */

	ieee80211_amrr_node_init(&(sc->sc_amrr), &(sc->sc_amn));

	/* set rate to some reasonable initial value */

	i = ni->ni_rates.rs_nrates;
	while(i) {
	    i--;
	    if ((ni->ni_rates.rs_rates[i] & IEEE80211_RATE_VAL) <= 72) break;
	}
	ni->ni_txrate = i;

	sc->sc_amrr_timer = (1*8);
	return;
}

static void
rum_cfg_amrr_timeout(struct rum_softc *sc,
		     struct rum_config_copy *cc, uint16_t refcount)
{
	struct ifnet *ifp = sc->sc_ic.ic_ifp;

	/* clear statistic registers (STA_CSR0 to STA_CSR5) */
	rum_cfg_read_multi(sc, RT2573_STA_CSR0, sc->sc_sta, sizeof(sc->sc_sta));

	if ((sc->sc_flags & RUM_FLAG_LL_READY) &&
	    (sc->sc_flags & RUM_FLAG_HL_READY)) {

	    /* count TX retry-fail as Tx errors */
	    ifp->if_oerrors += (le32toh(sc->sc_sta[5]) >> 16);

	    sc->sc_amn.amn_retrycnt =
	      (le32toh(sc->sc_sta[4]) >> 16) +	/* TX one-retry ok count */
	      (le32toh(sc->sc_sta[5]) & 0xffff) + /* TX more-retry ok count */
	      (le32toh(sc->sc_sta[5]) >> 16); /* TX retry-fail count */

	    sc->sc_amn.amn_txcnt =
	      sc->sc_amn.amn_retrycnt +
	      (le32toh(sc->sc_sta[4]) & 0xffff); /* TX no-retry ok count */

	    ieee80211_amrr_choose(&(sc->sc_amrr), sc->sc_ic.ic_bss, 
				  &(sc->sc_amn));
	}
	return;
}

static void
rum_cfg_load_microcode(struct rum_softc *sc, const uint8_t *ucode, uint16_t size)
{
	usb_device_request_t req;
	uint16_t reg = RT2573_MCU_CODE_BASE;

	/* copy firmware image into NIC */
	while (size >= 4) {
	    rum_cfg_write(sc, reg, UGETDW(ucode));
	    reg += 4;
	    ucode += 4;
	    size -= 4;
	}

	if (size != 0) {
	    DPRINTF(sc, 0, "possibly invalid firmware\n");
	}

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = RT2573_MCU_CNTL;
	USETW(req.wValue, RT2573_MCU_RUN);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	rum_cfg_do_request(sc, &req, NULL);

	return;
}

static void
rum_cfg_prepare_beacon(struct rum_softc *sc,
		       struct rum_config_copy *cc, uint16_t refcount)
{
	struct rum_tx_desc desc;
	struct ieee80211com *ic = &(sc->sc_ic);
	struct ieee80211_node *ni = sc->sc_ic.ic_bss;
	struct mbuf *m;
	uint16_t rate;

	if ((sc->sc_flags & RUM_FLAG_LL_READY) &&
	    (sc->sc_flags & RUM_FLAG_HL_READY)) {

	    m = ieee80211_beacon_alloc(ic, ic->ic_bss, &sc->sc_bo);
	    if (m == NULL) {
	        return;
	    }

	    /* send beacons at the lowest available rate */
	    rate = IEEE80211_IS_CHAN_5GHZ(ni->ni_chan) ? 12 : 2;

	    rum_setup_tx_desc(sc, RT2573_TX_TIMESTAMP, RT2573_TX_HWSEQ,
			      m->m_len, rate);

	    /* make a copy ! */
	    desc = sc->sc_tx_desc;

	    /* copy the first 24 bytes of Tx descriptor into NIC memory */
	    rum_cfg_write_multi(sc, RT2573_HW_BEACON_BASE0, &desc, 24);

	    /* copy beacon header and payload into NIC memory */
	    rum_cfg_write_multi(sc, RT2573_HW_BEACON_BASE0 + 24,
				m->m_data, m->m_len);
	    m_freem(m);
	}
	return;
}

