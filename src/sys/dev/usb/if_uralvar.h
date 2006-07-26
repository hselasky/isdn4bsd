/*	$FreeBSD: src/sys/dev/usb2/if_uralvar.h,v 1.6 2005/11/29 20:55:53 damien Exp $	*/

/*-
 * Copyright (c) 2005
 *	Damien Bergamini <damien.bergamini@free.fr>
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

struct ural_softc;
struct ural_config_copy;

typedef void (ural_command_t)(struct ural_softc *sc, 
			      struct ural_config_copy *cc);
struct ural_config_copy {
	struct {
	  u_int32_t		chan_to_ieee;
	  u_int8_t		chan_is_2ghz;
	} ic_curchan;

	struct {
	  struct {
	    u_int8_t		chan_is_5ghz;
	  } ni_chan;

	  u_int16_t		ni_intval;
	  u_int8_t		ni_bssid[IEEE80211_ADDR_LEN];
	} ic_bss;

	struct {
	  struct {
	    ieee80211_keyix wk_keyix;
	    u_int8_t wk_key[IEEE80211_KEYBUF_SIZE];
	  } cs_nw_keys[IEEE80211_WEP_NKID];
	} ic_crypto;

	enum ieee80211_opmode	ic_opmode;
	enum ieee80211_state	ic_state;
	u_int32_t		ic_flags;
	u_int32_t		if_flags;

	u_int16_t		ic_txpowlimit;
	u_int16_t		ic_curmode;

	u_int8_t		ic_myaddr[IEEE80211_ADDR_LEN];

	ural_command_t *	command_func;
};

struct ural_rx_radiotap_header {
	struct ieee80211_radiotap_header wr_ihdr;
	u_int8_t	wr_flags;
	u_int8_t	wr_rate;
	u_int16_t	wr_chan_freq;
	u_int16_t	wr_chan_flags;
	u_int8_t	wr_antenna;
	u_int8_t	wr_antsignal;
};

#define RAL_RX_RADIOTAP_PRESENT						\
	((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_CHANNEL) |				\
	 (1 << IEEE80211_RADIOTAP_ANTENNA) |				\
	 (1 << IEEE80211_RADIOTAP_DB_ANTSIGNAL))

struct ural_tx_radiotap_header {
	struct ieee80211_radiotap_header wt_ihdr;
	u_int8_t	wt_flags;
	u_int8_t	wt_rate;
	u_int16_t	wt_chan_freq;
	u_int16_t	wt_chan_flags;
	u_int8_t	wt_antenna;
};

#define RAL_TX_RADIOTAP_PRESENT						\
	((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_CHANNEL) |				\
	 (1 << IEEE80211_RADIOTAP_ANTENNA))

struct ural_amrr {
	int	txcnt;
	int	retrycnt;
	int	success;
	int	success_threshold;
	int	recovery;
};

union ural_rxtap {
	struct ural_rx_radiotap_header h;
	u_int8_t pad[64];
};

union ural_txtap {
	struct ural_tx_radiotap_header h;
	u_int8_t pad[64];
};

struct ural_bbp_prom {
	u_int8_t	val;
	u_int8_t	reg;
} __packed;

#define URAL_N_TRANSFER 4

struct ural_softc {

	struct ural_tx_desc		sc_tx_desc;
	struct ural_rx_desc		sc_rx_desc;
	struct ieee80211com		sc_ic;
	struct ural_amrr		sc_amrr;
	struct ieee80211_beacon_offsets	sc_bo;
	struct mtx			sc_mtx;
	struct usbd_memory_wait		sc_mem_wait;
	struct __callout		sc_watchdog;
	struct ural_bbp_prom 		sc_bbp_prom[16];
	struct usbd_xfer *		sc_xfer[URAL_N_TRANSFER];
	struct usbd_ifqueue		sc_cmd_free;
	struct usbd_ifqueue		sc_cmd_used;
	union ural_rxtap		sc_rxtap;
	union ural_txtap		sc_txtap;

	struct mbuf *			sc_bcn_mbuf;
 	struct proc *			sc_config_thread;
	struct ifnet *			sc_ifp;
	struct bpf_if *			sc_drvbpf;
	void *				sc_cmd_queue_ptr;
	struct usbd_device *		sc_udev;

	int (*sc_newstate)
	  (struct ieee80211com *, enum ieee80211_state, int);

	enum ieee80211_state		sc_state;

	u_int32_t			sc_bcn_flags;
	u_int32_t			sc_unit;
	u_int32_t			sc_asic_rev;
	u_int32_t			sc_rf_regs[4];
	u_int32_t			sc_flags;
#define URAL_FLAG_DEV_GONE		0x0001
#define URAL_FLAG_TD_EXIT		0x0002
#define URAL_FLAG_CMD_SLEEP		0x0004
#define URAL_FLAG_READ_STALL		0x0008
#define URAL_FLAG_WRITE_STALL		0x0010
#define URAL_FLAG_SEND_BYTE_FRAME	0x0020
#define URAL_FLAG_SEND_BCN_FRAME	0x0040
#define URAL_FLAG_LL_READY		0x0080
#define URAL_FLAG_HL_READY		0x0100
#define URAL_FLAG_WAIT_COMMAND		0x0200

	u_int16_t			sc_txtap_len;
	u_int16_t			sc_rxtap_len;
	u_int16_t			sc_sta[11];
	u_int16_t			sc_bcn_rate;

	u_int8_t			sc_rf_rev;
	u_int8_t			sc_txpow[14];
	u_int8_t			sc_led_mode;
	u_int8_t			sc_hw_radio;
	u_int8_t			sc_rx_ant;
	u_int8_t			sc_tx_ant;
	u_int8_t			sc_nb_ant;
	u_int8_t			sc_if_timer;
	u_int8_t			sc_tx_timer;
	u_int8_t			sc_scan_timer;
	u_int8_t			sc_amrr_timer;
	u_int8_t			sc_name[32];

	u_int8_t			sc_wakeup_td_gone;
	u_int8_t			sc_wakeup_cfg;
	u_int8_t			sc_wakeup_bcn;
};

