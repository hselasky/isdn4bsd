/*	$FreeBSD: src/sys/dev/usb/if_uralvar.h,v 1.7 2006/09/07 00:06:41 imp Exp $	*/

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

#define URAL_N_TRANSFER 4

struct ural_config_copy {
	struct {
		uint32_t	chan_to_ieee;
		uint8_t		chan_is_2ghz;
	} ic_curchan;

	struct {
		struct {
			uint8_t		chan_is_5ghz;
		} ni_chan;

		uint16_t	ni_intval;
		uint8_t		ni_bssid[IEEE80211_ADDR_LEN];
	} ic_bss;

	struct {
		struct {
			ieee80211_keyix	wk_keyix;
			uint8_t		wk_key[IEEE80211_KEYBUF_SIZE];
		} cs_nw_keys[IEEE80211_WEP_NKID];
	} ic_crypto;

	enum ieee80211_opmode	ic_opmode;
	enum ieee80211_state	ic_state;
	uint32_t		ic_flags;
	uint32_t		if_flags;

	uint16_t		ic_txpowlimit;
	uint16_t		ic_curmode;

	uint8_t			ic_myaddr[IEEE80211_ADDR_LEN];
};

struct ural_rx_radiotap_header {
	struct ieee80211_radiotap_header wr_ihdr;
	uint8_t		wr_flags;
	uint8_t		wr_rate;
	uint16_t	wr_chan_freq;
	uint16_t	wr_chan_flags;
	uint8_t		wr_antenna;
	uint8_t		wr_antsignal;
};

#define RAL_RX_RADIOTAP_PRESENT						\
	    ((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	     (1 << IEEE80211_RADIOTAP_RATE) |				\
	     (1 << IEEE80211_RADIOTAP_CHANNEL) |			\
	     (1 << IEEE80211_RADIOTAP_ANTENNA) |			\
	     (1 << IEEE80211_RADIOTAP_DB_ANTSIGNAL))

struct ural_tx_radiotap_header {
	struct ieee80211_radiotap_header wt_ihdr;
	uint8_t		wt_flags;
	uint8_t		wt_rate;
	uint16_t	wt_chan_freq;
	uint16_t	wt_chan_flags;
	uint8_t		wt_antenna;
};

#define RAL_TX_RADIOTAP_PRESENT						\
	    ((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	     (1 << IEEE80211_RADIOTAP_RATE) |				\
	     (1 << IEEE80211_RADIOTAP_CHANNEL) |			\
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
	uint8_t pad[64];
};

union ural_txtap {
	struct ural_tx_radiotap_header h;
	uint8_t pad[64];
};

struct ural_bbp_prom {
	uint8_t	val;
	uint8_t	reg;
} __packed;

struct ural_softc {
	void				*sc_evilhack; /* XXX this pointer must be first */

	struct usbd_config_td		sc_config_td;
	struct ural_tx_desc		sc_tx_desc;
	struct ural_rx_desc		sc_rx_desc;
	struct ieee80211com		sc_ic;
	struct ural_amrr		sc_amrr;
	struct ieee80211_beacon_offsets	sc_bo;
	struct mtx			sc_mtx;
	struct __callout		sc_watchdog;
	struct ural_bbp_prom 		sc_bbp_prom[16];
	union ural_rxtap		sc_rxtap;
	union ural_txtap		sc_txtap;

	struct usbd_xfer 		*sc_xfer[URAL_N_TRANSFER];
	struct mbuf 			*sc_bcn_mbuf;
	struct ifnet 			*sc_ifp;
	struct bpf_if 			*sc_drvbpf;
	struct usbd_device 		*sc_udev;

	int (*sc_newstate)
	    (struct ieee80211com *, enum ieee80211_state, int);

	uint32_t			sc_bcn_flags;
	uint32_t			sc_unit;
	uint32_t			sc_asic_rev;
	uint32_t			sc_rf_regs[4];

	uint16_t			sc_flags;
#define	URAL_FLAG_READ_STALL		0x0001
#define	URAL_FLAG_WRITE_STALL		0x0002
#define	URAL_FLAG_SEND_BYTE_FRAME	0x0004
#define	URAL_FLAG_SEND_BCN_FRAME	0x0008
#define	URAL_FLAG_LL_READY		0x0010
#define	URAL_FLAG_HL_READY		0x0020
#define	URAL_FLAG_WAIT_COMMAND		0x0040
	uint16_t			sc_txtap_len;
	uint16_t			sc_rxtap_len;
	uint16_t			sc_sta[11];
	uint16_t			sc_bcn_rate;

	uint8_t				sc_rf_rev;
	uint8_t				sc_txpow[14];
	uint8_t				sc_led_mode;
	uint8_t				sc_hw_radio;
	uint8_t				sc_rx_ant;
	uint8_t				sc_tx_ant;
	uint8_t				sc_nb_ant;
	uint8_t				sc_if_timer;
	uint8_t				sc_scan_timer;
	uint8_t				sc_amrr_timer;
	uint8_t				sc_name[32];
};

