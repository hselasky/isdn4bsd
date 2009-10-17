/*-
 * Copyright (c) 2009 Hans Petter Selasky. All rights reserved.
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
 *
 */

#ifndef _DSS1_LITE_H_
#define	_DSS1_LITE_H_

#define	DL_TEI 0x01
#define	DL_NUM_MAX 0x80
#define	DL_WIN_MAX 0x10

#define	DL_ST_FREE 0xFF
#define	DL_STATUS_COUNT_MAX 8		/* timeouts ~ 8*8 = 64 seconds */

/*
 * Odd states are timeout states.
 */

enum {
	DL_ST_OUT_U0 = 0x00,
	DL_ST_OUT_U0_TO,
	DL_ST_OUT_U1,
	DL_ST_OUT_U1_TO,
	DL_ST_OUT_U2,
	DL_ST_OUT_U2_ACK,
	DL_ST_OUT_U3,
	DL_ST_OUT_U3_TO,
	DL_ST_OUT_U4,
	DL_ST_OUT_U4_TO,
	DL_ST_OUT_UA,
	DL_ST_OUT_UA_TO,
	DL_ST_OUT_UC,
	DL_ST_OUT_UC_TO,
	DL_ST_OUT_MAX,
};

enum {
	DL_ST_IN_U0 = 0x80,
	DL_ST_IN_U0_ACK,
	DL_ST_IN_U6,
	DL_ST_IN_U6_TO,
	DL_ST_IN_U7,
	DL_ST_IN_U7_TO,
	DL_ST_IN_U8,
	DL_ST_IN_U8_TO,
	DL_ST_IN_UA,
	DL_ST_IN_UA_TO,
	DL_ST_IN_UC,
	DL_ST_IN_UC_TO,
	DL_ST_IN_MAX,
};

struct dss1_lite;

typedef uint8_t (dss1_lite_ie_t)(struct dss1_lite *, uint8_t *);

struct dss1_lite_ie_func {
	dss1_lite_ie_t *pfunc;
	uint32_t mask;
};

/* definition of "mask" */

#define	DL_IE_HEADER_MASK (1UL << 0)
#define	DL_IE_SENDING_COMPLETE_MASK (1UL << 1)
#define	DL_IE_BEARER_CAP_MASK (1UL << 2)
#define	DL_IE_CHANNEL_ID_MASK (1UL << 3)
#define	DL_IE_KEYPAD_MASK (1UL << 4)
#define	DL_IE_CALLING_PARTY_MASK (1UL << 5)
#define	DL_IE_CALLING_SUBADDR_MASK (1UL << 6)
#define	DL_IE_USERUSER_MASK (1UL << 7)
#define	DL_IE_DISPLAY_MASK (1UL << 8)
#define	DL_IE_CAUSE_MASK (1UL << 9)
#define	DL_IE_CALLSTATE_MASK (1UL << 10)
#define	DL_IE_PROGRESS_MASK (1UL << 11)
#define	DL_IE_CALLED_PARTY_MASK (1UL << 12)
#define	DL_IE_CALLED_SUBADDR_MASK (1UL << 13)
#define	DL_IE_DEFLECT_MASK (1UL << 14)
#define	DL_IE_MCID_MASK (1UL << 15)
#define	DL_IE_DATE_MASK (1UL << 16)

struct dss1_lite_state {
	uint8_t	timeout;
	uint8_t	next_state;
	uint8_t	q931_state;
	uint8_t	flags;
};

typedef void (dss1_lite_set_ring_t)(struct dss1_lite *, uint8_t);

struct dss1_lite_methods {
	dss1_lite_set_ring_t *set_ring;
};

struct dss1_lite_ifq {
	struct mbuf *ifq_head;
	struct mbuf *ifq_tail;
	uint8_t	ifq_len;
	uint8_t	ifq_maxlen;
};

struct dss1_lite {

	struct callout dl_timer;
	struct dss1_lite_ifq dl_outq;
	struct dss1_lite_ifq dl_inq;

	const struct dss1_lite_methods *dl_methods;
	void   *dl_softc;
	struct mtx *dl_pmtx;
	const struct dss1_lite_state *dl_pstate;	/* current state */
	struct mbuf *dl_tx_mbuf[DL_WIN_MAX];

	int	dl_timeout_tick;

	uint8_t	dl_timeout_active;
	uint8_t	dl_tx_in;
	uint8_t	dl_tx_out;
	uint8_t	dl_tx_num;
	uint8_t	dl_rx_num;
	uint8_t	dl_state_index;
	uint8_t	dl_no_rc;		/* Set if no release complete */
	uint8_t	dl_status_count;
	uint8_t	dl_channel;		/* Data channel */
	uint8_t	dl_curr_callref;
	uint8_t	dl_next_callref;
	uint8_t	dl_is_nt_mode;
};

#endif					/* _DSS1_LITE_H_ */
