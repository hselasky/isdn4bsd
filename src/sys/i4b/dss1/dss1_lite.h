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
#define	DL_QUEUE_MAX 0x8
#define	DL_BPS_MAX 512			/* bytes/second */

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
typedef void (dss1_lite_set_ring_t)(struct dss1_lite *, uint8_t);

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

/* definition of "channel_id" */

enum {
	CHAN_NOT_ANY = 0xFFFE,
	CHAN_ANY = 0xFFFF,
	CHAN_D1 = 0,
	CHAN_B1 = 1,
	CHAN_B2 = 2,
};

/* definition of B-protocol, "bprot" */

enum {
	BSUBPROT_G711_ALAW,
	BSUBPROT_G711_ULAW,
};

/* definition of sub B-protocol, "bsubprot" */

enum {
	BPROT_TELEPHONY,
	BPROT_DIGITAL,
};

/* definition of type of number, "ton" */

enum {
	TON_OTHER,			/* other type of number */
	TON_INTERNAT,			/* international number */
	TON_NATIONAL,			/* national number */
};

/* definition of screening indicator, "scr" */

enum {
	SCR_NONE,			/* no screening indicator transmitted */
	SCR_USR_NOSC,			/* screening user provided, not
					 * screened */
	SCR_USR_PASS,			/* screening user provided, verified &
					 * passed */
	SCR_USR_FAIL,			/* screening user provided, verified &
					 * failed */
	SCR_NET,			/* screening network provided */
};

/* definition of presentation indicator, "prs" */

enum {
	PRS_NONE,			/* no presentation indicator
					 * transmitted */
	PRS_ALLOWED,			/* presentation allowed */
	PRS_RESTRICT,			/* presentation restricted */
	PRS_NNINTERW,			/* number not available due to
					 * interworking */
	PRS_RESERVED,			/* reserved */
};

/* PD: Protocol discriminator */
#define	PD_Q931         0x08		/* Q.931/I.451 */

/* EXT: Extension bit */
#define	EXT_LAST                0x80	/* last octett */

/* BC: information xfer capability */
#define	IT_CAP_SPEECH           (0x00|EXT_LAST)
#define	IT_CAP_UNR_DIG_INFO     (0x08|EXT_LAST)
#define	IT_CAP_R_DIG_INFO       (0x09|EXT_LAST)
#define	IT_CAP_AUDIO_3100Hz     (0x10|EXT_LAST)
#define	IT_CAP_UNR_DIG_TONES    (0x11|EXT_LAST)
#define	IT_CAP_VIDEO            (0x18|EXT_LAST)

/* BC: information xfer rate    */
#define	IT_RATE_64K             0x90
#define	IT_RATE_56K             0x8f

/* L1: protocol G.711 A-law  */
#define	IT_UL1_G711U            0xa2
#define	IT_UL1_G711A            0xa3

/* L1: channel ID */
#define	IE_CHAN_ID_NO           0x00	/* no channel */
#define	IE_CHAN_ID_B1           0x01	/* B1 channel */
#define	IE_CHAN_ID_B2           0x02	/* B2 channel */
#define	IE_CHAN_ID_ANY          0x03	/* ANY channel */

/* TN: number types */
#define	NUMBER_TYPE_NSAP        0x80	/* subaddr: type=NSAP */
#define	NUMBER_TYPE_PLAN        0x81	/* type of numbering plan */

/* CAUSE: Cause location */
#define	CAUSE_STD_LOC_OUT       0x80	/* std = CCITT, loc = user */
#define	CAUSE_STD_LOC_PUBLIC    0x82	/* std = CCITT, loc = public */

/* IEI: Information Element ID */
#define	IEI_SENDCOMPL   0xa1
#define	IEI_BEARERCAP   0x04
#define	IEI_CAUSE       0x08
#define	IEI_CALLID      0x10
#define	IEI_CALLSTATE   0x14
#define	IEI_CHANNELID   0x18
#define	IEI_PROGRESSI   0x1e
#define	IEI_DISPLAY     0x28
#define	IEI_DATETIME    0x29
#define	IEI_KEYPAD	0x2c
#define	IEI_CALLINGPN   0x6c
#define	IEI_CALLINGPS   0x6d
#define	IEI_CALLEDPN    0x70
#define	IEI_CALLEDPS    0x71
#define	IEI_USERUSER    0x7e
#define	IEI_FACILITY    0x1c

struct dss1_lite_num {
	char	telno[64];		/* includes terminating zero */
	uint8_t	prs_ind;		/* See PRS_XXX */
	uint8_t	ton;			/* See TON_XXX */
};

struct dss1_lite_state {
	uint8_t	timeout;
	uint8_t	next_state;
	uint8_t	q931_state;
	uint8_t	flags;
};

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
	struct dss1_lite_num dl_src[2];
	struct dss1_lite_num dl_dst[2];
	struct dss1_lite_num dl_part;

	const struct dss1_lite_methods *dl_methods;
	void   *dl_softc;
	struct mtx *dl_pmtx;
	const struct dss1_lite_state *dl_pstate;	/* current state */
	struct mbuf *dl_tx_mbuf[DL_QUEUE_MAX];

	int	dl_timeout_tick;
	int	dl_tx_end_tick;

	uint16_t dl_channel_id;

	uint8_t	dl_timeout_active;
	uint8_t	dl_tx_in;
	uint8_t	dl_tx_out;
	uint8_t	dl_tx_num;
	uint8_t	dl_tx_window;
	uint8_t	dl_rx_num;
	uint8_t	dl_state_index;
	uint8_t	dl_no_rc;		/* Set if no release complete */
	uint8_t	dl_status_count;
	uint8_t	dl_channel_bprot;	/* See BPROT_XXX */
	uint8_t	dl_channel_bsubprot;	/* See BSUBPROT_XXX */
	uint8_t	dl_curr_callref;
	uint8_t	dl_next_callref;
	uint8_t	dl_is_nt_mode;
	uint8_t	dl_channel_allocated;
};

#endif					/* _DSS1_LITE_H_ */
