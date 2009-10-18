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
 *---------------------------------------------------------------------------
 *
 *	dss1_lite.c - Point2Point EuroISDN DSS1 implementation for
 *	non-digital phone interfaces
 *	-----------------------------------------------------------------
 *
 * $FreeBSD: $
 *
 *
 * NOTE: This library is not thread safe and the caller is responsible
 * for the correct locking.
 *---------------------------------------------------------------------------*/

#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/unistd.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>

#include <i4b/include/i4b_controller.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/dss1/dss1_lite.h>

static const struct dss1_lite_state dss1_lite_out_states[DL_ST_OUT_MAX - DL_ST_OUT_U0] = {
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U0] = {0, DL_ST_FREE, 0x00, 0x00},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U0_TO] = {8, DL_ST_FREE, 0x00, 0x00},	/* unused */
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U1] = {16, DL_ST_FREE, 0x01, 0x00},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U1_TO] = {16, DL_ST_FREE, 0x01, 0x00},	/* unused */
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U2] = {16, DL_ST_FREE, 0x02, 0x00},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U2_ACK] = {16, DL_ST_FREE, 0x02, 0x00},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U3] = {8, DL_ST_OUT_U3_TO, 0x03, 0x00},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U3_TO] = {4, DL_ST_FREE, 0x03, 0x01},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U4] = {8, DL_ST_OUT_U4_TO, 0x04, 0x00},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_U4_TO] = {4, DL_ST_FREE, 0x04, 0x01},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_UA] = {8, DL_ST_OUT_UA_TO, 0x0A, 0x00},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_UA_TO] = {4, DL_ST_FREE, 0x0A, 0x01},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_UC] = {8, DL_ST_OUT_UC_TO, 0x0C, 0x00},
	[0 - DL_ST_OUT_U0 + DL_ST_OUT_UC_TO] = {4, DL_ST_FREE, 0x0C, 0x01},
};

static const struct dss1_lite_state dss1_lite_in_states[0 - DL_ST_IN_U0 + DL_ST_IN_MAX] = {
	[0 - DL_ST_IN_U0 + DL_ST_IN_U0] = {8, DL_ST_FREE, 0x00, 0x00},
	[0 - DL_ST_IN_U0 + DL_ST_IN_U0_ACK] = {16, DL_ST_FREE, 0x19, 0x00},
	[0 - DL_ST_IN_U0 + DL_ST_IN_U6] = {8, DL_ST_IN_U6_TO, 0x06, 0x00},
	[0 - DL_ST_IN_U0 + DL_ST_IN_U6_TO] = {4, DL_ST_FREE, 0x06, 0x01},
	[0 - DL_ST_IN_U0 + DL_ST_IN_U7] = {8, DL_ST_IN_U7_TO, 0x07, 0x00},
	[0 - DL_ST_IN_U0 + DL_ST_IN_U7_TO] = {4, DL_ST_FREE, 0x07, 0x01},
	[0 - DL_ST_IN_U0 + DL_ST_IN_U8] = {4, DL_ST_FREE, 0x08, 0x00},
	[0 - DL_ST_IN_U0 + DL_ST_IN_U8_TO] = {4, DL_ST_FREE, 0x08, 0x00},	/* unused */
	[0 - DL_ST_IN_U0 + DL_ST_IN_UA] = {8, DL_ST_IN_UA_TO, 0x0A, 0x00},
	[0 - DL_ST_IN_U0 + DL_ST_IN_UA_TO] = {4, DL_ST_FREE, 0x0A, 0x01},
	[0 - DL_ST_IN_U0 + DL_ST_IN_UC] = {8, DL_ST_IN_UC_TO, 0x0C, 0x00},
	[0 - DL_ST_IN_U0 + DL_ST_IN_UC_TO] = {4, DL_ST_FREE, 0x0C, 0x01},
};

static uint8_t dss1_lite_send_mbuf(struct dss1_lite *pdl, struct mbuf *m);
static uint8_t dss1_lite_send_message(struct dss1_lite_call_desc *cd, uint8_t msg_type, uint32_t mask);
static uint8_t dss1_lite_proceed_request(struct dss1_lite_call_desc *cd);
static uint8_t dss1_lite_alert_request(struct dss1_lite_call_desc *cd);
static uint8_t dss1_lite_setup_accept_resp(struct dss1_lite_call_desc *cd);

static struct i4b_controller *
dss1_lite_get_ctrl(struct dss1_lite_call_desc *cd)
{
	return (cd->dl_parent->dl_ctrl);
}

static uint8_t
dss1_lite_is_nt_mode(struct dss1_lite_call_desc *cd)
{
	return (cd->dl_parent->dl_is_nt_mode);
}

static uint8_t
dss1_lite_want_dialtone(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_is_nt_mode(cd) &&
	    (!dss1_lite_get_ctrl(cd)->no_layer1_dialtone) &&
	    (cd->dl_channel_bprot == BPROT_NONE));
}

static void
dss1_lite_set_ring(struct dss1_lite_call_desc *cd, uint8_t on)
{
	(cd->dl_parent->dl_methods->set_ring) (cd->dl_parent, on);
}

static void
dss1_lite_set_dtmf(struct dss1_lite_call_desc *cd, const char *pdtmf)
{
	(cd->dl_parent->dl_methods->set_dtmf) (cd->dl_parent, pdtmf);
}

static uint8_t
dss1_lite_send_setup(struct dss1_lite_call_desc *cd,
    uint8_t sending_complete)
{
	return (dss1_lite_send_message(cd, 0x05 /* SETUP */ ,
	    (IE_HEADER_MASK | IE_BEARER_CAP_MASK | IE_CHANNEL_ID_MASK |
	    IE_KEYPAD_MASK | IE_CALLING_PARTY_MASK | IE_USERUSER_MASK |
	    IE_DISPLAY_MASK) |
	    (sending_complete ? IE_SENDING_COMPLETE_MASK : 0)));
}

static uint8_t
dss1_lite_send_setup_acknowledge(struct dss1_lite_call_desc *cd)
{
	uint8_t send_chan_id = dss1_lite_is_nt_mode(cd);
	uint8_t send_progress = dss1_lite_want_dialtone(cd);

	return (dss1_lite_send_message(cd, 0x0d /* SETUP_ACKNOWLEDGE */ ,
	    (IE_HEADER_MASK | (send_progress ? IE_PROGRESS_MASK : 0) |
	    (send_chan_id ? IE_CHANNEL_ID_MASK : 0))));
}

static uint8_t
dss1_lite_send_alert(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x01 /* ALERT */ , IE_HEADER_MASK));
}

/*
 * NOTE: If one sends the channel-ID to some PBXs while in TE-mode,
 * and the call is incoming, the PBX might reject the message and send
 * an error to the "up-link"
 */

static uint8_t
dss1_lite_send_call_proceeding(struct dss1_lite_call_desc *cd)
{
	uint8_t send_chan_id = dss1_lite_is_nt_mode(cd);
	uint8_t send_progress = dss1_lite_want_dialtone(cd);

	return (dss1_lite_send_message(cd, 0x02 /* CALL_PROCEEDING */ ,
	    IE_HEADER_MASK |
	    (send_progress ? IE_PROGRESS_MASK : 0) |
	    (send_chan_id ? IE_CHANNEL_ID_MASK : 0)));
}

static uint8_t
dss1_lite_send_information(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x7b /* INFORMATION */ ,
	    IE_HEADER_MASK | IE_CALLED_PARTY_MASK));
}

static uint8_t
dss1_lite_send_connect(struct dss1_lite_call_desc *cd)
{
	uint8_t send_date_time = dss1_lite_is_nt_mode(cd);

	return (dss1_lite_send_message(cd, 0x07 /* CONNECT */ ,
	    IE_HEADER_MASK | (send_date_time ? IE_DATE_TIME_MASK : 0)));
}

static uint8_t
dss1_lite_send_connect_acknowledge(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x0f /* CONNECT_ACKNOWLEDGE */ ,
	    IE_HEADER_MASK));
}

static uint8_t
dss1_lite_send_disconnect(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x45 /* DISCONNECT */ ,
	    IE_HEADER_MASK | IE_CAUSE_MASK));
}

#if 0
static uint8_t
dss1_lite_send_release(struct dss1_lite_call_desc *cd, uint8_t send_cause_flag)
{
	return (dss1_lite_send_message(cd, 0x4d /* RELEASE */ ,
	    send_cause_flag ? IE_HEADER_MASK | IE_CAUSE_MASK : IE_HEADER_MASK));
}

#endif

/*
 * NOTE: Some ISDN phones require the "cause" information element when
 * sending RELEASE_COMPLETE
 */
static uint8_t
dss1_lite_send_release_complete(struct dss1_lite_call_desc *cd, uint8_t send_cause_flag)
{
	return (dss1_lite_send_message(cd, 0x5a /* RELEASE_COMPLETE */ ,
	    send_cause_flag ? (IE_HEADER_MASK | IE_CAUSE_MASK) : IE_HEADER_MASK));
}

static uint8_t
dss1_lite_send_status(struct dss1_lite_call_desc *cd, uint8_t q850cause)
{
	uint8_t retval;

	cd->dl_cause_out = q850cause;
	retval = dss1_lite_send_message(cd, 0x7d /* STATUS */ ,
	    IE_HEADER_MASK | IE_CAUSE_MASK | IE_CALLSTATE_MASK);
	cd->dl_cause_out = 0;
	return (retval);
}

static uint8_t
dss1_lite_send_status_enquiry(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x75 /* STATUS_ENQUIRY */ , IE_HEADER_MASK));
}

#if 0
static uint8_t
dss1_lite_send_progress(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x03 /* PROGRESS */ ,
	    IE_HEADER_MASK | IE_PROGRESS_MASK));
}

#endif

#if 0
static uint8_t
dss1_lite_send_hold(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x24 /* HOLD */ ,
	    IE_HEADER_MASK));
}

#endif

#if 0
static uint8_t
dss1_lite_send_hold_acknowledge(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x28 /* HOLD_ACKNOWLEDGE */ , IE_HEADER_MASK));
}

#endif

#if 0
static uint8_t
dss1_lite_send_hold_reject(struct dss1_lite_call_desc *cd, uint8_t q850cause)
{
	uint8_t retval;

	cd->dl_cause_out = q850cause;
	retval = dss1_lite_send_message(cd, 0x30 /* HOLD_REJECT */ ,
	    IE_HEADER_MASK | IE_CAUSE_MASK);
	cd->dl_cause_out = 0;
	return (retval);
}

#endif

#if 0
static uint8_t
dss1_lite_send_retrieve(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x31 /* RETRIEVE */ ,
	    IE_HEADER_MASK));
}

#endif

#if 0
static uint8_t
dss1_lite_send_retrieve_acknowledge(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x33 /* RETRIEVE_ACKNOWLEDGE */ ,
	    IE_HEADER_MASK | IE_CHANNEL_ID_MASK));
}

#endif

#if 0
static uint8_t
dss1_lite_send_retrieve_reject(struct dss1_lite_call_desc *cd, uint8_t q850cause)
{
	uint8_t retval;

	cd->dl_cause_out = q850cause;
	retval = dss1_lite_send_message(cd, 0x37 /* RETRIEVE_REJECT */ ,
	    IE_HEADER_MASK | IE_CAUSE_MASK);
	cd->dl_cause_out = 0;
	return (retval);
}

#endif

static uint8_t
dss1_lite_send_deflect_call(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x62 /* FACILITY */ ,
	    IE_HEADER_MASK | IE_DEFLECT_MASK));
}

static uint8_t
dss1_lite_send_mcid_call(struct dss1_lite_call_desc *cd)
{
	return (dss1_lite_send_message(cd, 0x62 /* FACILITY */ ,
	    IE_HEADER_MASK | IE_MCID_MASK));
}

static void
dss1_lite_string_filter(char *ptr)
{
	char temp;

	while (1) {
		temp = *ptr;
		if (temp == 0)
			break;

		/*
		 * If you don't understand why this filter is here, then you
		 * should never write PBX software
		 */
		if (((temp >= 'a') && (temp <= 'z')) ||
		    ((temp >= 'A') && (temp <= 'Z')) ||
		    ((temp >= '0') && (temp <= '9')) ||
		    (temp == '*') || (temp == '#')) {

			/* allow */

		} else {
			/* disallow */

			*ptr = '_';
		}
		ptr++;
	}
}

static uint8_t
dss1_lite_peek_mbuf(struct mbuf *m, uint16_t offset)
{
	if (offset >= m->m_len)
		return (0);
	else
		return (((uint8_t *)m->m_data)[offset]);
}

static void
dss1_lite_poke_mbuf(struct mbuf *m, uint16_t offset, uint8_t data)
{
	if (offset < m->m_len)
		((uint8_t *)m->m_data)[offset] = data;
}

static uint8_t
dss1_lite_has_call(struct dss1_lite_call_desc *cd)
{
	return (cd->dl_pstate != NULL);
}

static uint8_t
dss1_lite_has_status_check(struct dss1_lite_call_desc *cd)
{
	return (cd->dl_pstate->flags & 0x01);
}

static uint8_t
dss1_lite_is_incoming(struct dss1_lite_call_desc *cd)
{
	return ((cd->dl_state_index & DL_ST_IN_U0) ? 1 : 0);
}

static uint8_t
dss1_lite_get_state(struct dss1_lite_call_desc *cd)
{
	/* return aligned state */
	return (cd->dl_state_index & -2U);
}

static uint8_t
dss1_lite_get_timeout(struct dss1_lite_call_desc *cd)
{
	/* return remainder state */
	return (cd->dl_state_index & 1U);
}

static uint8_t
dss1_lite_set_state(struct dss1_lite_call_desc *cd, uint8_t state)
{
	const struct dss1_lite_state *nst;

	if (state == DL_ST_FREE) {
		/* call complete */
		if (!dss1_lite_has_call(cd)) {
			return (0);	/* invalid state */
		}
		nst = NULL;
	} else if (state >= DL_ST_IN_MAX) {
		return (0);		/* invalid state */
	} else if (state >= DL_ST_IN_U0) {
		if (dss1_lite_has_call(cd)) {
			/* check for outgoing call */
			if (!dss1_lite_is_incoming(cd))
				return (0);	/* invalid state */
			/* state cannot go backwards except for timeout */
			if ((cd->dl_state_index & -2U) > (state & -2U))
				return (0);	/* invalid state */
		}
		nst = &dss1_lite_in_states[0 - DL_ST_IN_U0 + state];
	} else if (state >= DL_ST_OUT_MAX) {
		return (0);		/* invalid state */
	} else {
		if (dss1_lite_has_call(cd)) {
			/* check for incoming call */
			if (dss1_lite_is_incoming(cd))
				return (0);	/* invalid state */
			/* state cannot go backwards except for timeout */
			if ((cd->dl_state_index & -2U) > (state & -2U))
				return (0);	/* invalid state */
		}
		nst = &dss1_lite_out_states[0 - DL_ST_OUT_U0 + state];
	}

	cd->dl_timeout_active = 0;
	cd->dl_timeout_tick = 0;

	cd->dl_pstate = nst;
	cd->dl_state_index = state;

	if (nst != NULL) {
		if (nst->timeout != 0) {
			cd->dl_timeout_active = 1;
			cd->dl_timeout_tick = ticks + (nst->timeout * hz);
		}
		if (dss1_lite_has_status_check(cd)) {
			dss1_lite_send_status_enquiry(cd);
		}
	} else {
		if (cd->dl_need_release) {
			dss1_lite_send_release_complete(cd, 1);
		}
		if (dss1_lite_is_nt_mode(cd)) {
			dss1_lite_set_ring(cd, 0);
		}
		if (cd->dl_parent->dl_active_call_desc == cd)
			cd->dl_parent->dl_active_call_desc = NULL;
	}
	return (1);			/* valid state transition */
}

static void
dss1_lite_receive_status(struct dss1_lite_call_desc *cd, uint8_t state)
{
	uint8_t next_state;

	if (dss1_lite_has_status_check(cd) == 0)
		return;

	next_state = dss1_lite_get_state(cd);

	switch (next_state) {
	case DL_ST_OUT_UA:
	case DL_ST_IN_UA:
		if (state == 0x0A) {
			dss1_lite_set_state(cd, next_state);
		}
		break;

	case DL_ST_OUT_UC:
	case DL_ST_IN_UC:
		if ((state == 0x0B) || (state == 0x0C)) {
			dss1_lite_set_state(cd, next_state);
		}
		break;

	default:
		/*
		 * NOTE: there are more callstates than
		 * 0x0..0xA, 0x19 ...
		 */
		if (((state == 0x19) ||
		    ((state >= 0x01) && (state <= 0x09))) &&
		    (cd->dl_status_count < DL_STATUS_COUNT_MAX)) {
			cd->dl_status_count++;
			dss1_lite_set_state(cd, next_state);
		}
		break;
	}
}

static uint8_t
dss1_lite_send_ctrl(struct dss1_lite *pdl, uint8_t sapi, uint8_t cntl)
{
	struct mbuf *m;
	uint8_t *ptr;
	uint8_t len = (cntl & 0x02) ? /* U-frame */ 3 : /* S-frame */ 4;

	m = i4b_getmbuf(len, M_NOWAIT);
	if (m == NULL)
		return (0);

	ptr = m->m_data;

	ptr[0] = sapi;
	ptr[1] = DL_TEI;
	ptr[2] = cntl;

	if (len >= 4) {
		ptr[3] = pdl->dl_rx_num;
		if (sapi & 0x02) {	/* CR-RESPONSE */
			ptr[3] |= 1;	/* F == 1 */
		} else {		/* CR-COMMAND */
			/* P == 0, with exception of I-frames */
		}
	}
	return (dss1_lite_send_mbuf(pdl, m));
}

static uint16_t
dss1_lite_find_ie_mbuf(struct mbuf *m, uint16_t offset, uint8_t ie)
{
	uint8_t temp;

	while (1) {
		/* get IE type byte */
		temp = dss1_lite_peek_mbuf(m, offset);
		if (temp == 0)
			break;

		/* check IE value */
		if (temp == ie)
			return (offset);

		/* skip IE byte */
		offset++;

		/* check for single byte element */
		if (temp & 0x80)
			continue;

		/* get length byte */
		temp = dss1_lite_peek_mbuf(m, offset);

		/* advance to next element */
		offset += temp + 1;
	}
	return (0);			/* the end */
}

static void
dss1_lite_decode_channel_id(struct dss1_lite_call_desc *cd,
    struct mbuf *m, uint16_t offset)
{
	uint8_t temp;
	uint8_t exclusive;
	uint8_t iid;

	if (cd->dl_channel_allocated)
		return;

	temp = dss1_lite_peek_mbuf(m, offset + 2);

	/*
	 * Exclusive is set if no other channel is acceptable:
	 */
	exclusive = (temp & 0x08) ||
	    (dss1_lite_is_nt_mode(cd) == 0);

	/*
	 * Interface identifier is set if there is an interface
	 * identifier byte:
	 */
	iid = (temp & 0x40) ? 1 : 0;

	if (temp & 0x20) {
		/* Primary Rate */
		if (temp & 0x04) {
			if (exclusive)
				cd->dl_channel_id = CHAN_D1;
		} else {
			switch (temp & 0x03) {
			case 0:
				cd->dl_channel_id = CHAN_NOT_ANY;
				break;
			case 1:
				/* time-slot follows */
				temp = dss1_lite_peek_mbuf(m, offset + iid + 3);

				if ((temp & 0x2F) == 0x03) {

					temp = dss1_lite_peek_mbuf(m, offset + iid + 4);

					if (exclusive) {
						int channel_id = (temp & 0x7F);

						L1_COMMAND_REQ(dss1_lite_get_ctrl(cd),
						    CMR_DECODE_CHANNEL, &channel_id);

						cd->dl_channel_id = channel_id;
					}
				}
				break;
			case 3:
				cd->dl_channel_id = CHAN_ANY;
				break;
			default:
				/* not supported */
				break;
			}
		}
	} else {
		/* Basic Rate */
		if (temp & 0x04) {
			if (exclusive)
				cd->dl_channel_id = CHAN_D1;
		} else {

			switch (temp & 0x03) {
			case IE_CHAN_ID_NO:
				cd->dl_channel_id = CHAN_NOT_ANY;
				break;
			case IE_CHAN_ID_ANY:
				cd->dl_channel_id = CHAN_ANY;
				break;
			case IE_CHAN_ID_B1:
				if (exclusive)
					cd->dl_channel_id = CHAN_B1;
				break;
			default:
				if (exclusive)
					cd->dl_channel_id = CHAN_B2;
				break;
			}

		}
	}
}

static void
dss1_lite_decode_bearer_cap(struct dss1_lite_call_desc *cd,
    struct mbuf *m, uint16_t offset)
{
	uint8_t temp;

	if (dss1_lite_is_incoming(cd) == 0)
		return;			/* invalid */

	temp = dss1_lite_peek_mbuf(m, offset + 2);

	switch (temp) {
	case 0x88:			/* unrestricted digital info */
		cd->dl_channel_bprot = BPROT_RHDLC;
		break;
	default:
		cd->dl_channel_bprot = BPROT_NONE;
		break;
	}

	temp = dss1_lite_peek_mbuf(m, offset + 4) & 0x1F;

	switch (temp) {
	case 0x02:
		cd->dl_channel_bsubprot = BSUBPROT_G711_ULAW;
		break;

	case 0x03:
		cd->dl_channel_bsubprot = BSUBPROT_G711_ALAW;
		break;
	default:
		cd->dl_channel_bsubprot = BSUBPROT_UNKNOWN;
		break;
	}
}

static void
dss1_lite_decode_called_party(struct dss1_lite_call_desc *cd,
    struct mbuf *m, uint16_t offset)
{
	uint8_t temp;
	uint8_t len;
	uint8_t i;

	len = dss1_lite_peek_mbuf(m, offset + 1);
	temp = dss1_lite_peek_mbuf(m, offset + 2);

	/* type of number (destination) */
	switch ((temp & 0x70) >> 4) {
	case 1:
		cd->dl_part.ton = TON_INTERNAT;
		break;
	case 2:
		cd->dl_part.ton = TON_NATIONAL;
		break;
	default:
		cd->dl_part.ton = TON_OTHER;
		break;
	}

	cd->dl_part.subaddr[0] = 0;
	cd->dl_part.prs_ind = PRS_ALLOWED;

	if (len != 0)
		len--;

	if (len > (sizeof(cd->dl_part.telno) - 1))
		len = (sizeof(cd->dl_part.telno) - 1);

	for (i = 0; i != len; i++) {
		temp = dss1_lite_peek_mbuf(m, offset + 3 + i);
		if (temp == 0)
			temp = '_';
		cd->dl_part.telno[i] = temp;
	}

	cd->dl_part.telno[i] = 0;

	dss1_lite_string_filter(cd->dl_part.telno);
}

static struct dss1_lite_call_desc *
dss1_lite_find_cd(struct dss1_lite *pdl, uint32_t crval)
{
	struct dss1_lite_call_desc *cd;
	uint8_t i;

	for (i = 0; i != DL_NCALL; i++) {

		cd = pdl->dl_cd + i;

		if (dss1_lite_has_call(cd) &&
		    (cd->dl_curr_callref == crval))
			return (cd);
	}
	return (NULL);
}

static struct dss1_lite_call_desc *
dss1_lite_alloc_cd(struct dss1_lite *pdl, uint8_t is_incoming,
    uint32_t crval)
{
	struct dss1_lite_call_desc *cd;
	uint8_t i;

	if (pdl->dl_active_call_desc != NULL)
		return (NULL);

	for (i = 0;; i++) {

		if (i == DL_NCALL)
			return (NULL);

		cd = pdl->dl_cd + i;

		if (dss1_lite_has_call(cd) == 0)
			break;
	}

	memset(cd, 0, sizeof(*cd));

	if (is_incoming == 0) {
		/* outgoing call */

		crval = pdl->dl_next_callref | 0x01;

		pdl->dl_next_callref += 0x10;
		pdl->dl_next_callref &= 0x7F;

	} else {
		/* incoming call */

	}

	cd->dl_parent = pdl;
	cd->dl_channel_id = CHAN_ANY;
	cd->dl_channel_bprot = BPROT_NONE;
	cd->dl_curr_callref = crval;

	if (is_incoming == 0) {
		dss1_lite_set_state(cd, DL_ST_OUT_U2);
	} else {
		dss1_lite_set_state(cd, DL_ST_IN_U0);
	}

	if (pdl->dl_active_call_desc == NULL)
		pdl->dl_active_call_desc = cd;

	return (cd);
}

static void
dss1_lite_alloc_channel(struct dss1_lite_call_desc *cd)
{
	struct i4b_controller *cntl = dss1_lite_get_ctrl(cd);
	int id;
	int end;

	id = cd->dl_channel_id;
	end = cntl->L1_channel_end;

	if (cd->dl_channel_allocated == 0) {

		if ((id >= 0) && (id < end)) {
			if (GET_CHANNEL_UTILIZATION(cntl, id) == 0) {
				goto found;
			} else {
				goto not_found;
			}
		} else {
			if (id == CHAN_NOT_ANY) {
				goto not_any;
			} else {
				for (id = 0; id < end; id++) {
					if (GET_CHANNEL_UTILIZATION(cntl, id) == 0) {
						goto found;
					}
				}
		not_found:
				id = CHAN_ANY;
				goto done;
		found:
				SET_CHANNEL_UTILIZATION(cntl, id, 1);
		not_any:
				cd->dl_channel_allocated = 1;
				goto done;
			}
		}
	}
done:
	cd->dl_channel_id = id;
}

static void
dss1_lite_decode_mbuf(struct dss1_lite *pdl, struct mbuf *m)
{
	struct dss1_lite_call_desc *cd;
	uint16_t offset;

	uint8_t tei;
	uint8_t sapi;
	uint8_t cntl;
	uint8_t tx_nr;
	uint8_t proto;
	uint8_t reflen;
	uint8_t callref;
	uint8_t event;
	uint8_t temp;

	sapi = dss1_lite_peek_mbuf(m, 0);
	tei = dss1_lite_peek_mbuf(m, 1);
	cntl = dss1_lite_peek_mbuf(m, 2);
	tx_nr = dss1_lite_peek_mbuf(m, 3);
	proto = dss1_lite_peek_mbuf(m, 4);
	reflen = dss1_lite_peek_mbuf(m, 5);
	callref = dss1_lite_peek_mbuf(m, 6);
	event = dss1_lite_peek_mbuf(m, 7);

	if (sapi & 0xFC)
		goto done;

	if (tei != DL_TEI)
		goto done;

	if (pdl->dl_is_nt_mode == 0)
		sapi ^= 0x02;		/* CR-bit is inverse */

	if (cntl & 0x01) {
		/* S- and U- frames */
		if (!(cntl & 0x02))
			goto check_nr;	/* S-frame */
		else
			goto done;
	} else {
		/* I-frame */
		if (proto != PD_Q931)
			goto done;
		if (reflen != 0x01)
			goto done;

		if (pdl->dl_rx_num == cntl) {
			pdl->dl_rx_num += 2;
			if (tx_nr & 1) {/* P=1 */
				dss1_lite_send_ctrl(cd->dl_parent, 0x02 /* F=1 */ , 0x01 /* RR */ );
			} else if (pdl->dl_tx_in == pdl->dl_tx_out) {
				dss1_lite_send_ctrl(cd->dl_parent, 0x00 /* F=0 */ , 0x01 /* RR */ );
			} else {
				/* RX-NR will be sent by an I-frame */
			}
			goto i_frame;
		} else {
			dss1_lite_send_ctrl(cd->dl_parent, 0 /* F=0 */ , 0x09 /* REJ */ );
			goto check_nr;
		}
	}

i_frame:

	if (event == 0x05) {

		/* SETUP */

		if (~callref & 0x80)	/* not originating */
			goto check_nr;
		if ((callref & ~0x80) == 0x00)	/* global call reference */
			goto check_nr;

		cd = dss1_lite_alloc_cd(pdl, 1, callref);
		if (cd == NULL)
			goto check_nr;

	} else {
		/* other message */

		cd = dss1_lite_find_cd(pdl, callref);
		if (cd == NULL)
			goto check_nr;
	}

	/* try to pickup sending complete */
	offset = dss1_lite_find_ie_mbuf(m, 0x08, IEI_SENDCOMPL);
	if (offset != 0) {
		cd->dl_sending_complete = 1;
	}
	/* try to pickup the channel ID */
	offset = dss1_lite_find_ie_mbuf(m, 0x08, IEI_CHANNELID);
	if (offset != 0) {
		dss1_lite_decode_channel_id(cd, m, offset);
	}
	/* try to pickup the bearer capability */
	offset = dss1_lite_find_ie_mbuf(m, 0x08, IEI_BEARERCAP);
	if (offset != 0) {
		dss1_lite_decode_bearer_cap(cd, m, offset);
	}
	/* try to pickup the destination number */
	offset = dss1_lite_find_ie_mbuf(m, 0x08, IEI_CALLEDPN);
	if (offset != 0) {
		dss1_lite_decode_called_party(cd, m, offset);
	} else {
		cd->dl_part.telno[0] = 0;
		cd->dl_part.subaddr[0] = 0;
		cd->dl_part.ton = TON_OTHER;
		cd->dl_part.prs_ind = PRS_ALLOWED;
	}

	switch (event) {
	case 0x01:			/* ALERT */
		temp = (dss1_lite_get_state(cd) < DL_ST_OUT_U3);
		if (dss1_lite_set_state(cd, DL_ST_OUT_U4) != 0) {
			if (temp) {
				/*
				 * try to allocate channel before proceeding
				 * indication
				 */
				if (cd->dl_channel_id != CHAN_ANY) {
					dss1_lite_alloc_channel(cd);
				}
				/* proceeding_indication() */
			}
			/* alert_indication() */
		}
		break;

	case 0x0d:			/* SETUP ACKNOWLEDGE */
		if (dss1_lite_get_state(cd) == DL_ST_OUT_U2) {
			if ((dss1_lite_get_timeout(cd) == 0) &&
			    (dss1_lite_set_state(cd, DL_ST_OUT_U2_ACK) != 0)) {
				/*
				 * try to allocate channel before proceeding
				 * indication
				 */
				if (cd->dl_channel_id != CHAN_ANY) {
					dss1_lite_alloc_channel(cd);
				}
				/* proceeding_indication() */
			}
			break;
		}
		/* FALLTHROUGH */
	case 0x02:			/* CALL PROCEEDING */
		if (dss1_lite_set_state(cd, DL_ST_OUT_U3)) {
			/*
			 * try to allocate channel before proceeding
			 * indication
			 */
			if (cd->dl_channel_id != CHAN_ANY) {
				dss1_lite_alloc_channel(cd);
			}
			/* proceeding_indication() */
		}
		break;

	case 0x03:			/* PROGRESS */
		break;

	case 0x05:			/* SETUP */
		cd->dl_need_release = 1;

		dss1_lite_alloc_channel(cd);

		if (cd->dl_channel_allocated == 0) {
	no_channel_available:
			/* no circuit-channel available */
			cd->dl_cause_out = CAUSE_Q850_NOCAVAIL;
			dss1_lite_set_state(cd, DL_ST_FREE);
			break;
		}
		if (dss1_lite_is_nt_mode(cd)) {
			dss1_lite_set_ring(cd, 1);
			dss1_lite_alert_request(cd);
		} else {
			if (cd->dl_part.telno[0])
				dss1_lite_set_dtmf(cd, cd->dl_part.telno);

			if (cd->dl_sending_complete) {
				if (dss1_lite_set_state(cd, DL_ST_IN_U6))
					dss1_lite_send_call_proceeding(cd);
			} else {
				if (dss1_lite_set_state(cd, DL_ST_IN_U0_ACK))
					dss1_lite_send_setup_acknowledge(cd);
			}
		}
		break;

	case 0x07:			/* CONNECT */
		if ((dss1_lite_get_state(cd) != DL_ST_OUT_UA) &&
		    (dss1_lite_set_state(cd, DL_ST_OUT_UA) != 0)) {

			if (cd->dl_channel_id != CHAN_ANY) {

				dss1_lite_alloc_channel(cd);

				if (cd->dl_channel_allocated) {
					/* we are connected */

					dss1_lite_send_connect_acknowledge(cd);

					/* connect_active() indication */

					break;
				}
			}
			/*
		         * If the one end doesn't select a channel, the
		         * other end must select a channel else there is an
		         * error!
		         */
			goto no_channel_available;
		}
		break;

	case 0x0f:			/* CONNECT ACKNOWLEDGE */
		if ((dss1_lite_get_state(cd) == DL_ST_IN_U8) &&
		    (dss1_lite_set_state(cd, DL_ST_IN_UA) != 0)) {
			/* we are connected */

			/* connect_active() indication */
		}
		break;

	case 0x45:			/* DISCONNECT */
		if (dss1_lite_is_incoming(cd)) {
			if ((dss1_lite_get_state(cd) != DL_ST_IN_UC) &&
			    (dss1_lite_set_state(cd, DL_ST_IN_UC) != 0)) {
				/* we are disconnected */
			}
		} else {
			if ((dss1_lite_get_state(cd) != DL_ST_OUT_UC) &&
			    (dss1_lite_set_state(cd, DL_ST_OUT_UC) != 0)) {
				/* we are disconnected */
			}
		}
		break;

	case 0x4d:			/* RELEASE */
		cd->dl_cause_out = CAUSE_Q850_NORMUNSP;
		dss1_lite_set_state(cd, DL_ST_FREE);
		break;

	case 0x5a:			/* RELEASE COMPLETE */
		cd->dl_need_release = 0;
		dss1_lite_set_state(cd, DL_ST_FREE);
		break;

	case 0x75:			/* STATUS_ENQUIRY */
		dss1_lite_send_status(cd, CAUSE_Q850_STENQRSP);
		break;

	case 0x7b:			/* INFORMATION */
		if ((dss1_lite_get_state(cd) == DL_ST_IN_U0) &&
		    (dss1_lite_set_state(cd, DL_ST_IN_U0_ACK) != 0)) {
			if ((dss1_lite_is_nt_mode(cd) == 0) && (cd->dl_part.telno[0] != 0)) {
				dss1_lite_set_dtmf(cd, cd->dl_part.telno);
			}
		}
		break;

	case 0x7d:			/* STATUS */
		offset = dss1_lite_find_ie_mbuf(m, 0x08, 0x14 /* callstate */ );
		if (offset != 0) {
			temp = dss1_lite_peek_mbuf(m, offset + 2);
			dss1_lite_receive_status(cd, temp & 0x3F);
		}
		break;

	default:
		break;
	}

check_nr:
	temp = tx_nr & 0xFE;

	/* get NR difference */
	tx_nr = (tx_nr - pdl->dl_tx_num) / 2;

	/* take new NR */
	pdl->dl_tx_num = temp;

	if (tx_nr != 0) {
		if (tx_nr > pdl->dl_tx_window) {
			/* retransmit all */
			pdl->dl_tx_window = 0;
		} else {
			/* decrement TX window */
			pdl->dl_tx_window -= tx_nr;

			/* valid NR event */
			while (tx_nr--) {
				struct mbuf *m2;
				uint8_t n;

				/* frame is transmitted - free it */
				n = pdl->dl_tx_in & (DL_QUEUE_MAX - 1);

				m2 = pdl->dl_tx_mbuf[n];
				pdl->dl_tx_mbuf[n] = NULL;
				m_freem(m2);

				pdl->dl_tx_in++;
			}
		}
	}
	/*
	 * Check if a response is needed:
	 *
	 * (CR_COMMAND) and (P == 1) and (S-frame)
	 */
	if ((~sapi & 0x02) && (cntl & tx_nr & 0x01)) {
		dss1_lite_send_ctrl(pdl, 0x02 /* F=1 */ , 0x01 /* RR */ );
	}
done:
	m_freem(m);
}

static uint8_t
dss1_lite_proceed_request(struct dss1_lite_call_desc *cd)
{
	uint8_t retval;

	if ((dss1_lite_get_state(cd) != DL_ST_IN_U6) &&
	    (dss1_lite_set_state(cd, DL_ST_IN_U6) != 0)) {
		retval = dss1_lite_send_call_proceeding(cd);
	} else {
		retval = 0;
	}
	return (retval);
}

static uint8_t
dss1_lite_alert_request(struct dss1_lite_call_desc *cd)
{
	uint8_t retval;

	dss1_lite_proceed_request(cd);

	if ((dss1_lite_get_state(cd) != DL_ST_IN_U7) &&
	    (dss1_lite_set_state(cd, DL_ST_IN_U7) != 0)) {
		retval = dss1_lite_send_alert(cd);
	} else {
		retval = 0;
	}
	return (retval);
}

uint8_t
dss1_lite_deflect_request(struct dss1_lite *pdl)
{
	struct dss1_lite_call_desc *cd;

	if (pdl->dl_is_nt_mode == 0)
		return (0);		/* wrong mode */

	cd = pdl->dl_active_call_desc;
	if (cd == NULL)
		return (0);

	return (dss1_lite_send_deflect_call(cd));
}

uint8_t
dss1_lite_mcid_request(struct dss1_lite *pdl)
{
	struct dss1_lite_call_desc *cd;

	if (pdl->dl_is_nt_mode == 0)
		return (0);		/* wrong mode */

	cd = pdl->dl_active_call_desc;
	if (cd == NULL)
		return (0);

	return (dss1_lite_send_mcid_call(cd));
}

static uint8_t
dss1_lite_setup_accept_resp(struct dss1_lite_call_desc *cd)
{
	uint8_t retval;

	dss1_lite_proceed_request(cd);

	if ((dss1_lite_get_state(cd) != DL_ST_IN_U8) &&
	    (dss1_lite_set_state(cd, DL_ST_IN_U8) != 0)) {

		if (dss1_lite_is_nt_mode(cd)) {
			dss1_lite_set_ring(cd, 0);
		}
		dss1_lite_send_connect(cd);

		cd->dl_cause_out = 0;
		cd->dl_need_release = 1;

		if (dss1_lite_is_nt_mode(cd)) {
			/*
			 * NOTE: some terminals might not respond with
			 * CONNECT ACKNOWLEDGE after CONNECT
			 */
			dss1_lite_set_state(cd, DL_ST_IN_UA);
		}
		retval = 1;
	} else {
		retval = 0;
	}
	return (retval);
}

uint8_t
dss1_lite_disconnect_request(struct dss1_lite *pdl)
{
	struct dss1_lite_call_desc *cd;
	uint8_t retval;

	if (pdl->dl_is_nt_mode == 0)
		return (0);		/* wrong mode */

	cd = pdl->dl_active_call_desc;
	if (cd == NULL)
		return (0);

	dss1_lite_proceed_request(cd);

	if (dss1_lite_is_incoming(cd)) {
		if ((dss1_lite_get_state(cd) != DL_ST_IN_UC) &&
		    (dss1_lite_set_state(cd, DL_ST_IN_UC) != 0)) {

			retval = dss1_lite_send_disconnect(cd);

		} else {
			retval = 0;
		}
	} else {
		if ((dss1_lite_get_state(cd) != DL_ST_OUT_UC) &&
		    (dss1_lite_set_state(cd, DL_ST_OUT_UC) != 0)) {

			retval = dss1_lite_send_disconnect(cd);
		} else {
			retval = 0;
		}
	}
	return (retval);
}

uint8_t
dss1_lite_ring_event(struct dss1_lite *pdl, uint8_t ison)
{
	struct dss1_lite_call_desc *cd;

	if (pdl->dl_is_nt_mode != 0)
		return (0);		/* wrong mode */

	cd = pdl->dl_active_call_desc;
	if (cd == NULL)
		return (0);		/* no active call descriptor */

	return (0);
}

uint8_t
dss1_lite_hook_off(struct dss1_lite *pdl)
{
	struct dss1_lite_call_desc *cd;
	uint8_t retval;

	if (pdl->dl_is_nt_mode == 0)
		return (0);		/* wrong mode */

	cd = pdl->dl_active_call_desc;
	if (cd == NULL) {
		cd = dss1_lite_alloc_cd(pdl, 0, 0);
		if (cd == NULL)
			return (0);

		retval = dss1_lite_send_setup(cd, 0);

		cd->dl_need_release = 1;
	} else {
		retval = dss1_lite_setup_accept_resp(cd);
	}
	return (retval);
}

uint8_t
dss1_lite_hook_on(struct dss1_lite *pdl)
{
	struct dss1_lite_call_desc *cd;
	uint8_t retval;

	if (pdl->dl_is_nt_mode == 0)
		return (0);		/* wrong mode */

	cd = pdl->dl_active_call_desc;
	if (cd == NULL)
		return (0);		/* no active call descriptor */

	cd->dl_cause_out = CAUSE_Q850_NORMUNSP;
	retval = dss1_lite_set_state(cd, DL_ST_FREE);

	return (retval);
}

uint8_t
dss1_lite_r_key_event(struct dss1_lite *pdl)
{
	struct dss1_lite_call_desc *cd;

	cd = pdl->dl_active_call_desc;
	if (cd == NULL)
		return (0);		/* no active call descriptor */

	return (0);
}

uint8_t
dss1_lite_dtmf_event(struct dss1_lite *pdl, const char *pdtmf)
{
	struct dss1_lite_call_desc *cd;
	uint8_t retval;

	if (pdl->dl_is_nt_mode == 0)
		return (0);		/* wrong mode */

	cd = pdl->dl_active_call_desc;
	if (cd == NULL)
		return (0);		/* no active call descriptor */

	if ((dss1_lite_has_call(cd) != 0) &&
	    (dss1_lite_get_state(cd) == DL_ST_OUT_U2)) {

		strlcpy(cd->dl_part.telno, pdtmf, sizeof(cd->dl_part.telno));

		retval = dss1_lite_send_information(cd);
	} else {
		retval = 0;
	}
	return (retval);
}

static uint8_t
dss1_lite_queue_mbuf(struct dss1_lite *pdl, struct mbuf *m)
{
	uint8_t retval;

	if (pdl->dl_is_nt_mode) {
		/* CR-bit is inverse */
		dss1_lite_poke_mbuf(m, 0,
		    dss1_lite_peek_mbuf(m, 0) ^ 0x02);
	}
	if (_IF_QFULL(&pdl->dl_outq)) {
		m_freem(m);
		retval = 0;
	} else {
		_IF_ENQUEUE(&pdl->dl_outq, m);
		retval = 1;
	}
	return (retval);
}

static void
dss1_lite_peer_put_mbuf(struct dss1_lite *pdl)
{
	struct mbuf *m;
	int delta;
	int len;

	len = 0;

	while (1) {
		_IF_DEQUEUE(&pdl->dl_outq, m);
		if (m == NULL)
			break;
		len += m->m_len;

		dss1_lite_l5_put_mbuf(pdl, pdl->dl_fifo + CHAN_D1, m);
	}

	delta = ((len * hz) / DL_BPS_MAX);
	if (delta < 0)
		delta = 0;
	else if (delta > (8 * hz))
		delta = (8 * hz);

	pdl->dl_tx_end_tick = ticks + delta;
}

static void
dss1_lite_peer_get_mbuf(struct dss1_lite *pdl)
{
	struct mbuf *m;

	while (1) {
		m = dss1_lite_l5_get_mbuf(pdl, pdl->dl_fifo + CHAN_D1);
		if (m == NULL)
			break;

		if (m->m_next) {
			/* defrag by need */
			struct mbuf *m_new;
			uint16_t len;

			m_new = m;
			len = 0;
			do {
				len += m_new->m_len;
			} while ((m_new = m_new->m_next));

			m_new = i4b_getmbuf(len, M_NOWAIT);
			if (m_new == NULL) {
				m_freem(m);
				continue;
			}
			m_copydata(m, 0, len, m_new->m_data);
			m_freem(m);
			m = m_new;
		}
		dss1_lite_decode_mbuf(pdl, m);
	}
}

static void
dss1_lite_check_timeout(struct dss1_lite_call_desc *cd)
{
	int delta;

	if (cd->dl_timeout_active) {
		delta = cd->dl_timeout_tick - ticks;
		if ((delta < 0) || (delta > (128 * hz))) {
			dss1_lite_set_state(cd, cd->dl_pstate->next_state);
		}
	}
}

void
dss1_lite_process(struct dss1_lite *pdl)
{
	enum {
		WIN_MAX = DL_QUEUE_MAX / 2,
	};
	int delta;
	uint8_t i;
	uint8_t n;
	uint8_t rx_num;

	if (pdl->dl_fifo[CHAN_D1].prot_curr.protocol_1 == P_DISABLE)
		return;

	dss1_lite_peer_get_mbuf(pdl);

	dss1_lite_check_timeout(pdl->dl_cd);

	delta = pdl->dl_tx_end_tick - ticks;

	if ((delta >= 0) && (delta < (128 * hz))) {
		/* limit TX througput */
		return;
	}
	delta = (pdl->dl_tx_out - pdl->dl_tx_in) & (DL_QUEUE_MAX - 1);
	if (delta > WIN_MAX)
		delta = WIN_MAX;

	pdl->dl_tx_window = delta;

	for (n = 0; n != delta; n++) {
		struct mbuf *m;

		i = (pdl->dl_tx_in + n) & (DL_QUEUE_MAX - 1);

		m = pdl->dl_tx_mbuf[i];

		if (m != NULL)
			m = m_dup(m, M_NOWAIT);

		if (m != NULL) {

			rx_num = pdl->dl_rx_num;

			/*
			 * The remote end must transmit a response on
			 * every I-frame with (P == 1)
			 */
			if (n == (delta - 1))
				rx_num |= 0x01;	/* P bit == 1 */

			dss1_lite_poke_mbuf(m, 0, 0x00 /* CR-COMMAND */ );
			dss1_lite_poke_mbuf(m, 1, DL_TEI);
			dss1_lite_poke_mbuf(m, 2, pdl->dl_tx_num + (2 * n));
			dss1_lite_poke_mbuf(m, 3, rx_num);

			if (dss1_lite_queue_mbuf(pdl, m) == 0)
				break;	/* queue is full */
		}
	}

	dss1_lite_peer_put_mbuf(pdl);
}

static uint8_t
dss1_lite_send_mbuf(struct dss1_lite *pdl, struct mbuf *m)
{
	uint8_t delta;
	uint8_t retval;
	uint8_t n;

	if (!(dss1_lite_peek_mbuf(m, 2) & 0x01)) {	/* I-frame */
		delta = (pdl->dl_tx_out - pdl->dl_tx_in) & (DL_QUEUE_MAX - 1);
		if (delta < (DL_QUEUE_MAX - 1)) {
			n = pdl->dl_tx_out & (DL_QUEUE_MAX - 1);
			pdl->dl_tx_mbuf[n] = m;
			pdl->dl_tx_out++;
			retval = 1;
		} else {
			m_freem(m);
			retval = 0;
		}
	} else {
		retval = dss1_lite_queue_mbuf(pdl, m);
	}

	return (retval);
}

static uint8_t
dss1_lite_ie_header(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	if (ptr != NULL) {

		*ptr++ = 0;
		*ptr++ = 0;
		*ptr++ = 0;		/* I-frame */
		*ptr++ = 0;

		*ptr++ = PD_Q931;
		*ptr++ = 1;		/* callref length */
		*ptr++ = cd->dl_curr_callref;
		*ptr++ = cd->dl_message_type;
	}
	return (8);			/* bytes */
}

static uint8_t
dss1_lite_ie_sending_complete(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_SENDCOMPL;
	}
	return (1);			/* bytes */
}

static uint8_t
dss1_lite_ie_bearer_cap(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	if (ptr != NULL) {

		*ptr++ = IEI_BEARERCAP;	/* bearer capability */

		switch (cd->dl_channel_bprot) {
		case BPROT_NONE:	/* telephony */
			*ptr++ = 3;
			*ptr++ = IT_CAP_SPEECH;
			*ptr++ = IT_RATE_64K;
			switch (cd->dl_channel_bsubprot) {
			case BSUBPROT_G711_ALAW:
				*ptr++ = IT_UL1_G711A;
				break;
			case BSUBPROT_G711_ULAW:
				*ptr++ = IT_UL1_G711U;
				break;
			default:
				*ptr++ = 0xA0;	/* reserved */
				break;
			}
			break;

		case BPROT_RHDLC:	/* HDLC */
		default:
			*ptr++ = 2;
			*ptr++ = IT_CAP_UNR_DIG_INFO;
			*ptr++ = IT_RATE_64K;
			break;
		}
	}
	switch (cd->dl_channel_bprot) {
	case BPROT_NONE:		/* telephony */
		return (5);		/* bytes */
	case BPROT_RHDLC:		/* HDLC */
	default:
		return (4);		/* bytes */
	}
}

static uint8_t
dss1_lite_ie_channel_id(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	enum {
		/* only accept this channel */
		EXT_EXCLUSIVE = EXT_LAST | 0x08,
	};

	if (!cd->dl_channel_allocated)
		return (0);		/* bytes */

	if (ptr != NULL) {

		*ptr++ = IEI_CHANNELID;	/* channel id */
		*ptr++ = 1;

		switch (cd->dl_channel_id) {
		case CHAN_NOT_ANY:
			*ptr = IE_CHAN_ID_NO | EXT_EXCLUSIVE;
			break;
		case CHAN_D1:
			*ptr = 0x04 | EXT_EXCLUSIVE;
			break;
		case CHAN_B1:
			*ptr = IE_CHAN_ID_B1 | EXT_EXCLUSIVE;
			break;
		case CHAN_B2:
			*ptr = IE_CHAN_ID_B2 | EXT_EXCLUSIVE;
			break;
		default:
			*ptr = IE_CHAN_ID_ANY | EXT_EXCLUSIVE;
			break;
		}
	}
	return (3);			/* bytes */
}

static uint8_t
dss1_lite_ie_keypad(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	const char *str;
	int len;

	str = cd->dl_keypad;
	len = strlen(str);

	if ((len <= 0) || (len >= 128))
		return (0);		/* bytes */

	if (ptr != NULL) {
		*ptr++ = IEI_KEYPAD;	/* keypad facility */
		*ptr++ = len;		/* keypad facility length */
		memcpy(ptr, str, len);
	}
	return (len + 2);		/* bytes */
}

static uint8_t
dss1_lite_ie_calling_party_sub(struct dss1_lite_call_desc *cd, uint8_t *ptr, uint8_t index)
{
	const char *str;
	int len;
	uint8_t ton;
	uint8_t prs_ind;

	str = cd->dl_src[index].telno;
	prs_ind = cd->dl_src[index].prs_ind;
	ton = cd->dl_src[index].ton;
	len = strlen(str);

	ton = (ton == TON_INTERNAT) ? (NUMBER_TYPE_PLAN | 0x10) :
	    (ton == TON_NATIONAL) ? (NUMBER_TYPE_PLAN | 0x20) :
	    NUMBER_TYPE_PLAN;

	if ((len <= 0) || (len >= 128))
		return (0);		/* bytes */

	if (ptr != NULL) {

		/* type of number, number plan id */

		if (prs_ind != PRS_NONE) {

			/* presentation indicator */

			*ptr++ = IEI_CALLINGPN;	/* calling party no */
			*ptr++ = len + 2;

			/* clear extension bit */
			*ptr++ = ton & ~0x80;
			*ptr++ = (prs_ind == PRS_RESTRICT) ?
			    (0x20 | 0x80) : (0x80);

		} else {

			*ptr++ = IEI_CALLINGPN;	/* calling party no */
			*ptr++ = len + 1;
			*ptr++ = ton;
		}

		memcpy(ptr, str, len);
	}
	if (prs_ind != PRS_NONE)
		return (len + 4);	/* bytes */
	else
		return (len + 3);	/* bytes */
}

static uint8_t
dss1_lite_ie_calling_party0(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	return (dss1_lite_ie_calling_party_sub(cd, ptr, 0));
}

static uint8_t
dss1_lite_ie_calling_party1(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	return (dss1_lite_ie_calling_party_sub(cd, ptr, 1));
}

static uint8_t
dss1_lite_ie_calling_subaddr_sub(struct dss1_lite_call_desc *cd, uint8_t *ptr, uint8_t index)
{
	const char *str;
	int len;

	str = cd->dl_src[index].subaddr;
	len = strlen(str);

	if ((len <= 0) || (len >= 128))
		return (0);		/* bytes */

	if (ptr != NULL) {

		*ptr++ = IEI_CALLINGPS;	/* calling subaddr */
		*ptr++ = 1 + len;
		*ptr++ = NUMBER_TYPE_NSAP;	/* type = NSAP */

		memcpy(ptr, str, len);
	}
	return (len + 3);		/* bytes */
}

static uint8_t
dss1_lite_ie_calling_subaddr0(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	return (dss1_lite_ie_calling_subaddr_sub(cd, ptr, 0));
}

static uint8_t
dss1_lite_ie_calling_subaddr1(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	return (dss1_lite_ie_calling_subaddr_sub(cd, ptr, 1));
}

static uint8_t
dss1_lite_ie_useruser(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	const char *str;
	int len;

	str = cd->dl_useruser;
	len = strlen(str);

	if ((len <= 0) || (len >= 128))
		return (0);		/* bytes */

	if (ptr != NULL) {

		*ptr++ = IEI_USERUSER;	/* user-user */
		*ptr++ = len;		/* user-user length */

		memcpy(ptr, str, len);
	}
	return (len + 2);		/* bytes */
}

static uint8_t
dss1_lite_ie_display(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	const char *str;
	int len;

	str = cd->dl_display;
	len = strlen(str);

	if ((len <= 0) || (len >= 128))
		return (0);		/* bytes */

	if (ptr != NULL) {

		*ptr++ = IEI_DISPLAY;	/* display */
		*ptr++ = len;		/* display length */

		memcpy(ptr, str, len);
	}
	return (len + 2);		/* bytes */
}

static uint8_t
dss1_lite_ie_cause(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_CAUSE;	/* cause ie */
		*ptr++ = 2;
		*ptr++ = dss1_lite_is_nt_mode(cd) ?
		    CAUSE_STD_LOC_PUBLIC : CAUSE_STD_LOC_OUT;
		*ptr++ = cd->dl_cause_out | EXT_LAST;
	}
	return (4);			/* bytes */
}

static uint8_t
dss1_lite_ie_callstate(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_CALLSTATE;	/* call state ie */
		*ptr++ = 1;
		*ptr++ = cd->dl_pstate->q931_state;
	}
	return (3);			/* bytes */
}

static uint8_t
dss1_lite_ie_progress(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_PROGRESSI;
		*ptr++ = 2;		/* bytes */
		*ptr++ = dss1_lite_is_nt_mode(cd) ?
		    CAUSE_STD_LOC_PUBLIC : CAUSE_STD_LOC_OUT;
		*ptr++ = 0x88;		/* in-band info available */
	}
	return (4);			/* bytes */
}

static uint8_t
dss1_lite_ie_called_party(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	const char *str;
	int len;

	str = cd->dl_part.telno;
	len = strlen(str);

	if ((len <= 0) || (len >= 128))
		return (0);		/* bytes */

	if (ptr != NULL) {

		*ptr++ = IEI_CALLEDPN;	/* calling party subaddr */
		*ptr++ = 1 + len;
		*ptr++ = NUMBER_TYPE_PLAN;	/* type = NSAP */

		memcpy(ptr, str, len);
	}
	return (len + 3);		/* bytes */
}

static uint8_t
dss1_lite_ie_called_subaddr(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	const char *str;
	int len;

	str = cd->dl_part.subaddr;
	len = strlen(str);

	if ((len <= 0) || (len >= 128))
		return (0);		/* bytes */

	if (ptr != NULL) {

		*ptr++ = IEI_CALLEDPS;	/* calling party subaddr */
		*ptr++ = 1 + len;
		*ptr++ = NUMBER_TYPE_NSAP;	/* type = NSAP */

		memcpy(ptr, str, len);
	}
	return (len + 3);		/* bytes */
}

static uint8_t
dss1_lite_ie_deflect(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	const char *str;
	int len;

	str = cd->dl_part.telno;
	len = strlen(str);

	if ((len <= 0) || (len >= 128))
		return (0);		/* bytes */

	if (ptr != NULL) {
		*ptr++ = IEI_FACILITY;	/* Facility IE */
		*ptr++ = 0x10 + len;	/* Length */
		*ptr++ = 0x91;		/* Remote Operations Protocol */
		*ptr++ = 0xa1;		/* Tag: context specific */

		*ptr++ = 0x0D + len;	/* Length */
		*ptr++ = 0x02;		/* Tag: Universal, Primitive, INTEGER */
		*ptr++ = 0x02;		/* Length */
		*ptr++ = 0x22;		/* Data: Invoke Identifier = 34 */

		*ptr++ = 0x00;		/* Data */
		*ptr++ = 0x02;		/* Tag: Universal, Primitive, INTEGER */
		*ptr++ = 0x01;		/* Length */
		*ptr++ = 0x0d;		/* Data: Operation Value = Call Defl. */

		*ptr++ = 0x30;		/* Tag: Universal, Constructor, SEQ. */
		*ptr++ = 0x04 + len;	/* Length */
		*ptr++ = 0x30;		/* Tag: Universal, Constructor, SEQ. */
		*ptr++ = 0x02 + len;	/* Length */

		*ptr++ = 0x80;		/* Tag: Context specific, Prim., c=0 */
		*ptr++ = 0x00 + len;

		memcpy(ptr, str, len);
	}
	return (0x12 + len);		/* bytes */
}

static uint8_t
dss1_lite_ie_mcid(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_FACILITY;	/* Facility IE */
		*ptr++ = 0x0a;		/* Length */
		*ptr++ = 0x91;		/* Remote Operations Protocol */
		*ptr++ = 0xa1;		/* Tag: Context-specific */

		*ptr++ = 0x07;		/* Length */
		*ptr++ = 0x02;		/* Tag: Universal, Primitive, INTEGER */
		*ptr++ = 0x02;		/* Length */
		*ptr++ = 0x22;		/* Data: Invoke Identifier = 34 */

		*ptr++ = 0x00;		/* Data */
		*ptr++ = 0x02;		/* Tag: Universal, Primitive, INTEGER */
		*ptr++ = 0x01;		/* Length */
		*ptr++ = 0x03;		/* Data: Operation Value = MCID
					 * request */
	}
	return (0x0C);			/* bytes */
}

static uint8_t
dss1_lite_ie_date(struct dss1_lite_call_desc *cd, uint8_t *ptr)
{
	uint8_t len;

	len = cd->dl_otime.len;
	if (len > 8)
		len = 8;
	if (len == 0)
		return (0);		/* bytes */

	if (ptr != NULL) {
		*ptr++ = IEI_DATETIME;	/* Date/Time IE */
		*ptr++ = len;
		memcpy(ptr, cd->dl_otime.data, len);
	}
	return (2 + len);		/* bytes */
}

static const struct dss1_lite_ie_func dss1_lite_ie_tab[] = {
	{&dss1_lite_ie_header, IE_HEADER_MASK},
	{&dss1_lite_ie_sending_complete, IE_SENDING_COMPLETE_MASK},
	{&dss1_lite_ie_bearer_cap, IE_BEARER_CAP_MASK},
	{&dss1_lite_ie_channel_id, IE_CHANNEL_ID_MASK},
	{&dss1_lite_ie_keypad, IE_KEYPAD_MASK},

	{&dss1_lite_ie_calling_party0, IE_CALLING_PARTY_MASK},
	{&dss1_lite_ie_calling_subaddr0, IE_CALLING_PARTY_MASK},
	{&dss1_lite_ie_calling_party1, IE_CALLING_PARTY_MASK},
	{&dss1_lite_ie_calling_subaddr1, IE_CALLING_PARTY_MASK},

	{&dss1_lite_ie_useruser, IE_USERUSER_MASK},
	{&dss1_lite_ie_display, IE_DISPLAY_MASK},
	{&dss1_lite_ie_cause, IE_CAUSE_MASK},
	{&dss1_lite_ie_callstate, IE_CALLSTATE_MASK},

	{&dss1_lite_ie_progress, IE_PROGRESS_MASK},
	{&dss1_lite_ie_called_party, IE_CALLED_PARTY_MASK},
	{&dss1_lite_ie_called_subaddr, IE_CALLED_PARTY_MASK},
	{&dss1_lite_ie_deflect, IE_DEFLECT_MASK},

	{&dss1_lite_ie_mcid, IE_MCID_MASK},
	{&dss1_lite_ie_date, IE_DATE_TIME_MASK},

	{NULL, 0},
};


static uint8_t
dss1_lite_send_message(struct dss1_lite_call_desc *cd, uint8_t msg_type, uint32_t mask)
{
	struct mbuf *m;
	uint8_t *ptr;
	uint16_t len;
	uint8_t i;

	len = 0;

	cd->dl_message_type = msg_type;

	for (i = 0; dss1_lite_ie_tab[i].pfunc; i++) {
		if (dss1_lite_ie_tab[i].mask & mask)
			len += (dss1_lite_ie_tab[i].pfunc) (cd, NULL);
	}

	if (len == 0)
		return (1);

	m = i4b_getmbuf(len, M_NOWAIT);
	if (m == NULL)
		return (0);

	ptr = m->m_data;

	for (i = 0; dss1_lite_ie_tab[i].pfunc; i++) {
		if (dss1_lite_ie_tab[i].mask & mask)
			ptr += (dss1_lite_ie_tab[i].pfunc) (cd, ptr);
	}

	return (dss1_lite_send_mbuf(cd->dl_parent, m));
}
