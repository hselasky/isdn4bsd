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
 *---------------------------------------------------------------------------*/

#include <sys/stdint.h>
#include <sys/callout.h>
#include <sys/mutex.h>

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

void
dss1_lite_has_call(struct dss1_lite *pst)
{
	return (pst->dl_pstate != NULL);
}

void
dss1_lite_has_status_check(struct dss1_lite *pst)
{
	return (pst->dl_pstate->flags & 0x01);
}

void
dss1_lite_is_incoming(struct dss1_lite *pst)
{
	return ((pst->dl_state_index & DL_ST_IN_U0) ? 1 : 0);
}

uint8_t
dss1_lite_get_state(struct dss1_lite *pst)
{
	/* return aligned state */
	return (pst->dl_state_index & -2U);
}

uint8_t
dss1_lite_set_state(struct dss1_lite *pst, uint8_t state)
{
	struct dss1_lite_state *nst;

	if (state == DL_ST_FREE) {
		/* call complete */
		if (!dss1_lite_has_call(pst)) {
			return (0);	/* invalid state */
		}
		nst = NULL;
	} else if (state >= DL_ST_IN_MAX) {
		return (0);		/* invalid state */
	} else if (state >= DL_ST_IN_U0) {
		if (dss1_lite_has_call(pst)) {
			/* check for outgoing call */
			if (!dss1_lite_is_incoming(pst))
				return (0);	/* invalid state */
			/* state cannot go backwards except for timeout */
			if ((pst->dl_state_index & -2U) > (state & -2U))
				return (0);	/* invalid state */
		}
		nst = &dss1_lite_in_states[0 - DL_ST_IN_U0 + state];
	} else if (state >= DL_ST_OUT_MAX) {
		return (0);		/* invalid state */
	} else {
		if (dss1_lite_has_call(pst)) {
			/* check for incoming call */
			if (dss1_lite_is_incoming(pst))
				return (0);	/* invalid state */
			/* state cannot go backwards except for timeout */
			if ((pst->dl_state_index & -2U) > (state & -2U))
				return (0);	/* invalid state */
		}
		nst = &dss1_lite_out_states[0 - DL_ST_OUT_U0 + state];
	}

	pst->dl_timeout_active = 0;
	pst->dl_timeout_tick = 0;

	pst->dl_pstate = nst;
	pst->dl_state_index = state;

	if (nst != NULL) {
		if (nst->timeout != 0) {
			pst->dl_timeout_active = 1;
			pst->dl_timeout_tick = ticks + (nst->timeout * hz);
		}
		if (dss1_lite_has_status_check(pst)) {
			dss1_lite_send_status_enquiry(pst);
		}
	} else {
		if (pst->dl_no_rc == 0) {
			dss1_lite_send_release_complete(pst);
		} else {
			pst->dl_no_rc = 0;
		}
		if (pst->dl_ringing != 0) {
			pst->dl_ringing = 0;
			pst->dl_methods->set_ring(pst, 0);
		}
		pst->dl_status_count = 0;
	}
	return (1);			/* valid state transition */
}

void
dss1_lite_receive_status(struct dss1_lite *pst, uint8_t state)
{
	uint8_t next_state;

	if (dss1_lite_has_call(pst) &&
	    dss1_lite_has_status_check(pst)) {

		next_state = dss1_lite_get_state(pst);

		switch (next_state) {
		case DL_ST_OUT_UA:
		case DL_ST_IN_UA:
			if (state == 0x0A) {
				dss1_lite_set_state(pst, next_state);
			}
			break;

		case DL_ST_OUT_UC:
		case DL_ST_IN_UC:
			if ((state == 0x0B) || (state == 0x0C)) {
				dss1_lite_set_state(pst, next_state);
			}
			break;

		default:
			/*
			 * NOTE: there are more callstates than
			 * 0x0..0xA, 0x19 ...
			 */
			if (((state == 0x19) ||
			    ((state >= 0x01) && (state <= 0x09))) &&
			    (pst->dl_status_count < DL_STATUS_COUNT_MAX)) {
				pst->dl_status_count++;
				dss1_lite_set_state(pst, next_state);
			}
			break;
		}
	}
}

uint8_t
dss1_lite_send_ctrl(struct dss1_lite *pst, uint8_t sapi, uint8_t cntl)
{
	struct mbuf *m;
	uint8_t *ptr;
	uint8_t len = (cntl & 2) ? /* U-frame */ 3 : /* S-frame */ 4;

	m = dss1_lite_getmbuf(len);
	if (m == NULL)
		return (0);

	ptr = m->m_data;

	ptr[0] = sapi;
	ptr[1] = DL_TEI;
	ptr[2] = cntl;

	if (len >= 4) {
		ptr[3] = pst->dl_rx_num;
		if (sapi & 0x02) {	/* CR-RESPONSE */
			ptr[3] |= 1;	/* F == 1 */
		} else {		/* CR-COMMAND */
			/* P == 0, with exception of I-frames */
		}
	}
	return (dss1_lite_send_mbuf(pst, m));
}

uint8_t
dss1_lite_peek_mbuf(struct mbuf *m, uint16_t offset)
{
	if (offset >= m->m_len)
		return (0);
	else
		return (((uint8_t *)m->m_data)[offset]);
}

void
dss1_lite_poke_mbuf(struct mbuf *m, uint16_t offset, uint8_t data)
{
	if (offset < m->m_len)
		((uint8_t *)m->m_data)[offset] = data;
}

uint16_t
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

void
dss1_lite_receive_mbuf(struct dss1_lite *pst, struct mbuf *m)
{
	uint16_t offset;

	uint8_t tei;
	uint8_t sapi;
	uint8_t cntl;
	uint8_t rx_nr;
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

	if (cntl & 0x01) {
		/* S- and U- frames */
		if (!(cntl & 0x02))
			goto check_nr;	/* S-frame */
		else
			goto done;
	} else {
		/* I-frame */
		if (proto != DL_PD_Q931)
			goto done;
		if (reflen != 0x01)
			goto done;

		if (pst->dl_rx_num == cntl) {
			pst->dl_rx_num += 2;
			if (tx_nr & 1) {/* P=1 */
				dss1_lite_send_ctrl(pst, 0x02 /* F=1 */ , 0x01 /* RR */ );
			} else if (pst->dl_tx_in == pst->dl_tx_out) {
				dss1_lite_send_ctrl(pst, 0x00 /* F=0 */ , 0x01 /* RR */ );
			} else {
				/* RX-NR will be sent by an I-frame */
			}
			goto i_frame;
		} else {
			dss1_lite_send_ctrl(pst, 0 /* F=0 */ , 0x09 /* REJ */ );
			goto check_nr;
		}
	}

i_frame:

	if (event == 0x05) {
		if (dss1_lite_has_call(pst) != 0)
			goto check_nr;
		pst->dl_curr_callref = callref;
	} else {
		if (dss1_lite_has_call(pst) == 0)
			goto check_nr;
	}

	if (callref != pst->dl_curr_callref)
		goto check_nr;

	if (pst->dl_is_nt_mode == 0) {
		/* try to pickup the channel ID */
		offset = dss1_lite_find_ie_mbuf(m, 0x08, 0x18 /* channel ID */ );
		if (offset != 0) {
			temp = dss1_lite_mbuf_peek(m, offset + 2);
			pst->dl_channel = temp & 0x03;
		}
	}
	switch (event) {
	case 0x01:			/* ALERT */
		break;
	case 0x02:			/* CALL PROCEEDING */
		break;
	case 0x03:			/* PROGRESS */
		break;
	case 0x05:			/* SETUP */
		if (dss1_lite_set_state(pst, DL_ST_IN_U0) != 0)
			dss1_lite_send_alerting(pst);
		break;
	case 0x07:			/* CONNECT */
		if ((dss1_lite_get_state(pst) != DL_ST_OUT_UA) &&
		    (dss1_lite_set_state(pst, DL_ST_OUT_UA) != 0)) {
			/* we are connected */
		}
		break;
	case 0x0d:			/* SETUP ACKNOWLEDGE */
		break;
	case 0x0f:			/* CONNECT ACKNOWLEDGE */
		if ((dss1_lite_get_state(pst) == DL_ST_IN_U8) &&
		    (dss1_lite_set_state(pst, DL_ST_IN_UA) != 0)) {
			/* we are connected */
		}
		break;
	case 0x45:			/* DISCONNECT */
		if (dss1_lite_is_incoming(pst)) {
			if ((dss1_lite_get_state(pst) != DL_ST_IN_UC) &&
			    (dss1_lite_set_state(pst, DL_ST_IN_UC) != 0)) {
				/* we are disconnected */
			}
		} else {
			if ((dss1_lite_get_state(pst) != DL_ST_OUT_UC) &&
			    (dss1_lite_set_state(pst, DL_ST_OUT_UC) != 0)) {
				/* we are disconnected */
			}
		}
		break;
	case 0x4d:			/* RELEASE */
		dss1_lite_set_state(pst, DL_ST_FREE);
		break;
	case 0x5a:			/* RELEASE COMPLETE */
		pst->dl_no_rc = 1;
		dss1_lite_set_state(pst, DL_ST_FREE);
		break;
	case 0x7b:			/* INFORMATION */
		break;
	case 0x7d:			/* STATUS */
		offset = dss1_lite_find_ie_mbuf(m, 0x08, 0x14 /* callstate */ );
		if (offset != 0) {
			temp = dss1_lite_mbuf_peek(m, offset + 2);
			dss1_lite_receive_status(pst, temp & 0x3F);
		}
		break;
	default:
		break;
	}

check_nr:
	if ((tx_nr & 0xFE) == pst->dl_tx_num) {
		/* idle state */
	} else if (((tx_nr - 2) & 0xFE) == pst->dl_tx_num) {
		if (pst->dl_ipending && (pst->dl_tx_in != pst->dl_tx_out)) {
			struct mbuf *m2;
			uint8_t n;

			/* frame is transmitted - free it */

			n = pst->dl_tx_in & (DL_WIN_MAX - 1);

			m2 = pst->dl_tx_mbuf[n];
			pst->dl_tx_mbuf[n] = NULL;
			m_freem(m2);

			pst->dl_ipending = 0;
			pst->dl_tx_in++;
		}
	}
	/* take new TX-NR */
	pst->dl_tx_num = tx_nr & 0xFE;

done:
	m_freem(m);
}

uint8_t
dss1_lite_send_setup(struct dss1_lite *pst)
{
	uint8_t retval;

	/* goto overlap sending state */

	if ((dss1_lite_has_call(pst) == 0) &&
	    (dss1_lite_set_state(pst, DL_ST_OUT_U2) != 0)) {
		send setup req to peer();

		retval = 1;
	} else {
		retval = 0;
	}
	return (retval);
}

uint8_t
dss1_lite_send_proceeding(struct dss1_lite *pst)
{
	uint8_t retval;

	if (dss1_lite_has_call(pst) &&
	    (dss1_lite_get_state(pst) != DL_ST_IN_U6) &&
	    (dss1_lite_set_state(pst, DL_ST_IN_U6) != 0)) {
		send proc req to peer();

		retval = 1;
	} else {
		retval = 0;
	}
	return (retval);
}

uint8_t
dss1_lite_send_alerting(struct dss1_lite *pst)
{
	uint8_t retval;

	dss1_lite_send_proceeding(pst);

	if (dss1_lite_has_call(pst) &&
	    (dss1_lite_get_state(pst) != DL_ST_IN_U7) &&
	    (dss1_lite_set_state(pst, DL_ST_IN_U7) != 0)) {

		pst->dl_methods->set_ring(pst, 1);
		pst->dl_ringing = 1;

		send alert req to peer();

		retval = 1;
	} else {
		retval = 0;
	}
	return (retval);
}

uint8_t
dss1_lite_send_accept(struct dss1_lite *pst)
{
	uint8_t retval;

	dss1_lite_send_proceeding(pst);

	if (dss1_lite_has_call(pst) &&
	    (dss1_lite_get_state(pst) != DL_ST_IN_U8) &&
	    (dss1_lite_set_state(pst, DL_ST_IN_U8) != 0)) {

		pst->dl_methods->set_ring(pst, 0);
		pst->dl_ringing = 0;

		send accept req to peer();

		retval = 1;
	} else {
		retval = 0;
	}
	return (retval);
}

uint8_t
dss1_lite_send_disconnect(struct dss1_lite *pst)
{
	uint8_t retval;

	dss1_lite_send_proceeding(pst);

	if (dss1_lite_has_call(pst)) {
		if (dss1_lite_is_incoming(pst)) {
			if ((dss1_lite_get_state(pst) != DL_ST_IN_UC) &&
			    (dss1_lite_set_state(pst, DL_ST_IN_UC) != 0)) {
				send disconnect req to peer();

				retval = 1;
			} else {
				retval = 0;
			}
		} else {
			if ((dss1_lite_get_state(pst) != DL_ST_OUT_UC) &&
			    (dss1_lite_set_state(pst, DL_ST_OUT_UC) != 0)) {
				send disconnect req to peer();

				retval = 1;
			} else {
				retval = 0;
			}
		}
	}
	return (retval);
}

uint8_t
dss1_lite_hook_off(struct dss1_lite *pst)
{
	uint8_t retval;

	if (dss1_lite_has_call(pst)) {
		retval = dss1_lite_send_accept(pst);
	} else {
		retval = dss1_lite_send_setup(pst);
	}
	return (retval);
}

uint8_t
dss1_lite_hook_on(struct dss1_lite *pst)
{
	uint8_t retval;

	retval = dss1_lite_set_state(pst, DL_ST_FREE);

	return (retval);
}

uint8_t
dss1_lite_r_key_event(struct dss1_lite *pst)
{
	return (0);
}

uint8_t
dss1_lite_dtmf_event(struct dss1_lite *pst, char dtmf)
{
	uint8_t retval;

	if ((dss1_lite_has_call(pst) != 0) &&
	    (dss1_lite_get_state(pst) == DL_ST_OUT_U2)) {
		send dtmf event to peer();

		retval = 1;
	} else {
		retval = 0;
	}
	return (retval);
}

void
dss1_lite_start_timer(struct dss1_lite *pst, uint8_t do_start)
{
	uint8_t llat;

	llat = (pst->dl_tx_in != pst->dl_tx_out) || (pst->dl_inq.ifq_len != 0);

	if ((pst->dl_llat == 0) && (llat != 0)) {
		callout_stop(&pst->dl_timer);
		do_start = 1;
	}
	if (do_start != 0) {
		pst->dl_llat = llat;
		callout_reset(&pst->dl_timer, llat ? (hz / 40) ? (2 * hz),
		    (void *)dss1_lite_event_timer, pst);
	}
}

uint8_t
dss1_lite_queue_mbuf(struct dss1_lite *pst, struct mbuf *m)
{
	uint8_t retval;

	if (_IF_QFULL(&pst->dl_outq)) {
		m_freem(m);
		retval = 0;
	} else {
		_IF_ENQUEUE(&pst->dl_outq, m);
		retval = 1;
	}
	return (retval);
}

void
dss1_lite_event_timer(struct dss1_lite *pst)
{
	int delta;
	uint8_t n;

	if (pst->dl_timeout_active) {
		delta = pst->dl_timeout_tick - ticks;
		if ((delta < 0) || (delta > (128 * hz))) {
			dss1_lite_set_state(pst, pst->pstate->next_state);
		}
	}
	if (pst->dl_tx_in != pst->dl_tx_out) {
		struct mbuf *m;

		pst->dl_ipending = 1;

		n = pst->dl_tx_in & (DL_WIN_MAX - 1);

		m = pst->dl_tx_mbuf[n];

		if (m != NULL)
			m = m_dupxxx(m);

		if (m != NULL) {
			dss1_lite_poke_mbuf(m, XXX, pst->dl_tx_num XXX);
			dss1_lite_queue_mbuf(pst, m);
		}
	}
	XXX signal DSS1 driver();

	dss1_lite_start_timer(pst, 1);
}

uint8_t
dss1_lite_send_mbuf(struct dss1_lite *pst, struct mbuf *m)
{
	uint8_t delta;
	uint8_t retval;
	uint8_t n;

	if (!(dss1_lite_peek_mbuf(m, 2) & 0x01)) {	/* I-frame */
		delta = (pst->dl_tx_out - pst->dl_tx_in) & (DL_WIN_MAX - 1);
		if (delta < (DL_WIN_MAX - 1)) {
			n = pst->dl_tx_out & (DL_WIN_MAX - 1);
			pst->dl_tx_mbuf[n] = m;
			pst->dl_tx_out++;
			retval = 1;
		} else {
			m_freem(m);
			retval = 0;
		}
	} else {
		retval = dss1_lite_queue_mbuf(pst, m);
	}

	dss1_lite_start_timer(pst, 0);

	return (retval);
}

uint8_t
dss1_lite_ie_header(struct dss1_lite *pst, uint8_t *ptr)
{
	if (ptr != NULL) {

		*ptr++ = 0;
		*ptr++ = 0;
		*ptr++ = 0;		/* I-frame */
		*ptr++ = 0;

		*ptr++ = DL_PD_Q931;
		*ptr++ = 1;		/* callref length */
		*ptr++ = pst->dl_callref;
		*ptr++ = pst->dl_message_type;
	}
	return (8);			/* bytes */
}

uint8_t
dss1_lite_ie_sending_complete(struct dss1_lite *pst, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_SENDCOMPL;
	}
	return (1);			/* bytes */
}

uint8_t
dss1_lite_ie_bearer_cap(struct dss1_lite *pst, uint8_t *ptr)
{
	if (ptr != NULL) {

		*ptr++ = IEI_BEARERCAP;	/* bearer capability */

		switch (cd->channel_bprot) {
		case BPROT_NONE:	/* telephony */
		case BPROT_RHDLC_DOV:	/* Data over Voice */
			*ptr++ = IEI_BEARERCAP_LEN + 1;
			*ptr++ = IT_CAP_SPEECH;
			*ptr++ = IT_RATE_64K;
			switch (cd->channel_bsubprot) {
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

		case BPROT_RHDLC:	/* raw HDLC */
		case BPROT_NONE_VOD:	/* Voice over Data */
		default:
			*ptr++ = IEI_BEARERCAP_LEN;
			*ptr++ = IT_CAP_UNR_DIG_INFO;
			*ptr++ = IT_RATE_64K;
			break;
		}
	}
	switch (cd->channel_bprot) {
	case BPROT_NONE:		/* telephony */
	case BPROT_RHDLC_DOV:		/* Data over Voice */
		return (5);		/* bytes */
	default:
		return (4);		/* bytes */
	}
}

uint8_t
dss1_lite_ie_channel_id(struct dss1_lite *pst, uint8_t *ptr)
{
	if (!cd->channel_allocated)
		return (0);		/* bytes */

	if (ptr != NULL) {

		/* only accept this channel */
		enum {
		EXT_EXCLUSIVE = 0x80 | 0x08};

		*ptr++ = IEI_CHANNELID;	/* channel id */
		*ptr++ = 1;

		switch (cd->channel_id) {
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

uint8_t
dss1_lite_ie_keypad(struct dss1_lite *pst, uint8_t *ptr)
{
	const char *str;
	int len;

	str = pst->dl_keypad;
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

uint8_t
dss1_lite_ie_calling_party_sub(struct dss1_lite *pst, uint8_t *ptr, uint8_t index)
{
	const char *str;
	int len;
	uint8_t ton;
	uint8_t prs_ind;

	str = pst->dl_src[index].telno;
	prs_ind = pst->dl_src[index].prs_ind;
	ton = pst->dl_src[index].ton;
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

uint8_t
dss1_lite_ie_calling_party0(struct dss1_lite *pst, uint8_t *ptr)
{
	return (dss1_lite_ie_calling_party_sub(pst, ptr, 0));
}

uint8_t
dss1_lite_ie_calling_party1(struct dss1_lite *pst, uint8_t *ptr)
{
	return (dss1_lite_ie_calling_party_sub(pst, ptr, 1));
}

uint8_t
dss1_lite_ie_calling_subaddr_sub(struct dss1_lite *pst, uint8_t *ptr, uint8_t index)
{
	const char *str;
	int len;

	str = pst->dl_src[index].subaddr;
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

uint8_t
dss1_lite_ie_calling_subaddr0(struct dss1_lite *pst, uint8_t *ptr)
{
	return (dss1_lite_ie_calling_subaddr_sub(pst, ptr, 0));
}

uint8_t
dss1_lite_ie_calling_subaddr1(struct dss1_lite *pst, uint8_t *ptr)
{
	return (dss1_lite_ie_calling_subaddr_sub(pst, ptr, 1));
}

uint8_t
dss1_lite_ie_useruser(struct dss1_lite *pst, uint8_t *ptr)
{
	const char *str;
	int len;

	str = pst->dl_useruser;
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

uint8_t
dss1_lite_ie_display(struct dss1_lite *pst, uint8_t *ptr)
{
	const char *str;
	int len;

	str = pst->dl_display;
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

uint8_t
dss1_lite_ie_cause(struct dss1_lite *pst, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_CAUSE;	/* cause ie */
		*ptr++ = 2;
		*ptr++ = NT_MODE(sc) ? CAUSE_STD_LOC_PUBLIC : CAUSE_STD_LOC_OUT;
		*ptr++ = i4b_make_q850_cause(cd->cause_out) | EXT_LAST;
	}
	return (4);			/* bytes */
}

uint8_t
dss1_lite_ie_callstate(struct dss1_lite *pst, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_CALLSTATE;	/* call state ie */
		*ptr++ = 1;
		*ptr++ = L3_STATES_Q931_CONV[cd->state];
	}
	return (3);			/* bytes */
}

uint8_t
dss1_lite_ie_progress(struct dss1_lite *pst, uint8_t *ptr)
{
	if (ptr != NULL) {
		*ptr++ = IEI_PROGRESSI;
		*ptr++ = 2;		/* bytes */
		*ptr++ = NT_MODE(sc) ? CAUSE_STD_LOC_PUBLIC : CAUSE_STD_LOC_OUT;
		*ptr++ = 0x88;		/* in-band info available */
	}
	return (4);			/* bytes */
}

uint8_t
dss1_lite_ie_called_party(struct dss1_lite *pst, uint8_t *ptr)
{
	const char *str;
	int len;

	str = pst->dl_part.telno;
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

uint8_t
dss1_lite_ie_called_subaddr(struct dss1_lite *pst, uint8_t *ptr)
{
	const char *str;
	int len;

	str = pst->dl_part.subaddr;
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

uint8_t
dss1_lite_ie_deflect(struct dss1_lite *pst, uint8_t *ptr)
{
	str = pst->dl_part.telno;
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

uint8_t
dss1_lite_ie_mcid(struct dss1_lite *pst, uint8_t *ptr)
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
	return (12);			/* bytes */
}

uint8_t
dss1_lite_ie_date(struct dss1_lite *pst, uint8_t *ptr)
{
	uint8_t len;

	len = pst->dl_otime.len;
	if (len > 8)
		len = 8;
	if (len == 0)
		return (0);		/* bytes */

	if (ptr != NULL) {
		*ptr++ = IEI_DATETIME;	/* Date/Time IE */
		*ptr++ = len;
		memcpy(ptr, pst->dl_otime.data, len);
	}
	return (2 + len);		/* bytes */
}

static const struct dss1_lite_ie_func dss1_lite_ie_tab[] = {
	{&dss1_lite_ie_header, DL_IE_HEADER_MASK},
	{&dss1_lite_ie_sending_complete, DL_IE_SENDING_COMPLETE_MASK},
	{&dss1_lite_ie_bearer_cap, DL_IE_BEARER_CAP_MASK},
	{&dss1_lite_ie_channel_id, DL_IE_CHANNEL_ID_MASK},
	{&dss1_lite_ie_keypad, DL_IE_KEYPAD_MASK},

	{&dss1_lite_ie_calling_party0, DL_IE_CALLING_PARTY_MASK},
	{&dss1_lite_ie_calling_subaddr0, DL_IE_CALLING_SUBADDR_MASK},
	{&dss1_lite_ie_calling_party1, DL_IE_CALLING_PARTY_MASK},
	{&dss1_lite_ie_calling_subaddr1, DL_IE_CALLING_SUBADDR_MASK},

	{&dss1_lite_ie_useruser, DL_IE_USERUSER_MASK},
	{&dss1_lite_ie_display, DL_IE_DISPLAY_MASK},
	{&dss1_lite_ie_cause, DL_IE_CAUSE_MASK},
	{&dss1_lite_ie_callstate, DL_IE_CALLSTATE_MASK},

	{&dss1_lite_ie_progress, DL_IE_PROGRESS_MASK},
	{&dss1_lite_ie_called_party, DL_IE_CALLED_PARTY_MASK},
	{&dss1_lite_ie_called_subaddr, DL_IE_CALLED_SUBADDR_MASK},
	{&dss1_lite_ie_deflect, DL_IE_DEFLECT_MASK},

	{&dss1_lite_ie_mcid, DL_IE_MCID_MASK},
	{&dss1_lite_ie_date, DL_IE_DATE_MASK},

	{NULL, 0},
};


uint8_t
dss1_lite_send_ie(struct dss1_lite *pst, uint8_t msg_type, uint32_t mask)
{
	uint8_t i;
	uint8_t *ptr;
	uint16_t len;

	len = 0;

	pst->dl_message_type = msg_type;

	for (i = 0; dss1_lite_ie_tab[i].pfunc; i++) {
		if (dss1_lite_ie_tab[i].mask & mask)
			len += (dss1_lite_ie_tab[i].pfunc) (pst, NULL);
	}

	if (len == 0)
		return (1);

	m = dss1_lite_getmbuf(len);
	if (m == NULL)
		return (0);

	ptr = m->m_data;

	for (i = 0; dss1_lite_ie_tab[i].pfunc; i++) {
		if (dss1_lite_ie_tab[i].mask & mask)
			ptr += (dss1_lite_ie_tab[i].pfunc) (pst, ptr);
	}

	return (dss1_lite_send_mbuf(pst, m));
}
