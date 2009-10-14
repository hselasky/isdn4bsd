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
 *	dss1_lite.c - terminal side only EuroISDN DSS1 implementation for
 *      non-digital phone interfaces
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
	[DL_ST_OUT_U0 - DL_ST_OUT_U0] = {0 /* hz */ , DL_ST_FREE, 0x00, 0x00},
	[DL_ST_OUT_U0_TO - DL_ST_OUT_U0] = {8 /* hz */ , DL_ST_FREE, 0x00, 0x00},	/* unused */
	[DL_ST_OUT_U1 - DL_ST_OUT_U0] = {16 /* hz */ , DL_ST_FREE, 0x01, 0x00},
	[DL_ST_OUT_U1_TO - DL_ST_OUT_U0] = {16 /* hz */ , DL_ST_FREE, 0x01, 0x00},	/* unused */
	[DL_ST_OUT_U2 - DL_ST_OUT_U0] = {16 /* hz */ , DL_ST_FREE, 0x02, 0x00},
	[DL_ST_OUT_U2_ACK - DL_ST_OUT_U0] = {16 /* hz */ , DL_ST_FREE, 0x02, 0x00},
	[DL_ST_OUT_U3 - DL_ST_OUT_U0] = {8 /* hz */ , DL_ST_OUT_U3_TO, 0x03, 0x00},
	[DL_ST_OUT_U3_TO - DL_ST_OUT_U0] = {4 /* hz */ , DL_ST_FREE, 0x03, 0x01},
	[DL_ST_OUT_U4 - DL_ST_OUT_U0] = {8 /* hz */ , DL_ST_OUT_U4_TO, 0x04, 0x00},
	[DL_ST_OUT_U4_TO - DL_ST_OUT_U0] = {4 /* hz */ , DL_ST_FREE, 0x04, 0x01},
	[DL_ST_OUT_UA - DL_ST_OUT_U0] = {8 /* hz */ , DL_ST_OUT_UA_TO, 0x0A, 0x00},
	[DL_ST_OUT_UA_TO - DL_ST_OUT_U0] = {4 /* hz */ , DL_ST_FREE, 0x0A, 0x01},
	[DL_ST_OUT_UC - DL_ST_OUT_U0] = {8 /* hz */ , DL_ST_OUT_UC_TO, 0x0C, 0x00},
	[DL_ST_OUT_UC_TO - DL_ST_OUT_U0] = {4 /* hz */ , DL_ST_FREE, 0x0C, 0x01},
};

static const struct dss1_lite_state dss1_lite_in_states[DL_ST_IN_MAX - DL_ST_IN_U0] = {
	[DL_ST_IN_U0 - DL_ST_IN_U0] = {8 /* hz */ , DL_ST_FREE, 0x00, 0x00},
	[DL_ST_IN_U0_ACK - DL_ST_IN_U0] = {16 /* hz */ , DL_ST_FREE, 0x19, 0x00},
	[DL_ST_IN_U6 - DL_ST_IN_U0] = {8 /* hz */ , DL_ST_IN_U6_TO, 0x06, 0x00},
	[DL_ST_IN_U6_TO - DL_ST_IN_U0] = {4 /* hz */ , DL_ST_FREE, 0x06, 0x01},
	[DL_ST_IN_U7 - DL_ST_IN_U0] = {8 /* hz */ , DL_ST_IN_U7_TO, 0x07, 0x00},
	[DL_ST_IN_U7_TO - DL_ST_IN_U0] = {4 /* hz */ , DL_ST_FREE, 0x07, 0x01},
	[DL_ST_IN_U8 - DL_ST_IN_U0] = {4 /* hz */ , DL_ST_FREE, 0x08, 0x00},
	[DL_ST_IN_U8_TO - DL_ST_IN_U0] = {4 /* hz */ , DL_ST_FREE, 0x08, 0x00},	/* unused */
	[DL_ST_IN_UA - DL_ST_IN_U0] = {8 /* hz */ , DL_ST_IN_UA_TO, 0x0A, 0x00},
	[DL_ST_IN_UA_TO - DL_ST_IN_U0] = {4 /* hz */ , DL_ST_FREE, 0x0A, 0x01},
	[DL_ST_IN_UC - DL_ST_IN_U0] = {8 /* hz */ , DL_ST_IN_UC_TO, 0x0C, 0x00},
	[DL_ST_IN_UC_TO - DL_ST_IN_U0] = {4 /* hz */ , DL_ST_FREE, 0x0C, 0x01},
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
		nst = &dss1_lite_in_states[state - DL_ST_IN_U0];
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
		nst = &dss1_lite_out_states[state - DL_ST_OUT_U0];
	}

	callout_stop(&pst->dl_timer);

	pst->dl_pstate = nst;
	pst->dl_state_index = state;

	if (nst != NULL) {
		if (nst->timeout != 0) {
			callout_reset(&pst->dl_timer, nst->timeout * hz,
			    (void *)dss1_lite_timeout_event, pst);
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

void
dss1_lite_rx_mbuf(struct dss1_lite *pst, struct mbuf *m)
{
	;				/* indent fix */
	switch (event) {
	case 0x01:			/* ALERT */
		break;
	case 0x02:			/* CALL PROCEEDING */
		break;
	case 0x03:			/* PROGRESS */
		break;
	case 0x05:			/* SETUP */
		break;
	case 0x07:			/* CONNECT */
		break;
	case 0x0d:			/* SETUP ACKNOWLEDGE */
		break;
	case 0x0f:			/* CONNECT ACKNOWLEDGE */
		break;
	case 0x45:			/* DISCONNECT */
		break;
	case 0x4d:			/* RELEASE */
		break;
	case 0x5a:			/* RELEASE COMPLETE */
		break;
	case 0x7b:			/* INFORMATION */
		break;
	case 0x7d:			/* STATUS */
		break;
	default:
		break;
	}
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
dss1_lite_receive_accept(struct dss1_lite *pst)
{
	uint8_t retval;

	if (dss1_lite_has_call(pst) != 0) {
		if (dss1_lite_is_incoming(pst)) {
			if ((dss1_lite_get_state(pst) == DL_ST_IN_U8) &&
			    (dss1_lite_set_state(pst, DL_ST_IN_UA) != 0)) {
				/* we are connected */
				retval = 1;
			} else {
				retval = 0;
			}
		} else {
			if ((dss1_lite_get_state(pst) != DL_ST_OUT_UA) &&
			    (dss1_lite_set_state(pst, DL_ST_OUT_UA) != 0)) {
				/* we are connected */
				retval = 1;
			} else {
				retval = 0;
			}
		}
	} else {
		retval = 0;
	}
	return (retval);
}

uint8_t
dss1_lite_receive_setup(struct dss1_lite *pst)
{
	uint8_t retval;

	/**/

	if ((dss1_lite_has_call(pst) == 0) &&
	    (dss1_lite_set_state(pst, DL_ST_IN_U0) != 0)) {
		retval = dss1_lite_send_alerting(pst);
	} else {
		retval = 0;
	}
	return (retval);



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
