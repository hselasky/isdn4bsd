/* $FreeBSD$ */
/*-
 * Copyright (c) 2000-2009 Hans Petter Selasky. All rights reserved.
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
 *	dss1_lite_l1if.c - layer 1 handler
 *	----------------------------------
 *
 *      This file defines the functions that are used
 *      to interface with ISDN4BSD
 *
 *---------------------------------------------------------------------------*/

#ifdef I4B_GLOBAL_INCLUDE_FILE
#include I4B_GLOBAL_INCLUDE_FILE
#else
#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/unistd.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#endif

#include <i4b/include/i4b_compat.h>
#include <i4b/include/i4b_controller.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/dss1/dss1_lite.h>

/*---------------------------------------------------------------------------*
 *	handle command from the upper layers
 *---------------------------------------------------------------------------*/
static int
dss1_lite_l1_ioctl(struct i4b_controller *cntl, int command, void *parm)
{
	struct dss1_lite *pdl = cntl->L1_sc;
	struct dss1_lite_fifo *f = cntl->L1_fifo;
#ifndef HAVE_NO_ECHO_CANCEL
	struct i4b_echo_cancel *ec_p;
#endif
	union {
		msg_ctrl_info_req_t *req;
		struct fifo_translator *ft;
		i4b_ec_debug_t *ec_dbg;
		i4b_debug_t *dbg;
		void   *parm;
	}     u;
#ifndef HAVE_NO_ECHO_CANCEL
	enum {
		EC_POINTS = (sizeof(ec_p->buf_HR[0]) /
		    sizeof(ec_p->buf_HR[0][0])),
		DBG_POINTS = (sizeof(u.ec_dbg->ydata) /
		    sizeof(u.ec_dbg->ydata[0])),
	};
	uint32_t points;
	uint32_t x;
#endif
	uint32_t temp;
	uint16_t n;

	u.parm = parm;

	switch (command) {
	case CMR_SETTRACE:
		/* set new trace mask */

		temp = *((uint32_t *)parm);

		/* clear trace bits first */

		for (n = 0; n != DL_NCHAN; n++) {
			(f + n)->is_tracing = 0;
		}

		/* set trace bits last */

		if (temp & TRACE_D_RX) {
			(f + 0)->is_tracing = 1;
		}
		if (temp & TRACE_B_RX) {
			for (n = 1; n != DL_NCHAN; n++) {
				(f + n)->is_tracing = 1;
			}
		}
		break;

	case CMR_RESET:
		/* NOP */
		break;

	case CMR_SET_I4B_OPTIONS:

		u.dbg->mask &= pdl->dl_option_mask;
		u.dbg->value &= pdl->dl_option_mask;

		temp = pdl->dl_option_value;
		temp &= ~(u.dbg->mask);
		temp |= u.dbg->value;

		pdl->dl_option_value = temp;
		break;

	case CMR_SET_L1_AUTO_ACTIVATE_VARIABLE:
		if (parm == NULL) {
			parm = &pdl->L1_auto_activate_variable;
			pdl->L1_auto_activate_variable = 0;
		}
		pdl->L1_auto_activate_ptr = parm;
		break;

	case CMR_SET_L1_ACTIVITY_VARIABLE:
		if (parm == NULL) {
			parm = &pdl->L1_activity_variable;
		}
		pdl->L1_activity_ptr = parm;

		/*
	         * Update L1_activity_variable
	         */
		*(pdl->L1_activity_ptr) = 1;
		break;

	case CMR_PH_ACTIVATE:
		/* NOP */
		break;

	case CMR_PH_DEACTIVATE:
		/* NOP */
		break;

	case CMR_SET_LAYER1_PROTOCOL:
		/* NOP */
		break;

	case CMR_INFO_REQUEST:
		snprintf(u.req->l1_desc, sizeof(u.req->l1_desc),
		    "DSS1-LITE device");

		snprintf(u.req->l1_state, sizeof(u.req->l1_state),
		    "L1-ACTIVE");

		u.req->l1_active = 1;
		break;

	case CMR_ENCODE_CHANNEL:
		/* NOP */
		break;

	case CMR_DECODE_CHANNEL:
		/* NOP */
		break;

	case CMR_SET_CHANNEL_MAPPING:
		/* NOP */
		break;

	case CMR_ENABLE_ECHO_CANCEL:

		f = u.ft->L1_fifo;

		if (PROT_IS_TRANSPARENT(&(f->prot_curr)) &&
		    pdl->dl_methods->support_echo_cancel) {
			f->prot_curr.u.transp.echo_cancel_enable = 1;
		} else {
			return (EINVAL);
		}
		break;

	case CMR_DISABLE_ECHO_CANCEL:

		f = u.ft->L1_fifo;

		if (PROT_IS_TRANSPARENT(&(f->prot_curr))) {
			f->prot_curr.u.transp.echo_cancel_enable = 0;
		} else {
			return (EINVAL);
		}
		break;

#ifndef HAVE_NO_ECHO_CANCEL
	case CMR_GET_EC_FIR_FILTER:

		if ((u.ec_dbg->chan >= cntl->L1_channel_end) ||
		    (u.ec_dbg->offset > EC_POINTS)) {
			return (EINVAL);
		}
		points = (EC_POINTS - u.ec_dbg->offset);
		if (points > DBG_POINTS) {
			points = DBG_POINTS;
		}
		f += u.ec_dbg->chan;

		if (PROT_IS_TRANSPARENT(&(f->prot_curr)) &&
		    f->prot_curr.u.transp.echo_cancel_enable) {
			ec_p = f->echo_cancel;
			u.ec_dbg->npoints = EC_POINTS;
			u.ec_dbg->decimal_point = I4B_ECHO_CANCEL_N_HR_DP;
			for (x = 0; x < points; x++) {
				u.ec_dbg->ydata[x] = ec_p->buf_HR[0][x + u.ec_dbg->offset];
			}
		} else {
			return (EINVAL);
		}
		break;
#endif
	case CMR_ENABLE_DTMF_DETECT:

		f = u.ft->L1_fifo;

		if (PROT_IS_TRANSPARENT(&(f->prot_curr))) {
			f->prot_curr.u.transp.dtmf_detect_enable = 1;
		} else {
			return (EINVAL);
		}
		break;

	case CMR_DISABLE_DTMF_DETECT:

		f = u.ft->L1_fifo;

		if (PROT_IS_TRANSPARENT(&(f->prot_curr))) {
			f->prot_curr.u.transp.dtmf_detect_enable = 0;
		} else {
			return (EINVAL);
		}
		break;

	default:
		break;
	}
	return (0);
}

/*---------------------------------------------------------------------------*
 *	trace routine
 *---------------------------------------------------------------------------*/
static void
dss1_lite_trace(struct dss1_lite *pdl, struct dss1_lite_fifo *f,
    int type, int is_transmit, struct mbuf *m)
{
	struct i4b_controller *cntl = pdl->dl_ctrl;
	uint8_t from_te = (pdl->dl_option_value & I4B_OPTION_NT_MODE) ? 0 : 1;

	if (type == TRC_CH_D) {
		type += f - pdl->dl_fifo;

		if (is_transmit) {
			from_te = !from_te;
		}
	}
	pdl->dl_trace_hdr.count++;
	pdl->dl_trace_hdr.unit = cntl->unit;
	pdl->dl_trace_hdr.type = type;
	pdl->dl_trace_hdr.dir = from_te ? FROM_TE : FROM_NT;

	microtime(&pdl->dl_trace_hdr.time);

	i4b_l1_trace_ind(&pdl->dl_trace_hdr, m);
}

/*---------------------------------------------------------------------------*
 *	trace info routine
 *---------------------------------------------------------------------------*/
void
dss1_lite_trace_info(struct dss1_lite *pdl,
    struct dss1_lite_fifo *f, const char *desc)
{
	int len = strlen(desc);
	struct mbuf *m = i4b_getmbuf(len, M_NOWAIT);

	if (m) {
		memcpy(mtod(m, char *), desc, len);

		dss1_lite_trace(pdl, f, TRC_CH_I, 1, m);

		m_freem(m);
	}
}

/*---------------------------------------------------------------------------*
 *	put sample to layer 5
 *---------------------------------------------------------------------------*/
void
dss1_lite_l5_put_sample(struct dss1_lite *pdl,
    struct dss1_lite_fifo *f, int32_t sample)
{
	uint8_t temp;

	f->m_tx_last_sample = sample;

	if (f->prot_curr.protocol_1 == P_DISABLE)
		return;

	while (f->m_tx_curr_rem == 0) {
		if (f->m_tx_curr != NULL) {
			dss1_lite_l5_put_mbuf(pdl, f, f->m_tx_curr);
		}
		f->m_tx_curr = dss1_lite_l5_get_new_mbuf(pdl, f);
		if (f->m_tx_curr == NULL)
			return;
		f->m_tx_curr_ptr = mtod(f->m_tx_curr, uint8_t *);
		f->m_tx_curr_rem = f->m_tx_curr->m_len;
	}

	/* XXX optimise */

	if (f->prot_curr.protocol_4 == BSUBPROT_G711_ALAW) {
		temp = i4b_signed_to_alaw(sample);
	} else {
		temp = i4b_signed_to_ulaw(sample);
	}
#ifndef HAVE_NO_ECHO_CANCEL
	if (f->prot_curr.u.transp.echo_cancel_enable)
		i4b_echo_cancel_merge(f->echo_cancel, &temp, 1);
#endif
	*(f->m_tx_curr_ptr)++ = temp;
	f->m_tx_curr_rem--;
}

void
dss1_lite_l5_put_sample_complete(struct dss1_lite *pdl,
    struct dss1_lite_fifo *f)
{
	if (f->prot_curr.protocol_1 == P_DISABLE)
		return;
#ifndef HAVE_NO_ECHO_CANCEL
	if (f->prot_curr.u.transp.echo_cancel_enable)
		i4b_echo_cancel_update_merger(f->echo_cancel, f->tx_timestamp);
#endif
}


/*---------------------------------------------------------------------------*
 *	put DTMF to layer 5
 *---------------------------------------------------------------------------*/
static void
dss1_lite_l5_put_dtmf(struct fifo_translator *f, 
    uint8_t *dtmf_ptr, uint16_t dtmf_len)
{
	struct dss1_lite *pdl = f->L1_sc;
	uint16_t x;
	char buf[2];

	for (x = 0; x != dtmf_len; x++) {

		buf[0] = dtmf_ptr[x];
		buf[1] = 0;

		dss1_lite_dtmf_event(pdl, buf);
	}
}

/*---------------------------------------------------------------------------*
 *	put mbuf to layer 5
 *---------------------------------------------------------------------------*/
void
dss1_lite_l5_put_mbuf(struct dss1_lite *pdl,
    struct dss1_lite_fifo *f, struct mbuf *m)
{
	fifo_translator_t *ft = f->ft;

	if (f->prot_curr.protocol_1 == P_DISABLE) {
		m_freem(m);
		return;
	}
	/* DTMF detect */
	if ((f->prot_curr.u.transp.dtmf_detect_enable) ||
	    (ft->L5_PUT_DTMF == &dss1_lite_l5_put_dtmf)) {
		i4b_dtmf_detect(ft, mtod(m, uint8_t *), m->m_len);
	}
	/* ISDN trace */
	if (f->is_tracing) {
		dss1_lite_trace(pdl, f, TRC_CH_D, 1, m);
	}
	f->out_stat += m->m_len;

	L5_PUT_MBUF(ft, m);
}

/*---------------------------------------------------------------------------*
 *	get new mbuf from layer 5
 *---------------------------------------------------------------------------*/
struct mbuf *
dss1_lite_l5_get_new_mbuf(struct dss1_lite *pdl, struct dss1_lite_fifo *f)
{
	struct mbuf *m;

	if (f->prot_curr.protocol_1 == P_DISABLE)
		return (NULL);

	m = L5_ALLOC_MBUF(f->ft, BCH_MAX_DATALEN, BCH_MAX_DATALEN);

	return (m);
}

/*---------------------------------------------------------------------------*
 *	get sample from layer 5
 *---------------------------------------------------------------------------*/
int16_t
dss1_lite_l5_get_sample(struct dss1_lite *pdl, struct dss1_lite_fifo *f)
{
	int16_t retval;
	uint8_t temp;

	if (f->prot_curr.protocol_1 == P_DISABLE)
		return (f->m_rx_last_sample);

	while (f->m_rx_curr_rem == 0) {
		if (f->m_rx_curr != NULL) {
			f->m_rx_curr = m_free(f->m_rx_curr);
		}
		if (f->m_rx_curr == NULL)
			f->m_rx_curr = dss1_lite_l5_get_mbuf(pdl, f);
		if (f->m_rx_curr == NULL)
			return (f->m_rx_last_sample);
		f->m_rx_curr_ptr = mtod(f->m_rx_curr, uint8_t *);
		f->m_rx_curr_rem = f->m_rx_curr->m_len;
	}

	temp = *(f->m_rx_curr_ptr)++;
	f->m_rx_curr_rem--;

	/* XXX optimise */
#ifndef HAVE_NO_ECHO_CANCEL
	if (f->prot_curr.u.transp.echo_cancel_enable)
		i4b_echo_cancel_feed(f->echo_cancel, &temp, 1);
#endif
	if (f->prot_curr.protocol_4 == BSUBPROT_G711_ALAW) {
		retval = i4b_alaw_to_signed[temp];
	} else {
		retval = i4b_ulaw_to_signed[temp];
	}
	f->m_rx_last_sample = retval;
	return (retval);
}

void
dss1_lite_l5_get_sample_complete(struct dss1_lite *pdl,
    struct dss1_lite_fifo *f)
{
	if (f->prot_curr.protocol_1 == P_DISABLE)
		return;
#ifndef HAVE_NO_ECHO_CANCEL
	if (f->prot_curr.u.transp.echo_cancel_enable)
		i4b_echo_cancel_update_feeder(f->echo_cancel, f->rx_timestamp);
#endif
}

/*---------------------------------------------------------------------------*
 *	get mbuf from layer 5
 *---------------------------------------------------------------------------*/
struct mbuf *
dss1_lite_l5_get_mbuf(struct dss1_lite *pdl, struct dss1_lite_fifo *f)
{
	fifo_translator_t *ft = f->ft;
	struct mbuf *m;

	if (f->prot_curr.protocol_1 == P_DISABLE)
		return (NULL);

	m = L5_GET_MBUF(ft);

	if (m != NULL) {

		f->in_stat += m->m_len;

		if (f->is_tracing) {
			dss1_lite_trace(pdl, f, TRC_CH_D, 0, m);
		}
	}
	return (m);
}

/*---------------------------------------------------------------------------*
 *	initialize rx/tx data structures
 *---------------------------------------------------------------------------*/
static void
dss1_lite_l1_setup(fifo_translator_t *ft, struct i4b_protocol *p)
{
	struct dss1_lite *pdl = ft->L1_sc;
	struct dss1_lite_fifo *f = ft->L1_fifo;
	struct mbuf *m;
	uint8_t fn;
	uint8_t i;

	f->prot_curr = *p;

	fn = f - pdl->dl_fifo;

	if (p->protocol_1 == P_DISABLE) {
		pdl->dl_methods->set_protocol(pdl, f, p);
	} else if (fn != 0) {
		pdl->dl_audio_channel = fn;
	}
	if (fn == 0) {

		pdl->dl_tx_end_tick = ticks - 1;

		/* the protocol runs in the opposite mode */
		if (pdl->dl_option_value & I4B_OPTION_NT_MODE)
			pdl->dl_is_nt_mode = 0;
		else
			pdl->dl_is_nt_mode = 1;

		for (i = 0; i != DL_QUEUE_MAX; i++) {
			if (pdl->dl_tx_mbuf[i] != NULL) {
				m_freem(pdl->dl_tx_mbuf[i]);
				pdl->dl_tx_mbuf[i] = NULL;
			}
		}

		while (1) {
			_IF_DEQUEUE(&pdl->dl_outq, m);
			if (m == NULL)
				break;
			m_freem(m);
		}

		pdl->dl_tx_in = pdl->dl_tx_out;
		pdl->dl_tx_window = 0;

	} else if (p->protocol_1 != P_DISABLE) {
#ifndef HAVE_NO_ECHO_CANCEL
		i4b_echo_cancel_init(f->echo_cancel, 0, f->prot_curr.protocol_4);
#endif
		/* init DTMF detector and generator */
		i4b_dtmf_init_rx(ft, f->prot_curr.protocol_4);
		i4b_dtmf_init_tx(ft, f->prot_curr.protocol_4);

		/* check if no DMTF detect is not enabled */
		if (ft->L5_PUT_DTMF == NULL)
			ft->L5_PUT_DTMF = dss1_lite_l5_put_dtmf;
	}
	if (f->m_tx_curr != NULL) {
		m_freem(f->m_tx_curr);
		f->m_tx_curr = NULL;
		f->m_tx_curr_rem = 0;
		f->m_tx_curr_ptr = 0;
	}
	if (f->m_rx_curr != NULL) {
		m_freem(f->m_rx_curr);
		f->m_rx_curr = NULL;
		f->m_rx_curr_rem = 0;
		f->m_rx_curr_ptr = 0;
	}
	f->m_tx_last_sample = 0;
	f->m_rx_last_sample = 0;

	if (p->protocol_1 != P_DISABLE)
		pdl->dl_methods->set_protocol(pdl, f, p);
}

/*---------------------------------------------------------------------------*
 *	start transmission
 *---------------------------------------------------------------------------*/
static void
dss1_lite_l1_start(fifo_translator_t *ft)
{
	struct dss1_lite *pdl = ft->L1_sc;
	struct dss1_lite_fifo *f = ft->L1_fifo;

	if (f->prot_curr.protocol_1 != P_DISABLE)
		pdl->dl_methods->set_start(pdl, f);
}

/*---------------------------------------------------------------------------*
 *	fill statistics structure
 *---------------------------------------------------------------------------*/
static void
dss1_lite_l1_stat(fifo_translator_t *ft, bchan_statistics_t *bsp)
{
	struct dss1_lite_fifo *f = ft->L1_fifo;

	bsp->inbytes = f->in_stat;
	bsp->outbytes = f->out_stat;

	f->in_stat = 0;
	f->out_stat = 0;
}

/*---------------------------------------------------------------------------*
 *	return FIFO-translator
 *
 * NOTE: "channel" range checking is performed by the caller
 *---------------------------------------------------------------------------*/
static fifo_translator_t *
dss1_lite_l1_get_ft(struct i4b_controller *cntl, int channel)
{
	struct dss1_lite *pdl = cntl->L1_sc;
	struct dss1_lite_fifo *f = cntl->L1_fifo;

	f += channel;

	f->ft[0].L1_sc = pdl;
	f->ft[0].L1_fifo = f;
	f->ft[0].L1_FIFO_SETUP = dss1_lite_l1_setup;
	f->ft[0].L1_FIFO_START = dss1_lite_l1_start;
	f->ft[0].L1_FIFO_STAT = dss1_lite_l1_stat;

	return (f->ft);
}

/*---------------------------------------------------------------------------*
 *	default I4B interface init
 *
 * Return values:
 * 0: Success
 * Else: Failure
 *---------------------------------------------------------------------------*/
uint8_t
dss1_lite_attach(struct dss1_lite *pdl, device_t dev,
    struct i4b_controller *ctrl,
    const struct dss1_lite_methods *mtod)
{
	uint8_t retval;

	if (dev != NULL)
		device_printf(dev, "Attaching I4B "
		    "controller %d.\n", ctrl->unit);
	else
		printf("i4b: Attaching I4B "
		    "controller %d.\n", ctrl->unit);

	pdl->dl_outq.ifq_maxlen = (2 * DL_QUEUE_MAX);
	pdl->dl_methods = mtod;
	pdl->dl_ctrl = ctrl;
	pdl->dl_option_mask = I4B_OPTION_NT_MODE;

	CNTL_LOCK(ctrl);

	/* init function pointers, type and controller */

	ctrl->L1_GET_FIFO_TRANSLATOR = &dss1_lite_l1_get_ft;
	ctrl->L1_COMMAND_REQ = &dss1_lite_l1_ioctl;

	ctrl->L1_sc = pdl;
	ctrl->L1_fifo = pdl->dl_fifo;	/* default */
	ctrl->L1_type = L1_TYPE_ISDN_BRI;
	ctrl->L1_channel_end = DL_NCHAN;

	CNTL_UNLOCK(ctrl);

	retval = i4b_controller_attach(ctrl, NULL);

	if (retval != 0) {
		i4b_controller_free(pdl->dl_ctrl, 1);
		pdl->dl_ctrl = NULL;
	}
	return (retval);
}

void
dss1_lite_detach(struct dss1_lite *pdl)
{
	if (pdl->dl_ctrl != NULL) {
		i4b_controller_detach(pdl->dl_ctrl);
		i4b_controller_free(pdl->dl_ctrl, 1);
		pdl->dl_ctrl = NULL;
	}
}
