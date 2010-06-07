/*-
 * Copyright (c) 2000-2006 Hans Petter Selasky. All rights reserved.
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
 *	i4b_ihfc2_l1if.c - layer 1 handler
 *	----------------------------------
 *
 * $FreeBSD: $
 *
 *      This file defines the functions that are used
 *      to interface with ISDN4BSD
 *
 *---------------------------------------------------------------------------*/

#include <i4b/layer1/ihfc3/i4b_ihfc2.h>
#include <i4b/layer1/ihfc3/i4b_ihfc2_ext.h>

/*---------------------------------------------------------------------------*
 *	handle command from the upper layers
 *---------------------------------------------------------------------------*/
static int
ihfc_mph_command_req(struct i4b_controller *cntl, int command, void *parm)
{
	ihfc_sc_t *sc = cntl->L1_sc;
	ihfc_fifo_t *f = cntl->L1_fifo;
	struct sc_state *st = &(sc->sc_state[f->sub_unit]);
	i4b_debug_t *dbg;
	u_int32_t temp;
	u_int16_t n;
	u_int8_t error[IHFC_MAX_ERR];

	error[0] = 0;

	IHFC_ASSERT_LOCKED(sc);

	IHFC_MSG("parm %s.\n", parm ? "!= NULL" : "== NULL");

	switch(command) {
	case CMR_SETTRACE:	/* set new trace mask */

	    temp = *((u_int32_t *)parm);

	    IHFC_MSG("CMR_SETTRACE: 0x%08x\n", temp);

	    /* clear trace bits first */

	    for(n = 0; n < cntl->L1_channel_end; n++)
	    {
	        (f + (2*n) +  receive)->state &= ~ST_I4B_TRACE;
		(f + (2*n) + transmit)->state &= ~ST_I4B_TRACE;
	    }

	    /* set trace bits */

	    if(cntl->L1_channel_end >= 1)
	    {
	        if(temp & TRACE_D_RX)
		{
		    (f + d1r)->state |= ST_I4B_TRACE;
		}
		if(temp & TRACE_D_TX)
		{
		    (f + d1t)->state |= ST_I4B_TRACE;
		}
	    }

	    for(n = 1; n < cntl->L1_channel_end; n++)
	    {
	        if(temp & TRACE_B_RX)
		{
		    (f + (2*n) + receive)->state |= ST_I4B_TRACE;
		}
		if(temp & TRACE_B_TX)
		{
		    (f + (2*n) + transmit)->state |= ST_I4B_TRACE;
		}
	    }
	    break;

	case CMR_RESET:
	    IHFC_MSG("CMR_RESET\n");

	    ihfc_reset(sc, &error[0]);
	    break;

	case CMR_SET_I4B_OPTIONS:
	    IHFC_MSG("CMR_SET_I4B_OPTIONS\n");

	    dbg = parm;

	    dbg->mask &= sc->sc_default.i4b_option_mask;
	    dbg->value &= sc->sc_default.i4b_option_mask;

	    temp = st->i4b_option_value;
	    temp &= ~(dbg->mask);
	    temp |= dbg->value;

	    if(st->i4b_option_value != temp)
	    {
	        const u_int32_t global_options =
		  (I4B_OPTION_PCM_SLAVE|
		   I4B_OPTION_PCM_SPEED_32|
		   I4B_OPTION_PCM_SPEED_64|
		   I4B_OPTION_PCM_SPEED_128|
		   I4B_OPTION_POLLED_MODE);

	        st->i4b_option_value = temp;

		IHFC_MSG("setting new options "
			 "value: 0x%08x\n", temp);

		temp &= global_options;

		for(n = 0; 
		    n < sc->sc_default.d_sub_controllers;
		    n++)
		{
		    st = &sc->sc_state[n];

		    st->i4b_option_value &= ~global_options;
		    st->i4b_option_value |=  temp;
		}
		ihfc_setup_softc(sc, &error[0]);
	    }
	    break;

	case CMR_SET_L1_AUTO_ACTIVATE_VARIABLE:
	    if(parm == NULL)
	    {
	        parm = &st->L1_auto_activate_variable;
		st->L1_auto_activate_variable = 0;
	    }
	    st->L1_auto_activate_ptr = parm;
	    break;

	case CMR_SET_L1_ACTIVITY_VARIABLE:
	    if(parm == NULL)
	    {
	        parm = &st->L1_activity_variable;
	    }
	    st->L1_activity_ptr = parm;

	    /*
	     * Update L1_activity_variable
	     */
	    *(st->L1_activity_ptr) = st->state.active;
	    break;

	case CMR_PH_ACTIVATE:
	    IHFC_MSG("CMR_PH_ACTIVATE\n");

	    ihfc_fsm_update(sc, f, 1);
	    break;

	case CMR_PH_DEACTIVATE:
	    IHFC_MSG("CMR_PH_DEACTIVATE\n");

	    ihfc_fsm_update(sc, f, 2);
	    break;

	case CMR_SET_LAYER1_PROTOCOL:
	{
	    dbg = parm;

	    if(dbg->chan >= cntl->L1_channel_end)
	    {
	        IHFC_MSG("Invalid channel >= %d!\n",
			 cntl->L1_channel_end);
		break;
	    }

	    dbg->chan *= 2;

	    IHFC_MSG("CMR_SET_LAYER1_PROTOCOL: 0x%02x(#%d,#%d)\n",
		     (dbg->value) & 0xff,
		     (FIFO_NO(f) + dbg->chan + receive),
		     (FIFO_NO(f) + dbg->chan + transmit));

	    (f + dbg->chan +  receive)->default_prot = dbg->value;
	    (f + dbg->chan + transmit)->default_prot = dbg->value;
	    break;
	}

	case CMR_INFO_REQUEST:
	{
	    msg_ctrl_info_req_t *req = parm;

	    if(sc->sc_device)
	    {
	        snprintf(req->l1_desc, sizeof(req->l1_desc),
			 "%s", device_get_desc(sc->sc_device));
	    }

	    if(st->state.description)
	    {
	        snprintf(req->l1_state, sizeof(req->l1_state),
			 "%s", st->state.description);
	    }

	    req->l1_active = st->state.active;

	    break;
	}

#if (CHAN_D1 != 0)
#error "(CHAN_D1 != 0)"
#endif
	case CMR_ENCODE_CHANNEL:
	{
	  int32_t *p_channel_id = parm;

	  switch(sc->sc_channel_mapping) {
	  default:
	    if(p_channel_id[0] < 0)
	    {
	        p_channel_id[0] = -1;
		break;
	    }
	    if(p_channel_id[0] == CHAN_D1)
	    {
	        p_channel_id[0] = 0x10;
		break;
	    }
	    if(p_channel_id[0] >= (CHAN_D1 + 0x10))
	    {
	        p_channel_id[0] += 1;
		break;
	    }
	    break;
	  }
	  break;
	}

	case CMR_DECODE_CHANNEL:
	{
	  int32_t *p_channel_id = parm;

	  switch(sc->sc_channel_mapping) {
	  default:
	    if(p_channel_id[0] == 0x10)
	    {
	        p_channel_id[0] = CHAN_D1;
		break;
	    }
	    if(p_channel_id[0] <= 0)
	    {
	        p_channel_id[0] = CHAN_NOT_ANY;
		break;
	    }
	    if(p_channel_id[0] > 0x10)
	    {
	        p_channel_id[0] -= 1;
		break;
	    }
	    break;
	  }
	  break;
	}

	case CMR_SET_CHANNEL_MAPPING:
	{
	    u_int16_t *p_map = parm;

	    IHFC_MSG("CMR_SET_CHANNEL_MAPPING\n");

	    if(sc->sc_channel_mapping != p_map[0])
	    {
	        /* need to reset after that channel
		 * mapping has changed:
		 */
	        sc->sc_channel_mapping = p_map[0];
		ihfc_setup_softc(sc, &error[0]);
	    }
	    break;
	}

	case CMR_ENABLE_ECHO_CANCEL:
	{
	    struct fifo_translator *ft = parm;
	    f = ft->L1_fifo;

	    if(PROT_IS_TRANSPARENT(&((f + receive)->prot_curr)) &&
	       PROT_IS_TRANSPARENT(&((f + transmit)->prot_curr)) &&
	       sc->sc_default.o_ECHO_CANCEL_ENABLED)
	    {
	        (f + receive)->prot_curr.u.transp.echo_cancel_enable = 1;
		(f + transmit)->prot_curr.u.transp.echo_cancel_enable = 1;
	    }
	    else
	    {
	        return EINVAL;
	    }
	    break;
	}

	case CMR_DISABLE_ECHO_CANCEL:
	{
	    struct fifo_translator *ft = parm;
	    f = ft->L1_fifo;

	    if(PROT_IS_TRANSPARENT(&((f + receive)->prot_curr)) &&
	       PROT_IS_TRANSPARENT(&((f + transmit)->prot_curr)))
	    {
	        (f + receive)->prot_curr.u.transp.echo_cancel_enable = 0;
		(f + transmit)->prot_curr.u.transp.echo_cancel_enable = 0;
	    }
	    else
	    {
	        return EINVAL;
	    }
	    break;
	}

	case CMR_GET_EC_FIR_FILTER:
	{
	    i4b_ec_debug_t *ec_dbg = parm;
	    struct i4b_echo_cancel *ec_p;
	    uint32_t points;
	    uint32_t x;
	    enum { EC_POINTS = (sizeof(ec_p->fet_hr)/
				sizeof(ec_p->fet_hr[0])),
		   DBG_POINTS = (sizeof(ec_dbg->ydata)/
				 sizeof(ec_dbg->ydata[0])) };

	    if ((ec_dbg->chan >= cntl->L1_channel_end) ||
		(ec_dbg->offset > EC_POINTS))
	    {
		return EINVAL;
	    }

	    points = (EC_POINTS - ec_dbg->offset);
	    if (points > DBG_POINTS) {
		points = DBG_POINTS;
	    }

	    f += (2 * ec_dbg->chan);

	    if (PROT_IS_TRANSPARENT(&((f + receive)->prot_curr)) &&
		PROT_IS_TRANSPARENT(&((f + transmit)->prot_curr)) &&
		(f + receive)->prot_curr.u.transp.echo_cancel_enable &&
		(f + transmit)->prot_curr.u.transp.echo_cancel_enable)
	    {
	        ec_p = &(sc->sc_echo_cancel[FIFO_NO(f)/2]);
		ec_dbg->npoints = EC_POINTS;
		ec_dbg->decimal_point = I4B_EC_N_HR_DP;
		for (x = 0; x < points; x++) {
			ec_dbg->ydata[x] = ec_p->fet_hr[x+ec_dbg->offset];
		}
	    }
	    else
	    {
	        return EINVAL;
	    }
	    break;
	}

	case CMR_ENABLE_DTMF_DETECT:
	{
	    struct fifo_translator *ft = parm;
	    f = ft->L1_fifo;

	    if(PROT_IS_TRANSPARENT(&((f + receive)->prot_curr)))
	    {
	        (f + receive)->prot_curr.u.transp.dtmf_detect_enable = 1;
	    }
	    else
	    {
	        return EINVAL;
	    }
	    break;
	}

	case CMR_DISABLE_DTMF_DETECT:
	{
	    struct fifo_translator *ft = parm;
	    f = ft->L1_fifo;

	    if(PROT_IS_TRANSPARENT(&((f + receive)->prot_curr)))
	    {
	        (f + receive)->prot_curr.u.transp.dtmf_detect_enable = 0;
	    }
	    else
	    {
	        return EINVAL;
	    }
	    break;
	}

	default:
	    IHFC_MSG("unsupported command, 0x%08x!\n", command);
	    break;
	}

	if(error[0] != 0)
	{
	    IHFC_ERR("Command failed: %s!\n", &error[0]);
	    return ENODEV;
	}

	return 0;
}

/*---------------------------------------------------------------------------*
 *	trace routine
 *---------------------------------------------------------------------------*/
static void
ihfc_trace(ihfc_sc_t *sc, ihfc_fifo_t *f, int type, struct mbuf *m)
{
	struct i4b_controller *cntl = sc->sc_state[f->sub_unit].i4b_controller;
	u_int8_t from_te = (IS_NT_MODE(sc,f->sub_unit) != 0);
	ihfc_fifo_t *f_start = cntl->L1_fifo;

	if(type == TRC_CH_D) {
	   type += ((FIFO_NO(f) - FIFO_NO(f_start)) / 2);

	   if(FIFO_DIR(f) == transmit) {
	       from_te = !from_te;
	   }
	}

	sc->sc_trace_hdr.count++;
	sc->sc_trace_hdr.unit = cntl->unit;
	sc->sc_trace_hdr.type = type;
	sc->sc_trace_hdr.dir = from_te ? FROM_TE : FROM_NT;

	microtime(&sc->sc_trace_hdr.time);

	i4b_l1_trace_ind(&sc->sc_trace_hdr, m);

	return;
}

/*---------------------------------------------------------------------------*
 *	trace info routine
 *---------------------------------------------------------------------------*/
void
ihfc_trace_info(ihfc_sc_t *sc, ihfc_fifo_t *f, const u_int8_t *desc)
{
	u_int16_t len = strlen(desc);
	struct mbuf *m = i4b_getmbuf(len, M_NOWAIT);

	if(m)
	{
	    bcopy(desc, m->m_data, len);

	    ihfc_trace(sc, f, TRC_CH_I, m);

	    m_freem(m);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	put mbuf to layer 5
 *---------------------------------------------------------------------------*/
void
ihfc_i4b_putmbuf(ihfc_sc_t *sc, ihfc_fifo_t *f, struct mbuf *m)
{
	fifo_translator_t *ft = FIFO_TRANSLATOR(sc,f);

	if(f->state & ST_I4B_TRACE)
	{
	    ihfc_trace(sc,f,TRC_CH_D,m);
	}

	f->io_stat += m->m_len;
	m->m_pkthdr.len = m->m_len;

	L5_PUT_MBUF(ft,m);

	return;
}

/*---------------------------------------------------------------------------*
 *	get mbuf from layer 5
 *---------------------------------------------------------------------------*/
struct mbuf *
ihfc_i4b_getmbuf(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	fifo_translator_t *ft = FIFO_TRANSLATOR(sc,f);
	register struct mbuf *m1;

 repeat:

	m1 = L5_GET_MBUF(ft);

	if(m1)
	{
	    if(m1->m_next)
	    {
	        /* XXX the system currently doesn't handle
		 * chained mbufs, so a pullup is needed:
		 */
	        struct mbuf *m2;
		struct mbuf *m3;
		u_int32_t total_len = 0;
		u_int8_t *ptr;

		m2 = m1;
		do {
		  total_len += m2->m_len;
		} while((m2 = m2->m_next));

		m3 = i4b_getmbuf(total_len, M_NOWAIT);

		if(m3 == NULL)
		{
		    printf("%s: %s: cannot pullup mbuf "
			   "with length %d bytes!\n",
			   __FILE__, __FUNCTION__, 
			   total_len);

		    m_freem(m1);
		    goto repeat;
		}

		ptr = m3->m_data;

		m2 = m1;
		do {
		  bcopy(m2->m_data, ptr, m2->m_len);
		  ptr += m2->m_len;
		} while((m2 = m2->m_next));

		m_freem(m1);
		m1 = m3;
	    }

	    f->io_stat += m1->m_len;

	    if(f->state & ST_I4B_TRACE)
	    {
	        ihfc_trace(sc,f,TRC_CH_D,m1);
	    }
#if 0
	    IHFC_ERR("(#%d) %p\n", FIFO_NO(f), m1);
#endif
	}
	return m1;
}

/*---------------------------------------------------------------------------*
 *	initialize rx/tx data structures
 *---------------------------------------------------------------------------*/
static void
ihfc_B_setup(fifo_translator_t *ft, struct i4b_protocol *p)
{
	ihfc_sc_t  *sc = ft->L1_sc;
	ihfc_fifo_t *f = ft->L1_fifo;

	IHFC_MSG("fifo(#%d/#%d), protocol_1=%d\n",
		 FIFO_NO(f)+transmit,FIFO_NO(f)+receive,p->protocol_1);

	/*
	 * Channels are setup in pairs(RX & TX) for I4B
	 */

	(f +  receive)->prot_curr = *p;
	(f + transmit)->prot_curr = *p;

	ihfc_fifo_setup(sc, f +  receive);
	ihfc_fifo_setup(sc, f + transmit);

	return;
}

/*---------------------------------------------------------------------------*
 *	start transmission
 *---------------------------------------------------------------------------*/
static void
ihfc_B_start(fifo_translator_t *ft)
{
	ihfc_sc_t *sc  = ft->L1_sc;
	ihfc_fifo_t *f = ft->L1_fifo;

	ihfc_fifo_call(sc,f+transmit); /* quick tx */

	return;
}

/*---------------------------------------------------------------------------*
 *	fill statistics structure
 *---------------------------------------------------------------------------*/
static void
ihfc_B_stat(fifo_translator_t *ft, bchan_statistics_t *bsp)
{
#if DO_I4B_DEBUG
        ihfc_sc_t *sc  = ft->L1_sc;
#endif
	ihfc_fifo_t *f = ft->L1_fifo;

	IHFC_MSG("\n");

	bsp->inbytes  = (f + receive  )->io_stat;
	bsp->outbytes = (f +  transmit)->io_stat;

	(f + receive  )->io_stat = 0;
	(f +  transmit)->io_stat = 0;

	return;
}

/*---------------------------------------------------------------------------*
 *	return FIFO-translator
 *
 * NOTE: "channel" range checking is performed by the caller
 *---------------------------------------------------------------------------*/
static fifo_translator_t *
ihfc_B_get_fifo_translator(struct i4b_controller *cntl, int channel)
{
	ihfc_sc_t *sc = cntl->L1_sc;
	ihfc_fifo_t *f = cntl->L1_fifo;
	fifo_translator_t *ft;

	IHFC_MSG("\n");

	f += (channel*2);
	ft = FIFO_TRANSLATOR(sc,f);

	ft->L1_sc		= sc;
	ft->L1_fifo		= f;
	ft->L1_FIFO_SETUP	= ihfc_B_setup;
	ft->L1_FIFO_START	= ihfc_B_start;
	ft->L1_FIFO_STAT	= ihfc_B_stat;

	return ft;
}

/*---------------------------------------------------------------------------*
 *	default I4B interface init
 *---------------------------------------------------------------------------*/
void
ihfc_init_i4b(ihfc_sc_t *sc, struct i4b_controller *cntl)
{
	CNTL_LOCK(cntl);

	/* init function pointers, type and controller */

	cntl->L1_GET_FIFO_TRANSLATOR = &ihfc_B_get_fifo_translator;
	cntl->L1_COMMAND_REQ = &ihfc_mph_command_req;

	cntl->L1_sc = sc;
	cntl->L1_fifo = &sc->sc_fifo[0]; /* default */
	cntl->L1_type = sc->sc_default.d_L1_type;
	cntl->L1_channel_end = (sc->sc_default.d_channels / 2); /* default */

	CNTL_UNLOCK(cntl);
	return;
}
 
/*---------------------------------------------------------------------------*
 *	default I4B interface setup
 *---------------------------------------------------------------------------*/
u_int8_t
ihfc_setup_i4b(ihfc_sc_t *sc, u_int8_t *error)
{
	i4b_controller_t *cntl = sc->sc_resources.i4b_controller;
	u_int8_t sub_controllers = sc->sc_default.d_sub_controllers;
	u_int8_t retval = 0;

	while(sub_controllers--) {
	    device_printf(sc->sc_device, "Attaching I4B "
			  "controller %d%s.\n", cntl->unit,
			  IS_PCM_SLAVE(sc,0) ? 
			  " (PCM slave mode)" : "");

	    retval |= i4b_controller_attach(cntl,error);
	    cntl++;
	}
	return retval;
}

/*---------------------------------------------------------------------------*
 *	default I4B interface unsetup
 *---------------------------------------------------------------------------*/
void
ihfc_unsetup_i4b(ihfc_sc_t *sc)
{
	i4b_controller_t *cntl = sc->sc_resources.i4b_controller;
	u_int8_t sub_controllers = sc->sc_default.d_sub_controllers;

	while(sub_controllers--) {
	    i4b_controller_detach(cntl); 
	    cntl++;
	}
	return;
}
