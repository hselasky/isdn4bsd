/*-
 * Copyright (c) 2001 Cubical Solutions Ltd. All rights reserved.
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
 * capi/capi_l4if.c	The CAPI i4b L4/device interface.
 *
 * $FreeBSD: src/sys/i4b/capi/capi_l4if.c $
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <net/if.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

#include <i4b/capi/capi.h>
#include <i4b/capi/capi_msgs.h>

static void n_connect_request(u_int cdid);
static void n_connect_response(u_int cdid, int response, int cause);
static void n_disconnect_request(u_int cdid, int cause);
static void n_alert_request(u_int cdid);
static void n_mgmt_command(int unit, int cmd, void *parm);
static int  n_download(int unit, int, struct isdn_dr_prot *);

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
	case CMR_DOWNLOAD:
	{
	    struct isdn_download_request *req = (void *)param;

	    if (sc->load && req->numprotos) {

	        (sc->load)(sc, req->protocols[0].bytecount,
			   req->protocols[0].microcode);
	    }

	    break;
	}


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

	capi_start_tx(sc,f);

	// ihfc_fifo_call(sc,f+transmit); /* quick tx */

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
	cntl->L1_type = sc->sc_default.d_L1_type; XXX CAPI_TYPE;
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

/*---------------------------------------------------------------------------*
 *	capi_get_mbuf - return frame to layer 1
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_get_mbuf(fifo_translator_t *f)
{
    return NULL;
}

/*---------------------------------------------------------------------------*
 *	capi_put_mbuf - process frame from layer 1
 *---------------------------------------------------------------------------*/
static void
capi_put_mbuf(fifo_translator_t *f, struct mbuf *m)
{
    m_freem(m);
    return;
}

static capi_softc_t capi_sc[MAX_CONTROLLERS];

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
fifo_translator_t *
capi_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
	      struct i4b_protocol *pp, u_int32_t driver_type, 
	      u_int32_t driver_unit, call_desc_t *cd)
{
	DSS1_TCP_pipe_t *pipe;
	l2softc_t *sc = &l2_softc[driver_unit];
	u_int16_t max_channels;

	if(!pp)
	{
	  return (driver_unit < MAX_CONTROLLERS) ? 
	    sc->sc_fifo_translator : FT_INVALID;
	}

	if(pp->protocol_1)
	{
#define BZERO(ptr) bzero(ptr,sizeof(*ptr))

	  /* clear all call-descriptors */
	  bzero(cntl->N_call_desc_start, 
		(((u_int8_t *)(cntl->N_call_desc_end)) -
		 ((u_int8_t *)(cntl->N_call_desc_start))));

	  /* set all channels free */
	  BZERO(&cntl->N_channel_utilization);

	  /**/
	  BZERO(sc);
	}

	cntl->N_fifo_translator =
	  sc->sc_fifo_translator = f;

	if(pp->protocol_1)
	{
	  /* connected */

	  __typeof(pipe->__pn)
	    pipe_number = 0;

	  f->L5_sc       = sc;
	  f->L5_GET_MBUF = &capi_get_mbuf;
	  f->L5_PUT_MBUF = &capi_put_mbuf;

	  pp->protocol_1 = P_HDLC;

	  cntl->N_RETRIEVE_REQUEST = n_retrieve_request;
	  cntl->N_HOLD_REQUEST = n_hold_request;
	  cntl->N_DEFLECT_REQUEST = n_deflect_request;
	  cntl->N_MCID_REQUEST = n_mcid_request;
	  cntl->N_CONNECT_REQUEST = n_connect_request;
	  cntl->N_INFORMATION_REQUEST = n_information_request;
	  cntl->N_CONNECT_RESPONSE = n_connect_response;
	  cntl->N_DISCONNECT_REQUEST = n_disconnect_request;
	  cntl->N_ALERT_REQUEST = n_alert_request;        
	  cntl->N_PROGRESS_REQUEST = n_progress_request;        
	  cntl->N_ALLOCATE_CD = n_allocate_cd;
	  cntl->N_FREE_CD = n_free_cd;

	  cntl->N_cdid_end = 0x7F; /* exclusive */
	  /* cntl->N_cdid_count = use last value */

	  cntl->N_lapdstat = NULL;

#define MAX_QUEUE_PER_CHANNEL 16

	  max_channels = cntl->L1_channel_end;

	  if(max_channels < 3)
	     max_channels = 3;

	  if(cntl->L1_type == L1_TYPE_ISDN_PRI)
	    sc->sc_primary_rate = 1;
	  else
	    sc->sc_primary_rate = 0;

	  if((driver_type == DRVR_DSS1_P2P_TE) ||
	     (driver_type == DRVR_DSS1_P2P_NT))
	    sc->sc_point_to_point = 1;
	  else
	    sc->sc_point_to_point = 0;

	  /* set default queue length */

	  _IF_QUEUE_GET(sc)->ifq_maxlen =
	    max_channels * MAX_QUEUE_PER_CHANNEL;

	  if((driver_type == DRVR_DSS1_NT) ||
	     (driver_type == DRVR_DSS1_P2P_NT))
	  {
	    /* set NT-mode */
	    i4b_l1_set_options(cntl, 
			       I4B_OPTION_NT_MODE, 
			       I4B_OPTION_NT_MODE);

	    cntl->N_nt_mode = 1;
	    sc->sc_nt_mode = 1;

	    if(!IS_POINT_TO_POINT(sc))
	    {
	        _IF_QUEUE_GET(sc)->ifq_maxlen =
		  max_channels * (PIPE_MAX * MAX_QUEUE_PER_CHANNEL);
	    }
	  }
	  else
	  {
	    /* set TE-mode */
	    i4b_l1_set_options(cntl, 
			       I4B_OPTION_NT_MODE, 
			       0);

	    cntl->N_nt_mode = 0;
	    sc->sc_nt_mode = 0;
	  }
#if 0
	  _IF_QUEUE_GET(sc)->ifq_maxlen = IFQ_MAXLEN;
#endif

	  /* initialize the callout handles for timeout routines */
	  __callout_init_mtx(&sc->ID_REQUEST_callout, 
			     CNTL_GET_LOCK(cntl), 0);

	  __callout_init_mtx(&sc->L1_activity_callout, 
			     CNTL_GET_LOCK(cntl), 0);

	  sc->sc_cntl = cntl;
	  sc->sc_unit = cntl->unit;

	  /* start L1-activity-callout */
	  dss1_L1_activity_timeout(sc);

	  sc->sc_current_pipe = &sc->sc_pipe[0];

	  L1_COMMAND_REQ(cntl,CMR_SET_L1_AUTO_ACTIVATE_VARIABLE,&sc->L1_auto_activate);
	  L1_COMMAND_REQ(cntl,CMR_SET_L1_ACTIVITY_VARIABLE,&sc->L1_activity);

	  PIPE_FOREACH(pipe,&sc->sc_pipe[0])
	  {
	    pipe->__pn = pipe_number++;

	    pipe->L5_sc = sc;

	    pipe->tei = (IS_POINT_TO_POINT(sc) && 
			 (PIPE_NO(pipe) == 0)) ? 
	      TEI_POINT2POINT :
	      TEI_BROADCAST;

	    pipe->serial_number = 
	      ((cntl->N_serial_number + PIPE_NO(pipe))
	       DSS1_TEI_IS_ODD(*2)) DSS1_TEI_IS_ODD(|1);

	    __callout_init_mtx(&pipe->set_state_callout, 
			       CNTL_GET_LOCK(cntl), 0);

	    __callout_init_mtx(&pipe->get_mbuf_callout, 
			       CNTL_GET_LOCK(cntl), 0);
#if 0
	    _IF_QUEUE_GET(pipe)->ifq_maxlen = IFQ_MAXLEN;
#else
	    _IF_QUEUE_GET(pipe)->ifq_maxlen = max_channels * MAX_QUEUE_PER_CHANNEL;
#endif
	  }

	  /* own receiver is always ready */

	  /* reserve D-channel */
	  SET_CHANNEL_UTILIZATION(cntl,CHAN_D1,1);
	}
	else
	{
	  /* not connected */

	  sc->L1_activity = 0; /* clear activity */
	  sc->L1_auto_activate = 0; /* clear auto-activate */
	  sc->L1_deactivate_count = 0; /* clear deactivate-count */
	  sc->sc_current_length = 0; /* clear current-length */

	  cntl->N_lapdstat = NULL;

	  /* untimeout */
	  __callout_stop(&sc->ID_REQUEST_callout);
	  __callout_stop(&sc->L1_activity_callout);

	  PIPE_FOREACH(pipe,&sc->sc_pipe[0])
	  {
		/**/
		dss1_pipe_set_state(pipe,ST_L2_PAUSE);

		/* _IF_DRAIN(pipe); */
		if(pipe->refcount != 0)
		{
		  /* error */
		}

		/* untimeout */
		__callout_stop(&pipe->set_state_callout);
		__callout_stop(&pipe->get_mbuf_callout);
	  }

	  _IF_DRAIN(sc);

	  L1_COMMAND_REQ(cntl,CMR_SET_L1_AUTO_ACTIVATE_VARIABLE,NULL);
	  L1_COMMAND_REQ(cntl,CMR_SET_L1_ACTIVITY_VARIABLE,NULL);
	}
	return f;
}

/*
//  capi_ll_attach
//      Called by a link layer driver at boot time.
*/

int
capi_ll_attach(capi_softc_t *sc)
{
    int i;


    /* Unit state */

    sc->sc_enabled = FALSE;
    sc->sc_state = C_DOWN;
    sc->sc_msgid = 0;

}

/*
//  n_connect_request
//      i4b L4 wants to connect. We assign a B channel to the call,
//      send a CAPI_CONNECT_REQ, and set the channel to B_CONNECT_CONF.
*/

static void
n_connect_request(u_int cdid)
{
    call_desc_t *cd = cd_by_cdid(cdid);
    capi_softc_t *sc;
    int bch, s;

    if (!cd) {
	printf("capi?: invalid cdid %d\n", cdid);
	return;
    }

    sc = capi_sc[ctrl_desc[cd->controller].unit];
    bch = cd->channelid;

    s = SPLI4B();

    if ((bch < 0) || (bch >= sc->sc_nbch))
	for (bch = 0; bch < sc->sc_nbch; bch++)
	    if (sc->sc_bchan[bch].state == B_FREE)
		break;

    if (bch == sc->sc_nbch) {
	splx(s);
	printf("capi%d: no free B channel\n", sc->sc_unit);
	return;
    }

    cd->channelid = bch;

    capi_connect_req(sc, cd);
    splx(s);
}

/*
//  n_connect_response
//      i4b L4 answers a call. We send a CONNECT_RESP with the proper
//      Reject code, and set the channel to B_CONNECT_B3_IND or B_FREE,
//      depending whether we answer or not.
*/

static void
n_connect_response(u_int cdid, int response, int cause)
{
    call_desc_t *cd = cd_by_cdid(cdid);
    capi_softc_t *sc;
    int bch, s;

    if (!cd) {
	printf("capi?: invalid cdid %d\n", cdid);
	return;
    }

    sc = capi_sc[ctrl_desc[cd->controller].unit];
    bch = cd->channelid;

    T400_stop(cd);
	
    cd->response = response;
    cd->cause_out = cause;

    s = SPLI4B();
    capi_connect_resp(sc, cd);
    splx(s);
}

/*
//  n_disconnect_request
//      i4b L4 wants to disconnect. We send a DISCONNECT_REQ and
//      set the channel to B_DISCONNECT_CONF.
*/

static void
n_disconnect_request(u_int cdid, int cause)
{
    call_desc_t *cd = cd_by_cdid(cdid);
    capi_softc_t *sc;
    int bch, s;

    if (!cd) {
	printf("capi?: invalid cdid %d\n", cdid);
	return;
    }

    sc = capi_sc[ctrl_desc[cd->controller].unit];
    bch = cd->channelid;

    cd->cause_out = cause;

    s = SPLI4B();
    capi_disconnect_req(sc, cd);
    splx(s);
}

/*
//  n_alert_request
//      i4b L4 wants to alert an incoming call. We send ALERT_REQ.
*/

static void
n_alert_request(u_int cdid)
{
    call_desc_t *cd = cd_by_cdid(cdid);
    capi_softc_t *sc;
    int s;

    if (!cd) {
	printf("capi?: invalid cdid %d\n", cdid);
	return;
    }

    sc = capi_sc[ctrl_desc[cd->controller].unit];

    s = SPLI4B();
    capi_alert_req(sc, cd);
    splx(s);
}

/*---------------------------------------------------------------------------*
 *      allocate and initialize a calldescriptor
 *---------------------------------------------------------------------------*/
static call_desc_t *
n_allocate_cd(struct i4b_controller *cntl, void *pipe, u_int crval,
              u_int ai_type, void *ai_ptr)
{
    l2softc_t *sc = cntl->N_fifo_translator->L5_sc; 
    call_desc_t *cd;
    u_int8_t state;

    cd = i4b_allocate_cd(cntl);

    if(cd == NULL) {
        goto done;
    }
    
    if(pipe == NULL) {

        /* outgoing call */

        state = ST_L3_OUTGOING;
	pipe = &sc->sc_pipe[0]; /* pipe adapter */
	crval = 1; /* controller */

	cd->dir_incoming = 0;

    } else {

        /* incoming call */

        state = ST_L3_INCOMING;

	cd->dir_incoming = 1;
    }

    /* setup variables */

    cd->channel_id = CHAN_ANY;
    cd->channel_bprot = BPROT_NONE;

    cd->cr = crval;

    cd->dst_telno_ptr = &(cd->dst_telno[0]);

    /* other variables are reset to zero by
     * "i4b_allocate_cd()"
     */

    /* set application interface before pipe and state */
    cd_set_appl_interface(cd,ai_type,ai_ptr);

    /* set pipe before state and tx */
    cd_set_pipe(cd,pipe);

    cd_set_state(cd,state);

 done:
    return cd;
}

/*---------------------------------------------------------------------------*
 *      free an allocated calldescriptor
 *
 * Allocated CD's need to be freed to decrement 
 * L1 activation references, at least.
 *---------------------------------------------------------------------------*/
static void
n_free_cd(call_desc_t *cd)
{
    i4b_l4_disconnect_ind(cd, 0);

    /* clear application interface after sending
     * the disconnect indication
     */
    cd_set_appl_interface(cd,0,NULL);

    /* clear pipe */
    cd_set_pipe(cd,NULL);

    /* free channel */
    cd_free_channel(cd);

    i4b_free_cd(cd);
    return;
}
