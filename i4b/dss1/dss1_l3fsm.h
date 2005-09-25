/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2004 Hans Petter Selasky. All rights reserved.
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
 *	dss1_l3fsm.h - layer 3 FSM
 *	--------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

static void
dss1_L1_activity_timeout(l2softc_t *sc)
{
  FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
  {
    /* connected */
    if(sc->L1_activity && !sc->L1_auto_activate)
    {
      if(sc->L1_deactivate_count)
      {
	/* no call for last 15*hz - deactivate */
	L1_COMMAND_REQ(sc->sc_cntl,CMR_PH_DEACTIVATE,NULL);
	sc->L1_deactivate_count = 0;
      }
      else
      {
	sc->L1_deactivate_count = 1;
      }
    }
    else
    {
      sc->L1_deactivate_count = 0;
    }

    /* start timer - should run when not auto-activated */
    callout_reset(&sc->L1_activity_callout, 15*hz,
		  (void *)(void *)&dss1_L1_activity_timeout, sc);
  },
  {
    /* not connected */
  });

  return;
}

static void
dss1_L1_deactivate_req(l2softc_t *sc)
{
  /* deactivate L1 */
  if(--sc->L1_auto_activate == 0)
  {
    dss1_L1_activity_timeout(sc);
  }

  return;
}

static void
dss1_L1_activate_req(l2softc_t *sc)
{
  /* activate L1 */
  if(!sc->L1_auto_activate++)
  {
    /* stop timer */
    callout_stop(&sc->L1_activity_callout);

    sc->L1_deactivate_count = 0;

    L1_COMMAND_REQ(sc->sc_cntl,CMR_PH_ACTIVATE,NULL);
  }

  return;
}

static void
cd_set_pipe(call_desc_t *cd, DSS1_TCP_pipe_t *pipe)
{
  if(pipe)
  {
    pipe->refcount++;
    /* L1 must auto-activate
     * while the refcount is set
     */
    dss1_L1_activate_req(pipe->L5_sc);

    /* pipe must auto-establish
     * while the refcount is set
     */
    dss1_pipe_activate_req(pipe);
  }

  if(cd->pipe)
  {
    ((__typeof(pipe))cd->pipe)->refcount--;
    dss1_L1_deactivate_req(((__typeof(pipe))cd->pipe)->L5_sc);
  }

  /* update pipe */
  cd->pipe = pipe;

  return;
}

static void
cd_set_state(call_desc_t *cd, u_int8_t newstate);

static void
cd_set_state_timeout(call_desc_t *cd)
{
  i4b_controller_t *cntl = 
    i4b_controller_by_cd(cd);
  static const __typeof(cd->state)
    MAKE_TABLE(L3_STATES,TIMEOUT_STATE,[]);

  CNTL_LOCK(cntl);

  if(cntl->N_fifo_translator)
  {
    /* connected */
    cd_set_state(cd,L3_STATES_TIMEOUT_STATE[cd->state]);
  }

  CNTL_UNLOCK(cntl);
  return;
}

static const char * const MAKE_TABLE(L3_STATES,DESC,[]);
static const char * const MAKE_TABLE(L3_EVENTS,DESC,[]);
static const u_int8_t MAKE_TABLE(L3_STATES,TIMEOUT_DELAY,[]);

static void
cd_set_state(call_desc_t *cd, u_int8_t newstate)
{
  DSS1_TCP_pipe_t *pipe = cd->pipe;
  l2softc_t *sc = pipe->L5_sc;

  /* stop timer */
  callout_stop(&cd->set_state_callout);

  if(newstate == ST_L3_U0)
  {
	if((cd->state == ST_L3_UA_TO) ||
	   (cd->state == ST_L3_U3_TO) ||
	   (cd->state == ST_L3_U4_TO) ||
	   (cd->state == ST_L3_U7_TO))
	{
		NDBGL3(L3_ERR,
		       "cdid=%d, no status reply - call disconnected",
		       cd->cdid);
	}
  }

  cd->state = newstate;

  if(newstate == ST_L3_U0)
  {
	/* cleanup L2 before sending
	 * RELEASE_COMPLETE
	 */
	dss1_pipe_data_acknowledge(pipe,cd);

	/* need to release the call when:
	 * sent connect
	 * sent setup
	 * setup rejected
	 */
	if(cd->need_release)
	{
	  if(cd->cause_out == 0)
	  {
	    /* recovery on timer expiry */
	    cd->cause_out = CAUSE_Q850_RECOTIMEXP;
	  }

	  /*
	   * send RELEASE_COMPLETE in all
	   * cases, instead of DISCONNECT and
	   * RELEASE, and send cause
	   */
	  dss1_l3_tx_release_complete(cd, 1);
	}
	else
	{
	  /* call ignored */
	}

	if(sc->L1_activity == 0)
	{
	  /* persistent L1 deactivation */
	  NDBGL3(L3_ERR,
		 "cdid=%d, persistent L1 deactivation - "
		 "check ISDN cable",
		 cd->cdid);
	  
	  SET_CAUSE_TV(cd->cause_in, CAUSET_I4B, CAUSE_I4B_L1ERROR);
	}
	else
	{
#if 0
	  if(L2_STATE_IS_DEACTIVATED(pipe))
	  {
	    /* persistent L2 deactivation */
	    NDBGL3(L3_ERR,
		   "cdid=%d, persistent L2 deactivation",
		   cd->cdid);

	    SET_CAUSE_TV(cd->cause_in, CAUSET_I4B, CAUSE_I4B_L2ERROR);
	  }
	  else
	  {
	    /* if( ??? )
	     * SET_CAUSE_TV(cd->cause_in, CAUSET_I4B, CAUSE_I4B_NORMAL);
	     */
	  }
#endif
	}

	/* free cd */
	N_FREE_CD(cd);
  }
  else
  {
    /* re-start timeout;
     * the timeout is increased when L1 is not activated
     * the timeout is always running while the CD is allocated
     */
    callout_reset(&cd->set_state_callout,
		  (L3_STATES_TIMEOUT_DELAY[newstate]*hz) +
		  (sc->L1_activity ? 0 : L1_ACTIVATION_TIME),
		  (void *)(void *)&cd_set_state_timeout, cd);
  }

  NDBGL3(L3_MSG, "cdid=%d, [%s]",
	 cd->cdid, L3_STATES_DESC[newstate]);

  if((newstate == ST_L3_UA_TO) ||
     (newstate == ST_L3_U3_TO) ||
     (newstate == ST_L3_U4_TO) ||
     (newstate == ST_L3_U7_TO))
  {
	/* need to check status regularly */
	dss1_l3_tx_status_enquiry(cd);
  }
  return;
}

/*
 * NOTE: pipe might be zero!
 */
static void
cd_update(call_desc_t *cd, DSS1_TCP_pipe_t *pipe, int event)
{
	l2softc_t *sc = ((__typeof(pipe))cd->pipe)->L5_sc;
	__typeof(cd->state)
	  state = cd->state;

	/*
	 * debugging
	 */

	NDBGL3(L3_F_MSG, "cdid=%d, [%s][%s] ",
	       cd->cdid,
	       L3_STATES_DESC[state],
	       L3_EVENTS_DESC[event]);

	/* check if event has
	 * right direction
	 */
	if(cd->dir_incoming)
	{
	  if(!L3_EVENT_IS_LOCAL_INCOMING(event))
	  {
	    goto F_UEM;
	  }
	}
	else
	{
	  if(!L3_EVENT_IS_LOCAL_OUTGOING(event))
	  {
	    goto F_UEM;
	  }
	}

	switch(event)
	{
/*
 * incoming and outgoing calls
 */
	case EV_L3_SETRJRS:
	  /* call rejected */
	  cd->cause_out = CAUSE_Q850_CALLREJ;
	  cd->need_release = 1;
	  cd_set_state(cd,ST_L3_U0);
	  break;

	case EV_L3_RELEASE:
	  /**/
	  cd_set_state(cd,ST_L3_U0);
	  break;

	case EV_L3_STATENQ:
	  /* send status enquiry response */
	  dss1_l3_tx_status(cd, CAUSE_Q850_STENQRSP);
	  break;

	case EV_L3_STATUS:
#define STATUS_ENQUIRY_TIMEOUT 8 /* timeouts */

	  if((state == ST_L3_U3_TO) ||
	     (state == ST_L3_U4_TO) ||
	     (state == ST_L3_U7_TO))
	  {
	      /* NOTE: there are more callstates than 0x0..0xA+0x19 ! */
	      if(((cd->call_state == 0x19) ||
		  ((cd->call_state >= 0x01) &&
		   (cd->call_state <= 0x09))) &&
		 (cd->status_enquiry_timeout < STATUS_ENQUIRY_TIMEOUT))
	      {
		  cd->status_enquiry_timeout++;

		  cd_set_state(cd, state -1);
	      }
	  }

	  if(state == ST_L3_UA_TO)
	  {
	      if(cd->call_state == 0xA)
	      {
		cd_set_state(cd, state -1);
	      }
	  }
	  break;

/*
 * outgoing calls
 */
	case EV_L3_SETUPRQ:
	  if(state == ST_L3_OUTGOING)
	  {
	    if(TE_MODE(sc) || IS_PRIMARY_RATE(sc))
	    {
	      dss1_l3_tx_setup(cd);
	    }

	    cd->cause_out = 0;
	    cd->need_release = 1;
	    cd_set_state(cd,
			 cd->dst_telno[0] != '\0' ? 
			 ST_L3_U1 /* non-overlap sending */:
			 ST_L3_U2 /* overlap sending */);
	  }
	  break;

	case EV_L3_INFORQ:
	  if((state == ST_L3_U2) ||
	     (state == ST_L3_U2_ACK))
	  {
	    if(TE_MODE(sc) || IS_PRIMARY_RATE(sc))
	    {
	      /* send next part of number, which
	       * should be added to "cd->dst_telno"
	       */
	      dss1_l3_tx_information(cd);
	    }

	    /* re-start timeout */
	    cd_set_state(cd,state);
	  }
	  break;

	case EV_L3_SETUPAK:
	  if(state == ST_L3_U2)
	  {
	    /* set state before indication */
	    cd_set_state(cd,ST_L3_U2_ACK);

	    /* try to allocate a channel before
	     * sending proceeding indication
	     */
	    if(cd->channel_id != CHAN_ANY)
	    {
		cd_allocate_channel(cd);
	    }

	    /* overlap sending */
	    i4b_l4_proceeding_ind(cd, 0/* sending not complete */);
	    break;
	  }

	  if(state == ST_L3_U2_ACK)
	  {
	    /* overlap sending */
	    break;
	  }

	case EV_L3_CALLPRC:
	case EV_L3_PROGIND:
	  if((state > ST_L3_OUTGOING) &&
	     (state < ST_L3_U3))
	  {
	    /* set state before indication */
	    cd_set_state(cd,ST_L3_U3);

	    /* try to allocate a channel before
	     * sending proceeding indication
	     */
	    if(cd->channel_id != CHAN_ANY)
	    {
		cd_allocate_channel(cd);
	    }

	    /*
	     * SETUP ACK:
	     *
	     * several PBX's react with a SETUP ACK, instead of CALL
	     * PROCEEDING, even if the called number is complete AND
	     * we sent a SENDING COMPLETE in the preceding SETUP
	     * message (-hm)
	     */
	    i4b_l4_proceeding_ind(cd, 1/* sending complete */);
	  }
	  break;

	case EV_L3_ALERT:
	  if((state > ST_L3_OUTGOING) &&
	     (state < ST_L3_U4))
	  {
	     if(cd->sms[0] != 0)
	     {
	       goto sms_release;
	     }

	     if(state < ST_L3_U3)
	     {
	         /* try to allocate a channel before
		  * sending proceeding indication
		  */
	         if(cd->channel_id != CHAN_ANY)
		 {
		     cd_allocate_channel(cd);
		 }

	         /* make sure the application doesn't 
		  * miss the proceeding and sending complete
		  * indication
		  */
	         i4b_l4_proceeding_ind(cd, 1);
	     }

	     /* set state before indication */
	     cd_set_state(cd,ST_L3_U4);

	     i4b_l4_alert_ind(cd);
	  }
	  break;

	case EV_L3_CONNECT:
	  if((state > ST_L3_OUTGOING) &&
	     (state < ST_L3_UA))
	  {
	    if(cd->sms[0] != 0)
	    {
	      goto sms_release;
	    }

	    if(cd->channel_id != CHAN_ANY)
	    {
		/* reserve channel */
		cd_allocate_channel(cd);

		if(cd->channel_allocated)
		{
		  /* disconnect non-connected pipes */
		  if(NT_MODE(sc))
		  {
			DSS1_TCP_pipe_t *p1;
			DSS1_TCP_pipe_t *p2;

			p1 = cd->pipe;

			PIPE_FOREACH(p2,&sc->sc_pipe[0])
			{
			  /* skip ``p1 == pipe_adapter'' and
			   * connected pipe
			   */
			  if((p2 != p1) &&
			     (p2 != pipe) &&
			     (p2->state != ST_L2_PAUSE))
			  {
			    cd->pipe = p2;

			    /* send RELEASE_COMPLETE */
			    dss1_l3_tx_release_complete(cd,0);
			  }
			}

			/* restore pipe pointer */
			cd->pipe = p1;
		  }

		  /* set pipe before state and tx */
		  cd_set_pipe(cd,pipe);

		  /* CONNECT_ACKNOWLEDGE to remote */
		  dss1_l3_tx_connect_acknowledge(cd);

		  goto event_connect_ack;
		}
	    }

	    /* if the one end doesn't select a channel,
	     * the other end must select a channel else
	     * there is an error!
	     */
	    goto no_channel_available;
	  }
	  break;

/*
 * incoming calls
 */
	case EV_L3_SETUP:
	  if((state == ST_L3_INCOMING) ||
	     (state == ST_L3_IN_ACK))
	  {
	    /* the other end is allowed
	     * to retransmit the SETUP 
	     * message with a new
	     * destination telephone
	     * number !
	     */

	    if(NT_MODE(sc))
	    {
	        /* some terminals require a
		 * RELEASE COMPLETE response before
		 * CONNECT is sent
		 */
	        cd->need_release = 1;
	    }

	    /* reserve channel */
	    cd_allocate_channel(cd);

	    if(cd->channel_allocated == 0)
	    {
	    no_channel_available:
	        /* no circuit-channel available */
	        cd->cause_out = CAUSE_Q850_NOCAVAIL;
		cd->need_release = 1;
		cd_set_state(cd,ST_L3_U0);
		break;
	    }

	    if(cd->dst_telno[0] == '\0')
	    {
	        /* overlap sending */

		/* need to send SETUP_ACKNOWLEDGE 
		 * with the B-channel to use
		 */
		dss1_l3_tx_setup_acknowledge(cd);

		/* set new state to allow 
		 * information messages
		 */
		cd_set_state(cd, ST_L3_IN_ACK);
	    }
	    else
	    {
		/* acknowledge the SETUP message
		 * and send B-channel to use
		 */
		dss1_l3_tx_call_proceeding(cd);

		/* set state before indication */
		cd_set_state(cd,ST_L3_U6);
	    }

	    /* tell l4 we have an incoming setup */	
	    i4b_l4_connect_ind(cd);
	  }
	  break;
#if 0
	case EV_L3_CALL_PROCEEDING_REQ:
	  if(state == ST_L3_IN_ACK)
	  {
	      /* sending complete */
	      dss1_l3_tx_call_proceeding(cd);

	      /* set new state */
	      cd_set_state(cd,ST_L3_U6);
	  }
	  break;
#endif
	default:
	case EV_L3_INFO:
	  if(state == ST_L3_IN_ACK)
	  {
	      if(cd->dst_telno[0] != '\0')
	      {
		/* re-start timeout */
		cd_set_state(cd,state);

		i4b_l4_information_ind(cd);
	      }
	  }
	  break;
#if 0
	case EV_L3_PROGRESSRQ:
	  /* (state > ST_L3_INCOMING) */
	  if(state == ST_L3_U6)
	  {
	  }
	  break;
#endif
	case EV_L3_ALERTRQ: /* local-CMD */
	  if((state > ST_L3_INCOMING) &&
	     (state < ST_L3_U7))
	  {
	     if(cd->sms[0] != 0)
	     {
	       goto sms_release;
	     }

	     if(state < ST_L3_U6)
	     {
	       /* always send call proceeding 
		* before any other messages
		* for sake of compatibility
		*/
	       dss1_l3_tx_call_proceeding(cd);
	     }

	     dss1_l3_tx_alert(cd);

	     cd_set_state(cd,ST_L3_U7);
	  }
	  break;

	case EV_L3_SETACRS:  /* local-CMD */
	  if((state > ST_L3_INCOMING) &&
	     (state < ST_L3_U8))
	  {
	     if(cd->sms[0] != 0)
	     {
	       goto sms_release;
	     }

	     if(state < ST_L3_U6)
	     {
	       /* always send call proceeding 
		* before any other messages
		* for sake of compatibility
		*/
	       dss1_l3_tx_call_proceeding(cd);
	     }

	     dss1_l3_tx_connect(cd);

	     cd->cause_out = 0;
	     cd->need_release = 1;
	     cd_set_state(cd,ST_L3_U8);

	     if(NT_MODE(sc))
	     {
	       /* NOTE: some terminals might not respond with
		* CONNECT ACKNOWLEDGE after CONNECT
		*/
	       goto event_connect_ack;
	     }
	  }
	  break;

	  /* double CONACK is allowed */
	case EV_L3_CONACK:
	  if(state == ST_L3_U8)
	  {
	  event_connect_ack:
	    /* in TE-mode there is only one PIPE,
	     * so it is not necessary to check 
	     * for broadcast CONNECT or 
	     * CONNECT ACKNOWLEDGE? In NT-mode
	     * I-frame broadcast is not allowed.
	     */
	    if(cd->sms[0] != 0)
	    {
	    sms_release:
	      /* no answer from user */
	      cd->cause_out = CAUSE_Q850_NOANSWR;
	      cd->need_release = 1;
	      cd_set_state(cd,ST_L3_U0);
	      break;
	    }

	    /* cleanup L2 ? */

	    /* set state before indication */
	    cd_set_state(cd,ST_L3_UA);

	    i4b_l4_connect_active_ind(cd);
	  }
	  break;

	  /* illegal event occured or unexpected message */
	case EV_L3_ILL:
	F_UEM:
	  /* only display an error message */
	  NDBGL3(L3_F_ERR, "cdid=%d, [%s][%s] event error! (dir=%s)",
		 cd->cdid,
		 L3_STATES_DESC[state],
		 L3_EVENTS_DESC[event],
		 cd->dir_incoming ? "incoming" : "outgoing");

#if 0
	  /* message not compatible with call state */
	  dss1_l3_tx_status(cd, CAUSE_Q850_MSGNCWCS);
#endif
	  break;
	}

	return;
}
