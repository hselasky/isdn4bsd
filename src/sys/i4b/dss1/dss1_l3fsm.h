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
    if(sc->L1_activity && 
       (!(sc->L1_auto_activate)) && 
       (!(sc->sc_received_frame)) &&
       (!(sc->sc_cntl->no_power_save)))
    {
      if(sc->L1_deactivate_count)
      {
	/* if there was no call or frame for the last
	 * 15 seconds, deactivate the ISDN line:
	 */
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
      sc->sc_received_frame = 0;
    }

    /* start timer - should run when not auto-activated */
    usb_callout_reset(&sc->L1_activity_callout, 15*hz,
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
    usb_callout_stop(&sc->L1_activity_callout);

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

  CNTL_LOCK_ASSERT(cntl);

  if(cntl->N_fifo_translator)
  {
    /* connected */
    cd_set_state(cd,L3_STATES_TIMEOUT_STATE[cd->state]);
  }
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
  u_int8_t send_status_enquiry;

  /* stop timer */
  usb_callout_stop(&cd->set_state_callout);

  if(newstate == ST_L3_U0)
  {
	if((cd->state == ST_L3_UC_TO) ||
	   (cd->state == ST_L3_UA_TO) ||
	   (cd->state == ST_L3_U3_TO) ||
	   (cd->state == ST_L3_U4_TO) ||
	   (cd->state == ST_L3_U6_TO) ||
	   (cd->state == ST_L3_U7_TO))
	{
	    NDBGL3(L3_MSG,
		   "cdid=%d, no status reply - call disconnected",
		   cd->cdid);
	}

	cd->state = newstate;

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
	   * This driver sends RELEASE_COMPLETE 
	   * in all cases, instead of DISCONNECT 
	   * and RELEASE, when the call is terminated
	   *
	   * It is expected that the disconnect message
	   * contains a hangup cause, which is stored
	   * in "cd->cause_out"
	   *
	   * If "cd->pipe" is still set to the 
	   * broadcast pipe, it means that no 
	   * device answered the outgoing call 
	   * from the NT-side. It appears that 
	   * sending a broadcast RELEASE_COMPLETE 
	   * will trigger some bugs, so instead
	   * send an individual RELEASE_COMPLETE
	   * message to all active pipes
	   */
	  if(NT_MODE(sc) &&
	     (!IS_POINT_TO_POINT(sc)) &&
	     (!cd->dir_incoming) &&
	     (pipe == &(sc->sc_pipe[0])))
	  {
	      dss1_l3_tx_release_complete_complement(cd, pipe, NULL);
	  }
	  else
	  {
	      dss1_l3_tx_release_complete(cd, 1);
	  }
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
	send_status_enquiry = 
	  ((newstate == ST_L3_UC_TO) ||
	   (newstate == ST_L3_UA_TO) ||
	   (newstate == ST_L3_U3_TO) ||
	   (newstate == ST_L3_U4_TO) ||
	   (newstate == ST_L3_U6_TO) ||
	   (newstate == ST_L3_U7_TO));

	if(NT_MODE(sc) && 
	   (!IS_POINT_TO_POINT(sc)) &&
	   ((newstate == ST_L3_U3_TO) ||
	    (newstate == ST_L3_U4_TO)))
	{
	    /* send no STATUS ENQUIRY, 
	     * hence some devices crash
	     * on it :-(
	     */
	    send_status_enquiry = 0;

	    if(cd->peer_responded &&
	       (cd->status_enquiry_timeout < STATUS_ENQUIRY_TIMEOUT))
	    {
	        cd->status_enquiry_timeout++;
		newstate--;
	    }
	}

	cd->state = newstate;

	/* re-start timeout;
	 * the timeout is increased when L1 is not activated
	 * the timeout is always running while the CD is allocated
	 */
	usb_callout_reset(&cd->set_state_callout,
			(L3_STATES_TIMEOUT_DELAY[newstate]*hz) +
			(sc->L1_activity ? 0 : L1_ACTIVATION_TIME),
			(void *)(void *)&cd_set_state_timeout, cd);

	if(send_status_enquiry)
	{
	    /* need to check status regularly */
	    dss1_l3_tx_status_enquiry(cd);
	}
	if((cd->dst_telno_early[0] != 0) &&
	    (newstate != ST_L3_U2))
	{
		if(TE_MODE(sc) || 
		   IS_POINT_TO_POINT(sc) ||
		   (newstate == ST_L3_UA) ||
		   (newstate == ST_L3_UA_TO)) {
			/* need to send accumulated digits */
			dss1_l3_tx_information(cd);
			/* clear any early digits */
			cd->dst_telno_early[0] = 0;
		}
	}
  }

  NDBGL3(L3_MSG, "cdid=%d, [%s]",
	 cd->cdid, L3_STATES_DESC[newstate]);

  /* store the Q.931 state for the tone generator */

  cd->tone_gen_state = L3_STATES_Q931_CONV[cd->state];

  return;
}

/*
 * NOTE: pipe might be zero!
 */
static void
cd_update(call_desc_t *cd, DSS1_TCP_pipe_t *pipe, int event)
{
	l2softc_t *sc = ((__typeof(pipe))(cd->pipe))->L5_sc;
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
	  cd->need_release = 1;
	  cd_set_state(cd,ST_L3_U0);
	  break;

	case EV_L3_DISCONNECT:
	  if((cd->state != ST_L3_UC) &&
	     (cd->state != ST_L3_UC_TO))
	  {
	      i4b_l4_pre_disconnect_ind(cd);
	      cd_set_state(cd,ST_L3_UC);
	  }
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

	  if((state == ST_L3_U3_TO) ||
	     (state == ST_L3_U4_TO) ||
	     (state == ST_L3_U6_TO) ||
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

	  if(((state == ST_L3_UA_TO) && (cd->call_state == 0xA)) ||
	     ((state == ST_L3_UC_TO) && ((cd->call_state == 0xB) ||  (cd->call_state == 0xC))))
	  {
	      cd_set_state(cd, state -1);
	  }
	  break;

	case EV_L3_HOLD_IND:
	  if(((cd->state == ST_L3_UA) ||
	      (cd->state == ST_L3_UA_TO) ||
	      ((cd->state >= ST_L3_U6) &&
	       (cd->state <= ST_L3_U8))) &&
	     (!cd->call_is_on_hold) && NT_MODE(sc))
	    {
	      if(cd->b_link_want_active)
	      {
		  /* disconnect B-channel if any */
		  i4b_link_bchandrvr(cd, 0);

		  /* still want the link active */
		  cd->b_link_want_active = 1;
	      }

	      /* free allocated channel if any */
	      cd_free_channel(cd);

	      /* send acknowledge back */
	      dss1_l3_tx_hold_acknowledge(cd);

	      /* forward hold notify indication */
	      i4b_l4_hold_ind(cd);

	      /**/
	      cd->call_is_on_hold = 1;
	  }
	  else
	  {
	      /* reject: message not compatible with call state */
	      dss1_l3_tx_hold_reject(cd, CAUSE_Q850_MSGNCMPT);
	  }
	  break;

	case EV_L3_RETRIEVE_IND:

	  if(cd->call_is_on_hold && NT_MODE(sc))
	  {
	      /* reserve channel */
	      cd_allocate_channel(cd);

	      if(cd->channel_allocated == 0)
	      {
		  /* reject: no circuit channel available */
		  dss1_l3_tx_retrieve_reject(cd, CAUSE_Q850_NOCAVAIL);
	      }
	      else
	      {
		  /* send acknowledge back */
		  dss1_l3_tx_retrieve_acknowledge(cd);

		  /* forward retrieve indication */
		  i4b_l4_retrieve_ind(cd);

		  /**/
		  cd->call_is_on_hold = 0;

		  if(cd->b_link_want_active)
		  {
		      i4b_link_bchandrvr(cd, 1);
		  }
	      }
	  }
	  else
	  {
	      /* reject: message not compatible with call state */
	      dss1_l3_tx_retrieve_reject(cd, CAUSE_Q850_MSGNCMPT);
	  }
	  break;


	case EV_L3_HOLD_REQ:
	  if(((cd->state == ST_L3_UA) ||
	      (cd->state == ST_L3_UA_TO) ||
	      ((cd->state >= ST_L3_U2_ACK) &&
	       (cd->state <= ST_L3_U4_TO))) &&
	     (!cd->call_is_on_hold) && TE_MODE(sc))
	    {
	      if(cd->b_link_want_active)
	      {
		  /* disconnect B-channel if any */
		  i4b_link_bchandrvr(cd, 0);

		  /* still want the link active */
		  cd->b_link_want_active = 1;
	      }

	      /* free allocated channel if any */
	      cd_free_channel(cd);

	      /* send hold request */
	      dss1_l3_tx_hold(cd);

	      /**/
	      cd->call_is_on_hold = 1;
	      cd->call_is_retrieving = 0;
	  }
	  break;

	case EV_L3_RETRIEVE_REQ:

	  if(cd->call_is_on_hold && TE_MODE(sc))
	  {
	      /* send retrive request */
	      dss1_l3_tx_retrieve(cd);

	      /**/
	      cd->call_is_on_hold = 0;
	      cd->call_is_retrieving = 1;
	  }
	  break;

	case EV_L3_RETRIEVE_ACK:

	  if(cd->call_is_retrieving && TE_MODE(sc))
	  {
	      /* reserve channel */
	      cd_allocate_channel(cd);

	      /**/
	      cd->call_is_retrieving = 0;

	      if(cd->b_link_want_active)
	      {
		  i4b_link_bchandrvr(cd, 1);
	      }
	  }
	  break;

/*
 * outgoing calls
 */
	case EV_L3_SETUPRQ:
	  if(state == ST_L3_OUTGOING)
	  {
	    if(TE_MODE(sc) || 
	       IS_POINT_TO_POINT(sc))
	    {
	      dss1_l3_tx_setup(cd);
	    }

	    cd->cause_out = 0;
	    cd->need_release = 1;
	    cd_set_state(cd,
			 cd->sending_complete ? 
			 ST_L3_U1 /* non-overlap sending */:
			 ST_L3_U2 /* overlap sending */);
	  }
	  break;

	case EV_L3_INFORQ:
	  if(TE_MODE(sc) || 
	     IS_POINT_TO_POINT(sc) ||
	     (state == ST_L3_UA) ||
	     (state == ST_L3_UA_TO))
	  {
	      /* send next part of number,
	       * "cd->dst_telno_part"
	       */
	      strlcat(cd->dst_telno_early,
	          cd->dst_telno_part, sizeof(cd->dst_telno_early));

	      /* check if it is too early to send digits */
	      if (state != ST_L3_U2) {
			dss1_l3_tx_information(cd);
			cd->dst_telno_early[0] = 0;
	      }
	  }

	  if((state == ST_L3_U2) ||
	     (state == ST_L3_U2_ACK))
	  {
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
	    i4b_l4_proceeding_ind(cd, 0, TE_MODE(sc));
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
	    i4b_l4_proceeding_ind(cd, 1, TE_MODE(sc));
	  }
	  break;

	case EV_L3_ALERT:
	  if((state > ST_L3_OUTGOING) &&
	     (state < ST_L3_U4))
	  {
	     if(cd->is_sms)
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
	         i4b_l4_proceeding_ind(cd, 1, TE_MODE(sc));
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
	    if(cd->is_sms)
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
		    if(NT_MODE(sc) && (!IS_POINT_TO_POINT(sc)))
		    {
		        cd->cause_out = CAUSE_Q850_NONSELUC;

			dss1_l3_tx_release_complete_complement
			  (cd, cd->pipe, pipe);

			cd->cause_out = 0;
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
	    u_int8_t want_dialtone =
	      (NT_MODE(sc) && (!i4b_controller_by_cd(cd)->no_layer1_dialtone) && 
	       ((cd->channel_bprot == BPROT_NONE) ||
		(cd->channel_bprot == BPROT_NONE_3_1_KHZ)));

	    /* the other end is allowed
	     * to retransmit the SETUP 
	     * message with a new
	     * destination telephone
	     * number !
	     */

	    if(NT_MODE(sc) || 
	       IS_POINT_TO_POINT(sc))
	    {
	        /* some terminals require a
		 * RELEASE COMPLETE response before
		 * CONNECT is sent, in case of
		 * failure
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
		if(NT_MODE(sc) || 
		   IS_POINT_TO_POINT(sc))
		{
		    /* the call cannot be accepted,
		     * hence there is no channel
		     * for it, and sending a
		     * RELEASE COMPLETE now
		     * will not arrive out
		     * of sequence at the
		     * other end:
		     */
		    cd->need_release = 1;
		}
		cd_set_state(cd,ST_L3_U0);
		break;
	    }

	    if(cd->sending_complete == 0)
	    {
	        /* overlap sending */

		/* need to send SETUP_ACKNOWLEDGE 
		 * with the B-channel to use
		 */
	        dss1_l3_tx_setup_acknowledge(cd,NT_MODE(sc),want_dialtone);

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
	        dss1_l3_tx_call_proceeding(cd,NT_MODE(sc),want_dialtone);

		/* set state before indication */
		cd_set_state(cd,ST_L3_U6);
	    }

	    if(want_dialtone) {

	        cd->driver_type = DRVR_DIAL_GEN;
		cd->driver_unit = 0;

		(void) i4b_link_bchandrvr(cd, 1);
	    }

	    /* tell l4 we have an incoming setup */	
	    cd->connect_ind_count = 
	        i4b_l4_connect_ind(cd);

	    if (cd->connect_ind_count == 0) {
	        /* no one wants the call */
	        goto sms_release;
	    }
	  }
	  break;

	default:
	case EV_L3_INFO:

	  /* catch digits */

	  if(cd->dst_telno_part[0] != '\0')
	  {
	      if(state == ST_L3_IN_ACK)
	      {
		  /* re-start timeout */
		  cd_set_state(cd,state);
	      }
	      i4b_l4_information_ind(cd);
	  }
	  break;

	case EV_L3_PROCEEDINGRQ: /* local-CMD */
	  if((state > ST_L3_INCOMING) &&
	     (state < ST_L3_U6))
	  {
	      /* sending complete */
	      dss1_l3_tx_call_proceeding(cd,NT_MODE(sc),0);

	      /* overlap sending is complete, set new state */
	      cd_set_state(cd,ST_L3_U6);
	  }
	  break;

	case EV_L3_ALERTRQ: /* local-CMD */
	  if((state > ST_L3_INCOMING) &&
	     (state < ST_L3_U7))
	  {
	     if(cd->is_sms)
	     {
	       goto sms_release;
	     }

	     if(state < ST_L3_U6)
	     {
	       /* always send call proceeding 
		* before any other messages
		* for sake of compatibility
		*/
	       dss1_l3_tx_call_proceeding(cd,NT_MODE(sc),0);
	     }

	     dss1_l3_tx_alert(cd);

	     cd_set_state(cd,ST_L3_U7);
	  }
	  break;

	case EV_L3_PROGRESSRQ: /* local-CMD */
	  if((state > ST_L3_INCOMING) &&
	     (state < ST_L3_UA))
	  {
	     dss1_l3_tx_progress(cd);
	  }
	  break;

	case EV_L3_SETACRS:  /* local-CMD */
	  if((state > ST_L3_INCOMING) &&
	     (state < ST_L3_U8))
	  {
	     if(cd->is_sms)
	     {
	       goto sms_release;
	     }

	     if(state < ST_L3_U6)
	     {
	       /* always send call proceeding 
		* before any other messages
		* for sake of compatibility
		*/
	       dss1_l3_tx_call_proceeding(cd,NT_MODE(sc),0);
	     }

	     dss1_l3_tx_connect(cd,NT_MODE(sc));

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
	    if(cd->is_sms)
	    {
	    sms_release:
	      /* no answer from user */
	      cd->cause_out = CAUSE_Q850_NOANSWR;
	      cd->need_release = 1;
	      cd_set_state(cd,ST_L3_U0);
	      break;
	    }

	    /* set state before indication */
	    cd_set_state(cd,ST_L3_UA);

	    i4b_l4_connect_active_ind(cd);
	  }
	  break;

	case EV_L3_DEFLECTRQ:
	  if(TE_MODE(sc))
	  {
	      dss1_l3_tx_deflect_call(cd);
	  }
	  break;

	case EV_L3_MCIDRQ:
	  if(TE_MODE(sc))
	  {
	      dss1_l3_tx_mcid_call(cd);
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

