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
 *	FSM for isdnd
 *	-------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*
 *
 *    ST_IDLE       ST_DIAL     ST_DIAL_    ST_         ST_WAIT     ST_        
 *                              FAIL        INCOMING    CONNECT     CONNECTED  
 *  =====+============+===========+===========+===========+===========+======  
 *       .            .           .           .           .           .        
 *       .dial from   .disc from  .           .           .           .        
 *       .user        .i4b        .           .           .           .        
 *       |----------->|---------->|           .           .           .        
 *       .            .           .           .           .           .        
 *       .            .  dial from.           .           .           .        
 *       .            .        i4b.           .           .           .        
 *       .            |<----------|           .           .           .        
 *       .            .           .           .           .           .        
 *       .            .connect    .           .           .           .        
 *       .            .from i4b   .           .           .           .        
 *       .            |-----------X-----------X-----------|---------->|        
 *       .            .           .           .           .           .        
 *       .incoming    .           .           .connect    .connect    .        
 *       .from i4b    .           .           .from user  .from i4b   .        
 *       |------------X-----------|---------->|---------->|---------->|        
 *       .            .           .           .           .           .        
 *       .            .           .           .           .  disc from.        
 *       .            .           .           .           .        i4b.        
 *       |<-----------X-----------X-----------|-----------|-----------|        
 *       .            .           .           .           .           .        
 *       .            .           .           .           .  disc from.        
 *       .            .           .           .           .       user.        
 *       |<-----------|-----------|-----------|-----------|-----------|        
 *       .            .           .           .           .           .        
 *       .            .           .           .           .           .        
 *                                                                             
 *   X: state is not changed                                                   
 *   |: state is changed to state at arrow-head                                
 *---------------------------------------------------------------------------*/

#include "isdnd.h"

/* table of state descriptions */

static const char * const
MAKE_TABLE(STATES,DESC,[]);

/*---------------------------------------------------------------------------*
 *	cep_set_state
 *
 * NOTE: when state is set to ST_IDLE, ``response_to_user()''
 *	 must also be called!
 *---------------------------------------------------------------------------*/
static void
cep_set_state(cfg_entry_t *cep, int state, u_int waittime, u_int waitfunc)
{
	DBGL(DL_STATE, (log(LL_DBG, "[%s] -> [%s]",
			    STATES_DESC[cep->state],
			    STATES_DESC[state])));

	cep->state = state;
	cep->last_set_state_time = time(NULL);
	cep->waitfunc = waitfunc;
	cep->waittime = waittime;
	return;
}

#include <msg_encoder.h>

static const char * const
MAKE_TABLE(I4B_CAUSES,DESC,[]);

static const char * const
MAKE_TABLE(Q850_CAUSES,DESC,[]);

static char *
print_cause(cause_t code)
{
	static char error_message[128];
	cause_t cause_val = GET_CAUSE_VAL(code);
	const char *ptr;

	snprintf(error_message, sizeof(error_message), "%d: ", cause_val);
	ptr = 0;

	switch(GET_CAUSE_TYPE(code))
	{
		case CAUSET_Q850:
			strcat(error_message, "Q.850: ");

			if(/* (cause_val >= 0) && */
			   (cause_val < (cause_t)INDEXES(Q850_CAUSES_DESC)))
			{
				ptr = Q850_CAUSES_DESC[cause_val];
			}
			break;

		case CAUSET_I4B:
			strcat(error_message, "I4B: ");

			if(/* (cause_val >= 0) && */
			   (cause_val < (cause_t)INDEXES(I4B_CAUSES_DESC)))
			{
				ptr = I4B_CAUSES_DESC[cause_val];
			}
			break;

		default:
			strcat(error_message, "unknown: ");
			break;
	}

	strcat(error_message, ptr ? ptr : "unknown cause value!");
	return(error_message);
}

static void
disconnect_code(cfg_entry_t *cep)
{
#ifdef I4B_EXTERNAL_MONITOR
	if(do_monitor && accepted)
		monitor_evnt_disconnect(cep);
#endif

	/* not sure when disconnect program should be run: */
	if(cep->disconnectprog)
		exec_connect_prog(cep, cep->disconnectprog, 1);

#ifdef USE_CURSES
	if(do_fullscreen)
		display_disconnect(cep);
#endif

	log(LL_CHD, "%05d %s %s call disconnected %s, cause %s",
	    cep->cdid,
	    cep->name,
	    cep->dir_incoming ? "incoming" : "outgoing",
	    cep->disc_location == DISC_LOCATION_LOCAL ? "(local)" : "(remote)",
	    print_cause(cep->disc_cause));

	if(cep->dir_incoming)
	{
		log(LL_CHD, "%05d %s connected %d seconds",
		    cep->cdid, cep->name,
		    (int)difftime(time(NULL), cep->last_set_state_time));
	}
	else
	{
		log(LL_CHD, "%05d %s charging: %d units, %d seconds",
		    cep->cdid, cep->name, cep->charge,
		    (int)difftime(time(NULL), cep->last_set_state_time));
	}

	if((cep->inbytes != INVALID) &&
	   (cep->outbytes != INVALID))
	{
		if((cep->ioutbytes != cep->outbytes) ||
		   (cep->iinbytes != cep->inbytes))
		{
			log(LL_CHD, "%05d %s accounting: in %d, out %d (in %d, out %d)",
			    cep->cdid, cep->name,
			    cep->inbytes, cep->outbytes,
			    cep->iinbytes, cep->ioutbytes);
		}
		else
		{
			log(LL_CHD, "%05d %s accounting: in %d, out %d",
			    cep->cdid, cep->name,
			    cep->inbytes, cep->outbytes);
		}
	}

	if(useacctfile)
	{
		int con_secs;
		char logdatetime[41];
		struct tm *tp;

		con_secs = difftime(time(NULL), cep->last_set_state_time);

		tp = localtime(&cep->last_set_state_time);
		
		strftime(logdatetime,40,I4B_TIME_FORMAT,tp);

		if((cep->inbytes != INVALID) && 
		   (cep->outbytes != INVALID))
		{
			fprintf(acctfp, "%s - %s %s %d (%d) (%d/%d)\n",
				logdatetime, getlogdatetime(),
				cep->name, cep->charge, con_secs,
				cep->inbytes, cep->outbytes);
		}
		else
		{
			fprintf(acctfp, "%s - %s %s %d (%d)\n",
				logdatetime, getlogdatetime(),
				cep->name, cep->charge, con_secs);
		}
	}

	/* set the B-channel inactive */
	set_channel_state(cep->isdncontrollerused, cep->isdnchannelused, CHAN_IDLE);

	/* check for an outstanding process, and kill it */
	if((cep->answerpid != 0) &&
	   (cep->answerpid != ((pid_t)-1)))
	{
		DBGL(DL_PROC, (log(LL_DBG, "killing pid %d", cep->answerpid)));

		kill(cep->answerpid, SIGHUP);

		cep->answerpid = 0;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	dial_fail_code
 *
 * NOTE: this routine must set ``cep->state''
 *---------------------------------------------------------------------------*/	
static void
dial_fail_code(cfg_entry_t *cep)
{
#if 0
	/* from F_ACBW: */
	if(cep->dialin_reaction == REACT_CALLBACK)
	{
		DBGL(DL_STATE, (log(LL_DBG, "dialing failed, waiting for dail-retry timeout")));

		cep->dial_count = 0; /* infinite dial-retry */
		select_first_dialno(cep);

		waittime = cep->callbackwait;
		return;
	}
#endif

	cep->dial_count++;
	
	if((cep->dial_count < cep->dialretries) || 
	   (cep->dialretries == -1))
	{
		/* inside normal retry cycle */

		__typeof(cep->calledbackwait) waittime;

		if(CEP_IS_CALLED_BACK(cep))
		{
		  waittime = cep->calledbackwait;
		}
		else
		{
		  waittime = cep->recoverytime;

		  if(cep->dialrandincr)
		  {
		    waittime += (random() & RANDOM_MASK);
		  }
		}

		DBGL(DL_STATE, (log(LL_DBG, "dialing failed, waiting for dail-retry timeout")));

		cep_set_state(cep,ST_DIAL_FAIL,waittime,2);

		select_next_dialno(cep);
		return;
	}
	
	DBGL(DL_STATE, (log(LL_DBG, "end of dial-retry cycle")));

	/* retries exhausted */

	if(cep->usedown)
	{
		/* set interface down */
		DBGL(DL_MSG, (log(LL_DBG, "taking %s%d down",
				  driver_name(cep->usrdevicename), cep->usrdeviceunit)));

		/* NOTE: a response to user will also be sent
		 * at timeout!
		 */
		response_to_user(cep, DSTAT_PFAIL);

#ifdef USE_CURSES
		if(do_fullscreen)
			display_updown(cep, 0);
#endif
#ifdef I4B_EXTERNAL_MONITOR
		monitor_evnt_updown(cep, 0);
#endif

		cep_set_state(cep,ST_DIAL_FAIL,cep->downtime,1);
	}
	else
	{
		response_to_user(cep, DSTAT_TFAIL);
		cep_set_state(cep,ST_IDLE,0,0);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_DISCONNECT_IND from i4b
 *---------------------------------------------------------------------------*/
static void
ev_disconnect_from_i4b(msg_disconnect_ind_t *mp, cfg_entry_t *cep_exception)
{
  CEP_FOREACH(cep,&cfg_entry_tab[0])
  {
    if((cep->cdid == (int)mp->header.cdid) &&
       (cep != cep_exception))
    {
	cep->disc_cause = mp->cause;
	cep->disc_location = DISC_LOCATION_REMOTE;

	if(cep->state == ST_DIAL)
	{
		/* dial_fail_code() sets ``cep->state'' ! */
		dial_fail_code(cep);
	}

	if(cep->state == ST_CONNECTED)
	{
		disconnect_code(cep);

		/* send disconnect to user */
		response_to_user(cep,DSTAT_NOT_CONNECTED);
		cep_set_state(cep,ST_IDLE,0,0);
	}

	if((cep->state == ST_INCOMING) ||
	   (cep->state == ST_WAIT_CONNECT))
	{
		/* send disconnect to user */
		response_to_user(cep,DSTAT_NOT_CONNECTED);
		cep_set_state(cep,ST_IDLE,0,0);
	}

	/* optional cdid clearing: */
	cep->cdid = CDID_UNUSED;
    }
  }
  return;
}

void
ev_disconnect_from_user(cfg_entry_t *cep)
{
  /* preset disconnect cause for ``response_to_user()'' */
  cep->disc_cause = (CAUSET_I4B << 8) | CAUSE_I4B_NORMAL;
  cep->disc_location = DISC_LOCATION_LOCAL;

  if((cep->state == ST_DIAL) ||
     /* (state == ST_INCOMING) ||
      * cannot send any response, because more than one
      * entry may respond to the same incoming call
      */
     (cep->state == ST_WAIT_CONNECT) ||
     (cep->state == ST_CONNECTED))
  {
	/* send disconnect to i4b */
	sendm_disconnect_req(cep, (CAUSET_I4B << 8) | CAUSE_I4B_NORMAL);
  }

  if(cep->state == ST_CONNECTED)
  {
	disconnect_code(cep);
  }

  if((cep->state == ST_DIAL_FAIL) &&
     (cep->usedown) &&
     (cep->dial_count >= cep->dialretries) &&
     (cep->dialretries != -1))
  {
	/* set interface up */
	
	DBGL(DL_MSG, (log(LL_DBG, "taking %s%d up",
			  driver_name(cep->usrdevicename), cep->usrdeviceunit)));
	
#ifdef USE_CURSES
	if(do_fullscreen)
		display_updown(cep, 1);
#endif
#ifdef I4B_EXTERNAL_MONITOR
	monitor_evnt_updown(cep, 1);
#endif
  }

  /* send disconnect [back] to user 
   *
   * NOTE: disconnect is sent back more
   * times than needed
   */

  response_to_user(cep,DSTAT_NOT_CONNECTED);
  cep_set_state(cep,ST_IDLE,0,0);

  /* optional cdid clearing */
  cep->cdid = CDID_UNUSED;
  return;
}

static void
ev_reject_from_user(cfg_entry_t *cep)
{
  if(cep->state == ST_INCOMING)
  {
    msg_disconnect_ind_t mp;

    /* send reject to i4b */
    sendm_connect_resp(cep, SETUP_RESP_REJECT,
		       (CAUSET_I4B << 8) | CAUSE_I4B_REJECT);

    mp.header.type = MSG_DISCONNECT_IND;
    mp.header.cdid = cep->cdid;
    mp.cause = (CAUSET_I4B << 8) | CAUSE_I4B_REJECT;

    /* send reject to user */
    ev_disconnect_from_i4b(&mp,NULL);
  }
  return;
}

static void
ev_alert_from_user(cfg_entry_t *cep)
{
  if(cep->state == ST_INCOMING)
  {
    /* send alert to i4b */
    if(sendm_alert_req(cep) < 0)
    {
      /* disconnect ? */
    }
  }
  return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_ALERT_IND from i4b
 *---------------------------------------------------------------------------*/
static void
ev_alert_from_i4b(msg_alert_ind_t *mp)
{
  cfg_entry_t *cep;
	
  cep = get_cep_by_cdid(mp->header.cdid);

  if(cep)
  {
	log(LL_CHD, "%05d %s incoming alert (controller %d)",
	    cep->cdid, cep->name,
	    cep->isdncontrollerused);

	if(cep->state == ST_DIAL)
	{
	  /* send alert to user ? */
	}
  }
  return;
}

static void
ev_connect_from_user(cfg_entry_t *cep)
{
  if(cep->state == ST_INCOMING)
  {
	msg_disconnect_ind_t mp;

	/* send connect to i4b */
	if(sendm_connect_resp(cep, SETUP_RESP_ACCEPT, 0) < 0)
	{
	  /* timeout below will disconnect */
	}

	/* if i4b doesn't respond, disconnect */
	cep_set_state(cep,ST_WAIT_CONNECT,TIMEOUT_CONNECT_ACTIVE,1);

	mp.header.type = MSG_DISCONNECT_IND;
	mp.header.cdid = cep->cdid;
	mp.cause = (CAUSET_I4B << 8) | CAUSE_I4B_NORMAL;

	/* disconnect other ceps with same cdid ! */
	ev_disconnect_from_i4b(&mp, cep);
  }
  return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_CONNECT_ACTIVE_IND from i4b
 *---------------------------------------------------------------------------*/
static void
ev_connect_from_i4b(msg_connect_active_ind_t *mp)
{
  cfg_entry_t *cep;

  cep = get_cep_by_cdid(mp->header.cdid);

  if(cep == NULL)
  {
	return;
  }

  if((cep->state == ST_DIAL) && CEP_IS_CALLED_BACK(cep))
  {
	log(LL_ERR, "ERROR, %s, both sides cannot run calledback "
	    "configuration - disconnecting!",
	    cep->name);
	/* this entry expects to be called back */
	ev_disconnect_from_user(cep);
  }

  if((cep->state == ST_WAIT_CONNECT) ||
     (cep->state == ST_DIAL))
  {
	msg_link_b_channel_driver_req_t mlr;

	memset(&mlr, 0, sizeof(mlr));

	mlr.cdid = mp->header.cdid;
	mlr.activate = 1;

	if(ioctl(isdnfd, I4B_LINK_B_CHANNEL_DRIVER_REQ, &mlr) < 0)
	{
	    /* issue a disconnect request ? */

	    log(LL_ERR, "cdid=%d: ioctl I4B_LINK_B_CHANNEL_DRIVER_REQ "
		"failed: %s", mp->header.cdid, strerror(errno));
	}

	/* send connect to user */
	response_to_user(cep,DSTAT_CONNECTED);

	/* if maxconnecttime is set,
	 * disconnect after maxconnecttime
	 */
	cep_set_state(cep,ST_CONNECTED,cep->maxconnecttime,
		      (cep->maxconnecttime > 0) ? 1 : 0);

	cep->isdncontrollerused = CDID2CONTROLLER(mp->header.cdid);
	cep->isdnchannelused = mp->channel;	

	cep->aoc_now = time(NULL);
	cep->aoc_last = 0;
	cep->aoc_diff = 0;
	cep->aoc_valid = AOC_INVALID;

	cep->inbytes = INVALID;
	cep->outbytes = INVALID;

	/* set the B-channel to active */
	set_channel_state(cep->isdncontrollerused, cep->isdnchannelused, CHAN_RUN);

#ifdef USE_CURSES
	if(do_fullscreen)
		display_connect(cep);
#endif
#ifdef I4B_EXTERNAL_MONITOR
	if(do_monitor && accepted)
		monitor_evnt_connect(cep);
#endif

	if(isdntime && (mp->datetime[0] != '\0'))
	{
		log(LL_DMN, "date/time from exchange = %s", mp->datetime);
	}

	if(cep->dir_incoming)
	{
		log(LL_CHD, "%05d %s incoming call active (ctl %d, ch %d, %s%d)",
			cep->cdid, cep->name,
			cep->isdncontrollerused, cep->isdnchannelused,
			driver_name(cep->usrdevicename), cep->usrdeviceunit);

		if(cep->dialin_reaction == REACT_ANSWER)
		{
			exec_answer(cep);
		}
	}
	else
	{
		log(LL_CHD, "%05d %s outgoing call active (ctl %d, ch %d, %s%d)",
			cep->cdid, cep->name,
			cep->isdncontrollerused, cep->isdnchannelused,
			driver_name(cep->usrdevicename), cep->usrdeviceunit);

		if(cep->budget_calltype)
		{
			if(cep->budget_calltype == BUDGET_TYPE_CBACK)
			{
				cep->budget_callback_done++;
				cep->budget_callbackncalls_cnt--;
				DBGL(DL_BDGT, (log(LL_DBG, "%s: new cback-budget = %d",
					cep->name, cep->budget_callbackncalls_cnt)));
				if(cep->budget_callbacks_file != NULL)
					upd_callstat_file(cep->budget_callbacks_file, cep->budget_callbacksfile_rotate);
			}
			else if(cep->budget_calltype == BUDGET_TYPE_COUT)
			{
				cep->budget_callout_done++;
				cep->budget_calloutncalls_cnt--;
				DBGL(DL_BDGT, (log(LL_DBG, "%s: new cout-budget = %d",
					cep->name, cep->budget_calloutncalls_cnt)));
				if(cep->budget_callouts_file != NULL)
					upd_callstat_file(cep->budget_callouts_file, cep->budget_calloutsfile_rotate);
			}
			cep->budget_calltype = 0;
		}
		select_this_dialno(cep);
	}
  }
  return;
}

/*---------------------------------------------------------------------------*
 *	dial_code
 *
 * NOTE: this routine must set ``cep->state''
 * NOTE: there is no dial-timeout in isdnd
 *---------------------------------------------------------------------------*/	
static void
dial_code(cfg_entry_t *cep)
{
	DBGL(DL_DRVR, (log(LL_DBG, "%s: dialout request for %s, unit %d",
			   cep->name, driver_name(cep->usrdevicename), cep->usrdeviceunit)));

	/* find entry by drivertype and driverunit and setup for dialing out */
    
	if(cep->inout == DIR_INONLY)
	{
		response_to_user(cep, DSTAT_INONLY);
		cep_set_state(cep,ST_IDLE,0,0);
		goto done;
	}

	if(isvalidtime(cep) == 0)
	{
		response_to_user(cep,DSTAT_NO_ENTRY);
		cep_set_state(cep,ST_IDLE,0,0);
		goto done;
	}

	/* kernel will check if channel and controller is available */

	cep->isdncontrollerused = cep->isdncontroller;
	cep->isdnchannelused = cep->isdnchannel;

	/* preset disconnect cause for ``response_to_user()'' */
	cep->disc_cause = (CAUSET_I4B << 8) | CAUSE_I4B_NORMAL;
	cep->disc_location = DISC_LOCATION_LOCAL;

	cep->charge = 0;
	cep->last_charge = 0;
	cep->unitlength = get_current_rate(cep, 1);
	cep->dir_incoming = 0;

	if(sendm_connect_req(cep) < 0)
	{
		/* dial_fail_code() sets ``cep->state'' ! */
		dial_fail_code(cep);
		goto done;
	}

	cep_set_state(cep,ST_DIAL,0,0);

 done:
	/* must clear keypad element! */
	cep->keypad[0] = '\0';
	return;
}

static void
ev_dial_from_user(cfg_entry_t *cep)
{
  if(cep->state == ST_IDLE)
  {
	  /* no budget */
	  cep->budget_calltype = 0;

	  /* reset dial_count */
	  cep->dial_count = 0;

	  /* select first dial-number */
	  select_first_dialno(cep);

	  if(cep->dialin_reaction == REACT_CALLBACK)
	  {
	    if(cep->budget_callbackperiod &&
	       cep->budget_callbackncalls)
	    {
		cep->budget_callback_req++;

		if(cep->budget_callbackncalls_cnt == 0)
		{
			log(LL_CHD, "%s no budget for callback",
			    cep->name);

			cep->budget_callback_rej++;
			/* dial_fail_code() sets ``cep->state'' ! */
			dial_fail_code(cep);
			goto done;
		}
		else
		{
			cep->budget_calltype = BUDGET_TYPE_CBACK;
		}
	    }

	    log(LL_CHD, "%s callback", cep->name);

	    cep_set_state(cep,ST_DIAL_FAIL,cep->callbackwait,2);
	  }
	  else
	  {
	    if(cep->budget_calloutperiod &&
	       cep->budget_calloutncalls)
	    {
		cep->budget_calltype = 0;
		cep->budget_callout_req++;
		
		if(cep->budget_calloutncalls_cnt == 0)
		{
			log(LL_CHD, "%s no budget for calling out",
			    cep->name);

			cep->budget_callout_rej++;
			/* dial_fail_code() sets ``cep->state'' ! */
			dial_fail_code(cep);
			goto done;
		}
		else
		{
			cep->budget_calltype = BUDGET_TYPE_COUT;
		}
	    }

	    /* dial_code() sets ``cep->state'' */
	    dial_code(cep);
	  }
  }

 done:
  return;
}

static void
ev_dial_from_i4b(cfg_entry_t *cep)
{
  if(cep->state == ST_DIAL_FAIL)
  {
	/* dial_code() sets ``cep->state'' */
	dial_code(cep);
  }
  return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_CONNECT_IND from i4b
 *---------------------------------------------------------------------------*/
static void
ev_incoming_from_i4b(msg_connect_ind_t *mp)
{
  char *src_tel;
  char *dst_tel;

  /* Add prefixes. All preexisting alias files are useless
   * if this is on.
   */
  if(addprefix)
  {
	add_number_prefix(mp->src_telno, mp->src_ton);
	add_number_prefix(mp->dst_telno, mp->dst_ton);
  }

  if(aliasing)
  {
	src_tel = get_alias(mp->src_telno);
	dst_tel = get_alias(mp->dst_telno);
  }
  else
  {
	src_tel = mp->src_telno;
	dst_tel = mp->dst_telno;
  }

  /* print out who is calling who */

  log(LL_CHD, "Incoming call from '%s' to '%s' (cdid=%05d)",
      src_tel, dst_tel, mp->header.cdid);

  /* check for CW (call waiting) early */

  if(mp->channel == CHAN_NOT_ANY)
  {
    log(LL_CHD, "%05d <unknown> call waiting from %s to %s (no channel free)",
	mp->header.cdid, src_tel, dst_tel);
    goto done;
  }

  /* NOTE: more than one entry is allowed
   * to answer a call, though only
   * one entry can connect!
   */
  CEP_FOREACH(cep,&cfg_entry_tab[0])
  {
    if(cep->inout == DIR_OUTONLY)
    {
	log(LL_CHD, "%05d %s incoming call from %s to %s not "
	    "allowed by configuration!",
	    mp->header.cdid, cep->name, src_tel, dst_tel);

	continue;
    }

    if((cep->state == ST_IDLE) ||
       (cep->state == ST_DIAL_FAIL))
    {
      if(number_matches(mp,cep))
      {
      connect:
	/* if no action within ``cep->alert'' seconds,
	 * connect
	 */
	cep_set_state(cep,ST_INCOMING,cep->alert,3);
	cep->charge = 0;
	cep->last_charge = 0;
	cep->cdid = mp->header.cdid;
	cep->isdncontrollerused = CDID2CONTROLLER(mp->header.cdid);
	cep->isdnchannelused = mp->channel;
	cep->dir_incoming = 1;

	/* copy number to real one used */

	strlcpy(cep->real_phone_incoming.number, mp->src_telno,
		sizeof(cep->real_phone_incoming.number));

	/* copy display string */

	strlcpy(cep->display, mp->display,
		sizeof(cep->display));

	/* copy user-user string */

	strlcpy(cep->user_user, mp->user_user,
		sizeof(cep->user_user));

	switch(cep->dialin_reaction)
	{
	case REACT_ACCEPT:
		log(LL_CHD, "%05d %s is accepting incoming call from %s to %s",
		    cep->cdid, cep->name, src_tel, dst_tel);

		/**/
		ev_connect_from_user(cep);

		/* must goto done, because of connect */
		goto done;
		break;

	case REACT_REJECT:
		log(LL_CHD, "%05d %s is rejecting incoming call from %s to %s",
		    cep->cdid, cep->name, src_tel, dst_tel);

		/**/
		ev_reject_from_user(cep);

		/* must goto done, because of reject */
		goto done;
		break;

	case REACT_ALERT:
		/* no timeout */
		cep_set_state(cep,ST_INCOMING,0,0);

		/* send incoming to user */
		response_to_user(cep, DSTAT_INCOMING_CALL);

		/**/
		ev_alert_from_user(cep);
		break;

	default:
		log(LL_WRN, "unknown response type");

		/* fallthrough to REACT_IGNORE: */

	case REACT_IGNORE:
		log(LL_CHD, "%05d %s is ignoring incoming call from %s to %s",
		    cep->cdid, cep->name, src_tel, dst_tel);

		/**/
		ev_disconnect_from_user(cep);
		break;

	case REACT_ANSWER:
		if(cep->alert)
		{
				if(cep->display)
				{
					log(LL_CHD, "%05d %s is alerting incoming call from %s to %s (%s)",
					        cep->cdid, cep->name, src_tel, dst_tel, cep->display);
				}
				else
				{
					log(LL_CHD, "%05d %s is alerting incoming call from %s to %s",
					        cep->cdid, cep->name, src_tel, dst_tel);
				}

				/* send incoming to user */
				response_to_user(cep, DSTAT_INCOMING_CALL);

				/**/
				ev_alert_from_user(cep);
		}
		else
		{
				if(cep->display)
				{				
					log(LL_CHD, "%05d %s is answering incoming call from %s to %s (%s)",
						cep->cdid, cep->name, src_tel, dst_tel, cep->display);
				}
				else
				{
					log(LL_CHD, "%05d %s is answering incoming call from %s to %s",
						cep->cdid, cep->name, src_tel, dst_tel);
				}

				/**/
				ev_connect_from_user(cep);

				/* must goto done, because of connect */
				goto done;
		}
		break;

	case REACT_CALLBACK:

		/**/
		ev_reject_from_user(cep);
		ev_dial_from_user(cep);

		/* must goto done, because of reject */
		goto done;
		break;
	}
      }
    }
    else /* some call in progress */
    {
      if(CEP_IS_CALLED_BACK(cep) && number_matches(mp,cep))
      {
	/* disconnect the current call,
	 * and connect to the remote callback-call
	 */
	ev_disconnect_from_user(cep);
	goto connect;
      }
    }
  }

 done:
  handle_scrprs(mp->header.cdid, mp->scr_ind, mp->prs_ind, src_tel);
  return;
}

#include <msg_decoder.h>

/*---------------------------------------------------------------------------*
 *	timeout, recovery and retry handling
 *---------------------------------------------------------------------------*/
void
handle_recovery(void)
{
	static unsigned last_time;
	static unsigned last_unit_check;
	char check_units;
	unsigned now;

	now = (unsigned)time(NULL);	/* get current time */

	if((now - last_time) >= 1)
	{
	  last_time = now;

	  /* NOTE: if time is adjusted, isdnd must be re-started !! XXX */
	}
	else
	{
	  return;
	}

	if((now - last_unit_check) >= 10)
	{
	  last_unit_check = now;
	  check_units = 1;
	}
	else
	{
	  check_units = 0;
	}

	/* walk through all entries, look for work to do */
	
	CEP_FOREACH(cep,&cfg_entry_tab[0])
	{
		if(cep->budget_callbackperiod &&
		   cep->budget_callbackncalls)
		{
			if(cep->budget_callbackperiod_time <= now)
			{
				DBGL(DL_BDGT, (log(LL_DBG, "%s: new cback-budget-period (%d s, %d left)",
					cep->name, cep->budget_callbackperiod, cep->budget_callbackncalls_cnt)));
				cep->budget_callbackperiod_time = now + cep->budget_callbackperiod;
				cep->budget_callbackncalls_cnt = cep->budget_callbackncalls;
			}
		}

		if(cep->budget_calloutperiod &&
		   cep->budget_calloutncalls)
		{
			if(cep->budget_calloutperiod_time <= now)
			{
				DBGL(DL_BDGT, (log(LL_DBG, "%s: new cout-budget-period (%d s, %d left)",
					cep->name, cep->budget_calloutperiod, cep->budget_calloutncalls_cnt)));
				cep->budget_calloutperiod_time = now + cep->budget_calloutperiod;
				cep->budget_calloutncalls_cnt = cep->budget_calloutncalls;
			}
		}

		if(cep->state == ST_IDLE)
		{
		  continue;
		}

		if(check_units)
		{
		  /*
		   * if shorthold mode is rates based, check if
		   * we entered a time with a new unit length
		   */
		  if(cep->unitlengthsrc == ULSRC_RATE)
		  {
			int newrate = get_current_rate(cep, 0);
	
			if(newrate != cep->unitlength)
			{
				DBGL(DL_MSG, (log(LL_DBG, "rates unit length updated %d -> %d", cep->unitlength, newrate)));
			
				cep->unitlength = newrate;
	
				unitlen_chkupd(cep);
			}
		  }
		}

		/* check timeout */

		if((now - cep->last_set_state_time) > cep->waittime)
		{
				switch(cep->waitfunc)
				{
				case 0:
					break;

				case 1:
					DBGL(DL_RCVRY, (log(LL_DBG, "%s: %s%d: timeout!", 
							    cep->name, driver_name(cep->usrdevicename), cep->usrdeviceunit)));
					/**/
					ev_disconnect_from_user(cep);
					return;

				case 2:
					DBGL(DL_RCVRY, (log(LL_DBG, "%s: %s%d: dial-retry number %d!",
							    cep->name, driver_name(cep->usrdevicename), cep->usrdeviceunit, cep->dial_count)));
					/**/
					ev_dial_from_i4b(cep);
					break;

				case 3:
					DBGL(DL_RCVRY, (log(LL_DBG, "%s: %s%d: answering incoming call!",
							    cep->name, driver_name(cep->usrdevicename), cep->usrdeviceunit)));
					/**/
					ev_connect_from_user(cep);
					break;
				}
		}
	}
	return;
}

