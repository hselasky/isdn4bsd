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
 *	dss1_l3decoder.h - layer 3 decoder
 *	----------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

struct buf_range {
  u_int8_t *start;
  u_int8_t *end;
};

static __inline u_int8_t
get_1(struct buf_range *buf, u_int8_t offset)
{
    u_int8_t *ptr = buf->start + offset;
    return ((ptr < buf->start) || (ptr >= buf->end)) ? 0x00 : ptr[0];
}

static __inline u_int8_t
get_valid(struct buf_range *buf, u_int8_t offset)
{
    u_int8_t *ptr = buf->start + offset;
    return ((ptr < buf->start) || (ptr >= buf->end)) ? 0 : 1;
}

static u_int16_t
get_multi_1(struct buf_range *buf, u_int8_t offset, 
	    u_int8_t *dst, u_int16_t len, u_int8_t filter)
{
    u_int8_t *ptr = buf->start + offset;
    u_int16_t max;

    if((ptr < buf->start) ||
       (ptr >= buf->end) || (len == 0))
    {
        if(len)
	  dst[0] = 0;
	return 0;
    }

    /* reserve the last byte for NUL */

    len--;

    max = buf->end - ptr;

    if(len > max)
    {
        len = max;
    }

    bcopy(ptr, dst, len);
    dst[len] = 0;

    if(filter)
    {
        while(dst[0])
	{
	    /* if you don't understand why this filter is here,
	     * then you should never write PBX software
	     */
	    if (((dst[0] >= 'a') && (dst[0] <= 'z')) ||
		((dst[0] >= 'A') && (dst[0] <= 'Z')) ||
		((dst[0] >= '0') && (dst[0] <= '9')) ||
		 (dst[0] == '*') || (dst[0] == '#')) {

		/* allow */

	    } else {

		dst[0] = '_';

	    }
	    dst++;
	}
    }
    return len;
}

/*---------------------------------------------------------------------------*
 *	decode a Q.931 codeset 0 information element to restart 
 *      indication byte
 *---------------------------------------------------------------------------*/
static void
dss1_decode_q931_cs0_ie_restart(void *arg, struct buf_range *src)
{
	u_int8_t *p_restart_ind = arg;

	if(get_1(src,0) == IEI_RESTARTI)
	{
	    p_restart_ind[0] = get_1(src,2);
	    NDBGL3(L3_P_MSG, "restart indication = 0x%02x", p_restart_ind[0]);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode a Q.931 codeset 0 information element to a call descriptor
 *---------------------------------------------------------------------------*/
static void
dss1_decode_q931_cs0_ie_cd(void *arg, struct buf_range *src)
{
	call_desc_t *cd = arg;
	u_int8_t temp = get_1(src,2);
	u_int8_t msg_type = get_1(src,0);

	if(msg_type & 0x80)
	{
	    /* single byte IE */

	    if(msg_type == IEI_SENDCOMPL)
	    {
	        /* sending complete */
	        if(cd->dir_incoming)
		{
		    NDBGL3(L3_P_MSG, "IEI_SENDCOMPL");
		    cd->sending_complete = 1;
		}
	    }
	    goto done;
	}

	switch(msg_type) {

	  /* ---- Q.931 ---- */
		
	  /* multi byte IE's */
		
	case IEI_SEGMMSG:	/* segmented message */
	    break;
			
	case IEI_BEARERCAP:	/* bearer capability */
	    if(cd->dir_incoming == 0)
	    {
	        /* invalid */
	        break;
	    }

	    switch(temp) {
	    case 0x80:	/* speech */
	    case 0x89:	/* restricted digital info */
	    case 0x90:	/* 3.1KHz audio */
	        cd->channel_bprot = BPROT_NONE;
		break;

	    case 0x88:	/* unrestricted digital info */
	        cd->channel_bprot = BPROT_RHDLC;
		break;

	    default:
	        cd->channel_bprot = BPROT_NONE;
		NDBGL3(L3_P_ERR, "IEI_BEARERCAP - "
		       "Unsupported B-Protocol 0x%02x", temp);
		break;
	    }
	    break;
	
	case IEI_CAUSE:		/* cause */
	    cd->cause_in = get_1(src,(temp & 0x80) ? 3 : 4) & 0x7f;
	    NDBGL3(L3_P_MSG, "IEI_CAUSE = %d", cd->cause_in);
	    break;
	
	case IEI_CALLID:	/* call identity */
	    break;

	case IEI_CALLSTATE:	/* call state */
	    cd->call_state = temp & 0x3f;		
	    NDBGL3(L3_P_MSG, "IEI_CALLSTATE = %d", cd->call_state);
	    break;
			
	case IEI_CHANNELID:	/* channel id */
	    if(cd->channel_allocated == 0)
	    {
	        DSS1_TCP_pipe_t *pipe = cd->pipe;
		l2softc_t *sc = pipe->L5_sc;

	        /* exclusive is set if no other 
		 * channel is acceptable
		 */
	        u_int8_t exclusive = (temp & 0x08) || TE_MODE(sc);

		/* interface identifier is set
		 * if there is an interface identifier
		 * byte
		 */
		u_int8_t iid = (temp & 0x40);

		if(temp & 0x20)
		{
		    /* Primary Rate */
		    if(temp & 0x04)
		    {
		        if(exclusive)
			  cd->channel_id = CHAN_D1;
		    }
		    else
		    {
		        temp &= 0x03;
			if(temp == 3)
			{
			    cd->channel_id = CHAN_ANY;
			}
			else if(temp == 1)
			{
			    /* time-slot follows */
			    temp = get_1(src,iid ? 4 : 3);

			    if((temp & 0x2F) == 0x03)
			    {
			        temp = get_1(src,iid ? 5 : 4);

				if(exclusive)
				{
				    int32_t channel_id = (temp & 0x7F);

				    L1_COMMAND_REQ
				      (sc->sc_cntl,CMR_DECODE_CHANNEL,
				       &channel_id);
				    cd->channel_id = channel_id;
				}
			    }
			}
			else if(temp == 0)
			{
			    cd->channel_id = CHAN_NOT_ANY;
			}
		    }
		}
		else
		{
		    /* Basic Rate */
		    if(temp & 0x04)
		    {
		        if(exclusive)
			  cd->channel_id = CHAN_D1;
		    }
		    else
		    {
		        temp &= 0x03;
			cd->channel_id =
			  (temp == IE_CHAN_ID_NO) ? CHAN_NOT_ANY :
			  (temp == IE_CHAN_ID_ANY) ? CHAN_ANY :
			  (exclusive) ? 
			  ((temp == IE_CHAN_ID_B1) ? CHAN_B1 : CHAN_B2) :
			  cd->channel_id;
		    }
		}
		NDBGL3(L3_P_MSG, "IEI_CHANNELID - channel %d, "
		       "exclusive = %d, interface id = %d", 
		       cd->channel_id, !!exclusive, !!iid);
	    }
	    break;				
	
	case IEI_PROGRESSI:	/* progress indicator	*/
	    break;
			
	case IEI_NETSPCFAC:	/* network specific fac */
	    break;
			
	case IEI_NOTIFIND:	/* notification indicator */
	    break;
			
	case IEI_DISPLAY:	/* display */
	    get_multi_1(src,2,&(cd->display[0]),sizeof(cd->display[0]),1);

	    NDBGL3(L3_P_MSG, "IEI_DISPLAY = %s", &(cd->display[0]));
	    break;
			
	case IEI_DATETIME:	/* date/time */
	    cd->datetime[0] = '\0';
	    temp = 0;

	    src->start += 2;
	    while(get_valid(src,0))
	    {
	        temp += snprintf(&(cd->datetime[temp]), DATETIME_MAX-temp, 
				 "%02d", get_1(src,0));

		if(temp >= DATETIME_MAX)
		{
		    break;
		}
		src->start++;
	    }
	    NDBGL3(L3_P_MSG, "IEI_DATETIME = %s", &(cd->datetime[0]));
	    break;
			
	case IEI_KEYPAD:	/* keypad facility */
	    break;
			
	case IEI_SIGNAL:	/* signal type */
	    NDBGL3(L3_P_MSG, "IEI_SIGNAL = 0x%02x", temp);
	    break;

	case IEI_INFRATE:	/* information rate */
	    break;

	case IEI_ETETDEL:	/* end to end transit delay */
	    break;

	case IEI_CUG:		/* closed user group */
	    break;

	case IEI_CALLINGPN:	/* calling party no */
	    if(cd->dir_incoming == 0)
	    {
	        /* invalid */
	        break;
	    }

	    /* type of number (source) */
	    switch ((temp & 0x70) >> 4) {
	    case 1:
	      cd->src_ton = TON_INTERNAT;
	      break;
	    case 2:
	      cd->src_ton = TON_NATIONAL;
	      break;
	    default:
	      cd->src_ton = TON_OTHER;
	      break;
	    }

	    if(temp & 0x80) /* no presentation/screening indicator ? */
	    {
		cd->scr_ind = SCR_NONE;
		cd->prs_ind = PRS_NONE;				

	        get_multi_1(src,3,&(cd->src_telno[0]),sizeof(cd->src_telno),1);
	    }
	    else
	    {
		temp = get_1(src,3);
		cd->scr_ind = (temp & 0x03) + SCR_USR_NOSC;
		cd->prs_ind = ((temp >> 5) & 0x03) + PRS_ALLOWED;

	        get_multi_1(src,4,&(cd->src_telno[0]),sizeof(cd->src_telno),1);
	    }
	    NDBGL3(L3_P_MSG, "IEI_CALLINGPN = %s", &(cd->src_telno[0]));
	    break;
	
	case IEI_CALLINGPS:	/* calling party subaddress */
	    if(cd->dir_incoming == 0)
	    {
	        /* invalid */
	        break;
	    }
	    get_multi_1(src,3,&(cd->src_subaddr[0]),sizeof(cd->src_subaddr),1);
	    NDBGL3(L3_P_MSG, "IEI_CALLINGPS = %s", &(cd->src_subaddr[0]));
	    break;
			
	case IEI_CALLEDPN:	/* called party number */

	    if(cd->dir_incoming)
	    {
	        /* type of number (destination) */
	        switch((temp & 0x70) >> 4) {
		case 1:	
		    cd->dst_ton = TON_INTERNAT;
		    break;
		case 2:
		    cd->dst_ton = TON_NATIONAL;
		    break;
		default:
		    cd->dst_ton = TON_OTHER;
		    break;
		}

		cd->dst_telno_ptr +=
		  get_multi_1(src,3,cd->dst_telno_ptr,
			      &(cd->dst_telno[TELNO_MAX])-cd->dst_telno_ptr,1);
	    }

	    get_multi_1(src,3,&(cd->dst_telno_part[0]), 
			sizeof(cd->dst_telno_part),1);

	    NDBGL3(L3_P_MSG, "IEI_CALLED = %s", &(cd->dst_telno_part[0])); 
	    break;
	
	case IEI_CALLEDPS:	/* called party subaddress */
	    if(cd->dir_incoming == 0)
	    {
	        /* invalid */
	        break;
	    }

	    get_multi_1(src,3,&(cd->dst_subaddr[0]),sizeof(cd->dst_subaddr),1);

	    NDBGL3(L3_P_MSG, "IEI_CALLEDPS = %s", &(cd->dst_subaddr[0]));
	    break;

	case IEI_REDIRNO:	/* redirecting number */
	    break;

	case IEI_TRNSEL:	/* transit network selection */
	    break;

	case IEI_RESTARTI:	/* restart indicator */
	    break;

	case IEI_LLCOMPAT:	/* low layer compat */
	    break;
			
	case IEI_HLCOMPAT:	/* high layer compat	*/
	    break;
			
	case IEI_USERUSER:	/* user-user */
	    get_multi_1(src,2,&(cd->sms[0]),sizeof(cd->sms),0);
	    NDBGL3(L3_P_MSG, "IEI_USERUSER = %s", &(cd->sms[0]));
	    break;
			
	case IEI_ESCAPE:	/* escape for extension */
	    break;

	    /* ---- Q.932 ---- */

	case IEI_FACILITY:	/* facility */
#if 0
	    XXX should this be accepted in TE-mode only ?

	    if(dss1_aoc(cd, src) == 0)
	    {
	        i4b_l4_charging_ind(cd);
	    }
#endif
	    break;

	    /* ---- Q.95x ---- */

	case IEI_CONCTDNO:	/* connected number */
	    break;

	default:
	    NDBGL3(L3_P_MSG, "Unknown IE 0x%02x = ", get_1(src,0));
#if DO_I4B_DEBUG
	    if(i4b_l3_debug & L3_P_MSG)
	    {
	        dss1_dump_buf(__FUNCTION__, src->start, src->end - src->start);
	    }
#endif
	    break;
	}
 done:
	return;
}

/*---------------------------------------------------------------------------*
 *	dummy decoder
 *---------------------------------------------------------------------------*/
static void
dss1_decode_dummy(void *arg, struct buf_range *src)
{
	return;
}

typedef void dss1_decode_ie_t(void *arg, struct buf_range *src);

/*---------------------------------------------------------------------------*
 *	generic information element decode
 *---------------------------------------------------------------------------*/
static void
dss1_decode_ie(void *arg, u_int8_t *msg_ptr, u_int8_t *msg_end, 
	       dss1_decode_ie_t *q931_func)
{
	struct buf_range buf_range;
	u_int8_t codeset = CODESET_0;
	u_int8_t codeset_next = CODESET_0;
	u_int8_t msg_type;
	u_int8_t *msg_start;

	if(q931_func == NULL)
	{
	    q931_func = &dss1_decode_dummy;
	}

	/* check length */
	while(msg_ptr < msg_end)
	{
	    /* store start of message */
	    msg_start = msg_ptr;

	    /* get message type */
	    msg_type = *msg_ptr;

	    /* skip to next byte */
	    msg_ptr++;

	    if(!(msg_type & 0x80))
	    {
	        /* multi byte IE */

	        /* process one IE for the selected codeset */

	        if(msg_ptr >= msg_end)
		{
		    /* a multi byte IE should 
		     * contain at least the length
		     * field, else one has reached
		     * the end
		     */
		    break;
		}

		/* compute message end */

		msg_ptr += msg_ptr[0] + 1;
	    }
	    /* else single byte IE */

	    /* check length again */

	    if(msg_ptr > msg_end)
	    {
	        break;
	    }

	    switch(codeset) {
	    case CODESET_0:
	        buf_range.start = msg_start;
		buf_range.end = msg_ptr;
		q931_func(arg, &buf_range);
		break;
				
	    default:
	        NDBGL3(L3_P_MSG, "unknown codeset %d, IE = ",
		       codeset);
#if DO_I4B_DEBUG
		if(i4b_l3_debug & L3_P_MSG)
		{
		    dss1_dump_buf(__FUNCTION__, msg_start,
				  msg_ptr - msg_start);
		}
#endif
		break;
	    }

	    /* check for codeset shift */

	    if((msg_type & 0xf0) == IEI_SHIFT)
	    {
	        codeset_next = codeset;

		codeset = (msg_type & CODESET_MASK);

		if(msg_type & SHIFT_LOCK)
		{
		    codeset_next = codeset;
		}
	    }
	    else
	    {
	        /* select next codeset */
	        codeset = codeset_next;
	    }
	}
	return;
}

static void
dss1_pipe_reset_ind(DSS1_TCP_pipe_t *pipe);

/*---------------------------------------------------------------------------*
 *	PIPE DATA INDICATION from Layer 2
 *---------------------------------------------------------------------------*/
static void
dss1_pipe_data_ind(DSS1_TCP_pipe_t *pipe, u_int8_t *msg_ptr, u_int msg_len,
		  u_int broadcast)
{
        l2softc_t *sc = pipe->L5_sc;
	u_int8_t *msg_end, *msg_tmp;
	call_desc_t *cd;
	u_int32_t crval;
	u_int8_t event;

	msg_end = msg_ptr + msg_len;

	if(msg_len < 4)
	{
	  NDBGL3(L3_P_MSG, "frame too short");
	  goto done;
	}

	if(NT_MODE(sc) && broadcast)
	{
	  /* disallow broadcast-NT connecting to
	   * broadcast-NT, because then there is
	   * no frame-number-check, hence NT
	   * replies to the same pipe that
	   * a message was received through
	   */
	  NDBGL3(L3_P_MSG, "broadcast I-frame in NT-mode");
	  goto done;
	}

	/* check protocol discriminator */
	
	if(*msg_ptr != PD_Q931)
	{
	  NDBGL3(L3_P_MSG, "unknown protocol "
		 "discriminator 0x%02x!", *msg_ptr);
	  goto done;
	}
	msg_ptr++;

	/* store a copy of "msg_ptr" */

	msg_tmp  = msg_ptr;

	/* extract call reference */

	msg_ptr += (msg_ptr[0] & CRLENGTH_MASK) + 1;

	/* check length (includes message-type byte!) */
	if(msg_ptr >= msg_end)
	{
	  NDBGL3(L3_P_MSG, "frame too short");
	  goto done;
	}

	crval = get_callreference(msg_tmp);

	/* incoming CR-flag is inverted */
	crval ^= 0x80;

	/* decode message-type */

	static const u_int8_t
	  MAKE_TABLE(Q931_MESSAGE_TYPES,EVENT,[0x80]);

	event = Q931_MESSAGE_TYPES_EVENT[*msg_ptr & 0x7F];

	NDBGL3(L3_PRIM|L3_P_MSG, "unit=%x, crlen=%d crval=0x%04x, "
	       "message_type=0x%02x", sc->sc_unit, 
	       (msg_tmp[0] & CRLENGTH_MASK), crval, *msg_ptr);

	msg_ptr++;

	/* find call-descriptor */

	cd = cd_by_unitcr(sc->sc_cntl,pipe,&sc->sc_pipe[0],crval);

	if(cd == NULL)
	{
		/* check if incoming call */
		if((crval & 0x80) &&
		   (event == EV_L3_SETUP))
		{
			/* allocate call-descriptor */

			cd = N_ALLOCATE_CD(sc->sc_cntl,pipe,crval,0,NULL);
			/* cdid filled in */
		}
		else if((crval & ~0x80) == 0)
		{
			/* global callreference */

			u_int8_t restart_ind;

			dss1_decode_ie(&restart_ind,msg_ptr,msg_end,
				       &dss1_decode_q931_cs0_ie_restart);

			if(event == EV_L3_RESTART_IND)
			{
			    /* send RESTART_ACKNOWLEDGE message
			     *
			     * NOTE: this driver only supports one thing
			     * and that is full reset
			     */
			    dss1_l3_tx_message_by_pipe_cr(pipe,crval,RESTART_ACKNOWLEDGE,
							  (L3_TX_HEADER|L3_TX_RESTARTI),0,7);
			    dss1_pipe_reset_ind(pipe);
			}

			if(event == EV_L3_RESTART_ACK)
			{
			}
		}
		else if(NT_MODE(sc) && 
			(!IS_POINT_TO_POINT(sc)) &&
			L3_EVENT_IS_LOCAL_OUTGOING(event) &&
			(!L3_EVENT_IS_LOCAL_INCOMING(event)) &&
			(!(crval & 0x80)))
		{
			NDBGL3(L3_PRIM, "unit=%x, shutting down invalid "
			       "callreference: 0x%04x", sc->sc_unit, crval);

			/* Some device sent a response to an outgoing call,
			 * to a non-existing call reference, that belongs 
			 * to the NT-mode side. Send a RELEASE_COMPLETE 
			 * message back just in case that the device 
			 * thinks that the call is still valid.
			 */
			dss1_l3_tx_message_by_pipe_cr(pipe,crval,RELEASE_COMPLETE,
						      (L3_TX_HEADER|L3_TX_CAUSE),
						      CAUSE_Q850_INVCLRFVAL,0);
		}

		if(cd == NULL)
		{
		    NDBGL3(L3_P_MSG, "cannot find calldescriptor for "
			   "crval=0x%02x", crval);

		    /* ignore CR, and let it timeout */
		    goto done;
		}
	}

	if((event == EV_L3_DISCONNECT) && (cd->want_late_inband == 0))
	{
	    /* hangup immediately */
	    event = EV_L3_RELEASE;
	}

	if(event == EV_L3_CONNECT)
	{
	    cd->datetime[0] = '\0';
	}

	if(event == EV_L3_SETUP)
	{
	    /* reset the destination
	     * telephone number, hence
	     * it is accumulated
	     */
	    cd->dst_telno_ptr =
	      &(cd->dst_telno[0]);

	    /* assuming that the other
	     * variables are not changed
	     */
	}

	/* reset partial telephone number */
	cd->dst_telno_part[0] = 0;

	/* set peer responded flag */
	cd->peer_responded = 1;

	/* keep track of which pipes that 
	 * responded to an [outgoing] call 
	 */
	if(PIPE_NO(pipe) < (8*sizeof(cd->peer_responded_bitmask)))
	{
	    cd->peer_responded_bitmask[PIPE_NO(pipe)/8] |= 1 << (PIPE_NO(pipe) % 8);
	}

	/* process information elements */

	dss1_decode_ie(cd,msg_ptr,msg_end,
		       &dss1_decode_q931_cs0_ie_cd);

	if(NT_MODE(sc) &&
	   (!IS_POINT_TO_POINT(sc)))
	{
	    if((event == EV_L3_RELEASE) &&
	       (cd->cause_in != CAUSE_Q850_CALLREJ) &&
	       (cd->pipe == (void *)&(sc->sc_pipe[0])))
	    {
	        /* just ignore it */
	        goto done;
	    }

	    if(event == EV_L3_DISCONNECT)
	    {
	        /* just ignore it */
	        goto done;
	    }
	}

	cd_update(cd, pipe, event);

 done:
	return;
}

/*---------------------------------------------------------------------------*
 *	PIPE RESEND INDICATION from Layer 2
 *
 * NOTE: pipe == pipe_adapter
 *---------------------------------------------------------------------------*/
static void
dss1_pipe_resend_ind(DSS1_TCP_pipe_t *pipe)
{
	l2softc_t *sc = pipe->L5_sc;
	struct call_desc *cd;

	NDBGL3(L3_MSG, "unit=%x, pipe=%x",
	       sc->sc_unit, PIPE_NO(pipe));

	/* inform all active call(s) of the event */

	CD_FOREACH(cd,&sc->sc_cntl->N_call_desc[0])
	{
	    if((cd->cdid != CDID_UNUSED) &&
	       (cd->pipe == pipe) &&
	       (cd->need_release))
	    {
	        if((cd->peer_responded == 0) &&
		   ((cd->setup_interleave == 0) ||
		    (cd->setup_interleave == 4) ||
		    (cd->setup_interleave >= 6)))
		{
		    /* need to repeat setup in case of frame loss 
		     *
		     * NOTE: some ISDN PBXs do not like when the
		     * SETUP message is repeated, so just repeat
		     * until one gets a response
		     */
		    dss1_l3_tx_setup(cd);
		}

		if(sc->L1_activity || 
		   cd->setup_interleave)
		{
		   cd->setup_interleave++;
		}

		/*
		 * NOTE: some ISDN PBXs will crash
		 * if one sends STATUS_ENQUIRY before 
		 * CONNECT ...
		 */
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	PIPE RESET INDICATION from Layer 2
 *---------------------------------------------------------------------------*/
static void
dss1_pipe_reset_ind(DSS1_TCP_pipe_t *pipe)
{
	l2softc_t *sc = pipe->L5_sc;
	struct call_desc *cd;

	NDBGL3(L3_MSG, "unit=%x, pipe=%x",
	       sc->sc_unit, PIPE_NO(pipe));

	/* inform all active call(s) of the event */

	CD_FOREACH(cd,&sc->sc_cntl->N_call_desc[0])
	{
		if( (cd->cdid != CDID_UNUSED) &&
		    (cd->pipe == pipe) )
                {
		  cd_update(cd, NULL, EV_L3_RELEASE);
		}
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle connect request message from userland
 *---------------------------------------------------------------------------*/
static void
n_connect_request(call_desc_t *cd)
{
	cd_update(cd, NULL, EV_L3_SETUPRQ);
	return;
}

/*---------------------------------------------------------------------------*
 *	handle information request message from userland
 *---------------------------------------------------------------------------*/
static void
n_information_request(call_desc_t *cd)
{
	cd_update(cd, NULL, EV_L3_INFORQ);
	return;
}

/*---------------------------------------------------------------------------*
 *	handle setup response message from userland
 *---------------------------------------------------------------------------*/
static void
n_connect_response(call_desc_t *cd, int response, int cause)
{
	cd->cause_out = cause;

	switch(response) {
	case SETUP_RESP_ACCEPT:
	    cd_update(cd, NULL, EV_L3_SETACRS);
	    break;
		
	case SETUP_RESP_REJECT:
	    cd_update(cd, NULL, EV_L3_SETRJRS);
	    break;

	case SETUP_RESP_DNTCRE:
	    cd_update(cd, NULL, EV_L3_RELEASE);
	    break;

	default:	/* failsafe */
	    cd_update(cd, NULL, EV_L3_RELEASE);
	    NDBGL3(L3_ERR, "unknown response, doing SETUP_RESP_DNTCRE");
	    break;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle disconnect request message from userland
 *---------------------------------------------------------------------------*/
static void
n_disconnect_request(call_desc_t *cd, int cause)
{
	cd->cause_out = cause;

	cd_update(cd, NULL, EV_L3_RELEASE);
	return;
}

/*---------------------------------------------------------------------------*
 *	handle alert request message from userland
 *---------------------------------------------------------------------------*/
static void
n_alert_request(call_desc_t *cd, int flag)
{
	cd_update(cd, NULL, (flag & 1) ? EV_L3_PROCEEDINGRQ : EV_L3_ALERTRQ);
	return;
}

/*---------------------------------------------------------------------------*
 *	handle progress request message from userland
 *---------------------------------------------------------------------------*/
static void
n_progress_request(call_desc_t *cd)
{
	cd_update(cd, NULL, EV_L3_PROGRESSRQ);
	return;
}

/*---------------------------------------------------------------------------*
 *	allocate and initialize a calldescriptor
 *---------------------------------------------------------------------------*/
static call_desc_t *
n_allocate_cd(struct i4b_controller *cntl, void *pipe, u_int crval,
	      u_int ai_type, void *ai_ptr)
{
  l2softc_t *sc = cntl->N_fifo_translator->L5_sc; 
  call_desc_t *cd;
  int state;

  cd = i4b_allocate_cd(cntl);

  if(cd != NULL)
  {
    if(pipe == NULL)
    {
      /* outgoing call */
      state = ST_L3_OUTGOING;
      pipe = &sc->sc_pipe[0]; /* pipe adapter */
      crval = CDID2CALLREFERENCE(cd->cdid);

      cd->dir_incoming = 0;
    }
    else
    {
      /* incoming call */
      state = ST_L3_INCOMING;
      cd->dir_incoming = 1;
    }

    /* setup variables */

    /* cd->cntl = cntl; */
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
  }
  return cd;
}

/*---------------------------------------------------------------------------*
 *	free an allocated calldescriptor
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

