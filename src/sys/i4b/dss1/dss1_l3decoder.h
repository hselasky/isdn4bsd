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

static u_int16_t
get_multi_1(struct dss1_buffer *buf, u_int16_t offset, 
	    u_int8_t *dst, u_int16_t len, u_int8_t filter)
{
    u_int8_t *dst_end = (dst + len);
    u_int8_t *dst_old = dst;

    if(dst_end == dst)
    {
       return 0;
    }

    /* reserve the last byte for NULL */

    dst_end--;

    while((dst < dst_end) && dss1_get_valid(buf,offset))
    {
        *dst = dss1_get_1(buf,offset);

	dst++;
	offset++;
    }

    *dst = '\0';

    if(filter)
    {
        for(dst = dst_old;
	    dst[0];
	    dst++)
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
	}
    }
    return (dst-dst_old);
}

/*---------------------------------------------------------------------------*
 *	decode a Q.931 codeset 0 information element to restart 
 *      indication byte
 *---------------------------------------------------------------------------*/
static void
dss1_decode_q931_cs0_ie_restart(void *arg, struct dss1_buffer *buf)
{
	u_int8_t *p_restart_ind = arg;

	if(dss1_get_1(buf,0) == IEI_RESTARTI)
	{
	    p_restart_ind[0] = dss1_get_1(buf,2);
	    NDBGL3(L3_P_MSG, "restart indication = 0x%02x", p_restart_ind[0]);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode a Q.931 codeset 0 information element to a call descriptor
 *---------------------------------------------------------------------------*/
static void
dss1_decode_q931_cs0_ie_cd(void *arg, struct dss1_buffer *buf)
{
	call_desc_t *cd = arg;
	struct i4b_src_telno *p_src;
	u_int8_t temp = dss1_get_1(buf,2);
	u_int8_t msg_type = dss1_get_1(buf,0);

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

	    temp = dss1_get_1(buf,4) & 0x1F;

	    switch(temp) {
	    case 0x02: 
	      cd->channel_bsubprot = BSUBPROT_G711_ULAW;
	      break;

	    case 0x03:
	      cd->channel_bsubprot = BSUBPROT_G711_ALAW;
	      break;

	    default:
	      NDBGL3(L3_P_ERR, "IEI_BEARERCAP - "
		     "Unsupported B-Sub-Protocol 0x%02x", temp);
	      cd->channel_bsubprot = BSUBPROT_UNKNOWN;
	    }
	    break;
	
	case IEI_CAUSE:		/* cause */
	    cd->cause_in = dss1_get_1(buf,(temp & 0x80) ? 3 : 4) & 0x7f;
	    NDBGL3(L3_P_MSG, "IEI_CAUSE = %d", cd->cause_in);
	    break;
	
	case IEI_CALLID:	/* call identity */
	    break;

	case IEI_CALLSTATE:	/* call state */
	    cd->call_state = (temp & 0x3f);
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
			    temp = dss1_get_1(buf,iid ? 4 : 3);

			    if((temp & 0x2F) == 0x03)
			    {
			        temp = dss1_get_1(buf,iid ? 5 : 4);

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
	    get_multi_1(buf,2,&(cd->display[0]),sizeof(cd->display[0]),1);

	    NDBGL3(L3_P_MSG, "IEI_DISPLAY = %s", &(cd->display[0]));
	    break;
			
	case IEI_DATETIME:	/* date/time */
	    cd->datetime[0] = '\0';
	    temp = 0;

	    buf->offset += 2;

	    while(dss1_get_valid(buf,0))
	    {
	        temp += snprintf(&(cd->datetime[temp]), DATETIME_MAX-temp, 
				 "%02d", dss1_get_1(buf,0));

		if(temp >= DATETIME_MAX)
		{
		    break;
		}
		buf->offset++;
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

	    if(cd->received_src_telno_1) {
	      cd->received_src_telno_2 = 1;
	      p_src = &(cd->src[1]);
	    } else {
	      cd->received_src_telno_1 = 1;
	      p_src = &(cd->src[0]);
	    }

	    /* type of number (source) */
	    switch ((temp & 0x70) >> 4) {
	    case 1:
	      p_src->ton = TON_INTERNAT;
	      break;
	    case 2:
	      p_src->ton = TON_NATIONAL;
	      break;
	    default:
	      p_src->ton = TON_OTHER;
	      break;
	    }

	    if(temp & 0x80) /* no presentation/screening indicator ? */
	    {
		p_src->scr_ind = SCR_NONE;
		p_src->prs_ind = PRS_NONE;				

		get_multi_1(buf,3,&(p_src->telno[0]),sizeof(p_src->telno),1);
	    }
	    else
	    {
		temp = dss1_get_1(buf,3);
		p_src->scr_ind = (temp & 0x03) + SCR_USR_NOSC;
		p_src->prs_ind = ((temp >> 5) & 0x03) + PRS_ALLOWED;

	        get_multi_1(buf,4,&(p_src->telno[0]),sizeof(p_src->telno),1);
	    }
	    NDBGL3(L3_P_MSG, "IEI_CALLINGPN = %s", &(p_src->telno[0]));
	    break;
	
	case IEI_CALLINGPS:	/* calling party subaddress */
	    if(cd->dir_incoming == 0)
	    {
	        /* invalid */
	        break;
	    }

	    if(cd->received_src_telno_2) {
	      p_src = &(cd->src[1]);
	    } else {
	      p_src = &(cd->src[0]);
	    }

	    get_multi_1(buf,3,&(p_src->subaddr[0]),sizeof(p_src->subaddr),1);
	    NDBGL3(L3_P_MSG, "IEI_CALLINGPS = %s", &(p_src->subaddr[0]));
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
		  get_multi_1(buf,3,cd->dst_telno_ptr,
			      &(cd->dst_telno[TELNO_MAX])-cd->dst_telno_ptr,1);
	    }

	    get_multi_1(buf,3,&(cd->dst_telno_part[0]), 
			sizeof(cd->dst_telno_part),1);

	    NDBGL3(L3_P_MSG, "IEI_CALLED = %s", &(cd->dst_telno_part[0])); 
	    break;
	
	case IEI_CALLEDPS:	/* called party subaddress */
	    if(cd->dir_incoming == 0)
	    {
	        /* invalid */
	        break;
	    }

	    get_multi_1(buf,3,&(cd->dst_subaddr[0]),sizeof(cd->dst_subaddr),1);

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
	    get_multi_1(buf,2,&(cd->user_user[0]),sizeof(cd->user_user),0);
	    NDBGL3(L3_P_MSG, "IEI_USERUSER = %s", &(cd->user_user[0]));
	    break;
			
	case IEI_ESCAPE:	/* escape for extension */
	    break;

	    /* ---- Q.932 ---- */

	case IEI_FACILITY:	/* facility */
#if 0
	    dss1_facility_decode(cd, buf);
#endif
	    break;

	    /* ---- Q.95x ---- */

	case IEI_CONCTDNO:	/* connected number */
	    break;

	default:
	    NDBGL3(L3_P_MSG, "Unknown IE 0x%02x = ", dss1_get_1(buf,0));
	    break;
	}
 done:
	return;
}

/*---------------------------------------------------------------------------*
 *	dummy decoder
 *---------------------------------------------------------------------------*/
static void
dss1_decode_dummy(void *arg, struct dss1_buffer *buf)
{
	return;
}

typedef void dss1_decode_ie_t(void *arg, struct dss1_buffer *src);

/*---------------------------------------------------------------------------*
 *	generic information element decode
 *---------------------------------------------------------------------------*/
static void
dss1_decode_ie(void *arg, struct dss1_buffer *buf,
	       dss1_decode_ie_t *q931_func)
{
	u_int8_t codeset = CODESET_0;
	u_int8_t codeset_next = CODESET_0;
	u_int8_t msg_type;
	u_int16_t old_len;
	u_int16_t msg_len;

	if(q931_func == NULL)
	{
	    q931_func = &dss1_decode_dummy;
	}

	/* check length */
	while(dss1_get_valid(buf,0))
	{
	    msg_type = dss1_get_1(buf, 0);

	    if(!(msg_type & 0x80))
	    {
	        /* multi byte IE */
	        msg_len = ((u_int16_t)dss1_get_1(buf, 1)) + 2;
	    }
	    else
	    {
		/* single byte IE */
	        msg_len = 1;
	    }

	    old_len = dss1_set_length(buf, buf->offset + msg_len);

	    switch(codeset) {
	    case CODESET_0:
	        q931_func(arg, buf);
		break;

	    default:
	        NDBGL3(L3_P_MSG, "unknown codeset %d, "
		       "IE = ", codeset);
		break;
	    }

	    buf->offset = buf->len; /* set new offset */
	    buf->len = old_len; /* restore length */
	    
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
dss1_pipe_data_ind(DSS1_TCP_pipe_t *pipe, struct dss1_buffer *buf,
		   u_int8_t broadcast)
{
	static const u_int8_t
	  MAKE_TABLE(Q931_MESSAGE_TYPES,EVENT,[0x80]);

        l2softc_t *sc = pipe->L5_sc;
	call_desc_t *cd;
	u_int32_t crval;
	u_int8_t event;
	u_int8_t pd;
	u_int8_t mt;
	u_int8_t cr_temp[5];

	if(dss1_get_valid(buf,0) == 0)
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

	pd = dss1_get_1(buf,0);
	
	if(pd != PD_Q931)
	{
	    NDBGL3(L3_P_MSG, "unknown protocol "
		   "discriminator 0x%02x!", pd);
	    goto done;
	}

	buf->offset++;

	cr_temp[0] = dss1_get_1(buf,0) & CRLENGTH_MASK;
	cr_temp[1] = dss1_get_1(buf,1);
	cr_temp[2] = dss1_get_1(buf,2);
	cr_temp[3] = dss1_get_1(buf,3);
	cr_temp[4] = dss1_get_1(buf,4);

	buf->offset += cr_temp[0] + 1;

	if(cr_temp[0] > 4) {
	   cr_temp[0] = 4;
	}

	crval = get_callreference(cr_temp);

	/* incoming CR-flag is inverted */
	crval ^= 0x80;

	mt = dss1_get_1(buf,0) & 0x7F;

	/* decode message-type */

	event = Q931_MESSAGE_TYPES_EVENT[mt];

	NDBGL3(L3_PRIM, "unit=%x, crlen=%d crval=0x%04x, "
	       "message_type=0x%02x", sc->sc_unit, 
	       cr_temp[0], crval, mt);

	buf->offset++;

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
		}
		else if((crval & ~0x80) == 0)
		{
			/* global callreference */

			u_int8_t restart_ind;

			dss1_decode_ie(&restart_ind,buf,
				       &dss1_decode_q931_cs0_ie_restart);

			if(event == EV_L3_RESTART_IND)
			{
			    /* send RESTART_ACKNOWLEDGE message
			     *
			     * NOTE: this driver only supports one thing
			     * and that is full reset
			     */
			    dss1_l3_tx_message_by_pipe_cr
			      (pipe,crval,RESTART_ACKNOWLEDGE,
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
			dss1_l3_tx_message_by_pipe_cr
			  (pipe,crval,RELEASE_COMPLETE,
			   (L3_TX_HEADER|L3_TX_CAUSE),
			   CAUSE_Q850_INVCLRFVAL,0);
		}
		else if(TE_MODE(sc) &&
			(!broadcast) &&
			(mt == RELEASE) &&
			(crval == 0x7F))
		{
			/* some exchanges apparently thinks
			 * that the call-reference value is
			 * active when STATUS_ENQUIRY is 
			 * transmitted, and sends a RELEASE
			 * message that must be answered!
			 */
			dss1_l3_tx_message_by_pipe_cr
			  (pipe,crval,RELEASE_COMPLETE,
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

	/* process information elements */

	dss1_decode_ie(cd,buf,
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
	struct i4b_controller *cntl = sc->sc_cntl;
	struct call_desc *cd;

	NDBGL3(L3_MSG, "unit=%x, pipe=%x",
	       sc->sc_unit, PIPE_NO(pipe));

	/* inform all active call(s) of the event */

	CD_FOREACH(cd, cntl)
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
	struct i4b_controller *cntl = sc->sc_cntl;
	struct call_desc *cd;

	NDBGL3(L3_MSG, "unit=%x, pipe=%x",
	       sc->sc_unit, PIPE_NO(pipe));

	/* inform all active call(s) of the event */

	CD_FOREACH(cd, cntl)
	{
	    if((cd->cdid != CDID_UNUSED) &&
	       (cd->pipe == pipe))
	    {
	        cd_update(cd, NULL, EV_L3_RELEASE);
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle retrieve request message from userland
 *---------------------------------------------------------------------------*/
static void
n_retrieve_request(call_desc_t *cd)
{
	cd_update(cd, NULL, EV_L3_RETRIEVE_REQ);
	return;
}

/*---------------------------------------------------------------------------*
 *	handle hold request message from userland
 *---------------------------------------------------------------------------*/
static void
n_hold_request(call_desc_t *cd)
{
	cd_update(cd, NULL, EV_L3_HOLD_REQ);
	return;
}

/*---------------------------------------------------------------------------*
 *	handle deflect request message from userland
 *---------------------------------------------------------------------------*/
static void
n_deflect_request(call_desc_t *cd)
{
	cd_update(cd, NULL, EV_L3_DEFLECTRQ);
	return;
}

/*---------------------------------------------------------------------------*
 *	handle Malicious Call IDentification request message from userland
 *---------------------------------------------------------------------------*/
static void
n_mcid_request(call_desc_t *cd)
{
	cd_update(cd, NULL, EV_L3_MCIDRQ);
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

