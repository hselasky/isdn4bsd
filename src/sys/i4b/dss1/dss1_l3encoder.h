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
 *	dss1_l3encoder.h - layer 3 encoder
 *	----------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

static const u_int8_t MAKE_TABLE(L3_STATES,Q931_CONV,[]);

static u_int8_t *
IEI_channelid(struct call_desc *cd, u_int8_t *ptr)
{
    DSS1_TCP_pipe_t *pipe = cd->pipe;
    l2softc_t *sc = pipe->L5_sc;
    u_int8_t *ptr_old = ptr;

    /* only accept this channel */
    enum { ext_exclusive = 0x80 | 0x08 }; 

    *ptr = IEI_CHANNELID;  /* channel id */

    ptr += 2;

    switch(cd->channel_id) {
    default:
        *ptr = IE_CHAN_ID_ANY | ext_exclusive;
	break;
    case CHAN_NOT_ANY:
        *ptr = IE_CHAN_ID_NO | ext_exclusive;
	break;
    case CHAN_D1:
        *ptr = 0x04 | ext_exclusive;
	break;
    case CHAN_B1:
        *ptr = IE_CHAN_ID_B1 | ext_exclusive;
	break;
    case CHAN_B2:
        *ptr = IE_CHAN_ID_B2 | ext_exclusive;
	break;
    }

    if(IS_PRIMARY_RATE(sc))
    {
        /* Primary Rate */

        *ptr |= 0x20;

	if(cd->channel_id >= CHAN_B1)
	{
	    int32_t channel_id = cd->channel_id;

	    L1_COMMAND_REQ(sc->sc_cntl,CMR_ENCODE_CHANNEL,&channel_id);

	    /* channel is indicated in the following octets */
	    *ptr &= ~0x03;
	    *ptr |= 0x01;

	    ptr++;

	    *ptr = 0x80 | 0x03; /* B-channel */

	    ptr++;

	    *ptr = channel_id | 0x80;
	}
    }
    ptr++;

    ptr_old[1] = ptr - ptr_old - 2;

    return ptr;
}

static u_int32_t
get_callreference(u_int8_t *ptr)
{
    u_int8_t len = ptr[0] & CRLENGTH_MASK;
    u_int32_t cr = 0;

    ptr++;

    while(len)
    {
        len--;
	cr <<= 8;
	cr |= ptr[len];
    }
    return cr;
}

static u_int8_t *
make_callreference(DSS1_TCP_pipe_t *pipe, u_int32_t cr, u_int8_t *ptr)
{
    l2softc_t *sc = pipe->L5_sc;
    u_int8_t len = 
      (cr & 0xFF000000) ? 4 :
      (cr & 0x00FF0000) ? 3 :
      (cr & 0x0000FF00) ? 2 : 1;

    if(IS_PRIMARY_RATE(sc) && (len < 2))
    {
        /* minimum 2 bytes are required by
	 * primary rate:
	 */
        len = 2;
    }

    *ptr++ = len; /* call reference length */

    while(len)
    {
        ptr[0] = cr;

	cr >>= 8;
	ptr++;
	len--;
    }
    return ptr;
}

/*---------------------------------------------------------------------------*
 *	send RESTART message
 *
 * (indication & 7):
 *   7: restart all interfaces
 *   6: restart single interface
 *   0: restart indicated channels
 *---------------------------------------------------------------------------*/
static void
dss1_l3_tx_restart(DSS1_TCP_pipe_t *pipe, u_int8_t what, 
		   u_int8_t indication, u_int32_t call_ref)
{
	struct mbuf *m;
	u_int8_t *ptr;

	NDBGL3(L3_PRIM, "");

#define DSS1_L3_TX_RESTART_LEN_MAX \
	  (I_HEADER_LEN + 5 + 3 + 16 /* extra buffer */)

	/* check length */

#if (DSS1_L3_TX_RESTART_LEN_MAX > BCH_MAX_LEN)
#error "DSS1_L3_TX_RESTART_LEN_MAX > BCH_MAX_LEN"
#endif

	m = i4b_getmbuf(BCH_MAX_LEN, M_NOWAIT);

	if(m)
	{
	   ptr = m->m_data + I_HEADER_LEN;
	
	  *ptr++ = PD_Q931;		/* protocol discriminator */
	   ptr   = make_callreference(pipe,call_ref,ptr);
	  *ptr++ = what;

	  *ptr++ = IEI_RESTARTI;
	  *ptr++ = 1; /* byte */
	  *ptr++ = indication | 0x80; /* restart indication */

	  /* update length */
	  m->m_len = ptr - ((__typeof(ptr))(m->m_data));

	  dss1_pipe_data_req(pipe,m);
	}
	else
	{
	  NDBGL3(L3_ERR, "out of mbufs!");
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send SETUP message
 *---------------------------------------------------------------------------*/
static void
dss1_l3_tx_setup(call_desc_t *cd)
{
	struct mbuf *m;
	u_char *ptr, *str;
	int len;

	NDBGL3(L3_PRIM, "cdid=%d, cr=%d",
	       cd->cdid, cd->cr);

	m = i4b_getmbuf(BCH_MAX_LEN, M_NOWAIT);

	if(m)
	{
	  ptr = m->m_data + I_HEADER_LEN;
	
	  *ptr++ = PD_Q931;		/* protocol discriminator */
	   ptr   = make_callreference(cd->pipe,cd->cr,ptr);
	  *ptr++ = SETUP;		/* message type = setup */

	  if(cd->dst_telno[0])
	  {
	      *ptr++ = IEI_SENDCOMPL;	/* sending complete */
	  }

	  *ptr++ = IEI_BEARERCAP;	/* bearer capability */

	  switch(cd->channel_bprot)
	  {
		case BPROT_NONE:	/* telephony */
		case BPROT_RHDLC_DOV:	/* Data over Voice */
			*ptr++ = IEI_BEARERCAP_LEN+1;
			*ptr++ = IT_CAP_SPEECH;
			*ptr++ = IT_RATE_64K;
			*ptr++ = IT_UL1_G711A;
			break;

		case BPROT_RHDLC:	/* raw HDLC */
		case BPROT_NONE_VOD:    /* Voice over Data */
		default:
			*ptr++ = IEI_BEARERCAP_LEN;
			*ptr++ = IT_CAP_UNR_DIG_INFO;
			*ptr++ = IT_RATE_64K;
			break;
	  }

	  if(cd->channel_allocated)
	  {
		ptr = IEI_channelid(cd, ptr);
	  }

	  str = &(cd->keypad[0]);

	  if(str[0] != 0)
	  {
		len = strlen(str);
		*ptr++ = IEI_KEYPAD;		/* keypad facility */
		*ptr++ = len;			/* keypad facility length */

		bcopy(str, ptr, len);
		ptr += len;
	  }
	
	  str = &(cd->src_telno[0]);

	  if(str[0] != 0)
	  {
		len = strlen(str);
		*ptr++ = IEI_CALLINGPN;	/* calling party no */
		ptr[0] = len; /* calling party no length */

		/* type of number, number plan id */

		ptr[1] = 
		  (cd->src_ton == TON_INTERNAT) ? (NUMBER_TYPE_PLAN | 0x10) :
		  (cd->src_ton == TON_NATIONAL) ? (NUMBER_TYPE_PLAN | 0x20) :
		  NUMBER_TYPE_PLAN;

		if(cd->prs_ind != PRS_NONE)
		{
		    /* presentation indicator */

		    /* clear extension bit */
		    ptr[1] &= ~0x80;

		    ptr[2] =
		      (cd->prs_ind == PRS_RESTRICT) ? 
		      (0x20|0x80) : (0x80);

		    ptr[0] += 2;
		    ptr += 3;
		}
		else
		{
		    ptr[0] += 1;
		    ptr += 2;
		}

		bcopy(str, ptr, len);
		ptr += len;
	  }

	  str = &(cd->src_subaddr[0]);

	  if(str[0] != 0)
	  {
		len = strlen(str);
		*ptr++ = IEI_CALLINGPS;		/* calling subaddr */
		*ptr++ = NUMBER_TYPE_LEN+len;	/* calling subaddr len */
		*ptr++ = NUMBER_TYPE_NSAP;	/* type = NSAP */

		bcopy(str, ptr, len);
		ptr += len;
	  }

	  /*
	   * re-transmit the complete destination telephone
	   * number, hence in NT-mode the SETUP message may
	   * be repeated:
	   */
	  str = &(cd->dst_telno[0]);

	  if(str[0] != 0)
	  {
		len = strlen(str);
		*ptr++ = IEI_CALLEDPN;		/* called party no */
		*ptr++ = NUMBER_TYPE_LEN+len;	/* called party no length */
		*ptr++ = NUMBER_TYPE_PLAN;	/* type of number, number plan id */

		bcopy(str, ptr, len);
		ptr += len;
		str += len;

		cd->dst_telno_ptr = str;
	  }

	  str = &(cd->dst_subaddr[0]);

	  if(str[0] != 0)
	  {
		len = strlen(str);
		*ptr++ = IEI_CALLEDPS;		/* calling party subaddr */
		*ptr++ = NUMBER_TYPE_LEN+len;	/* calling party subaddr len */
		*ptr++ = NUMBER_TYPE_NSAP;	/* type = NSAP */

		bcopy(str, ptr, len);
		ptr += len;
	  }

	  str = &(cd->sms[0]);

	  if(str[0] != 0)
	  {
		len = strlen(str);
		*ptr++ = IEI_USERUSER;		/* SMS */
		*ptr++ = IEI_USERUSER_LEN+len;	/* SMS length */

		bcopy(str, ptr, len);
		ptr += len;
	  }

	  str = &(cd->display[0]);

	  if(str[0] != 0)
	  {
		len = strlen(str);
		*ptr++ = IEI_DISPLAY; /* display */
		*ptr++ = len;         /* display string length */

		bcopy(str, ptr, len);
		ptr += len;
	  }

	/* check length */
#if (I_HEADER_LEN   +\
    3               +\
    4               +\
                \
    1               +\
    1               +\
                \
    4               +\
                \
    5               +\
                \
    2               +\
    KEYPAD_MAX      +\
                \
    3               +\
    TELNO_MAX       +\
                \
    3               +\
    SUBADDR_MAX     +\
                \
    3               +\
    1               +\
    TELNO_MAX       +\
                \
    3               +\
    SUBADDR_MAX     +\
                \
    2               +\
    SMS_MAX         +\
		\
    2               +\
    DISPLAY_MAX     +\
                \
    0) > BCH_MAX_LEN
#error " > BCH_MAX_LEN"
#endif

	  /* update length */
	  m->m_len = ptr - ((__typeof(ptr))m->m_data);

	  dss1_pipe_data_req(cd->pipe,m);
	}
	else
	{
	  NDBGL3(L3_ERR, "out of mbufs!");
	}
	return;
}

#define dss1_l3_tx_alert(cd)			\
	dss1_l3_tx_message(cd,ALERT,		\
			   L3_TX_HEADER)
/*
 * NOTE: some PBXs crash if one sends
 * the channel ID back in TE-mode
 */
#define dss1_l3_tx_setup_acknowledge(cd,send_chan_id)			\
	dss1_l3_tx_message(cd,SETUP_ACKNOWLEDGE, (send_chan_id) ?	\
			   (L3_TX_HEADER|L3_TX_CHANNELID) : L3_TX_HEADER)

#define dss1_l3_tx_information(cd)			\
	dss1_l3_tx_message(cd,INFORMATION,		\
			   L3_TX_HEADER|L3_TX_CALLEDPN)

#define dss1_l3_tx_call_proceeding(cd,send_chan_id)			\
	dss1_l3_tx_message(cd,CALL_PROCEEDING, (send_chan_id) ?		\
			   (L3_TX_HEADER|L3_TX_CHANNELID) : L3_TX_HEADER)

#define dss1_l3_tx_connect(cd)			\
	dss1_l3_tx_message(cd,CONNECT,		\
			   L3_TX_HEADER)

#define dss1_l3_tx_connect_acknowledge(cd)		\
	dss1_l3_tx_message(cd,CONNECT_ACKNOWLEDGE,	\
			   L3_TX_HEADER)

#define dss1_l3_tx_disconnect(cd)			\
	dss1_l3_tx_message(cd,DISCONNECT,		\
			   L3_TX_HEADER|L3_TX_CAUSE)

#define dss1_l3_tx_release(cd,send_cause_flag)				\
	dss1_l3_tx_message(cd,RELEASE,(send_cause_flag) ? 		\
			   L3_TX_HEADER|L3_TX_CAUSE : L3_TX_HEADER)

/* NOTE: some ISDN phones require
 * the "cause" information element
 * when sending RELEASE_COMPLETE
 */
#define dss1_l3_tx_release_complete(cd,send_cause_flag)			\
	dss1_l3_tx_message(cd,RELEASE_COMPLETE,(send_cause_flag) ? 	\
			  L3_TX_HEADER|L3_TX_CAUSE : L3_TX_HEADER)

#define dss1_l3_tx_release_complete_complement(cd,p1,p2)		\
	dss1_l3_tx_message_complement(cd,p1,p2,RELEASE_COMPLETE,	\
				      L3_TX_HEADER|L3_TX_CAUSE)

#define dss1_l3_tx_status(cd,q850cause)			\
{							\
	(cd)->cause_out = q850cause;			\
	dss1_l3_tx_message(cd,STATUS,			\
			  L3_TX_CAUSE|L3_TX_CALLSTATE);	\
	(cd)->cause_out = 0;				\
}

#define dss1_l3_tx_status_enquiry(cd)		\
	dss1_l3_tx_message(cd,STATUS_ENQUIRY,	\
			   L3_TX_HEADER)

#define dss1_l3_tx_progress(cd)		\
	dss1_l3_tx_message(cd,PROGRESS,	\
			   L3_TX_HEADER|L3_TX_PROGRESSI)

#define L3_TX_HEADER     0x00
#define L3_TX_CAUSE      0x01
#define L3_TX_CALLSTATE  0x02
#define L3_TX_CHANNELID  0x04
#define L3_TX_CALLEDPN   0x08
#define L3_TX_PROGRESSI  0x10
/*---------------------------------------------------------------------------*
 *	send message
 *---------------------------------------------------------------------------*/
static void
dss1_l3_tx_message(call_desc_t *cd, u_int8_t message_type, u_int8_t flag)
{
	DSS1_TCP_pipe_t *pipe = cd->pipe;
	l2softc_t *sc = pipe->L5_sc;
	struct mbuf *m;
	u_int8_t *ptr;
	u_int8_t *str;
	size_t len;

	NDBGL3(L3_PRIM, "cdid=%d, cr=%d, cause=0x%02x, "
	       "state=0x%x, channel_id=0x%x",
	       cd->cdid, cd->cr,
	       cd->cause_out, cd->state, cd->channel_id);

	m = i4b_getmbuf(DCH_MAX_LEN, M_NOWAIT);

	if(m)
	{
	  ptr = m->m_data + I_HEADER_LEN;
	
	  *ptr++ = PD_Q931;               /* protocol discriminator */
	   ptr   = make_callreference(cd->pipe,cd->cr,ptr);
	  *ptr++ = message_type;          /* message type */

	  if(flag & L3_TX_PROGRESSI)
	  {
	    *ptr++ = IEI_PROGRESSI;
	    *ptr++ = 2; /* bytes */
	    *ptr++ = NT_MODE(sc) ? CAUSE_STD_LOC_PUBLIC : CAUSE_STD_LOC_OUT;
	    *ptr++ = 0x88; /* in-band info available */
	  }

	  if(flag & L3_TX_CAUSE)
	  {
	    *ptr++ = IEI_CAUSE;                      /* cause ie */
	    *ptr++ = IEI_CAUSE_LEN;
	    *ptr++ = NT_MODE(sc) ? CAUSE_STD_LOC_PUBLIC : CAUSE_STD_LOC_OUT;
	    *ptr++ = i4b_make_q850_cause(cd->cause_out)|EXT_LAST;
	  }

	  if(flag & L3_TX_CALLSTATE)
	  {
	    *ptr++ = IEI_CALLSTATE;             /* call state ie */
	    *ptr++ = IEI_CALLSTATE_LEN;
	    *ptr++ = L3_STATES_Q931_CONV[cd->state];
	  }

	  if(flag & L3_TX_CHANNELID)
	  {
	    if(cd->channel_allocated)
	    {
	      ptr = IEI_channelid(cd, ptr);
	    }
	  }

	  if(flag & L3_TX_CALLEDPN)
	  {
	    str = &(cd->dst_telno_part[0]);

	    if(str[0] != 0)
	    {
		len = strlen(str);
		*ptr++ = IEI_CALLEDPN;		/* called party no */
		*ptr++ = NUMBER_TYPE_LEN+len;	/* called party no length */
		*ptr++ = NUMBER_TYPE_PLAN;	/* type of number, number plan id */

		while(len--)
		{
		  *ptr++ = *str++;
		}
	    }
	  }

	  /* check length */
#if (I_HEADER_LEN   +\
    3               +\
    4               +\
                \
    4               +\
                \
    4               +\
                \
    3               +\
                \
    5               +\
                \
    3               +\
    TELNO_MAX       +\
                \
    0) > DCH_MAX_LEN
#error " > DCH_MAX_LEN"
#endif

	  /* update length */
	  m->m_len = ptr - ((__typeof(ptr))m->m_data);

	  dss1_pipe_data_req(cd->pipe,m);
	}
	else
	{
	  NDBGL3(L3_ERR, "out of mbufs!");
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send message to the pipes complementing the given pipes
 *---------------------------------------------------------------------------*/
static void
dss1_l3_tx_message_complement(struct call_desc *cd,
			      DSS1_TCP_pipe_t *pipe_skip_1,
			      DSS1_TCP_pipe_t *pipe_skip_2,
			      u_int8_t message_type, 
			      u_int8_t flag)
{
	DSS1_TCP_pipe_t *pipe_curr;
	DSS1_TCP_pipe_t *pipe_old;
	l2softc_t *sc;

	/* save current pipe pointer and
	 * get pointer to softc
	 */
	pipe_old = cd->pipe;
	sc = pipe_old->L5_sc;

	PIPE_FOREACH(pipe_curr,&sc->sc_pipe[0])
	{
	    if((pipe_curr != pipe_skip_1) &&
	       (pipe_curr != pipe_skip_2) &&
	       (pipe_curr->state != ST_L2_PAUSE))
	    {
	        cd->pipe = pipe_curr;

		dss1_l3_tx_message(cd, message_type, flag);
	    }
	}

	/* restore pipe pointer */
	cd->pipe = pipe_old;
	return;
}
