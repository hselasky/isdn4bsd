/* $FreeBSD$ */
/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
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
 *	i4b_l4.c - kernel interface to userland
 *	---------------------------------------
 *
 *---------------------------------------------------------------------------*/

#ifdef I4B_GLOBAL_INCLUDE_FILE
#include I4B_GLOBAL_INCLUDE_FILE
#else
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <net/if.h>
#endif

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

/*---------------------------------------------------------------------------*
 *	send MSG_L12STAT_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_l12stat(int controller, int layer, int state)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_l12stat_ind_t), M_NOWAIT)) != NULL)
	{
		msg_l12stat_ind_t *md = (void *)m->m_data;

		md->header.type = MSG_L12STAT_IND;
		md->header.cdid = controller;

		md->layer = layer;
		md->state = state;

		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_TEIASG_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_teiasg(int controller, int tei)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_teiasg_ind_t), M_NOWAIT)) != NULL)
	{
		msg_teiasg_ind_t *md = (void *)m->m_data;

		md->header.type = MSG_TEIASG_IND;
		md->header.cdid = controller;

		md->tei = tei;

		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_DIALOUT_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_dialout(int driver, int driver_unit)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_dialout_ind_t), M_NOWAIT)) != NULL)
	{
		msg_dialout_ind_t *md = (void *)m->m_data;

		md->header.type = MSG_DIALOUT_IND;
		md->header.cdid = -1;

		md->driver = driver;
		md->driver_unit = driver_unit;	

		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_DIALOUTNUMBER_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_dialoutnumber(int driver, int driver_unit, int cmdlen, char *cmd)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_dialoutnumber_ind_t), M_NOWAIT)) != NULL)
	{
		msg_dialoutnumber_ind_t *md = (void *)m->m_data;
		int i;

		md->header.type = MSG_DIALOUTNUMBER_IND;
		md->header.cdid = -1;

		md->driver = driver;
		md->driver_unit = driver_unit;

		for(i = 0; i < cmdlen; i++)
		{
			if(cmd[i] == '*')
			{
				break;
			}
		}

		/* XXX: TELNO_MAX is _with_ tailing '\0',
		 * so max is actually TELNO_MAX - 1
		 */
		md->cmdlen = min(i,TELNO_MAX-1);
		bcopy(cmd, md->cmd, md->cmdlen);
		md->cmd[md->cmdlen] = '\0';

		md->subaddrlen = min(cmdlen-i,SUBADDR_MAX-1);
		if(md->subaddrlen)
		{
		  /* skip the first '*' */
		  md->subaddrlen--;
		  i++;

		  bcopy(cmd+i, md->subaddr, md->subaddrlen); 
		}
		md->subaddr[md->subaddrlen] = '\0';

		NDBGL4(L4_TIMO, "cmd[%d]=%s, subaddr[%d]=%s",
		       md->cmdlen, md->cmd, md->subaddrlen, md->subaddr);
		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_KEYPAD_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_keypad(int driver, int driver_unit, int cmdlen, char *cmd)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_keypad_ind_t), M_NOWAIT)) != NULL)
	{
		msg_keypad_ind_t *md = (void *)m->m_data;

		md->header.type = MSG_KEYPAD_IND;
		md->header.cdid = -1;

		md->driver = driver;
		md->driver_unit = driver_unit;

		if(cmdlen > KEYPAD_MAX)
		{
			cmdlen = KEYPAD_MAX;
		}

		md->cmdlen = cmdlen;
		bcopy(cmd, md->cmd, cmdlen);
		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_NEGCOMP_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_negcomplete_ind(call_desc_t *cd)
{
	struct mbuf *m;

	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	  if((m = i4b_getmbuf(sizeof(msg_negcomplete_ind_t), M_NOWAIT)) != NULL)
	    {
		msg_negcomplete_ind_t *md = (void *)m->m_data;

		md->header.type = MSG_NEGCOMP_IND;
		md->header.cdid = cd->cdid;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_IFSTATE_CHANGED_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_ifstate_changed(call_desc_t *cd, int new_state)
{
	struct mbuf *m;

	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	  if((m = i4b_getmbuf(sizeof(msg_ifstatechg_ind_t), M_NOWAIT)) != NULL)
	    {
		msg_ifstatechg_ind_t *md = (void *)m->m_data;

		md->header.type = MSG_IFSTATE_CHANGED_IND;
		md->header.cdid = cd->cdid;
		md->state = new_state;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_DRVRDISC_REQ message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_drvrdisc(int driver, int driver_unit)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_drvrdisc_req_t), M_NOWAIT)) != NULL)
	{
		msg_drvrdisc_req_t *md = (void *)m->m_data;

		md->header.type = MSG_DRVRDISC_REQ;
		md->header.cdid = -1;

		md->driver = driver;
		md->driver_unit = driver_unit;	

		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_DRVRANSWER_REQ message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_drvranswer(int driver, int driver_unit)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_drvranswer_req_t), M_NOWAIT)) != NULL)
	{
		msg_drvranswer_req_t *md = (void *)m->m_data;

		md->header.type = MSG_DRVRANSWER_REQ;
		md->header.cdid = -1;

		md->driver = driver;
		md->driver_unit = driver_unit;

		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_DRVRREJECT_REQ message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_drvrreject(int driver, int driver_unit)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_drvrreject_req_t), M_NOWAIT)) != NULL)
	{
		msg_drvrreject_req_t *md = (void *)m->m_data;

		md->header.type = MSG_DRVRREJECT_REQ;
		md->header.cdid = -1;

		md->driver = driver;
		md->driver_unit = driver_unit;

		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

#if I4B_ACCOUNTING
/*---------------------------------------------------------------------------*
 *	send MSG_ACCT_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_accounting(int driver, int driver_unit, int accttype, int ioutbytes,
		  int iinbytes, int ro, int ri, int outbytes, int inbytes)
{
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(msg_accounting_ind_t), M_NOWAIT)) != NULL)
	{
		msg_accounting_ind_t *md = (void *)m->m_data;

		md->header.type = MSG_ACCT_IND;
		md->header.cdid = -1;

		md->driver = driver;
		md->driver_unit = driver_unit;	

		md->accttype = accttype;
		md->ioutbytes = ioutbytes;
		md->iinbytes = iinbytes;
		md->outbps = ro;
		md->inbps = ri;
		md->outbytes = outbytes;
		md->inbytes = inbytes;
		
		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *      accounting timeout
 *---------------------------------------------------------------------------*/
void
i4b_accounting_timeout(struct i4b_accounting *sc)
{
        bchan_statistics_t bs;
        
	FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	{
	  /* connected */

	  /* get number of bytes in and out from the HSCX driver */
	  L1_FIFO_STAT(sc->sc_fifo_translator, &bs);

	  sc->sc_ioutb += bs.outbytes;
	  sc->sc_iinb += bs.inbytes;

	  if((sc->sc_iinb != sc->sc_linb) || 
	     (sc->sc_ioutb != sc->sc_loutb) || !sc->sc_fn)
	  {
		int ri = (sc->sc_iinb - sc->sc_linb)/I4B_ACCOUNTING_INTERVAL;
		int ro = (sc->sc_ioutb - sc->sc_loutb)/I4B_ACCOUNTING_INTERVAL;

                if((sc->sc_iinb == sc->sc_linb) && 
		   (sc->sc_ioutb == sc->sc_loutb))
                        sc->sc_fn = 1;
                else
                        sc->sc_fn = 0;

                sc->sc_linb = sc->sc_iinb;
                sc->sc_loutb = sc->sc_ioutb;

                i4b_l4_accounting(sc->sc_driver_type,
				  sc->sc_driver_unit, ACCT_DURING,
				  sc->sc_ioutb, sc->sc_iinb,
				  ro, ri,
				  sc->sc_outb ? sc->sc_outb : sc->sc_ioutb,
				  sc->sc_inb ? sc->sc_inb : sc->sc_iinb);
	  }

	  callout_reset(&sc->sc_callout,
			  I4B_ACCOUNTING_INTERVAL*hz,
			  (void *)(void *)i4b_accounting_timeout, sc);
	},
	{
	  /* not connected */

	  i4b_l4_accounting(sc->sc_driver_type,
			    sc->sc_driver_unit, ACCT_FINAL,
			    sc->sc_ioutb, sc->sc_iinb, 
			    0, 0, 
			    sc->sc_outb, sc->sc_inb);

	  callout_stop(&sc->sc_callout);

	  sc->sc_iinb = 0;
	  sc->sc_ioutb = 0;
	  sc->sc_inb = 0;
	  sc->sc_outb = 0;
	  sc->sc_linb = 0;
	  sc->sc_loutb = 0;
	  sc->sc_inpkt = 0;
	  sc->sc_outpkt = 0;
	  sc->sc_fn = 0;
	});

	return;
}
#endif

static void
telno_copy(char *dst, const char *src, uint16_t len)
{
    strlcpy(dst, src[0] ? src : TELNO_EMPTY, len);
    return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_CONNECT_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_ai_connect_ind(struct call_desc *cd, struct i4b_ai_softc *ai_ptr,
		   uint16_t *p_copy_count)
{
	msg_connect_ind_t *mp;
	struct mbuf *m;

	if((m = i4b_getmbuf(sizeof(*mp), M_NOWAIT)))
	{
		mp = (void *)(m->m_data);

		mp->header.type = MSG_CONNECT_IND;
		mp->header.cdid = cd->cdid;

		mp->channel = cd->channel_id;
		mp->bprot = cd->channel_bprot;

		/* store wether the number is complete or not */

		mp->sending_complete = cd->sending_complete;

		/* just copy the destination telephone 
		 * number like is, hence it might be updated
		 * later due to overlap sending:
		 */
		strlcpy(mp->dst_telno, cd->dst_telno,
			sizeof(mp->dst_telno));

		strlcpy(mp->dst_subaddr, cd->dst_subaddr,
			sizeof(mp->dst_subaddr));

		telno_copy(mp->src_telno,
			   cd->src[0].telno,
			   sizeof(mp->src_telno));

		telno_copy(mp->src_subaddr,
			   cd->src[0].subaddr,
			   sizeof(mp->src_subaddr));

		mp->src_ton = cd->src[0].ton;
		mp->dst_ton = cd->dst_ton;

		strlcpy(mp->display, cd->display,
			sizeof(mp->display));

		strlcpy(mp->user_user, cd->user_user,
			sizeof(mp->user_user));

		mp->scr_ind = cd->src[0].scr_ind;
		mp->prs_ind = cd->src[0].prs_ind;

		i4b_ai_putqueue(ai_ptr,0,m,p_copy_count);
	}
	return;
}

/* The following function returns the number of connect 
 * indication messages sent to userland:
 */
uint16_t
i4b_l4_connect_ind(call_desc_t *cd)
{
	uint16_t count = 0;

	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
		i4b_ai_connect_ind(cd, cd->ai_ptr, &count);
	}

	if((cd->ai_type == I4B_AI_CAPI) || (cd->ai_type == I4B_AI_BROADCAST))
	{
		capi_ai_connect_ind(cd, &count);

		/* send info indication last
		 * to make software happy:
		 */
		capi_ai_info_ind(cd, 0, 0x8005 /* SETUP */, NULL, 0);
	}
	return count;
}

/*---------------------------------------------------------------------------*
 *	send MSG_INFORMATION_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_information_ind(call_desc_t *cd)
{
	msg_information_ind_t *mp;
	struct mbuf *m;
	char buffer[TELNO_MAX];
	int len;
	uint8_t temp;

	if (cd->dst_telno_part[0] == 0) {
	    NDBGL4(L4_MSG, "cdid=%d: cd->dst_telno_part[0] == 0",
		   cd->cdid);
	    return;
	}

	switch (cd->dst_ton) {
	case TON_INTERNAT:
		temp = 0x80 | 0x10 | 0x01;
		break;
	case TON_NATIONAL:
		temp = 0x80 | 0x20 | 0x01;
		break;
	default:
		temp = 0x80 | 0x00 | 0x01;
		break;
	}

	len = snprintf(buffer, sizeof(buffer),
		       "%c%s", temp, cd->dst_telno_part);

	if (len < 1) {
	    /* shouldn't happen */
	    return;
	}

	len--;
	
	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	    if((m = i4b_getmbuf(sizeof(*mp), M_NOWAIT)) != NULL)
	    {
	        mp = (void *)(m->m_data);

		mp->header.type = MSG_INFORMATION_IND;
		mp->header.cdid = cd->cdid;

		if(len > (int)(sizeof(mp->dst_telno)-1))
		{
		    /* should not happen */
		    len = (sizeof(mp->dst_telno)-1);
		}

		memcpy(mp->dst_telno, buffer + 1, len);
		mp->dst_telno[len] = 0;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	    capi_ai_info_ind(cd, 0, 0x0070 /* IEI_CALLEDPN */,
			     &(buffer[0]), len+1);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_IDLE_TIMEOUT_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_idle_timeout_ind(call_desc_t *cd)
{
	struct mbuf *m;

	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	    if((m = i4b_getmbuf(sizeof(msg_idle_timeout_ind_t), M_NOWAIT)) != NULL)
	    {
		msg_idle_timeout_ind_t *mp = (void *)m->m_data;

		mp->header.type = MSG_IDLE_TIMEOUT_IND;
		mp->header.cdid = cd->cdid;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || (cd->ai_type == I4B_AI_BROADCAST))
	{
		/* CAPI does not have this signal */
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_CHARGING_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_charging_ind(call_desc_t *cd)
{
	struct mbuf *m;

	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	  if((m = i4b_getmbuf(sizeof(msg_charging_ind_t), M_NOWAIT)) != NULL)
	    {
		msg_charging_ind_t *mp = (void *)m->m_data;

		mp->header.type = MSG_CHARGING_IND;
		mp->header.cdid = cd->cdid;
		mp->units_type = cd->units_type;

/*XXX*/		if(mp->units_type == CHARGE_CALC)
			mp->units = cd->cunits;
		else
			mp->units = cd->units;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || (cd->ai_type == I4B_AI_BROADCAST))
	{
		/* CAPI does not have this signal */
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_ALERT_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_alert_ind(call_desc_t *cd)
{
	struct mbuf *m;

	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	    if((m = i4b_getmbuf(sizeof(msg_alert_ind_t), M_NOWAIT)) != NULL)
	    {
		msg_alert_ind_t *mp = (void *)m->m_data;

		mp->header.type = MSG_ALERT_IND;
		mp->header.cdid = cd->cdid;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || (cd->ai_type == I4B_AI_BROADCAST))
	{
		capi_ai_info_ind(cd, 0, 0x8001 /* ALERT */, NULL, 0);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_PROCEEDING_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_proceeding_ind(call_desc_t *cd, uint8_t sending_complete, 
		      uint8_t progress)
{
	struct mbuf *m;

	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	    if((m = i4b_getmbuf(sizeof(msg_proceeding_ind_t), M_NOWAIT)) != NULL)
	    {
		msg_proceeding_ind_t *mp = (void *)m->m_data;

		mp->header.type = MSG_PROCEEDING_IND;
		mp->header.cdid = cd->cdid;
		mp->sending_complete = sending_complete;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	    static const uint8_t progress_indicator[] = { 0x02, 0x80, 0x88 
							   /* | 0x02 if analog */ };
	    if(sending_complete)
	    {
	        capi_ai_info_ind(cd, 0, 0x8002 /* CALL_PROCEEDING */, NULL, 0);
	    }
	    else 
	    {
	        capi_ai_info_ind(cd, 0, 0x800d /* SETUP_ACKNOWLEDGE */, NULL, 0);
	    }

	    if((cd->channel_id != CHAN_ANY) &&
	       (cd->channel_id != CHAN_NOT_ANY) && progress)
	    {
		capi_ai_info_ind(cd, 0, 0x001e /* PROGRESS */, 
				 &progress_indicator, sizeof(progress_indicator));
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_RETRIEVE_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_retrieve_ind(call_desc_t *cd)
{
	struct mbuf *m;
	msg_retrieve_ind_t *mp;

	if((cd->ai_type == I4B_AI_I4B) || 
	   (cd->ai_type == I4B_AI_BROADCAST))
	{
	    if((m = i4b_getmbuf(sizeof(*mp), M_NOWAIT)) != NULL)
	    {
	        mp = (void *)(m->m_data);

		mp->header.type = MSG_RETRIEVE_IND;
		mp->header.cdid = cd->cdid;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || 
	   (cd->ai_type == I4B_AI_BROADCAST))
	{

	    static const uint8_t retrieve_indicator[] = { 0x01, 0xfa };

	    capi_ai_info_ind(cd, 0, 0x0027 /* NOTIFY */, 
			     &retrieve_indicator, sizeof(retrieve_indicator));
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_HOLD_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_hold_ind(call_desc_t *cd)
{
	struct mbuf *m;
	msg_hold_ind_t *mp;

	if((cd->ai_type == I4B_AI_I4B) || 
	   (cd->ai_type == I4B_AI_BROADCAST))
	{
	    if((m = i4b_getmbuf(sizeof(*mp), M_NOWAIT)) != NULL)
	    {
	        mp = (void *)(m->m_data);

		mp->header.type = MSG_HOLD_IND;
		mp->header.cdid = cd->cdid;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || 
	   (cd->ai_type == I4B_AI_BROADCAST))
	{

	    static const uint8_t hold_indicator[] = { 0x01, 0xf9 };

	    capi_ai_info_ind(cd, 0, 0x0027 /* NOTIFY */, 
			     &hold_indicator, sizeof(hold_indicator));
	}
	return;
}

/*---------------------------------------------------------------------------*
 *    send MSG_PACKET_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_packet_ind(int driver, int driver_unit, int dir_out, struct mbuf *pkt)
{
	struct mbuf *m;
	int len = pkt->m_pkthdr.len;
	void *ip = mtod(pkt, void *);

	if((m = i4b_getmbuf(sizeof(msg_packet_ind_t), M_NOWAIT)) != NULL)
	{
		msg_packet_ind_t *mp = (void *)m->m_data;

		mp->header.type = MSG_PACKET_IND;
		mp->header.cdid = -1;
		mp->driver = driver;
		mp->driver_unit = driver_unit;
		mp->direction_out = dir_out;
		memcpy(mp->pktdata, ip, min(len,MAX_PACKET_LOG));

		i4b_ai_putqueue(NULL,0,m,NULL);
	}
	return;
}

setup_ft_t *i4b_drivers_setup_ft[N_I4B_DRIVERS];
response_to_user_t *i4b_drivers_response_to_user[N_I4B_DRIVERS];

static struct mbuf *
i4b_default_alloc_mbuf(struct fifo_translator *f, uint16_t def_len, uint16_t tr_len)
{
    return i4b_getmbuf(def_len, M_NOWAIT);
}

void
i4b_register_driver(int type, setup_ft_t *sft, response_to_user_t *rtu)
{
    if (type < 0 || type >= N_I4B_DRIVERS)
        return;

    i4b_drivers_setup_ft[type] = sft;
    i4b_drivers_response_to_user[type] = rtu;
}

void
i4b_unregister_driver(int type)
{
    if (type < 0 || type >= N_I4B_DRIVERS)
        return;

    i4b_drivers_setup_ft[type] = NULL;
    i4b_drivers_response_to_user[type] = NULL;
}

/*---------------------------------------------------------------------------*
 *    i4b_setup_driver
 *
 * returns 1 when disconnected and 0 when connected
 *---------------------------------------------------------------------------*/
int
i4b_setup_driver(i4b_controller_t *cntl, uint32_t channel, 
		 struct i4b_protocol *pp, uint32_t driver_type, 
		 uint32_t driver_unit, struct call_desc *cd)
{
	setup_ft_t *setup_ft;

	static const uint8_t
	  MAKE_TABLE(L1_TYPES,DEFAULT_DRIVER_TYPE,[]);

	struct fifo_translator *f1;
	struct fifo_translator *f2;

	int error = 1;

	CNTL_LOCK(cntl);
	mtx_lock(&i4b_global_lock);
	
	if(driver_type == DRVR_D_CHANNEL)
	{
	  driver_type = L1_TYPES_DEFAULT_DRIVER_TYPE[cntl->L1_type];
	}

	if(driver_type >= N_I4B_DRIVERS)
	{
	  driver_type = DRVR_DUMMY;
	}

	setup_ft = i4b_drivers_setup_ft[driver_type];

	if(setup_ft && cntl->L1_GET_FIFO_TRANSLATOR && 
	   (channel < cntl->L1_channel_end) && 
	   (f1 = L1_GET_FIFO_TRANSLATOR(cntl,channel)))
	{
	  /* get current FIFO translator, if any */
	  f2 = setup_ft(cntl,0,0,driver_type,driver_unit,cd);

	  if(pp->protocol_1 == P_D_CHANNEL)
	  {
	    pp->protocol_1 = (i4b_open_refcount) ? 
	      P_HDLC : P_DISABLE;
	  }

	  if(pp->protocol_1 != P_DISABLE)
	  {
	    uint8_t l1_in_use =
	      f1->L5_sc || 
	      f1->L5_fifo ||
	      f1->L5_PUT_MBUF || 
	      f1->L5_GET_MBUF;

	    if(l1_in_use || f2)
	    {
	      NDBGL4(L4_ERR, "connect: FIFO(%d.%d)=%d or driver(%d.%d)=%d "
		     "already connected", cntl->unit, channel, l1_in_use,
		     driver_type, driver_unit, (f2 != 0));
	      goto done;
	    }
	    f2 = f1;
	  }
	  else
	  {
	    if(f1 != f2)
	    {
	      NDBGL4(L4_MSG, "disconnect: FIFO(%d.%d) not connected to "
		     "driver(%d.%d)", cntl->unit, channel,
		     driver_type, driver_unit);
	      goto done;
	    }
	    /* f2 = f1; */
	    /* call L1_FIFO_SETUP before setup_ft,
	     * so that L5_PUT_MBUF and L5_GET_MBUF is
	     * not called after that setup_ft is called,
	     * when disconnecting
	     */
	  unsetup_L1:
	    L1_FIFO_SETUP(f1,pp);

	  unsetup_ft:
	    f1 = 0;
	  }

	  _IF_DRAIN(&f2->tx_queue);
	  _IF_DRAIN(&f2->rx_queue);
	  f2->tx_queue.ifq_maxlen = IFQ_MAXLEN;
	  f2->rx_queue.ifq_maxlen = IFQ_MAXLEN;

	  f2->L5_sc = NULL;
	  f2->L5_fifo = NULL;
	  f2->L5_PUT_MBUF = NULL;
	  f2->L5_PUT_DTMF = NULL;
	  f2->L5_GET_MBUF = NULL;
	  f2->L5_RX_INTERRUPT = NULL;
	  f2->L5_TX_INTERRUPT = NULL;
	  f2->L5_ALLOC_MBUF = &i4b_default_alloc_mbuf; /* default */

	  /* set mutex */
	  f2->mtx = CNTL_GET_LOCK(cntl);

	  /* increase refcount in case
	   * the disconnect happens while
	   * another thread is sleeping
	   * on this FIFO translator
	   */
	  f2->refcount++;

	  setup_ft(cntl,f1,pp,driver_type,driver_unit,cd);

	  if(f1)
	  {
	    if(pp->protocol_1 != P_DISABLE)
	    {
	      /* call L1_FIFO_SETUP after setup_ft
	       * so that L5_PUT_MBUF and L5_GET_MBUF is
	       * not called before setup_ft is called,
	       * when connecting
	       */
	      L1_FIFO_SETUP(f1,pp);

	      if(pp->protocol_1 != P_DISABLE)
	      {
		error = 0;
	      }
	      else
	      {
		goto unsetup_L1;
	      }
	    }
	    else
	    {
	      goto unsetup_ft;
	    }
	  }
	}
	else
	{
	  /* not connected */
	  NDBGL4(L4_MSG,"invalid parameters or "
		 "controller not connected");
	}

 done:
	mtx_unlock(&i4b_global_lock);
	CNTL_UNLOCK(cntl);

	return error;
}

/*---------------------------------------------------------------------------*
 *	link a driver(unit) to a B-channel(controller,channel)
 *
 * returns 1 when disconnected and 0 when connected
 *---------------------------------------------------------------------------*/
int
i4b_link_bchandrvr(call_desc_t *cd, int activate)
{
	static const uint8_t
	  MAKE_TABLE(I4B_B_PROTOCOLS,PROTOCOL,[]);
	struct i4b_protocol p;

	memset(&p, 0, sizeof(p));

	if (activate) {

	    /* disconnect any previously connected channel drivers */

	    (void) i4b_link_bchandrvr(cd, 0);

	    /* make a copy of the "driver_type" and 
	     * the "driver_unit", hence these variables
	     * might be changed by the software:
	     */

	    cd->driver_type_copy = cd->driver_type;
	    cd->driver_unit_copy = cd->driver_unit;
	    cd->b_link_want_active = 1;
	    p.protocol_1 = I4B_B_PROTOCOLS_PROTOCOL[cd->channel_bprot];
	    p.protocol_4 = cd->channel_bsubprot;

	} else {

	    cd->b_link_want_active = 0;
	    p.protocol_1 = P_DISABLE;
	}

	return
	  (cd->channel_allocated) ?
	  i4b_setup_driver(i4b_controller_by_cd(cd),
			   cd->channel_id, &p,
			   cd->driver_type_copy,
			   cd->driver_unit_copy,
			   cd) : 1;
}

/*---------------------------------------------------------------------------*
 *	send MSG_CONNECT_ACTIVE_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_connect_active_ind(call_desc_t *cd)
{
	msg_connect_active_ind_t *mp;
	struct mbuf *m;

	cd->last_active_time = cd->connect_time = SECOND;

	NDBGL4(L4_TIMO, "last_active/connect_time=%ld", (long)cd->connect_time);

	/* it is currently the applications job
	 * to link the B-channel drivers !
	 */

	i4b_l4_setup_timeout(cd);
	
	if((cd->ai_type == I4B_AI_I4B) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	    if((m = i4b_getmbuf(sizeof(*mp), M_NOWAIT)))
	    {
		mp = (void *)(m->m_data);

		mp->header.type = MSG_CONNECT_ACTIVE_IND;
		mp->header.cdid = cd->cdid;

		mp->channel = cd->channel_id;

		if (cd->idate_time_len != 0) {
		    if (cd->idate_time_len < 6) {
			bzero(cd->idate_time_data + cd->idate_time_len,
			      6 - cd->idate_time_len);
		    }
		    snprintf(mp->datetime, sizeof(mp->datetime),
			     "%02d%02d%02d%02d%02d%02d",
			     cd->idate_time_data[0] % 100, cd->idate_time_data[1] % 100,
			     cd->idate_time_data[2] % 100, cd->idate_time_data[3] % 100, 
			     cd->idate_time_data[4] % 100, cd->idate_time_data[5] % 100);
		} else {
		    mp->datetime[0] = 0;
		}

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || (cd->ai_type == I4B_AI_BROADCAST))
	{
	    capi_ai_connect_active_ind(cd);

	    /* send info indication last
	     * to make software happy:
	     */
	    capi_ai_info_ind(cd, 0, cd->dir_incoming ? 
			     0x800f /* CONNECT_ACKNOWLEDGE */:
			     0x8007 /* CONNECT */, NULL, 0);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_PRE_DISCONNECT_IND message to userland
 *
 * This message is used when the charging has ended, and there
 * is inband signalling available.
 *---------------------------------------------------------------------------*/
void
i4b_l4_pre_disconnect_ind(call_desc_t *cd)
{
	struct mbuf *m;

	if((cd->ai_type == I4B_AI_I4B) || 
	   (cd->ai_type == I4B_AI_BROADCAST))
	{
	    if((m = i4b_getmbuf(sizeof(msg_disconnect_ind_t), M_NOWAIT)) != NULL)
	    {
		msg_pre_disconnect_ind_t *mp = (void *)m->m_data;

		mp->header.type = MSG_PRE_DISCONNECT_IND;
		mp->header.cdid = cd->cdid;
		mp->cause = cd->cause_in;

		i4b_ai_putqueue(cd->ai_ptr,0,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || 
	   (cd->ai_type == I4B_AI_BROADCAST))
	{
	    capi_ai_info_ind(cd, 0, 0x8045 /* DISCONNECT */, NULL, 0);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	send MSG_DISCONNECT_IND message to userland
 *---------------------------------------------------------------------------*/
void
i4b_l4_disconnect_ind(call_desc_t *cd, uint8_t complement)
{
	struct mbuf *m;

	if(complement == 0)
	{
		i4b_link_bchandrvr(cd,0);
	}

	if((cd->ai_type == I4B_AI_I4B) || 
	   (cd->ai_type == I4B_AI_BROADCAST) || complement)
	{
	    if((m = i4b_getmbuf(sizeof(msg_disconnect_ind_t), M_NOWAIT)) != NULL)
	    {
		msg_disconnect_ind_t *mp = (void *)m->m_data;

		mp->header.type = MSG_DISCONNECT_IND;
		mp->header.cdid = cd->cdid;
		mp->cause = cd->cause_in;

		i4b_ai_putqueue(cd->ai_ptr,complement,m,NULL);
	    }
	}
	if((cd->ai_type == I4B_AI_CAPI) || 
	   (cd->ai_type == I4B_AI_BROADCAST) || complement)
	{
	    capi_ai_info_ind(cd, complement, 
			     0x805a /* RELEASE_COMPLETE */, NULL, 0);

	    capi_ai_disconnect_ind(cd, complement);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *	How shorthold mode works for OUTGOING connections
 *	=================================================
 *
 *--->|<--------unitlen-------->|<--------unitlen-------->|<
 *----+-------------------------+-------------------------+--time->+
 *    |                         |                         | 
 *    |          |<--earlyhup-->|          |<**earlyhup-->| 
 *    |          |              |          |              | 
 *-idle_time---->|<--->|<-***idle_time---->|<--->|<----idle_
 *    |                         |                         | 
 *    |                         |          ^              | 
 *                                         +-hangup         
 *
 *	       **earlyhup_time must be less than unitlen_time
 *	       ***idle_time is not limited by unitlen_time
 *
 *	  unitlen - specifies the time a charging unit lasts
 *	idle_time - specifies the time the line must be idle before
 *		    it is closed
 *	 earlyhup - specifies the timing safety zone before the
 *		    next charging unit starts
 *
 *	The algorithm works as follows: lets assume the unitlen is 100
 *	seconds, idle_time is 40 seconds and earlyhup is 10 seconds.
 *	If the line has been without activity for 40 seconds, before
 *	the beginning of earlyhup, the line is closed 10 seconds
 *	before the next charging unit starts. Else it is not closed.
 *	In this setup an average of 100-40 = 60 seconds is not checked
 *	for traffic, because the idle_time is less than the unitlen.
 *
 *
 *	How shorthold mode works for INCOMING connections
 *	=================================================
 *
 *	it is just possible to specify a maximum idle time for incoming
 *	connections, after this time of no activity on the line, the line
 *	is closed
 *
 *---------------------------------------------------------------------------*/

#define i4b_get_last_active_time(cd) \
	 ((cd)->last_active_time)

/*---------------------------------------------------------------------------*
 *	B channel idle check timeout function
 *---------------------------------------------------------------------------*/
static void
i4b_idle_check(call_desc_t *cd)
{
	i4b_controller_t *cntl = 
	  i4b_controller_by_cd(cd);
	time_t time;

	CNTL_LOCK_ASSERT(cntl);

	if((cntl->N_fifo_translator) &&
	   (cd->cdid != CDID_UNUSED))
	{
	  /* connected */

	  switch(cd->idle_state)
	  {
	  case IST_CHECK:
	
		time = SECOND;

		if((cd->shorthold_data.idle_time <= 0) ||
		   ((time - i4b_get_last_active_time(cd)) < 
				cd->shorthold_data.idle_time))
		{
			/* activity */
			NDBGL4(L4_TIMO, "%ld: activity at %ld",
			       (long)time, (long)i4b_get_last_active_time(cd));

			if(cd->shorthold_data.earlyhup_time <= 0)
			{
			case IST_STARTED:

			  if(cd->aocd_flag == 0)
			  {
			    cd->units_type = CHARGE_CALC;
			    cd->cunits++;
			    i4b_l4_charging_ind(cd);
			  }

			  time = cd->shorthold_data.unitlen_time;
			  cd->idle_state = IST_CHECK;
			}
			else
			{
			  time = cd->shorthold_data.earlyhup_time;
			  cd->idle_state = IST_STARTED;
			}

			/* start timeout */
			callout_reset(&cd->idle_callout, time*hz,
					(void *)(void *)i4b_idle_check, cd);
			break;
		}
		else
		{
			/* no activity - hangup */
			NDBGL4(L4_TIMO, "%ld: last activity at %ld",
			       (long)time, (long)i4b_get_last_active_time(cd));

			/* send timeout_indication before
			 * N_DISCONNECT_REQUEST, hence the
			 * calldescriptor might be freed at
			 * N_DISCONNECT_REQUEST
			 */
			cd->idle_state = IST_NOT_STARTED;
			i4b_l4_idle_timeout_ind(cd);

			N_DISCONNECT_REQUEST(cd, (CAUSET_I4B << 8) | 
					     CAUSE_I4B_NORMAL);
		}
		break;

	  default:
		NDBGL4(L4_ERR, "called too early!");
		break;
	  }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	B channel idle check timeout setup
 *
 * NOTE: this routine can only be called once,
 *	 after that the shorthold-data has been set! 
 *---------------------------------------------------------------------------*/ 
void
i4b_l4_setup_timeout(call_desc_t *cd)
{
	uint16_t next_unitlen_time;

	next_unitlen_time = 0;

	NDBGL4(L4_TIMO, "%s, alg %d, unitlen %d, idle %d, earlyhup %d!",
	       cd->dir_incoming ? "incoming" : "outgoing",
	       cd->shorthold_data.shorthold_algorithm,
	       cd->shorthold_data.unitlen_time,
	       cd->shorthold_data.idle_time,
	       cd->shorthold_data.earlyhup_time);

	if(cd->dir_incoming)
	{
	  NDBGL4(L4_TIMO, "simple max idletime check");

	  cd->shorthold_data.unitlen_time = 1; /* second */
	  cd->shorthold_data.earlyhup_time = 0; /* seconds */
	  cd->shorthold_data.idle_time =
	    cd->shorthold_data.idle_time;

	  /* no charge */
	  cd->aocd_flag = 1;
	}
	else
	{
	  /* outgoing call */
	  if((cd->shorthold_data.unitlen_time > 0) &&
	     (cd->shorthold_data.unitlen_time > 
	      cd->shorthold_data.earlyhup_time))
	  {
			if(cd->shorthold_data.shorthold_algorithm == SHA_VARU)
			{
			  NDBGL4(L4_TIMO, "variable unit algorithm");

			  /* check for activity one second before the
			   * end of the unit. The one second takes
			   * into account of rounding due to the
			   * driver only using the seconds and not the
			   * micro-seconds of the current time
			   */

			  cd->shorthold_data.unitlen_time = 
			    cd->shorthold_data.unitlen_time -1;

			  cd->shorthold_data.earlyhup_time = 0; /* seconds */

			  cd->shorthold_data.idle_time =
			    cd->shorthold_data.idle_time;

			  next_unitlen_time = 1; /* second */

			  /* no charge */
			  cd->aocd_flag = 1;
			}
			else
			{
			  NDBGL4(L4_TIMO, "fixed unit algorithm");

			  cd->shorthold_data.unitlen_time =
			    cd->shorthold_data.unitlen_time -
			    cd->shorthold_data.earlyhup_time;

			  cd->shorthold_data.earlyhup_time =
			    cd->shorthold_data.earlyhup_time;

			  cd->shorthold_data.idle_time =
			    cd->shorthold_data.idle_time;

			  /* ``cd->aocd_flag'' is initialized
			   * at I4B_CONNECT_REQ
			   */
 			}
	  }
	  else
	  {
		if(cd->shorthold_data.unitlen_time || 
		   cd->shorthold_data.earlyhup_time)
		{
		  /* parms somehow got wrong .. */

		  NDBGL4(L4_ERR, "ERROR: earlyhup[%d] > unitlength[%d], "
			 "only idletime is used!",
			 cd->shorthold_data.earlyhup_time,
			 cd->shorthold_data.unitlen_time);
		}

		/* fall into the old fix algorithm */

		NDBGL4(L4_TIMO, "simple max idletime check");

		cd->shorthold_data.unitlen_time = 1; /* second */
		cd->shorthold_data.earlyhup_time = 0; /* seconds */

		cd->shorthold_data.idle_time =
		  cd->shorthold_data.idle_time;

		/* no charge */
		cd->aocd_flag = 1;
	  }
	}

	/* unitlen_time must always be valid;
	 * idle_time and earlyhup_time is checked by 
	 * ``i4b_idle_check''
	 */
	if(cd->idle_state == IST_NOT_STARTED)
	{
	  if((cd->aocd_flag == 0) || 
	     (cd->shorthold_data.idle_time > 0))
	  {
	    cd->idle_state = IST_STARTED;

	    /* start idle check */
	    i4b_idle_check(cd);
	  }
	  else
	  {
	    NDBGL4(L4_TIMO, "no idle_timeout configured");
	  }
	}

	if(next_unitlen_time)
	{
	  cd->shorthold_data.unitlen_time =
	    next_unitlen_time;
	}
	return;
}

void
i4b_l3_information_req(struct call_desc *cd, uint8_t *ptr, uint16_t len)
{
	char *dst = &(cd->dst_telno[0]);
	char *dst_end = &(cd->dst_telno[TELNO_MAX-1]);

	enum { max_telno = sizeof(cd->dst_telno_part)-1 };

	if(len > max_telno)
	{
	    len = max_telno;
	}

	bcopy(ptr, &(cd->dst_telno_part[0]), len);
	cd->dst_telno_part[len] = 0; /* zero terminate */

	if(!(cd->dir_incoming))
	{
	    /* append digits to destination 
	     * telephone number
	     */
	    if(cd->dst_telno_ptr < dst)
	    {
	        cd->dst_telno_ptr = dst;
	    }
	    else
	    {
	        dst = cd->dst_telno_ptr;

		if(dst > dst_end)
		{
		    dst = dst_end;
		}
	    }

	    len = min(len, (dst_end - dst));
	    bcopy(ptr, dst, len);
	    dst[len] = 0; /* zero terminate */
	    cd->dst_telno_ptr = dst + len;
	}
	N_INFORMATION_REQUEST(cd);
	return;
}
