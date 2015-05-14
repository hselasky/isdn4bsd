/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
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
 *	msg_decoder.h - decoding of messages from kernel
 *	------------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 *	handle MSG_PROCEEDING_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_proceeding_ind(msg_proceeding_ind_t *mp)
{
	cfg_entry_t *cep;

	cep = get_cep_by_cdid(mp->header.cdid);

	if(cep)
	{
	    log(LL_CHD, "%05d %s outgoing call proceeding (controller %d)",
		cep->cdid, cep->name,
		cep->isdncontrollerused);

	    if((cep->dir_incoming == 0) && 
	       (cep->dialin_reaction == REACT_ALERT) &&
	       (cep->usrdevicename == DRVR_TEL))
	    {
	        /*
		 * connect B-channel early, in case
		 * the device is a telephone.
		 *
		 * XXX this should be implemented 
		 * like a separate option, but
		 * currently only "isdnphone" is
		 * using this
		 */

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
	    }
	}
	return;
}
                                                                                
/*---------------------------------------------------------------------------*
 *	handle MSG_L12STAT_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_l12stat_ind(msg_l12stat_ind_t *ml)
{
	int controller = CDID2CONTROLLER(ml->header.cdid);

	if((controller < 0) || (controller >= I4B_MAX_CONTROLLERS))
	{
		log(LL_ERR, "invalid controller number [%d]!", controller);
		return;
	}

#ifdef USE_CURSES
	if(do_fullscreen)
		display_l12stat(controller, ml->layer, ml->state);
#endif
#ifdef I4B_EXTERNAL_MONITOR
	if(do_monitor && accepted)
		monitor_evnt_l12stat(controller, ml->layer, ml->state);
#endif

	DBGL(DL_CNST, (log(LL_DBG, "unit %d, layer %d, state %d",
		controller, ml->layer, ml->state)));

	if(ml->layer == LAYER_ONE)
	{
		if(ml->state == LAYER_IDLE)
			isdn_ctrl_tab[controller].l2stat = ml->state;
		isdn_ctrl_tab[controller].l1stat = ml->state;
	}
	else if(ml->layer == LAYER_TWO)
	{
		if(ml->state == LAYER_ACTIVE)
			isdn_ctrl_tab[controller].l1stat = ml->state;
		isdn_ctrl_tab[controller].l2stat = ml->state;
	}
	else
	{
		log(LL_ERR, "invalid layer number [%d]!", ml->layer);
	}
	return;
}
                                                                                
/*---------------------------------------------------------------------------*
 *	handle MSG_TEIASG_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_teiasg_ind(msg_teiasg_ind_t *mt)
{
	int controller = CDID2CONTROLLER(mt->header.cdid);

	if((controller < 0) || (controller >= I4B_MAX_CONTROLLERS))
	{
		log(LL_ERR, "invalid controller number [%d]!", controller);
		return;
	}

#ifdef USE_CURSES
	if(do_fullscreen)
		display_tei(controller, mt->tei);
#endif
#ifdef I4B_EXTERNAL_MONITOR
	if(do_monitor && accepted)
		monitor_evnt_tei(controller, mt->tei);
#endif

	DBGL(DL_CNST, (log(LL_DBG, "unit %d, tei = %d",
		controller, mt->tei)));

	isdn_ctrl_tab[controller].tei = mt->tei;
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_NEGCOMPLETE_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_negcomplete_ind(msg_negcomplete_ind_t *mp)
{
	cfg_entry_t *cep;

	cep = get_cep_by_cdid(mp->header.cdid);

	if(cep)
	{
	  if(cep->connectprog)
	  {
	    exec_connect_prog(cep, cep->connectprog, 0);
	  }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_IFSTATE_CHANGED_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_ifstatechg_ind(msg_ifstatechg_ind_t *mp)
{
	cfg_entry_t *cep;

	cep = get_cep_by_cdid(mp->header.cdid);

	if(cep)
	{
	  log(LL_DBG, "%s%d: switched to state %d",
	      driver_name(cep->usrdevicename), cep->usrdeviceunit, mp->state);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_DIALOUT_IND from user
 *---------------------------------------------------------------------------*/
static void
msg_dialout(msg_dialout_ind_t *mp)
{
	cfg_entry_t *cep;

	cep = get_cep_by_driver(mp->driver,mp->driver_unit);

	if(cep)
	{
	  /**/
	  ev_dial_from_user(cep);
	}
	else
	{
	  dial_fail_response_to_user(mp->driver,mp->driver_unit);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_DIALOUTNUMBER_IND from user
 *---------------------------------------------------------------------------*/
static void
msg_dialoutnumber(msg_dialoutnumber_ind_t *mp)
{
	cfg_entry_t *cep;
	int j;

	cep = get_cep_by_driver(mp->driver,mp->driver_unit);

	if(cep)
	{
	  if(cep->state == ST_IDLE)
	  {
		/* check number and copy to cep->remote_numbers[] */
		
		for(j = 0; j < mp->cmdlen; j++)
		{
			if(!(isdigit(*(mp->cmd+j))))
			{
				log(LL_ERR,"%s, dial string contains "
				    "non-digit at pos %d", cep->name, j);
				return;
			}
			/* fill in number to dial */
			cep->remote_numbers[0].number[j] = *(mp->cmd+j);
		}				
		cep->remote_numbers[0].number[j] = '\0';

/* XXX subaddr does not have to be a digit! isgraph() would be a better idea */
		for(j = 0; j < mp->subaddrlen; j++)
		{
			if(!(isdigit(*(mp->subaddr+j))))
			{
				log(LL_ERR, "%s, subaddr string contains "
				    "non-digit at pos %d", cep->name, j);
				return;
			}
			/* fill in number to dial */
			cep->remote_numbers[0].subaddr[j] = *(mp->subaddr+j);
		}
		cep->remote_numbers[0].subaddr[j] = '\0';

		cep->remote_numbers_count = 1;

		/**/
		ev_dial_from_user(cep);
	  }
	  else
	  {
		log(LL_ERR,"cannot set remote_number. %s is in use!",
		    cep->name);
	  }
	}
	else
	{
	  dial_fail_response_to_user(mp->driver,mp->driver_unit);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_KEYPAD_IND from user
 *---------------------------------------------------------------------------*/
static void
msg_keypad(msg_keypad_ind_t *mp)
{
	cfg_entry_t *cep;

	cep = get_cep_by_driver(mp->driver,mp->driver_unit);

	if(cep)
	{
	  if(cep->state == ST_IDLE)
	  {
		cep->remote_numbers[0].number[0] = '\0';
		cep->remote_numbers_count = 0;
		cep->remote_phone_dialout.number[0] = '\0';
		
		BZERO(&cep->keypad);
		strncpy(cep->keypad, mp->cmd, mp->cmdlen);

		DBGL(DL_MSG, (log(LL_DBG, "%s: keypad string is %s",
				  cep->name, cep->keypad)));
		/**/
		ev_dial_from_user(cep);
	  }
	  else
	  {
		log(LL_ERR,"cannot set keypad. %s is in use!",
		    cep->name);
	  }
	}
	else
	{
	  dial_fail_response_to_user(mp->driver,mp->driver_unit);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_DRVRDISC_REQ from user
 *---------------------------------------------------------------------------*/
static void
msg_drvrdisc_req(msg_drvrdisc_req_t *mp)
{
	cfg_entry_t *cep;
	
	cep = get_cep_by_driver(mp->driver,mp->driver_unit);

	if(cep)
	{
	  /**/
	  ev_disconnect_from_user(cep);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_DRVRANSWER_REQ from user
 *---------------------------------------------------------------------------*/
static void
msg_drvranswer_req(msg_drvranswer_req_t *mp)
{
	cfg_entry_t *cep;

	cep = get_cep_by_driver(mp->driver,mp->driver_unit);

	if(cep)
	{
	  /**/
	  ev_connect_from_user(cep);
	}
	else
	{
	  dial_fail_response_to_user(mp->driver,mp->driver_unit);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_DRVRREJECT_REQ from user
 *---------------------------------------------------------------------------*/
static void
msg_drvrreject_req(msg_drvrreject_req_t *mp)
{
	cfg_entry_t *cep;

	cep = get_cep_by_driver(mp->driver,mp->driver_unit);

	if(cep)
	{
	  /**/
	  ev_reject_from_user(cep);
	}
	else
	{
	  dial_fail_response_to_user(mp->driver,mp->driver_unit);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_ACCOUNTING_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_accounting(msg_accounting_ind_t *mp)
{
	cfg_entry_t *cep;

	cep = get_cep_by_driver(mp->driver,mp->driver_unit);

	if(cep)
	{
	  if(cep->state == ST_CONNECTED)
	  {
	    cep->inbytes = mp->inbytes;
	    cep->iinbytes = mp->iinbytes;
	    cep->outbytes = mp->outbytes;
	    cep->ioutbytes = mp->ioutbytes;
	    cep->inbps = mp->inbps;
	    cep->outbps = mp->outbps;

	    if(mp->accttype == ACCT_DURING) 
	    {
#ifdef USE_CURSES
		if(do_fullscreen)
			display_acct(cep);
#endif
#ifdef I4B_EXTERNAL_MONITOR
		if(do_monitor && accepted)
			monitor_evnt_acct(cep);
#endif
	    }
	  }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	process AOCD charging messages
 *---------------------------------------------------------------------------*/
static void
handle_charge(cfg_entry_t *cep)
{
	time_t now = time(NULL);

	if(cep->aoc_last == 0)		/* no last timestamp yet ? */
	{
		cep->aoc_last = now;	/* add time stamp */
	}
	else if(cep->aoc_now == 0)	/* no current timestamp yet ? */
	{
		cep->aoc_now = now;	/* current timestamp */
	}
	else
	{
		cep->aoc_last = cep->aoc_now;
		cep->aoc_now = now;
		cep->aoc_diff = cep->aoc_now - cep->aoc_last;
		cep->aoc_valid = AOC_VALID;
	}
	
#ifdef USE_CURSES
	if(do_fullscreen)
		display_charge(cep);
#endif

#ifdef I4B_EXTERNAL_MONITOR
	if(do_monitor && accepted)
		monitor_evnt_charge(cep, cep->charge, 0);
#endif

	if(cep->aoc_valid == AOC_VALID)
	{
		if(cep->aoc_diff != cep->unitlength)
		{
			DBGL(DL_MSG, (log(LL_DBG, "AOCD unit length updated %d -> %d secs", cep->unitlength, cep->aoc_diff)));

			cep->unitlength = cep->aoc_diff;

			unitlen_chkupd(cep);
		}
		else
		{
#ifdef NOTDEF
			DBGL(DL_MSG, (log(LL_DBG, "AOCD unit length still %d secs", cep->unitlength)));
#endif
		}
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_CHARGING_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_charging_ind(msg_charging_ind_t *mp)
{
	static const char * const cttab[] = {
		"invalid",
		"AOCD",
		"AOCE",
		"estimated" };
		
	cfg_entry_t *cep;

	cep = get_cep_by_cdid(mp->header.cdid);

	if(cep)
	{
	  if((mp->units_type < CHARGE_INVALID) || 
	     (mp->units_type > CHARGE_CALC))
	  {
		log(LL_ERR, "units_type %d out of range!", mp->units_type);
		error_exit(1, "msg_charging_ind: units_type %d out of range!",
			   mp->units_type);
	  }
	
	  DBGL(DL_DRVR, (log(LL_DBG, "%d unit(s) (%s)",
			     mp->units, cttab[mp->units_type])));

	  cep->charge = mp->units;

	  switch(mp->units_type)
	  {
		case CHARGE_AOCD:
			if((cep->unitlengthsrc == ULSRC_DYN) &&
			   (cep->charge != cep->last_charge))
			{
				cep->last_charge = cep->charge;
				handle_charge(cep);
			}
			break;
			
		case CHARGE_CALC:
#ifdef USE_CURSES
		        if(do_fullscreen)
                	        display_ccharge(cep, mp->units);
#endif
#ifdef I4B_EXTERNAL_MONITOR
			if(do_monitor && accepted)
				monitor_evnt_charge(cep, mp->units, 1);
#endif
			break;
	  }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_IDLE_TIMEOUT_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_idle_timeout_ind(msg_idle_timeout_ind_t *mp)
{
	cfg_entry_t *cep;
	
	cep = get_cep_by_cdid(mp->header.cdid);

	if(cep)
	{
	  DBGL(DL_DRVR, (log(LL_DBG, "kernel sent disconnect!")));
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static char *
strapp(char *buf, const char *txt)
{
	while(*txt)
		*buf++ = *txt++;
	*buf = '\0';
	return buf;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static char *
ipapp(char *buf, unsigned long a )
{
	unsigned long ma = ntohl( a );

	buf += sprintf(buf, "%lu.%lu.%lu.%lu",
				(ma>>24)&0xFF,
				(ma>>16)&0xFF,
				(ma>>8)&0xFF,
				(ma)&0xFF);
	return buf;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_PACKET_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_packet_ind(msg_packet_ind_t *mp)
{
	cfg_entry_t *cep;
	struct ip *ip;
	u_char *proto_hdr;
	char tmp[80];
	char *cptr = tmp;
	const char *name;

	cep = get_cep_by_driver(mp->driver,mp->driver_unit);

	if(cep)
	{
		name = cep->name;
	}
	else
	{
		name = "???";
	}

	ip = (struct ip*)mp->pktdata;
	proto_hdr = mp->pktdata + ((ip->ip_hl)<<2);

	if( ip->ip_p == IPPROTO_TCP )
	{
		struct tcphdr* tcp = (struct tcphdr*)proto_hdr;

		cptr = strapp( cptr, "TCP " );
		cptr = ipapp( cptr, ip->ip_src.s_addr );
		cptr += sprintf( cptr, ":%u -> ", ntohs( tcp->th_sport ) );
		cptr = ipapp( cptr, ip->ip_dst.s_addr );
		cptr += sprintf( cptr, ":%u", ntohs( tcp->th_dport ) );

		if(tcp->th_flags & TH_FIN)  cptr = strapp( cptr, " FIN" );
		if(tcp->th_flags & TH_SYN)  cptr = strapp( cptr, " SYN" );
		if(tcp->th_flags & TH_RST)  cptr = strapp( cptr, " RST" );
		if(tcp->th_flags & TH_PUSH) cptr = strapp( cptr, " PUSH" );
		if(tcp->th_flags & TH_ACK)  cptr = strapp( cptr, " ACK" );
		if(tcp->th_flags & TH_URG)  cptr = strapp( cptr, " URG" );
	}
	else if( ip->ip_p == IPPROTO_UDP )
	{
		struct udphdr* udp = (struct udphdr*)proto_hdr;

		cptr = strapp( cptr, "UDP " );
		cptr = ipapp( cptr, ip->ip_src.s_addr );
		cptr += sprintf( cptr, ":%u -> ", ntohs( udp->uh_sport ) );
		cptr = ipapp( cptr, ip->ip_dst.s_addr );
		cptr += sprintf( cptr, ":%u", ntohs( udp->uh_dport ) );
	}
	else if( ip->ip_p == IPPROTO_ICMP )
	{
		struct icmp* icmp = (struct icmp*)proto_hdr;

		cptr += sprintf( cptr, "ICMP:%u.%u", icmp->icmp_type, icmp->icmp_code);
		cptr = ipapp( cptr, ip->ip_src.s_addr );
		cptr = strapp( cptr, " -> " );
		cptr = ipapp( cptr, ip->ip_dst.s_addr );
	}
	else
	{
		cptr += sprintf( cptr, "PROTO=%u ", ip->ip_p);
		cptr = ipapp( cptr, ip->ip_src.s_addr);
		cptr = strapp( cptr, " -> " );
		cptr = ipapp( cptr, ip->ip_dst.s_addr);
	}

	log(LL_PKT, "%s %s %u %s",
		name, mp->direction_out ? "send" : "recv",
		ntohs( ip->ip_len ), tmp );
	return;
}

/*---------------------------------------------------------------------------*
 *	handle MSG_INFORMATION_IND from kernel
 *---------------------------------------------------------------------------*/
static void
msg_information_ind(msg_information_ind_t *mp)
{
	msg_connect_replay_req_t mcrr;

	memset(&mcrr, 0, sizeof(mcrr));

	/* this indication indicates that one is
	 * getting additional dial digits.
	 * Ask the kernel to replay the connect 
	 * indication with the new dial digits
	 * added to the destination telephone
	 * number, hence this application is
	 * not keeping track of incoming calls
	 * which number does not have a match!
	 */

	mcrr.cdid = mp->header.cdid;

        if(ioctl(isdnfd, I4B_CONNECT_REPLAY_REQ, &mcrr) < 0)
        {
		log(LL_ERR, "cdid=%d: ioctl I4B_CONNECT_REPLAY_REQ "
		    "failed: %s", mp->header.cdid, strerror(errno));
        }
	return;
}

#define msg_disconnect_ind ev_disconnect_from_i4b
#define msg_alert_ind ev_alert_from_i4b
#define msg_connect_active_ind ev_connect_from_i4b
#define msg_connect_ind ev_incoming_from_i4b
/*---------------------------------------------------------------------------*
 *	process data from /dev/i4b
 *---------------------------------------------------------------------------*/
void
isdnrdhdl(void)
{
	static u_int8_t msg_rd_buf[2048];
	int error;

 repeat:
	error = read(isdnfd, &msg_rd_buf, sizeof(msg_rd_buf));

	if(error > 0)
	{
	    switch(((msg_hdr_t *)&msg_rd_buf)->type) {
	    case MSG_CONNECT_IND:				
	        msg_connect_ind((void *)&msg_rd_buf);
		break;

	    case MSG_INFORMATION_IND:
	        msg_information_ind((void *)&msg_rd_buf);
		break;

	    case MSG_CONNECT_ACTIVE_IND:
	        msg_connect_active_ind((void *)&msg_rd_buf);
		break;

	    case MSG_DISCONNECT_IND:
	        msg_disconnect_ind((void *)&msg_rd_buf,NULL);
		break;
				
	    case MSG_DIALOUT_IND:
	        msg_dialout((void *)&msg_rd_buf);
		break;

	    case MSG_ACCT_IND:
	        msg_accounting((void *)&msg_rd_buf);
		break;

	    case MSG_IDLE_TIMEOUT_IND:
	        msg_idle_timeout_ind((void *)&msg_rd_buf);
		break;

	    case MSG_CHARGING_IND:
	        msg_charging_ind((void *)&msg_rd_buf);
		break;

	    case MSG_PROCEEDING_IND:
	        msg_proceeding_ind((void *)&msg_rd_buf);
		break;

	    case MSG_ALERT_IND:
	        msg_alert_ind((void *)&msg_rd_buf);
		break;

	    case MSG_DRVRDISC_REQ:
	        msg_drvrdisc_req((void *)&msg_rd_buf);
		break;

	    case MSG_DRVRANSWER_REQ:
	        msg_drvranswer_req((void *)&msg_rd_buf);
		break;

	    case MSG_DRVRREJECT_REQ:
	        msg_drvrreject_req((void *)&msg_rd_buf);
		break;

	    case MSG_L12STAT_IND:
	        msg_l12stat_ind((void *)&msg_rd_buf);
		break;

	    case MSG_TEIASG_IND:
	        msg_teiasg_ind((void *)&msg_rd_buf);
		break;

	    case MSG_NEGCOMP_IND:
	        msg_negcomplete_ind((void *)&msg_rd_buf);
		break;

	    case MSG_IFSTATE_CHANGED_IND:
	        msg_ifstatechg_ind((void *)&msg_rd_buf);
		break;

	    case MSG_DIALOUTNUMBER_IND:
	        msg_dialoutnumber((void *)&msg_rd_buf);
		break;

	    case MSG_PACKET_IND:
	        msg_packet_ind((void *)&msg_rd_buf);
		break;

	    case MSG_KEYPAD_IND:
	        msg_keypad((void *)&msg_rd_buf);
		break;

	    case MSG_HOLD_IND:
	    case MSG_RETRIEVE_IND:
		/* just ignore */
		break;

	    default:
	        log(LL_WRN, "ERROR, unknown message received from %s (0x%02x)", 
		    I4BDEVICE, msg_rd_buf[0]);
		break;
	    }
	    goto repeat;
	}
	else if(error == 0) goto repeat;
	else if((error < 0) && (errno != EWOULDBLOCK))
	{
	    log(LL_WRN, "ERROR, cannot read " I4BDEVICE ", %s",
		strerror(errno));
	}
	return;
}
