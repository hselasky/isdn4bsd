/*-
 * Copyright (c) 1997, 2003 Hellmuth Michaelis. All rights reserved.
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
 *	i4b daemon - misc support routines
 *	----------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#include "isdnd.h"

/*---------------------------------------------------------------------------*
 *	find entry by drivertype and driverunit
 *---------------------------------------------------------------------------*/
cfg_entry_t *
get_cep_by_driver(int drivertype, int driverunit)
{
	CEP_FOREACH(cep,&cfg_entry_tab[0])
	{
	  if((cep->usrdevicename == drivertype) &&
	     (cep->usrdeviceunit == driverunit))
	  {
	    /* return first match!
	     *
	     * NOTE: the kernel can only connect
	     * one call to each drivertype/
	     * driverunit combination !
	     */
	    return(cep);
	  }
	}
	log(LL_WRN, "no entry found!");
	return(NULL);
}

/*---------------------------------------------------------------------------*
 *	return address of config entry identified by cdid
 *---------------------------------------------------------------------------*/
cfg_entry_t *
get_cep_by_cdid(int cdid)
{
	CEP_FOREACH(cep,&cfg_entry_tab[0])
	{
	  if(cep->cdid == cdid)
	  {
	    return(cep);
	  }
	}
	log(LL_WRN, "cdid(%05d) not found", cdid);
	return(NULL);
}

/*---------------------------------------------------------------------------*
 *	find entry by controller and channel, which is connected
 *---------------------------------------------------------------------------*/
cfg_entry_t *
get_cep_by_cc(int controller, int chan)
{
	if((controller >= 0) && (controller < I4B_MAX_CONTROLLERS) &&
	   (chan >= 0) && (chan < I4B_MAX_CHANNELS))
	{
	  CEP_FOREACH(cep,&cfg_entry_tab[0])
	  {
	    if((cep->isdnchannelused == chan) &&
	       (cep->isdncontrollerused == controller) &&
	       (cep->state == ST_CONNECTED))
	    {
	      return(cep);
	    }
	  }
	}
	return(NULL);
}

/*---------------------------------------------------------------------------*
 *	check if an incoming call matches this entry
 *
 * - returns 1 if match, else 0
 * - errors are logged with LL_CHD
 *---------------------------------------------------------------------------*/
int
number_matches(msg_connect_ind_t *mp, cfg_entry_t *cep)
{
	int n;

	/* check telephone-number(s) */

	/* could have allowed any number
	 * if ``cep->local_phone_incoming.number[0] == '*' ''
	 */
	if(strncmp(cep->local_phone_incoming.number, mp->dst_telno,
		   strlen(cep->local_phone_incoming.number)))
	{
		DBGL(DL_MSG, (log(LL_DBG, "%s, myno %s != incomingno %s", cep->name,
				  cep->local_phone_incoming.number, mp->dst_telno)));
		goto error;
	}

	if(cep->usesubaddr && strncmp(cep->local_phone_incoming.subaddr, mp->dst_subaddr, strlen(cep->local_phone_incoming.subaddr)))
	{
		DBGL(DL_MSG, (log(LL_DBG, "%s, mysubno %s != incomingsubno %s", cep->name,
				  cep->local_phone_incoming.subaddr, mp->dst_subaddr)));
		goto error;
	}

	/* check all allowed remote number's for this entry */

	for (n = 0; n < cep->incoming_numbers_count; n++)
	{
		number_t *in = &cep->remote_phone_incoming[n];

		if(in->number[0] == '*')
		{
		  goto check_subaddr;
		}

		if(strncmp(in->number, mp->src_telno, strlen(in->number)) == 0)
		{
		check_subaddr:
			if(cep->usesubaddr)
			{
			  if(in->subaddr[0] == '*')
			  {
			    break;
			  }
			  if(strncmp(in->subaddr, mp->src_subaddr, strlen(in->subaddr)) == 0)
			  {
			    break;
			  }
			}
			else
			{
			  break;
			}
		}
		DBGL(DL_MSG, (log(LL_DBG, "%s, remno %s != incomingfromno %s", cep->name,
				  in->number, mp->src_telno)));
	}

	if(n >= cep->incoming_numbers_count)
	{
		goto error;
	}

	/* check b-protocol */

	if(cep->bprotocol != mp->bprot)
	{
		DBGL(DL_MSG, (log(LL_DBG, "%s, bprot %d != incomingprot %d", cep->name,
				  cep->bprotocol, mp->bprot)));
		goto error;
	}

	/* 
	 * check if controller matches
	 */
	if(/* (cep->isdncontroller != -1) && XXX */
	   (cep->isdncontroller != (int)CDID2CONTROLLER(mp->header.cdid)))
	{ 
		log(LL_CHD, "%05d %s incoming call, controller %d != incoming %d",
		    mp->header.cdid, cep->name, 
		    cep->isdncontroller, CDID2CONTROLLER(mp->header.cdid));
		goto error;
	}

	/* check time interval */
		
	if(isvalidtime(cep) == 0)
	{
		DBGL(DL_MSG, (log(LL_DBG, "%s, time not valid!", cep->name)));
		goto error;
	}
		
	/* found a matching entry */

	return(1);

 error:
	return(0);
}

/*---------------------------------------------------------------------------*
 *	return driver type-name string
 *---------------------------------------------------------------------------*/
const char *
driver_name(int driver_type)
{
	static const char * const
	  MAKE_TABLE(I4B_DRIVERS,NAME,[]);

	return(((driver_type >= 0) && (driver_type < N_I4B_DRIVERS)) ?
	       I4B_DRIVERS_NAME[driver_type] : "unknown");
}

/*---------------------------------------------------------------------------*
 *	return driver device-name string
 *---------------------------------------------------------------------------*/
const char *
driver_devicename(int driver_type)
{
	static const char * const
	  MAKE_TABLE(I4B_DRIVERS,DEVICE,[]);

	return(((driver_type >= 0) && (driver_type < N_I4B_DRIVERS)) ?
	       I4B_DRIVERS_DEVICE[driver_type] : "null");
}

/*---------------------------------------------------------------------------*
 *	update kernel idle_time, earlyhup_time and unitlen_time
 *---------------------------------------------------------------------------*/
void
unitlen_chkupd(cfg_entry_t *cep)
{
	msg_timeout_upd_t tupd;

	BZERO(&tupd);

	tupd.cdid = cep->cdid;

	/* init the short hold data based on the shorthold algorithm type */
	
	switch(cep->shorthold_algorithm)
	{
		case SHA_FIXU:
			tupd.shorthold_data.shorthold_algorithm = SHA_FIXU;
			tupd.shorthold_data.unitlen_time = cep->unitlength;
			tupd.shorthold_data.idle_time = cep->idle_time_out;
			tupd.shorthold_data.earlyhup_time = cep->earlyhangup;
			break;

		case SHA_VARU:
			tupd.shorthold_data.shorthold_algorithm = SHA_VARU;
			tupd.shorthold_data.unitlen_time = cep->unitlength;
			tupd.shorthold_data.idle_time = cep->idle_time_out;
			tupd.shorthold_data.earlyhup_time = 0;
			break;
		default:
			log(LL_ERR, "bad shorthold_algorithm %d", cep->shorthold_algorithm );
			return;
			break;			
	}

	if((ioctl(isdnfd, I4B_TIMEOUT_UPD, &tupd)) < 0)
	{
		log(LL_ERR, "ioctl I4B_TIMEOUT_UPD failed: %s", strerror(errno));
	}
	return;
}

/*--------------------------------------------------------------------------*
 *	screening/presentation indicator
 *--------------------------------------------------------------------------*/
void
handle_scrprs(int cdid, int scr, int prs, char *caller)
{
	/* screening indicator */
	
	if((scr < SCR_NONE) || (scr > SCR_NET))
	{
		log(LL_ERR, "invalid screening indicator value %d!", scr);
	}
	else
	{
		static const char *scrtab[] = {
			"no screening indicator",
			"sreening user provided, not screened",
			"screening user provided, verified & passed",
			"screening user provided, verified & failed",
			"screening network provided", };

		if(extcallattr)
		{
			log(LL_CHD, "%05d %s %s", cdid, caller, scrtab[scr]);
		}
		else
		{
			DBGL(DL_MSG, (log(LL_DBG, "%s - %s", caller, scrtab[scr])));
		}
	}
			
	/* presentation indicator */
	
	if((prs < PRS_NONE) || (prs > PRS_RESERVED))
	{
		log(LL_ERR, "invalid presentation indicator value %d!", prs);
	}
	else
	{
		static const char *prstab[] = {
			"no presentation indicator",
			"presentation allowed",
			"presentation restricted",
			"number not available due to interworking",
			"reserved presentation value" };
			
		if(extcallattr)
		{
			log(LL_CHD, "%05d %s %s", cdid, caller, prstab[prs]);
		}
		else
		{
			DBGL(DL_MSG, (log(LL_DBG, "%s - %s", caller, prstab[prs])));
		}
	}
	return;
}

/*--------------------------------------------------------------------------*
 *	check if the time is valid for an entry
 *--------------------------------------------------------------------------*/
int 
isvalidtime(cfg_entry_t *cep)
{
	time_t t;
	struct tm *tp;

	if(cep->day == 0)
		return(1);

	t = time(NULL);
	tp = localtime(&t);

	if(cep->day & HD)
	{
		if(isholiday(tp->tm_mday, (tp->tm_mon)+1, (tp->tm_year)+1900))
		{
			DBGL(DL_VALID, (log(LL_DBG, "holiday %d.%d.%d", tp->tm_mday, (tp->tm_mon)+1, (tp->tm_year)+1900)));
			goto dayok;
		}
	}
	
	if(cep->day & (1 << tp->tm_wday))
	{
		DBGL(DL_VALID, (log(LL_DBG, "day match")));	
		goto dayok;
	}

	return(0);
	
dayok:
	if((cep->fromhr == 0) &&
	   (cep->frommin == 0) &&
	   (cep->tohr == 0) &&
	   (cep->tomin == 0))
	{
		DBGL(DL_VALID, (log(LL_DBG, "no time specified, match!")));
		return(1);
	}

	if(cep->tohr < cep->fromhr)
	{
		/* before 00:00 */
		
		if( (tp->tm_hour > cep->fromhr) ||
		    ((tp->tm_hour == cep->fromhr) && (tp->tm_min > cep->frommin)) )
		{
			DBGL(DL_VALID, (log(LL_DBG, "t<f-1, spec=%02d:%02d-%02d:%02d, curr=%02d:%02d, match!",
					    cep->fromhr, cep->frommin,
					    cep->tohr, cep->tomin,
					    tp->tm_hour, tp->tm_min)));
			return(1);
		}

		/* after 00:00 */
		
		if( (tp->tm_hour < cep->tohr) ||
		    ((tp->tm_hour == cep->tohr) && (tp->tm_min < cep->tomin)) )
		{
			DBGL(DL_VALID, (log(LL_DBG, "t<f-2, spec=%02d:%02d-%02d:%02d, curr=%02d:%02d, match!",
					    cep->fromhr, cep->frommin,
					    cep->tohr, cep->tomin,
					    tp->tm_hour, tp->tm_min)));
			return(1);
		}
	}
	else if(cep->fromhr == cep->tohr)
	{
		if((tp->tm_min >= cep->frommin) && (tp->tm_min < cep->tomin))
		{
			DBGL(DL_VALID, (log(LL_DBG, "f=t, spec=%02d:%02d-%02d:%02d, curr=%02d:%02d, match!",
					    cep->fromhr, cep->frommin,
					    cep->tohr, cep->tomin,
					    tp->tm_hour, tp->tm_min)));
			
			return(1);
		}
	}
	else
	{
		if( ((tp->tm_hour > cep->fromhr) && (tp->tm_hour < cep->tohr)) ||
		    ((tp->tm_hour == cep->fromhr) && (tp->tm_min >= cep->frommin)) ||
		    ((tp->tm_hour == cep->tohr) && (tp->tm_min < cep->tomin)) )
		{
			DBGL(DL_VALID, (log(LL_DBG, "t>f, spec=%02d:%02d-%02d:%02d, curr=%02d:%02d, match!",
					    cep->fromhr, cep->frommin,
					    cep->tohr, cep->tomin,
					    tp->tm_hour, tp->tm_min)));
			return(1);
		}
	}
	DBGL(DL_VALID, (log(LL_DBG, "spec=%02d:%02d-%02d:%02d, curr=%02d:%02d, no match!",
			    cep->fromhr, cep->frommin,
			    cep->tohr, cep->tomin,
			    tp->tm_hour, tp->tm_min)));
	return(0);	
}

/*--------------------------------------------------------------------------*
 *	prepend national or international prefix to a number
 *--------------------------------------------------------------------------*/
int
add_number_prefix(char *number, int type_of_number)
{
	char tmp[TELNO_MAX];
	char *prefix;
	int result = 0;

	if((type_of_number == TON_NATIONAL) ||
	   (type_of_number == TON_INTERNAT))
	{
		if(type_of_number == TON_NATIONAL)
			prefix = prefixnational;
		else 
			prefix = prefixinternational;
		
		/* Add prefix only if not already there */
		if(strncmp(number, prefix, strlen(prefix)) != 0)
		{
			snprintf(tmp, sizeof(tmp)-1, "%s%s", prefix, number);
			strncpy(number, tmp, TELNO_MAX-1);
			result = 1;
		}
	}
	return result;
}

/*--------------------------------------------------------------------------*
 *	this is intended to be called by exit and closes down all
 *	active connections before the daemon exits or is reconfigured.
 *--------------------------------------------------------------------------*/
void
close_allactive(void)
{
	int j;

	j = 0;

	CEP_FOREACH(cep,&cfg_entry_tab[0])
	{
	  if(cep->state != ST_IDLE)
	  {
	    j = 1;
	  }

	  /**/
	  ev_disconnect_from_user(cep);
	}

	if(j)
	{
		log(LL_DMN, "waiting for all connections terminated");
		sleep(5);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	get name of a controller
 *---------------------------------------------------------------------------*/
const u_char *
name_of_controller(u_int16_t controller)
{
	if(controller < I4B_MAX_CONTROLLERS)
	  return &isdn_ctrl_tab[controller].l1_desc[0];
	else
	  return "not present";
}

/*--------------------------------------------------------------------------*
 *	init active or capi controller
 *--------------------------------------------------------------------------*/
void
init_active_controller(void)
{
	int ret;
	int unit = 0;
	int controller;
	char cmdbuf[MAXPATHLEN+128];

	for(controller = 0; controller < I4B_MAX_CONTROLLERS; controller++)
	{
		if(isdn_ctrl_tab[controller].l1_type == L1_TYPE_TINADD)
		{
			DBGL(DL_RCCF, (log(LL_DBG, "tina-dd %d: executing [%s %d]", unit, tinainitprog, unit)));
			
			snprintf(cmdbuf, sizeof(cmdbuf), "%s %d", tinainitprog, unit);

			if((ret = system(cmdbuf)) != 0)
			{
				log(LL_ERR, "tina-dd %d: %s returned %d!", unit, tinainitprog, ret);
				exit(1);
			}
		}

		/*
		 *  Generic microcode loading. If a controller has
		 *  defined a microcode file, load it using the
		 *  I4B_CTRL_DOWNLOAD ioctl.
		 */
		
		if(isdn_ctrl_tab[controller].firmware != NULL)
		{
		    int fd;
		    struct isdn_dr_prot idp;
		    struct isdn_download_request idr;

		    fd = open(isdn_ctrl_tab[controller].firmware, O_RDONLY);
		    if (fd < 0) {
			log(LL_ERR, "%d: open %s: %s!",
			    controller, isdn_ctrl_tab[controller].firmware,
			    strerror(errno));
			exit(1);
		    }

		    idp.bytecount = lseek(fd, 0, SEEK_END);
		    idp.microcode = mmap(0, idp.bytecount, PROT_READ,
					 MAP_SHARED, fd, 0);
		    if (idp.microcode == MAP_FAILED) {
			log(LL_ERR, "%d: mmap %s: %s!",
			    controller, isdn_ctrl_tab[controller].firmware,
			    strerror(errno));
			exit(1);
		    }
		    
		    DBGL(DL_RCCF, (log(LL_DBG, "%d: loading firmware from [%s]", controller, isdn_ctrl_tab[controller].firmware)));

		    idr.controller = controller;
		    idr.numprotos = 1;
		    idr.protocols = &idp;
		    
		    ret = ioctl(isdnfd, I4B_CTRL_DOWNLOAD, &idr, sizeof(idr));
		    if (ret) {
			log(LL_ERR, "%d: load %s: %s!",
			    controller, isdn_ctrl_tab[controller].firmware,
			    strerror(errno));
			exit(1);
		    }

		    munmap(idp.microcode, idp.bytecount);
		    close(fd);
		}
	}
	return;
}

/*--------------------------------------------------------------------------*
 *	set channel state
 *--------------------------------------------------------------------------*/
int
set_channel_state(int controller, int channel, int state)
{
	if((controller < 0) || (controller >= I4B_MAX_CONTROLLERS))
	{
		log(LL_ERR, "invalid controller number [%d]!",
		    controller);
		return(ERROR);
	}
	if((channel < 0) || (channel >= I4B_MAX_CHANNELS))
	{
		log(LL_ERR, "controller [%d] "
		    "invalid channel [%d]!", controller, channel);
		return(ERROR);
	}
	if(isdn_ctrl_tab[controller].ch_state[channel] == state)
	{
		DBGL(DL_CNST, (log(LL_DBG, "controller [%d] "
				   "channel B%d already in state %d!",
				   controller, channel-CHAN_B1+1, state)));
	}
	else
	{
		isdn_ctrl_tab[controller].ch_state[channel] = state;

		DBGL(DL_CNST, (log(LL_DBG, "controller [%d] "
				   "channel B%d state set to %d!",
				   controller, channel-CHAN_B1+1, state)));
	}
	return(GOOD);
}

/*--------------------------------------------------------------------------*
 *	return channel state
 *--------------------------------------------------------------------------*/
int
ret_channel_state(int controller, int channel)
{
	if((controller < 0) || (controller >= I4B_MAX_CONTROLLERS))
	{
		log(LL_ERR, "invalid "
		    "controller number [%d]!", controller);
		return(ERROR);
	}
	if((channel < 0) || (channel >= I4B_MAX_CHANNELS))
	{
		log(LL_ERR, "controller [%d] "
		    "invalid channel [%d]!", controller, channel);
		return(ERROR);
	}
	return(isdn_ctrl_tab[controller].ch_state[channel]);
}

/*---------------------------------------------------------------------------*
 *	select the first remote number to dial according to the
 *	dial strategy
 *---------------------------------------------------------------------------*/
void
select_first_dialno(cfg_entry_t *cep)
{
	int i, j;

	if(cep->keypad[0] != '\0')
		return;

	if(cep->remote_numbers_count < 1)
	{
		log(LL_ERR, "remote_numbers_count < 1!");
		return;
	}

	if(cep->remote_numbers_count == 1)
	{
		strcpy(cep->remote_phone_dialout.number, cep->remote_numbers[0].number);
		strcpy(cep->remote_phone_dialout.subaddr, cep->remote_numbers[0].subaddr);
		DBGL(DL_DIAL, (log(LL_DBG, "only one no, no = %s", cep->remote_phone_dialout.number)));
		cep->last_remote_number = 0;
		return;
	}

	if(cep->remote_numbers_handling == RNH_FIRST)
	{
		strcpy(cep->remote_phone_dialout.number, cep->remote_numbers[0].number);
		strcpy(cep->remote_phone_dialout.subaddr, cep->remote_numbers[0].subaddr);
		DBGL(DL_DIAL, (log(LL_DBG, "use first, no = %s", cep->remote_phone_dialout.number)));
		cep->last_remote_number = 0;
		return;
	}

	i = cep->last_remote_number;
	   
	for(j = cep->remote_numbers_count; j > 0; j--)
	{
		if(cep->remote_numbers[i].flag == RNF_SUCC)
		{
			if(cep->remote_numbers_handling == RNH_LAST)
			{
				strcpy(cep->remote_phone_dialout.number, cep->remote_numbers[i].number);
				strcpy(cep->remote_phone_dialout.subaddr, cep->remote_numbers[i].subaddr);
				DBGL(DL_DIAL, (log(LL_DBG, "use last, no = %s", cep->remote_phone_dialout.number)));
				cep->last_remote_number = i;
				return;
			}
			else
			{
				if(++i >= cep->remote_numbers_count)
					i = 0;

				strcpy(cep->remote_phone_dialout.number, cep->remote_numbers[i].number);
				strcpy(cep->remote_phone_dialout.subaddr, cep->remote_numbers[i].subaddr);
				DBGL(DL_DIAL, (log(LL_DBG, "use next, no = %s", cep->remote_phone_dialout.number)));
				cep->last_remote_number = i;
				return;
			}
		}

		if(++i >= cep->remote_numbers_count)
			i = 0;
	}
	strcpy(cep->remote_phone_dialout.number, cep->remote_numbers[0].number);
	DBGL(DL_DIAL, (log(LL_DBG, "no last found (use 0), no = %s", cep->remote_phone_dialout.number)));
	cep->last_remote_number = 0;
	return;
}

/*---------------------------------------------------------------------------*
 *	select next remote number to dial (last was unsuccesfull)
 *---------------------------------------------------------------------------*/
void
select_next_dialno(cfg_entry_t *cep)
{
	if(cep->remote_numbers_count < 1)
	{
		log(LL_ERR, "remote_numbers_count < 1!");
		return;
	}

	if(cep->remote_numbers_count == 1)
	{
		strcpy(cep->remote_phone_dialout.number, cep->remote_numbers[0].number);
		strcpy(cep->remote_phone_dialout.subaddr, cep->remote_numbers[0].subaddr);
		DBGL(DL_DIAL, (log(LL_DBG, "only one no, no = %s", cep->remote_phone_dialout.number)));
		cep->last_remote_number = 0;
		return;
	}

	/* mark last try as bad */

	cep->remote_numbers[cep->last_remote_number].flag = RNF_IDLE;

	/* next one to try */
	
	cep->last_remote_number++;

	if(cep->last_remote_number >= cep->remote_numbers_count)
		cep->last_remote_number = 0;

	strcpy(cep->remote_phone_dialout.number, cep->remote_numbers[cep->last_remote_number].number);
	
	DBGL(DL_DIAL, (log(LL_DBG, "index=%d, no=%s",
		cep->last_remote_number,
		cep->remote_numbers[cep->last_remote_number].number)));
	return;
}

/*---------------------------------------------------------------------------*
 *	dial succeded, store this number as the last successful
 *---------------------------------------------------------------------------*/
void
select_this_dialno(cfg_entry_t *cep)
{
	cep->remote_numbers[cep->last_remote_number].flag = RNF_SUCC;
	
	DBGL(DL_DIAL, (log(LL_DBG, "index = %d, no = %s",
		cep->last_remote_number,
		cep->remote_numbers[cep->last_remote_number].number)));
	return;
}

/*---------------------------------------------------------------------------*
 *	check if another instance of us is already running
 *---------------------------------------------------------------------------*/
void
check_pid(void)
{
	FILE *fp;
	
	/* check if another lock-file already exists */

	if((fp = fopen(PIDFILE, "r")) != NULL)
	{
		/* lockfile found, check */
		
		int oldpid;

		/* read pid from file */
		
		if((fscanf(fp, "%d", &oldpid)) != 1)
		{
			log(LL_ERR, "ERROR, reading pid from lockfile failed, terminating!");
			exit(1);
		}

		/* check if process got from file is still alive */
		
		if((kill(oldpid, 0)) != 0)
		{
			/* process does not exist */

			/* close file */

			fclose(fp);

			DBGL(DL_PROC, (log(LL_DBG, "removing old lock-file %s", PIDFILE)));

			/* remove file */
			
			unlink(PIDFILE);
		}
		else
		{
			/* process is still alive */
			
			log(LL_ERR, "ERROR, another daemon is already running, pid = %d, terminating!", oldpid);
			exit(1);
		}
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	establish and init process lock file
 *---------------------------------------------------------------------------*/
void
write_pid(void)
{
	FILE *fp;
	
	/* write my pid into lock-file */
	
	if((fp = fopen(PIDFILE, "w")) == NULL)
	{
		log(LL_ERR, "ERROR, can't open lockfile for writing, terminating");
		exit(1);
	}

	if((fprintf(fp, "%d", (int)getpid())) == EOF)
	{
		log(LL_ERR, "ERROR, can't write pid to lockfile, terminating");
		exit(1);
	}

	fsync(fileno(fp));

	fclose(fp);
	return;
}

/*---------------------------------------------------------------------------*
 *	become a daemon
 *---------------------------------------------------------------------------*/
void
daemonize(void)
{
	int fd;

	switch (fork())
	{
		case -1:		/* error */
			log(LL_ERR, "ERROR, daemonize/fork: %s", strerror(errno));
			exit(1);
		case 0:			/* child */
			break;
		default:		/* parent */
			_exit(0);
	}

	/* new session / no control tty */

	if(setsid() == -1)
	{
		log(LL_ERR, "ERROR, setsid returns: %s", strerror(errno));
		exit(1);
	}

	/* go away from mounted dir */
	
	chdir("/");

	/* move i/o to another device ? */
	
	if(do_fullscreen && do_rdev)
	{
		char *tp;
		
		if((fd = open(rdev, O_RDWR, 0)) != -1)
		{
			if(!isatty(fd))
			{
				log(LL_ERR, "ERROR, device %s is not a tty!", rdev);
				exit(1);
			}
			if((dup2(fd, STDIN_FILENO)) == -1)
			{
				log(LL_ERR, "ERROR, dup2 stdin: %s", strerror(errno));
				exit(1);
			}				
			if((dup2(fd, STDOUT_FILENO)) == -1)
			{
				log(LL_ERR, "ERROR, dup2 stdout: %s", strerror(errno));
				exit(1);
			}				
			if((dup2(fd, STDERR_FILENO)) == -1)
			{
				log(LL_ERR, "ERROR, dup2 stderr: %s", strerror(errno));
				exit(1);
			}				
		}
		else
		{
			log(LL_ERR, "ERROR, cannot open redirected device: %s", strerror(errno));
			exit(1);
		}
			
		if(fd > 2)
		{
			if((close(fd)) == -1)
			{
				log(LL_ERR, "ERROR, close in daemonize: %s", strerror(errno));
				exit(1);
			}				
		}

		/* curses output && fork NEEDS controlling tty */
		
		if((ioctl(STDIN_FILENO, TIOCSCTTY, (char *)NULL)) < 0)
		{
			log(LL_ERR, "ERROR, cannot setup tty as controlling terminal: %s", strerror(errno));
			exit(1);
		}

		/* in case there is no environment ... */

		if(((tp = getenv("TERM")) == NULL) || (*tp == '\0'))
		{
			if(do_ttytype == 0)
			{
				log(LL_ERR, "ERROR, no environment variable TERM found and -t not specified!");
				exit(1);
			}

			if((setenv("TERM", ttype, 1)) != 0)
			{
				log(LL_ERR, "ERROR, setenv TERM=%s failed: %s", ttype, strerror(errno));
				exit(1);
			}
		}
	}
	return;
}

