/*-
 * Copyright (c) 1997, 1999 Hellmuth Michaelis. All rights reserved.
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
 *	i4b daemon - encoding of messages to kernel
 *	-------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

static const char * const
MAKE_TABLE(DSTATS,DESC,[]);

/*--------------------------------------------------------------------------*
 *	send I4B_RESPONSE_TO_USER to kernel
 *--------------------------------------------------------------------------*/
static void
dial_fail_response_to_user(u_int driver_type, u_int driver_unit)
{
	msg_response_to_user_t mrtu;

	BZERO(&mrtu);

	mrtu.controller = 0;
	mrtu.driver_type = driver_type;
	mrtu.driver_unit = driver_unit;
	mrtu.status = DSTAT_NO_ENTRY;
	
	if((ioctl(isdnfd, I4B_RESPONSE_TO_USER, &mrtu)) < 0)
	{
		log(LL_ERR, "ioctl I4B_RESPONSE_TO_USER failed: %s",
		    strerror(errno));
	}
	else
	{
		DBGL(DL_DRVR, (log(LL_DBG, "sent I4B_RESPONSE_TO_USER [%s]", 
				   DSTATS_DESC[DSTAT_NO_ENTRY])));
	}
	return;
}

/*--------------------------------------------------------------------------*
 *	send I4B_RESPONSE_TO_USER to kernel
 *--------------------------------------------------------------------------*/
static void
response_to_user(cfg_entry_t *cep, int dstat)
{
	msg_response_to_user_t mrtu;

	BZERO(&mrtu);

	mrtu.controller = 0;
	mrtu.driver_type = cep->usrdevicename;
	mrtu.driver_unit = cep->usrdeviceunit;
	mrtu.status = dstat;
	mrtu.cause = cep->disc_cause;	

	if(dstat == DSTAT_INCOMING_CALL)
	{
		/* copy display, sms and telephone number */

		strcpy(&mrtu.src_display[0], cep->display);

		strcpy(&mrtu.src_sms[0], cep->sms);

		strcpy(&mrtu.src_telno[0], cep->real_phone_incoming.number);
	}
	
	if((ioctl(isdnfd, I4B_RESPONSE_TO_USER, &mrtu)) < 0)
	{
		log(LL_ERR, "ioctl I4B_RESPONSE_TO_USER failed: %s",
		    strerror(errno));
	}
	else
	{
		DBGL(DL_DRVR, (log(LL_DBG, "sent I4B_RESPONSE_TO_USER [%s]", 
				   DSTATS_DESC[dstat])));
	}
	return;
}

/*---------------------------------------------------------------------------*
 *      send I4B_CONNECT_REQ to kernel
 *---------------------------------------------------------------------------*/
static int
sendm_connect_req(cfg_entry_t *cep)
{
        msg_connect_req_t mcr;
        int ret;

	/* get a cdid from kernel */
	cep->cdid = cep->isdncontrollerused;

	/* use ?? msg_cdid_req_t mcr; ?? */
	
	if((ret = ioctl(isdnfd, I4B_CDID_REQ, &cep->cdid)) < 0)
	{
		log(LL_ERR, "ioctl I4B_CDID_REQ failed: %s", strerror(errno));
		cep->cdid = CDID_UNUSED;
		goto done;
	}

	BZERO(&mcr);

	mcr.cdid = cep->cdid;

	mcr.channel = cep->isdnchannelused;
	mcr.txdelay = cep->isdntxdelout;

	mcr.bprot = cep->bprotocol;

	mcr.driver = cep->usrdevicename;
	mcr.driver_unit = cep->usrdeviceunit;

	/* setup the shorthold data */
	mcr.shorthold_data.shorthold_algorithm = cep->shorthold_algorithm;
	mcr.shorthold_data.unitlen_time = cep->unitlength;
	mcr.shorthold_data.idle_time = cep->idle_time_out;		
	mcr.shorthold_data.earlyhup_time = cep->earlyhangup;

	if(cep->unitlengthsrc == ULSRC_DYN)
		mcr.unitlen_method = ULEN_METHOD_DYNAMIC;
	else
		mcr.unitlen_method = ULEN_METHOD_STATIC;
	
	strcpy(mcr.dst_telno, cep->remote_phone_dialout.number);
	if(cep->usesubaddr)
		strcpy(mcr.dst_subaddr, cep->remote_phone_dialout.subaddr);
	strcpy(mcr.src_telno, cep->local_phone_dialout.number);
	if(cep->usesubaddr)
		strcpy(mcr.src_subaddr, cep->local_phone_dialout.subaddr);
	strcpy(mcr.keypad, cep->keypad);	

	DBGL(DL_CNST, (log(LL_DBG, "controller = %d, chan = %d",
			   cep->isdncontrollerused, cep->isdnchannelused)));
		
	if((ret = ioctl(isdnfd, I4B_CONNECT_REQ, &mcr)) < 0)
	{
		log(LL_ERR, "ioctl I4B_CONNECT_REQ failed: %s",
		    strerror(errno));
	}
	else
	{
		log(LL_CHD, "%05d %s dialing out from %s to %s",
		    cep->cdid,
		    cep->name,
		    aliasing ? get_alias(cep->local_phone_dialout.number) : cep->local_phone_dialout.number,
		    aliasing ? get_alias(cep->remote_phone_dialout.number) : cep->remote_phone_dialout.number);
	}
 done:
	return(ret);
}

/*---------------------------------------------------------------------------*
 *	send I4B_CONNECT_RESP to kernel
 *---------------------------------------------------------------------------*/
static int
sendm_connect_resp(cfg_entry_t *cep, int response, cause_t cause)
{
	msg_connect_resp_t mcr;
	int ret;

	BZERO(&mcr);

	mcr.cdid = cep->cdid;

	mcr.response = response;

	if(response == SETUP_RESP_REJECT)
	{
		mcr.cause = cause;
		DBGL(DL_DRVR, (log(LL_DBG, "reject, cause=0x%x", cause)));
	}
	else if(response == SETUP_RESP_ACCEPT)
	{
		mcr.txdelay = cep->isdntxdelin;

		mcr.bprot = cep->bprotocol;

		mcr.driver = cep->usrdevicename;
		mcr.driver_unit = cep->usrdeviceunit;

		mcr.max_idle_time = cep->idle_time_in;

		DBGL(DL_DRVR, (log(LL_DBG, "accept")));
	}
	
	if((ret = ioctl(isdnfd, I4B_CONNECT_RESP, &mcr)) < 0)
	{
		log(LL_ERR, "ioctl I4B_CONNECT_RESP failed: %s",
		    strerror(errno));
	}
	else
	{
		DBGL(DL_DRVR, (log(LL_DBG, "sent I4B_CONNECT_RESP")));
	}
	return(ret);
}

/*---------------------------------------------------------------------------*
 *	send I4B_DISCONNECT_REQ to kernel
 *---------------------------------------------------------------------------*/
static int
sendm_disconnect_req(cfg_entry_t *cep, cause_t cause)
{
	msg_disconnect_req_t mcr;
	int ret;

	BZERO(&mcr);

	mcr.cdid = cep->cdid;

	mcr.cause = cause;

	if((ret = ioctl(isdnfd, I4B_DISCONNECT_REQ, &mcr)) < 0)
	{
		log(LL_ERR, "ioctl I4B_DISCONNECT_REQ failed: %s",
		    strerror(errno));
	}
	else
	{
		DBGL(DL_DRVR, (log(LL_DBG, "sent I4B_DISCONNECT_REQ")));
	}
	return(ret);
}

/*---------------------------------------------------------------------------*
 *	send I4B_ALERT_REQ to kernel
 *---------------------------------------------------------------------------*/
static int
sendm_alert_req(cfg_entry_t *cep)
{
	msg_alert_req_t mar;
	int ret;

	BZERO(&mar);

	mar.cdid = cep->cdid;
	
	if((ret = ioctl(isdnfd, I4B_ALERT_REQ, &mar)) < 0)
	{
		log(LL_ERR, "ioctl I4B_ALERT_REQ failed: %s", strerror(errno));
	}
	else
	{
		DBGL(DL_DRVR, (log(LL_DBG, "sent I4B_ALERT_REQ")));
	}
	return(ret);
}
/* EOF */
