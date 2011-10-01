/*
 * Copyright (c) 1999 Hellmuth Michaelis. All rights reserved.
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
 *	isdn.h - isdnphone isdn-handling
 *	================================
 *
 * $FreeBSD: $
 *
 *      last edit-date: []
 *
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 *	dialer init
 *---------------------------------------------------------------------------*/
static int
init_dial(char *device)
{
	int ret;
	
	if((ret = open(device, O_RDWR)) < 0)
		exit_fatal("unable to open %s: %s\n",
			   device, strerror(errno));

	return(ret);
}

static const struct { uint8_t event; uint8_t exitvalue; const char * desc; }
MAKE_TABLE(DSTATS,IPHONE_TABLE,[]);

/*---------------------------------------------------------------------------*
 *	dialer data handler
 *---------------------------------------------------------------------------*/
static void
dial_hdlr(void)
{
	u_int8_t result;

	if((read (dialerfd, &result, 1) < 0))
	{
		exit_fatal("read failed: %s", strerror(errno));
	}

	debug("result=0x%02x\n", result);

	if(result >= N_DSTATS)
	{
	  update_state(EV_UPDATE,
		       "ignored unknown response (please recompile this program)");
	}
	else
	{
	  /* please see ``i4b_ioctl.h'' */

	  update_state(DSTATS_IPHONE_TABLE[result].event,
		       DSTATS_IPHONE_TABLE[result].desc);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	dial number
 *---------------------------------------------------------------------------*/
static void
do_dial(char *number)
{
	char commandbuffer[80];	
	sprintf(commandbuffer, "D%s", number);

	if((write(dialerfd, commandbuffer, strlen(commandbuffer))) < 0)
	{
		exit_fatal("write commandbuffer failed: %s",
			   strerror(errno));
	}
	return;
}

#define do_hangup() do_command(CMD_HANGUP)
#define do_answer() do_command(CMD_ANSWER)
#define do_reject() do_command(CMD_REJECT)

/*---------------------------------------------------------------------------*
 *	command
 *---------------------------------------------------------------------------*/
static void
do_command(u_int8_t command)
{
	if(write(dialerfd, &command, sizeof(command)) < 0)
	{
		exit_fatal("write command '%c' failed: %s",
		      command, strerror(errno));
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	telephone init
 *---------------------------------------------------------------------------*/
static int
init_tel(char *device, int isdn_input_format)
{
	int ret;

	if((ret = open(device, O_RDWR)) < 0)
		exit_fatal("unable to open %s: %s\n",
			   device, strerror(errno));

	if((ioctl(ret, I4B_TEL_SETAUDIOFMT, &isdn_input_format)) < 0)
		exit_fatal("ioctl I4B_TEL_SETAUDIOFMT failed: %s",
			   strerror(errno));

	if((ioctl(ret, I4B_TEL_SET_SOUNDBRIDGE, NULL)) < 0)
		exit_fatal("ioctl I4B_TEL_SET_SOUNDBRIDGE failed: %s",
			   strerror(errno));

	if((ioctl(ret, I4B_TEL_SET_AUDIOAMP, &loudness)) < 0)
		exit_fatal("ioctl I4B_TEL_SET_AUDIOAMP failed: %s",
			   strerror(errno));

	return(ret);
}
/* EOF */
