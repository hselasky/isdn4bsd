/*-
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
 *	isdnphone - header file
 *      =======================
 *
 *	$Id: defs.h,v 1.6 1999/12/13 21:25:26 hm Exp $ 
 *
 * $FreeBSD: src/usr.sbin/i4b/isdnphone/defs.h,v 1.2 2000/10/13 11:55:13 bde Exp $
 *
 *      last edit-date: [Mon Dec 13 21:52:46 1999]
 *
 *----------------------------------------------------------------------------*/

#include <curses.h>
#include <stdio.h>
#include <stdarg.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>

#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/param.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_tel_ioctl.h>

/* device file prefixes */

#define I4BTELDEVICE	"/dev/i4btel"
#define I4BTELDDEVICE	"/dev/i4bteld"

/* debugging */

#define exit_fatal(fmt,args...)			\
{						\
  fprintf(stderr, "\nFatal error: "		\
	  fmt "\n\n",## args);			\
  exit(1);					\
}						\
/**/

#define debug(fmt,args...)			\
{						\
  if(opt_d)					\
  {						\
    if(curses_ready)				\
    {						\
      wprintw(dbg_w, "%s: " fmt,		\
	       __FUNCTION__,## args);		\
      wrefresh(dbg_w);				\
    }						\
    else					\
    {						\
      printf("%s: " fmt,			\
	     __FUNCTION__,## args);		\
    }						\
  }						\
}						\
/**/

/* main window dimensions */

#define MW_ROW		5
#define MW_COL		8

#define MW_WIDTH	60
#define MW_HEIGHT	8

#define DB_ROW		15
#define DB_COL		1
#define DB_WID		79
#define DB_HGT		9

#define MW_STATEY	2
#define MW_STATEX	1
#define MW_STX		10

#define MW_NUMY		4
#define MW_NUMX		1
#define MW_NUX		10

#define MW_MSGY		6
#define MW_MSGX		1
#define MW_MSX		10

#define CR		0x0d
#define LF		0x0a
#define	TAB		0x09
#define CNTRL_C		0x03
#define	CNTRL_D		0x04
#define CNTRL_L		0x0c

#define STATES_MESSAGE(enum,value,desc,message) message
#define    STATES_DESC(enum,value,desc,message) desc
#define STATES(m)\
m(ST_IDLE      ,, "IDLE",\
  "press ENTER to dial number ...."\
)\
m(ST_INCOMING  ,, "INCOMING CALL",\
  "press ENTER to connect "\
  "or ESCAPE to ignore ...."\
)\
m(ST_OUTGOING  ,, "DIALING",\
"press ESCAPE to disconnect ...."\
)\
m(ST_ACTIVE    ,, "CONNECTED",\
"press ESCAPE to disconnect ...."\
)\
/**/

MAKE_ENUM(STATES,
	N_STATES);

#define EVENTS_DESC(enum,value,desc) #enum
#define EVENTS(m)\
m(EV_CALL_IN  ,,)\
m(EV_DISCONNECT  ,,)\
m(EV_CONNECTED  ,,)\
m(EV_ENTER  ,,)\
m(EV_EXIT  ,,)\
m(EV_MUTE  ,,)\
m(EV_LOUDNESS_UPDATE ,,)\
m(EV_LARGE_BUFFER  ,,)\
m(EV_UPDATE  ,,)\
/**/

MAKE_ENUM(EVENTS,
	N_EVENTS);

/* global variables */

WINDOW *main_w;			/* curses main window pointer */
WINDOW *dbg_w;
WINDOW *help_w;			/* set when help-window is shown */

u_int8_t curses_ready = 0;     	/* flag, curses display is initialized */
u_int8_t state = ST_IDLE;

int dialerfd;
int tel_fd;
int curx;
char numberbuffer[TELNO_MAX];

u_int8_t loudness = AUDIOAMP_DP;
u_int8_t large_buffer = 0;
u_int8_t mute = 0;
u_int8_t opt_d = 0;

/* EOF */
