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
 *	I4B daemon - main header file
 *	-----------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdnd/isdnd.h,v 1.18 2002/08/12 07:52:39 hm Exp $
 *
 *      last edit-date: [Sun Aug 11 12:31:44 2002]
 *
 *---------------------------------------------------------------------------*/

#ifndef _ISDND_H_
#define _ISDND_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <ctype.h>
#include <syslog.h>
#include <regex.h>
#include <time.h>
#include <paths.h>
#include <errno.h>
#include <netdb.h>

#ifdef USE_CURSES
#include <curses.h>
#endif

#include <fcntl.h>
#include <signal.h>

#include <sys/queue.h>	/* TAILQ_ macros */
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/un.h>

#ifdef __NetBSD__
#undef USE_RTPRIO
#endif

#ifdef USE_RTPRIO
#include <sys/rtprio.h>
#endif

#include <arpa/inet.h>
#include <net/if.h>
#include <net/if_types.h>
#include <net/if_sppp.h>

#if defined(__FreeBSD__)
#include <net/if_var.h>
#endif

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <netinet/ip_icmp.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>

#include "config.h"		/* compile time configuration	*/
#include "alias.h"		/* alias file processing	*/
#include "monitor.h"

/*---------------------------------------------------------------------------*
 *	some general definitions
 *---------------------------------------------------------------------------*/
#define GOOD	0		/* general "good" or "ok" return*/
#define ERROR	(-1)		/* general error return		*/
#define INVALID (-1)		/* an invalid integer		*/
#define BZERO(ptr) bzero(ptr,sizeof(*ptr))

/*---------------------------------------------------------------------------*
 *	misc
 *---------------------------------------------------------------------------*/
#define RTPRIO_NOTUSED (-1)	/* rtprio is not used for isdnd	*/

/*---------------------------------------------------------------------------*
 *	debug flag bits
 *---------------------------------------------------------------------------*/
#define DL_MSG		0x0001	/* general debug messages		*/
#define DL_RATES	0x0002	/* messages related to rates		*/
#define DL_TIME		0x0004	/* messages related to timing		*/
#define DL_STATE	0x0008	/* messages related to states changes	*/
#define DL_RCVRY	0x0010	/* messages related to dial recovery	*/
#define DL_DIAL		0x0020	/* messages related to dial recovery	*/
#define DL_PROC		0x0040	/* messages related to process handling	*/
#define DL_DRVR		0x0080	/* messages related to kernel i4b msg i/o*/
#define DL_CNST		0x0100	/* messages related to controller state	*/
#define DL_RCCF		0x0200	/* messages related to isdnd.rc at boot	*/
#define DL_BDGT		0x0400	/* messages related to budgets		*/
#define DL_VALID	0x0800	/* messages related to valid keyword	*/

#ifdef DEBUG
#define DBGL(cond, dolog) { if(debug_flags & (cond)) { dolog; }}
#else
#define DBGL(cond, dolog)
#endif

/*---------------------------------------------------------------------------*
 *	curses fullscreen display definitions
 *---------------------------------------------------------------------------*/

/* window dimensions */
#define UPPER_B		2		/* upper window start  */

/* horizontal positions for upper window */
#define H_CNTL		0		/* controller		*/
#define H_TEI		2		/* TEI			*/
#define H_CHAN		(H_TEI+4)	/* channel		*/
#define H_TELN		(H_CHAN+2)	/* telephone number	*/
#define H_IFN		(H_TELN+23)	/* interfacename	*/
#define H_IO		(H_IFN+7)	/* incoming or outgoing */
#define H_OUT		(H_IO+4)	/* # of bytes out	*/
#define H_OUTBPS	(H_OUT+11)	/* bytes per second out	*/
#define H_IN		(H_OUTBPS+5)	/* # of bytes in	*/
#define H_INBPS		(H_IN+11)	/* bytes per second in	*/
#define H_UNITS		(H_INBPS+6)	/* # of charging units	*/

/* fullscreen mode menu window */
#define WMENU_LEN 	35		/* width of menu window */
#define WMENU_TITLE 	"Command"	/* title string */
#define WMENU_POSLN	10		/* menu position, line */
#define WMENU_POSCO	5		/* menu position, col */
#define WMENU_HGT 	(N_WMITEMS + 4)	/* menu window height */

#define WMITEMS_DESC(enum,value,desc) desc
#define WMITEMS(m)\
m(WREFRESH      ,,"1 - (D)isplay refresh")\
m(WHANGUP       ,,"2 - (H)angup (choose a channel)")\
m(WREREAD       ,,"3 - (R)eread config file")\
m(WSHOW         ,,"4 - (S)how card types")\
m(WBUDGET       ,,"5 - (B)udget information")\
m(WQUIT         ,,"6 - (Q)uit the program")\
/**/

MAKE_ENUM(WMITEMS,
	N_WMITEMS);

#define WMTIMEOUT	5		/* timeout in seconds */

/*---------------------------------------------------------------------------*
 *	charging rates
 *---------------------------------------------------------------------------*/
#define NDAYS	7	/* number of days in a week		*/
#define NRATES	4	/* number of rate structures supported 	*/

/* struct for rates - each day has one or more */
struct rates
{
	int start_time;	/* hour and min at which this rate starts, e.g. 12:20 12*60+20*/
	int end_time;	/* hour and min at which this rate ends, e.g. 19:10 19*60+10*/
	int rate;	/* how long can I telephone at this price, seconds */
	struct rates *next;
};

/*---------------------------------------------------------------------------*
 * the internal identifiers for isdnd log levels
 *---------------------------------------------------------------------------*/
#define LOGIDS_TABLE(enum,value,text,pri,desc) { text, pri }
#define LOGIDS(m)\
m(LL_ERR,,"ERR", LOG_ERR,\
  "error conditions - everything which caused an error")\
m(LL_WRN,,"WRN", LOG_WARNING,\
  "warning conditions - nonfatal abnormal conditions")\
m(LL_DMN,,"DMN", LOG_NOTICE,\
  "normal but significant condition - status of daemon")\
m(LL_CHD,,"CHD", LOG_INFO,\
  "informational - everything regarding call handling")\
m(LL_DBG,,"DBG", LOG_DEBUG,\
  "debug messages - everything which helps debugging")\
m(LL_MER,,"MER", LOG_ERR,\
  "monitor error messages - not sent to remote")\
m(LL_PKT,,"PKT", LOG_INFO,\
  "packet logging - log the first few packets")\
/**/

MAKE_ENUM(LOGIDS);

/*---------------------------------------------------------------------------*
 *	state machine states
 *---------------------------------------------------------------------------*/
#define STATES_DESC(enum,value,desc) #enum ": " desc
#define STATES(m)\
m(ST_IDLE		,,"entry is not active")\
\
/* outgoing states */\
\
m(ST_DIAL		,,"dialing")\
m(ST_DIAL_FAIL	        ,,"waiting for dial-retry-time to be reached")\
\
/* incoming states */\
\
m(ST_INCOMING 		,,"incoming call, waiting for user-response")\
m(ST_WAIT_CONNECT	,,"incoming call, waiting for i4b-response")\
\
/* connected states */\
\
m(ST_CONNECTED		,,"connected to remote")\
/**/

MAKE_ENUM(STATES,
        N_STATES);

/*---------------------------------------------------------------------------*
 *	this struct describes the numbers to try to dial out and
 *	the numbers allowed to dial in
 *---------------------------------------------------------------------------*/
typedef struct {
	char number[TELNO_MAX];		/* number	*/
	char subaddr[SUBADDR_MAX];	/* subaddr	*/
	char flag;			/* usage flag	*/
#define RNF_IDLE	0
#define RNF_SUCC	1	/* last dial was ok */
} number_t;

/*---------------------------------------------------------------------------*
 *	this struct describes one complete configuration entry
 *---------------------------------------------------------------------------*/
typedef struct cfg_entry {

	/* ====== filled in at startup configuration, then static ===========*/

	char name[32];			/* id for this entry */

	int isdncontroller;		/* controller to use 0 ... n */
	int isdnchannel;		/* channel to use */

	int isdntxdelin;		/* tx delay, incoming connections */
	int isdntxdelout;		/* tx delay, outgoing connections */
	
	int usrdevicename;		/* userland device to use */
	int usrdeviceunit;		/* userland unit to use */

	int remote_numbers_count;	/* number of remote numbers	*/
	int remote_subaddr_count;	/* number of remote subaddr	*/
#define MAXRNUMBERS 8			/* max remote numbers		*/

	number_t remote_numbers[MAXRNUMBERS];	/* remote numbers to dial */
	

	int remote_numbers_handling;	/* how to handle the remote dialing */
#define RNH_NEXT	0	/* use next number after last successfull   */
#define RNH_LAST	1	/* use last successfull for next call	    */
#define RNH_FIRST	2	/* always use first number for next call    */

	number_t local_phone_dialout;	/* our number to tell remote*/
	number_t local_phone_incoming;	/* answer calls for this local number */

#define	MAX_INCOMING	8
	int incoming_numbers_count;	/* number of incoming allowed numbers */
	number_t remote_phone_incoming[MAX_INCOMING];	/* answer calls from this remote machine */

	int dialin_reaction;		/* what to do with incoming calls */
#define REACT_ACCEPT	 0
#define REACT_REJECT	 1
#define REACT_IGNORE	 2
#define REACT_ANSWER	 3
#define REACT_CALLBACK	 4
#define REACT_ALERT	 5

#define CEP_IS_CALLED_BACK(cep)				\
      (((cep)->dialin_reaction == REACT_ACCEPT) && 	\
       ((cep)->dialouttype == DIALOUT_CALLEDBACK))	\
/**/

	int bprotocol;			/* BPROT_XXX */

	int idle_time_in;		/* max idle time incoming calls */
	int idle_time_out;		/* max idle time outgoing calls */

	int shorthold_algorithm;	/* shorthold algorithm		*/

	int unitlength;			/* length of a charging unit	*/
#define UNITLENGTH_DEFAULT	60	/* last resort unit length	*/

	int earlyhangup;		/* time in seconds to hangup 	*/
					/* before the next expected	*/
					/* charging unit		*/
#define EARLYHANGUP_DEFAULT	5
					
	int ratetype;			/* type of rate	*/
#define NO_RATE		(NRATES+1)
#define INVALID_RATE	(-1)

 	int  unitlengthsrc;	/* where we get the unit length from	*/
#define ULSRC_NONE	0		/* nowhere specified		*/
#define ULSRC_CMDL	1		/* specified on commandline	*/
#define   ULSRC_CMDLMIN	5		/* minimum value from cmdl	*/
#define   ULSRC_CMDLMAX	3600		/* minimum value from cmdl	*/
#define ULSRC_CONF	2		/* get it from config file	*/
#define ULSRC_RATE	3		/* get it dynamic from ratesfile*/
#define ULSRC_DYN	4		/* dynamic calculated from AOCD */

	char *answerprog;		/* program to use for answering */
	pid_t answerpid;

	char *connectprog;	/* program run after negotiation finished */
	char *disconnectprog;	/* program run after shutdown is complete */

	int  callbackwait;		/* time to wait before calling back */
#define CALLBACKWAIT_MIN	1

	int  calledbackwait;		/* time to wait for remote callback */	
#define CALLEDBACKWAIT_MIN	2

	int  dialretries;		/* no. of dial tries		*/
#define DIALRETRIES_DEF		1
	
	int  recoverytime;		/* time between 2 dial tries	*/
#define RECOVERYTIME_MIN	1
	
	int  dialrandincr;		/* use random dial time incr	*/
	
	int  usedown;			/* set interface down yes/no	*/
	int  downtime;			/* time i/f is down		*/
#define DOWN_TIME_MIN 10	/* 10 seconds */
#define DOWN_TIME_MAX 3600	/* 1 hour */

	int  dialouttype;		/* type of outgoing connection	*/
#define DIALOUT_NORMAL     0		/* normal dialout behaviour	*/
#define DIALOUT_CALLEDBACK 1		/* remote is expected to callback */

	int	alert;			/* alert time in sec if nonzero	*/
#define MINALERT 5			/* 5 secs min			*/
#define MAXALERT (3*60)			/* 3 minutes max		*/

	int	inout;			/* in/out, in-only or out-only	*/
#define DIR_INOUT	0
#define DIR_INONLY	1
#define DIR_OUTONLY	2

	int	usesubaddr;		/* use subaddresses		*/

	int	budget_callbackperiod;	/* length of a budget period (s)*/
	int	budget_callbackncalls;	/* call budget for a period	*/
	char	*budget_callbacks_file;	/* filename to store callback stats */
	char	budget_callbacksfile_rotate;

	int	budget_calloutperiod;	/* length of a budget period (s)*/
	int	budget_calloutncalls;	/* call budget for a period	*/
	char	*budget_callouts_file;	/* filename to store callout stats */
	char	budget_calloutsfile_rotate;
	
	int	ppp_expect_auth;
	int	ppp_send_auth;
#define AUTH_UNDEF	0
#define AUTH_NONE	1
#define AUTH_PAP	2
#define AUTH_CHAP	3

	int	ppp_auth_flags;
#define AUTH_RECHALLENGE 0x01
#define AUTH_REQUIRED    0x02

#ifdef __NetBSD__
#define AUTHNAMELEN     64
#define AUTHKEYLEN      16
#endif

	char	ppp_expect_name[AUTHNAMELEN];	/* PPP PAP/CHAP login name */
	char	ppp_send_name[AUTHNAMELEN];

	char	ppp_expect_password[AUTHKEYLEN];
	char	ppp_send_password[AUTHKEYLEN];

	int	day;				/* days valid */
#define		SU	0x01
#define		MO	0x02
#define		TU	0x04
#define		WE	0x08
#define		TH	0x10
#define		FR	0x20
#define		SA	0x40
#define		HD	0x80	/* holiday */
        int	fromhr;				/* time valid */
        int	frommin;
        int	tohr;
        int	tomin;

	time_t	maxconnecttime;			/* maximum connection time */

	time_t	last_set_state_time;
	int	waittime;
	int	waitfunc;
	
/*===========================================================================*/	
/*============ filled in after start, then dynamic ==========================*/
/*===========================================================================*/	

	int cdid;			/* cdid for call		*/

	int isdncontrollerused;		/* the one we are using		*/
	int isdnchannelused;		/* the one we are using		*/
	
	int fs_position;		/* fullscreen position		*/

	int	state;			/* state of connection 		*/

	cause_t disc_cause;		/* cause from disconnect */
	int	disc_location;		/* who disconnected */
#define DISC_LOCATION_LOCAL	0
#define	DISC_LOCATION_REMOTE	1

	number_t real_phone_incoming;	/* real remote telno in case of wildcard */

	int last_remote_number;		/* index of last used dialout number*/

	number_t remote_phone_dialout;	/* used remote number to dial */
	
	char dir_incoming;	       	/* set: incoming or cleared: outgoing */
	
	int charge;			/* charge in units */
	int last_charge;		/* last charge in units */
	
	int inbytes;			/* # of bytes from remote */
	int iinbytes;			/* # of bytes from remote on the line */
	int inbps;			/* bytes/sec from remote */
	int outbytes;			/* # of bytes to remote */
	int ioutbytes;			/* # of bytes to remote on the line */	
	int outbps;			/* bytes/sec to remote */
	
	time_t aoc_last;		/* last AOCD timestamp		*/
	time_t aoc_now;			/* current AOCD timestamp	*/
	time_t aoc_diff;		/* current unit length		*/
	time_t aoc_lastdiff;		/* last charge unit length	*/
	int aoc_valid;			/* flag: time diff is valid	*/
#define AOC_INVALID	0		/* aoc_diff is NOT valid	*/
#define AOC_VALID	1		/* aoc_diff is valid		*/

	int	dial_count;		/* number of dialout tries	*/
#define RANDOM_MASK 0x03		/* bits used from randomtime	*/

	char display[DISPLAY_MAX];
	char user_user[USER_USER_MAX];

	time_t	budget_callbackperiod_time; /* end of current period	*/
	int	budget_callbackncalls_cnt;  /* amount of calls left	*/

	int	budget_callback_req;	/* requests			*/
	int	budget_callback_done;	/* call done			*/
	int	budget_callback_rej;	/* call refused			*/

	time_t	budget_calloutperiod_time; /* end of current period	*/
	int	budget_calloutncalls_cnt;  /* amount of calls left	*/

	int	budget_callout_req;	/* requests			*/
	int	budget_callout_done;	/* call done			*/
	int	budget_callout_rej;	/* call refused			*/
	
	int	budget_calltype;	/* type of call			*/
#define	BUDGET_TYPE_CBACK 1
#define	BUDGET_TYPE_COUT  2

	char 	keypad[KEYPAD_MAX];	/* keypad string		*/
} cfg_entry_t;

/*---------------------------------------------------------------------------*
 *	this struct describes state of controller
 *---------------------------------------------------------------------------*/
typedef struct isdn_ctrl_state {
	u_int8_t   l1_type;		/* type: active/passive 	*/
	u_int8_t   l1_desc[64];		/* controller description, zero terminated */
	char* firmware;			/* loadable firmware file name 	*/
    
        char ch_state[MAX_CHANNELS];    /* channel state		*/
#define  CHAN_IDLE	0		/* channel is free for usage	*/
#define  CHAN_RUN	1		/* channel is occupied		*/
	int tei;			/* tei or -1 if invalid		*/
	int l1stat;			/* layer 1 state		*/
	int l2stat;			/* layer 2 state		*/
} isdn_ctrl_state_t;

/*---------------------------------------------------------------------------*
 *	this struct describes a logging regular expression
 *---------------------------------------------------------------------------*/
struct rarr {
	int re_flg;		/* valid entry flag */
	char *re_expr;		/* plain text expression */
	regex_t re;		/* compiled expression */
	char *re_prog;		/* the program to be executed */
};

#ifdef I4B_EXTERNAL_MONITOR
/* for each rights entry we keep one of this structures around: */
struct monitor_rights {
	TAILQ_ENTRY(monitor_rights) list;	/* a list of this structures */
	char name[FILENAME_MAX];	/* net/host spec or filename */
	int rights;			/* bitmask of allowed acces rights */
	u_int32_t net;			/* net/host address (host byte order!) */
	u_int32_t mask;			/* bitmask 1 = network, 0 = host (host byte order!) */
	int local;			/* zero if remote access via tcp/ip */
};
#endif

#define CEP_FOREACH(cep,var)			\
__typeof(var) cep;				\
for(cep = var; cep < ((var)+nentries); cep++)	\
/**/

/*---------------------------------------------------------------------------*
 *	global variables, storage allocation
 *---------------------------------------------------------------------------*/
#ifdef MAIN

int isdnfd;					/* file handle, /dev/i4b */

char mailto[MAXPATHLEN] = "";			/* panic mail address */
char mailer[MAXPATHLEN] = "";			/* panic mail address */

const char *configfile = CONFIG_FILE_DEF;	/* configuration filename */
int config_error_flag = 0;			/* error counter */

#ifdef DEBUG
int do_debug = 0;				/* debug mode flag	*/
int debug_flags = 0;				/* debug options	*/
int debug_noscreen = 0;				/* not on fullscreen	*/
#endif

int do_bell = 0;				/* bell on connect/disconnect */

int do_fork = 1;				/* run as daemon/foreground */

int do_ttytype = 0;				/* got new terminal type */
const char *ttype = "";				/* termcap entry name string */

int do_rdev = 0;				/* redirect output	*/
const char *rdev = "";				/* new device string */

int do_print = 0;				/* config file printout */

int got_unitlen = 0;				/* flag, got length of a unit */
time_t unit_length;				/* length of a unit */

cfg_entry_t cfg_entry_tab[CFG_ENTRY_MAX];	/* configuration table */
isdn_ctrl_state_t isdn_ctrl_tab[MAX_CONTROLLERS];	/* controller states table */

int nentries = 0;				/* # of entries in config tab */

int uselogfile = 0;				/* flag, use a logfile */
char logfile[MAXPATHLEN] = LOG_FILE_DEF;	/* log filename */
FILE *logfp = NULL;                             /* log file pointer */
int logfacility = LOG_LOCAL0;   		/* the syslog facility used */
int nregex = 0;					/* number of reg expr */
struct rarr rarr[MAX_RE]; 			/* regexpr & progs table */

char ratesfile[MAXPATHLEN] = RATES_FILE_DEF;	/* rates filename */
int got_rate = 0;				/* flag, ratesfile found */
struct rates *rates[NRATES][NDAYS];		/* the rates structure */

int useacctfile = 0;				/* flag, write accounting */
char acctfile[MAXPATHLEN] = ACCT_FILE_DEF;	/* accounting  filename */
FILE *acctfp = NULL;				/* accounting file pointer */
int acct_all = 1;				/* account all connections */

int aliasing = 0;				/* enable alias processing */
char aliasfile[MAXPATHLEN] = ALIASFILE;		/* alias file location */

int do_fullscreen = 0;				/* fullscreen log	*/
int curses_ready = 0;				/* curses initialized */

#ifdef USE_CURSES
WINDOW *upper_w;		/* curses upper window pointer */
WINDOW *mid_w;			/* curses mid window pointer */
WINDOW *lower_w;		/* curses lower window pointer */
#endif

int rt_prio = RTPRIO_NOTUSED;			/* realtime priority */

/* monitor via network */

int do_monitor = 0;
int inhibit_monitor = 0;
#ifdef I4B_EXTERNAL_MONITOR
int monitorport = DEF_MONPORT;
#else
int monitorport = -1;
#endif
int accepted = 0;

int isdntime = 0;		/* flag, log time from exchange	*/
int extcallattr = 0;		/* flag, display extended caller attributes */

char tinainitprog[MAXPATHLEN] = TINA_FILE_DEF;

char rotatesuffix[MAXPATHLEN] = "";

time_t starttime = 0;

char holidayfile[MAXPATHLEN] = HOLIDAY_FILE_DEF; /* holiday filename */

int addprefix = 0;
char prefixnational[TELNO_MAX] = "";
char prefixinternational[TELNO_MAX] = "";

#else /* !MAIN */

extern struct monitor_rights *
monitor_next_rights(const struct monitor_rights *r);

extern FILE *logfp;

int isdnfd;

char mailto[MAXPATHLEN];
char mailer[MAXPATHLEN];

char *configfile;
int config_error_flag;

#ifdef DEBUG
int do_debug;
int debug_flags;
int debug_noscreen;
#endif

int do_bell;

int do_fork;

int do_ttytype;
char *ttype;

int do_rdev;
char *rdev;

int do_print;

int got_unitlen;
time_t unit_length;

cfg_entry_t cfg_entry_tab[CFG_ENTRY_MAX];	/* configuration table */
isdn_ctrl_state_t isdn_ctrl_tab[MAX_CONTROLLERS];	/* controller states table */

int nentries;

int uselogfile;
char logfile[MAXPATHLEN];
int logfacility;
int nregex;
struct rarr rarr[MAX_RE];

char ratesfile[MAXPATHLEN];
int got_rate;
struct rates *rates[NRATES][NDAYS];

int useacctfile;
char acctfile[MAXPATHLEN];
FILE *acctfp;
int acct_all;

int aliasing;
char aliasfile[MAXPATHLEN];

int do_fullscreen;
int curses_ready;

#ifdef USE_CURSES
WINDOW *upper_w;
WINDOW *mid_w;
WINDOW *lower_w;
#endif

int rt_prio;

int do_monitor;
int inhibit_monitor;
int monitorport;
int accepted;

int isdntime;
int extcallattr;

char tinainitprog[MAXPATHLEN];

char rotatesuffix[MAXPATHLEN];

time_t starttime;

char holidayfile[MAXPATHLEN];

int addprefix;
char prefixnational[TELNO_MAX];
char prefixinternational[TELNO_MAX];

#endif /* MAIN */

const char * driver_name ( int drivertype );
const char * driver_devicename ( int drivertype );
void cfg_setval ( int keyword );
void check_pid ( void );
void close_allactive ( void );
void configure ( const char *filename, int reread );
void daemonize ( void );
void display_acct ( cfg_entry_t *cep );
void display_bell ( void );
void display_ccharge ( cfg_entry_t *cep, int units );
void display_chans ( void );
void display_charge ( cfg_entry_t *cep );
void display_connect ( cfg_entry_t *cep );
void display_disconnect ( cfg_entry_t *cep );
void display_l12stat(int controller, int layer, int state);
void display_tei(int controller, int tei);
void display_updown ( cfg_entry_t *cep, int updown );
void do_menu ( void );
int exec_answer ( cfg_entry_t *cep );
int exec_connect_prog ( cfg_entry_t *cep, const char *prog, int link_down );
pid_t exec_prog ( const char *prog, const char ** arglist );
void finish_log ( void );
char * getlogdatetime ( void );
cfg_entry_t * get_cep_by_cc ( int ctrlr, int chan );
cfg_entry_t * get_cep_by_driver ( int drivertype, int driverunit );
cfg_entry_t * get_cep_by_cdid ( int cdid );
int get_current_rate ( cfg_entry_t *cep, int logit );
void handle_recovery ( void );
void handle_scrprs(int cdid, int scr, int prs, char *caller);
void init_log ( void );
void init_screen ( void );
void do_log ( int what, const char *fmt, ... );
int main ( int argc, char **argv );
const u_char * name_of_controller(u_int16_t controller);
int readrates ( char *filename );
void reopenfiles ( int dummy );
void rereadconfig ( int dummy );
void select_first_dialno ( cfg_entry_t *cep );
void select_next_dialno ( cfg_entry_t *cep );
void select_this_dialno ( cfg_entry_t *cep );
void sigchild_handler ( int sig );
void unitlen_chkupd( cfg_entry_t *cep );
void write_pid ( void );
void error_exit(int exitval, const char *fmt, ...);

/* monitor.c */

void monitor_init(void);
void monitor_exit(void);
void monitor_clear_rights(void);
void monitor_fixup_rights(void);
int monitor_start_rights(const char *clientspec);
void monitor_add_rights(int rights);

/* possible return codes from monitor_start_rights: */
#define	I4BMAR_OK	0	/* rights added successfully */
#define	I4BMAR_LENGTH	1	/* local socket name to long */
#define	I4BMAR_DUP	2	/* entry already exists */
#define	I4BMAR_CIDR	3	/* cidr netmask is invalid */
#define	I4BMAR_NOIP	4	/* host/net could not be resolved */

int monitor_create_local_socket(void);

#ifndef I4B_NOTCPIP_MONITOR
int monitor_create_remote_socket(int);
#endif

void monitor_prepselect(fd_set *selset, int *max_fd);
void monitor_handle_input(fd_set *selset);
void monitor_handle_connect(int sockfd, int is_local);
void monitor_evnt_charge(cfg_entry_t *cep, int units, int estimated);
void monitor_evnt_connect(cfg_entry_t *cep);
void monitor_evnt_disconnect(cfg_entry_t *cep);
void monitor_evnt_updown(cfg_entry_t *cep, int up);
void monitor_evnt_log(int prio, const char * what, const char * msg);
void monitor_evnt_l12stat(int controller, int layer, int state);
void monitor_evnt_tei(int controller, int tei);
void monitor_evnt_acct(cfg_entry_t *cep);

/* support.c */

void init_active_controller(void);
int set_channel_state(int controller, int channel, int state);
int ret_channel_state(int controller, int channel);
void ev_disconnect_from_user(cfg_entry_t *cep);
int isvalidtime(cfg_entry_t *cep);
int add_number_prefix(char *number, int type_of_number);
int number_matches(msg_connect_ind_t *mp, cfg_entry_t *cep);

/* alias.c */

void init_alias(char *filename);
void free_aliases(void);
char *get_alias(char *number);

/* exec.c */

void upd_callstat_file(char *filename, int rotateflag);

/* holiday.c */

void init_holidays(char *filename);
void free_holidays(void);
int isholiday(int d, int m, int y);

/**/

extern void	reset_scanner(FILE *infile);
extern int	yylex(void);
int yyparse(void);

extern int	lineno;
extern char	*yytext;
extern int	entrycount;
extern int      controllercount;
extern FILE     *yyin;

void isdnrdhdl(void);

#define log(what,fmt,args...)\
	do_log(what,"%s: " fmt, __FUNCTION__,## args)

#endif /* _ISDND_H_ */
