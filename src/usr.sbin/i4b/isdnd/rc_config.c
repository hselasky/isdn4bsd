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
 *	i4b daemon - config file processing
 *	-----------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdnd/rc_config.c,v 1.17 2002/08/12 07:52:39 hm Exp $
 *
 *      last edit-date: [Sun Aug 11 12:30:49 2002]
 *
 *---------------------------------------------------------------------------*/

#include "isdnd.h"
#include "rc_parse.h"

static void check_config(void);
static void print_config(void);
static void parse_valid(int entrycount, char *dt);

static int nregexpr = 0;
static int nregprog = 0;

/*---------------------------------------------------------------------------*
 *	fill all config entries with default values
 *---------------------------------------------------------------------------*/
static void
set_config_defaults(void)
{
	cfg_entry_t *cep = &cfg_entry_tab[0];	/* ptr to config entry */
	int i,j;

	/* zero all config entries */
	BZERO(&cfg_entry_tab);

	/* system section cleanup */
	
	nregprog = nregexpr = 0;

	rt_prio = RTPRIO_NOTUSED;

	mailer[0] = '\0';
	mailto[0] = '\0';	
	
	/* clean regular expression table */
	
	for(i=0; i < MAX_RE; i++)
	{
		if(rarr[i].re_expr)
			free(rarr[i].re_expr);
		rarr[i].re_expr = NULL;
		
		if(rarr[i].re_prog)
			free(rarr[i].re_prog);
		rarr[i].re_prog = NULL;

		rarr[i].re_flg = 0;
	}

	rotatesuffix[0] = '\0';
	
	/*
	 * controller table cleanup
	 */
	
	for(i=0; i < MAX_CONTROLLERS; i++)
	{
		msg_ctrl_info_req_t mcir;
		bzero(&mcir, sizeof(mcir));

		mcir.controller = i;

		if((ioctl(isdnfd, I4B_CTRL_INFO_REQ, &mcir)) < 0)
		{
			log(LL_ERR, "ioctl I4B_CTRL_INFO_REQ "
			    "failed for controller %d: %s", 
			    i, strerror(errno));
			exit(1);
		}

		if(mcir.l1_type >= N_L1_TYPES)
		{
			mcir.l1_type = 0;
		}

		if(mcir.l1_channels > MAX_CHANNELS)
		{
			log(LL_ERR, "too many channels");
			exit(1);
		}

		/* init controller tab */
		isdn_ctrl_tab[i].l1_type = mcir.l1_type;
		snprintf(&isdn_ctrl_tab[i].l1_desc[0],
			 sizeof(isdn_ctrl_tab[i].l1_desc), 
			 "%s", &mcir.l1_desc[0]);

		isdn_ctrl_tab[i].tei = -1;
		isdn_ctrl_tab[i].l1stat = LAYER_IDLE;
		isdn_ctrl_tab[i].l2stat = LAYER_IDLE;
		isdn_ctrl_tab[i].firmware = NULL;

		for (j = 0;
		     j < MAX_CHANNELS;
		     j++)
		{
		    isdn_ctrl_tab[i].ch_state[j] = CHAN_IDLE;
		}

		log(LL_DMN, "controller %d is %s",
		    i, name_of_controller(i));
	}

	/* entry section cleanup */
	
	for(i=0; i < CFG_ENTRY_MAX; i++, cep++)
	{

		/* ====== filled in at startup configuration, then static */

		sprintf(cep->name, "ENTRY%d", i);	

		cep->isdncontroller = 0;
		cep->isdnchannel = CHAN_ANY;

		cep->usrdevicename = INVALID;
		cep->usrdeviceunit = INVALID;
		
		cep->remote_numbers_handling = RNH_LAST;

		cep->dialin_reaction = REACT_IGNORE;

		cep->bprotocol = BPROT_NONE;

		cep->unitlength = UNITLENGTH_DEFAULT;

		cep->earlyhangup = EARLYHANGUP_DEFAULT;
		
		cep->ratetype = INVALID_RATE;
		
	 	cep->unitlengthsrc = ULSRC_NONE;

		cep->answerprog = ANSWERPROG_DEF;

		cep->callbackwait = CALLBACKWAIT_MIN;

		cep->calledbackwait = CALLEDBACKWAIT_MIN;		

		cep->dialretries = DIALRETRIES_DEF;

		cep->recoverytime = RECOVERYTIME_MIN;
	
		cep->dialouttype = DIALOUT_NORMAL;
		
		cep->inout = DIR_INOUT;
		
		cep->ppp_expect_auth = AUTH_UNDEF;
		
		cep->ppp_send_auth = AUTH_UNDEF;
		
		cep->ppp_auth_flags = AUTH_RECHALLENGE | AUTH_REQUIRED;
		
		/* ======== filled in after start, then dynamic */

		cep->cdid = CDID_UNUSED;

		cep->state = ST_IDLE;

		cep->aoc_valid = AOC_INVALID;

		cep->usesubaddr = 0;

		for(j = 0;
		    j < MAX_INCOMING;
		    j++)
		{
		  cep->remote_phone_incoming[j].number[0] = '*';
		  cep->remote_phone_incoming[j].subaddr[0] = '*';
		}
 	}
	return;
}

/*---------------------------------------------------------------------------*
 *	called from main to read and process config file
 *---------------------------------------------------------------------------*/
void
configure(char *filename, int reread)
{
	extern void reset_scanner(FILE *inputfile);

	close_allactive();

#if I4B_EXTERNAL_MONITOR
	monitor_clear_rights();
#endif
	controllercount = -1;
	entrycount = -1;
	nentries = 0;
	
	set_config_defaults();

	yyin = fopen(filename, "r");

	if(yyin == NULL)
	{
		log(LL_ERR, "cannot fopen file [%s]", filename);
		exit(1);
	}

	if(reread)
	{
		reset_scanner(yyin);
	}
	
	yyparse();
	
	monitor_fixup_rights();

	check_config();		/* validation and consistency check */

	fclose(yyin);

	if(config_error_flag)
	{
		log(LL_ERR, "there were %d error(s) in the configuration "
		    "file, terminating!", config_error_flag);

		if(reread)
		{
		  error_exit(1, "rereadconfig: there were %d error(s) "
			     "in the configuration file, terminating!",
			     config_error_flag);
		}
		else
		{
		  exit(1);
		}
	}

	if(do_print)
	{
		print_config();
		exit(0);
	}

	/* init aliases */
	if(aliasing)
	{
		/* reread alias database */
		free_aliases();
		init_alias(aliasfile);
	}

	/* init holidays */
	free_holidays();
	init_holidays(holidayfile);		

	return;
}

#define PPP_PAP		0xc023
#define PPP_CHAP	0xc223

static void
set_isppp_auth(cfg_entry_t *cep)
{
#ifdef __FreeBSD__
	struct ifreq ifr;
	struct spppreq spr;
#else
	struct spppauthcfg spcfg = { /* zero */ };
#endif
	int s;
	int doioctl = 0;

	if(cep->usrdevicename != DRVR_ISPPP)
	{
		return;
	}

	if((cep->ppp_expect_auth == AUTH_UNDEF) &&
	   (cep->ppp_send_auth == AUTH_UNDEF))
	{
		return;
	}

	if((cep->ppp_expect_auth == AUTH_NONE) ||
	   (cep->ppp_send_auth == AUTH_NONE))
	{
		doioctl = 1;
	}

	if(((cep->ppp_expect_auth == AUTH_CHAP) ||
	    (cep->ppp_expect_auth == AUTH_PAP)) &&
	   (cep->ppp_expect_name[0] != 0) &&
	   (cep->ppp_expect_password[0] != 0))
	{
		doioctl = 1;
	}

	if(((cep->ppp_send_auth == AUTH_CHAP) ||
	    (cep->ppp_send_auth == AUTH_PAP)) &&
	   (cep->ppp_send_name[0] != 0) && 
	   (cep->ppp_send_password[0] != 0))
	{
		doioctl = 1;
	}

	if(!doioctl)
		return;

#ifdef __FreeBSD__
	snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "isp%d", cep->usrdeviceunit);
#else
	snprintf(spcfg.ifname, sizeof(spcfg.ifname), "isp%d", cep->usrdeviceunit);
#endif

	/* use a random AF to create the socket */
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		log(LL_ERR, "ERROR opening control socket at line %d!", lineno);
		config_error_flag++;
		return;
	}

#ifdef __FreeBSD__
	spr.cmd = (int)(long)SPPPIOGDEFS;
	ifr.ifr_data = (caddr_t)&spr;

	if (ioctl(s, SIOCGIFGENERIC, &ifr) == -1) {
		log(LL_ERR, "ERROR fetching active PPP authentication "
		    "info for %s at line %d!", ifr.ifr_name, lineno);
		close(s);
		config_error_flag++;
		return;
	}

	if (cep->ppp_expect_auth != AUTH_UNDEF)
	{
		if(cep->ppp_expect_auth == AUTH_NONE)
		{
			spr.defs.hisauth.proto = 0;
		}
		else if(((cep->ppp_expect_auth == AUTH_CHAP) ||
			 (cep->ppp_expect_auth == AUTH_PAP)) &&
			(cep->ppp_expect_name[0] != 0) &&
			(cep->ppp_expect_password[0] != 0))
		{
			spr.defs.hisauth.proto = (cep->ppp_expect_auth == AUTH_PAP) ? PPP_PAP : PPP_CHAP;
			strlcpy(spr.defs.hisauth.name, cep->ppp_expect_name, AUTHNAMELEN);
			strlcpy(spr.defs.hisauth.secret, cep->ppp_expect_password, AUTHKEYLEN);
		}
	}

	if (cep->ppp_send_auth != AUTH_UNDEF)
	{
		if(cep->ppp_send_auth == AUTH_NONE)
		{
			spr.defs.myauth.proto = 0;
		}
		else if(((cep->ppp_send_auth == AUTH_CHAP) ||
			 (cep->ppp_send_auth == AUTH_PAP)) &&
			(cep->ppp_send_name[0] != 0) &&
			(cep->ppp_send_password[0] != 0))
		{
			spr.defs.myauth.proto = (cep->ppp_send_auth == AUTH_PAP) ? PPP_PAP : PPP_CHAP;
			strlcpy(spr.defs.myauth.name, cep->ppp_send_name, AUTHNAMELEN);
			strlcpy(spr.defs.myauth.secret, cep->ppp_send_password, AUTHKEYLEN);

			if(cep->ppp_auth_flags & AUTH_REQUIRED)
				spr.defs.hisauth.flags &= ~AUTHFLAG_NOCALLOUT;
			else
				spr.defs.hisauth.flags |= AUTHFLAG_NOCALLOUT;

			if(cep->ppp_auth_flags & AUTH_RECHALLENGE)
				spr.defs.hisauth.flags &= ~AUTHFLAG_NORECHALLENGE;
			else
				spr.defs.hisauth.flags |= AUTHFLAG_NORECHALLENGE;
		}
	}

	spr.cmd = (int)(long)SPPPIOSDEFS;

	if (ioctl(s, SIOCSIFGENERIC, &ifr) == -1) {
		log(LL_ERR, "ERROR setting new PPP authentication parameters "
		    "for %s at line %d!", ifr.ifr_name, lineno);
		config_error_flag++;
	}
#else
	if (ioctl(s, SPPPGETAUTHCFG, &spcfg) == -1) {
		log(LL_ERR, "ERROR fetching active PPP authentication "
		    "info for %s at line %d!", spcfg.ifname, lineno);
		close(s);
		config_error_flag++;
		return;
	}
	if (cep->ppp_expect_auth != AUTH_UNDEF)
	{
		if (cep->ppp_expect_auth == AUTH_NONE)
		{
			spcfg.hisauth = SPPP_AUTHPROTO_NONE;
		}
		else if (((cep->ppp_expect_auth == AUTH_CHAP) ||
			  (cep->ppp_expect_auth == AUTH_PAP)) &&
			 (cep->ppp_expect_name[0] != 0) &&
			 (cep->ppp_expect_password[0] != 0))
		{
			spcfg.hisauth = (cep->ppp_expect_auth == AUTH_PAP) ?
			  SPPP_AUTHPROTO_PAP : SPPP_AUTHPROTO_CHAP;
			spcfg.hisname = &(cep->ppp_expect_name[0]);
			spcfg.hisname_length = strlen(&(cep->ppp_expect_name[0]))+1;
			spcfg.hissecret = &(cep->ppp_expect_password[0]);
			spcfg.hissecret_length = strlen(&(cep->ppp_expect_password[0]))+1;
		}
	}
	if (cep->ppp_send_auth != AUTH_UNDEF)
	{
		if (cep->ppp_send_auth == AUTH_NONE)
		{
			spcfg.myauth = SPPP_AUTHPROTO_NONE;
		}
		else if (((cep->ppp_send_auth == AUTH_CHAP) ||
			  (cep->ppp_send_auth == AUTH_PAP)) &&
			 (cep->ppp_send_name[0] != 0) &&
			 (cep->ppp_send_password[0] != 0))
		{
			spcfg.myauth = (cep->ppp_send_auth == AUTH_PAP) ?
			  SPPP_AUTHPROTO_PAP : SPPP_AUTHPROTO_CHAP;
			spcfg.myname = &(cep->ppp_send_name[0]);
			spcfg.myname_length = strlen(&(cep->ppp_send_name[0]))+1;
			spcfg.mysecret = &(cep->ppp_send_password[0]);
			spcfg.mysecret_length = strlen(&(cep->ppp_send_password[0]))+1;

			if (cep->ppp_auth_flags & AUTH_REQUIRED)
				spcfg.hisauthflags &= ~SPPP_AUTHFLAG_NOCALLOUT;
			else
				spcfg.hisauthflags |= SPPP_AUTHFLAG_NOCALLOUT;

			if (cep->ppp_auth_flags & AUTH_RECHALLENGE)
				spcfg.hisauthflags &= ~SPPP_AUTHFLAG_NORECHALLENGE;
			else
				spcfg.hisauthflags |= SPPP_AUTHFLAG_NORECHALLENGE;
		}
	}

	if (ioctl(s, SPPPSETAUTHCFG, &spcfg) == -1) {
		log(LL_ERR, "ERROR setting new PPP authentication parameters "
		    "for %s at line %d!", spcfg.ifname, lineno);
		config_error_flag++;
	}
#endif
	close(s);
	return;
}

/*---------------------------------------------------------------------------*
 *	parse a date/time range
 *---------------------------------------------------------------------------*/
static void
parse_valid_dt(cfg_entry_t *cep, char *dt)
{
	/* a valid string consists of some days of week separated by
	 * commas, where 0=sunday, 1=monday .. 6=saturday and a special
	 * value of 7 which is a holiday from the holiday file.
	 * after the days comes an optional (!) time range in the form
	 * aa:bb-cc:dd, this format is fixed to be parsable by sscanf.
	 * Valid specifications looks like this:
	 * 1,2,3,4,5,09:00-18:00	Monday-Friday 9-18h
	 * 1,2,3,4,5,18:00-09:00	Monday-Friday 18-9h
	 * 6				Saturday (whole day)
	 * 0,7				Sunday and Holidays
	 */

	int day = 0;
	int fromhr = 0;
	int frommin = 0;
	int tohr = 0;
	int tomin = 0;
	int ret;
	
	for(;;)
	{
		if( ( ((*dt >= '0') && (*dt <= '9')) && (*(dt+1) == ':') ) ||
		    ( ((*dt >= '0') && (*dt <= '2')) && ((*(dt+1) >= '0') && (*(dt+1) <= '9')) && (*(dt+2) == ':') ) )
		{
			/* dt points to time spec */
			ret = sscanf(dt, "%d:%d-%d:%d", &fromhr, &frommin, &tohr, &tomin);
			if(ret !=4)
			{
				log(LL_ERR, "ERROR parsing config file: timespec [%s] error at line %d!", *dt, lineno);
				config_error_flag++;
				return;
			}

			if(fromhr < 0 || fromhr > 24 || tohr < 0 || tohr > 24 ||
			   frommin < 0 || frommin > 59 || tomin < 0 || tomin > 59)
			{
				log(LL_ERR, "ERROR parsing config file: invalid time [%s] at line %d!", *dt, lineno);
				config_error_flag++;
				return;
			}
			break;
		}
		else if ((*dt >= '0') && (*dt <= '7'))
		{
			/* dt points to day spec */
			day |= 1 << (*dt - '0');
			dt++;
			continue;
		}
		else if (*dt == ',')
		{
			/* dt points to delimiter */
			dt++;
			continue;
		}
		else if (*dt == '\0')
		{
			/* dt points to end of string */
			break;
		}
		else
		{
			/* dt points to illegal character */
			log(LL_ERR, "ERROR parsing config file: illegal character "
			    "[%c=0x%x] in date/time spec at line %d!", *dt, *dt, lineno);
			config_error_flag++;
			return;
		}
	}
	cep->day = day;
	cep->fromhr = fromhr;
	cep->frommin = frommin;
	cep->tohr = tohr;
	cep->tomin = tomin;
	return;
}

/*---------------------------------------------------------------------------*
 *	extract values from config and fill table
 *---------------------------------------------------------------------------*/
void
cfg_setval(int keyword)
{
	cfg_entry_t *cep;
	cep = &cfg_entry_tab[entrycount];

	switch(keyword)
	{
		case ACCTALL:
			acct_all = yylval.booln;
			DBGL(DL_RCCF, (log(LL_DBG, "system: acctall = %d", yylval.booln)));
			break;
			
		case ACCTFILE:
			strcpy(acctfile, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "system: acctfile = %s", yylval.str)));
			break;
	
		case ADDPREFIX:
			addprefix = yylval.booln;
			DBGL(DL_RCCF, (log(LL_DBG, "system: add-prefix = %d", yylval.booln)));
			break;

		case ALERT:
			if(yylval.num < MINALERT)
			{
				yylval.num = MINALERT;
				DBGL(DL_RCCF, (log(LL_DBG, "%s: alert < %d, min = %d", 
						   cep->name, MINALERT, yylval.num)));
			}
			else if(yylval.num > MAXALERT)
			{
				yylval.num = MAXALERT;
				DBGL(DL_RCCF, (log(LL_DBG, "%s: alert > %d, min = %d", 
						   cep->name, MAXALERT, yylval.num)));
			}
				
			DBGL(DL_RCCF, (log(LL_DBG, "%s: alert = %d", cep->name, yylval.num)));
			cep->alert = yylval.num;
			break;

		case ALIASING:
			DBGL(DL_RCCF, (log(LL_DBG, "system: aliasing = %d", yylval.booln)));
			aliasing = yylval.booln;
			break;

		case ALIASFNAME:
			strcpy(aliasfile, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "system: aliasfile = %s", yylval.str)));
			break;

		case ANSWERPROG:
			if((cep->answerprog = malloc(strlen(yylval.str)+1)) == NULL)
			{
				log(LL_ERR, "%s: answerstring, malloc failed!", cep->name);
				exit(1);
			}
			strcpy(cep->answerprog, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "%s: answerprog = %s", cep->name, yylval.str)));
			break;
			
		case B1PROTOCOL:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: b1protocol = %s", cep->name, yylval.str)));
			if(!(strcmp(yylval.str, "raw")))
				cep->bprotocol = BPROT_NONE;
			else if(!(strcmp(yylval.str, "hdlc")))
				cep->bprotocol = BPROT_RHDLC;
			else if(!(strcmp(yylval.str, "hdlc_dov")))
				cep->bprotocol = BPROT_RHDLC_DOV;
			else if (!(strcmp(yylval.str, "fax")))
                                cep->bprotocol = BPROT_NONE_3_1_KHZ;
#define m(enum,args...)								\
			else if(!(strcmp(yylval.str, #enum)))			\
				cep->bprotocol = enum;	\
			/**/
			I4B_B_PROTOCOLS(m)
#undef m
 			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"b1protocol\" at line %d!", lineno);
				config_error_flag++;
			}
			break;

		case BEEPCONNECT:
			do_bell = yylval.booln;
			DBGL(DL_RCCF, (log(LL_DBG, "system: beepconnect = %d", yylval.booln)));
			break;

		case BUDGETCALLBACKPERIOD:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: budget-callbackperiod = %d", cep->name, yylval.num)));
			cep->budget_callbackperiod = yylval.num;
			break;

		case BUDGETCALLBACKNCALLS:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: budget-callbackncalls = %d", cep->name, yylval.num)));
			cep->budget_callbackncalls = yylval.num;
			break;
			
		case BUDGETCALLOUTPERIOD:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: budget-calloutperiod = %d", cep->name, yylval.num)));
			cep->budget_calloutperiod = yylval.num;
			break;

		case BUDGETCALLOUTNCALLS:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: budget-calloutncalls = %d", cep->name, yylval.num)));
			cep->budget_calloutncalls = yylval.num;
			break;

		case BUDGETCALLBACKSFILEROTATE:
			cep->budget_callbacksfile_rotate = yylval.booln;
			DBGL(DL_RCCF, (log(LL_DBG, "%s: budget-callbacksfile-rotate = %d", 
					   cep->name, yylval.booln)));
			break;
			
		case BUDGETCALLBACKSFILE:
			{
				FILE *fp;
				int s, l;
				int n;
				DBGL(DL_RCCF, (log(LL_DBG, "%s: budget-callbacksfile = %s", 
						   cep->name, yylval.str)));
				fp = fopen(yylval.str, "r");
				if(fp != NULL)
				{
					if((fscanf(fp, "%d %d %d", (int *)&s, (int *)&l, &n)) != 3)
					{
						DBGL(DL_RCCF, (log(LL_DBG, "%s: initializing "
								   "budget-callbacksfile %s", 
								   cep->name, yylval.str)));
						fclose(fp);
						fp = fopen(yylval.str, "w");
						if(fp != NULL)
						{
							fprintf(fp, "%d %d %d", (int)time(NULL), 
								(int)time(NULL), 0);
							fclose(fp);
						}
					}
				}
				else
				{
					DBGL(DL_RCCF, (log(LL_DBG, "%s: creating budget-callbacksfile %s",
							   cep->name, yylval.str)));
					fp = fopen(yylval.str, "w");
					if(fp != NULL)
					{
						fprintf(fp, "%d %d %d", (int)time(NULL), (int)time(NULL), 0);
						fclose(fp);
					}
				}

				fp = fopen(yylval.str, "r");
				if(fp != NULL)
				{
					if((fscanf(fp, "%d %d %d", (int *)&s, (int *)&l, &n)) == 3)
					{
						if((cep->budget_callbacks_file = malloc(strlen(yylval.str)+1)) == NULL)
						{
							log(LL_ERR, "%s: budget-callbacksfile, malloc failed!", cep->name);
							exit(1);
						}
						strcpy(cep->budget_callbacks_file, yylval.str);
						DBGL(DL_RCCF, (log(LL_DBG, "%s: using callbacksfile %s", cep->name, yylval.str)));
					}
					fclose(fp);
				}
			}
			break;

		case BUDGETCALLOUTSFILEROTATE:
			cep->budget_calloutsfile_rotate = yylval.booln;
			DBGL(DL_RCCF, (log(LL_DBG, "%s: budget-calloutsfile-rotate = %d", cep->name, yylval.booln)));
			break;

		case BUDGETCALLOUTSFILE:
			{
				FILE *fp;
				int s, l;
				int n;
				DBGL(DL_RCCF, (log(LL_DBG, "%s: budget-calloutsfile = %s", 
						   cep->name, yylval.str)));
				fp = fopen(yylval.str, "r");
				if(fp != NULL)
				{
					if((fscanf(fp, "%d %d %d", (int *)&s, (int *)&l, &n)) != 3)
					{
						DBGL(DL_RCCF, (log(LL_DBG, "%s: initializing "
								   "budget-calloutsfile %s", 
								   cep->name, yylval.str)));
						fclose(fp);
						fp = fopen(yylval.str, "w");
						if(fp != NULL)
						{
							fprintf(fp, "%d %d %d", (int)time(NULL), 
								(int)time(NULL), 0);
							fclose(fp);
						}
					}
				}
				else
				{
					DBGL(DL_RCCF, (log(LL_DBG, "%s: creating budget-calloutsfile %s",
							   cep->name, yylval.str)));
					fp = fopen(yylval.str, "w");
					if(fp != NULL)
					{
						fprintf(fp, "%d %d %d", (int)time(NULL), 
							(int)time(NULL), 0);
						fclose(fp);
					}
				}

				fp = fopen(yylval.str, "r");
				if(fp != NULL)
				{
					if((fscanf(fp, "%d %d %d", (int *)&s, (int *)&l, &n)) == 3)
					{
						if((cep->budget_callouts_file = malloc(strlen(yylval.str)+1)) == NULL)
						{
							log(LL_ERR, "%s: budget-calloutsfile, "
							    "malloc failed!", cep->name);
							exit(1);
						}
						strcpy(cep->budget_callouts_file, yylval.str);
						DBGL(DL_RCCF, (log(LL_DBG, "%s: using calloutsfile %s",
								   cep->name, yylval.str)));
					}
					fclose(fp);
				}
			}
			break;
		
		case CALLBACKWAIT:
			if(yylval.num < CALLBACKWAIT_MIN)
			{
				yylval.num = CALLBACKWAIT_MIN;
				DBGL(DL_RCCF, (log(LL_DBG, "%s: callbackwait < %d, min = %d", 
						   cep->name, CALLBACKWAIT_MIN, yylval.num)));
			}

			DBGL(DL_RCCF, (log(LL_DBG, "%s: callbackwait = %d", cep->name, yylval.num)));
			cep->callbackwait = yylval.num;
			break;
			
		case CALLEDBACKWAIT:
			if(yylval.num < CALLEDBACKWAIT_MIN)
			{
				yylval.num = CALLEDBACKWAIT_MIN;
				DBGL(DL_RCCF, (log(LL_DBG, "%s: calledbackwait < %d, min = %d", 
						   cep->name, CALLEDBACKWAIT_MIN, yylval.num)));
			}

			DBGL(DL_RCCF, (log(LL_DBG, "%s: calledbackwait = %d", cep->name, yylval.num)));
			cep->calledbackwait = yylval.num;
			break;

		case CLONE:
		{
		    /*
		     *  clone = <entryname>
		     *      Loads the entry from the named, existing one.
		     *      Fields such as name and usrdeviceunit should
		     *      always be specified after clone as they must be
		     *      unique.
		     */
		    int i;
		    __typeof(cep->name) name_temp;

		    for (i = 0; i < entrycount; i++)
		    {
			if (!strcmp(cfg_entry_tab[i].name, yylval.str))
			{
			    break;
			}
		    }
		    if (i == entrycount)
		    {
			log(LL_ERR, "%s: clone, unknown entry %s!", cep->name, yylval.str);
			exit(1);
		    }
		    
		    DBGL(DL_RCCF, (log(LL_DBG, "%s: clone = %s", cep->name, yylval.str)));

		    /* store cep->name */
		    bcopy(&cep->name, &name_temp, sizeof(name_temp));

		    /* copy data */
		    bcopy(&cfg_entry_tab[i], cep, sizeof(*cep));

		    /* restore cep->name */
		    bcopy(&name_temp, &cep->name, sizeof(cep->name));

		    /* NOTE: all malloc()'d fields must be dup()'d,
		     * hence we can't have multiple references to
		     * the same storage
		     */
		    if (cep->answerprog)
			cep->answerprog = strdup(cep->answerprog);
		    if (cep->budget_callbacks_file)
			cep->budget_callbacks_file = strdup(cep->budget_callbacks_file);
		    if (cep->budget_callouts_file)
			cep->budget_callouts_file = strdup(cep->budget_callouts_file);
		    if (cep->connectprog)
			cep->connectprog = strdup(cep->connectprog);
		    if (cep->disconnectprog)
			cep->disconnectprog = strdup(cep->disconnectprog);
		    break;
		}
		case CONNECTPROG:
			if((cep->connectprog = malloc(strlen(yylval.str)+1)) == NULL)
			{
				log(LL_ERR, "%s: connectprog, malloc failed!", cep->name);
				exit(1);
			}
			strcpy(cep->connectprog, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "%s: connectprog = %s", cep->name, yylval.str)));
			break;
			
		case DIALOUTTYPE:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: dialouttype = %s", cep->name, yylval.str)));
			if(!(strcmp(yylval.str, "normal")))
				cep->dialouttype = DIALOUT_NORMAL;
			else if(!(strcmp(yylval.str, "calledback")))
				cep->dialouttype = DIALOUT_CALLEDBACK;
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"dialout-type\" at line %d!", lineno);
				config_error_flag++;
			}
			break;

		case DIALRETRIES:
		case DOWNTRIES:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: dialretries = %d", cep->name, yylval.num)));
			cep->dialretries = yylval.num;
			break;

		case DIALRANDINCR:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: dialrandincr = %d", cep->name, yylval.booln)));
			cep->dialrandincr = yylval.booln;
			break;

		case DIRECTION:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: direction = %s", cep->name, yylval.str)));

			if(!(strcmp(yylval.str, "inout")))
				cep->inout = DIR_INOUT;
			else if(!(strcmp(yylval.str, "in")))
				cep->inout = DIR_INONLY;
			else if(!(strcmp(yylval.str, "out")))
				cep->inout = DIR_OUTONLY;
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"direction\" at line %d!", lineno);
				config_error_flag++;
			}
			break;

		case DISCONNECTPROG:
			if((cep->disconnectprog = malloc(strlen(yylval.str)+1)) == NULL)
			{
				log(LL_ERR, "%s: disconnectprog, malloc failed!", cep->name);
				exit(1);
			}
			strcpy(cep->disconnectprog, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "%s: disconnectprog = %s", cep->name, yylval.str)));
			break;

		case DOWNTIME:
			if(yylval.num > DOWN_TIME_MAX)
				yylval.num = DOWN_TIME_MAX;
			else if(yylval.num < DOWN_TIME_MIN)
				yylval.num = DOWN_TIME_MIN;
		
			DBGL(DL_RCCF, (log(LL_DBG, "%s: downtime = %d", cep->name, yylval.num)));
			cep->downtime = yylval.num;
			break;

		case EARLYHANGUP:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: earlyhangup = %d", cep->name, yylval.num)));
			cep->earlyhangup = yylval.num;
			break;

		case EXTCALLATTR:
			DBGL(DL_RCCF, (log(LL_DBG, "system: extcallattr = %d", yylval.booln)));
			extcallattr = yylval.booln;
			break;

		case FIRMWARE:
			DBGL(DL_RCCF, (log(LL_DBG, "controller %d: firmware = %s", 
					   controllercount, yylval.str)));
			isdn_ctrl_tab[controllercount].firmware = strdup(yylval.str);
			break;

		case HOLIDAYFILE:
			strcpy(holidayfile, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "system: holidayfile = %s", yylval.str)));
			break;

		case IDLE_ALG_OUT:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: idle-algorithm-outgoing = %s", 
					   cep->name, yylval.str)));

			if(!(strcmp(yylval.str, "fix-unit-size")))
			{
				cep->shorthold_algorithm = SHA_FIXU;
			}
			else if(!(strcmp(yylval.str, "var-unit-size")))
			{
				cep->shorthold_algorithm = SHA_VARU;
			}
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"idle-algorithm-outgoing\" at line %d!", lineno);
				config_error_flag++;
			}
			break;

		case IDLETIME_IN:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: idle_time_in = %d", cep->name, yylval.num)));
			cep->idle_time_in = yylval.num;
			break;
			
		case IDLETIME_OUT:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: idle_time_out = %d", cep->name, yylval.num)));
			cep->idle_time_out = yylval.num;
			break;

		case ISDNCONTROLLER:
			cep->isdncontroller = yylval.num;
			DBGL(DL_RCCF, (log(LL_DBG, "%s: isdncontroller = %d", cep->name, yylval.num)));
			break;

		case ISDNCHANNEL:
		        if (yylval.num == 0 || yylval.num == -1)
			{
					yylval.num = CHAN_ANY;
					DBGL(DL_RCCF, (log(LL_DBG, "%s: isdnchannel = any", cep->name)));
			}
			else
			{
					yylval.num += -1 +CHAN_B1;
					DBGL(DL_RCCF, (log(LL_DBG, "%s: isdnchannel = B%d", 
							   cep->name, yylval.num)));

					if((yylval.num < 0) ||
					   (yylval.num >= MAX_CHANNELS))
					{
					  log(LL_DBG, "%s: isdnchannel value out of range", cep->name);
					  config_error_flag++;
					  yylval.num = CHAN_ANY;
					}
			}
			cep->isdnchannel = yylval.num;
			break;

		case ISDNTIME:
			DBGL(DL_RCCF, (log(LL_DBG, "system: isdntime = %d", yylval.booln)));
			isdntime = yylval.booln;
			break;

		case ISDNTXDELIN:
			cep->isdntxdelin = yylval.num;
			DBGL(DL_RCCF, (log(LL_DBG, "%s: isdntxdel-incoming = %d", cep->name, yylval.num)));
			break;

		case ISDNTXDELOUT:
			cep->isdntxdelout = yylval.num;
			DBGL(DL_RCCF, (log(LL_DBG, "%s: isdntxdel-outgoing = %d", cep->name, yylval.num)));
			break;

		case LOCAL_PHONE_DIALOUT:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: local_phone_dialout = %s", cep->name, yylval.str)));
			strcpy(cep->local_phone_dialout.number, yylval.str);
			break;

		case LOCAL_SUBADDR_DIALOUT:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: local_subaddr_dialout = %s", cep->name, yylval.str)));
			strcpy(cep->local_phone_dialout.subaddr, yylval.str);
			break;

		case LOCAL_PHONE_INCOMING:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: local_phone_incoming = %s", cep->name, yylval.str)));
			strcpy(cep->local_phone_incoming.number, yylval.str);
			break;

		case LOCAL_SUBADDR_INCOMING:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: local_subaddr_incoming = %s", cep->name, yylval.str)));
			strcpy(cep->local_phone_incoming.subaddr, yylval.str);
			break;

		case MAILER:
			strcpy(mailer, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "system: mailer = %s", yylval.str)));
			break;

		case MAILTO:
			strcpy(mailto, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "system: mailto = %s", yylval.str)));
			break;

		case MAXCONNECTTIME:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: maxconnecttime = %d", cep->name, yylval.num)));
			cep->maxconnecttime = yylval.num;
			break;

		case MONITORPORT:
			monitorport = yylval.num;
			DBGL(DL_RCCF, (log(LL_DBG, "system: monitorport = %d", yylval.num)));
			break;

		case MONITORSW:
			if (yylval.booln && inhibit_monitor)
			{
				do_monitor = 0;
				DBGL(DL_RCCF, (log(LL_DBG, "system: monitor-enable overriden "
						   "by command line flag")));
			}
			else
			{
				do_monitor = yylval.booln;
				DBGL(DL_RCCF, (log(LL_DBG, "system: monitor-enable = %d", yylval.booln)));
			}
			break;

		case NAME:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: name = %s", cep->name, yylval.str)));
			strcpy(cep->name, yylval.str);
			break;

		case PPP_AUTH_RECHALLENGE:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ppp-auth-rechallenge = %d", 
					   cep->name, yylval.booln)));
			if(yylval.booln)
				cep->ppp_auth_flags |= AUTH_RECHALLENGE;
			else
				cep->ppp_auth_flags &= ~AUTH_RECHALLENGE;
			set_isppp_auth(cep);
			break;

		case PPP_AUTH_PARANOID:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ppp-auth-paranoid = %d", cep->name, yylval.booln)));
			if(yylval.booln)
				cep->ppp_auth_flags |= AUTH_REQUIRED;
			else
				cep->ppp_auth_flags &= ~AUTH_REQUIRED;
			set_isppp_auth(cep);
			break;

		case PPP_EXPECT_AUTH:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ppp-expect-auth = %s", cep->name, yylval.str)));
			if(!(strcmp(yylval.str, "none")))
				cep->ppp_expect_auth = AUTH_NONE;
			else if(!(strcmp(yylval.str, "pap")))
				cep->ppp_expect_auth = AUTH_PAP;
			else if(!(strcmp(yylval.str, "chap")))
				cep->ppp_expect_auth = AUTH_CHAP;
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"ppp-expect-auth\" at line %d!", lineno);
				config_error_flag++;
				break;
			}
			set_isppp_auth(cep);
			break;

		case PPP_EXPECT_NAME:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ppp-expect-name = %s", cep->name, yylval.str)));
			strlcpy(cep->ppp_expect_name, yylval.str, sizeof(cep->ppp_expect_name));
			set_isppp_auth(cep);
			break;

		case PPP_EXPECT_PASSWORD:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ppp-expect-password = %s", cep->name, yylval.str)));
			strlcpy(cep->ppp_expect_password, yylval.str, sizeof(cep->ppp_expect_password));
			set_isppp_auth(cep);
			break;

		case PPP_SEND_AUTH:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ppp-send-auth = %s", cep->name, yylval.str)));
			if(!(strcmp(yylval.str, "none")))
				cep->ppp_send_auth = AUTH_NONE;
			else if(!(strcmp(yylval.str, "pap")))
				cep->ppp_send_auth = AUTH_PAP;
			else if(!(strcmp(yylval.str, "chap")))
				cep->ppp_send_auth = AUTH_CHAP;
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"ppp-send-auth\" at line %d!", lineno);
				config_error_flag++;
				break;
			}
			set_isppp_auth(cep);
			break;

		case PPP_SEND_NAME:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ppp-send-name = %s", cep->name, yylval.str)));
			strlcpy(cep->ppp_send_name, yylval.str, sizeof(cep->ppp_send_name));
			set_isppp_auth(cep);
			break;

		case PPP_SEND_PASSWORD:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ppp-send-password = %s", cep->name, yylval.str)));
			strlcpy(cep->ppp_send_password, yylval.str, sizeof(cep->ppp_send_password));
			set_isppp_auth(cep);
			break;

		case PREFIXINTERNATIONAL:
			strlcpy(prefixinternational, yylval.str, sizeof(prefixinternational));
			DBGL(DL_RCCF, (log(LL_DBG, "system: prefix-international = %s", 
					   prefixinternational)));
			break;

		case PREFIXNATIONAL:
			strlcpy(prefixnational, yylval.str, sizeof(prefixnational));
			DBGL(DL_RCCF, (log(LL_DBG, "system: prefix-national = %s", 
					   prefixnational)));
			break;

		case REACTION:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: dialin_reaction = %s", cep->name, yylval.str)));
			if(!(strcmp(yylval.str, "accept")))
				cep->dialin_reaction = REACT_ACCEPT;
			else if(!(strcmp(yylval.str, "reject")))
				cep->dialin_reaction = REACT_REJECT;
			else if(!(strcmp(yylval.str, "ignore")))
				cep->dialin_reaction = REACT_IGNORE;
			else if(!(strcmp(yylval.str, "answer")))
				cep->dialin_reaction = REACT_ANSWER;
			else if(!(strcmp(yylval.str, "callback")))
				cep->dialin_reaction = REACT_CALLBACK;
			else if(!(strcmp(yylval.str, "alert")))
				cep->dialin_reaction = REACT_ALERT;
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"dialin_reaction\" at line %d!", lineno);
				config_error_flag++;
			}
			break;

		case REMOTE_PHONE_DIALOUT:
			if(cep->remote_numbers_count >= MAXRNUMBERS)
			{
				log(LL_ERR, "ERROR parsing config file: too many remote "
				    "numbers at line %d!", lineno);
				config_error_flag++;
				break;
			}				
			
			DBGL(DL_RCCF, (log(LL_DBG, "%s: remote_phone_dialout #%d = %s",
				cep->name, cep->remote_numbers_count, yylval.str)));

			strcpy(cep->remote_numbers[cep->remote_numbers_count].number, yylval.str);
			cep->remote_numbers[cep->remote_numbers_count].flag = 0;

			cep->remote_numbers_count++;
			
			break;

		case REMOTE_SUBADDR_DIALOUT:
			if(cep->remote_subaddr_count >= MAXRNUMBERS)
			{
				log(LL_ERR, "ERROR parsing config file: too many remote "
				    "subaddresses at line %d!", lineno);
				config_error_flag++;
				break;
			}				
			
			DBGL(DL_RCCF, (log(LL_DBG, "%s: remote_subaddr_dialout #%d = %s",
				cep->name, cep->remote_numbers_count, yylval.str)));

			strcpy(cep->remote_numbers[cep->remote_numbers_count].subaddr, yylval.str);

			break;

		case REMOTE_NUMBERS_HANDLING:			
			DBGL(DL_RCCF, (log(LL_DBG, "%s: remdial_handling = %s", cep->name, yylval.str)));
			if(!(strcmp(yylval.str, "next")))
				cep->remote_numbers_handling = RNH_NEXT;
			else if(!(strcmp(yylval.str, "last")))
				cep->remote_numbers_handling = RNH_LAST;
			else if(!(strcmp(yylval.str, "first")))
				cep->remote_numbers_handling = RNH_FIRST;
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter for "
				    "keyword \"remdial_handling\" at line %d!", lineno);
				config_error_flag++;
			}
			break;

		case REMOTE_PHONE_INCOMING:
			{
				int n;
				n = cep->incoming_numbers_count;
				if (n >= MAX_INCOMING)
				{
					log(LL_ERR, "ERROR parsing config file: too many "
					    "\"remote_phone_incoming\" entries at line %d!", lineno);
					config_error_flag++;
					break;
				}
				DBGL(DL_RCCF, (log(LL_DBG, "%s: remote_phone_incoming "
						   "#%d = %s", cep->name, n, yylval.str)));
				strcpy(cep->remote_phone_incoming[n].number, yylval.str);
				cep->incoming_numbers_count++;
			}
			break;

		case REMOTE_SUBADDR_INCOMING:
			{
				int n;
				n = cep->incoming_numbers_count;
				if (n >= MAX_INCOMING)
				{
					log(LL_ERR, "ERROR parsing config file: too many "
					    "\"remote_subaddr_incoming\" entries at line %d!", lineno);
					config_error_flag++;
					break;
				}
				DBGL(DL_RCCF, (log(LL_DBG, "%s: remote_subaddr_incoming "
						   "#%d = %s", cep->name, n, yylval.str)));
				strcpy(cep->remote_phone_incoming[n].subaddr, yylval.str);
			}
			break;

		case RATESFILE:
			strcpy(ratesfile, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "system: ratesfile = %s", yylval.str)));
			break;

		case RATETYPE:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: ratetype = %d", cep->name, yylval.num)));
			cep->ratetype = yylval.num;
			break;
		
		case RECOVERYTIME:
			if(yylval.num < RECOVERYTIME_MIN)
			{
				yylval.num = RECOVERYTIME_MIN;
				DBGL(DL_RCCF, (log(LL_DBG, "%s: recoverytime < %d, min = %d", 
						   cep->name, RECOVERYTIME_MIN, yylval.num)));
			}

			DBGL(DL_RCCF, (log(LL_DBG, "%s: recoverytime = %d", cep->name, yylval.num)));
			cep->recoverytime = yylval.num;
			break;
		
		case REGEXPR:
		{
			int i;
			if(nregexpr >= MAX_RE)
			{
				log(LL_ERR, "system: regexpr #%d >= MAX_RE", nregexpr);
				config_error_flag++;
				break;
			}

			if((i = regcomp(&(rarr[nregexpr].re), yylval.str, REG_EXTENDED|REG_NOSUB)) != 0)
		        {
                		char buf[256];
                		regerror(i, &(rarr[nregexpr].re), buf, sizeof(buf));
				log(LL_ERR, "system: regcomp error for %s: [%s]", yylval.str, buf);
				config_error_flag++;
                		break;
			}
			else
			{
				if((rarr[nregexpr].re_expr = malloc(strlen(yylval.str)+1)) == NULL)
				{
					log(LL_ERR, "system: regexpr malloc error error for %s", yylval.str);
					config_error_flag++;
					break;
				}
				strcpy(rarr[nregexpr].re_expr, yylval.str);

				DBGL(DL_RCCF, (log(LL_DBG, "system: regexpr %s stored into "
						   "slot %d", yylval.str, nregexpr)));
				
				if(rarr[nregexpr].re_prog != NULL)
					rarr[nregexpr].re_flg = 1;
				
				nregexpr++;
				
			}
			break;
		}
		case REGPROG:
			if(nregprog >= MAX_RE)
			{
				log(LL_ERR, "system: regprog #%d >= MAX_RE", nregprog);
				config_error_flag++;
				break;
			}
			if((rarr[nregprog].re_prog = malloc(strlen(yylval.str)+1)) == NULL)
			{
				log(LL_ERR, "system: regprog malloc error error for %s", yylval.str);
				config_error_flag++;
				break;
			}
			strcpy(rarr[nregprog].re_prog, yylval.str);

			DBGL(DL_RCCF, (log(LL_DBG, "system: regprog %s stored into slot %d", 
					   yylval.str, nregprog)));
			
			if(rarr[nregprog].re_expr != NULL)
				rarr[nregprog].re_flg = 1;

			nregprog++;
			break;

		case ROTATESUFFIX:
			strcpy(rotatesuffix, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "system: rotatesuffix = %s", yylval.str)));
			break;

		case RTPRIO:
#ifdef USE_RTPRIO
			rt_prio = yylval.num;
			if(rt_prio < RTP_PRIO_MIN || rt_prio > RTP_PRIO_MAX)
			{
				config_error_flag++;
				log(LL_ERR, "system: error, rtprio (%d) out of range!", yylval.num);
			}
			else
			{
				DBGL(DL_RCCF, (log(LL_DBG, "system: rtprio = %d", yylval.num)));
			}
#else
			rt_prio = RTPRIO_NOTUSED;
#endif
			break;

		case TINAINITPROG:
			strcpy(tinainitprog, yylval.str);
			DBGL(DL_RCCF, (log(LL_DBG, "system: tinainitprog = %s", yylval.str)));
			break;

		case UNITLENGTH:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: unitlength = %d", cep->name, yylval.num)));
			cep->unitlength = yylval.num;
			break;

		case UNITLENGTHSRC:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: unitlengthsrc = %s", cep->name, yylval.str)));
			if(!(strcmp(yylval.str, "none")))
				cep->unitlengthsrc = ULSRC_NONE;
			else if(!(strcmp(yylval.str, "cmdl")))
				cep->unitlengthsrc = ULSRC_CMDL;
			else if(!(strcmp(yylval.str, "conf")))
				cep->unitlengthsrc = ULSRC_CONF;
			else if(!(strcmp(yylval.str, "rate")))
				cep->unitlengthsrc = ULSRC_RATE;
			else if(!(strcmp(yylval.str, "aocd")))
				cep->unitlengthsrc = ULSRC_DYN;
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"unitlengthsrc\" at line %d!", lineno);
				config_error_flag++;
			}
			break;

		case USRDEVICENAME:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: usrdevicename = %s", cep->name, yylval.str)));
			if(!strcmp(yylval.str, "rbch"))
				cep->usrdevicename = DRVR_RBCH;
			else if(!strcmp(yylval.str, "tel"))
				cep->usrdevicename = DRVR_TEL;
			else if(!strcmp(yylval.str, "ipr"))
				cep->usrdevicename = DRVR_IPR;
			else if(!strcmp(yylval.str, "isp"))
				cep->usrdevicename = DRVR_ISPPP;
			else if(!strcmp(yylval.str, "ibc"))
				cep->usrdevicename = DRVR_IBC;
			else if(!strcmp(yylval.str, "ing"))
				cep->usrdevicename = DRVR_ING;
#define m(enum,args...)						\
			else if(!strcmp(yylval.str, #enum))	\
				cep->usrdevicename = enum;	\
			/**/
			I4B_B_DRIVERS(m)
#undef m
			else
			{
				log(LL_ERR, "ERROR parsing config file: unknown parameter "
				    "for keyword \"usrdevicename\" at line %d!", lineno);
				config_error_flag++;
			}
			break;

		case USRDEVICEUNIT:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: usrdeviceunit = %d", cep->name, yylval.num)));
			cep->usrdeviceunit = yylval.num;
			break;

		case USEACCTFILE:
			useacctfile = yylval.booln;
			DBGL(DL_RCCF, (log(LL_DBG, "system: useacctfile = %d", yylval.booln)));
			break;

		case USESUBADDR:
			cep->usesubaddr = yylval.booln;
			DBGL(DL_RCCF, (log(LL_DBG, "%s: usesubaddr = %d", cep->name, yylval.booln)));
			break;

		case USEDOWN:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: usedown = %d", cep->name, yylval.booln)));
			cep->usedown = yylval.booln;
			break;

		case VALID:
			DBGL(DL_RCCF, (log(LL_DBG, "%s: valid = %s", cep->name, yylval.str)));
			parse_valid_dt(cep, yylval.str);
			break;

		default:
			log(LL_ERR, "ERROR parsing config file: unknown keyword at line %d!", lineno);
			config_error_flag++;
			break;			
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	configuration validation and consistency check
 *---------------------------------------------------------------------------*/
static void
check_config(void)
{
	cfg_entry_t *cep = &cfg_entry_tab[0];	/* ptr to config entry */
	int i,j;
	int error = 0;

	/* regular expression table */
	
	for(i=0; i < MAX_RE; i++)
	{
		if((rarr[i].re_expr != NULL) && (rarr[i].re_prog == NULL))
		{
			log(LL_ERR, "regular expression %d "
			    "without program!", i);
			error++;
		}
		if((rarr[i].re_prog != NULL) && (rarr[i].re_expr == NULL))
		{
			log(LL_ERR, "regular expression program %d "
			    "without expression!", i);
			error++;
		}
	}

	/* entry sections */
	
	for(i=0; i <= entrycount; i++, cep++)
	{
		for(j=0; j < i; j++)
		{
		  if((cep->usrdevicename == cfg_entry_tab[j].usrdevicename) &&
		     (cep->usrdeviceunit == cfg_entry_tab[j].usrdeviceunit))
		  {
			log(LL_ERR, "ENTRY%d (%s) and ENTRY%d (%s) have same "
			    "usrdevicename/usrdeviceunit combination",
			    i, cfg_entry_tab[i].name, 
			    j, cfg_entry_tab[j].name);
			error++;
		  }
		}

		/* isdn controller number */

		if((cep->isdncontroller < 0) || 
		   (cep->isdncontroller >= MAX_CONTROLLERS))
		{
			log(LL_ERR, "WARNING, %s: isdncontroller out "
			    "of range in entry %d!", cep->name, i);
		}

		/* numbers used for dialout */
		
		if((cep->inout != DIR_INONLY) &&
		   (cep->dialin_reaction != REACT_ANSWER) &&
		   (cep->dialin_reaction != REACT_ALERT))
		{
			if(cep->remote_numbers_count == 0)
			{
				log(LL_ERR, "%s: remote-phone-dialout not set "
				    "in entry %d!", cep->name, i);
				error++;
			}
			if(strlen(cep->local_phone_dialout.number) == 0)
			{
				log(LL_ERR, "%s: local-phone-dialout not set "
				    "in entry %d!", cep->name, i);
				error++;
			}
		}

		/* numbers used for incoming calls */
		
		if(cep->inout != DIR_OUTONLY)
		{
			if(strlen(cep->local_phone_incoming.number) == 0)
			{
				log(LL_ERR, "%s: local-phone-incoming not "
				    "set in entry %d!", cep->name, i);
				error++;
			}
			if(cep->incoming_numbers_count == 0)
			{
				log(LL_ERR, "%s: remote-phone-incoming not "
				    "set in entry %d!", cep->name, i);
				error++;
			}
		}

		if(((cep->dialin_reaction == REACT_ALERT) ||
		    (cep->dialin_reaction == REACT_ANSWER)) && 
		   (cep->bprotocol != BPROT_NONE) &&
		   (cep->bprotocol != BPROT_NONE_3_1_KHZ) &&
		   (cep->bprotocol != BPROT_NONE_VOD) &&
		   (cep->usrdevicename == DRVR_TEL))
		{
			log(LL_ERR, "%s: bprotocol not raw for telephony "
			    "in entry %d!", cep->name, i);
			error++;
		}

		if((cep->ppp_send_auth == AUTH_PAP) || 
		   (cep->ppp_send_auth == AUTH_CHAP))
		{
			if(cep->ppp_send_name[0] == 0)
			{
				log(LL_ERR, "%s: no remote authentification "
				    "name in entry %d!", cep->name, i);
				error++;
			}
			if(cep->ppp_send_password[0] == 0)
			{
				log(LL_ERR, "%s: no remote authentification "
				    "password in entry %d!", cep->name, i);
				error++;
			}
		}
		if((cep->ppp_expect_auth == AUTH_PAP) || 
		   (cep->ppp_expect_auth == AUTH_CHAP))
		{
			if(cep->ppp_expect_name[0] == 0)
			{
				log(LL_ERR, "%s: no local authentification "
				    "name in entry %d!", cep->name, i);
				error++;
			}
			if(cep->ppp_expect_password[0] == 0)
			{
				log(LL_ERR, "%s: no local authentification "
				    "secret in entry %d!", cep->name, i);
				error++;
			}
		}
	}
	if(error)
	{
		log(LL_ERR, "%d error(s) in configuration file, exit!", error);
		exit(1);
	}
}

static const char *
bprotocol_string(int bprotocol)
{
  static const char * const
    MAKE_TABLE(I4B_B_PROTOCOLS,ISDND_CONFIG_DESC,[]);

  if((bprotocol < 0) ||
     (bprotocol >= N_I4B_B_PROTOCOLS))
  {
    bprotocol = 0;
  }

  return(I4B_B_PROTOCOLS_ISDND_CONFIG_DESC[bprotocol]);
}

/*---------------------------------------------------------------------------*
 *	print the configuration
 *---------------------------------------------------------------------------*/
static void
print_config(void)
{
#define PFILE stdout

#ifdef I4B_EXTERNAL_MONITOR
	extern struct monitor_rights * monitor_next_rights(const struct monitor_rights *r);
	struct monitor_rights *m_rights;
#endif
	cfg_entry_t *cep = &cfg_entry_tab[0];	/* ptr to config entry */
	int i, j;
	time_t clock;
	char mytime[64];

	time(&clock);
	strcpy(mytime, ctime(&clock));
	mytime[strlen(mytime)-1] = '\0';

	fprintf(PFILE, "#---------------------------------------------------------------------------\n");
	fprintf(PFILE, "# system section (generated %s)\n", mytime);
	fprintf(PFILE, "#---------------------------------------------------------------------------\n");
	fprintf(PFILE, "system\n");
	fprintf(PFILE, "useacctfile     = %s\n", useacctfile ? "on\t\t\t\t# update accounting information file" : "off\t\t\t\t# don't update accounting information file");
	fprintf(PFILE, "acctall         = %s\n", acct_all ? "on\t\t\t\t# put all events into accounting file" : "off\t\t\t\t# put only charged events into accounting file");
	fprintf(PFILE, "acctfile        = %s\t\t# accounting information file\n", acctfile);
	fprintf(PFILE, "ratesfile       = %s\t\t# charging rates database file\n", ratesfile);

#ifdef USE_RTPRIO
	if(rt_prio == RTPRIO_NOTUSED)
		fprintf(PFILE, "# rtprio is unused\n");
	else
		fprintf(PFILE, "rtprio          = %d\t\t\t\t# isdnd runs at realtime priority\n", rt_prio);
#endif

	/* regular expression table */
	
	for(i=0; i < MAX_RE; i++)
	{
		if(rarr[i].re_expr != NULL)
		{
			fprintf(PFILE, "regexpr         = \"%s\"\t\t# scan logfile for this expression\n", rarr[i].re_expr);
		}
		if(rarr[i].re_prog != NULL)
		{
			fprintf(PFILE, "regprog         = %s\t\t# program to run when expression is matched\n", rarr[i].re_prog);
		}
	}

#ifdef I4B_EXTERNAL_MONITOR

	fprintf(PFILE, "monitor-allowed = %s\n", do_monitor ? "on\t\t\t\t# remote isdnd monitoring allowed" : "off\t\t\t\t# remote isdnd monitoring disabled");
	fprintf(PFILE, "monitor-port    = %d\t\t\t\t# TCP/IP port number used for remote monitoring\n", monitorport);

	m_rights = monitor_next_rights(NULL);
	if(m_rights != NULL)
	{
		char *s = "error\n";
		char b[512];

		for ( ; m_rights != NULL; m_rights = monitor_next_rights(m_rights))
		{
			if(m_rights->local)
			{
				fprintf(PFILE, "monitor         = \"%s\"\t\t# local "
					"socket name for monitoring\n", m_rights->name);
			}
			else
			{
				struct in_addr ia;
				ia.s_addr = ntohl(m_rights->net);

				switch(m_rights->mask)
				{
					case 0xffffffff:
						s = "32";
						break;
					case 0xfffffffe:
						s = "31";
						break;
					case 0xfffffffc:
						s = "30";
						break;
					case 0xfffffff8:
						s = "29";
						break;
					case 0xfffffff0:
						s = "28";
						break;
					case 0xffffffe0:
						s = "27";
						break;
					case 0xffffffc0:
						s = "26";
						break;
					case 0xffffff80:
						s = "25";
						break;
					case 0xffffff00:
						s = "24";
						break;
					case 0xfffffe00:
						s = "23";
						break;
					case 0xfffffc00:
						s = "22";
						break;
					case 0xfffff800:
						s = "21";
						break;
					case 0xfffff000:
						s = "20";
						break;
					case 0xffffe000:
						s = "19";
						break;
					case 0xffffc000:
						s = "18";
						break;
					case 0xffff8000:
						s = "17";
						break;
					case 0xffff0000:
						s = "16";
						break;
					case 0xfffe0000:
						s = "15";
						break;
					case 0xfffc0000:
						s = "14";
						break;
					case 0xfff80000:
						s = "13";
						break;
					case 0xfff00000:
						s = "12";
						break;
					case 0xffe00000:
						s = "11";
						break;
					case 0xffc00000:
						s = "10";
						break;
					case 0xff800000:
						s = "9";
						break;
					case 0xff000000:
						s = "8";
						break;
					case 0xfe000000:
						s = "7";
						break;
					case 0xfc000000:
						s = "6";
						break;
					case 0xf8000000:
						s = "5";
						break;
					case 0xf0000000:
						s = "4";
						break;
					case 0xe0000000:
						s = "3";
						break;
					case 0xc0000000:
						s = "2";
						break;
					case 0x80000000:
						s = "1";
						break;
					case 0x00000000:
						s = "0";
						break;
				}
				fprintf(PFILE, "monitor         = \"%s/%s\"\t\t# host (net/mask) allowed to connect for monitoring\n", inet_ntoa(ia), s);
			}
			b[0] = '\0';
			
			if((m_rights->rights) & I4B_CA_COMMAND_FULL)
				strcat(b, "fullcmd,");
			if((m_rights->rights) & I4B_CA_COMMAND_RESTRICTED)
				strcat(b, "restrictedcmd,");
			if((m_rights->rights) & I4B_CA_EVNT_CHANSTATE)
				strcat(b, "channelstate,");
			if((m_rights->rights) & I4B_CA_EVNT_CALLIN)
				strcat(b, "callin,");
			if((m_rights->rights) & I4B_CA_EVNT_CALLOUT)
				strcat(b, "callout,");
			if((m_rights->rights) & I4B_CA_EVNT_I4B)
				strcat(b, "logevents,");

			if(b[strlen(b)-1] == ',')
				b[strlen(b)-1] = '\0';
				
			fprintf(PFILE, "monitor-access  = %s\t\t# monitor access rights\n", b);
		}
	}
	
#endif
	/* entry sections */
	
	for(i=0; i <= entrycount; i++, cep++)
	{
		fprintf(PFILE, "\n");
		fprintf(PFILE, "#---------------------------------------------------------------------------\n");
		fprintf(PFILE, "# entry section %d\n", i);
		fprintf(PFILE, "#---------------------------------------------------------------------------\n");
		fprintf(PFILE, "entry\n");

		fprintf(PFILE, "name                  = %s\t\t# name for this entry section\n", cep->name);

		fprintf(PFILE, "isdncontroller        = %d\t\t# ISDN card number used for this entry\n", cep->isdncontroller);
		fprintf(PFILE, "isdnchannel           = ");
		switch(cep->isdnchannel)
		{
				case CHAN_ANY:
					fprintf(PFILE, "-1\t\t# any ISDN B-channel may be used\n");
					break;
    		                default:
					fprintf(PFILE, "%d\t\t# only ISDN B-channel %d may be used\n", 
						cep->isdnchannel-CHAN_B1+1, 
						cep->isdnchannel-CHAN_B1+1);
					break;
		}

		fprintf(PFILE, "usrdevicename         = %s\t\t# name of userland ISDN B-channel device\n", driver_name(cep->usrdevicename));
		fprintf(PFILE, "usrdeviceunit         = %d\t\t# unit number of userland ISDN B-channel device\n", cep->usrdeviceunit);

		fprintf(PFILE, "b1protocol            = %s\n", bprotocol_string(cep->bprotocol));

		if(!(cep->usrdevicename == DRVR_TEL))
		{
			fprintf(PFILE, "direction             = ");
			switch(cep->inout)
			{
				case DIR_INONLY:
					fprintf(PFILE, "in\t\t# only incoming connections allowed\n");
					break;
				case DIR_OUTONLY:
					fprintf(PFILE, "out\t\t# only outgoing connections allowed\n");
					break;
				case DIR_INOUT:
					fprintf(PFILE, "inout\t\t# incoming and outgoing connections allowed\n");
					break;
			}
		}
		
		if(!((cep->usrdevicename == DRVR_TEL) || (cep->inout == DIR_INONLY)))
		{
			if(cep->remote_numbers_count > 1)
			{
				for(j=0; j<cep->remote_numbers_count; j++)
					fprintf(PFILE, "remote-phone-dialout  = %s\t\t# telephone number %d for dialing out to remote\n", cep->remote_numbers[j].number, j+1);

				fprintf(PFILE, "remdial-handling      = ");
		
				switch(cep->remote_numbers_handling)
				{
					case RNH_NEXT:
						fprintf(PFILE, "next\t\t# use next number after last successfull for new dial\n");
						break;
					case RNH_LAST:
						fprintf(PFILE, "last\t\t# use last successfull number for new dial\n");
						break;
					case RNH_FIRST:
						fprintf(PFILE, "first\t\t# always start with first number for new dial\n");
						break;
				}
			}
			else
			{
				fprintf(PFILE, "remote-phone-dialout  = %s\t\t# telephone number for dialing out to remote\n", cep->remote_numbers[0].number);
			}

			fprintf(PFILE, "local-phone-dialout   = %s\t\t# show this number to remote when dialling out\n", cep->local_phone_dialout.number);
			fprintf(PFILE, "dialout-type          = %s\n", cep->dialouttype ? "calledback\t\t# i am called back by remote" : "normal\t\t# i am not called back by remote");
		}

		if(!(cep->inout == DIR_OUTONLY))
		{
			int n;
			
			fprintf(PFILE, "local-phone-incoming  = %s\t\t# incoming calls must match this (mine) telephone number\n", cep->local_phone_incoming.number);
			for (n = 0; n < cep->incoming_numbers_count; n++)
				fprintf(PFILE, "remote-phone-incoming = %s\t\t# this is a valid remote number to call me\n",
					cep->remote_phone_incoming[n].number);

			fprintf(PFILE, "dialin-reaction       = ");
			switch(cep->dialin_reaction)
			{
				case REACT_ACCEPT:
					fprintf(PFILE, "accept\t\t# i accept a call from remote and connect\n");
					break;
				case REACT_REJECT:
					fprintf(PFILE, "reject\t\t# i reject the call from remote\n");
					break;
				case REACT_IGNORE:
					fprintf(PFILE, "ignore\t\t# i ignore the call from remote\n");
					break;
				case REACT_ANSWER:
					fprintf(PFILE, "answer\t\t# i will start telephone answering when remote calls in\n");
					break;
				case REACT_CALLBACK:
					fprintf(PFILE, "callback\t\t# when remote calls in, i will hangup and call back\n");
					break;
				case REACT_ALERT:
					fprintf(PFILE, "alert\t\t# i am using isdnphone\n");
					break;
			}
		}

		if(cep->usrdevicename == DRVR_ISPPP)
		{
			char *s;
			switch(cep->ppp_expect_auth)
			{
				case AUTH_NONE:
					s = "none";
					break;
				case AUTH_PAP:
					s = "pap";
					break;
				case AUTH_CHAP:
					s = "chap";
					break;
				default:
					s = NULL;
					break;
			}
			if(s != NULL)
			{
				fprintf(PFILE, "ppp-expect-auth       = %s\t\t# the auth protocol we expect to receive on dial-in (none,pap,chap)\n", s);
				if(cep->ppp_expect_auth != AUTH_NONE)
				{
					fprintf(PFILE, "ppp-expect-name       = %s\t\t# the user name allowed in\n", cep->ppp_expect_name);
					fprintf(PFILE, "ppp-expect-password   = %s\t\t# the key expected from the other side\n", cep->ppp_expect_password);
					fprintf(PFILE, "ppp-auth-paranoid     = %s\t\t# do we require remote to authenticate even if we dial out\n", cep->ppp_auth_flags & AUTH_REQUIRED ? "yes" : "no");
				}
			}
			switch(cep->ppp_send_auth)
			{
				case AUTH_NONE:
					s = "none";
					break;
				case AUTH_PAP:
					s = "pap";
					break;
				case AUTH_CHAP:
					s = "chap";
					break;
				default:
					s = NULL;
					break;
			}
			if(s != NULL)
			{
				fprintf(PFILE, "ppp-send-auth         = %s\t\t# the auth protocol we use when dialing out (none,pap,chap)\n", s);
				if(cep->ppp_send_auth != AUTH_NONE)
				{
					fprintf(PFILE, "ppp-send-name         = %s\t\t# our PPP account used for dial-out\n", cep->ppp_send_name);
					fprintf(PFILE, "ppp-send-password     = %s\t\t# the key sent to the other side\n", cep->ppp_send_password);
				}
			}
			if(cep->ppp_send_auth == AUTH_CHAP ||
			   cep->ppp_expect_auth == AUTH_CHAP) {
				fprintf(PFILE, "ppp-auth-rechallenge   = %s\t\t# rechallenge CHAP connections once in a while\n", cep->ppp_auth_flags & AUTH_RECHALLENGE ? "yes" : "no");
			}
		}

		if(!((cep->inout == DIR_INONLY) || (cep->usrdevicename == DRVR_TEL)))
		{
			char *s;
			fprintf(PFILE, "idletime-outgoing     = %d\t\t# outgoing call idle timeout\n", cep->idle_time_out);

			switch( cep->shorthold_algorithm )
			{
				case SHA_FIXU:
					s = "fix-unit-size";
					break;
				case SHA_VARU:
					s = "var-unit-size";
					break;
				default:
					s = "error!!!";
					break;
			}

			fprintf(PFILE, "idle-algorithm-outgoing     = %s\t\t# outgoing call idle algorithm\n", s);
		}

		if(!(cep->inout == DIR_OUTONLY))
			fprintf(PFILE, "idletime-incoming     = %d\t\t# incoming call idle timeout\n", cep->idle_time_in);

		if(!(cep->usrdevicename == DRVR_TEL))
		{		
	 		fprintf(PFILE, "unitlengthsrc         = ");
			switch(cep->unitlengthsrc)
			{
				case ULSRC_NONE:
					fprintf(PFILE, "none\t\t# no unit length specified, using default\n");
					break;
				case ULSRC_CMDL:
					fprintf(PFILE, "cmdl\t\t# using unit length specified on commandline\n");
					break;
				case ULSRC_CONF:
					fprintf(PFILE, "conf\t\t# using unitlength specified by unitlength-keyword\n");
					fprintf(PFILE, "unitlength            = %d\t\t# fixed unitlength\n", cep->unitlength);
					break;
				case ULSRC_RATE:
					fprintf(PFILE, "rate\t\t# using unitlength specified in rate database\n");
					fprintf(PFILE, "ratetype              = %d\t\t# type of rate from rate database\n", cep->ratetype);
					break;
				case ULSRC_DYN:
					fprintf(PFILE, "aocd\t\t# using dynamically calculated unitlength based on AOCD subscription\n");
					fprintf(PFILE, "ratetype              = %d\t\t# type of rate from rate database\n", cep->ratetype);
					break;
			}

			fprintf(PFILE, "earlyhangup           = %d\t\t# early hangup safety time\n", cep->earlyhangup);

		}
		
		if(cep->answerprog && cep->answerprog[0])
		{
			fprintf(PFILE, "answerprog            = %s\t\t# "
				"program used to answer incoming "
				"telephone calls\n", cep->answerprog);
		}

		if(cep->usrdevicename == DRVR_TEL)
		{
			fprintf(PFILE, "alert                 = %d\t\t# number of seconds to wait before accepting a call\n", cep->alert);
		}

		if(!(cep->usrdevicename == DRVR_TEL))
		{		
			if(cep->dialin_reaction == REACT_CALLBACK)
				fprintf(PFILE, "callbackwait          = %d\t\t# i am waiting this time before calling back remote\n", cep->callbackwait);
	
			if(cep->dialouttype == DIALOUT_CALLEDBACK)
				fprintf(PFILE, "calledbackwait        = %d\t\t# i am waiting this time for a call back from remote\n", cep->calledbackwait);
	
			if(!(cep->inout == DIR_INONLY))
			{
				fprintf(PFILE, "dialretries           = %d\t\t# number of dialing retries\n", cep->dialretries);
				fprintf(PFILE, "recoverytime          = %d\t\t# time to wait between dialling retries\n", cep->recoverytime);
				fprintf(PFILE, "dialrandincr          = %s\t\t# use random dialing time addon\n", cep->dialrandincr ? "on" : "off");

				fprintf(PFILE, "usedown               = %s\n", cep->usedown ? "on\t\t# ISDN device switched off on excessive dial failures" : "off\t\t# no device switchoff on excessive dial failures");
				if(cep->usedown)
				{
					fprintf(PFILE, "downtime              = %d\t\t# time device is switched off\n", cep->downtime);
				}
			}
		}		
	}
	fprintf(PFILE, "\n");	
}

/* EOF */
