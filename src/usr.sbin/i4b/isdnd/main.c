/*-
 * Copyright (c) 1997, 2001 Hellmuth Michaelis. All rights reserved.
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
 *	i4b daemon - main program entry
 *	-------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#include <locale.h>

#define MAIN
#include "isdnd.h"
#undef MAIN

#ifdef I4B_EXTERNAL_MONITOR
static int localmonitor = -1;		/* local monitor socket */
#ifndef I4B_NOTCPIP_MONITOR
static int remotemonitor = -1;		/* tcp/ip monitor socket */
#endif
#endif

#ifdef USE_CURSES
static void kbdrdhdl(void);
#endif

static void usage(void);
static void mloop(void);

/*---------------------------------------------------------------------------*
 *	usage display and exit
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
	fprintf(stderr, "\n");
	fprintf(stderr, "isdnd - i4b ISDN manager daemon, version %02d.%02d.%d, %s %s\n", I4B_VERSION, I4B_REL, I4B_STEP, __DATE__, __TIME__);
#ifdef DEBUG
	fprintf(stderr, "  usage: isdnd [-c file] [-d level] [-F] [-f [-r dev] [-t termtype]]\n");
#else
	fprintf(stderr, "  usage: isdnd [-c file] [-F] [-f [-r dev] [-t termtype]]\n");
#endif	
	fprintf(stderr, "               [-l] [-L file] [-m] [-s facility] [-u time]\n");
	fprintf(stderr, "    -c <filename> configuration file name (def: %s)\n", CONFIG_FILE_DEF);
#ifdef DEBUG
	fprintf(stderr, "    -d <level>    set debug flag bits:\n");
	fprintf(stderr, "                  general = 0x%04x, rates  = 0x%04x, timing   = 0x%04x\n", DL_MSG,   DL_RATES, DL_TIME);
	fprintf(stderr, "                  state   = 0x%04x, retry  = 0x%04x, dial     = 0x%04x\n", DL_STATE, DL_RCVRY, DL_DIAL);
	fprintf(stderr, "                  process = 0x%04x, kernio = 0x%04x, ctrlstat = 0x%04x\n", DL_PROC,  DL_DRVR,  DL_CNST);
	fprintf(stderr, "                  rc-file = 0x%04x, budget = 0x%04x, valid    = 0x%04x\n", DL_RCCF,  DL_BDGT, DL_VALID);
	fprintf(stderr, "    -dn           no debug output on fullscreen display\n");
#endif
	fprintf(stderr, "    -f            fullscreen status display\n");
	fprintf(stderr, "    -F            do not become a daemon process\n");
	fprintf(stderr, "    -l            use a logfile instead of syslog\n");
	fprintf(stderr, "    -L <file>     use file instead of %s for logging\n", LOG_FILE_DEF);
	fprintf(stderr, "    -P            pretty print real config to stdout and exit\n");
	fprintf(stderr, "    -r <device>   redirect output to other device    (for -f)\n");
	fprintf(stderr, "    -s <facility> use facility instead of %d for syslog logging\n", LOG_LOCAL0 >> 3);
	fprintf(stderr, "    -t <termtype> terminal type of redirected screen (for -f)\n");
	fprintf(stderr, "    -u <time>     length of a charging unit in seconds\n");
#ifdef I4B_EXTERNAL_MONITOR
	fprintf(stderr, "    -m            inhibit network/local monitoring (protocol %02d.%02d)\n", MPROT_VERSION, MPROT_REL);
#endif	
	fprintf(stderr, "\n");
	exit(1);
}

/*---------------------------------------------------------------------------*
 *	program exit
 *---------------------------------------------------------------------------*/
static void
at_exit(void)
{
	close_allactive();

	unlink(PIDFILE);

	log(LL_DMN, "daemon terminating");
	
#ifdef USE_CURSES
	if(do_fullscreen)
		endwin();
#endif

#ifdef I4B_EXTERNAL_MONITOR
	monitor_exit();
#endif
	return;
}

/*---------------------------------------------------------------------------*
 *	program entry
 *---------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
	int i;
	msg_vr_req_t mvr;
	
	atexit(&at_exit);

	setlocale (LC_ALL, "");
	
	while ((i = getopt(argc, argv, "mc:d:fFlL:Pr:s:t:u:")) != -1)
	{
		switch (i)
		{
#ifdef I4B_EXTERNAL_MONITOR
			case 'm':
				inhibit_monitor = 1;
				break;
#endif
				
			case 'c':
				configfile = optarg;
				break;

#ifdef DEBUG				
			case 'd':
				if(*optarg == 'n')
					debug_noscreen = 1;
				else if((sscanf(optarg, "%i", &debug_flags)) == 1)
					do_debug = 1;
				else
					usage();				
				break;
#endif

			case 'f':
				do_fullscreen = 1;
				do_fork = 0;			
#ifndef USE_CURSES
				fprintf(stderr, "Sorry, no fullscreen mode available - daemon compiled without USE_CURSES\n");
				exit(1);
#endif
				break;

			case 'F':
				do_fork = 0;
				break;

			case 'l':
				uselogfile = 1;
				break;

			case 'L':
				strlcpy(logfile, optarg, sizeof(logfile));
				break;

			case 'P':
				do_print = 1;
				break;

			case 'r':
				rdev = optarg;
				do_rdev = 1;
				break;

			case 's':
				if(isdigit(*optarg))
				{
					int facility;
					logfacility = strtoul(optarg, NULL, 10);
					facility = logfacility << 3;

					if((facility < LOG_KERN) ||
					   (facility > LOG_FTP && facility < LOG_LOCAL0) ||
					   (facility > LOG_LOCAL7))
					{
						fprintf(stderr, "Error, option -s has invalid logging facility %d", logfacility);
						usage();
					}
					logfacility = facility;
				}
				else
				{
					fprintf(stderr, "Error: option -s requires a numeric argument!\n");
					usage();
				}
				break;

			case 't':
				ttype = optarg;
				do_ttytype = 1;
				break;

			case 'u':
				if(isdigit(*optarg))
				{
					unit_length = strtoul(optarg, NULL, 10);
					if(unit_length < ULSRC_CMDLMIN)
						unit_length = ULSRC_CMDLMIN;
					else if(unit_length > ULSRC_CMDLMAX)
						unit_length = ULSRC_CMDLMAX;
					got_unitlen = 1;
				}
				else
				{
					fprintf(stderr, "Error: option -u requires a numeric argument!\n");
					usage();
				}
				break;

			case '?':
			default:
				usage();
				break;
		}
	}
#ifdef DEBUG
	if(!do_debug)
		debug_noscreen = 0;
#endif

	if(!do_print)
	{
		umask(UMASK);	/* set our umask ... */	
	
		init_log();	/* initialize the logging subsystem */
	}
	
	check_pid();	/* check if we are already running */

	if(!do_print)
	{
		if(do_fork || (do_fullscreen && do_rdev)) /* daemon mode ? */
			daemonize();
	
		write_pid();	/* write our pid to file */
			
		/* set signal handler(s) */
	
		signal(SIGCHLD, sigchild_handler); /* process handling */
		signal(SIGHUP,  rereadconfig);	   /* reread configuration */
		signal(SIGUSR1, reopenfiles);	   /* reopen acct/log files */
		signal(SIGPIPE, SIG_IGN);	   /* handled manually */
		signal(SIGINT,  exit);		   /* clean up on SIGINT */
		signal(SIGTERM, exit);		   /* clean up on SIGTERM */
		signal(SIGQUIT, exit);		   /* clean up on SIGQUIT */
	}

	/* open isdn device */
	
	if((isdnfd = open(I4BDEVICE, O_RDWR)) < 0)
	{
		log(LL_ERR, "cannot open %s: %s", I4BDEVICE, strerror(errno));
		exit(1);
	}

	/* check kernel and userland have same version/release numbers */
	
	if((ioctl(isdnfd, I4B_VR_REQ, &mvr)) < 0)
	{
		log(LL_ERR, "ioctl I4B_VR_REQ failed: %s", strerror(errno));
		exit(1);
	}

	if(mvr.version != I4B_VERSION)
	{
		log(LL_ERR, "version mismatch, kernel %d, daemon %d",
		    mvr.version, I4B_VERSION);
		exit(1);
	}

	if(mvr.release != I4B_REL)
	{
		log(LL_ERR, "release mismatch, kernel %d, daemon %d",
		    mvr.release, I4B_REL);
		exit(1);
	}

	if(mvr.step != I4B_STEP)
	{
		log(LL_ERR, "step mismatch, kernel %d, daemon %d",
		    mvr.step, I4B_STEP);
		exit(1);
	}

	if(mvr.max_controllers != MAX_CONTROLLERS)
	{
		log(LL_ERR, "max_controller mismatch, kernel %d, daemon %d",
		    mvr.max_controllers, MAX_CONTROLLERS);
		exit(1);
	}

	if(MAX_CONTROLLERS == 0)
	{
		log(LL_ERR, "no ISDN controller found!");
		exit(1);
	}

	if(mvr.max_channels != MAX_CHANNELS)
	{
		log(LL_ERR, "max_channels mismatch, kernel %d, daemon %d",
		    mvr.max_channels, MAX_CHANNELS);
		exit(1);
	}

	/* set non-blocking mode */
	if(ioctl(isdnfd, FIONBIO, "\1\1\1") < 0)
	{
		log(LL_ERR, "ioctl FIONBIO failed: %s", strerror(errno));
		exit(1);
	}

	/* read runtime configuration file and configure ourselves */
	
	configure(configfile, 0);

	/* init active controllers, if any */
	
	signal(SIGCHLD, SIG_IGN);		/*XXX*/

	init_active_controller();

	signal(SIGCHLD, sigchild_handler);	/*XXX*/
	
	/* handle the rates stuff */
	
	readrates(ratesfile);

	/* if writing accounting info, open file, set unbuffered */
	
	if(useacctfile)
	{
		if((acctfp = fopen(acctfile, "a")) == NULL)
		{
			log(LL_ERR, "ERROR, can't open acctfile %s for writing, terminating!", acctfile);
			exit(1);
		}
		setvbuf(acctfp, (char *)NULL, _IONBF, 0);		
	}

	/* init remote monitoring */
	
#ifdef I4B_EXTERNAL_MONITOR
	if(do_monitor)
	{
		monitor_init();
		localmonitor = monitor_create_local_socket();
#ifndef I4B_NOTCPIP_MONITOR
		remotemonitor = monitor_create_remote_socket(monitorport);
#endif
	}
#endif
	
	/* in case fullscreendisplay, initialize */

#ifdef USE_CURSES
	if(do_fullscreen)
	{
		init_screen();
	}
#endif

	/* init realtime priority */
  		
#ifdef USE_RTPRIO
  	if(rt_prio != RTPRIO_NOTUSED)
  	{
  		struct rtprio rtp;

  		rtp.type = RTP_PRIO_REALTIME;
  		rtp.prio = rt_prio;

  		if((rtprio(RTP_SET, getpid(), &rtp)) == -1)
  		{
			log(LL_ERR, "rtprio failed: %s", strerror(errno));
			exit(1);
		}
	}
#endif

	starttime = time(NULL);	/* get starttime */

	srandom(580403);	/* init random number gen */

	mloop();       		/* enter loop of no return .. */

	exit(0);
	return(0);
}

/*---------------------------------------------------------------------------*
 *	program exit
 *---------------------------------------------------------------------------*/
void
error_exit(int exitval, const char *fmt, ...)
{
	log(LL_DMN, "fatal error, daemon terminating, exitval = %d", exitval);
	
	if(mailto[0] && mailer[0])
	{

#define EXITBL 2048

		char ebuffer[EXITBL];
		char sbuffer[EXITBL];
		va_list ap;

		va_start(ap, fmt);
		vsnprintf(ebuffer, EXITBL-1, fmt, ap);
		va_end(ap);

		signal(SIGCHLD, SIG_IGN);	/* remove handler */
		
		snprintf(sbuffer, sizeof(sbuffer), "%s%s%s%s%s%s%s%s",
			"cat << ENDOFDATA | ",
			mailer,
			" -s \"i4b isdnd: fatal error, terminating\" ",
			mailto,
			"\nThe isdnd terminated because of a fatal error:\n\n",
			ebuffer,
			"\n\nYours sincerely,\n   the isdnd\n",
			"\nENDOFDATA\n");
		system(sbuffer);
	}

	exit(exitval);
}

/*---------------------------------------------------------------------------*
 *	main loop
 *---------------------------------------------------------------------------*/
static void
mloop(void)
{
	fd_set set;
	struct timeval timo;
	int ret;
	int high_selfd;

 	/* go into loop */
	
 	log(LL_DMN, "i4b isdn daemon started (pid = %d)", getpid());
 
	for(;;)
	{
		FD_ZERO(&set);

#ifdef USE_CURSES
		if(do_fullscreen)
			FD_SET(fileno(stdin), &set);
#endif

		FD_SET(isdnfd, &set);

		high_selfd = isdnfd;
		
#ifdef I4B_EXTERNAL_MONITOR
		if(do_monitor)
		{
			if (localmonitor != -1) {
				/* always watch for new connections */
				FD_SET(localmonitor, &set);
				if(localmonitor > high_selfd)
					high_selfd = localmonitor;
			}
#ifndef I4B_NOTCPIP_MONITOR
			if (remotemonitor != -1) {
				FD_SET(remotemonitor, &set);
				if(remotemonitor > high_selfd)
					high_selfd = remotemonitor;
			}
#endif

			/* if there are client connections, let monitor module
			 * enter them into the fdset */
			if(accepted)
			{
				monitor_prepselect(&set, &high_selfd);
			}
		}
#endif

		timo.tv_sec = 1;
		timo.tv_usec = 0;

		ret = select(high_selfd + 1, &set, NULL, NULL, &timo);

		if(ret > 0)
		{	
			if(FD_ISSET(isdnfd, &set))
				isdnrdhdl();

#ifdef USE_CURSES
			if(FD_ISSET(fileno(stdin), &set))
				kbdrdhdl();
#endif

#ifdef I4B_EXTERNAL_MONITOR
			if(do_monitor)
			{
				if(localmonitor != -1 && FD_ISSET(localmonitor, &set))
					monitor_handle_connect(localmonitor, 1);

#ifndef I4B_NOTCPIP_MONITOR
				if(remotemonitor != -1 && FD_ISSET(remotemonitor, &set))
					monitor_handle_connect(remotemonitor, 0);
#endif
				if(accepted)
					monitor_handle_input(&set);
			}
#endif
		}
		else if(ret == -1)
		{
			if(errno != EINTR)
			{
				log(LL_ERR, "ERROR, select error on isdn device, errno = %d!", errno);
				error_exit(1, "mloop: ERROR, select error on isdn device, errno = %d!", errno);
			}
		}			

		/* handle timeout and recovery */		

		handle_recovery();
	}
	return;
}

#ifdef USE_CURSES
/*---------------------------------------------------------------------------*
 *	data from keyboard available, read and process it 
 *---------------------------------------------------------------------------*/
static void
kbdrdhdl(void)
{
	int ch = getch();

	if(ch == ERR)
	{
		log(LL_ERR, "ERROR, read error on controlling tty, errno = %d!", errno);
		error_exit(1, "kbdrdhdl: ERROR, read error on controlling tty, errno = %d!", errno);
	}

	switch(ch)
	{
		case 0x0c:	/* control L */
			wrefresh(curscr);
			break;

		case 0x1b:	/* escape */
		case '\r':
			do_menu();
			break;
	}
	return;
}
#endif

/*---------------------------------------------------------------------------*
 *	re-read the config file on SIGHUP or menu command
 *---------------------------------------------------------------------------*/
void
rereadconfig(int dummy)
{
	log(LL_DMN, "re-reading configuration file");

	(void)dummy;
	
	/* read runtime configuration file and configure ourselves */
	
	configure(configfile, 1);

	return;
}

/*---------------------------------------------------------------------------*
 *	re-open the log/acct files on SIGUSR1
 *---------------------------------------------------------------------------*/
void
reopenfiles(int dummy)
{
	(void)dummy;

        if(useacctfile)
	{
		/* close file */
		
		if(acctfp)
		{
		        fflush(acctfp);
		        fclose(acctfp);
		}

	        /* if user specified a suffix, rename the old file */
	        
	        if(rotatesuffix[0] != '\0')
	        {
	        	char filename[MAXPATHLEN];

	        	snprintf(filename, sizeof(filename), "%s%s", acctfile, rotatesuffix);

			if((rename(acctfile, filename)) != 0)
			{
				log(LL_ERR, "acct rename failed, cause = %s", strerror(errno));
				error_exit(1, "reopenfiles: acct rename failed, cause = %s", strerror(errno));
			}
		}

		if((acctfp = fopen(acctfile, "a")) == NULL)
		{
			log(LL_ERR, "ERROR, can't open acctfile %s for writing, terminating!", acctfile);
			error_exit(1, "ERROR, can't open acctfile %s for writing, terminating!", acctfile);
		}
		setvbuf(acctfp, (char *)NULL, _IONBF, 0);
	}

	if(uselogfile)
	{
	        finish_log();

	        /* if user specified a suffix, rename the old file */
	        
	        if(rotatesuffix[0] != '\0')
	        {
	        	char filename[MAXPATHLEN];

	        	snprintf(filename, sizeof(filename), "%s%s", logfile, rotatesuffix);

			if((rename(logfile, filename)) != 0)
			{
				log(LL_ERR, "log rename failed, cause = %s", strerror(errno));
				error_exit(1, "reopenfiles: log rename failed, cause = %s", strerror(errno));
			}
		}

	        if((logfp = fopen(logfile, "a")) == NULL)
		{
			fprintf(stderr, "ERROR, cannot open logfile %s: %s\n",
				logfile, strerror(errno));
			error_exit(1, "reopenfiles: ERROR, cannot open logfile %s: %s\n",
				logfile, strerror(errno));
		}

		/* set unbuffered operation */

		setvbuf(logfp, (char *)NULL, _IONBF, 0);
	}
	return;
}

