/*-
 * Copyright (c) 1999, 2002 Hellmuth Michaelis. All rights reserved.
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
 *	isdnphone - main module
 *	=======================
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

static void update_state(int event, char *reason);
static void kbd_hdlr(void);

#include "defs.h"
#include "isdn.h"
#include "display.h"

/*---------------------------------------------------------------------------*
 *	usage display and exit
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
	fprintf(stderr,
	"\nisdnphone - i4b phone program, version %d.%d.%d, compiled %s %s"
	"\nusage: isdnphone -d -h -k <string> -n <number> -u <unit> -v -w"
	"\n       -d            debug"
	"\n       -h            hangup"
	"\n       -k string     keypad string"
	"\n       -n number     dial number"
	"\n       -l loudness   set loudness(%d..%d)"
	"\n       -u unit       set unit number"
	"\n       -v            be verbose"
	"\n       -w            wait for response (with -n)"
	"\n", I4B_VERSION, I4B_REL, I4B_STEP, __DATE__, __TIME__,
		AUDIOAMP_MIN, AUDIOAMP_MAX);

	exit(1);
}

/*---------------------------------------------------------------------------*
 *	exit program
 *---------------------------------------------------------------------------*/
static void
at_exit(void)
{
	if(dialerfd != -1)
	{
	  update_state(EV_EXIT,
		       NULL);

	  close(dialerfd);
	  dialerfd = -1;
	}

	if(curses_ready)
	{
	  move(LINES-1, 0);
	  clrtoeol();
	  refresh();
	  endwin();
	  curses_ready = 0;
	}
	return;
}

u_int8_t select_timeout = 0;

/*---------------------------------------------------------------------------*
 *	program entry
 *---------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
	int c;
	int ret;
	char opt_n = 0;
	char opt_s = 0;
	char opt_h = 0;
	char opt_k = 0;
	char opt_w = 0;
	char opt_unit = 0;
	char *number = "";
	char *subaddr = "";

	/* init non-zero-global variables */

	dialerfd = -1;
	tel_fd = -1;

	while ((c = getopt(argc, argv, "dhk:l:n:s:u:vw")) != -1)
	{
		switch(c)
		{
			case 'h':
				opt_h = 1;
				break;
				
			case 'k':
				number = optarg;
				opt_k = 1;
				break;

			case 'l':
				loudness = strtol(optarg,0,0);
#if (AUDIOAMP_MAX < 0xFF)
				if(/*(loudness < AUDIOAMP_MIN) || */
				   (loudness > AUDIOAMP_MAX))
				{
				  usage();
				}
#endif
				break;

			case 'n':
				number = optarg;
				opt_n = 1;
				break;

			case 's':
				subaddr = optarg;
				opt_s = 1;
				break;
				
			case 'u':
				opt_unit = strtol(optarg,0,0);
				if((opt_unit < 0) /*||
				   (opt_unit > 9)*/)
				{
				  usage();
				}
				break;

			case 'd':
			case 'v':
				opt_d = 1;
				break;
				
			case 'w':
				opt_w = 1;
				break;
				
			case '?':
			default:
				usage();
				break;
		}
	}

	sprintf(&numberbuffer[0],"%s%d", I4BTELDDEVICE, opt_unit);

	if((dialerfd = init_dial(&numberbuffer[0])) == -1)
	{
		exit(1);
	}

	if(opt_n || opt_h || opt_k)
	{
		char commandbuffer[80];
		int exitval = 0;
		
		/* commandline operation goes here */
		
		if(opt_n)
		{
			if(opt_s)
				sprintf(commandbuffer, "D%s*%s", number, subaddr);
			else
				sprintf(commandbuffer, "D%s", number);
	
		}
		else if(opt_k)
		{
			sprintf(commandbuffer, "K%s", number);
	
		}
		else if(opt_h)
		{
			sprintf(commandbuffer, "H");
		}
	
		if((ret = write(dialerfd, commandbuffer, strlen(commandbuffer))) < 0)
		{
			exit_fatal("write commandbuffer failed: %s",
				   strerror(errno));
		}
	
		if(opt_n && opt_w)
		{
			u_int8_t result;

			if(read(dialerfd, &result, 1) < 0)
			{
				exitval = 99;
				debug("error\n");
				fprintf(stderr, "error, read failed: %s\n", strerror(errno));
			}
			else
			{
				if(result >= N_DSTATS)
				{
					exitval = 99;
					debug("error\n");
					fprintf(stderr, "unknown response = 0x%2x!", result);
				}
				else
				{
					/* please see ``i4b_ioctl.h'' */
					exitval = DSTATS_IPHONE_TABLE[result].exitvalue;
					debug("%s\n",DSTATS_IPHONE_TABLE[result].desc);
				}
			}
		}

		close(dialerfd);

		exit(exitval);
	}

	/* must setup ``atexit()'' after commandline operation(s)! */

	atexit(&at_exit);

	/* only setup tel-device when it should be used */

	sprintf(&numberbuffer[0],"%s%d", I4BTELDEVICE, opt_unit);

	if((tel_fd = init_tel(&numberbuffer[0],CVT_ALAW2ULAW)) == -1)
	{
		exit(1);
	}

	/* reset numberbuffer */
	numberbuffer[0] = '\0';	

	/* fullscreen operation here */	

	init_mainw();
	
	/* go into loop */

	for (;;)
	{
		int sound;
		int maxfd = 0;
		fd_set set;
		struct timeval timeout;

		FD_ZERO(&set);
		
		FD_SET(STDIN_FILENO, &set);
		if(STDIN_FILENO > maxfd)
			maxfd = STDIN_FILENO;
		
		FD_SET(dialerfd, &set);
		if(dialerfd > maxfd)
			maxfd = dialerfd;
		
		timeout.tv_sec = 4;
		timeout.tv_usec = 0;

		if(state == ST_INCOMING)
		{
		  select_timeout = 1;

		  if(sound & 3)
		  {
		    timeout.tv_sec = 0;
		    timeout.tv_usec = 250000;
		  }
		  else
		  {
		    timeout.tv_sec = 1;
		    timeout.tv_usec = 0;
		  }
		  sound++;
		  beep();
		}
		else
		{
		  sound = 1;
		}

		wrefresh(main_w);
		
		/* if no char is available within timeout, do something */
		
		ret = select(maxfd+1, &set, NULL, NULL,
			     select_timeout ? &timeout : NULL);

		if(ret > 0)
		{
			if(FD_ISSET(dialerfd, &set))
			{
				dial_hdlr();
			}
			if(FD_ISSET(STDIN_FILENO, &set))
			{
				kbd_hdlr();
			}
		}

		if(ret == 0)
		{
		  select_timeout = 0;
		  update_state(EV_UPDATE,
			 NULL);
		}
	}
	return(0);
}

static const char help[] = 
{
  "\n key | description"
  "\n ----+----------------------------------"
  "\n l   | increases loudness of loudspeaker"
  "\n L   | decreases loudness of loudspeaker"
  "\n ^C  | exits program"
  "\n ^D  | exits program"
  "\n ^L  | refreshes display"
  "\n b   | toggles large/normal buffer size"
  "\n h   | displays or hides help"
  "\n m   | toggles microphone mute when"
  "\n     | connected"
  "\n 1-9 | generates tone when connected"
  "\n A-D | generates tone when connected"
  "\n #   | generates tone when connected"
  "\n *   | generates tone when connected"
  "\n DEL | clears current telephone number"
  "\n ESC | disconnects call"
  "\n     |"
};

static void
dtmf_tone(u_int8_t key)
{
	struct i4b_tel_tones tt;

	bzero(&tt, sizeof(tt));

	tt.duration[0] = 8000 / 2; /* tone */
	tt.duration[1] = 8000 / 4; /* silence */
	tt.duration[2] = 0; /* end */

	switch(key)
	{
	case '#': tt.frequency_1[0] = 941; tt.frequency_2[0] = 1477; break; 
	case '*': tt.frequency_1[0] = 941; tt.frequency_2[0] = 1209; break; 
	case '0': tt.frequency_1[0] = 941; tt.frequency_2[0] = 1336; break; 
	case '1': tt.frequency_1[0] = 697; tt.frequency_2[0] = 1209; break; 
	case '2': tt.frequency_1[0] = 697; tt.frequency_2[0] = 1336; break; 
	case '3': tt.frequency_1[0] = 697; tt.frequency_2[0] = 1477; break; 
	case '4': tt.frequency_1[0] = 770; tt.frequency_2[0] = 1209; break; 
	case '5': tt.frequency_1[0] = 770; tt.frequency_2[0] = 1336; break; 
	case '6': tt.frequency_1[0] = 770; tt.frequency_2[0] = 1477; break; 
	case '7': tt.frequency_1[0] = 852; tt.frequency_2[0] = 1209; break; 
	case '8': tt.frequency_1[0] = 852; tt.frequency_2[0] = 1336; break; 
	case '9': tt.frequency_1[0] = 852; tt.frequency_2[0] = 1477; break; 
	case 'A': tt.frequency_1[0] = 697; tt.frequency_2[0] = 1633; break; 
	case 'B': tt.frequency_1[0] = 770; tt.frequency_2[0] = 1633; break; 
	case 'C': tt.frequency_1[0] = 852; tt.frequency_2[0] = 1633; break; 
	case 'D': tt.frequency_1[0] = 941; tt.frequency_2[0] = 1633; break;
	default:
		return;
	}
	ioctl(tel_fd,I4B_TEL_TONES,&tt);
	return;
}

/*---------------------------------------------------------------------------*
 *	keyboard character available handler
 *---------------------------------------------------------------------------*/
static void
kbd_hdlr(void)
{		
	int kchar;

	kchar = wgetch(main_w);		/* get char */

	debug("char=0x%02x\n", kchar);

	if(state == ST_ACTIVE)
	{
		if((kchar == '#') ||
		   (kchar == '*') ||
		   ((kchar >= '0') && (kchar <= '9')) ||
		   ((kchar >= 'A') && (kchar <= 'D')))
		{
			dtmf_tone(kchar);
			goto display_char;
		}
	}

	switch (kchar)
	{
		case 0x1b:      /* escape */
			update_state(EV_EXIT,
			       NULL);
			break;

		case CR:
		case LF:
#ifndef KEY_ENTER
		case KEY_ENTER:
#endif
			update_state(EV_ENTER,
			       NULL);
			break;

		case 'L':
		case '-':
			if(loudness >= (AUDIOAMP_MIN+(AUDIOAMP_DP/4)))
			{
			  loudness -= AUDIOAMP_DP/4;
			}
			else
			{
			  loudness = AUDIOAMP_MIN;
			}

			/* update loudness */
			update_state(EV_LOUDNESS_UPDATE,
			       NULL);
			break;

		case 'l':
		case '+':
			if(loudness <= (AUDIOAMP_MAX-(AUDIOAMP_DP/4)))
			{
			  loudness += AUDIOAMP_DP/4;
			}
			else
			{
			  loudness = AUDIOAMP_MAX;
			}

			/* update loudness */
			update_state(EV_LOUDNESS_UPDATE,
			       NULL);
			break;

		case 'B':
		case 'b':
			update_state(EV_LARGE_BUFFER,
			       NULL);
			break;

		case 'M':
		case 'm':
			update_state(EV_MUTE,
			       NULL);
			break;

		case 'h':
		case 'H':
			do_help();
			break;

		case CNTRL_C:
		case CNTRL_D:
			exit(0);
			break;
			/* beep(); */

		case CNTRL_L:	/* refresh */
			touchwin(curscr);
			wrefresh(curscr);
			break;

		case KEY_DC:
			curx = 0;
			numberbuffer[curx] = '\0';
			update_state(EV_UPDATE,
			       NULL);
			break;

		case KEY_BACKSPACE:
			if(curx)
			{
			  curx--;
			  numberbuffer[curx] = '\0';
			  update_state(EV_UPDATE,
				 NULL);
			}
			break;
			
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
	    display_char:
			if(curx < (INDEXES(numberbuffer)-1))
			{
			  numberbuffer[curx] = kchar;
			  curx++;
			  numberbuffer[curx] = '\0';
			  update_state(EV_UPDATE,
				 NULL);
			}
			break;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	update state
 *---------------------------------------------------------------------------*/
static void
update_state(int event, char *reason)
{
	static const char * const
	  MAKE_TABLE(EVENTS,DESC,[]);
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
	u_int8_t buffer[max(max(max(64,USER_USER_MAX),TELNO_MAX),DISPLAY_MAX)];
	u_int16_t temp;

	debug("%s (%d)\n",
	      EVENTS_DESC[event], event);

	switch(event)
	{
	case EV_CALL_IN:
		state = ST_INCOMING;
		reason = 0;

		wprintw(dbg_w, "incoming call from ");

		if(ioctl(tel_fd,I4B_TEL_GET_TELNO,&buffer[0]) == 0)
		{
			wprintw(dbg_w, "%s", &buffer[0]);
		}

		if((ioctl(tel_fd,I4B_TEL_GET_DISPLAY,&buffer[0]) == 0) &&
		   buffer[0])
		{
			wprintw(dbg_w, " (%s)", &buffer[0]);
		}

		if((ioctl(tel_fd,I4B_TEL_GET_SMS,&buffer[0]) == 0) &&
		   buffer[0])
		{
			wprintw(dbg_w, " (SMS: %s)", &buffer[0]);
		}

		wprintw(dbg_w, "\n");
		wrefresh(dbg_w);
		break;

	case EV_DISCONNECT:
		state = ST_IDLE;
		break;

	case EV_CONNECTED:
		state = ST_ACTIVE;
		reason = 0;
		break;

	case EV_ENTER:
		if(state == ST_IDLE)
		{
		  /* check if there is an
		   * output-telephone-number
		   */
		  if(numberbuffer[0] != '\0')
		  {
			state = ST_OUTGOING;
			do_dial(&numberbuffer[0]);
		  }
		}
		if(state == ST_INCOMING)
		{
			state = ST_ACTIVE;
			do_answer();
		}
		break;

	case EV_EXIT:
		if((state == ST_INCOMING) ||
		   (state == ST_OUTGOING) ||
		   (state == ST_ACTIVE) ||
		   (state == ST_IDLE))
		{
			state = ST_IDLE;
			do_hangup();
		}
		break;

	case EV_LARGE_BUFFER:
		if(large_buffer)
		{
		disable_large_buffer:
		  /* disable large_buffer */
		  ioctl(tel_fd,I4B_TEL_DISABLE_LARGE_BUFFER,NULL);
		  reason = "normal buffer";
		  large_buffer = 0;
		}
		else
		{
		  /* enable large_buffer */
		  ioctl(tel_fd,I4B_TEL_ENABLE_LARGE_BUFFER,NULL);
		  reason = "large buffer";
		  large_buffer = 1;
		}
		select_timeout = 1;
		break;

	case EV_LOUDNESS_UPDATE:
		/* update loudness */
		ioctl(tel_fd,I4B_TEL_SET_AUDIOAMP,&loudness);

		/* print loudness */
		strcpy(&buffer[0], 
		       "loudness: -[--------------------------------]+");

		temp = loudness;
		temp *= 32;
		temp /= AUDIOAMP_MAX;

		while(temp--)
		{
		  buffer[0x0C+temp] = '|';
		}

		reason = &buffer[0];
		select_timeout = 1;
		break;

	case EV_MUTE:
		if(state == ST_ACTIVE)
		{
		  if(mute)
		  {
		  disable_mute:
		    /* disable mute */
		    ioctl(tel_fd,I4B_TEL_DISABLE_MUTE,NULL);
		    mute = 0;
		  }
		  else
		  {
		    /* enable mute */
		    ioctl(tel_fd,I4B_TEL_ENABLE_MUTE,NULL);
		    mute = 1;
		  }
		}
		break;
	}

	if(state == ST_ACTIVE)
	{
	  /* could start some processes 
	   * enter_tel()
	   */
	}
	else
	{
	  if(mute)
	  {
	    goto disable_mute;
	  }

	  /* could stop some processes
	   * exit_tel()
	   */
	}

	if(help_w)
	{
	  /* remove help-window
	   */
	  do_help();
	}

	if(curses_ready)
	{
	  /* update screen */
	  static const char * const
	    MAKE_TABLE(STATES,DESC,[]);
	  static const char * const
	    MAKE_TABLE(STATES,MESSAGE,[]);

	  mvwprintw(main_w, MW_STATEY, MW_STX, "%-48s",
		    STATES_DESC[state]);

	  mvwprintw(main_w, MW_NUMY, MW_NUX, "%-48s",
		    &numberbuffer[0]);

	  mvwprintw(main_w, MW_MSGY, MW_MSX, "%-48s",
		    reason ? reason : STATES_MESSAGE[state]);

	  mvwprintw(main_w,           1, MW_WIDTH-1-10, "%-10s",
		    mute ? "(m)ute on" : "");

	  wmove(main_w, MW_NUMY, MW_NUX + curx);
	  wrefresh(main_w);
	}
	return;
}
/* EOF */
