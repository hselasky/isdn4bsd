/*
 * Copyright (c) 1997, 2001 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2004-2006 Hans Petter Selasky. All rights reserved.
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
 *	main.c - I4B set debug options
 *	------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdndebug/main.c $
 *
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <err.h>
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

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>

#define I4BCTLDEVICE	"/dev/i4bctl"

static int isdnfd;
static u_int opt_layer = -1;
static u_int opt_chan = -1;
static u_int opt_unit = 0;

static i4b_debug_t dbg;

static const struct enum_list {
  const char *name;
  u_int8_t value;
}
	enum_list[] =
{
#define MACRO_0(enum,...) { #enum, enum },
    I4B_PROTOCOLS(MACRO_0)
    I4B_DRIVERS(MACRO_0)
    { NULL, 0 }
};

static int
__atoi(const u_int8_t *arg)
{
    const struct enum_list *item = &enum_list[0];

    while(item->name)
    {
        if(strcmp(item->name,arg) == 0)
	{
	    return(item->value);
	}
	item++;
    }
    return(atoi(arg));
}

/*---------------------------------------------------------------------------*
 *	usage display and exit
 *
 * NOTE: the text below uses tabs
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
#define MACRO_1(enum,...) #enum ", "

  fprintf(stdout,
  """isdndebug - I4B set debug level, version %02d.%02d.%d"
  "\nusage:	isdndebug -c -e -g -l <layer> -s <keyword> -m -q -r" 
  "\n	isdndebug -u <mux unit> -z -C -Q" 
  "\n	isdndebug -u 0 -ta -r -b1 -p11 -l1 -g -l2 -g" 
  "\n	-a		activate PH-line" 
  "\n	-D		deactivate PH-line" 
  "\n	-r		reset device" 
  "\n	-p value	set protocol (P_XXX)" 
  "\n	-b value	set channel(0..2) for protocol(default=all channels)"
  "\n	-i value	set N_serial_number for D-channel"
  "\n	-T value	set N_driver_type for D-channel"
  "\n	-m		set maximum debugging bits" 
  "\n	-d		set default debugging bits" 
  "\n	-e		set error   debugging bits" 
  "\n	-z		set zero    debugging bits" 
  "\n	-s +/-keyword	set custom  debugging bit"
  "\n	-l layer	set custom  debugging layer(1..4,default=all layers)"
  "\n	-g		get current debugging bits" 
  "\n	-E enum		get value of enum"
  "\n	-c		get chipset statistics" 
  "\n	-q		get Q.921 statistics" 
  "\n	-C		reset chipset statistics" 
  "\n	-Q		reset Q.921 statistics" 
  "\n	-u mux unit	mux unit number(0..,default=0)" 
  "\n" 
  "\nenums: "
  I4B_PROTOCOLS(MACRO_1)
  I4B_DRIVERS(MACRO_1)
  "\n",
	  I4B_VERSION, I4B_REL, I4B_STEP);

  exit(1);
}

/*---------------------------------------------------------------------------*
 *	i4b_print_events
 *---------------------------------------------------------------------------*/
static void
i4b_print_events(void)
{
    printf("\n"
	   "Chip statistics for unit %2d, chan %2d:\n"
	   "    VFR"
	   "    RDO"
	   "    CRC"
	   "    RAB"
	   "    XDU"
	   "    RFO\n"
	   "%7d%7d%7d%7d%7d%7d\n\n",
	   dbg.unit,
	   dbg.chan,
	   dbg.chanstat.vfr,
	   dbg.chanstat.rdo,
	   dbg.chanstat.crc,
	   dbg.chanstat.rab,
	   dbg.chanstat.xdu,
	   dbg.chanstat.rfo);
    return;
}

/*---------------------------------------------------------------------------*
 *	i4b_ioctl
 *---------------------------------------------------------------------------*/
static void
i4b_ioctl(int cmdr, const char *err_msg, void *arg)
{
    if(ioctl(isdnfd, cmdr, arg) < 0)
    {
        err(1, "ioctl %s failed!", err_msg);
    }
    return;
}

#define i4b_ioctl(cmdr,arg) \
        i4b_ioctl(cmdr,#cmdr,arg)

enum {
    I4B_DEBUG_FIELDS(I4B_DEBUG_MAKE_ENUM,)
    I4B_DEBUG_ENUMS
};

static const char *i4b_debug_desc[I4B_DEBUG_ENUMS] = {
    I4B_DEBUG_FIELDS(I4B_DEBUG_MAKE_TABLE_0, )
};

/*---------------------------------------------------------------------------*
 *	get a debug bit by number
 *---------------------------------------------------------------------------*/
static u_int8_t
i4b_debug_get_bit(struct i4b_debug_mask *pmask, u_int16_t n)
{
    switch(n) {
      I4B_DEBUG_FIELDS(I4B_DEBUG_MAKE_CASE_GET, pmask);
    }
    return 0;
 one:
    return 1;
}

/*---------------------------------------------------------------------------*
 *	set a debug bit by number
 *---------------------------------------------------------------------------*/
static void
i4b_debug_set_bit(struct i4b_debug_mask *pmask, 
		  u_int16_t n, u_int8_t value)
{
    switch(n) {
      I4B_DEBUG_FIELDS(I4B_DEBUG_MAKE_CASE_SET, pmask);
    }
    return;
}

#define PRINT 4
#define ADD   2
#define REM   1

/*---------------------------------------------------------------------------*
 *	update debugging bits
 *---------------------------------------------------------------------------*/
static void
i4b_debug_mask_update(const char *str, u_int8_t cmd)
{
    u_int8_t buf[256];
    u_int8_t layer[8];
    u_int16_t n;
    u_int8_t c;

    i4b_ioctl(I4B_CTL_GET_DEBUG, &dbg);

    if(opt_layer == ((__typeof(opt_layer))-1))
    {
        layer[0] = 0; /* all layers */
    }
    else
    {
        snprintf(layer, sizeof(layer), 
		 "(L%d_", opt_layer);
    }

    for(n = 0; n < I4B_DEBUG_ENUMS; n++)
    {
        c = i4b_debug_get_bit(&dbg.debug, n);

        /* print out the full 
	 * description to a buffer:
	 */
        snprintf(buf, sizeof(buf), "	[%c] %s\n",
		 (c ? 'X' : '-'), i4b_debug_desc[n]);

	/* check if keyword does not match */

	if(strstr(buf, str) == NULL)
	{
	    continue;
	}

	/* check if layer does not match */

	if(strstr(buf, layer) == NULL)
	{
	    continue;
	}

	/* add bit */

	if(cmd & ADD)
	{
	    c = 1;
	}

	/* remove bit */

	if(cmd & REM)
	{
	    c = 0;
	}

	/* print bit */

	if(cmd & PRINT)
	{
	   if(n == L1_ERROR_ENUM)
	   {
	       printf("\nLayer 1 messages:\n\n");
	   }

	   if(n == L2_ERROR_ENUM)
	   {
	       printf("\nLayer 2 messages:\n\n");
	   }

	   if(n == L3_ERR_ENUM)
	   {
	       printf("\nLayer 3 messages:\n\n");
	   }

	   if(n == L4_ERR_ENUM)
	   {
	       printf("\nLayer 4 messages:\n\n");
	   }

	   printf("%s", &buf[0]);
	}

	i4b_debug_set_bit(&dbg.debug, n, c);
    }

    i4b_ioctl(I4B_CTL_SET_DEBUG, &dbg);
    return;
}

/*---------------------------------------------------------------------------*
 *	program entry
 *---------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
    register int c;

    if(argc <= 1)
    {
        usage();
    }

    if((isdnfd = open(I4BCTLDEVICE, O_RDWR)) < 0)
    {
        err(1, "cannot open %s", I4BCTLDEVICE);
    }

    while ((c = getopt(argc, argv, "ab:cdeE:ghi:l:mnp:qrtT:s:u:zCDHPQS")) != -1)
    {
	    /* zero is default */
	    bzero(&dbg, sizeof(dbg));

	    /* setup unit number */
	    dbg.unit = opt_unit;

	    switch(c) {
	    case 'T':
	      dbg.value = __atoi(optarg);
	      i4b_ioctl(I4B_CTL_SET_N_DRIVER_TYPE, &dbg);
	      break;

	    case 'i':
	      dbg.value = __atoi(optarg);
	      i4b_ioctl(I4B_CTL_SET_N_SERIAL_NUMBER, &dbg);
	      break;

	    case 'P':
	    case 'S':
	    case 't':
	    case 'n':
	      warn("option '-%c' is obsolete. "
		   "Use 'isdnconfig' instead!", c);
	      break;

	    case 'a':
	      i4b_ioctl(I4B_CTL_PH_ACTIVATE, &dbg);
	      break;

	    case 'D':
	      i4b_ioctl(I4B_CTL_PH_DEACTIVATE, &dbg);
	      break;

	    case 'r':
	      i4b_ioctl(I4B_CTL_RESET, &dbg);
	      break;

	    case 'E':
	      printf("enum ``%s'' has value 0x%02x\n",
		     optarg, __atoi(optarg));
	      break;

	    case 'd':
	    case 'e':
	      dbg.debug = i4b_debug_err;
	      i4b_ioctl(I4B_CTL_SET_DEBUG, &dbg);
	      break;

	    case 'g': 
	      i4b_debug_mask_update("", PRINT);
	      break;

	    case 'm':
	      dbg.debug = i4b_debug_max;
	      i4b_ioctl(I4B_CTL_SET_DEBUG, &dbg);
	      break;

	    case 'z':
	      dbg.debug = i4b_debug_zero;
	      i4b_ioctl(I4B_CTL_SET_DEBUG, &dbg);
	      break;

	    case 's':
	      /* get keyword */
	      if(optarg[0] == '-')
	      {
		  /* remove ``bits'' that match keyword */
		  i4b_debug_mask_update(optarg+1, REM);
	      }
	      else
	      {
		  /* allow a '+' before
		   * the keyword
		   */
		  if(optarg[0] == '+')
		  {
		      optarg++;
		  }

		  /* add ``bits'' that match keyword */
		  i4b_debug_mask_update(optarg, ADD);
	      }
	      break;

	    case 'u':
	      /* get unit */
	      opt_unit = __atoi(optarg);
	      opt_layer = -1;
	      opt_chan = -1;
	      break;

	    case 'l':
	      /* get layer */
	          opt_layer = __atoi(optarg);
	      if((opt_layer < 1) ||
		 (opt_layer > 4))
	      {
		  usage();
	      }
	      break;

	    case 'b':
	      /* get channel */
	         opt_chan = (1 << __atoi(optarg));
	      if(opt_chan == 0)
	      {
		  usage();
	      }
	      break;

	    case 'c':
	    case 'C':
	      /* get chipset statistics */

	      for(c = 1; c; c <<= 1)
	      {
		  if(opt_chan & c)
		  {
		    i4b_ioctl(I4B_CTL_GET_CHIPSTAT, &dbg);

		    i4b_print_events();

		    if(c == 'C')
		    {
		      /* clear chipset statistics */
		      i4b_ioctl(I4B_CTL_CLR_CHIPSTAT, &dbg);
		    }
		  }

		  /* next channel */
		  dbg.chan++;
	      }
	      break;

	    case 'p':
	      /* set protocol(s) */
	      dbg.value = __atoi(optarg);

	      for(c = 1; c; c <<= 1)
	      {
		  if(opt_chan & c)
		  {
		      i4b_ioctl(I4B_CTL_SET_PROTOCOL, &dbg);
		  }

		  /* next channel */
		  dbg.chan++;
	      }
	      break;

	    case 'Q':
	    case 'q':
	      i4b_ioctl(I4B_CTL_GET_LAPDSTAT, &dbg);

	      printf("\nmux unit %d Q.921 statistics: receive     transmit"
		     "\n-------------------------------------------------"
		     "\n# of I-frames           %12u %12u"
		     "\n# of RR-frames          %12u %12u"
		     "\n# of RNR-frames         %12u %12u"
		     "\n# of REJ-frames         %12u %12u"
		     "\n# of SABME-frames       %12u %12u"
		     "\n# of DM-frames          %12u %12u"
		     "\n# of DISC-frames        %12u %12u"
		     "\n# of UA-frames          %12u %12u"
		     "\n# of FRMR-frames        %12u %12u"
		     "\n# of TEI-frames         %12u %12u"
		     "\n# of UI-frames          %12u      "
		     "\n# of XID-frames         %12u      "
		     "\n                                           errors "
		     "\n------------------------------------------------- "
		     "\n# of frames with incorrect length    %12u"
		     "\n# of frames with bad frame type      %12u"
		     "\n# of bad S-frames                    %12u"
		     "\n# of bad U-frames                    %12u"
		     "\n# of bad UI-frames                   %12u"
		     "\n",
		     dbg.unit,
		     (unsigned)dbg.lapdstat.rx_i,     (unsigned)dbg.lapdstat.tx_i,
		     (unsigned)dbg.lapdstat.rx_rr,    0,
		     (unsigned)dbg.lapdstat.rx_rnr,   0,
		     (unsigned)dbg.lapdstat.rx_rej,   0,
		     (unsigned)dbg.lapdstat.rx_sabme, 0,
		     (unsigned)dbg.lapdstat.rx_dm,    0,
		     (unsigned)dbg.lapdstat.rx_disc,  0,
		     (unsigned)dbg.lapdstat.rx_ua,    0,
		     (unsigned)dbg.lapdstat.rx_frmr,  0,
		     (unsigned)dbg.lapdstat.rx_tei,   (unsigned)dbg.lapdstat.tx_tei,
		     (unsigned)dbg.lapdstat.rx_ui,
		     (unsigned)dbg.lapdstat.rx_xid,
		     0,
		     0,
		     0,
		     0,
		     0);

	      if(c == 'Q')
	      {
		i4b_ioctl(I4B_CTL_CLR_LAPDSTAT, &dbg);

		printf("\nQ.921 statistics counters for mux unit "
		       "%d, reset to zero!\n", dbg.unit);
	      }
	      break;

	    case '?':
	    default:

	      usage();
	      break;

	    } /* end of switch(c) */

    } /* end of while((c = ...)) */

    return 0;
}
