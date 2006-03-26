/*
 * Copyright (c) 1997, 2001 Hellmuth Michaelis. All rights reserved.
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
 *	main.c - i4b set debug options
 *	------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdndebug/main.c,v 1.11 2001/05/25 08:36:45 hm Exp $
 *
 *      last edit-date: [Mon May 21 10:09:23 2001]
 *
 *---------------------------------------------------------------------------*/

#include <stdio.h>
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

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>

static int isdnfd;

#define I4BCTLDEVICE	"/dev/i4bctl"

u_int opt_layer = -1;
u_int opt_chan = -1;
u_int opt_unit = 0;

i4b_debug_t dbg;

static const struct
{
  u_int8_t *name;
  u_int8_t value;
}
	enum_list[] =
{
#define MACRO_0(enum,args...) { #enum, enum },
    I4B_PROTOCOLS(MACRO_0)
    I4B_DRIVERS(MACRO_0)
};

static int
_atoi(u_int8_t *optarg)
{
  static const typeof(enum_list[0]) *item;
  u_int8_t n;

  item = &enum_list[0];
  n = INDEXES(enum_list);

  while(n--)
  {
    if(strcmp(item->name,optarg) == 0)
    {
      return(item->value);
    }
    item++;
  }

  return(atoi(optarg));
}

#define atoi _atoi

/*---------------------------------------------------------------------------*
 *	usage display and exit
 *
 * NOTE: the text below use tabs
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
#define MACRO_1(enum,args...) #enum ", "

  fprintf(stdout,
  """isdndebug - i4b set debug level, version %d.%d.%d, compiled %s %s"
  "\nusage:	isdndebug -c -e -g -l <layer> -s <keyword> -m -q -r" 
  "\n	isdndebug -u <mux unit> -z -C -Q" 
  "\n	isdndebug -u 0 -ta -r -b1 -p11 -l1 -g -l2 -g" 
  "\n	-a		activate PH-line" 
  "\n	-D		deactivate PH-line" 
  "\n	-t		set TE-mode" 
  "\n	-n		set NT-mode" 
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
  "\n	-P		set polled mode" 
  "\n	-S		set standard mode" 
  "\n	-u mux unit	mux unit number(0..,default=0)" 
  "\n" 
  "\nenums: "
  I4B_PROTOCOLS(MACRO_1)
  I4B_DRIVERS(MACRO_1)
  "\n",
	  I4B_VERSION, I4B_REL, I4B_STEP, __DATE__, __TIME__);

  exit(1);
}

/*---------------------------------------------------------------------------*
 *	i4b_print_events
 *---------------------------------------------------------------------------*/
static void
i4b_print_events()
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
}

/*---------------------------------------------------------------------------*
 *	i4b_ioctl
 *---------------------------------------------------------------------------*/
static void
i4b_ioctl(int cmdr, const char *err_msg, void *arg)
{
  if(ioctl(isdnfd, cmdr, arg) < 0)
  {
    fprintf(stderr, "ioctl %s failed: %s.\n",
	    err_msg, strerror(errno));
    exit(1);
  }
}

#define i4b_ioctl(cmdr,arg) i4b_ioctl(cmdr,#cmdr,arg)

#define START      0xff
#define END        0x00

#define PRINT 8
#define RESET 4
#define ADD   2
#define REM   1

#define DEBUG_DESC_1(layer, name, group, desc, first, last)	\
first ("") #name ") " desc "\000" last ("\377" /*0xff*/)

#define DEBUG_DESC_2(layer, name, group, desc, first, last)	\
first (L##layer##_DEBUG_ERR,)

#define DEBUG_DESC_3(layer, name, group, desc, first, last)	\
first (L##layer##_DEBUG_MAX,)

#define DEBUG_DESC_4(layer, name, group, desc, first, last)	\
first (L##layer##_DEBUG_DEFAULT,)

#define DEBUG_DESC_5(layer, name, group, desc, first, last)	\
first (-1,)

/*
 * Format of layer_start[]:
 * ========================
 *
 * +--- - - - ---+------+--- - - - ---+------+-------+
 * | description | END  | description | END  | START |
 * +--- - - - ---+------+--- - - - ---+------+-------+
 * |     ...     | 0x00 |     ...     | 0x00 |  0xff |
 * +--- - - - ---+------+--- - - - ---+------+-------+
 *
 */

static const u_int8_t layer_start[] = I4B_DEBUG_MASKS(DEBUG_DESC_1);

typedef __typeof(dbg.debug[0]) mask_t;

static const mask_t

group_err[] =
{
  I4B_DEBUG_MASKS(DEBUG_DESC_2)
},

group_max[] =
{
  I4B_DEBUG_MASKS(DEBUG_DESC_3)
},

group_def[] =
{
  I4B_DEBUG_MASKS(DEBUG_DESC_4)
},

group_all[] =
{
  I4B_DEBUG_MASKS(DEBUG_DESC_5)
};

/*---------------------------------------------------------------------------*
 *	update debugging masks
 *---------------------------------------------------------------------------*/
static void
debug_mask_update(const mask_t *group, u_int8_t *str, u_int8_t _cmd)
{
  u_int8_t buf[256];

  __typeof(_cmd) cmd;

  mask_t          mask;
  u_int          layer = 1;
  mask_t          *var = &dbg.debug[1];
  const mask_t    *err = &group_err[0];
  const u_int8_t *desc = &layer_start[0];

	i4b_ioctl(I4B_CTL_GET_DEBUG, &dbg);

 start_layer:
	/* check if end of
	 * layer
	 */
	if(desc[0] == END)
	{
	  goto done;
	}

	/* disable commands */
	cmd = 0;

	/* reset mask */
	mask = 1;

	/* check if layer
	 * matches
	 */
	if((opt_layer == (__typeof(opt_layer))-1) ||
	   (opt_layer == layer))
	{
	  /* enable commands, if any */
	  cmd = _cmd;

	  if(cmd & PRINT)
	  {
	    printf("\nLayer%d messages (0x%08x):\n\n",
		   layer, *var);
	  }
	}

 start_mask:
	/* check for matching
	 * group
	 */
	if((*group) & mask)
	{
	  /* print out full description
	   * to buffer
	   */
	  snprintf(&buf[0], sizeof(buf), "	[%s] (L%d_%s %s\n",
		   ((*var) & mask) ? "X" : "-",
		   layer, desc, ((*err) & mask) ? "errors" : "messages");

	  /* check if keyword, pointed to
	   * by str, has a match in the
	   * full description
	   */
	  if(strstr(&buf[0], str))
	  {
	    /* add bit */
	    if(cmd & (ADD|RESET))
	    {
	      (*var) |= (mask);
	    }

	    /* remove bit */
	    if(cmd & (REM))
	    {
	    remove_bit:
	      (*var) &= ~(mask);
	    }
#if 0
	    printf("(0x%08x 0x%08x)", mask, (*group));
#endif
	    /* print */
	    if(cmd & (PRINT))
	    {
	      printf(&buf[0]);
	    }
	  }
	}
	else
	{
	    if(cmd & (RESET))
	    {
	      /* force bits to have
	       * the same value as
	       * the selected group,
	       * in case of RESET
	       */
	      goto remove_bit;
	    }
	}

	/* update mask */
	mask <<= 1;

 search:
	/* new layer or new mask */
	if(desc[0] == END)
	{
	  /* get next byte */
	  desc++;

	  /* new layer */
	  if(desc[0] == START)
	  {
	    /* get next byte */
	    desc  ++;

	    /* increment
	     * layer number
	     */
	    group ++;
	    layer ++;
	    var   ++;
	    err   ++;

	    goto start_layer;
	  }
	  else
	  {
	    goto start_mask;
	  }
	}

	/* get next byte */
	desc++;

	goto search;

 done:
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
	  usage();

	if((isdnfd = open(I4BCTLDEVICE, O_RDWR)) < 0)
	{
		fprintf(stderr, "i4bctl: cannot open %s: %s\n",
			I4BCTLDEVICE, strerror(errno));
		exit(1);
	}

	while ((c = getopt(argc, argv, "ab:cdeE:ghi:l:mnp:qrtT:s:u:zCDHPQS")) != -1)
	{
	  /* zero is default */
	  bzero(&dbg, sizeof(dbg));

	  /* setup unit number */
	  dbg.unit = opt_unit;

	  switch(c)
	  {
	    case 'T':
	      dbg.value = atoi(optarg);
	      i4b_ioctl(I4B_CTL_SET_N_DRIVER_TYPE, &dbg);
	      break;

	    case 'i':
	      dbg.value = atoi(optarg);
	      i4b_ioctl(I4B_CTL_SET_N_SERIAL_NUMBER, &dbg);
	      break;

	    case 'P':
	      i4b_ioctl(I4B_CTL_SET_POLLED_MODE, &dbg);
	      break;

	    case 'S':
	      i4b_ioctl(I4B_CTL_SET_STANDARD_MODE, &dbg);
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

	    case 't':
	      i4b_ioctl(I4B_CTL_SET_TE_MODE, &dbg);
	      break;

	    case 'n':
	      i4b_ioctl(I4B_CTL_SET_NT_MODE, &dbg);
	      break;

	    case 'd':
	      debug_mask_update(&group_def[0], "", RESET);
	      break;

	    case 'E':
	      printf("enum ``%s'' has value 0x%02x\n",
		     optarg, atoi(optarg));
	      break;

	    case 'e':
	      debug_mask_update(&group_err[0], "", RESET);
	      break;

	    case 'g': 
	      debug_mask_update(&group_all[0], "", PRINT);
	      break;

	    case 'm':
	      debug_mask_update(&group_max[0], "", RESET);
	      break;

	    case 'z':
	      debug_mask_update(&group_all[0], "", REM);
	      break;

	    case 's':
	      /* get keyword */
	      if(optarg[0] == '-')
	      {
		  /* remove ``bits'' that match keyword */
		  debug_mask_update(&group_all[0], optarg+1, REM);
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
		  debug_mask_update(&group_all[0], optarg, ADD);
	      }
	      break;

	    case 'u':
	      /* get unit */
	      opt_unit = atoi(optarg);
	      opt_layer = -1;
	      opt_chan = -1;
	      break;

	    case 'l':
	      /* get layer */
	          opt_layer = atoi(optarg);
	      if((opt_layer < 1) ||
		 (opt_layer > 4))
	      {
		  usage();
	      }
	      break;

	    case 'b':
	      /* get channel */
	         opt_chan = (1 << atoi(optarg));
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
	      dbg.value = atoi(optarg);

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
		     "\n# of I-frames           %12lu %12lu"
		     "\n# of RR-frames          %12lu %12lu"
		     "\n# of RNR-frames         %12lu %12lu"
		     "\n# of REJ-frames         %12lu %12lu"
		     "\n# of SABME-frames       %12lu %12lu"
		     "\n# of DM-frames          %12lu %12lu"
		     "\n# of DISC-frames        %12lu %12lu"
		     "\n# of UA-frames          %12lu %12lu"
		     "\n# of FRMR-frames        %12lu %12lu"
		     "\n# of TEI-frames         %12lu %12lu"
		     "\n# of UI-frames          %12lu      "
		     "\n# of XID-frames         %12lu      "
		     "\n                                           errors "
		     "\n------------------------------------------------- "
		     "\n# of frames with incorrect length    %12lu"
		     "\n# of frames with bad frame type      %12lu"
		     "\n# of bad S-frames                    %12lu"
		     "\n# of bad U-frames                    %12lu"
		     "\n# of bad UI-frames                   %12lu"
		     "\n",
		     dbg.unit,
		     dbg.lapdstat.rx_i,     dbg.lapdstat.tx_i,
		     dbg.lapdstat.rx_rr,    0,
		     dbg.lapdstat.rx_rnr,   0,
		     dbg.lapdstat.rx_rej,   0,
		     dbg.lapdstat.rx_sabme, 0,
		     dbg.lapdstat.rx_dm,    0,
		     dbg.lapdstat.rx_disc,  0,
		     dbg.lapdstat.rx_ua,    0,
		     dbg.lapdstat.rx_frmr,  0,
		     dbg.lapdstat.rx_tei,   dbg.lapdstat.tx_tei,
		     dbg.lapdstat.rx_ui,
		     dbg.lapdstat.rx_xid,
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

/* EOF */
