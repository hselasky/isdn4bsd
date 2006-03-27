/*-
 * Copyright (c) 2005-2006 Hans Petter Selasky. All rights reserved.
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
 *	main.c - utility to configure ISDN4BSD
 *	--------------------------------------
 *
 * $FreeBSD: $
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
#include <err.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>

#define CAPI_DEVICE "/dev/capi20"

static int isdnfd;

struct options {
    u_int16_t unit;
    u_int32_t serial;
    u_int32_t driver_type;

    u_int8_t  got_any : 1;
    u_int8_t  got_u : 1;
    u_int8_t  got_i : 1;
    u_int8_t  got_p : 1;
    u_int8_t  got_nt_mode : 1;
    u_int8_t  got_te_mode : 1;
    u_int8_t  got_hi_pri : 1;
    u_int8_t  got_lo_pri : 1;
    u_int8_t  got_up : 1;
    u_int8_t  got_down : 1;
    u_int8_t  got_pwr_save : 1;
    u_int8_t  got_pwr_on : 1;
    u_int8_t  got_poll_mode : 1;
    u_int8_t  got_intr_mode : 1;
    u_int8_t  got_reset : 1;
};

static const struct enum_desc 
{
    const u_int8_t * const name;
    const u_int8_t * const desc;
    u_int16_t value;
}
    enum_list[] =
{
  I4B_D_DRIVERS(I4B_DRIVERS_LLIST) { NULL, NULL, 0 }
};

static int
__atoi(const u_int8_t *str)
{
    const struct enum_desc *entry = &enum_list[0];

    while(entry->name)
    {
        if(strcmp(entry->name,str) == 0)
	{
	    return entry->value;
	}
	entry++;
    }
    err(1, "invalid enum, \"%s\"!", str);
    return(atoi(str));
}

static const char *
get_layer1_desc(u_int16_t value)
{
    static const char * 
      MAKE_TABLE(L1_TYPES, DEFAULT_DRIVER_DESC, [N_L1_TYPES]);

    return (value < N_L1_TYPES) ? 
      L1_TYPES_DEFAULT_DRIVER_DESC[value] : "unknown";
}

static const char *
get_layer2_enum(u_int16_t value)
{
    const struct enum_desc *entry = &enum_list[0];

    while(entry->name)
    {
        if(entry->value == value)
	{
	  return entry->name;
	}
	entry++;
    }
    return "unknown";
}

static const char *
get_layer2_desc(u_int16_t value)
{
    const struct enum_desc *entry = &enum_list[0];

    while(entry->name)
    {
        if(entry->value == value)
	{
	  return entry->desc;
	}
	entry++;
    }
    return "unknown";
}

/*---------------------------------------------------------------------------*
 *	usage - display usage and exit
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
    fprintf
      (stdout,
       """isdnconfig - configure ISDN4BSD, version %d.%d.%d, compiled %s %s"
       "\nusage:  isdnconfig -u <unit> -i <number> -p <protocol> [parameters]" 
       "\n	-u unit         set controller unit (default is zero)"
       "\n	-p enum         set D-channel protocol"
       "\n	-i value        set D-channel serial number"
       "\n	-a              activate PH-line"
       "\n	-D              deactivate PH-line"
       "\n	-t              set TE-mode"
       "\n	-n              set NT-mode"
       "\n	-r              reset device"
       "\n	-E enum         display information about enum"
       "\n"
       "\nenums: "
       "\n" I4B_D_DRIVERS(I4B_DRIVERS_DLIST)
       "\n",
       I4B_VERSION, I4B_REL, I4B_STEP, __DATE__, __TIME__);

    exit(1);
    return;
}


/*---------------------------------------------------------------------------*
 *	reset_options - reset options
 *---------------------------------------------------------------------------*/
static void
reset_options(struct options *opt)
{
    bzero(opt, sizeof(*opt));
    opt->serial = 0xAB01;
    return;
}

/*---------------------------------------------------------------------------*
 *	i4b_ioctl - do a debug command
 *---------------------------------------------------------------------------*/
static void
i4b_ioctl(int cmdr, const char *msg, void *arg)
{
    if(ioctl(isdnfd, cmdr, arg) < 0)
    {
        err(1, "parameter '%s' failed! "
	    "(maybe it is not supported)", msg);
    }
    return;
}

/*---------------------------------------------------------------------------*
 *	flush_command - flush a command
 *---------------------------------------------------------------------------*/
static void
flush_command(struct options *opt)
{
    i4b_debug_t dbg = { /* zero */ };

    /* first check for contradictions */

    if(opt->got_nt_mode && 
       opt->got_te_mode)
    {
        err(1, "cannot specify 'nt_mode' and 'te_mode' "
	    "at the same time!");
    }

    if(opt->got_hi_pri &&
       opt->got_lo_pri)
    {
        err(1, "cannot specify 'hi_pri' and 'lo_pri' "
	    "at the same time!");
    }

    if(opt->got_up &&
       opt->got_down)
    {
        err(1, "cannot specify 'up' and 'down' "
	    "at the same time!");
    }

    if(opt->got_pwr_save &&
       opt->got_pwr_on)
    {
        err(1, "cannot specify 'pwr_save' and 'pwr_on' "
	    "at the same time!");
    }

    if(opt->got_poll_mode &&
       opt->got_intr_mode)
    {
        err(1, "cannot specify 'poll_mode' and 'intr_mode' "
	    "at the same time!");
    }

    if(opt->got_i &&
       (opt->got_p == 0))
    {
        err(1, "'-i' option requires '-p' option!");
    }

    /* execute commands */

    if(opt->got_p)
    {
        msg_prot_ind_t mpi = { /* zero */ };

	mpi.serial_number = 
	  opt->got_i ? opt->serial : (opt->unit + 0xAB01);

	mpi.driver_type = opt->driver_type;

	mpi.controller = opt->unit;

	if(ioctl(isdnfd, I4B_PROT_IND, &mpi) < 0)
	{
            err(1, "ioctl I4B_PROT_IND failed!");
	}
    }

    dbg.unit = opt->unit;

    if(opt->got_poll_mode) {
      i4b_ioctl(I4B_CTL_SET_POLLED_MODE, "poll_mode", &dbg);
    }

    if(opt->got_intr_mode) {
      i4b_ioctl(I4B_CTL_SET_STANDARD_MODE, "intr_mode", &dbg);
    }

    if(opt->got_reset) {
      i4b_ioctl(I4B_CTL_RESET, "reset", &dbg);
    }

    if(opt->got_te_mode) {
      i4b_ioctl(I4B_CTL_SET_TE_MODE, "te_mode", &dbg);
    }

    if(opt->got_nt_mode) {
      i4b_ioctl(I4B_CTL_SET_NT_MODE, "nt_mode", &dbg);
    }

    if(opt->got_up) {
      i4b_ioctl(I4B_CTL_PH_ACTIVATE, "up", &dbg);
    }

    if(opt->got_down) {
      i4b_ioctl(I4B_CTL_PH_DEACTIVATE, "down", &dbg);
    }

    if(opt->got_hi_pri) {
      i4b_ioctl(I4B_CTL_SET_HI_PRIORITY, "hi_pri", &dbg);
    }

    if(opt->got_lo_pri) {
      i4b_ioctl(I4B_CTL_SET_LO_PRIORITY, "lo_pri", &dbg);
    }

    if(opt->got_pwr_save) {
      i4b_ioctl(I4B_CTL_SET_POWER_SAVE, "pwr_save", &dbg);
    }

    if(opt->got_pwr_on) {
      i4b_ioctl(I4B_CTL_SET_POWER_ON, "pwr_on", &dbg);
    }

    reset_options(opt);
    return;
}

/*---------------------------------------------------------------------------*
 *	controller_info - print information about a controller
 *---------------------------------------------------------------------------*/
static void
controller_info(u_int32_t controller)
{
    msg_ctrl_info_req_t mcir = { /* zero */ };

    mcir.controller = controller;

    if(ioctl(isdnfd, I4B_CTRL_INFO_REQ, &mcir) >= 0)
    {
        printf("controller %d = {\n", controller);
	printf("  Layer 1:\n");
	printf("    description : %s\n", 
	       mcir.l1_desc);
 	printf("    type        : %s\n", 
	       get_layer1_desc(mcir.l1_type));
	printf("    channels    : 0x%x\n", 
	       mcir.l1_channels);
	printf("    serial      : 0x%04x\n", 
	       mcir.l1_serial);
	printf("    power_save  : %s\n", 
	       mcir.l1_no_power_save ? "off" : "on");
	printf("    dialtone    : %s\n",
	       mcir.l1_no_dialtone ? "disabled" : "enabled");
	printf("    attached    : %s\n",
	       mcir.l1_attached ? "yes" : "no");
	printf("    PH-state    : %s\n",
	       mcir.l1_state[0] ? 
	       (const char *)(mcir.l1_state) : 
	       (const char *)("unknown"));
	printf("  Layer 2:\n");
	printf("    driver_type : %s\n",
	       get_layer2_enum(mcir.l2_driver_type));
	printf("}\n");
    }
    return;
}

/*---------------------------------------------------------------------------*
 *	program entry
 *---------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
    register int c;
    msg_vr_req_t mvr = { /* zero */ };
    struct options *opt = alloca(sizeof(*opt));

    reset_options(opt);

    if(argc < 1)
    {
        usage();
    }

    if((isdnfd = open(CAPI_DEVICE, O_RDWR)) < 0)
    {
        err(1, "cannot open \"%s\"!", CAPI_DEVICE);
    }

    /* check I4B kernel version */
        
    if(ioctl(isdnfd, I4B_VR_REQ, &mvr) < 0)
    {
        warn("Could not get I4B version from kernel! "
	     "This program might have to be recompiled.");
    }
    else
    {
        if((mvr.version != I4B_VERSION) ||
	   (mvr.release != I4B_REL) ||
	   (mvr.step != I4B_STEP))
	{
	    warn("Version mismatch: kernel:%d.%d.%d != software:%d.%d.%d. "
		 "This program might have to be recompiled.",
		 mvr.version, mvr.release, mvr.step, 
		 I4B_VERSION, I4B_REL, I4B_STEP);
	}
    }

    for(optind = 1; optind < argc; )
    {
        c = getopt(argc, argv, "hu:E:p:ntraD");

        switch(c) {
	case 'u':
	    flush_command(opt);
	    opt->unit = atoi(optarg);
	    opt->got_u = 1;
	    break;
	case 'i':
	    opt->serial = atoi(optarg);
	    opt->got_i = 1;
	    opt->got_any = 1;
	    break;
	case 'p':
	    opt->driver_type = __atoi(optarg);
	    opt->got_p = 1;
	    opt->got_any = 1;
	    break;
	case 'a':
	    opt->got_up = 1;
	    opt->got_any = 1;
	    break;
	case 'D':
	    opt->got_down = 1;
	    opt->got_any = 1;
	    break;
	case 'r':
	    opt->got_reset = 1;
	    opt->got_any = 1;
	    break;
	case 'n':
	    opt->got_nt_mode = 1;
	    opt->got_any = 1;
	    break;
	case 't':
	    opt->got_te_mode = 1;
	    opt->got_any = 1;
	    break;
	case 'E':
	    c = __atoi(optarg);
	    printf("enum %s = {\n"
		   "  value       : 0x%02x\n"
		   "  description : %s\n"
		   "}\n",
		   optarg, c, get_layer2_desc(c));
	    opt->got_any = 1;
	    break;
	case (-1):
	    break;
	default:
	    usage();
	    break;
	}

	if((optind < argc) && (c == -1)) {

	  const char *ptr = argv[optind];

	  if(strcmp(ptr, "nt_mode") == 0) {
	    opt->got_nt_mode = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "te_mode") == 0) {
	    opt->got_te_mode = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "hi_pri") == 0) {
	    opt->got_hi_pri = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "lo_pri") == 0) {
	    opt->got_lo_pri = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "up") == 0) {
	    opt->got_up = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "down") == 0) {
	    opt->got_down = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "pwr_save") == 0) {
	    opt->got_pwr_save = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "pwr_on") == 0) {
	    opt->got_pwr_on = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "poll_mode") == 0) {
	    opt->got_poll_mode = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "intr_mode") == 0) {
	    opt->got_intr_mode = 1;
	    opt->got_any = 1;
	  } else if(strcmp(ptr, "reset") == 0) {
	    opt->got_reset = 1;
	    opt->got_any = 1;
	  } else {
	    err(1, "unrecognized parameter "
		"'%s'!", ptr);
	  }
	  optind++;
	}
    }

    if(opt->got_any == 0) {
      if(opt->got_u) {
	  controller_info(opt->unit);
      } else {

	  /* display info about all controllers */

	  for(c = 0; c < mvr.max_controllers; c++) {
	      controller_info(c);
	  }
      }
    } else {
      flush_command(opt);
    }
    return 0;
}

