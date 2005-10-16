/*-
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
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
 *	main.c - configure ISDN4BSD
 *	---------------------------
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

#define I4B_DEVICE            "/dev/i4b"
#define MACRO_0(enum,args...) { #enum, enum },
#define MACRO_1(enum,args...) #enum ", "

static int isdnfd;
static msg_prot_ind_t mpi = { /* zero */ };
static u_int16_t unit = 0;
static u_int32_t serial = 0xAB01;
static u_int8_t i_opt;
static u_int8_t p_opt;

static const struct enum_desc
{
    const u_int8_t * const name;
    u_int16_t value;
}
	enum_list[] =
{
    I4B_D_DRIVERS(MACRO_0)
    { NULL, 0 }
};

static int
__atoi(u_int8_t *str)
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

/*---------------------------------------------------------------------------*
 *	display usage and exit
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
    fprintf
      (stdout,
       """isdnconfig - configure ISDN4BSD, version %d.%d.%d, compiled %s %s"
       "\nusage:  isdnconfig -u <unit> -i <number> -p <protocol>" 
       "\n	-u unit         set controller unit (default is zero)"
       "\n	-p enum         set D-channel protocol"
       "\n	-i value        set D-channel serial number"
       "\n	-E enum         get value of enum"
       "\n"
       "\nenums: "
       I4B_D_DRIVERS(MACRO_1)
       "\n",
       I4B_VERSION, I4B_REL, I4B_STEP, __DATE__, __TIME__);

    exit(1);
    return;
}

/*---------------------------------------------------------------------------*
 *	command flusher
 *---------------------------------------------------------------------------*/
static void
flush_command()
{
    if(i_opt || p_opt)
    {
        if(i_opt == 0)
	{
	    mpi.serial_number = mpi.controller + 0xAB01;
	}

	if(p_opt == 0)
	{
	    err(1, "need to specify a protocol!");
	}

	if(ioctl(isdnfd, I4B_PROT_IND, &mpi) < 0)
	{
            err(1, "ioctl I4B_PROT_IND failed!");
	}
    }

    bzero(&mpi, sizeof(mpi));
    i_opt = 0;
    p_opt = 0;
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

    if((isdnfd = open(I4B_DEVICE, O_RDWR)) < 0)
    {
        err(1, "cannot open \"%s\"!", I4B_DEVICE);
    }

    while((c = getopt(argc, argv, "u:E:p:")) != -1)
    {
        switch(c) {
	case 'u':
	    flush_command();
	    mpi.controller = atoi(optarg);
	    break;
	case 'i':
	    mpi.serial_number = atoi(optarg);
	    i_opt = 1;
	    break;
	case 'p':
	    mpi.driver_type = __atoi(optarg);
	    p_opt = 1;
	    break;
	case 'E':
	    printf("enum \"%s\" has value 0x%02x\n",
		   optarg, __atoi(optarg));
	    break;

	case '?':
	default:
	    usage();
	    break;

	}
    }

    flush_command();

    return 0;
}

