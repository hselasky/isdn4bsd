/*-
 * Copyright (c) 1997, 2000 Hellmuth Michaelis. All rights reserved.
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
 *	main.c - isdndecode main program file
 *	-------------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdndecode/main.c,v 1.9 2000/10/09 14:22:41 hm Exp $
 *
 *---------------------------------------------------------------------------*/

#include "decode.h"

static FILE *    Fout = NULL;
static FILE *    BP = NULL;
static u_int16_t u_Rx = RxUDEF;
static u_int16_t u_Tx = TxUDEF;
static u_int16_t unit = 0;

static u_int8_t  outflag = 1;
static u_int8_t  header = 1;
static u_int8_t  print_q921 = 1;
static u_int8_t  traceon = 0;
static u_int8_t  analyze = 0;
static u_int8_t  Bopt = 0;
static u_int8_t  Popt = 0;
static u_int8_t  bpopt = 0;
static u_int8_t  info = 0;
static u_int8_t  xflag = 0;
static u_int8_t  ropt = 0;
static u_int8_t  outfileset = 0;
static u_int8_t  npoll = 0;

static u_int32_t enable_trace = TRACE_D_RX | TRACE_D_TX;

static u_int8_t *outfile = DECODE_FILE_NAME;
static u_int8_t *binfile = BIN_FILE_NAME;

static int f_Rx;
static int f_Tx;

static u_int8_t  outfilename[MAXPATHLEN];
static u_int8_t  BPfilename[MAXPATHLEN];
static u_int8_t  tempbuffer[BSIZE];

static void dumpbuf(i4b_trace_hdr_t *hdr, void *pframe, u_int16_t len);
static int  switch_driver(int value);
static int  read_driver(void *buffer, u_int32_t len);
static void reopenfiles( int );

/*---------------------------------------------------------------------------*
 *	a safe way to read a byte
 *---------------------------------------------------------------------------*/
u_int8_t
get_1(struct buffer *src, u_int16_t offset)
{
    offset += src->offset;
    return
      ((offset < src->offset) || 
       (offset >= src->len)) ? 0 : src->start[offset];
}

/*---------------------------------------------------------------------------*
 *	check if a offset is valid
 *
 * returns 1 if valid else 0
 *---------------------------------------------------------------------------*/
u_int8_t
get_valid(struct buffer *src, u_int16_t offset)
{
    offset += src->offset;
    return ((offset >= src->offset) &&
	    (offset < src->len));
}

/*---------------------------------------------------------------------------*
 *	set new length for a buffer
 *
 * returns the old length
 *---------------------------------------------------------------------------*/
u_int16_t
set_length(struct buffer *src, u_int16_t new_len)
{
    u_int16_t old_len = src->len;

    if(new_len < old_len)
    {
        src->len = new_len;
    }
    return old_len;
}

/*---------------------------------------------------------------------------*
 *	initialize a buffer
 *---------------------------------------------------------------------------*/
void
buf_init(struct buffer *dst, void *start, u_int16_t len)
{
    dst->start = start;
    dst->len = len;
    dst->offset = 0;
    return;
}

/*---------------------------------------------------------------------------*
 *	usage intructions
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
  fprintf
    (stderr,
     "\n""isdndecode - ISDN4BSD package ISDN decoder for passive cards, v%d.%d.%d"
     "\n""usage: isdndecode -a -R <unit> -T <unit> -b -d -h -i -l -x -r -o -u <unit>"
     "\n""                  -B -P -p <file> -f <file>"
     "\n""                                                                         default"
     "\n""  -a        toggle analyzer mode .......................................... off"
     "\n""  -R <unit> analyze Rx controller unit number ............................. %d"
     "\n""  -T <unit> analyze Tx controller unit number ............................. %d"
     "\n""  -b        toggle B channel tracing  ..................................... off"
     "\n""  -d        toggle D channel tracing ...................................... on"
     "\n""  -h        toggle printing of message header ............................. on"
     "\n""  -i        toggle printing of layer 1 INFO signals ....................... off"
     "\n""  -l        toggle printing of layer 2 messages (D-channel) ............... on"
     "\n""  -x        toggle printing of unknown layer 2 messages (D-channel) ....... off"
     "\n""  -r        toggle printing of raw D-channel dump ......................... off"
     "\n""  -o        toggle writing of output to a file ............................ on"
     "\n""  -f <file> write output to <file> ............................. " DECODE_FILE_NAME "-XXX"
     "\n""  -u <unit> specify controller unit number ................................ 0"
     "\n""  -B        toggle writing of binary trace data to file ................... off"
     "\n""  -P        toggle playback of binary trace data from a file .............. off"
     "\n""  -p <file> specify filename for -B and -P options .......... " BIN_FILE_NAME "-XXX"
     "\n"
     "\n",
     I4B_VERSION, I4B_REL, I4B_STEP, RxUDEF, TxUDEF);
  exit(1);
}

/*---------------------------------------------------------------------------*
 *	exit handler function to be called at program exit
 *---------------------------------------------------------------------------*/
static void
exit_hdl()
{
	if(traceon)
		switch_driver(TRACE_OFF);
	return;
}

/*---------------------------------------------------------------------------*
 *	main
 *---------------------------------------------------------------------------*/
int
main(int argc, char *argv[])
{	
	i4b_trace_hdr_t *ithp = NULL;
	time_t tm;
	int n;
	int c;

	while((c = getopt(argc, argv, "abdf:hiln:op:u:xBPrR:T:")) != -1)
	{
	    switch(c) {
	    case 'a':
	        analyze = 1;
		break;
				
	    case 'b':
	        enable_trace |= (TRACE_B_RX | TRACE_B_TX);
		break;

	    case 'd':
	        enable_trace &= ~(TRACE_D_TX | TRACE_D_RX);
		break;

	    case 'o':
	        outflag = 0;
		break;

	    case 'f':
	        outfile = optarg;
		outfileset = 1;
		break;
		        
	    case 'h':
	        header = 0;
		break;

	    case 'i':
	        enable_trace |= TRACE_I;
		info = 1;
		break;

	    case 'l':
	        print_q921 = 0;
		break;

	    case 'p':
	        binfile = optarg;
		bpopt = 1;
		break;
		        
	    case 'u':
	        unit = atoi(optarg);
#if 0
		if(unit >= MAX_CONTROLLERS)
		    usage();
#endif
		break;

	    case 'x':
	        xflag = 1;
		break;

	    case 'B':
	        Bopt = 1;
		break;
		        
	    case 'P':
	        Popt = 1;
		break;

	    case 'r':
	        ropt = 1;
		break;

	    case 'R':
	        u_Rx = atoi(optarg);
#if 0
		if(u_Rx >= MAX_CONTROLLERS)
		    usage();
#endif
		break;

	    case 'T':
	        u_Tx = atoi(optarg);
#if 0
		if(Tx >= MAX_CONTROLLERS)
		    usage();
#endif
		break;

	    case '?':
	    default:
	        usage();
		break;
	    }
	}

	if(enable_trace == 0)
		usage();

	if(Bopt && Popt)
		usage();
	        
	atexit(&exit_hdl);

	if(Bopt)
	{
		if(bpopt)
		    snprintf(&BPfilename[0], sizeof(BPfilename), 
			     "%s", binfile);
		else
		    snprintf(&BPfilename[0], sizeof(BPfilename), 
			     "%s%d", BIN_FILE_NAME, unit);
		        
		if((BP = fopen(&BPfilename[0], "r")) != NULL)
		{
			fclose(BP);
			snprintf(&tempbuffer[0], sizeof(tempbuffer), "%s%s", 
				 &BPfilename[0], DECODE_FILE_NAME_BAK); 
			rename(&BPfilename[0], &tempbuffer[0]);
		}		        
		if((BP = fopen(&BPfilename[0], "w")) == NULL)
		{
			err(1, "Error opening file [%s]", &BPfilename[0]);
		}
	        
		if((setvbuf(BP, NULL, _IONBF, 0)) != 0)
		{
			err(1, "Error setting file [%s] to unbuffered", 
			    &BPfilename[0]);
		}
	}	        

	if(Popt)
	{
		if(bpopt)
		    snprintf(&BPfilename[0], sizeof(BPfilename), "%s", 
			     binfile);
		else
		    snprintf(&BPfilename[0], sizeof(BPfilename), "%s%d", 
			     BIN_FILE_NAME, unit);
		        
		if((BP = fopen(&BPfilename[0], "r")) == NULL)
		{
			err(1, "Error opening file [%s]", 
			    &BPfilename[0]);
		}
	}
        
	if(outflag)
	{
		if(outfileset == 0)
		    snprintf(&outfilename[0], sizeof(outfilename), "%s%d", 
			     DECODE_FILE_NAME, unit);
		else
		    snprintf(&outfilename[0], sizeof(outfilename), "%s", 
			     outfile);
		        
		if((Fout = fopen(&outfilename[0], "r")) != NULL)
		{
			fclose(Fout);
			snprintf(&tempbuffer[0], sizeof(tempbuffer), "%s%s", 
				 &outfilename[0], DECODE_FILE_NAME_BAK);
			rename(&outfilename[0], &tempbuffer[0]);
		}
		        
		if((Fout = fopen(&outfilename[0], "w")) == NULL)
		{
			err(1, "Error opening file [%s]", &outfilename[0]);
		}
	        
		if((setvbuf(Fout, NULL, _IONBF, 0)) != 0)
		{
			err(1, "Error setting file [%s] to unbuffered",
			    outfile);
		}
	}

	if((setvbuf(stdout, NULL, _IOLBF, 0)) != 0)
	{
		err(1, "Error setting stdout to line-buffered");
	}

	if(!Popt)
	{
		if(switch_driver(enable_trace))
			exit(1);
		else
			traceon = 1;
	}
	        
	signal(SIGHUP, SIG_IGN);	/* ignore hangup signal */
	signal(SIGUSR1, reopenfiles);	/* rotate logfile(s)	*/      

	time(&tm);
        
	if(analyze)
	{
	    snprintf(&tempbuffer[0], sizeof(tempbuffer), "\n==== isdnanalyze controller rx #%d - tx #%d ==== started %s",
		     u_Rx, u_Tx, ctime(&tm));
	}
	else
	{
	    snprintf(&tempbuffer[0], sizeof(tempbuffer), "\n=========== isdndecode controller #%d ========== started %s",
		     unit, ctime(&tm));
	}
        
	printf("%s", &tempbuffer[0]);

	if(outflag)
		fprintf(Fout, "%s", &tempbuffer[0]);

	for (;;)
	{
		if(Popt == 0)
		{
			n = read_driver(&tempbuffer[0], sizeof(tempbuffer));

			if(n < sizeof(i4b_trace_hdr_t))
			{
			    err(1, "Invalid trace length, %d bytes!", n);
			}

			if(Bopt)
			{
			    if(fwrite(&tempbuffer[0], 1, n, BP) != n)
			    {
			        err(1, "Error writing file [%s]", 
				    &BPfilename[0]);
			    }
			}

			n -= sizeof(i4b_trace_hdr_t);
		}
		else
		{
			n = sizeof(i4b_trace_hdr_t);

			if(fread(&tempbuffer[0], 1, n, BP) != n)
			{
			    if(feof(BP))
			    {
			        printf("\nEnd of playback input file reached.\n");
				exit(0);
			    }
			    else
			    {
			        err(1, "Error reading hdr from file [%s]", 
				    &BPfilename[0]);
			    }
			}

			ithp = (void *)&tempbuffer[0];
			n = ithp->length - sizeof(i4b_trace_hdr_t);

			if((n < 0) || (n > (sizeof(tempbuffer) - sizeof(i4b_trace_hdr_t))))
			{
			    err(1, "Invalid trace data length in header, %d bytes!", n);
			}
		        
			if(fread(&tempbuffer[sizeof(i4b_trace_hdr_t)], 1, n, BP) != n)
			{
			    err(1, "Error reading data from file [%s]", 
				&BPfilename[0]);
			}
		}
		dumpbuf((void *)&tempbuffer[0], &tempbuffer[sizeof(i4b_trace_hdr_t)], n);
	}
	return 0;
}

/*---------------------------------------------------------------------------*
 *	generate header
 *---------------------------------------------------------------------------*/
static void
fmt_hdr(struct buffer *dst, i4b_trace_hdr_t *hdr, u_int16_t len)
{
	struct tm *s;
	u_int16_t i = dst->offset;

	s = localtime((time_t *)&(hdr->time.tv_sec));

	if(hdr->type == TRC_CH_I)		/* Layer 1 INFO's */
	{
	    bsprintf(dst, "\n-- %s - unit:%02d --------------- "
		     "time:%2.2d.%2.2d %2.2d:%2.2d:%2.2d.%06u ",
		     ((hdr->dir) ? "NT->TE" : "TE->NT"),
		     hdr->unit,
		     s->tm_mday,
		     s->tm_mon + 1,
		     s->tm_hour,
		     s->tm_min,
		     s->tm_sec,
		     (u_int32_t)hdr->time.tv_usec);
	}
	else
	{
	    if(hdr->trunc > 0)
	    {
	        bsprintf(dst, "\n-- %s - unit:%02d  frame:%6.6u - "
			 "time:%2.2d.%2.2d %2.2d:%2.2d:%2.2d.%06u - "
			 "length:%d %struncated: (%d) ",
			 ((hdr->dir) ? "NT->TE" : "TE->NT"),
			 hdr->unit,
			 hdr->count,
			 s->tm_mday,
			 s->tm_mon + 1,
			 s->tm_hour,
			 s->tm_min,
			 s->tm_sec,
			 (u_int32_t)hdr->time.tv_usec,
			 len,
			 (NCOLS < 90) ? "\n-- " : "- ",
			 hdr->trunc);
	    }
	    else
	    {
	        bsprintf(dst, "\n-- %s - unit:%02d  frame:%6.6u - "
			 "time:%2.2d.%2.2d %2.2d:%2.2d:%2.2d.%06u - length:%d ",
			 ((hdr->dir) ? "NT->TE" : "TE->NT"),
			 hdr->unit,
			 hdr->count,
			 s->tm_mday,
			 s->tm_mon + 1,
			 s->tm_hour,
			 s->tm_min,
			 s->tm_sec,
			 (u_int32_t)hdr->time.tv_usec,
			 len);
	    }
	}

	for(i = (dst->offset - i) % NCOLS; i < NCOLS; i++)
	{
	    bsprintf(dst, "-");
	}

	bsprintf(dst, "\n");
	return;
}

/*---------------------------------------------------------------------------*
 *	decode protocol and output to file(s)
 *---------------------------------------------------------------------------*/
static void
dumpbuf(i4b_trace_hdr_t *hdr, void *pframe, u_int16_t len)
{
	u_int8_t buffer[65000]; /* must be less than 65536 */
	struct buffer dst;
	struct buffer src;
	u_int16_t h;
	u_int16_t i;
	u_int8_t j;

	/* initialize buffers */

	buf_init(&dst, &buffer[0], sizeof(buffer));
	buffer[0] = '\0';

	buf_init(&src, pframe, len);

	if(header)
	{
		fmt_hdr(&dst, hdr, len);
	}

	h = dst.offset; /* store header offset */

	switch(hdr->type) {
	case TRC_CH_I:		/* Layer 1 INFO's */
	    if(enable_trace & TRACE_I)
	    {
	        layer1(&dst, &src);
	    }
	    break;
		        
	case TRC_CH_D:		/* D-channel data */

	    if(ropt)
	    {
	        dump_raw(&dst, &src, "Layer2/Layer3");
	    }

	    h = dst.offset; /* update header offset */

	    layer2(&dst, &src, hdr->dir);

	    if(print_q921 == 0)
	    {
	        dst.offset = h;
		dst.start[dst.offset] = '\0';
	    }

	    if(!layer3_dss1(&dst, &src))
	      break;
	    if(!layer3_1tr6(&dst, &src))
	      break;

	    if(xflag == 0)
	    {
	        dst.offset = h;
		dst.start[dst.offset] = '\0';
		break;
	    }

	    if(!layer3_unknown(&dst, &src))
	      break;

	    break;

	default:	/* B-channel data */
	
	    for (i = 0; get_valid(&src,i); i += 16)
	    {
	        bsprintf(&dst, "B%02d:%03x  ", 
			 hdr->type-TRC_CH_B1+1, i);
	
		do {
		    if (get_valid(&src, i))
		      bsprintf(&dst, "%02x ", get_1(&src, i));
		    else
		      bsprintf(&dst,"   ");
	
		    i++;

		    if((i & 15) == 8)
		    {
		        bsprintf(&dst, "   ");
		    }

		} while(i & 15);

		i -= 16;

		bsprintf(&dst, "   ");

		do {
		    j = get_1(&src, i);

		    bsprintf(&dst, "%c", isprint(j) ? j : '.');

		    i++;
		} while(i & 15);

		i -= 16;

		bsprintf(&dst,"\n");
	    }
	    break;
	}

	if(dst.offset != h)
	{
	    printf("%s", dst.start);

	    if(outflag)
	      fprintf(Fout, "%s", dst.start);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	switch driver debugging output on/off
 *---------------------------------------------------------------------------*/
static int
switch_driver(int value)
{
	int v = value;

	if(analyze == 0)
	{
	    u_Rx = unit;
	    u_Tx = unit;
	}

	if(f_Rx < 1)
	{
	    if(value == TRACE_OFF)
	    {
	        goto done;
	    }

	    snprintf(&tempbuffer[0], sizeof(tempbuffer), 
		     "%s%d", I4BTRC_DEVICE, u_Rx);
	
	    if((f_Rx = open(&tempbuffer[0], O_RDWR)) < 0)
	    {
	        warn("Error: opening trace device [%s]", 
		     &tempbuffer[0]);
		goto error;
	    }

	    /* set non-blocking operation */
	    if(ioctl(f_Rx, FIONBIO, "\1\1\1") < 0)
	    {
	        warn("Error: ioctl FIONBIO, val = 0x%08x", v);
		goto error;
	    }
	}

	if((f_Tx < 1) && (u_Rx != u_Tx))
	{
	    if(value == TRACE_OFF)
	    {
	        goto done;
	    }

	    snprintf(&tempbuffer[0], sizeof(tempbuffer), 
		     "%s%d", I4BTRC_DEVICE, u_Tx);
	
	    if((f_Tx = open(&tempbuffer[0], O_RDWR)) < 0)
	    {
	        warn("Error: opening trace device [%s]", 
		     &tempbuffer[0]);
		goto error;
	    }

	    /* set non-blocking operation */
	    if(ioctl(f_Tx, FIONBIO, "\1\1\1") < 0)
	    {
	        warn("Error: ioctl FIONBIO, val = 0x%08x", v);
		goto error;
	    }
	}

	if(f_Rx > 0)
	{
	    if(ioctl(f_Rx, I4B_TRC_SET, &v) < 0)
	    {
	        warn("Error: ioctl I4B_TRC_SET, val = 0x%08x", v);
		goto error;
	    }
	}

	if(f_Tx > 0)
	{
	    if(ioctl(f_Tx, I4B_TRC_SET, &v) < 0)
	    {
	        warn("Error: ioctl I4B_TRC_SET, val = 0x%08x", v);
		goto error;
	    }
	}
 done:
	npoll = (f_Rx > 0) ? ((f_Tx > 0) ? 2 : 1) : 0;
	return 0;
 error:
	return 1;
}

/*---------------------------------------------------------------------------*
 *	read trace data from driver
 *---------------------------------------------------------------------------*/
static int
read_driver(void *buffer, u_int32_t len)
{
    static u_int8_t last = 0;
    struct pollfd pollfd[2];
    u_int8_t count = npoll;
    int error;

    if(npoll == 0)
    {
        return -1;
    }

    while(1)
    {
        if(!count)
	{
	    count = npoll;

	    pollfd[0].fd = f_Rx;
	    pollfd[0].events = (POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI | 
				POLLERR | POLLHUP | POLLNVAL);
	    pollfd[0].revents = 0;

	    pollfd[1].fd = f_Tx;
	    pollfd[1].events = (POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI | 
				POLLERR | POLLHUP | POLLNVAL);
	    pollfd[1].revents = 0;

	    error = poll(&pollfd[0], npoll, -1);

	    if(error < 0)
	    {
	        break;
	    }
	}

        if(last == 0)
	{
	    error = read(f_Rx, buffer, len);

	    if((error < 0) && (errno == EWOULDBLOCK))
	    {
	        if(npoll > 1)
		{
		   last = 1;
		}
		count--;
		continue;
	    }
	}
	else
	{
	    error = read(f_Tx, buffer, len);

	    if((error < 0) && (errno == EWOULDBLOCK))
	    {
	        last = 0;
		count--;
		continue;
	    }
	}
	break;
    }

    if(analyze && (error >= sizeof(i4b_trace_hdr_t)))
    {
        i4b_trace_hdr_t *hdr = buffer;

	if(u_Rx != u_Tx)
	  hdr->dir = (hdr->unit == u_Rx) ? FROM_NT : FROM_TE;
	hdr->unit = unit;
    }
    return error;
}

/*---------------------------------------------------------------------------*
 *	reopen files to support rotating logfile(s) on SIGUSR1
 *
 *	based on an idea from Ripley (ripley@nostromo.in-berlin.de)
 *
 *	close file and reopen it for append. this will be a nop
 *	if the previously opened file hasn't moved but will open
 *	a new one otherwise, thus enabling a rotation...
 * 
 *---------------------------------------------------------------------------*/
static void
reopenfiles(int dummy)
{
	if(outflag)
	{
		fclose(Fout);

		if((Fout = fopen(&outfilename[0], "a")) == NULL)
		{
			err(1, "Error re-opening file [%s]", 
			    &outfilename[0]);
		}

		if((setvbuf(Fout, NULL, _IONBF, 0)) != 0)
		{
			err(1, "Error re-setting file [%s] to unbuffered", 
			    &outfilename[0]);
		}
	}

	if(Bopt)
	{
		fclose(BP);

		if((BP = fopen(&BPfilename[0], "a")) == NULL)
		{
			err(1, "Error re-opening file [%s]", 
			    &BPfilename[0]);
		}

		if((setvbuf(BP, NULL, _IONBF, 0)) != 0)
		{
			err(1, "Error re-setting file [%s] to unbuffered", 
			    &BPfilename[0]);
		}
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode extension bit
 *---------------------------------------------------------------------------*/
void
extension(int layer, struct buffer *dst, u_int16_t cnt, 
	  u_int8_t value, u_int8_t mask)
{
    bsprintline(layer, dst, cnt, value, mask, "Extension Bit = %c (%s)",
		(value & mask) ? '1' : '0',
		(value & mask) ? "no extension, final octet" : 
		"with extension, octet follows");
    return;
}

/*---------------------------------------------------------------------------*
 *	print bits as 0/1 available for mask
 *---------------------------------------------------------------------------*/
static const u_int8_t *
print_bits(u_int8_t val, u_int8_t mask)
{
	static u_int8_t buffer[10];
	u_int8_t i;

	for(i = 0; i < 8; i++)
	{
		if(mask & 0x80)
		{
			if(val & 0x80)
				buffer[i] = '1';
			else
				buffer[i] = '0';
		}
		else
		{
			buffer[i] = '-';
		}
		val <<= 1;
		mask <<= 1;
	}
	buffer[i] = '\0';
	return &buffer[0];
}

/*---------------------------------------------------------------------------*
 *	print text safely to a buffer
 *---------------------------------------------------------------------------*/
void
bsprintf(struct buffer *dst, const void *fmt, ...)
{
	int error;

	va_list ap;

	va_start(ap, fmt);

	error = vsnprintf(dst->start + dst->offset, 
			  dst->len - dst->offset, fmt, ap);

	if(error > 0)
	  dst->offset += error;

	if(dst->offset >= dst->len)
	  dst->offset = dst->len-1;

        va_end(ap);
	return;
}

/*---------------------------------------------------------------------------*
 *	print one decoded output line
 *---------------------------------------------------------------------------*/
void
bsprintline(u_int8_t layer, struct buffer *dst,
	    u_int16_t oct_count, u_int8_t oct_val, 
	    u_int8_t oct_mask, const void *fmt, ...)
{
	static u_int16_t lastcount = -1;
	u_int8_t *buffer = dst->start + dst->offset; /* inclusive */
	u_int8_t *buffer_end = dst->start + dst->len; /* exclusive */
	u_int8_t *col_end = buffer + (NCOLS+1); /* exlusive */
	u_int8_t *ptr;
	int       error;
	va_list   ap;

	if(oct_count != lastcount)
	{
	    lastcount = oct_count;

	    bsprintf(dst, "L%d %02X %02X %s ",
		     layer, oct_count, oct_val, 
		     print_bits(oct_val, oct_mask));
	}
	else
	{
	    bsprintf(dst, "         %s ",
		     print_bits(oct_val, oct_mask));
	}

	va_start(ap, fmt);

	error = vsnprintf(dst->start + dst->offset,
			  dst->len - dst->offset, fmt, ap);
	if(error > 0)
	  dst->offset += error;

	if(dst->offset >= dst->len)
	  dst->offset = dst->len-1;

        va_end(ap);

        bsprintf(dst, "\n");

	/* get current write pointer */

	ptr = dst->start + dst->offset;

	if((col_end > ptr) && 
	   (col_end <= buffer_end) && 
	   strstr(buffer, "("))
	{
	    col_end--; /* cannot write to end */

	    /* update offset */

	    dst->offset = col_end - dst->start;

	    /* right adjust some text */

	    while(1)
	    {
	       col_end[0] = ptr[0];
	       col_end--;

	       if(ptr[0] == '(')
	       {
		   /* fill space */

		   while(col_end >= ptr)
		   {
		       col_end[0] = ' ';
		       col_end--;
		   }
		   break;
	       }
	       ptr--;
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	dump data in hex and ascii to text buffer
 *---------------------------------------------------------------------------*/
void
dump_raw(struct buffer *dst, struct buffer *src, const u_int8_t *desc)
{
	u_int16_t i;
	u_int8_t j;

	bsprintf(dst, "Dumping %s data, %d bytes:\n",
		 desc, get_valid(src,0) ? src->len - src->offset : 0);

	for(i = 0; get_valid(src,i); i += 16)
	{
	    bsprintf(dst, "D1 %03x: ", 
		     src->offset + i);

	    do {
	        if(get_valid(src,i))
		  bsprintf(dst, " %02x", get_1(src,i));
		else
		  bsprintf(dst, "   ");

		i++;

		if((i & 15) == 8)
		{
		    bsprintf(dst, "   ");
		}

	    } while(i & 15);

	    i -= 16;

	    bsprintf(dst, "    ");

	    do {
	      j = get_1(src, i);
	      bsprintf(dst, "%c", isprint(j) ? j : '.');
	      i++;
	    } while(i & 15);

	    i -= 16;

	    bsprintf(dst, "\n");
	}
	return;
}
