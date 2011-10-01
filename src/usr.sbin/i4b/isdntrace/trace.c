/*-
 * Copyright (c) 1996, 2000 Hellmuth Michaelis.  All rights reserved.
 *
 * Copyright (c) 1996 Gary Jennejohn.  All rights reserved. 
 *
 * Copyright (c) 2005 Hans Petter Selasky.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 4. Altered versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software and/or documentation.
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
 *---------------------------------------------------------------------------*
 *
 *	trace.c - print traces of D- or B-channel activity in ISDN4BSD
 *	--------------------------------------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdntrace/trace.c,v 1.9 2000/10/09 14:22:49 hm Exp $
 *
 *---------------------------------------------------------------------------*/

#include "trace.h"

static FILE *    Fout = NULL;
static FILE *    BP = NULL;

static u_int16_t unit = 0;
static u_int16_t u_Rx = RxUDEF;
static u_int16_t u_Tx = TxUDEF;

static u_int8_t  outflag = 1;
static u_int8_t  header = 1;
static u_int8_t  traceon = 0;
static u_int8_t  analyze = 0;
static u_int8_t  Bopt = 0;
static u_int8_t  Popt = 0;
static u_int8_t  bpopt = 0;
static u_int8_t  info = 0;
static u_int8_t  Fopt = 0;
static u_int8_t  outfileset = 0;
static const char *outfile = TRACE_FILE_NAME;
static const char *binfile = BIN_FILE_NAME;
static u_int8_t  once = 1;
static u_int8_t  npoll = 0;

static u_int32_t enable_trace = TRACE_D_RX | TRACE_D_TX;

static int f_Rx;
static int f_Tx;

static int min_size = 0;

static u_int8_t tempbuffer[BSIZE];
static u_int8_t outfilename[MAXPATHLEN];
static u_int8_t BPfilename[MAXPATHLEN];

static struct stat fst_old;
static struct stat fst_curr;	

static int  switch_driver(int value);
static int  read_driver(void *buffer, u_int32_t len);
static void reopenfiles(int dummy);
static void dump_trace(i4b_trace_hdr_t *hdr, void *pframe, u_int16_t len);
static void dump_data(struct buffer *dst, struct buffer *src, 
		      u_int8_t chan_id, u_int16_t chan_num);

/*---------------------------------------------------------------------------*
 *	usage instructions
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
  fprintf
    (stderr,
     "\n""isdntrace - ISDN4BSD package ISDN trace utility for passive cards, v%d.%d.%d"
     "\n""usage: isdntrace -a -R <unit> -T <unit> -b -d -h -i -o -f <file>"
     "\n""                 -u <unit> -n <val> -B -P -p <file> -F"
     "\n""                                                                         default"
     "\n""   -a        toggle analyzer mode ......................................... off"
     "\n""   -R <unit> specify analyze Rx controller unit number .................... %d"
     "\n""   -T <unit> specify analyze Tx controller unit number .................... %d"
     "\n""   -b        toggle B channel tracing ..................................... off"
     "\n""   -d        toggle D channel tracing ..................................... on"
     "\n""   -h        toggle printing of message header ............................ on"
     "\n""   -i        toggle printing of layer 1 INFO signals ...................... off"
     "\n""   -o        toggle writing of output to a file ........................... on"
     "\n""   -f <file> write output to <file> ............................. " TRACE_FILE_NAME "-XXX"
     "\n""   -u <unit> specify controller unit number ............................... 0"
     "\n""   -n <val>  set minimum packet size to <val> bytes ....................... 0"
     "\n""   -B        toggle writing of binary trace data to a file ................ off"
     "\n""   -P        toggle playback of binary trace data from a file ............. off"
     "\n""   -p <file> specify filename for -B and -P options .......... " BIN_FILE_NAME "-XXX"
     "\n""   -F        wait for more data at EOF, used with -P and -p options ....... off"
     "\n"
     "\n", I4B_VERSION, I4B_REL, I4B_STEP, RxUDEF, TxUDEF);
  exit(1);
}

/*---------------------------------------------------------------------------*
 *	exit handler function to be called at program exit
 *---------------------------------------------------------------------------*/
static void
exit_hdl(void)
{
	if(traceon)
	    switch_driver(TRACE_OFF);
	return;
}

/*---------------------------------------------------------------------------*
 *	a safe way to read a byte
 *---------------------------------------------------------------------------*/
static u_int8_t
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
static u_int8_t
get_valid(struct buffer *src, u_int16_t offset)
{
    offset += src->offset;
    return ((offset >= src->offset) &&
	    (offset < src->len));
}

/*---------------------------------------------------------------------------*
 *	initialize a buffer
 *---------------------------------------------------------------------------*/
static void
buf_init(struct buffer *dst, void *start, u_int16_t len)
{
    dst->start = start;
    dst->len = len;
    dst->offset = 0;
    return;
}

/*---------------------------------------------------------------------------*
 *	print text safely to a buffer
 *---------------------------------------------------------------------------*/
static void
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
 *	decode layer 1 information
 *---------------------------------------------------------------------------*/
static void
layer1(struct buffer *dst, struct buffer *src)
{
    u_int8_t j;
    u_int16_t i = 0;

    bsprintf(dst, "L1 STATE: ");

    while((j = get_1(src,i)))
    {
        bsprintf(dst, "%c", isprint(j) ? j : '?');
	i++;
    }

    bsprintf(dst, "\n");
    return;
}

/*---------------------------------------------------------------------------*
 *	dump data in hex and ascii
 *---------------------------------------------------------------------------*/
static void
dump_data(struct buffer *dst, struct buffer *src, u_int8_t chan_id, 
	  u_int16_t chan_num)
{
	u_int16_t i;
	u_int8_t j;

	for(i = 0; get_valid(src,i); i += 16)
	{
	    bsprintf(dst, "%c%02d:%03x  ",
		     chan_id, chan_num, i);

	    do {

	      if(get_valid(src,i))
		  bsprintf(dst, "%02x ", get_1(src,i));
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

	      j = get_1(src,i);

	      if(!isprint(j)) j = '.';

	      bsprintf(dst, "%c", j);

	      i++;
	    } while(i & 15);

	    i -= 16;

	    bsprintf(dst, "\n");
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	append time stamp and refcount to a filename
 *---------------------------------------------------------------------------*/
static u_int8_t 
add_datetime(u_int8_t *dst, u_int16_t len, u_int8_t refcount)
{
	const u_int8_t *dst_old = dst;
	time_t time_temp;
	struct tm *tm_temp;
	int off = strlen(dst);

	dst += off;
	len -= off;

	time(&time_temp);
	tm_temp = localtime(&time_temp);

	off = strftime(dst, len, "-%Y%m%d-%H%M%S", tm_temp);

	if(off == 0)
	{
	    err(1, "strftime returned error!");
	}

	dst += off;
	len -= off;

	snprintf(dst, len, "-%02x", refcount);

	/* check if file already exists */

	off = open(dst_old, O_RDONLY);

	if(off >= 0)
	{
	    close(off);
	    return 1;
	}
	return 0;
}

/*---------------------------------------------------------------------------*
 *	generate binary input/output file names
 *---------------------------------------------------------------------------*/
static void
get_filename_bin(u_int8_t *dst, u_int16_t len)
{
	u_int8_t count = 0;

 again:
	dst[0] = 0;

	if(bpopt)
	    snprintf(dst, len, "%s", binfile);
	else
	    snprintf(dst, len, "%s%d", BIN_FILE_NAME, unit);

	if(Bopt)
	{
	    if(add_datetime(dst, len, count))
	    {
	        if(count == 0xff)
		{
		    err(1, "%s: reference count exhaused: "
			"software is raceing!", __FUNCTION__);
		}
	        count++;
		goto again;
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	generate ASCII output filename
 *---------------------------------------------------------------------------*/
static void
get_filename_txt(u_int8_t *dst, u_int16_t len)
{
	u_int8_t count = 0;

 again:
	dst[0] = 0;

	if(outfileset == 0)
	    snprintf(dst, len, "%s%d", TRACE_FILE_NAME, unit);
	else
	    snprintf(dst, len, "%s", outfile);

	if(add_datetime(dst, len, count))
	{
	    if(count == 0xff)
	    {
	        err(1, "%s: reference count exhaused: "
		    "software is raceing!", __FUNCTION__);
	    }
	    count++;
	    goto again;
	}
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

	while((c = getopt(argc, argv, "abdf:hin:op:u:BFPR:T:")) != -1)
	{
	    switch(c) {
	    case 'a':
	        analyze = 1;
		break;
				
	    case 'b':
	        enable_trace |= (TRACE_B_RX | TRACE_B_TX);
		break;

	    case 'd':
	        enable_trace &= (~(TRACE_D_TX | TRACE_D_RX));
		break;

	    case 'o':
	        outflag = 0;
		break;

	    case 'f':
	        outfile = optarg;
		outfileset = 1;
		break;
			
	    case 'n':
	        min_size = atoi(optarg);
		break;

	    case 'h':
	        header = 0;
		break;

	    case 'i':
	        enable_trace |= TRACE_I;
		info = 1;
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

	    case 'B':
	        Bopt = 1;
		break;

	    case 'F':
	        Fopt = 1;
		break;

	    case 'P':
	        Popt = 1;
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
		if(u_Tx >= MAX_CONTROLLERS)
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
		get_filename_bin(&BPfilename[0], sizeof(BPfilename));

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
		get_filename_bin(&BPfilename[0], sizeof(BPfilename));

		if((BP = fopen(&BPfilename[0], "r")) == NULL)
		{
			err(1, "Error opening file [%s]", 
			    &BPfilename[0]);
		}
		if(Fopt)
		{
			if(fstat(fileno(BP), &fst_old))
			{
				err(1, "Error fstat file [%s]", 
				    &BPfilename[0]);
			}
		}
	}
	
	if(outflag)
	{
		get_filename_txt(&outfilename[0], sizeof(outfilename));

		if((Fout = fopen(&outfilename[0], "w")) == NULL)
		{
			err(1, "Error opening file [%s]", 
			    &outfilename[0]);
		}
		
		if((setvbuf(Fout, NULL, _IONBF, 0)) != 0)
		{
			err(1, "Error setting file [%s] to "
			    "unbuffered", &outfilename[0]);
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
	signal(SIGUSR1, &reopenfiles);	/* rotate logfile(s) */

	time(&tm);
	
	if(analyze)
	{
		snprintf(&tempbuffer[0], sizeof(tempbuffer), 
			 "\n==== isdnanalyze controller rx #%d - tx #%d ==== started %s",
			 u_Rx, u_Tx, ctime(&tm));
	}
	else
	{
		snprintf(&tempbuffer[0], sizeof(tempbuffer), 
			 "\n=========== isdntrace controller #%d =========== started %s",
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

			if(n < (int)sizeof(i4b_trace_hdr_t))
			{
			    err(1, "Invalid trace length, %d bytes!", n);
			}

			if(Bopt)
			{
			    if(fwrite(&tempbuffer[0], 1, n, BP) != (size_t)n)
			    {
			        err(1, "Error writing file [%s]", 
				    &BPfilename[0]);
			    }
			}

			n -= (int)sizeof(i4b_trace_hdr_t);			
		}
		else
		{
again:
			n = sizeof(i4b_trace_hdr_t);

			if(fread(&tempbuffer[0], 1, n, BP) != (size_t)n)
			{
			    if(feof(BP))
			    {
			        if(Fopt)
				{
				    if(ferror(BP))
				    {
				        err(1, "Error reading hdr "
					    "from file [%s]", 
					    &BPfilename[0]);
				    }

				    usleep(500000);

				    clearerr(BP);

				    if(stat(&BPfilename[0], &fst_curr) != -1)
				    {
				        /* check if the file has been deleted */

				        if((fst_curr.st_ino != fst_old.st_ino) ||
					   (fst_curr.st_nlink == 0))
					{
					    bcopy(&fst_curr, &fst_old, sizeof(fst_old));

					    if((BP = freopen(&BPfilename[0], "r", BP)) == NULL)
					    {
					        err(1, "Error reopening file [%s]", 
						    &BPfilename[0]);
					    }
					}
				    }
				    goto again;
				}
				else
				{
				    printf("\nEnd of playback input file is reached!\n");
				    exit(0);
				}
			    }
			    else
			    {
			        err(1, "Error reading hdr from file [%s]", 
				    &BPfilename[0]);
			    }
			}

			ithp = (void *)&tempbuffer[0];
			n = ithp->length - sizeof(i4b_trace_hdr_t);

			if((n < 0) || (n > (int)(sizeof(tempbuffer) - sizeof(i4b_trace_hdr_t))))
			{
			    err(1, "Invalid trace data length in header, %d bytes!", n);
			}

			if(fread(&tempbuffer[sizeof(i4b_trace_hdr_t)], 1, n, BP) != (size_t)n)
			{
			    err(1, "Error reading data from "
				"file [%s]", &BPfilename[0]);
			}
		}
		if(n > min_size)
		{
		    dump_trace((void *)&tempbuffer[0], &tempbuffer[sizeof(i4b_trace_hdr_t)], n);
		}
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
	    bsprintf(dst, "\n-- %s - unit:%02d --------------- time:%2.2d.%2.2d %2.2d:%2.2d:%2.2d.%06u ",
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
			 "time:%2.2d.%2.2d %2.2d:%2.2d:%2.2d.%06u - "
			 "length:%d ",
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
dump_trace(i4b_trace_hdr_t *hdr, void *pframe, u_int16_t len)
{
	u_int8_t buffer[65000]; /* must be less than 65536 */
	struct buffer dst;
	struct buffer src;
	u_int16_t h;

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
	    dump_data(&dst, &src, 'D', 1);
	    if(once)
	    {
	        once = 0;
		bsprintf(&dst, "D01 (\"isdndecode\" can decode the data above)\n");
	    }
	    break;

	default:	/* B-channel data */
	    dump_data(&dst, &src, 'B', hdr->type-TRC_CH_B1+1);
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

    if(analyze && (error >= (int)sizeof(i4b_trace_hdr_t)))
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
	(void)dummy;

	if(outflag)
	{
		fclose(Fout);

		get_filename_txt(&outfilename[0], sizeof(outfilename));

		if((Fout = fopen(&outfilename[0], "a")) == NULL)
		{
			err(1, "Error re-opening file [%s]", 
			    &outfilename[0]);
		}

		if((setvbuf(Fout, NULL, _IONBF, 0)) != 0)
		{
			err(1, "Error re-setting file [%s] "
			    "to unbuffered", &outfilename[0]);
		}
	}

	if(Bopt)
	{
		
		fclose(BP);

		get_filename_bin(&BPfilename[0], sizeof(BPfilename));
		
		if((BP = fopen(BPfilename, "a")) == NULL)
		{
			err(1, "Error re-opening file [%s]", 
			    &BPfilename[0]);
		}

		if((setvbuf(BP, NULL, _IONBF, 0)) != 0)
		{
			err(1, "Error re-setting file [%s] "
			    "to unbuffered", &BPfilename[0]);
		}
	}
	return;
}

