/*-
 * Copyright (c) 1996, 2000 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 1996 Gary Jennejohn.  All rights reserved.
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
 *---------------------------------------------------------------------------
 *
 *	trace.h - header file for isdn trace
 *	------------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdntrace/trace.h,v 1.7 2000/10/09 14:22:49 hm Exp $
 *
 *---------------------------------------------------------------------------*/

#ifndef _TRACE_H_
#define _TRACE_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <err.h>
#include <poll.h>
#include <errno.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/param.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_trace.h>

#define I4BTRC_DEVICE		"/dev/i4btrc"	/* trace device file */
#define TRACE_FILE_NAME		"isdntrace"	/* default output filename */
#define BIN_FILE_NAME		"isdntracebin"	/* default binary filename */

#define BSIZE	4096	/* read buffer size */
#define NCOLS	80	/* screen width	*/

#define RxUDEF	0	/* analyze mode, default unit for receiver side */
#define TxUDEF	1	/* analyze mode, default unit for transmitter side */

struct buffer {
  u_int8_t *start;      /* inclusive */
  u_int16_t offset;     /* offset from start */
  u_int16_t len;        /* length from start */
};

#endif /* _TRACE_H_ */
