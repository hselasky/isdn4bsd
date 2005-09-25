/*-
 * Copyright (c) 1997, 1999 Hellmuth Michaelis. All rights reserved.
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
 *	decode.h - isdndecode header file
 *	---------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdndecode/decode.h,v 1.7 2000/10/09 14:22:41 hm Exp $
 *
 *---------------------------------------------------------------------------*/

#ifndef _DECODE_H_
#define _DECODE_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <err.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/param.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_trace.h>

#include <dss1_l3.h>

#define I4BTRC_DEVICE		"/dev/i4btrc"	/* trace device file */
#define DECODE_FILE_NAME	"isdndecode"	/* default output filename */
#define DECODE_FILE_NAME_BAK	".last"		/* backup filename trailer */
#define BIN_FILE_NAME		"isdntracebin"	/* default binary filename */

#define BSIZE	4096	/* read buffer size */
#define NCOLS	80	/* screen width	*/

#define RxUDEF	0	/* analyze mode, default unit for receiver side */
#define TxUDEF	1	/* analyze mode, default unit for transmitter side */

struct buffer {
  u_int8_t *start;      /* inclusive */
  u_int16_t offset;     /* offset from start */
  u_int16_t len;        /* length from start */
  u_int8_t  state;      /* used by AOC */
};

struct ie {
  u_int8_t         code;  /* information element identifier code */
  const u_int8_t * name;  /* IE name */
  void           (*func)(struct buffer *, struct buffer *);
};

extern const struct ie Q931_INFORMATION_ELEMENTS_TABLE_0[];

extern u_int8_t  get_1(struct buffer *src, u_int16_t offset);
extern u_int8_t  get_valid(struct buffer *src, u_int16_t offset);
extern u_int16_t set_length(struct buffer *src, u_int16_t new_len);
extern void      buf_init(struct buffer *dst, void *start, u_int16_t len);

extern void layer1(struct buffer *dst, struct buffer *src);
extern void layer2(struct buffer *dst, struct buffer *src, u_int8_t dir);
extern u_int8_t layer3_dss1(   struct buffer *dst, struct buffer *src);
extern u_int8_t layer3_1tr6(   struct buffer *dst, struct buffer *src);
extern u_int8_t layer3_unknown(struct buffer *dst, struct buffer *src);

extern void extension(int layer, struct buffer *dst, u_int16_t cnt, 
		      u_int8_t value, u_int8_t mask);
extern void bsprintf(struct buffer *dst, const void *fmt, ...);
extern void bsprintline(u_int8_t layer, struct buffer *dst,
		       u_int16_t oct_count, u_int8_t oct_val, 
		       u_int8_t oct_mask, const void *fmt, ...);

extern void q932_facility(struct buffer *dst, struct buffer *src);
extern void dump_raw(struct buffer *dst, struct buffer *src, const u_int8_t *desc);

#endif /* _DECODE_H_ */
