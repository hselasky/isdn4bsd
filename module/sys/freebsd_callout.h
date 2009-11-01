/*-
 * Copyright (c) 2006 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/sys/callout.h"
 */
#ifndef __FREEBSD_CALLOUT_H__
#define	__FREEBSD_CALLOUT_H__

struct usb_callout {
	LIST_ENTRY(usb_callout) entry;
	struct mtx *mtx;
	void    (*func) (void *);
	void   *arg;
	u_int32_t flags;
	int	timeout;
};

#define	CALLOUT_RETURNUNLOCKED 0x0001

extern void usb_callout_init_mtx(struct usb_callout *c, struct mtx *mtx, u_int32_t flags);

extern void usb_callout_reset(struct usb_callout *c, u_int32_t to_ticks, void (*func) (void *), void *arg);
extern void usb_callout_stop(struct usb_callout *c);
extern void usb_callout_drain(struct usb_callout *c);
extern u_int8_t usb_callout_pending(struct usb_callout *c);
extern void usb_schedule(void);

#endif
