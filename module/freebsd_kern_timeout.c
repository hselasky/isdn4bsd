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
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/kern/kern_timeout.c"
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kthread.h>

#include <sys/freebsd_compat.h>

static volatile struct usb_callout *curr_callout;
static volatile int curr_cancel;
static LIST_HEAD(, usb_callout) head_callout = LIST_HEAD_INITIALIZER(&head_callout);

static struct mtx mtx_callout;

static int usb_callout_process(void);

MTX_SYSINIT(mtx_callout, &mtx_callout, "Callout", MTX_DEF | MTX_RECURSE);

void
usb_schedule(void)
{
	wakeup(&mtx_callout);
}

static void
usb_schedule_thread(void *arg)
{
	int timeout;

	while (1) {

		bus_handle_intr();

		timeout = usb_callout_process();

		tsleep(&mtx_callout, PRIBIO, "WSCHED", timeout);
	}
}

static void
usb_schedule_init(void *arg)
{
#if (__NetBSD_Version__ < 500000000)
	kthread_create1(&usb_schedule_thread, NULL,
	    NULL, "I4B core thread");
#else
	kthread_create(PRI_BIO, KTHREAD_MPSAFE, NULL,
	    &usb_schedule_thread, NULL, NULL, "I4BTHREAD");
#endif
}

SYSINIT(usb_schedule_init, SI_SUB_DRIVERS, SI_ORDER_FIRST,
    usb_schedule_init, NULL);

static void
usb_callout_callback(struct usb_callout *c)
{
	mtx_lock(c->mtx);

	mtx_lock(&mtx_callout);
	if (c->entry.le_prev != NULL) {
		LIST_REMOVE(c, entry);
		c->entry.le_prev = NULL;
	}
	mtx_unlock(&mtx_callout);

	if (curr_cancel) {
		mtx_unlock(c->mtx);
		return;
	}
	if (c->func) {
		(c->func) (c->arg);
	}
	if (!(c->flags & CALLOUT_RETURNUNLOCKED))
		mtx_unlock(c->mtx);
}

static int
usb_callout_process(void)
{
	struct usb_callout *c;
	int delta;
	int min_delay = hz / 16;

repeat:

	mtx_lock(&mtx_callout);

	curr_callout = NULL;

	LIST_FOREACH(c, &head_callout, entry) {

		delta = c->timeout - ticks;
		if (delta < 0) {
			curr_callout = c;
			curr_cancel = 0;
			mtx_unlock(&mtx_callout);

			usb_callout_callback(c);

			goto repeat;

		} else if (min_delay > delta) {
			min_delay = delta;
		}
	}

	mtx_unlock(&mtx_callout);

	if (min_delay <= 0)
		min_delay = 1;

	return (min_delay);
}

void
usb_callout_init_mtx(struct usb_callout *c, struct mtx *mtx, u_int32_t flags)
{
	bzero(c, sizeof(*c));

	if (mtx == NULL) {
		mtx = &Giant;
	}
	c->mtx = mtx;
	c->flags = (flags & CALLOUT_RETURNUNLOCKED);
}

void
usb_callout_reset(struct usb_callout *c, u_int32_t to_ticks,
    void (*func) (void *), void *arg)
{
	mtx_assert(c->mtx, MA_OWNED);

	usb_callout_stop(c);

	c->func = func;
	c->arg = arg;
	c->timeout = ticks + to_ticks;

	mtx_lock(&mtx_callout);
	LIST_INSERT_HEAD(&head_callout, c, entry);
	mtx_unlock(&mtx_callout);

	usb_schedule();
}

void
usb_callout_stop(struct usb_callout *c)
{
	if (c->mtx) {
		mtx_assert(c->mtx, MA_OWNED);
	}
	mtx_lock(&mtx_callout);

	if (c->entry.le_prev != NULL) {
		LIST_REMOVE(c, entry);
		c->entry.le_prev = NULL;
	}
	if (curr_callout == c)
		curr_cancel = 1;

	mtx_unlock(&mtx_callout);

	c->func = NULL;
	c->arg = NULL;
}

void
usb_callout_drain(struct usb_callout *c)
{
	if (c->mtx == NULL)
		return;			/* not initialised */

	usb_callout_stop(c);

	mtx_lock(&mtx_callout);
	while (curr_callout == c) {
		msleep(c, &mtx_callout, PRIBIO, "WCO", 1);
	}
	mtx_unlock(&mtx_callout);
}

u_int8_t
usb_callout_pending(struct usb_callout *c)
{
	u_int8_t retval;

	mtx_assert(c->mtx, MA_OWNED);

	mtx_lock(&mtx_callout);
	retval = (c->entry.le_prev != NULL);
	mtx_unlock(&mtx_callout);

	return (retval);
}
