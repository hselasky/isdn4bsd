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

#include <sys/freebsd_compat.h>

struct __callout {
    struct callout c_old;
    struct mtx *mtx;
    void *softintr;
    void (*func)(void *);
    void *arg;
    u_int32_t flags;
};

#define CALLOUT_RETURNUNLOCKED 0x0001

extern void
__callout_init_mtx(struct __callout *c, struct mtx *mtx, u_int32_t flags);

extern void
__callout_reset(struct __callout *c, u_int32_t to_ticks, 
		void (*func)(void *), void *arg);
extern void
__callout_stop(struct __callout *c);

static void
__callout_cb_1(void *arg)
{
    struct __callout *c = (struct __callout *)arg;
    if(c->softintr) {
      softintr_schedule(c->softintr);
    } else {
      printf("%s:%d: WARNING: no soft interrupt "
	     "handler!\n", __FILE__, __LINE__);
    }
    return;
}

static void
__callout_cb_2(void *arg)
{
    struct __callout *c = (struct __callout *)arg;

    mtx_lock(c->mtx);

    (c->func)(c->arg);

    if(!(c->flags & CALLOUT_RETURNUNLOCKED))
    {
        mtx_unlock(c->mtx);
    }
    return;
}

void
__callout_init_mtx(struct __callout *c, struct mtx *mtx, u_int32_t flags)
{
    bzero(c, sizeof(*c));

    if(mtx == NULL)
    {
       mtx = &Giant;
    }
    c->mtx = mtx;
    c->flags = (flags & CALLOUT_RETURNUNLOCKED);
    return;
}

void
__callout_reset(struct __callout *c, u_int32_t to_ticks, 
		void (*func)(void *), void *arg)
{
    mtx_assert(c->mtx, MA_OWNED);

    if(c->softintr) {
        __callout_stop(c);
    }

    c->func = func;
    c->arg = arg;
    c->softintr = softintr_establish(IPL_NET, &__callout_cb_2, c);

    callout_reset(&c->c_old, to_ticks, &__callout_cb_1, c);
    return;
}

void
__callout_stop(struct __callout *c)
{
    mtx_assert(c->mtx, MA_OWNED);

    callout_stop(&c->c_old);

    if(c->softintr) {

        /* XXX if the callback has already 
	 * been called back, this can cause
	 * a deadlock:
	 */
        softintr_disestablish(c->softintr);
	c->softintr = NULL;
    }
    return;
}
