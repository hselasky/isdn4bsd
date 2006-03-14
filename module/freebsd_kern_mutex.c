/*-
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
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
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/kern/kern_mutex.c"
 */
#include <sys/param.h>
#include <sys/systm.h>

#include <sys/freebsd_compat.h>

struct mtx Giant;
struct mtx Atomic;

MTX_SYSINIT(Giant,  &Giant,  "Giant",  MTX_DEF|MTX_RECURSE);
MTX_SYSINIT(Atomic, &Atomic, "Atomic", MTX_DEF);

void
atomic_add_int(u_int *p, u_int v)
{
    mtx_lock(&Atomic);
    p[0] += v;
    mtx_unlock(&Atomic);
    return;
}

void
atomic_sub_int(u_int *p, u_int v)
{
    mtx_lock(&Atomic);
    p[0] -= v;
    mtx_unlock(&Atomic);
    return;
}

int
atomic_cmpset_int(volatile u_int *dst, u_int exp, u_int src)
{
    u_int8_t ret = 0;

    mtx_lock(&Atomic);
    if(dst[0] == exp)
    {
        dst[0] = src;
	ret = 1;
    }
    mtx_unlock(&Atomic);
    return ret;
}

#if 0
#define cli() mtx_lock(0)
#define sti() mtx_unlock(0);
#endif

static __inline u_int8_t
mtx_lock_held(struct mtx *mtx)
{
#ifdef simple_lock_held
  return simple_lock_held(&mtx->lock);
#else
  return mtx->held;
#endif
}

#ifdef MA_OWNED
void
_mtx_assert(struct mtx *mtx, u_int32_t what,
	    const char *file, u_int32_t line)
{
    u_int8_t own;

    own = mtx_lock_held(mtx);

    if((what & MA_OWNED) && (own == 0))
    {
	printf("%s:%d: mutex %s not owned!\n", 
	       file, line, mtx->name ? mtx->name : "unknown");
    }
    if((what & MA_NOTOWNED) && (own == 1))
    {
	printf("%s:%d: mutex %s owned!\n", 
	       file, line, mtx->name ? mtx->name : "unknown");
    }
    return;
}
#endif

u_int8_t
mtx_initialized(struct mtx *mtx)
{
    return mtx->init;
}

void
mtx_init(struct mtx *mtx, const char *name, 
	 const char *type, u_int32_t opts)
{
    bzero(mtx, sizeof(*mtx));
    simple_lock_init(&mtx->lock);

    if(name == NULL)
    {
        name = "unknown lock";
    }

    if(type == NULL)
    {
        type = name;
    }

    mtx->name = name;
    mtx->init = 1;

    return;
}

void
mtx_lock(struct mtx *mtx)
{
    u_int32_t s = splhigh();

    if(mtx_lock_held(mtx))
    {
        mtx->mtx_recurse++;
	splx(s);
	return;
    }

    simple_lock(&mtx->lock);

    mtx->s = s;
    mtx->held = 1;

    return;
}

u_int8_t
mtx_trylock(struct mtx *mtx)
{
    u_int32_t s = splhigh();
    u_int8_t r;

    if(mtx_lock_held(mtx))
    {
        mtx->mtx_recurse++;
	splx(s);
	return 1;
    }

    r = simple_lock_try(&mtx->lock);

    if(r == 0)
    {
        splx(s);
    }
    else
    {
        mtx->s = s;
	mtx->held = 1;
    }
    return r;
}

void
_mtx_unlock(struct mtx *mtx)
{
    u_int32_t s;

    if(mtx->mtx_recurse == 0)    
    {
      s = mtx->s;
      mtx->held = 0;
      simple_unlock(&mtx->lock);
      splx(s);
    }
    else
    {
       mtx->mtx_recurse --;
    }
    return;
}

void
mtx_destroy(struct mtx *mtx)
{
    return;
}

void
mtx_sysinit(void *arg)
{
    struct mtx_args *ma = arg;
    mtx_init(ma->mtx, ma->desc, NULL, ma->flags);
    return;
}

int
msleep(void *ident, struct mtx *mtx, int priority,
       const char *wmesg, int timeout)
{
    int error;
    u_int32_t mtx_recurse;
    u_int32_t s;

    if(mtx == NULL)
    {
        mtx = &Giant;
    }

    mtx_assert(mtx, MA_OWNED);
  
    mtx_recurse = mtx->mtx_recurse;
    mtx->mtx_recurse = 0;
    mtx->held = 0;

    s = mtx->s;

    /* XXX one never sleeps from an interrupt handler
     * so it is safe to exit the current interrupt
     * level before exiting the lock, though actually
     * one should exit this level after exiting the lock,
     * but that is not supported by "lt_sleep()"
     */
    splx(s);

    error = ltsleep(ident, priority, wmesg, timeout, &mtx->lock);

    /* XXX ltsleep should have done this before entering the lock */
    mtx->s = splhigh();
    mtx->mtx_recurse = mtx_recurse;
    mtx->held = 1;

    return error;
}
