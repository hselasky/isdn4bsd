/*-
 * Copyright (c) 2005-2006 Hans Petter Selasky. All rights reserved.
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

static void
mtx_warning(void *arg)
{
	printf("NOTE: The FreeBSD emulation layer"
	       "will not work reliably on "
	       "multiprocessor systems!\n");
	return;
}

SYSINIT(mtx_warning, SI_SUB_LOCK, SI_ORDER_ANY, 
        mtx_warning, NULL);

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

static __inline u_int8_t
mtx_lock_held(struct mtx *mtx)
{
    u_int8_t result;
    result = (mtx->owner_td == (void *)curthread);
    return result;
}

#ifdef MA_OWNED
void
_mtx_assert(struct mtx *mtx, u_int32_t what,
	    const char *file, u_int32_t line)
{
    u_int8_t own;
    u_int32_t s = splhigh();

    own = mtx_lock_held(mtx);

    splx(s);

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
    /* simple_lock_init() */

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

    if(mtx->owner_td)
    {
        if(curlwp)
	{
	    while(mtx->owner_td)
	    {
	        mtx->waiting = 1;
		(void) ltsleep(mtx, 0, "wait lock", 0, NULL);
	    }
	}
	else
	{
	    printf("WARNING: something is sleeping with "
		   "mutex '%s' locked!\n", mtx->name ? 
		   mtx->name : "unknown");
	    splx(s);
	    return;
	}
    }

    /* simple_lock() */

    mtx->s = s;
    mtx->owner_td = (void *)curthread;

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

    if(mtx->owner_td)
    {
        splx(s);
	return 0;
    }

    r = 1; /* simple_lock_try() */

    if(r == 0)
    {
        splx(s);
    }
    else
    {
        mtx->s = s;
	mtx->owner_td = (void *)curthread;
    }
    return r;
}

void
_mtx_unlock(struct mtx *mtx)
{
    u_int32_t s = splhigh();

    if(!mtx_lock_held(mtx))
    {
        goto done;
    }

    if(mtx->mtx_recurse == 0)    
    {
        splx(s);

        s = mtx->s;
	mtx->owner_td = NULL;
	mtx->s = 0;

	/* simple_unlock() */

	if(mtx->waiting) {
	   mtx->waiting = 0;
	   wakeup(mtx);
	}
    }
    else
    {
        mtx->mtx_recurse --;
    }

 done:
    splx(s);
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
    u_int32_t mtx_recurse = 0;
    u_int32_t s;
    u_int8_t held;

    if(mtx == NULL)
    {
        mtx = &Giant;
    }

    s = splhigh();
    held = mtx_lock_held(mtx);
    splx(s);

    if(held)
    {
        /* drop the lock */
        mtx_recurse = mtx->mtx_recurse;
	s = mtx->s;
	mtx->mtx_recurse = 0;
	mtx->owner_td = NULL;
	mtx->s = 0;
    }
    else
    {
        printf("WARNING: mutex '%s' was not locked when "
	       "trying to sleep '%s'!\n", mtx->name ? mtx->name : 
	       "unknown", wmesg ? wmesg : "unknown");
    }

    error = tsleep(ident, priority, wmesg, timeout);

    if(held)
    {
        /* pickup the lock */

        while(mtx->owner_td)
	{
	    mtx->waiting = 1;

	    (void) tsleep(mtx, 0, "wait lock", 0);
	}
        mtx->s = s;
	mtx->mtx_recurse = mtx_recurse;
	mtx->owner_td = (void *)curthread;
    }

    return error;
}
