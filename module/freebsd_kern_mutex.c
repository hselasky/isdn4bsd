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

#define MTX_NO_THREAD ((void *)1) /* NULL is interrupt handler */

struct mtx Giant;

MTX_SYSINIT(Giant,  &Giant,  "Giant",  MTX_DEF|MTX_RECURSE);

static void
mtx_warning(void *arg)
{
	printf("NOTE: The FreeBSD emulation layer "
	       "will not work reliably on "
	       "multiprocessor systems!\n");
	return;
}

SYSINIT(mtx_warning, SI_SUB_LOCK, SI_ORDER_ANY, 
        mtx_warning, NULL);

static uint32_t atomic_recurse = 0;
static int atomic_spl = 0;

void
atomic_lock()
{
	int s = splhigh();

#ifdef MTX_DEBUG
	printf("+%u", atomic_recurse);
#endif
	if (atomic_recurse == 0) {
		atomic_spl = s;
		atomic_recurse = 1;
	} else {
		if (++atomic_recurse == 0xFFFFFFFF) {
			panic("freebsd_kern_mutex.c: atomic_lock - "
			      "refcount is wrapping!\n");
		}
	}
	return;
}

void
atomic_unlock()
{
#ifdef MTX_DEBUG
	printf("-%u", atomic_recurse);
#endif
	if (atomic_recurse == 0) {
		panic("freebsd_kern_mutex.c: atomic_unlock "
		      "- invalid refcount!\n");
	} else {
		if (--atomic_recurse == 0) {
			splx(atomic_spl);
		}
	}
	return;
}

#if (__NetBSD_Version__ < 500000000)
void
atomic_add_int(u_int *p, u_int v)
{
    atomic_lock();
    p[0] += v;
    atomic_unlock();
    return;
}

void
atomic_sub_int(u_int *p, u_int v)
{
    atomic_lock();
    p[0] -= v;
    atomic_unlock();
    return;
}

int
atomic_cmpset_int(volatile u_int *dst, u_int exp, u_int src)
{
    u_int8_t ret = 0;

    atomic_lock();
    if(dst[0] == exp)
    {
        dst[0] = src;
	ret = 1;
    }
    atomic_unlock();
    return ret;
}
#endif

static __inline u_int8_t
mtx_lock_held(struct mtx *mtx)
{
    u_int8_t result;

    if (!mtx->init) {
	panic("Mutex is not initialised.\n");
    }

    result = (mtx->owner_td == (void *)curthread);
    return result;
}

#ifdef MA_OWNED
void
_mtx_assert(struct mtx *mtx, u_int32_t what,
	    const char *file, u_int32_t line)
{
    u_int8_t own;

    atomic_lock();
    own = mtx_lock_held(mtx);
    atomic_unlock();

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
    mtx->owner_td = MTX_NO_THREAD;

    return;
}

void
mtx_lock(struct mtx *mtx)
{
#ifdef MTX_DEBUG
    printf("mtx_lock %s %u\n", mtx->name, mtx->mtx_recurse);
#endif

    atomic_lock();

    if(mtx_lock_held(mtx))
    {
        mtx->mtx_recurse++;
	atomic_unlock();
	return;
    }

    if(mtx->owner_td != MTX_NO_THREAD)
    {
        if(curlwp)
	{
	    while(mtx->owner_td != MTX_NO_THREAD)
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
	    atomic_unlock();
	    return;
	}
    }

    mtx->owner_td = (void *)curthread;

    return;
}

u_int8_t
mtx_trylock(struct mtx *mtx)
{
    u_int8_t r;

#ifdef MTX_DEBUG
    printf("mtx_trylock %s %u\n", mtx->name, mtx->mtx_recurse);
#endif

    atomic_lock();

    if(mtx_lock_held(mtx))
    {
        mtx->mtx_recurse++;
	atomic_unlock();
	return 1;
    }

    if(mtx->owner_td != MTX_NO_THREAD)
    {
	atomic_unlock();
	return 0;
    }

    r = 1;

    if(r == 0)
    {
	atomic_unlock();
    }
    else
    {
	mtx->owner_td = (void *)curthread;
    }
    return r;
}

void
_mtx_unlock(struct mtx *mtx)
{
#ifdef MTX_DEBUG
    printf("mtx_unlock %s %u\n", mtx->name, mtx->mtx_recurse);
#endif

    atomic_lock();

    if(!mtx_lock_held(mtx))
    {
        goto done;
    }

    if(mtx->mtx_recurse == 0)    
    {
	atomic_unlock();

	mtx->owner_td = MTX_NO_THREAD;

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
    atomic_unlock();
    return;
}

void
mtx_destroy(struct mtx *mtx)
{
    mtx->init = 0;
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
    u_int8_t held;
    u_int32_t a_recurse = 0;
    int a_spl = 0;

    if(mtx == NULL)
    {
        mtx = &Giant;
    }

    atomic_lock();
    held = mtx_lock_held(mtx);
    atomic_unlock();

    if(held)
    {
        /* drop the lock */
        mtx_recurse = mtx->mtx_recurse;
	mtx->mtx_recurse = 0;
	mtx->owner_td = MTX_NO_THREAD;

	a_recurse = atomic_recurse;
	a_spl = atomic_spl;
	atomic_recurse = 0;
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

        while(mtx->owner_td != MTX_NO_THREAD)
	{
	    mtx->waiting = 1;

	    (void) tsleep(mtx, 0, "wait lock", 0);
	}
	mtx->mtx_recurse = mtx_recurse;
	mtx->owner_td = (void *)curthread;

	atomic_recurse = a_recurse;
	atomic_spl = a_spl;
    }

    return error;
}
