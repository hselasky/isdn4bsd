/*-
 * Copyright (c) 2009 Hans Petter Selasky. All rights reserved.
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
#include <sys/mutex.h>

#include <sys/freebsd_compat.h>

#define	MTX_NO_THREAD ((void *)1)

struct mtx Giant;

MTX_SYSINIT(Giant, &Giant, "Giant", MTX_DEF | MTX_RECURSE);

static uint32_t atomic_recurse = 0;
static kmutex_t atomic_mutex;

static void
atomic_init(void *arg)
{
	mutex_init(&atomic_mutex, MUTEX_DEFAULT, IPL_HIGH);
}

SYSINIT(atomic_init, SI_SUB_DONE, SI_ORDER_FIRST, atomic_init, NULL);

void
atomic_lock()
{
	if (!mutex_owned(&atomic_mutex))
		mutex_enter(&atomic_mutex);
	atomic_recurse++;
}
void
atomic_unlock()
{
	if (atomic_recurse == 0) {
		panic("freebsd_kern_mutex_v5.c: atomic_unlock "
		    "- invalid refcount!\n");
	} else {
		if (--atomic_recurse == 0) {
			mutex_exit(&atomic_mutex);
		}
	}
}

int
atomic_cmpset_int(volatile u_int *dst, u_int exp, u_int src)
{
	uint8_t ret = 0;

	atomic_lock();
	if (dst[0] == exp) {
		dst[0] = src;
		ret = 1;
	}
	atomic_unlock();
	return (ret);
}

static __inline uint8_t
mtx_lock_held(struct mtx *mtx)
{
	if (!mtx->init) {
		panic("Mutex is not initialised.\n");
	}
	return ((void *)curthread == mtx->owner_td);
}

#ifdef MA_OWNED
void
_mtx_assert(struct mtx *mtx, uint32_t what,
    const char *file, uint32_t line)
{
	uint8_t own;


	atomic_lock();
	own = mtx_lock_held(mtx);
	atomic_unlock();

	if ((what & MA_OWNED) && (own == 0)) {
		printf("%s:%d: mutex %s not owned!\n",
		    file, line, mtx->name ? mtx->name : "unknown");
	}
	if ((what & MA_NOTOWNED) && (own == 1)) {
		printf("%s:%d: mutex %s owned!\n",
		    file, line, mtx->name ? mtx->name : "unknown");
	}
}

#endif

uint8_t
mtx_initialized(struct mtx *mtx)
{
	return (mtx->init);
}

void
mtx_init(struct mtx *mtx, const char *name,
    const char *type, uint32_t opts)
{
	memset(mtx, 0, sizeof(*mtx));

	if (name == NULL) {
		name = "unknown lock";
	}
	if (type == NULL) {
		type = name;
	}
	mtx->name = name;
	mtx->init = 1;
	mtx->owner_td = MTX_NO_THREAD;
}

void
mtx_lock(struct mtx *mtx)
{
#ifdef MTX_DEBUG
	printf("mtx_lock %s %u\n", mtx->name, mtx->mtx_recurse);
#endif

	atomic_lock();

	if (mtx_lock_held(mtx)) {
		mtx->mtx_recurse++;
		atomic_unlock();
		return;
	}
	if (mtx->owner_td != MTX_NO_THREAD) {
		printf("WARNING: something is sleeping with "
		    "mutex '%s' locked!\n", mtx->name ?
		    mtx->name : "unknown");
		atomic_unlock();
		return;
	}
	mtx->owner_td = (void *)curthread;
}

uint8_t
mtx_trylock(struct mtx *mtx)
{
#ifdef MTX_DEBUG
	printf("mtx_trylock %s %u\n", mtx->name, mtx->mtx_recurse);
#endif

	atomic_lock();

	if (mtx_lock_held(mtx)) {
		mtx->mtx_recurse++;
		atomic_unlock();
		return (1);
	}
	if (mtx->owner_td != MTX_NO_THREAD) {
		atomic_unlock();
		return (0);
	}
	mtx->owner_td = (void *)curthread;
	return (1);
}

void
_mtx_unlock(struct mtx *mtx)
{
#ifdef MTX_DEBUG
	printf("mtx_unlock %s %u\n", mtx->name, mtx->mtx_recurse);
#endif

	atomic_lock();

	if (!mtx_lock_held(mtx)) {
		goto done;
	}
	if (mtx->mtx_recurse == 0) {
		atomic_unlock();

		mtx->owner_td = MTX_NO_THREAD;
	} else {
		mtx->mtx_recurse--;
	}

done:
	atomic_unlock();
}

void
mtx_destroy(struct mtx *mtx)
{
	mtx->init = 0;
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
	uint32_t mtx_recurse = 0;
	uint32_t a_recurse = 0;
	uint8_t held;

	if (mtx == NULL) {
		mtx = &Giant;
	}
	atomic_lock();
	held = mtx_lock_held(mtx);
	atomic_unlock();

	if (held) {
		/* drop the lock */
		mtx_recurse = mtx->mtx_recurse;
		mtx->mtx_recurse = 0;
		mtx->owner_td = MTX_NO_THREAD;

		a_recurse = atomic_recurse;
		atomic_recurse = 0;
	} else {
		printf("WARNING: mutex '%s' was not locked when "
		    "trying to sleep '%s'!\n", mtx->name ? mtx->name :
		    "unknown", wmesg ? wmesg : "unknown");
	}

	priority &= PCATCH;
	priority |= PRIBIO;

	error = mtsleep(ident, priority, wmesg, timeout, &atomic_mutex);

	if (held) {
		mtx->mtx_recurse = mtx_recurse;
		mtx->owner_td = (void *)curthread;

		atomic_recurse = a_recurse;
	}
	return (error);
}

int
__lockmgr(struct lock *lock, int what, void *dummy, struct thread *td)
{
	return (vlockmgr(&lock->lock, what));
}

void
lockinit(struct lock *lock, int pri, const char *desc, int x, int y)
{
	memset(lock, 0, sizeof(*lock));
	rw_init(&lock->lock.vl_lock);
}
