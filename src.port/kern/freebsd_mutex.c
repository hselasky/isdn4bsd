/*-
 * Copyright (c) 2005-2008 Hans Petter Selasky. All rights reserved.
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
 */

/*
 * This file contains a lightweight implementation of mutexes.
 */

#include <bsd_module_all.h>

#define	MTX_NO_THREAD ((void *)(0-1))	/* NULL is interrupt handler */

struct mtx Giant;

static void mtx_sysinit(void *arg);
static void mtx_sysuninit(void *arg);

SYSINIT(Giant_sysinit, SI_SUB_LOCK, SI_ORDER_MIDDLE, mtx_sysinit, &Giant);
SYSUNINIT(Giant_sysuninit, SI_SUB_LOCK, SI_ORDER_MIDDLE, mtx_sysuninit, &Giant);

#if 1
static uint32_t atomic_recurse = 0;

static void
atomic_lock(void)
{
  disable_intr();

  if (++atomic_recurse == 0xFFFFFFFF) {
    panic("freebsd_mutex: Atomic lock overflow!\n");
  }
  return;
}

static void
atomic_unlock(void)
{
  if (--atomic_recurse == 0) {
    enable_intr();
  }
  return;
}
#endif

void
atomic_add_int(uint32_t *p, uint32_t v)
{
	atomic_lock();
	p[0] += v;
	atomic_unlock();
	return;
}

void
atomic_sub_int(uint32_t *p, uint32_t v)
{
	atomic_lock();
	p[0] -= v;
	atomic_unlock();
	return;
}

int
atomic_cmpset_int(volatile uint32_t *dst, uint32_t exp, uint32_t src)
{
	int ret = 0;

	atomic_lock();
	if (dst[0] == exp) {
		dst[0] = src;
		ret = 1;
	}
	atomic_unlock();
	return (ret);
}

int
mtx_owned(struct mtx *mtx)
{
	if (!mtx->init) {
		panic("Mutex is not initialised.\n");
	}
	return (mtx->owner_td == (void *)curthread);
}

#ifdef MA_OWNED
void
_mtx_assert(struct mtx *mtx, int what,
    const char *file, int line)
{
	int own;

	own = mtx_owned(mtx);

	if ((what & MA_OWNED) && (own == 0)) {
		panic("%s:%d: mutex '%s' not owned!\n",
		    file, line, mtx->name);
	}
	if ((what & MA_NOTOWNED) && (own == 1)) {
		panic("%s:%d: mutex '%s' owned!\n",
		    file, line, mtx->name);
	}
	return;
}

#endif

int
mtx_initialized(struct mtx *mtx)
{
	return (mtx->init);
}

void
mtx_init(struct mtx *mtx, const char *name,
    const char *type, int opts)
{
	bzero(mtx, sizeof(*mtx));

	if (name == NULL) {
		name = "unknown lock";
	}
	if (type == NULL) {
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

	if (mtx_owned(mtx)) {
		mtx->mtx_recurse++;
		atomic_unlock();
		return;
	}
	if (mtx->owner_td != MTX_NO_THREAD) {
		panic("Something is sleeping with "
		    "mutex '%s' locked!\n", mtx->name ?
		    mtx->name : "unknown");
	}
	mtx->owner_td = (void *)curthread;
	return;
}

int
mtx_trylock(struct mtx *mtx)
{
#ifdef MTX_DEBUG
	printf("mtx_trylock %s %u\n", mtx->name, mtx->mtx_recurse);
#endif

	atomic_lock();

	if (mtx_owned(mtx)) {
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
mtx_unlock(struct mtx *mtx)
{
#ifdef MTX_DEBUG
	printf("mtx_unlock %s %u\n", mtx->name, mtx->mtx_recurse);
#endif

	atomic_lock();

	if (!mtx_owned(mtx)) {
		panic("Mutex '%s' is not locked\n", mtx->name);
	}
	if (mtx->mtx_recurse == 0) {
		atomic_unlock();	/* double unlock - yes */

		mtx->owner_td = MTX_NO_THREAD;
	} else {
		mtx->mtx_recurse--;
	}

	atomic_unlock();
	return;
}

void
mtx_destroy(struct mtx *mtx)
{
	while (mtx_owned(mtx)) {
		mtx_unlock(mtx);
	}
	mtx->init = 0;
	return;
}

static void
mtx_sysinit(void *arg)
{
	struct mtx *m = arg;

	mtx_init(m, "Giant", NULL, MTX_DEF | MTX_RECURSE);
	return;
}

static void
mtx_sysuninit(void *arg)
{
	struct mtx *m = arg;

	mtx_destroy(m);
	return;
}
