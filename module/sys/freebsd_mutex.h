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
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/sys/mutex.h"
 */
#ifndef __FREEBSD_SYS_MUTEX_H__
#define __FREEBSD_SYS_MUTEX_H__

struct mtx {
  u_int32_t mtx_recurse;
  u_int8_t init;
  u_int8_t waiting;
  const char * name;
  void *owner_td;
};

#define MTX_DEF         0x00000000      /* DEFAULT (sleep) lock */ 
#define MTX_SPIN        0x00000001      /* Spin lock (disables interrupts) */
#define MTX_RECURSE     0x00000004      /* Option: lock allowed to recurse */
#define MTX_NOWITNESS   0x00000008      /* Don't do any witness checking. */
#define MTX_DUPOK       0x00000020      /* Don't log a duplicate acquire */
#define MTX_QUIET       LOP_QUIET       /* Don't log a mutex event */

#define mtx_lock_spin(m)        mtx_lock(m)
#define mtx_unlock_spin(m)      mtx_unlock(m)

extern u_int8_t
  mtx_initialized(struct mtx *m);

extern void
  mtx_init(struct mtx *m, const char *name, 
	   const char *type, u_int32_t opts);
extern void
  mtx_destroy(struct mtx *m);
extern void
  mtx_sysinit(void *arg);

extern void
  mtx_lock(struct mtx *m);
extern void
  _mtx_unlock(struct mtx *m);

#define mtx_unlock(mtx) do { \
    mtx_assert(mtx, MA_OWNED);	\
    _mtx_unlock(mtx); } while (0)

extern u_int8_t
  mtx_trylock(struct mtx *m);
extern void
  _mtx_assert(struct mtx *m, u_int32_t what, 
	      const char *file, u_int32_t line);
extern int
  msleep(void *ident, struct mtx *mtx, int priority,
	 const char *wmesg, int timeout);

extern void  atomic_add_int(u_int *p, u_int v);
extern void atomic_sub_int(u_int *p, u_int v);
extern int  atomic_cmpset_int(volatile u_int *dst, u_int exp, u_int src);
extern void atomic_lock(void);
extern void atomic_unlock(void);

extern struct mtx Giant;

struct mtx_args {
  struct mtx *mtx;
  const u_int8_t *desc;
  u_int32_t flags;
};

#define MTX_SYSINIT(name, mtx, desc, opts)                              \
        static struct mtx_args name##_args = {                          \
                (mtx),                                                  \
                (desc),                                                 \
                (opts)                                                  \
        };                                                              \
        SYSINIT(name##_mtx_sysinit, SI_SUB_LOCK, SI_ORDER_MIDDLE,       \
		mtx_sysinit, &name##_args)

#if 1
#define MA_OWNED        0x01
#define MA_NOTOWNED     0x02
#define MA_RECURSED     0x04
#define MA_NOTRECURSED  0x08
#define mtx_assert(m, what)				\
        _mtx_assert((m), (what), __FILE__, __LINE__)
#else
#define mtx_assert(a...) 
#endif

#if (!defined(PSR_IMPL))

/*
 * On NetBSD locking a mutex will
 * disable the interrupts!
 */
static __inline register_t
intr_disable(void)
{
    atomic_lock();
    return 0;
}

static __inline void
intr_restore(register_t restore)
{
    atomic_unlock();
    return;
}

#endif

#endif
