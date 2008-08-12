/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
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
 */

struct mtx;

void	mtx_init(struct mtx *mutex, const char *name, const char *type, int opts);
void	mtx_destroy(struct mtx *mutex);
void	mtx_lock(struct mtx *mutex);
void	mtx_unlock(struct mtx *mutex);
int	mtx_owned(struct mtx *mutex);
void	_mtx_assert(struct mtx *mutex, int what, const char *file, int line);
int	mtx_trylock(struct mtx *mtx);
int	mtx_initialized(struct mtx *mtx);
void	atomic_add_int(uint32_t *p, uint32_t v);
void	atomic_sub_int(uint32_t *p, uint32_t v);
int	atomic_cmpset_int(volatile uint32_t *dst, uint32_t exp, uint32_t src);

extern struct mtx Giant;

#define	MTX_DEF         0x00000000	/* DEFAULT (sleep) lock */
#define	MTX_SPIN        0x00000001	/* Spin lock (disables interrupts) */
#define	MTX_RECURSE     0x00000004	/* Option: lock allowed to recurse */
#define	MTX_NOWITNESS   0x00000008	/* Don't do any witness checking. */
#define	MTX_DUPOK       0x00000020	/* Don't log a duplicate acquire */
#define	MTX_QUIET       LOP_QUIET	/* Don't log a mutex event */

#if 1
#define	MA_OWNED        0x01
#define	MA_NOTOWNED     0x02
#define	MA_RECURSED     0x04
#define	MA_NOTRECURSED  0x08
#define	mtx_assert(m, what)				\
        _mtx_assert((m), (what), __FILE__, __LINE__)
#else
#define	mtx_assert(...) do { } while (0)
#endif

struct mtx {
	uint32_t mtx_recurse;
	const char *name;
	void   *owner_td;
	uint8_t	init;			/* set if initialised */
};

#define	DROP_GIANT() do {			\
	uint32_t level = 0;			\
	while (mtx_owned(&Giant)) {		\
		mtx_unlock(&Giant);		\
		level++;			\
	}

#define	PICKUP_GIANT()				\
	while (level--) {			\
		mtx_lock(&Giant);		\
	}					\
} while (0)
