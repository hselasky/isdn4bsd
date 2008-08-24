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

struct sx;

void	sx_init(struct sx *sx, const char *description);
void	sx_destroy(struct sx *sx);
void	sx_slock(struct sx *sx);
void	sx_xlock(struct sx *sx);
void	sx_sunlock(struct sx *sx);
void	sx_xunlock(struct sx *sx);
void	sx_unlock(struct sx *sx);
int	sx_xlocked(struct sx *sx);
void	_sx_assert(struct sx *sx, int what, const char *file, int line);

#if 1
#define	SA_LOCKED		0x01
#define	SA_SLOCKED		0x02
#define	SA_XLOCKED		0x04
#define	SA_UNLOCKED		0x08
#define	SA_RECURSED		0x10
#define	SA_NOTRECURSED		0x20

#define	sx_assert(m, what)				\
        _sx_assert((m), (what), __FILE__, __LINE__)
#else
#define	sx_assert(...) do { } while (0)
#endif

struct sx {
	struct cv wait;
	volatile uint32_t sx_recurse;
	const char *desc;
	volatile void *owner_td;
	volatile uint8_t init;		/* set if initialised */
};
