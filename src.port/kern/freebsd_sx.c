/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
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

#include <bsd_module_all.h>

#define	SX_NO_THREAD ((void *)(0-1))	/* NULL is interrupt handler */

void
sx_init(struct sx *sx, const char *description)
{
	bzero(sx, sizeof(*sx));
	if (description == NULL) {
		description = "unknown lock";
	}
	sx->desc = description;
	sx->init = 1;
	sx->owner_td = SX_NO_THREAD;
	cv_init(&(sx->wait), description);
	return;
}

void
sx_destroy(struct sx *sx)
{
	while (sx_xlocked(sx)) {
		sx_unlock(sx);
	}
	sx->init = 0;
	cv_destroy(&(sx->wait));
	return;
}

void
sx_slock(struct sx *sx)
{
	/* we don't support shared locks */
	sx_xlock(sx);
	return;
}

void
sx_xlock(struct sx *sx)
{
	mtx_lock(&Giant);

	if (sx_xlocked(sx)) {
		sx->sx_recurse++;
		goto done;
	}
	while (sx->owner_td != SX_NO_THREAD) {
		cv_wait(&(sx->wait), &Giant);
	}
	sx->owner_td = (void *)curthread;
done:
	mtx_unlock(&Giant);
	return;
}

void
sx_sunlock(struct sx *sx)
{
	/* we don't support shared locks */
	sx_xunlock(sx);
	return;
}

void
sx_xunlock(struct sx *sx)
{
	/* just an alias */
	sx_unlock(sx);
	return;
}

void
sx_unlock(struct sx *sx)
{
	mtx_lock(&Giant);

	if (!sx_xlocked(sx)) {
		panic("Lock '%s' is not locked\n", sx->desc);
	}
	if (sx->sx_recurse == 0) {
		sx->owner_td = SX_NO_THREAD;
		cv_signal(&(sx->wait));
	} else {
		sx->sx_recurse--;
	}

	mtx_unlock(&Giant);
	return;
}

int
sx_xlocked(struct sx *sx)
{
	if (!sx->init) {
		panic("Mutex is not initialised.\n");
	}
	return (sx->owner_td == (void *)curthread);
}

#ifdef SA_LOCKED
void
_sx_assert(struct sx *sx, int what,
    const char *file, int line)
{
	int own;

	own = sx_xlocked(sx);

	if ((what & (SA_LOCKED | SA_SLOCKED | SA_XLOCKED)) && (own == 0)) {
		panic("%s:%d: Lock '%s' not owned!\n",
		    file, line, sx->desc);
	}
	if ((what & SA_UNLOCKED) && (own == 1)) {
		panic("%s:%d: Lock '%s' owned!\n",
		    file, line, sx->desc);
	}
	return;
}

#endif
