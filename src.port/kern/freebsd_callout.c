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

#include <bsd_module_all.h>

uint32_t ticks = 0;

TAILQ_HEAD(callout_head, callout);

#define	MAX_POWER 16

static struct callout_head callwheel[2][MAX_POWER];
static struct cv callout_drain_cv;
static struct mtx callout_mtx;
static struct callout *curr_callout = NULL;

static uint8_t
callout_power(uint32_t x)
{
	uint8_t n = 0;

	while ((x /= 2))
		n++;
	if (n > (MAX_POWER - 1)) {
		n = MAX_POWER - 1;
	}
	return (n);
}

static void
callout_module_init(void *arg)
{
	struct callout_head *coh;
	uint16_t x;
	uint16_t y;

	mtx_init(&callout_mtx, "callout-mtx", NULL, MTX_DEF);
	cv_init(&callout_drain_cv, "callout-cv");

	for (x = 0; x != 2; x++) {
		for (y = 0; y != MAX_POWER; y++) {
			coh = &callwheel[x][y];
			TAILQ_INIT(coh);
		}
	}
	return;
}

SYSINIT(callout_module_init, SI_SUB_LOCK, SI_ORDER_MIDDLE, callout_module_init, NULL);

static void
callout_module_uninit(void *arg)
{
	cv_destroy(&callout_drain_cv);
	mtx_destroy(&callout_mtx);
}

SYSUNINIT(callout_module_uninit, SI_SUB_LOCK, SI_ORDER_MIDDLE, callout_module_uninit, NULL);

static void
callout_do(struct callout_head *coh)
{
	struct callout *co;
	struct mtx *mtx;
	uint8_t do_unlock;
	uint8_t cancelled;

	mtx_lock(&callout_mtx);
	while (1) {

		co = TAILQ_FIRST(coh);
		curr_callout = co;
		if (co == NULL) {
			break;
		}
		TAILQ_REMOVE(coh, co, co_entry);
		co->co_flags &= ~CALLOUT_PENDING;
		co->co_flags |= CALLOUT_CALLING_BACK;
		mtx = co->co_mtx;
		mtx_unlock(&callout_mtx);

		/* avoid lock order reversal */

		mtx_lock(mtx);
		mtx_lock(&callout_mtx);
		if (co == curr_callout) {
			do_unlock =
			    (co->co_flags & CALLOUT_RETURNUNLOCKED) ? 0 : 1;
			mtx_unlock(&callout_mtx);
			(co->co_func) (co->co_arg);
			mtx_lock(&callout_mtx);
			co->co_flags &= ~CALLOUT_CALLING_BACK;
			if (co == curr_callout) {
				cancelled = 0;
			} else {
				cancelled = 1;
			}
		} else {
			/* callout was cancelled */
			do_unlock = 1;
			cancelled = 1;
		}
		if (do_unlock) {
			/* need to unlock before checking cancelled */
			mtx_unlock(mtx);
		}
		if (cancelled) {
			cv_broadcast(&callout_drain_cv);
		}
	}
	mtx_unlock(&callout_mtx);
	return;
}

#define	DELTA 1

static uint8_t callout_module_teardown = 0;

static void
callout_module_tick(void *arg)
{
	uint32_t m;
	uint32_t delta;
	uint8_t power;

repeat:

	m = 1;

	power = 0;

	mtx_lock(&callout_mtx);
	ticks++;
	mtx_unlock(&callout_mtx);

	delta = ticks ^ (ticks - 1);

	while (power != MAX_POWER) {
		if (delta & m) {
			if (ticks & m)
				callout_do(&callwheel[1][power]);
			else
				callout_do(&callwheel[0][power]);
		}
		m *= 2;
		power++;
	}

	if (callout_module_teardown == 0) {
		pause("TWAIT", DELTA);
		goto repeat;
	}
	kproc_exit(0);

	return;
}

static void
callout_sysinit(void *arg)
{
	int err;
	struct proc *proc;

	err = kproc_create(&callout_module_tick, NULL,
	    &proc, RFHIGHPID, 0, "USBTICK");
	if (err) {
		printf("WARNING: Could not create callout "
		    "tick process, err=%d!\n", err);
	}
	return;
}

SYSINIT(callout_sysinit, SI_SUB_KLD, SI_ORDER_FIRST, &callout_sysinit, NULL);

static void
callout_sysuninit(void *arg)
{
	callout_module_teardown = 1;

	pause("SYNC", 2 * DELTA);

	return;
}

SYSUNINIT(callout_sysuninit, SI_SUB_KLD, SI_ORDER_FIRST, &callout_sysuninit, NULL);


void
callout_init_mtx(struct callout *co, struct mtx *mtx, int flags)
{
	memset(co, 0, sizeof(*co));
	co->co_mtx = mtx;
	co->co_flags = flags & CALLOUT_RETURNUNLOCKED;
	return;
}

void
callout_stop(struct callout *co)
{
	struct callout_head *coh;

	mtx_assert(co->co_mtx, MA_OWNED);

	mtx_lock(&callout_mtx);
	if (co->co_flags & CALLOUT_PENDING) {
		co->co_flags &= ~CALLOUT_PENDING;
		coh = &callwheel[co->co_index][co->co_power];
		TAILQ_REMOVE(coh, co, co_entry);
	}
	mtx_unlock(&callout_mtx);
	return;
}

void
callout_drain(struct callout *co)
{
	if (co->co_mtx == NULL) {
		/* XXX not initialised */
		return;
	}
	mtx_assert(co->co_mtx, MA_NOTOWNED);

	mtx_lock(co->co_mtx);
	callout_stop(co);
	mtx_unlock(co->co_mtx);

	mtx_lock(&callout_mtx);
	while (co->co_flags & CALLOUT_CALLING_BACK) {
		curr_callout = NULL;
		cv_wait(&callout_drain_cv, &callout_mtx);
	}
	mtx_unlock(&callout_mtx);
	return;
}

void
callout_reset(struct callout *co, uint32_t to_ticks,
    timeout_t *func, void *arg)
{
	uint8_t power = callout_power(to_ticks);
	struct callout_head *coh;

	mtx_assert(co->co_mtx, MA_OWNED);

	callout_stop(co);

	co->co_func = func;
	co->co_arg = arg;

	mtx_lock(&callout_mtx);
	if (ticks & (1 << power)) {
		coh = &callwheel[1][power];
		co->co_index = 1;
		co->co_power = power;
	} else {
		co->co_index = 0;
		co->co_power = power;
		coh = &callwheel[0][power];
	}
	TAILQ_INSERT_TAIL(coh, co, co_entry);
	mtx_unlock(&callout_mtx);

	return;
}
