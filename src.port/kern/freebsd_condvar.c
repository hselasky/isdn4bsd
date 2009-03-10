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

static struct mtx cv_mtx;

static void
cv_module_init(void *arg)
{
	mtx_init(&cv_mtx, "CV-mutex", NULL, MTX_DEF);
	return;
}

SYSINIT(cv_module_init, SI_SUB_LOCK, SI_ORDER_MIDDLE, cv_module_init, NULL);

static void
cv_module_uninit(void *arg)
{
	mtx_destroy(&cv_mtx);
	return;
}

SYSUNINIT(cv_module_uninit, SI_SUB_LOCK, SI_ORDER_MIDDLE, cv_module_uninit, NULL);

#ifndef SIMULATOR
static void
cv_timeout_cb(void *arg)
{
	struct cv *cv = arg;

	cv->cv_timeout = 1;
	cv_broadcast(arg);
	return;
}

#endif

void
cv_init(struct cv *cv, const char *desc)
{
	memset(cv, 0, sizeof(*cv));
	cv->cv_desc = desc;
#ifdef SIMULATOR
	if (pthread_cond_init(&(cv->pthread_cond), NULL)) {
		panic("%s\n", desc);
	}
#else
	callout_init_mtx(&(cv->cv_co), &cv_mtx, 0);
#endif
	return;
}

void
cv_destroy(struct cv *cv)
{
#ifdef SIMULATOR
	if (pthread_cond_destroy(&(cv->pthread_cond))) {
		panic("%s\n", cv->cv_desc);
	}
#else
	callout_drain(&(cv->cv_co));
#endif
	return;
}

int
cv_timedwait_sub(struct cv *cv, struct mtx *mtx, uint32_t timo)
{
	int err;
	uint32_t rlevel;
	uint64_t timeout;

	mtx_assert(mtx, MA_OWNED);

#ifdef SIMULATOR

	mtx->owner_td = MTX_NO_THREAD;
	rlevel = mtx->mtx_recurse;
	mtx->mtx_recurse = 0;

	DROP_GIANT();

	if (timo != 0xFFFFFFFF) {
		struct timespec ts[2];

		timeout = (1000000000ULL / hz) * (uint64_t)timo;

		ts[0].tv_nsec = timeout % 1000000000ULL;
		ts[0].tv_sec = timeout / 1000000000ULL;

		if (clock_gettime(0 /* CLOCK_REALTIME */ , ts + 1)) {
			panic("%s\n", cv->cv_desc);
		}
		ts[0].tv_sec += ts[1].tv_sec;
		ts[0].tv_nsec += ts[1].tv_nsec;

		if (ts[0].tv_nsec >= 1000000000) {
			ts[0].tv_sec++;
			ts[0].tv_nsec -= 1000000000;
		}
		err = pthread_cond_timedwait(&(cv->pthread_cond), &(mtx->pthread_mtx), ts);
	} else {

		err = pthread_cond_wait(&(cv->pthread_cond), &(mtx->pthread_mtx));
	}

	mtx->owner_td = curthread;
	mtx->mtx_recurse = rlevel;

	mtx_unlock(mtx);

	PICKUP_GIANT();

	mtx_lock(mtx);


#ifdef PTHREAD_SETS_ERRNO
	if (err) {
		err = errno;
	}
#endif
	return (err);
#else
	mtx_lock(&cv_mtx);

	while (1) {
		if (cv->cv_waiters == 0xFFFF) {
			/* overflow */
			panic("cv_timedwait_sub: Overflow "
			    "on %s\n", cv->cv_desc);
		}
		cv->cv_waiters++;
		if (timo != 0xFFFFFFFF) {
			callout_reset(&(cv->cv_co), timo,
			    &cv_timeout_cb, cv);
		}
		mtx_unlock(&cv_mtx);
		mtx_unlock(mtx);

		DROP_GIANT();
#if 0
		wait_sem(&(cv->cv_sem));
#endif
		PICKUP_GIANT();

		mtx_lock(mtx);
		mtx_lock(&cv_mtx);
		cv->cv_waiters--;
		if (timo != 0xFFFFFFFF) {
			callout_stop(&(cv->cv_co));
		}
		if (cv->cv_timeout) {
			cv->cv_timeout = 0;
			mtx_unlock(&cv_mtx);
			return (EWOULDBLOCK);
		}
		if (cv->cv_signal_all) {
			if (cv->cv_waiters == 0) {
				cv->cv_signal_all = 0;
				cv->cv_signalled = 0;
			}
			break;
		} else if (cv->cv_signalled) {
			cv->cv_signalled = 0;
			break;
		}
	}

	mtx_unlock(&cv_mtx);
	return (0);
#endif
}

void
cv_wait(struct cv *cv, struct mtx *mtx)
{
	if (cv_timedwait_sub(cv, mtx, 0 - 1)) {
		/* ignore any errors */
	}
	return;
}

int
cv_wait_sig(struct cv *cv, struct mtx *mtx)
{
	return (cv_timedwait_sub(cv, mtx, 0 - 1));
}

int
cv_timedwait(struct cv *cv, struct mtx *mtx, uint32_t timo)
{
	return (cv_timedwait_sub(cv, mtx, timo));
}

int
cv_timedwait_sig(struct cv *cv, struct mtx *mtx, uint32_t timo)
{
	return (cv_timedwait_sub(cv, mtx, timo));
}

void
cv_signal(struct cv *cv)
{
#ifdef SIMULATOR
	pthread_cond_signal(&(cv->pthread_cond));
#else
	uint16_t waiters;

	mtx_lock(&cv_mtx);
	waiters = cv->cv_waiters;
	cv->cv_signalled = 1;
	mtx_unlock(&cv_mtx);
	if (waiters) {
#if 0
		signal_sem(&(cv->cv_sem));
#endif
	}
#endif
	return;
}

void
cv_broadcast(struct cv *cv)
{
#ifdef SIMULATOR
	pthread_cond_broadcast(&(cv->pthread_cond));
#else
	uint16_t waiters;

	mtx_lock(&cv_mtx);
	waiters = cv->cv_waiters;
	cv->cv_signalled = 1;
	cv->cv_signal_all = 1;
	mtx_unlock(&cv_mtx);
	while (waiters--) {
#if 0
		signal_sem(&(cv->cv_sem));
#endif
	}
#endif
	return;
}

const char *
cv_wmesg(struct cv *cv)
{
	return (cv->cv_desc);
}
