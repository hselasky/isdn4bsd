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

struct cv;
struct timespec;

#ifdef SIMULATOR
extern int pthread_cond_wait(void **, void **mutex);
extern int pthread_cond_timedwait(void **, void **mutex, const struct timespec *abstime);
extern int pthread_cond_broadcast(void *cond);
extern int pthread_cond_signal(void **cond);
extern int pthread_cond_init(void **, void *);
extern int pthread_cond_destroy(void **);

#endif
void	cv_init(struct cv *cvp, const char *desc);
void	cv_destroy(struct cv *cvp);
void	cv_wait(struct cv *cvp, struct mtx *mtx);
int	cv_wait_sig(struct cv *cvp, struct mtx *mtx);
int	cv_timedwait(struct cv *cvp, struct mtx *mtx, uint32_t timo);
int	cv_timedwait_sig(struct cv *cvp, struct mtx *mtx, uint32_t timo);
void	cv_signal(struct cv *cvp);
void	cv_broadcast(struct cv *cvp);
const char *cv_wmesg(struct cv *cvp);

struct cv {
#if 0
	SEMAPHORE cv_sem;
#endif
	const char *cv_desc;
	struct mtx *cv_mtx;
#ifdef SIMULATOR
	void   *pthread_cond;
#else
	struct callout cv_co;
	uint16_t cv_waiters;
	uint8_t	cv_signal_all;
	uint8_t	cv_signalled;
	uint8_t	cv_timeout;
#endif
};
