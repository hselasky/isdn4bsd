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

/*
 * This file contains definitions for FreeBSD callouts.
 */

enum {
	hz = 16,
};

#define	CALLOUT_RETURNUNLOCKED 0x01
#define	CALLOUT_PENDING 0x02
#define	CALLOUT_CALLING_BACK 0x04

typedef void (timeout_t)(void *arg);

extern uint32_t ticks;

struct callout {
	TAILQ_ENTRY(callout) co_entry;
	timeout_t *co_func;
	void   *co_arg;
	struct mtx *co_mtx;
	uint8_t	co_flags;
	uint8_t	co_index;
	uint8_t	co_power;
};

void	callout_init_mtx(struct callout *c, struct mtx *mtx, int flags);
void	callout_stop(struct callout *c);
void	callout_drain(struct callout *c);
void	callout_reset(struct callout *c, uint32_t ticks, timeout_t *func, void *arg);
