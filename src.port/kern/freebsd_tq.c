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
 * This function defines a simple tailqueue.
 */

#include <bsd_module_all.h>

void
bsd_tq_frag_set_buf_len(struct bsd_tq_frag *frag, void *buf,
    uint32_t len)
{
	frag->pbuf = buf;
	frag->len = len;
	return;
}

void
bsd_tq_frag_get_buf_len(struct bsd_tq_frag *frag, void **ppbuf,
    uint32_t *plen)
{
	*ppbuf = frag->pbuf;
	*plen = frag->len;
	return;
}

void
bsd_tq_frag_set_buf_len_arg(struct bsd_tq_frag *frag, void *buf,
    uint32_t len, void *arg)
{
	frag->pbuf = buf;
	frag->len = len;
	frag->arg = arg;
	return;
}

void
bsd_tq_frag_get_buf_len_arg(struct bsd_tq_frag *frag, void **ppbuf,
    uint32_t *plen, void **pparg)
{
	*ppbuf = frag->pbuf;
	*plen = frag->len;
	*pparg = frag->arg;
	return;
}

struct bsd_tq_frag *
bsd_tq_frag_get_free(struct bsd_tq_fifo *queue)
{
	struct bsd_tq_frag *frag;

	frag = TAILQ_FIRST(&queue->free_q);
	if (frag != NULL) {
		TAILQ_REMOVE(&queue->free_q, frag, entry);
	}
	return (frag);
}

struct bsd_tq_frag *
bsd_tq_frag_get_used(struct bsd_tq_fifo *queue)
{
	struct bsd_tq_frag *frag;

	frag = TAILQ_FIRST(&queue->used_q);
	if (frag != NULL) {
		TAILQ_REMOVE(&queue->used_q, frag, entry);
	}
	return (frag);
}

struct bsd_tq_frag *
bsd_tq_frag_get_temp(struct bsd_tq_fifo *queue)
{
	struct bsd_tq_frag *frag;

	frag = TAILQ_FIRST(&queue->temp_q);
	if (frag != NULL) {
		TAILQ_REMOVE(&queue->temp_q, frag, entry);
	}
	return (frag);
}

struct bsd_tq_frag *
bsd_tq_frag_get_done(struct bsd_tq_fifo *queue)
{
	struct bsd_tq_frag *frag;

	frag = TAILQ_FIRST(&queue->done_q);
	if (frag != NULL) {
		TAILQ_REMOVE(&queue->done_q, frag, entry);
	}
	return (frag);
}

void
bsd_tq_frag_put_free(struct bsd_tq_fifo *queue, struct bsd_tq_frag *frag)
{
	TAILQ_INSERT_TAIL(&queue->free_q, frag, entry);
	return;
}

void
bsd_tq_frag_put_used(struct bsd_tq_fifo *queue, struct bsd_tq_frag *frag)
{
	TAILQ_INSERT_TAIL(&queue->used_q, frag, entry);
	return;
}

void
bsd_tq_frag_put_temp(struct bsd_tq_fifo *queue, struct bsd_tq_frag *frag)
{
	TAILQ_INSERT_TAIL(&queue->temp_q, frag, entry);
	return;
}

void
bsd_tq_frag_put_done(struct bsd_tq_fifo *queue, struct bsd_tq_frag *frag)
{
	TAILQ_INSERT_TAIL(&queue->done_q, frag, entry);
	return;
}

void
bsd_tq_fifo_init(struct bsd_tq_fifo *queue, struct bsd_tq_frag *pfrags,
    uint32_t nfrags)
{
	uint32_t n;

	TAILQ_INIT(&queue->free_q);
	TAILQ_INIT(&queue->used_q);
	TAILQ_INIT(&queue->temp_q);
	TAILQ_INIT(&queue->done_q);

	for (n = 0; n != nfrags; n++) {
		bsd_tq_frag_put_free(queue, pfrags + n);
	}
	return;
}

void
bsd_tq_fifo_uninit(struct bsd_tq_fifo *queue)
{
	return;
}
