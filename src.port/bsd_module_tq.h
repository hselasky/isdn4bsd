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

typedef void (bsd_tq_cmd_t)(void *arg);	/* called unlocked */

#ifdef __FreeBSD__
struct bsd_tq_frag {
	TAILQ_ENTRY(bsd_tq_frag) entry;
	void   *pbuf;
	uint32_t len;
#define	BSD_TQ_FRAG_ERROR 0x80000000 /* set stall / clear stall */
#define	BSD_TQ_FRAG_SHORT 0x40000000 /* receive short OK / force short TX */
};

struct bsd_tq_fifo {
	TAILQ_HEAD(, bsd_tq_frag) free_q;
	TAILQ_HEAD(, bsd_tq_frag) used_q;
	TAILQ_HEAD(, bsd_tq_frag) temp_q;
	TAILQ_HEAD(, bsd_tq_frag) done_q;
};

#else
struct bsd_tq_frag;
struct bsd_tq_fifo;

#endif

/* prototypes */

struct bsd_tq_frag *bsd_tq_frag_get_free(struct bsd_tq_fifo *queue);	/* X */
struct bsd_tq_frag *bsd_tq_frag_get_used(struct bsd_tq_fifo *queue);	/* X */
struct bsd_tq_frag *bsd_tq_frag_get_temp(struct bsd_tq_fifo *queue);	/* X */
struct bsd_tq_frag *bsd_tq_frag_get_done(struct bsd_tq_fifo *queue);	/* X */
void	bsd_tq_fifo_init(struct bsd_tq_fifo *queue, struct bsd_tq_frag *pfrags, uint32_t nfrags);
void	bsd_tq_fifo_uninit(struct bsd_tq_fifo *queue);
void	bsd_tq_frag_get_buf_len(struct bsd_tq_frag *frag, void **ppbuf, uint32_t *plen);
void	bsd_tq_frag_put_free(struct bsd_tq_fifo *queue, struct bsd_tq_frag *frag);	/* X */
void	bsd_tq_frag_put_used(struct bsd_tq_fifo *queue, struct bsd_tq_frag *frag);	/* X */
void	bsd_tq_frag_put_temp(struct bsd_tq_fifo *queue, struct bsd_tq_frag *frag);	/* X */
void	bsd_tq_frag_put_done(struct bsd_tq_fifo *queue, struct bsd_tq_frag *frag);	/* X */
void	bsd_tq_frag_set_buf_len(struct bsd_tq_frag *frag, void *buf, uint32_t len);

/*
 * X: The queue must be locked before you can add or remove fragments.
 */
