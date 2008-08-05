/*-
 * Copyright (c) 1996 Charles M. Hannum. All rights reserved.
 * Copyright (c) 1996 Christopher G. Demetriou. All rights reserved.
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

#ifndef _BSD_MODULE_BUSDMA_H_
#define	_BSD_MODULE_BUSDMA_H_

typedef void *bus_dmamap_t;
typedef void *bus_dma_tag_t;

typedef void *bus_dma_filter_t;

typedef struct bus_dma_segment {
	bus_addr_t ds_addr;		/* DMA address */
	bus_size_t ds_len;		/* length of transfer */
} bus_dma_segment_t;

typedef void bus_dmamap_callback_t (void *, bus_dma_segment_t *, int, int);

typedef enum {
	BUS_DMA_LOCK = 0x01,
	BUS_DMA_UNLOCK = 0x02,
} bus_dma_lock_op_t;

typedef void bus_dma_lock_t (void *, bus_dma_lock_op_t);

int	bus_dma_tag_create(bus_dma_tag_t parent, bus_size_t alignment, bus_size_t boundary, bus_addr_t lowaddr, bus_addr_t highaddr, bus_dma_filter_t *filtfunc, void *filtfuncarg, bus_size_t maxsize, int nsegments, bus_size_t maxsegsz, int flags, bus_dma_lock_t *lockfunc, void *lockfuncarg, bus_dma_tag_t *dmat);
int	bus_dma_tag_destroy(bus_dma_tag_t dmat);
int	bus_dmamap_create(bus_dma_tag_t dmat, int flags, bus_dmamap_t *mapp);
int	bus_dmamap_destroy(bus_dma_tag_t dmat, bus_dmamap_t map);
int	bus_dmamap_load(bus_dma_tag_t dmat, bus_dmamap_t map, void *buf, bus_size_t buflen, bus_dmamap_callback_t *callback, void *callback_arg, int flags);
void	bus_dmamap_unload(bus_dma_tag_t dmat, bus_dmamap_t map);
void	bus_dmamap_sync(bus_dma_tag_t dmat, bus_dmamap_t map, int op);
int	bus_dmamem_alloc(bus_dma_tag_t dmat, void **vaddr, int flags, bus_dmamap_t *mapp);
void	bus_dmamem_free(bus_dma_tag_t dmat, void *vaddr, bus_dmamap_t map);

#define	BUS_DMASYNC_PREREAD 0x0001
#define	BUS_DMASYNC_PREWRITE 0x0002
#define	BUS_DMASYNC_POSTREAD 0x0004
#define	BUS_DMASYNC_POSTWRITE 0x0008

#define	BUS_DMA_WAITOK M_WAITOK
#define	BUS_DMA_COHERENT 0x1000

#define	BUS_SPACE_MAXADDR 0xFFFFFFFF	/* 32-bit */

#endif					/* _BSD_MODULE_BUSDMA_H_ */
