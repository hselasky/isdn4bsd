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

#include <dev/usb2/core/usb2_core.h>
#include <dev/usb2/core/usb2_busdma.h>
#include <dev/usb2/core/usb2_process.h>
#include <dev/usb2/core/usb2_transfer.h>

#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_error.h>
#include <dev/usb2/include/usb2_standard.h>

static void usb2_dma_tag_create(struct usb2_dma_tag *udt, uint32_t size, uint32_t align);
static void usb2_dma_tag_destroy(struct usb2_dma_tag *udt);

#ifdef __FreeBSD__
static void usb2_dma_lock_cb(void *arg, bus_dma_lock_op_t op);
static int32_t usb2_m_copy_in_cb(void *arg, void *src, uint32_t count);
static void usb2_pc_alloc_mem_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error);
static void usb2_pc_load_mem_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error);
static void usb2_pc_common_mem_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error, uint8_t isload);

#endif

#ifdef __NetBSD__
static int32_t usb2_m_copy_in_cb(void *arg, caddr_t src, uint32_t count);
static void usb2_pc_common_mem_cb(struct usb2_page_cache *pc, bus_dma_segment_t *segs, int nseg, int error, uint8_t isload);

#endif

/*------------------------------------------------------------------------*
 *  usb2_get_page - lookup DMA-able memory for the given offset
 *
 * NOTE: Only call this function when the "page_cache" structure has
 * been properly initialized !
 *------------------------------------------------------------------------*/
void
usb2_get_page(struct usb2_page_cache *pc, uint32_t offset,
    struct usb2_page_search *res)
{
	struct usb2_page *page;

	if (pc->page_start) {

		/* Case 1 - something has been loaded into DMA */

		if (pc->buffer) {

			/* Case 1a - Kernel Virtual Address */

			res->buffer = USB_ADD_BYTES(pc->buffer, offset);
		}
		offset += pc->page_offset_buf;

		/* compute destination page */

		page = pc->page_start;

		if (pc->ismultiseg) {

			page += (offset / USB_PAGE_SIZE);

			offset %= USB_PAGE_SIZE;

			res->length = USB_PAGE_SIZE - offset;
			res->physaddr = page->physaddr + offset;
		} else {
			res->length = 0 - 1;
			res->physaddr = page->physaddr + offset;
		}
		if (!pc->buffer) {

			/* Case 1b - Non Kernel Virtual Address */

			res->buffer = USB_ADD_BYTES(page->buffer, offset);
		}
	} else {

		/* Case 2 - Plain PIO */

		res->buffer = USB_ADD_BYTES(pc->buffer, offset);
		res->length = 0 - 1;
		res->physaddr = 0;
	}
	return;
}

/*------------------------------------------------------------------------*
 *  usb2_copy_in - copy directly to DMA-able memory
 *------------------------------------------------------------------------*/
void
usb2_copy_in(struct usb2_page_cache *cache, uint32_t offset,
    const void *ptr, uint32_t len)
{
	struct usb2_page_search buf_res;

	while (len != 0) {

		usb2_get_page(cache, offset, &buf_res);

		if (buf_res.length > len) {
			buf_res.length = len;
		}
		bcopy(ptr, buf_res.buffer, buf_res.length);

		offset += buf_res.length;
		len -= buf_res.length;
		ptr = USB_ADD_BYTES(ptr, buf_res.length);
	}
	return;
}

/*------------------------------------------------------------------------*
 *  usb2_m_copy_in - copy a mbuf chain directly into DMA-able memory
 *------------------------------------------------------------------------*/
struct usb2_m_copy_in_arg {
	struct usb2_page_cache *cache;
	uint32_t dst_offset;
};

static int32_t
#ifdef __FreeBSD__
usb2_m_copy_in_cb(void *arg, void *src, uint32_t count)
#else
usb2_m_copy_in_cb(void *arg, caddr_t src, uint32_t count)
#endif
{
	register struct usb2_m_copy_in_arg *ua = arg;

	usb2_copy_in(ua->cache, ua->dst_offset, src, count);
	ua->dst_offset += count;
	return (0);
}

void
usb2_m_copy_in(struct usb2_page_cache *cache, uint32_t dst_offset,
    struct mbuf *m, uint32_t src_offset, uint32_t src_len)
{
	struct usb2_m_copy_in_arg arg = {cache, dst_offset};
	register int error;

	error = m_apply(m, src_offset, src_len, &usb2_m_copy_in_cb, &arg);
	return;
}

/*------------------------------------------------------------------------*
 *  usb2_uiomove - factored out code
 *------------------------------------------------------------------------*/
int
usb2_uiomove(struct usb2_page_cache *pc, struct uio *uio,
    uint32_t pc_offset, uint32_t len)
{
	struct usb2_page_search res;
	int error = 0;

	while (len != 0) {

		usb2_get_page(pc, pc_offset, &res);

		if (res.length > len) {
			res.length = len;
		}
		/*
		 * "uiomove()" can sleep so one needs to make a wrapper,
		 * exiting the mutex and checking things
		 */
		error = uiomove(res.buffer, res.length, uio);

		if (error) {
			break;
		}
		pc_offset += res.length;
		len -= res.length;
	}
	return (error);
}

/*------------------------------------------------------------------------*
 *  usb2_copy_out - copy directly from DMA-able memory
 *------------------------------------------------------------------------*/
void
usb2_copy_out(struct usb2_page_cache *cache, uint32_t offset,
    void *ptr, uint32_t len)
{
	struct usb2_page_search res;

	while (len != 0) {

		usb2_get_page(cache, offset, &res);

		if (res.length > len) {
			res.length = len;
		}
		bcopy(res.buffer, ptr, res.length);

		offset += res.length;
		len -= res.length;
		ptr = USB_ADD_BYTES(ptr, res.length);
	}
	return;
}

/*------------------------------------------------------------------------*
 *  usb2_bzero - zero DMA-able memory
 *------------------------------------------------------------------------*/
void
usb2_bzero(struct usb2_page_cache *cache, uint32_t offset, uint32_t len)
{
	struct usb2_page_search res;

	while (len != 0) {

		usb2_get_page(cache, offset, &res);

		if (res.length > len) {
			res.length = len;
		}
		bzero(res.buffer, res.length);

		offset += res.length;
		len -= res.length;
	}
	return;
}


#ifdef __FreeBSD__

/*------------------------------------------------------------------------*
 *	usb2_dma_lock_cb - dummy callback
 *------------------------------------------------------------------------*/
static void
usb2_dma_lock_cb(void *arg, bus_dma_lock_op_t op)
{
	/* we use "mtx_owned()" instead of this function */
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_dma_tag_create - allocate a DMA tag
 *
 * NOTE: If the "align" parameter has a value of 1 the DMA-tag will
 * allow multi-segment mappings. Else all mappings are single-segment.
 *------------------------------------------------------------------------*/
static void
usb2_dma_tag_create(struct usb2_dma_tag *udt,
    uint32_t size, uint32_t align)
{
	bus_dma_tag_t tag;

	if (bus_dma_tag_create
	    ( /* parent    */ udt->tag_parent->tag,
	     /* alignment */ align,
	     /* boundary  */ 0,
	     /* lowaddr   */ (2ULL << (udt->tag_parent->dma_bits - 1)) - 1,
	     /* highaddr  */ BUS_SPACE_MAXADDR,
	     /* filter    */ NULL,
	     /* filterarg */ NULL,
	     /* maxsize   */ size,
	     /* nsegments */ (align == 1) ?
	    (2 + (size / USB_PAGE_SIZE)) : 1,
	     /* maxsegsz  */ (align == 1) ?
	    USB_PAGE_SIZE : size,
	     /* flags     */ 0,
	     /* lockfn    */ &usb2_dma_lock_cb,
	     /* lockarg   */ NULL,
	    &tag)) {
		tag = NULL;
	}
	udt->tag = tag;
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_dma_tag_free - free a DMA tag
 *------------------------------------------------------------------------*/
static void
usb2_dma_tag_destroy(struct usb2_dma_tag *udt)
{
	bus_dma_tag_destroy(udt->tag);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_alloc_mem_cb - BUS-DMA callback function
 *------------------------------------------------------------------------*/
static void
usb2_pc_alloc_mem_cb(void *arg, bus_dma_segment_t *segs,
    int nseg, int error)
{
	usb2_pc_common_mem_cb(arg, segs, nseg, error, 0);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_load_mem_cb - BUS-DMA callback function
 *------------------------------------------------------------------------*/
static void
usb2_pc_load_mem_cb(void *arg, bus_dma_segment_t *segs,
    int nseg, int error)
{
	usb2_pc_common_mem_cb(arg, segs, nseg, error, 1);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_common_mem_cb - BUS-DMA callback function
 *------------------------------------------------------------------------*/
static void
usb2_pc_common_mem_cb(void *arg, bus_dma_segment_t *segs,
    int nseg, int error, uint8_t isload)
{
	struct usb2_dma_parent_tag *uptag;
	struct usb2_page_cache *pc;
	struct usb2_page *pg;
	uint32_t rem;
	uint8_t owned;
	uint8_t ext_seg;		/* extend last segment */

	pc = arg;
	uptag = pc->tag_parent;

	/*
	 * XXX There is sometimes recursive locking here.
	 * XXX We should try to find a better solution.
	 * XXX Until further the "owned" variable does
	 * XXX the trick.
	 */

	if (error) {
		goto done;
	}
	pg = pc->page_start;
	pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	rem = segs->ds_addr & (USB_PAGE_SIZE - 1);
	pc->page_offset_buf = rem;
	pc->page_offset_end += rem;
	if (nseg < ((pc->page_offset_end +
	    (USB_PAGE_SIZE - 1)) / USB_PAGE_SIZE)) {
		ext_seg = 1;
	} else {
		ext_seg = 0;
	}
	nseg--;

	while (nseg > 0) {
		nseg--;
		segs++;
		pg++;
		pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	}

	/*
	 * XXX The segments we get from BUS-DMA are not aligned,
	 * XXX so we need to extend the last segment if we are
	 * XXX unaligned and cross the segment boundary!
	 */
	if (ext_seg && pc->ismultiseg) {
		(pg + 1)->physaddr = pg->physaddr + USB_PAGE_SIZE;
	}
done:
	owned = mtx_owned(uptag->mtx);
	if (!owned)
		mtx_lock(uptag->mtx);

	uptag->dma_error = (error ? 1 : 0);
	if (isload) {
		(uptag->func) (uptag);
	} else {
		cv_broadcast(uptag->cv);
	}
	if (!owned)
		mtx_unlock(uptag->mtx);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_alloc_mem - allocate DMA'able memory
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usb2_pc_alloc_mem(struct usb2_page_cache *pc, struct usb2_page *pg,
    uint32_t size, uint32_t align)
{
	struct usb2_dma_parent_tag *uptag;
	struct usb2_dma_tag *utag;
	bus_dmamap_t map;
	void *ptr;
	int err;

	uptag = pc->tag_parent;

	if (align != 1) {
		/*
	         * The alignment must be greater or equal to the
	         * "size" else the object can be split between two
	         * memory pages and we get a problem!
	         */
		while (align < size) {
			align *= 2;
			if (align == 0) {
				goto error;
			}
		}
#if 1
		/*
		 * XXX BUS-DMA workaround - FIXME later:
		 *
		 * We assume that that the aligment at this point of
		 * the code is greater than or equal to the size and
		 * less than two times the size, so that if we double
		 * the size, the size will be greater than the
		 * alignment.
		 *
		 * The bus-dma system has a check for "alignment"
		 * being less than "size". If that check fails we end
		 * up using contigmalloc which is page based even for
		 * small allocations. Try to avoid that to save
		 * memory, hence we sometimes to a large number of
		 * small allocations!
		 */
		if (size < USB_PAGE_SIZE) {
			size *= 2;
		}
#endif
	}
	/* get the correct DMA tag */
	utag = usb2_dma_tag_find(uptag, size, align);
	if (utag == NULL) {
		goto error;
	}
	/* allocate memory */
	if (bus_dmamem_alloc(
	    utag->tag, &ptr, (BUS_DMA_WAITOK | BUS_DMA_COHERENT), &map)) {
		goto error;
	}
	/* setup page cache */
	pc->buffer = ptr;
	pc->page_start = pg;
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;
	pc->map = map;
	pc->tag = utag->tag;
	pc->ismultiseg = (align == 1);

	mtx_lock(uptag->mtx);

	/* load memory into DMA */
	err = bus_dmamap_load(
	    utag->tag, map, ptr, size, &usb2_pc_alloc_mem_cb,
	    pc, (BUS_DMA_WAITOK | BUS_DMA_COHERENT));

	if (err == EINPROGRESS) {
		cv_wait(uptag->cv, uptag->mtx);
		err = 0;
	}
	mtx_unlock(uptag->mtx);

	if (err || uptag->dma_error) {
		bus_dmamem_free(utag->tag, ptr, map);
		goto error;
	}
	bzero(ptr, size);

	usb2_pc_cpu_flush(pc);

	return (0);

error:
	/* reset most of the page cache */
	pc->buffer = NULL;
	pc->page_start = NULL;
	pc->page_offset_buf = 0;
	pc->page_offset_end = 0;
	pc->map = NULL;
	pc->tag = NULL;
	return (1);
}

/*------------------------------------------------------------------------*
 *	usb2_pc_free_mem - free DMA memory
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usb2_pc_free_mem(struct usb2_page_cache *pc)
{
	if (pc && pc->buffer) {

		bus_dmamap_unload(pc->tag, pc->map);

		bus_dmamem_free(pc->tag, pc->buffer, pc->map);

		pc->buffer = NULL;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_load_mem - load virtual memory into DMA
 *
 * Return values:
 * 0: Success
 * Else: Error
 *------------------------------------------------------------------------*/
uint8_t
usb2_pc_load_mem(struct usb2_page_cache *pc, uint32_t size, uint8_t sync)
{
	/* setup page cache */
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;
	pc->ismultiseg = 1;

	mtx_assert(pc->tag_parent->mtx, MA_OWNED);

	if (size > 0) {
		if (sync) {
			struct usb2_dma_parent_tag *uptag;
			int err;

			uptag = pc->tag_parent;

			/*
			 * Try to load memory into DMA.
			 */
			mtx_lock(uptag->mtx);
			err = bus_dmamap_load(
			    pc->tag, pc->map, pc->buffer, size,
			    &usb2_pc_alloc_mem_cb, pc, BUS_DMA_WAITOK);
			if (err == EINPROGRESS) {
				cv_wait(uptag->cv, uptag->mtx);
				err = 0;
			}
			mtx_unlock(uptag->mtx);

			if (err || uptag->dma_error) {
				return (1);
			}
		} else {

			/*
			 * Try to load memory into DMA. The callback
			 * will be called in all cases:
			 */
			if (bus_dmamap_load(
			    pc->tag, pc->map, pc->buffer, size,
			    &usb2_pc_load_mem_cb, pc, BUS_DMA_WAITOK)) {
			}
		}
	} else {
		if (!sync) {
			/*
			 * Call callback so that refcount is decremented
			 * properly:
			 */
			pc->tag_parent->dma_error = 0;
			(pc->tag_parent->func) (pc->tag_parent);
		}
	}
	return (0);
}

/*------------------------------------------------------------------------*
 *	usb2_pc_cpu_invalidate - invalidate CPU cache
 *------------------------------------------------------------------------*/
void
usb2_pc_cpu_invalidate(struct usb2_page_cache *pc)
{
	bus_dmamap_sync(pc->tag, pc->map,
	    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_cpu_flush - flush CPU cache
 *------------------------------------------------------------------------*/
void
usb2_pc_cpu_flush(struct usb2_page_cache *pc)
{
	bus_dmamap_sync(pc->tag, pc->map,
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_dmamap_create - create a DMA map
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usb2_pc_dmamap_create(struct usb2_page_cache *pc, uint32_t size)
{
	struct usb2_xfer_root *info;
	struct usb2_dma_tag *utag;

	/* get info */
	info = pc->tag_parent->info;

	/* sanity check */
	if (info == NULL) {
		goto error;
	}
	utag = usb2_dma_tag_find(pc->tag_parent, size, 1);
	if (utag == NULL) {
		goto error;
	}
	/* create DMA map */
	if (bus_dmamap_create(utag->tag, 0, &(pc->map))) {
		goto error;
	}
	pc->tag = utag->tag;
	return 0;			/* success */

error:
	pc->map = NULL;
	pc->tag = NULL;
	return 1;			/* failure */
}

/*------------------------------------------------------------------------*
 *	usb2_pc_dmamap_destroy
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usb2_pc_dmamap_destroy(struct usb2_page_cache *pc)
{
	if (pc && pc->tag) {
		bus_dmamap_destroy(pc->tag, pc->map);
		pc->tag = NULL;
		pc->map = NULL;
	}
	return;
}

#endif

#ifdef __NetBSD__

/*------------------------------------------------------------------------*
 *	usb2_dma_tag_create - allocate a DMA tag
 *
 * NOTE: If the "align" parameter has a value of 1 the DMA-tag will
 * allow multi-segment mappings. Else all mappings are single-segment.
 *------------------------------------------------------------------------*/
static void
usb2_dma_tag_create(struct usb2_dma_tag *udt,
    uint32_t size, uint32_t align)
{
	uint32_t nseg;

	if (align == 1) {
		nseg = (2 + (size / USB_PAGE_SIZE));
	} else {
		nseg = 1;
	}

	udt->p_seg = malloc(nseg * sizeof(*(udt->p_seg)),
	    M_USB, M_WAITOK | M_ZERO);

	if (udt->p_seg == NULL) {
		return;
	}
	udt->tag = udt->tag_parent->tag;
	udt->n_seg = nseg;
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_dma_tag_free - free a DMA tag
 *------------------------------------------------------------------------*/
static void
usb2_dma_tag_destroy(struct usb2_dma_tag *udt)
{
	free(udt->p_seg, M_USB);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_common_mem_cb - BUS-DMA callback function
 *------------------------------------------------------------------------*/
static void
usb2_pc_common_mem_cb(struct usb2_page_cache *pc, bus_dma_segment_t *segs,
    int nseg, int error, uint8_t isload, uint8_t dolock)
{
	struct usb2_dma_parent_tag *uptag;
	struct usb2_page *pg;
	uint32_t rem;
	uint8_t ext_seg;		/* extend last segment */

	uptag = pc->tag_parent;

	if (error) {
		goto done;
	}
	pg = pc->page_start;
	pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	rem = segs->ds_addr & (USB_PAGE_SIZE - 1);
	pc->page_offset_buf = rem;
	pc->page_offset_end += rem;
	if (nseg < ((pc->page_offset_end +
	    (USB_PAGE_SIZE - 1)) / USB_PAGE_SIZE)) {
		ext_seg = 1;
	} else {
		ext_seg = 0;
	}
	nseg--;

	while (nseg > 0) {
		nseg--;
		segs++;
		pg++;
		pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	}

	/*
	 * XXX The segments we get from BUS-DMA are not aligned,
	 * XXX so we need to extend the last segment if we are
	 * XXX unaligned and cross the segment boundary!
	 */
	if (ext_seg && pc->ismultiseg) {
		(pg + 1)->physaddr = pg->physaddr + USB_PAGE_SIZE;
	}
done:
	if (dolock)
		mtx_lock(uptag->mtx);

	uptag->dma_error = (error ? 1 : 0);
	if (isload) {
		(uptag->func) (uptag);
	}
	if (dolock)
		mtx_unlock(uptag->mtx);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_alloc_mem - allocate DMA'able memory
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usb2_pc_alloc_mem(struct usb2_page_cache *pc, struct usb2_page *pg,
    uint32_t size, uint32_t align)
{
	struct usb2_dma_parent_tag *uptag;
	struct usb2_dma_tag *utag;
	caddr_t ptr = NULL;
	bus_dmamap_t map;
	int seg_count;

	uptag = pc->tag_parent;

	if (align != 1) {
		/*
	         * The alignment must be greater or equal to the
	         * "size" else the object can be split between two
	         * memory pages and we get a problem!
	         */
		while (align < size) {
			align *= 2;
			if (align == 0) {
				goto done_5;
			}
		}
	}
	/* get the correct DMA tag */
	utag = usb2_dma_tag_find(pc->tag_parent, size, align);
	if (utag == NULL) {
		goto done_5;
	}
	if (bus_dmamem_alloc(utag->tag, size, align, 0, utag->p_seg,
	    utag->n_seg, &seg_count, BUS_DMA_WAITOK)) {
		goto done_4;
	}
	if (bus_dmamem_map(utag->tag, utag->p_seg, seg_count, size,
	    &ptr, BUS_DMA_WAITOK | BUS_DMA_COHERENT)) {
		goto done_3;
	}
	if (bus_dmamap_create(utag->tag, size, utag->n_seg, (align == 1) ?
	    USB_PAGE_SIZE : size, 0, BUS_DMA_WAITOK, &map)) {
		goto done_2;
	}
	if (bus_dmamap_load(utag->tag, map, ptr, size, NULL,
	    BUS_DMA_WAITOK)) {
		goto done_1;
	}
	pc->p_seg = malloc(seg_count * sizeof(*(pc->p_seg)),
	    M_USB, M_WAITOK | M_ZERO);
	if (pc->p_seg == NULL) {
		goto done_0;
	}
	/* store number if actual segments used */
	pc->n_seg = seg_count;

	/* make a copy of the segments */
	bcopy(utag->p_seg, pc->p_seg,
	    seg_count * sizeof(*(pc->p_seg)));

	/* setup page cache */
	pc->buffer = ptr;
	pc->page_start = pg;
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;
	pc->map = map;
	pc->tag = utag->tag;
	pc->ismultiseg = (align == 1);

	usb2_pc_common_mem_cb(pc, utag->p_seg, seg_count, 0, 0, 1);

	bzero(ptr, size);

	usb2_pc_cpu_flush(pc);

	return (0);

done_0:
	bus_dmamap_unload(utag->tag, map);
done_1:
	bus_dmamap_destroy(utag->tag, map);
done_2:
	bus_dmamem_unmap(utag->tag, ptr, size);
done_3:
	bus_dmamem_free(utag->tag, utag->p_seg, seg_count);
done_4:
	/* utag is destroyed later */
done_5:
	/* reset most of the page cache */
	pc->buffer = NULL;
	pc->page_start = NULL;
	pc->page_offset_buf = 0;
	pc->page_offset_end = 0;
	pc->map = NULL;
	pc->tag = NULL;
	pc->n_seg = 0;
	pc->p_seg = NULL;
	return (1);
}

/*------------------------------------------------------------------------*
 *	usb2_pc_free_mem - free DMA memory
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usb2_pc_free_mem(struct usb2_page_cache *pc)
{
	if (pc && pc->buffer) {
		bus_dmamap_unload(pc->tag, pc->map);
		bus_dmamap_destroy(pc->tag, pc->map);
		bus_dmamem_unmap(pc->tag, pc->buffer,
		    pc->page_offset_end - pc->page_offset_buf);
		bus_dmamem_free(pc->tag, pc->p_seg, pc->n_seg);
		free(pc->p_seg, M_USB);
		pc->buffer = NULL;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_load_mem - load virtual memory into DMA
 *
 * Return values:
 * 0: Success
 * Else: Error
 *------------------------------------------------------------------------*/
uint8_t
usb2_pc_load_mem(struct usb2_page_cache *pc, uint32_t size, uint8_t sync)
{
	int error;

	/* setup page cache */
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;
	pc->ismultiseg = 1;

	if (size > 0) {

		/* try to load memory into DMA using using no wait option */
		if (bus_dmamap_load(pc->tag, pc->map, pc->buffer,
		    size, NULL, BUS_DMA_NOWAIT)) {
			error = ENOMEM;
		} else {
			error = 0;
		}

		usb2_pc_common_mem_cb(pc, pc->map->dm_segs,
		    pc->map->dm_nsegs, error, !sync);

		if (error) {
			return (1);
		}
	} else {
		if (!sync) {
			/*
			 * Call callback so that refcount is decremented
			 * properly:
			 */
			pc->tag_parent->dma_error = 0;
			(pc->tag_parent->func) (pc->tag_parent);
		}
	}
	return (0);
}

/*------------------------------------------------------------------------*
 *	usb2_pc_cpu_invalidate - invalidate CPU cache
 *------------------------------------------------------------------------*/
void
usb2_pc_cpu_invalidate(struct usb2_page_cache *pc)
{
	uint32_t len;

	len = pc->page_offset_end - pc->page_offset_buf;

	bus_dmamap_sync(pc->tag, pc->map, 0, len,
	    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_cpu_flush - flush CPU cache
 *------------------------------------------------------------------------*/
void
usb2_pc_cpu_flush(struct usb2_page_cache *pc)
{
	uint32_t len;

	len = pc->page_offset_end - pc->page_offset_buf;

	bus_dmamap_sync(pc->tag, pc->map, 0, len,
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_pc_dmamap_create - create a DMA map
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usb2_pc_dmamap_create(struct usb2_page_cache *pc, uint32_t size)
{
	struct usb2_xfer_root *info;
	struct usb2_dma_tag *utag;

	/* get info */
	info = pc->tag_parent->info;

	/* sanity check */
	if (info == NULL) {
		goto error;
	}
	utag = usb2_dma_tag_find(pc->tag_parent, size, 1);
	if (utag == NULL) {
		goto error;
	}
	if (bus_dmamap_create(utag->tag, size, utag->n_seg,
	    USB_PAGE_SIZE, 0, BUS_DMA_WAITOK, &(pc->map))) {
		goto error;
	}
	pc->tag = utag->tag;
	pc->p_seg = utag->p_seg;
	pc->n_seg = utag->n_seg;
	return 0;			/* success */

error:
	pc->map = NULL;
	pc->tag = NULL;
	pc->p_seg = NULL;
	pc->n_seg = 0;
	return 1;			/* failure */
}

/*------------------------------------------------------------------------*
 *	usb2_pc_dmamap_destroy
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usb2_pc_dmamap_destroy(struct usb2_page_cache *pc)
{
	if (pc && pc->tag) {
		bus_dmamap_destroy(pc->tag, pc->map);
		pc->tag = NULL;
		pc->map = NULL;
	}
	return;
}

#endif

/*------------------------------------------------------------------------*
 *	usb2_dma_tag_find - factored out code
 *------------------------------------------------------------------------*/
struct usb2_dma_tag *
usb2_dma_tag_find(struct usb2_dma_parent_tag *udpt,
    uint32_t size, uint32_t align)
{
	struct usb2_dma_tag *udt;
	uint8_t nudt;

	USB_ASSERT(align > 0, ("Invalid parameter align = 0!\n"));
	USB_ASSERT(size > 0, ("Invalid parameter size = 0!\n"));

	udt = udpt->utag_first;
	nudt = udpt->utag_max;

	while (nudt--) {

		if (udt->align == 0) {
			usb2_dma_tag_create(udt, size, align);
			if (udt->tag == NULL) {
				return (NULL);
			}
			udt->align = align;
			udt->size = size;
			return (udt);
		}
		if ((udt->align == align) && (udt->size == size)) {
			return (udt);
		}
		udt++;
	}
	return (NULL);
}

/*------------------------------------------------------------------------*
 *	usb2_dma_tag_setup - initialise USB DMA tags
 *------------------------------------------------------------------------*/
void
usb2_dma_tag_setup(struct usb2_dma_parent_tag *udpt,
    struct usb2_dma_tag *udt, bus_dma_tag_t dmat,
    struct mtx *mtx, usb2_dma_callback_t *func,
    struct usb2_xfer_root *info, uint8_t ndmabits,
    uint8_t nudt)
{
	bzero(udpt, sizeof(*udpt));

	/* sanity checking */
	if ((nudt == 0) ||
	    (ndmabits == 0) ||
	    (mtx == NULL)) {
		/* something is corrupt */
		return;
	}
#ifdef __FreeBSD__
	/* initialise condition variable */
	cv_init(udpt->cv, "USB DMA CV");
#endif

	/* store some information */
	udpt->mtx = mtx;
	udpt->info = info;
	udpt->func = func;
	udpt->tag = dmat;
	udpt->utag_first = udt;
	udpt->utag_max = nudt;
	udpt->dma_bits = ndmabits;

	while (nudt--) {
		bzero(udt, sizeof(*udt));
		udt->tag_parent = udpt;
		udt++;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_bus_tag_unsetup - factored out code
 *------------------------------------------------------------------------*/
void
usb2_dma_tag_unsetup(struct usb2_dma_parent_tag *udpt)
{
	struct usb2_dma_tag *udt;
	uint8_t nudt;

	udt = udpt->utag_first;
	nudt = udpt->utag_max;

	while (nudt--) {

		if (udt->align) {
			/* destroy the USB DMA tag */
			usb2_dma_tag_destroy(udt);
			udt->align = 0;
		}
		udt++;
	}

	if (udpt->utag_max) {
#ifdef __FreeBSD__
		/* destroy the condition variable */
		cv_destroy(udpt->cv);
#endif
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_bdma_work_loop
 *
 * This function handles loading of virtual buffers into DMA and is
 * only called when "dma_refcount" is zero.
 *------------------------------------------------------------------------*/
void
usb2_bdma_work_loop(struct usb2_xfer_queue *pq)
{
	struct usb2_xfer_root *info;
	struct usb2_xfer *xfer;
	uint32_t nframes;

	xfer = pq->curr;
	info = xfer->usb2_root;

	mtx_assert(info->priv_mtx, MA_OWNED);

	if (xfer->error) {
		/* some error happened */
		mtx_lock(xfer->usb2_mtx);
		usb2_transfer_done(xfer, 0);
		mtx_unlock(xfer->usb2_mtx);
		return;
	}
	if (!xfer->flags_int.bdma_setup) {
		struct usb2_page *pg;
		uint32_t frlength_0;
		uint8_t isread;

		xfer->flags_int.bdma_setup = 1;

		/* reset BUS-DMA load state */

		info->dma_error = 0;

		if (xfer->flags_int.isochronous_xfr) {
			/* only one frame buffer */
			nframes = 1;
			frlength_0 = xfer->sumlen;
		} else {
			/* can be multiple frame buffers */
			nframes = xfer->nframes;
			frlength_0 = xfer->frlengths[0];
		}

		/*
		 * Set DMA direction first. This is needed to
		 * select the correct cache invalidate and cache
		 * flush operations.
		 */
		isread = USB_GET_DATA_ISREAD(xfer);
		pg = xfer->dma_page_ptr;

		if (xfer->flags_int.control_xfr &&
		    xfer->flags_int.control_hdr) {
			/* special case */
			if (xfer->flags_int.usb2_mode == USB_MODE_DEVICE) {
				/* The device controller writes to memory */
				xfer->frbuffers[0].isread = 1;
			} else {
				/* The host controller reads from memory */
				xfer->frbuffers[0].isread = 0;
			}
		} else {
			/* default case */
			xfer->frbuffers[0].isread = isread;
		}

		/*
		 * Setup the "page_start" pointer which points to an array of
		 * USB pages where information about the physical address of a
		 * page will be stored. Also initialise the "isread" field of
		 * the USB page caches.
		 */
		xfer->frbuffers[0].page_start = pg;

		info->dma_nframes = nframes;
		info->dma_currframe = 0;
		info->dma_frlength_0 = frlength_0;

		pg += (frlength_0 / USB_PAGE_SIZE);
		pg += 2;

		while (--nframes > 0) {
			xfer->frbuffers[nframes].isread = isread;
			xfer->frbuffers[nframes].page_start = pg;

			pg += (xfer->frlengths[nframes] / USB_PAGE_SIZE);
			pg += 2;
		}

	}
	if (info->dma_error) {
		mtx_lock(xfer->usb2_mtx);
		usb2_transfer_done(xfer, USB_ERR_DMA_LOAD_FAILED);
		mtx_unlock(xfer->usb2_mtx);
		return;
	}
	if (info->dma_currframe != info->dma_nframes) {

		if (info->dma_currframe == 0) {
			/* special case */
			usb2_pc_load_mem(xfer->frbuffers,
			    info->dma_frlength_0, 0);
		} else {
			/* default case */
			nframes = info->dma_currframe;
			usb2_pc_load_mem(xfer->frbuffers + nframes,
			    xfer->frlengths[nframes], 0);
		}

		/* advance frame index */
		info->dma_currframe++;

		return;
	}
	/* go ahead */
	usb2_bdma_pre_sync(xfer);

	/* start loading next USB transfer, if any */
	usb2_command_wrapper(pq, NULL);

	/* finally start the hardware */
	usb2_pipe_enter(xfer);

	return;
}

/*------------------------------------------------------------------------*
 *	usb2_bdma_done_event
 *
 * This function is called when the BUS-DMA has loaded virtual memory
 * into DMA, if any.
 *------------------------------------------------------------------------*/
void
usb2_bdma_done_event(struct usb2_dma_parent_tag *udpt)
{
	struct usb2_xfer_root *info;

	info = udpt->info;

	mtx_assert(info->priv_mtx, MA_OWNED);

	/* copy error */
	info->dma_error = udpt->dma_error;

	/* enter workloop again */
	usb2_command_wrapper(&(info->dma_q),
	    info->dma_q.curr);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_bdma_pre_sync
 *
 * This function handles DMA synchronisation that must be done before
 * an USB transfer is started.
 *------------------------------------------------------------------------*/
void
usb2_bdma_pre_sync(struct usb2_xfer *xfer)
{
	struct usb2_page_cache *pc;
	uint32_t nframes;

	if (xfer->flags_int.isochronous_xfr) {
		/* only one frame buffer */
		nframes = 1;
	} else {
		/* can be multiple frame buffers */
		nframes = xfer->nframes;
	}

	pc = xfer->frbuffers;

	while (nframes--) {

		if (pc->page_offset_buf != pc->page_offset_end) {
			if (pc->isread) {
				usb2_pc_cpu_invalidate(pc);
			} else {
				usb2_pc_cpu_flush(pc);
			}
		}
		pc++;
	}

	return;
}

/*------------------------------------------------------------------------*
 *	usb2_bdma_post_sync
 *
 * This function handles DMA synchronisation that must be done after
 * an USB transfer is complete.
 *------------------------------------------------------------------------*/
void
usb2_bdma_post_sync(struct usb2_xfer *xfer)
{
	struct usb2_page_cache *pc;
	uint32_t nframes;

	if (xfer->flags_int.isochronous_xfr) {
		/* only one frame buffer */
		nframes = 1;
	} else {
		/* can be multiple frame buffers */
		nframes = xfer->nframes;
	}

	pc = xfer->frbuffers;

	while (nframes--) {

		if (pc->page_offset_buf != pc->page_offset_end) {
			if (pc->isread) {
				usb2_pc_cpu_invalidate(pc);
			}
		}
		pc++;
	}
	return;
}