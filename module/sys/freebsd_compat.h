/*-
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
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
 *
 * This file will include files that will emulate the FreeBSD kernel.
 */

#ifndef __FREEBSD_SYS_COMPAT_H__
#define	__FREEBSD_SYS_COMPAT_H__

#if defined(__FreeBSD__)

#include <sys/conf.h>

#ifdef _KERNEL
#include <machine/resource.h>
#include <dev/sound/pcm/sound.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <fs/devfs/devfs.h>
#include <isa/isa_common.h>
#include <isa/isavar.h>
#include <net/if_var.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_message.h>
#include <netgraph/ng_parse.h>
#include <sys/bus.h>
#include <sys/lockmgr.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/rtprio.h>
#include <sys/selinfo.h>
#include <sys/sysctl.h>
#else
#include <ncurses.h>
#include <osreldate.h>
#endif

#ifdef _KERNEL
typedef struct cdevsw cdevsw_t;

#define	__lockmgr lockmgr
#define	__KASSERT KASSERT
#endif

#elif (defined(__NetBSD__) || defined(__OpenBSD__))

#include <sys/cdefs.h>
#include <sys/types.h>

#ifdef _KERNEL
#include <machine/bus.h>
#include <sys/device.h>
#include <sys/lock.h>
#include <sys/queue.h>
#include <sys/sysctl.h>
#include <sys/namei.h>
#include <sys/unistd.h>
#include <sys/select.h>
#include <sys/kernel.h>
#include <sys/reboot.h>
#include <sys/mbuf.h>
#include <net/if.h>
#include <net/if_spppvar.h>
#if (__NetBSD_Version__ >= 400000000)
#include <net80211/ieee80211_netbsd.h>
#include <sys/kauth.h>
#else
#include <net80211/ieee80211_compat.h>
#endif
#include <sys/callout.h>
#else
#include <sys/queue.h>
#endif

#ifdef _KERNEL
typedef struct __cdevsw cdevsw_t;

#ifndef __KASSERT
#define	__KASSERT(exp,msg)	\
do {				\
  if(__predict_false(!(exp)))	\
  {				\
      panic msg;		\
  }				\
} while (0)
#endif

#if (__NetBSD_Version__ < 500000000)
#if defined(LOCKDEBUG)
#define	__lockmgr(l, f, i, p) _lockmgr((l), (f), (i), __FILE__, __LINE__)
#else
#define	__lockmgr(l, f, i, p) lockmgr(l, f, i)
#endif
#endif

#define	thread proc
#define	curthread curproc

#define	WITNESS_WARN(...)

#undef si_uid
#undef si_gid

#ifndef SPECNAMELEN
#define	SPECNAMELEN     63		/* max length of devicename */
#endif

typedef u_int32_t vm_offset_t;
typedef u_int32_t vm_size_t;

typedef u_int64_t vm_ooffset_t;
typedef u_int64_t vm_paddr_t;
typedef u_int64_t vm_pindex_t;

#if (__NetBSD_Version__ >= 500000000)
typedef char *caddr_t;

#endif

#if (__NetBSD_Version__ >= 300000000)
#ifdef PCI_MACHDEP_ENUMERATE_BUS
#define	pci_enumerate_bus PCI_MACHDEP_ENUMERATE_BUS
#else
struct pci_softc;
struct pci_attach_args;
extern int pci_enumerate_bus
        (struct pci_softc *, const int *,
    	int    (*) (struct pci_attach_args *),
    	struct	pci_attach_args *);

#endif
#define	copy_statfs_info copy_statvfs_info
#define	set_statfs_info set_statvfs_info
#define	vfs_statfs vfs_statvfs
#define	statfs statvfs
#endif

extern void i4b_load(void);

#endif

#ifndef __used
#if (__GNUC_PREREQ__(2, 7) || defined(__INTEL_COMPILER))
#define	__used          __attribute__((__used__))
#else
#define	__used
#endif
#endif

#ifndef __printflike
#if (__GNUC_PREREQ__(2, 7) || defined(__INTEL_COMPILER))
#define	__printflike(fmtarg, firstvararg) \
     __attribute__((__format__ (__printf__, fmtarg, firstvararg)))
#else
#define	__printflike(a,b)
#endif
#endif

#ifndef __scanflike
#if (__GNUC_PREREQ__(2, 7) || defined(__INTEL_COMPILER))
#define	__scanflike(fmtarg, firstvararg) \
     __attribute__((__format__ (__scanf__, fmtarg, firstvararg)))
#else
#define	__scanflike(a,b)
#endif
#endif

#ifndef CTASSERT			/* Allow lint to override */
#define	CTASSERT(x)             _CTASSERT(x, __LINE__)
#define	_CTASSERT(x, y)         __CTASSERT(x, y)
#define	__CTASSERT(x, y)        typedef char __assert ## y[(x) ? 1 : -1]
#endif

#ifdef _KERNEL
#include <sys/freebsd_linker_set.h>
#include <sys/freebsd_mutex.h>
#include <sys/freebsd_mbuf.h>
#include <sys/freebsd_ethernet.h>
#include <sys/freebsd_eventhandler.h>
#include <sys/freebsd_sysctl.h>
#include <sys/freebsd_devfs.h>
#include <sys/freebsd_bus.h>
#include <sys/freebsd_kernel.h>
#include <sys/freebsd_section.h>
#include <sys/freebsd_if_var.h>
#include <sys/freebsd_sound.h>
#include <sys/freebsd_callout.h>
#if (__NetBSD_Version__ >= 400000000)
#ifndef selrecord
#define	selrecord(a,b) do { \
      struct lwp *temp_lwp = curlwp; \
      __KASSERT(temp_lwp->l_proc == (a), ("Wrong lwp!")); \
      selrecord(temp_lwp,b); \
    } while (0)
#endif
#endif
#if (__NetBSD_Version__ >= 500000000)
#ifndef selwakeup
#define	selwakeup(a) do {\
      selnotify(a,0,0);	\
    } while (0)
#endif
#endif
#ifndef selwakeuppri
#define	selwakeuppri(sel, pri) selwakeup(sel)
#endif
#ifndef PROC_LOCK
#define	PROC_LOCK(x)
#endif
#ifndef PROC_UNLOCK
#define	PROC_UNLOCK(x)
#endif
#endif

#include <sys/freebsd_conf.h>

#undef O_NONBLOCK
#undef O_FSYNC

#define	O_NONBLOCK IO_NDELAY
#define	O_FSYNC IO_SYNC

#ifndef lockdestroy
#define	lockdestroy(args...)
#endif

#ifndef POLLSTANDARD
#define	POLLSTANDARD    (POLLIN|POLLPRI|POLLOUT|POLLRDNORM|POLLRDBAND|\
			   POLLWRBAND|POLLERR|POLLHUP|POLLNVAL)
#endif

#ifndef LIST_FOREACH_SAFE
#define	LIST_FOREACH_SAFE(var, head, field, tvar)		\
        for ((var) = LIST_FIRST((head));			\
	     (var) && ((tvar) = LIST_NEXT((var), field), 1);	\
	     (var) = (tvar))
#endif

#ifndef __FBSDID
#define	__FBSDID(args...)
#endif

#if (__NetBSD_Version__ < 400000000)
static __inline int
suser_cred(struct ucred *cred, int flag)
{
	return suser(cred, NULL);
}

#ifndef suser
#define	suser(td) suser((td)->p_ucred, NULL)
#endif
#define	kauth_cred_getuid(cr) (cr)->cr_uid
#define	kauth_cred_getgid(cr) (cr)->cr_gid
#define	kauth_cred_getegid(cr) (cr)->p_rgid
#define	kauth_cred_geteuid(cr) (cr)->p_ruid
#else
static __inline int
suser_cred(struct kauth_cred *cred, int flag)
{
	return (kauth_authorize_generic(cred,
	    KAUTH_GENERIC_ISSUSER, NULL));
}
static __inline int
suser(struct proc *td)
{
	struct lwp *temp_lwp = curlwp;

	__KASSERT(temp_lwp->l_proc == (td), ("Wrong lwp!"));
	return (kauth_authorize_generic(temp_lwp->l_cred,
	    KAUTH_GENERIC_ISSUSER, NULL));
}
static __inline int
groupmember(gid_t _gid, struct kauth_cred *cred)
{
	int result;

	if (kauth_cred_ismember_gid(cred, _gid, &result)) {
		result = 0;
	}
	return (result);
}

#endif

#if (__NetBSD_Version__ < 400000000)
#ifndef time_second
static __inline time_t 
time_second()
{
	return time.tv_sec;
}

#define	time_second time_second()
#endif
#endif

#ifndef M_CACHE
#define	M_CACHE M_TEMP
#endif

#undef if_attach
#define	if_attach(ifp) \
  { if_attach(ifp); if_alloc_sadl(ifp); }

#ifndef IHFC_USB_ENABLED

#define	USB_PAGE_SIZE PAGE_SIZE

/* helper for computing offsets */
#define	POINTER_TO_UNSIGNED(ptr) \
  (((const uint8_t *)(ptr)) - ((const uint8_t *)0))

#define	USBD_ADD_BYTES(ptr,size) \
  ((void *)(POINTER_TO_UNSIGNED(ptr) + (size)))

struct usbd_dma_parent_tag;

/*
 * The following typedef defines the USB DMA load done callback.
 */

typedef void (usbd_dma_callback_t)(struct usbd_dma_parent_tag *udpt);

/*
 * The following structure describes the parent USB DMA tag.
 */
struct usbd_dma_parent_tag {
	bus_dma_tag_t tag;		/* always set */

	struct mtx *mtx;		/* private mutex, always set */
	struct usbd_memory_info *info;	/* used by the callback function */
	usbd_dma_callback_t *func;	/* load complete callback function */
	struct usbd_dma_tag *utag_first;/* pointer to first USB DMA tag */

	uint8_t	dma_error;		/* set if DMA load operation failed */
	uint8_t	dma_bits;		/* number of DMA address lines */
	uint8_t	utag_max;		/* number of USB DMA tags */
};

/*
 * The following structure describes an USB DMA tag.
 */
struct usbd_dma_tag {
#ifdef __NetBSD__
	bus_dma_segment_t *p_seg;
#endif
	struct usbd_dma_parent_tag *tag_parent;
	bus_dma_tag_t tag;

	uint32_t align;
	uint32_t size;
#ifdef __NetBSD__
	uint32_t n_seg;
#endif
};

struct usbd_page {
	bus_size_t physaddr;
	void   *buffer;			/* non Kernel Virtual Address */
};

struct usbd_page_search {
	void   *buffer;
	bus_size_t physaddr;
	uint32_t length;
};

struct usbd_page_cache {

#ifdef __FreeBSD__
	bus_dma_tag_t tag;
	bus_dmamap_t map;
#endif
#ifdef __NetBSD__
	bus_dma_tag_t tag;
	bus_dmamap_t map;
	bus_dma_segment_t *p_seg;
#endif
	struct usbd_page *page_start;
	struct usbd_dma_parent_tag *tag_parent;	/* always set */

	void   *buffer;			/* virtual buffer pointer */
#ifdef __NetBSD__
	int	n_seg;
#endif
	uint32_t page_offset_buf;
	uint32_t page_offset_end;
	uint8_t	isread:1;
	uint8_t	ismultiseg:1;
};

void	usbd_dma_tag_setup(struct usbd_dma_parent_tag *udpt, struct usbd_dma_tag *udt, bus_dma_tag_t dmat, struct mtx *mtx, usbd_dma_callback_t *func, struct usbd_memory_info *info, uint8_t ndmabits, uint8_t nudt);
struct usbd_dma_tag *usbd_dma_tag_find(struct usbd_dma_parent_tag *updt, uint32_t size, uint32_t align);
void	usbd_dma_tag_unsetup(struct usbd_dma_parent_tag *udpt);
uint8_t	usbd_pc_alloc_mem(struct usbd_page_cache *pc, struct usbd_page *pg, uint32_t size, uint32_t align);
void	usbd_pc_free_mem(struct usbd_page_cache *pc);
void	usbd_pc_cpu_invalidate(struct usbd_page_cache *pc);
void	usbd_pc_cpu_flush(struct usbd_page_cache *pc);
void	usbd_get_page(struct usbd_page_cache *pc, uint32_t offset, struct usbd_page_search *res);

#endif

#else
#error "Operating system not supported"
#endif

#endif					/* __FREEBSD_SYS_COMPAT_H__ */
