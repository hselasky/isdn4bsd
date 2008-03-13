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
#define __FREEBSD_SYS_COMPAT_H__

#if defined(__FreeBSD__)

# include <sys/conf.h>

# ifdef _KERNEL
#  include <machine/resource.h>
#  include <dev/sound/pcm/sound.h>
#  include <dev/pci/pcireg.h>
#  include <dev/pci/pcivar.h>
#  include <fs/devfs/devfs.h>
#  include <isa/isa_common.h>
#  include <isa/isavar.h>
#  include <net/if_var.h>
#  include <netgraph/netgraph.h>
#  include <netgraph/ng_message.h>
#  include <netgraph/ng_parse.h>
#  include <sys/bus.h>
#  include <sys/lockmgr.h>
#  include <sys/module.h>
#  include <sys/mutex.h>
#  include <sys/rman.h>
#  include <sys/rtprio.h>
#  include <sys/selinfo.h>
#  include <sys/sysctl.h>
# else
#  include <ncurses.h>
#  include <osreldate.h>
# endif

# ifdef _KERNEL
typedef struct cdevsw cdevsw_t;
# define __lockmgr lockmgr
# define __KASSERT KASSERT
# endif

#elif (defined(__NetBSD__) || defined(__OpenBSD__))

# include <sys/cdefs.h>
# include <sys/types.h>

# ifdef _KERNEL
#  include <machine/bus.h>
#  include <sys/device.h>
#  include <sys/lock.h>
#  include <sys/queue.h>
#  include <sys/sysctl.h>
#  include <sys/namei.h>
#  include <sys/unistd.h>
#  include <sys/select.h>
#  include <sys/kernel.h>
#  include <sys/reboot.h>
#  include <sys/mbuf.h>
#  include <net/if.h>
#  include <net/if_spppvar.h>
#  if (__NetBSD_Version__ >= 400000000)
#   include <net80211/ieee80211_netbsd.h>
#   include <sys/kauth.h>
#  else
#   include <net80211/ieee80211_compat.h>
#  endif
#  include <sys/callout.h>
# else
#  include <sys/queue.h>
# endif

# ifdef _KERNEL
typedef struct __cdevsw cdevsw_t;
#  ifndef __KASSERT
#   define __KASSERT(exp,msg)	\
do {				\
  if(__predict_false(!(exp)))	\
  {				\
      panic msg;		\
  }				\
} while (0)
#  endif

#  if defined(LOCKDEBUG)
#   define __lockmgr(l, f, i, p) _lockmgr((l), (f), (i), __FILE__, __LINE__)
#  else
#   define __lockmgr(l, f, i, p) lockmgr(l, f, i)
#  endif

#  define thread proc
#  define curthread curproc

#  define WITNESS_WARN(...) 

#  undef si_uid
#  undef si_gid

#  ifndef SPECNAMELEN
#   define SPECNAMELEN     63   /* max length of devicename */
#  endif

typedef u_int32_t  vm_offset_t;
typedef u_int32_t  vm_size_t;

typedef u_int64_t  vm_ooffset_t;
typedef u_int64_t  vm_paddr_t;
typedef u_int64_t  vm_pindex_t;

#  if (__NetBSD_Version__ >= 300000000)
#   ifdef PCI_MACHDEP_ENUMERATE_BUS
#    define pci_enumerate_bus PCI_MACHDEP_ENUMERATE_BUS
#   else
struct pci_softc;
struct pci_attach_args;
extern int pci_enumerate_bus
  (struct pci_softc *, const int *,
   int (*)(struct pci_attach_args *), 
   struct pci_attach_args *);
#   endif
#   define copy_statfs_info copy_statvfs_info
#   define set_statfs_info set_statvfs_info
#   define vfs_statfs vfs_statvfs
#   define statfs statvfs
#  endif

# endif

# ifndef __used
#  if (__GNUC_PREREQ__(2, 7) || defined(__INTEL_COMPILER))
#   define __used          __attribute__((__used__))
#  else
#   define __used
#  endif
# endif

# ifndef __printflike
#  if (__GNUC_PREREQ__(2, 7) || defined(__INTEL_COMPILER))
#   define __printflike(fmtarg, firstvararg) \
     __attribute__((__format__ (__printf__, fmtarg, firstvararg)))
#  else
#   define __printflike(a,b)
#  endif
# endif

# ifndef __scanflike
#  if (__GNUC_PREREQ__(2, 7) || defined(__INTEL_COMPILER))
#   define __scanflike(fmtarg, firstvararg) \
     __attribute__((__format__ (__scanf__, fmtarg, firstvararg)))
#  else
#   define __scanflike(a,b)
#  endif
# endif

# ifndef CTASSERT                /* Allow lint to override */
#  define CTASSERT(x)             _CTASSERT(x, __LINE__)
#  define _CTASSERT(x, y)         __CTASSERT(x, y)
#  define __CTASSERT(x, y)        typedef char __assert ## y[(x) ? 1 : -1]
# endif

# ifdef _KERNEL
#  include <sys/freebsd_linker_set.h>
#  include <sys/freebsd_mutex.h>
#  include <sys/freebsd_mbuf.h>
#  include <sys/freebsd_ethernet.h>
#  include <sys/freebsd_eventhandler.h>
#  include <sys/freebsd_sysctl.h>
#  include <sys/freebsd_devfs.h>
#  include <sys/freebsd_bus.h>
#  include <sys/freebsd_kernel.h>
#  include <sys/freebsd_section.h>
#  include <sys/freebsd_if_var.h>
#  include <sys/freebsd_sound.h>
#  include <sys/freebsd_callout.h>
# if (__NetBSD_Version >= 400000000)
#  ifndef selrecord
#   define selrecord(a,b) do { \
      struct lwp *temp_lwp = curlwp; \
      __KASSERT(temp_lwp->l_proc == (a), "Wrong lwp!"); \
      selrecord(temp_lwp,b); \
    } while (0)
#  endif
# endif
#  ifndef selwakeuppri
#   define selwakeuppri(sel, pri) selwakeup(sel)
#  endif
#  ifndef PROC_LOCK
#   define PROC_LOCK(x)
#  endif
#  ifndef PROC_UNLOCK
#   define PROC_UNLOCK(x)
#  endif
# endif

# include <sys/freebsd_conf.h>

# undef O_NONBLOCK
# undef O_FSYNC

# define O_NONBLOCK IO_NDELAY
# define O_FSYNC IO_SYNC

# ifdef _KERNEL
#  define GENERIC_DIRSIZ(dp) \
    ((sizeof (struct dirent) - (MAXNAMLEN+1)) + (((dp)->d_namlen+1 + 3) &~ 3))
# endif

# ifndef lockdestroy
#  define lockdestroy(args...) 
# endif

# ifndef POLLSTANDARD
#  define POLLSTANDARD    (POLLIN|POLLPRI|POLLOUT|POLLRDNORM|POLLRDBAND|\
			   POLLWRBAND|POLLERR|POLLHUP|POLLNVAL)
# endif

# ifndef LIST_FOREACH_SAFE
#  define LIST_FOREACH_SAFE(var, head, field, tvar)		\
        for ((var) = LIST_FIRST((head));			\
	     (var) && ((tvar) = LIST_NEXT((var), field), 1);	\
	     (var) = (tvar))
# endif

# ifndef __FBSDID
#  define __FBSDID(args...)
# endif

static __inline int
suser_cred(struct ucred *cred, int flag)
{
    return suser(cred, NULL);
}

# ifndef suser
#  define suser(td) suser((td)->p_ucred, NULL)
# endif

#if (__NetBSD_Version__ < 400000000)
# ifndef time_second
static __inline time_t time_second() { return time.tv_sec; }
#  define time_second time_second()
# endif
#endif

# ifndef M_CACHE
#  define M_CACHE M_TEMP
# endif

# undef if_attach
# define if_attach(ifp) \
  { if_attach(ifp); if_alloc_sadl(ifp); }

#ifndef IHFC_USB_ENABLED

#define	USB_PAGE_SIZE PAGE_SIZE

/* helper for computing offsets */
#define	POINTER_TO_UNSIGNED(ptr) \
  (((const uint8_t *)(ptr)) - ((const uint8_t *)0))

#define	USBD_ADD_BYTES(ptr,size) \
  ((void *)(POINTER_TO_UNSIGNED(ptr) + (size)))

struct usbd_dma_tag {
#ifdef __NetBSD__
	bus_dma_segment_t *p_seg;
#endif
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
	void   *buffer;			/* virtual buffer pointer */
#ifdef __NetBSD__
	int	n_seg;
#endif
	uint32_t page_offset_buf;
	uint32_t page_offset_end;
	uint8_t	isread:1;
	uint8_t ismultiseg:1;
};

struct usbd_dma_tag * usbd_dma_tag_setup(bus_dma_tag_t tag_parent, struct usbd_dma_tag *udt, uint32_t size, uint32_t align, uint8_t nudt);
void usbd_dma_tag_unsetup(struct usbd_dma_tag *udt, uint8_t nudt);
uint8_t usbd_pc_alloc_mem(bus_dma_tag_t parent_tag, struct usbd_dma_tag *utag, struct usbd_page_cache *pc, struct usbd_page *pg, uint32_t size, uint32_t align, uint8_t utag_max);
void usbd_pc_free_mem(struct usbd_page_cache *pc);
void usbd_pc_cpu_invalidate(struct usbd_page_cache *pc);
void usbd_pc_cpu_flush(struct usbd_page_cache *pc);
void usbd_get_page(struct usbd_page_cache *pc, uint32_t offset, struct usbd_page_search *res);

#endif

#else
#error "Operating system not supported"
#endif

#endif /* __FREEBSD_SYS_COMPAT_H__ */
