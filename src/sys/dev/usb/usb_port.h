/* $NetBSD: usb_port.h,v 1.54 2002/03/28 21:49:19 ichiro Exp $ */
/* $FreeBSD: src/sys/dev/usb/usb_port.h,v 1.99 2007/10/20 23:23:18 julian Exp $ */

/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _USB_PORT_H
#define	_USB_PORT_H

#ifdef _KERNEL

#ifdef __FreeBSD__

#include <sys/conf.h>
#include <machine/bus.h>		/* bus_space_xxx() */
#include <machine/resource.h>		/* SYS_XXX */
#include <sys/bus.h>			/* device_xxx() */
#ifdef INCLUDE_PCIXXX_H
/* NOTE: one does not want to include these
 * files when building the USB device driver
 * modules!
 */
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#endif
#include <sys/lockmgr.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/rman.h>
#include <sys/selinfo.h>
#include <sys/sysctl.h>			/* SYSCTL_XXX() */
#include <sys/fcntl.h>
#include <sys/taskqueue.h>
#include <sys/callout.h>		/* callout_xxx() */

#include <net/ethernet.h>		/* ETHER_XXX */

#ifndef __KASSERT
typedef struct cdevsw cdevsw_t;

#define	__lockmgr lockmgr
#define	__KASSERT KASSERT
#define	uio_procp uio_td
#endif
#ifndef usb_callout_init_mtx
#define	usb_callout_init_mtx(c,m,f) callout_init_mtx(&(c)->co,m,f)
#define	usb_callout_reset(c,t,f,d) callout_reset(&(c)->co,t,f,d)
#define	usb_callout_stop(c) callout_stop(&(c)->co)
#define	usb_callout_drain(c) callout_drain(&(c)->co)
#define	usb_callout_pending(c) callout_pending(&(c)->co)
struct usb_callout {
	struct callout co;
};

#endif

#include "usb_if.h"

#else
#include <sys/freebsd_compat.h>
#endif
#endif

/*
 * Macros to cope with the differences between operating systems.
 */

#ifdef __NetBSD__
/*
 * NetBSD
 */

#include "opt_usbverbose.h"

#define	SCSI_MODE_SENSE		MODE_SENSE

#define	usb_thread_create	kthread_create1
#define	usb_thread_exit(err)	kthread_exit(err)

#if (__NetBSD_Version__ >= 300000000)
typedef void *usb_malloc_type;

#else
typedef int usb_malloc_type;

#endif

#define	Ether_ifattach ether_ifattach

#elif defined(__OpenBSD__)
/*
 * OpenBSD
 */

#define	UCOMBUSCF_PORTNO		-1
#define	UCOMBUSCF_PORTNO_DEFAULT	-1

#define	SCSI_MODE_SENSE		MODE_SENSE
#define	XS_STS_DONE		ITSDONE
#define	XS_CTL_POLL		SCSI_POLL
#define	XS_CTL_DATA_IN		SCSI_DATA_IN
#define	XS_CTL_DATA_OUT		SCSI_DATA_OUT
#define	scsipi_adapter		scsi_adapter
#define	scsipi_cmd		scsi_cmd
#define	scsipi_device		scsi_device
#define	scsipi_done		scsi_done
#define	scsipi_link		scsi_link
#define	scsipi_minphys		scsi_minphys
#define	scsipi_sense		scsi_sense
#define	scsipi_xfer		scsi_xfer
#define	xs_control		flags
#define	xs_status		status

#define	memcpy(d, s, l)		bcopy((s),(d),(l))
#define	memset(d, v, l)		bzero((d),(l))
#define	bswap32(x)		swap32(x)
#define	bswap16(x)		swap16(x)

/*
 * The UHCI/OHCI controllers are little endian, so on big endian machines
 * the data strored in memory needs to be swapped.
 */

#if defined(letoh32)
#define	le32toh(x) letoh32(x)
#define	le16toh(x) letoh16(x)
#endif

#if (BYTE_ORDER == BIG_ENDIAN)
#define	htole32(x) (bswap32(x))
#define	le32toh(x) (bswap32(x))
#else
#define	htole32(x) (x)
#define	le32toh(x) (x)
#endif

#define	usb_thread_create	kthread_create
#define	usb_thread_exit(err)	kthread_exit(err)

typedef int usb_malloc_type;

#define	Ether_ifattach(ifp, eaddr) ether_ifattach(ifp)
#define	if_deactivate(x)

#define	powerhook_establish(fn, sc) (fn)
#define	powerhook_disestablish(hdl)
#define	PWR_RESUME 0

#define	swap_bytes_change_sign16_le swap_bytes_change_sign16
#define	change_sign16_swap_bytes_le change_sign16_swap_bytes
#define	change_sign16_le change_sign16

extern int cold;

#elif defined(__FreeBSD__)
/*
 * FreeBSD
 */

#include "opt_usb.h"

#if (__FreeBSD_version >= 800000)
#define	usb_thread_create(f, s, p, ...) \
		kproc_create((f), (s), (p), RFHIGHPID, 0, __VA_ARGS__)
#define	usb_thread_exit(err)	kproc_exit(err)
#else
#define	usb_thread_create(f, s, p, ...) \
		kthread_create((f), (s), (p), RFHIGHPID, 0, __VA_ARGS__)
#define	usb_thread_exit(err)	kthread_exit(err)
#define	thread_lock(td) mtx_lock_spin(&sched_lock)
#define	thread_unlock(td) mtx_unlock_spin(&sched_lock)
#endif

#define	clalloc(p, s, x) (clist_alloc_cblocks((p), (s), (s)), 0)
#define	clfree(p) clist_free_cblocks((p))

#define	PWR_RESUME 0
#define	PWR_SUSPEND 1

typedef struct malloc_type *usb_malloc_type;

#endif					/* __FreeBSD__ */

#define	USBVERBOSE

#ifndef Static
#define	Static               static
#endif

#ifndef logprintf
#define	logprintf printf
#endif

#ifdef MALLOC_DECLARE
MALLOC_DECLARE(M_USB);
MALLOC_DECLARE(M_USBDEV);
MALLOC_DECLARE(M_USBHC);
#endif

#ifdef SYSCTL_DECL
SYSCTL_DECL(_hw_usb);
#endif

/* force debugging until further */
#ifndef USB_DEBUG
#define	USB_DEBUG
#endif

#ifdef USB_DEBUG
#define	PRINTF(x)      { if (usbdebug) { printf("%s: ", __FUNCTION__); printf x ; } }
#define	PRINTFN(n,x)   { if (usbdebug > (n)) { printf("%s: ", __FUNCTION__); printf x ; } }
extern int usbdebug;

#else
#define	PRINTF(x)
#define	PRINTFN(n,x)
#endif

#define	_MAKE_ENUM(enum,value,arg...)		\
        enum value,				\
					/**/

#define	MAKE_ENUM(macro,end...)			\
enum { macro(_MAKE_ENUM) end }			\
					/**/

#define	__MAKE_TABLE(a...) a		/* double pass to expand all macros */
#define	_MAKE_TABLE(a...) (a),		/* add comma */
#define	MAKE_TABLE(m,field,p,a...) m##_##field p = { __MAKE_TABLE(m(m##_##field _MAKE_TABLE)) a }

#ifndef LOG2
#define	LOG2(x) ( \
((x) <= (1<<0x0)) ? 0x0 : \
((x) <= (1<<0x1)) ? 0x1 : \
((x) <= (1<<0x2)) ? 0x2 : \
((x) <= (1<<0x3)) ? 0x3 : \
((x) <= (1<<0x4)) ? 0x4 : \
((x) <= (1<<0x5)) ? 0x5 : \
((x) <= (1<<0x6)) ? 0x6 : \
((x) <= (1<<0x7)) ? 0x7 : \
((x) <= (1<<0x8)) ? 0x8 : \
((x) <= (1<<0x9)) ? 0x9 : \
((x) <= (1<<0xA)) ? 0xA : \
((x) <= (1<<0xB)) ? 0xB : \
((x) <= (1<<0xC)) ? 0xC : \
((x) <= (1<<0xD)) ? 0xD : \
((x) <= (1<<0xE)) ? 0xE : \
((x) <= (1<<0xF)) ? 0xF : \
0x10)
#endif					/* LOG2 */

/* preliminary fix for a bug in msleep on FreeBSD,
 * which cannot sleep with Giant:
 */
#ifdef mtx_sleep
#undef mtx_sleep
#define	mtx_sleep(i,m,p,w,t) \
  _sleep(i,(((m) == &Giant) ? NULL : &(m)->lock_object),p,w,t)
#else
#define	mtx_sleep(i,m,p,w,t) \
  msleep(i,(((m) == &Giant) ? NULL : (m)),p,w,t)
#endif

#endif					/* _USB_PORT_H */
