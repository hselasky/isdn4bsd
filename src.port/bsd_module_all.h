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

#ifndef _BSD_MODULE_ALL_H_
#define	_BSD_MODULE_ALL_H_

#include <bsd_module_rename.h>

#define	__FreeBSD_version 800000
#undef __FreeBSD__
#define	__FreeBSD__
#define	_KERNEL
#define	__packed __attribute__((__packed__))
#define	__aligned(s) __attribute__((__aligned__(s)))
#define	__printflike(...)
#define	__DECONST(type, ptr) \
  ((type)(((const uint8_t *)(ptr)) - ((const uint8_t *)0)))

#define	__FBSDID(...)
#define	SYSCTL_DECL(...)
#define	SYSCTL_NODE(...)
#define	SYSCTL_INT(...)

#define	WARN_GIANTOK(...) do { } while (0)
#define	KASSERT(...) do { } while (0)
#define	WARN_SLEEPOK(...) do { } while (0)
#define	WITNESS_WARN(...) do { } while (0)
#define	MIN(a,b) (((a) < (b)) ? (a) : (b))
#define	MAX(a,b) (((a) > (b)) ? (a) : (b))

#define	PAGE_SIZE 4096			/* bytes */

#define	PI_NET 15
#define	PI_DISK 25
#define	RFHIGHPID 0

#define	va_size(type)				\
        (((sizeof(type) + sizeof(int) - 1) /	\
		sizeof(int)) * sizeof(int))

#define	va_start(ap, last) do {				\
	(ap) = ((va_list)&(last)) + va_size(last);	\
} while (0)

#define	va_end(ap) do { } while (0)

#define	NULL ((void *)0)
#define	cold 0

typedef unsigned int size_t;
typedef unsigned int off_t;
typedef unsigned long bus_addr_t;
typedef unsigned long bus_size_t;

typedef unsigned long long int uint64_t;
typedef signed long long int int64_t;

typedef unsigned int uint32_t;
typedef signed int int32_t;

typedef unsigned short uint16_t;
typedef signed short int16_t;

typedef unsigned char uint8_t;
typedef signed char int8_t;

typedef unsigned char *caddr_t;

typedef char *va_list;

typedef unsigned long u_long;

struct selinfo {
};
struct uio;
struct mbuf;

#define	fbsd_uiomove(...) EINVAL

#include <bsd_module_devmethod.h>
#include <bsd_module_queue.h>
#include <bsd_module_errno.h>
#include <bsd_module_kernel.h>
#include <bsd_module_mutex.h>
#include <bsd_module_callout.h>
#include <bsd_module_condvar.h>
#include <bsd_module_sx.h>
#include <bsd_module_busspace.h>
#include <bsd_module_bus.h>
#include <bsd_module_proc.h>
#include <bsd_module_malloc.h>
#include <bsd_module_libkern.h>
#include <bsd_module_busdma.h>
#include <bsd_module_usb.h>
#include <bsd_module_ioccom.h>
#include <bsd_module_file.h>

/* panic */

void	panic(const char *fmt,...)__printflike(1, 2);

/* delay */

void	DELAY(uint32_t us);

#endif					/* _BSD_MODULE_ALL_H_ */
