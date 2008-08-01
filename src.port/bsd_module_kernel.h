/*-
 * Copyright (c) 1995 Terrence R. Lambert
 * All rights reserved.
 *
 * Copyright (c) 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
 * (c) UNIX System Laboratories, Inc.
 * All or some portions of this file are derived from material licensed
 * to the University of California by American Telephone and Telegraph
 * Co. or Unix System Laboratories, Inc. and are reproduced herein with
 * the permission of UNIX System Laboratories, Inc.
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
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
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
 * This is a lite version of "FreeBSD/src/sys/sys/kernel.h"
 */

#ifndef _BSD_MODULE_KERNEL_H_
#define	_BSD_MODULE_KERNEL_H_

/* startup groups */

enum sysinit_sub_id {
	SI_SUB_DUMMY		= 0x0000000,	/* not executed; for linker*/
	SI_SUB_DONE		= 0x0000001,	/* processed*/
	SI_SUB_TUNABLES		= 0x0700000,	/* establish tunable values */
	SI_SUB_CONSOLE		= 0x0800000,	/* console*/
	SI_SUB_COPYRIGHT	= 0x0800001,	/* first use of console*/
	SI_SUB_SETTINGS		= 0x0880000,	/* check and recheck settings */
	SI_SUB_MTX_POOL_STATIC	= 0x0900000,	/* static mutex pool */
	SI_SUB_LOCKMGR		= 0x0980000,	/* lockmgr locks */
	SI_SUB_VM		= 0x1000000,	/* virtual memory system init*/
	SI_SUB_KMEM		= 0x1800000,	/* kernel memory*/
	SI_SUB_KVM_RSRC		= 0x1A00000,	/* kvm operational limits*/
	SI_SUB_WITNESS		= 0x1A80000,	/* witness initialization */
	SI_SUB_MTX_POOL_DYNAMIC	= 0x1AC0000,	/* dynamic mutex pool */
	SI_SUB_LOCK		= 0x1B00000,	/* various locks */
	SI_SUB_EVENTHANDLER	= 0x1C00000,	/* eventhandler init */
	SI_SUB_KLD		= 0x2000000,	/* KLD and module setup */
	SI_SUB_CPU		= 0x2100000,	/* CPU resource(s)*/
	SI_SUB_MAC		= 0x2180000,	/* TrustedBSD MAC subsystem */
	SI_SUB_MAC_POLICY	= 0x21C0000,	/* TrustedBSD MAC policies */
	SI_SUB_MAC_LATE		= 0x21D0000,	/* TrustedBSD MAC subsystem */
	SI_SUB_INTRINSIC	= 0x2200000,	/* proc 0*/
	SI_SUB_VM_CONF		= 0x2300000,	/* config VM, set limits*/
	SI_SUB_RUN_QUEUE	= 0x2400000,	/* set up run queue*/
	SI_SUB_KTRACE		= 0x2480000,	/* ktrace */
	SI_SUB_AUDIT		= 0x24C0000,	/* audit */
	SI_SUB_CREATE_INIT	= 0x2500000,	/* create init process*/
	SI_SUB_SCHED_IDLE	= 0x2600000,	/* required idle procs */
	SI_SUB_MBUF		= 0x2700000,	/* mbuf subsystem */
	SI_SUB_INTR		= 0x2800000,	/* interrupt threads */
	SI_SUB_SOFTINTR		= 0x2800001,	/* start soft interrupt thread */
	SI_SUB_DEVFS		= 0x2F00000,	/* devfs ready for devices */
	SI_SUB_INIT_IF		= 0x3000000,	/* prep for net interfaces */
	SI_SUB_NETGRAPH		= 0x3010000,	/* Let Netgraph initialize */
	SI_SUB_DRIVERS		= 0x3100000,	/* Let Drivers initialize */
	SI_SUB_CONFIGURE	= 0x3800000,	/* Configure devices */
	SI_SUB_VFS		= 0x4000000,	/* virtual filesystem*/
	SI_SUB_CLOCKS		= 0x4800000,	/* real time and stat clocks*/
	SI_SUB_CLIST		= 0x5800000,	/* clists*/
	SI_SUB_SYSV_SHM		= 0x6400000,	/* System V shared memory*/
	SI_SUB_SYSV_SEM		= 0x6800000,	/* System V semaphores*/
	SI_SUB_SYSV_MSG		= 0x6C00000,	/* System V message queues*/
	SI_SUB_P1003_1B		= 0x6E00000,	/* P1003.1B realtime */
	SI_SUB_PSEUDO		= 0x7000000,	/* pseudo devices*/
	SI_SUB_EXEC		= 0x7400000,	/* execve() handlers */
	SI_SUB_PROTO_BEGIN	= 0x8000000,	/* XXX: set splimp (kludge)*/
	SI_SUB_PROTO_IF		= 0x8400000,	/* interfaces*/
	SI_SUB_PROTO_DOMAIN	= 0x8800000,	/* domains (address families?)*/
	SI_SUB_PROTO_IFATTACHDOMAIN	= 0x8800001,	/* domain dependent data init*/
	SI_SUB_PROTO_END	= 0x8ffffff,	/* XXX: set splx (kludge)*/
	SI_SUB_KPROF		= 0x9000000,	/* kernel profiling*/
	SI_SUB_KICK_SCHEDULER	= 0xa000000,	/* start the timeout events*/
	SI_SUB_INT_CONFIG_HOOKS	= 0xa800000,	/* Interrupts enabled config */
	SI_SUB_ROOT_CONF	= 0xb000000,	/* Find root devices */
	SI_SUB_DUMP_CONF	= 0xb200000,	/* Find dump devices */
	SI_SUB_RAID		= 0xb380000,	/* Configure GEOM classes */
	SI_SUB_MOUNT_ROOT	= 0xb400000,	/* root mount*/
	SI_SUB_SWAP		= 0xc000000,	/* swap */
	SI_SUB_INTRINSIC_POST	= 0xd000000,	/* proc 0 cleanup*/
	SI_SUB_KTHREAD_INIT	= 0xe000000,	/* init process*/
	SI_SUB_KTHREAD_PAGE	= 0xe400000,	/* pageout daemon*/
	SI_SUB_KTHREAD_VM	= 0xe800000,	/* vm daemon*/
	SI_SUB_KTHREAD_BUF	= 0xea00000,	/* buffer daemon*/
	SI_SUB_KTHREAD_UPDATE	= 0xec00000,	/* update daemon*/
	SI_SUB_KTHREAD_IDLE	= 0xee00000,	/* idle procs*/
	SI_SUB_SMP		= 0xf000000,	/* start the APs*/
	SI_SUB_RUN_SCHEDULER	= 0xfffffff	/* scheduler*/
};

/* startup order within startup group */

enum sysinit_elem_order {
	SI_ORDER_FIRST		= 0x0000000,	/* first*/
	SI_ORDER_SECOND		= 0x0000001,	/* second*/
	SI_ORDER_THIRD		= 0x0000002,	/* third*/
	SI_ORDER_MIDDLE		= 0x1000000,	/* somewhere in the middle */
	SI_ORDER_ANY		= 0xfffffff	/* last*/
};

typedef void (*sysinit_nfunc_t)(void *);

struct sysinit {
	enum sysinit_sub_id	subsystem;	/* subsystem identifier*/
	enum sysinit_elem_order	order;		/* init order within subsystem*/
	sysinit_nfunc_t func;			/* function		*/
	void *    udata;			/* function argument */
	const char *file;
	int line;
};

/*
 * Called on module load/unload
 */
#define SYSINIT_PASS_DATA(...) __VA_ARGS__

#define	SYSINIT(uniquifier, subsystem, order, func, ident)	\
  const struct sysinit bsd_##uniquifier##_sys_init =		\
  { subsystem, order, func, __DECONST(void *,ident), __FILE__, __LINE__ }

#define	SYSUNINIT(uniquifier, subsystem, order, func, ident)	\
  const struct sysinit bsd_##uniquifier##_sys_uninit =		\
  { subsystem, order, func, __DECONST(void*,ident), __FILE__, __LINE__ }

#endif /* _BSD_MODULE_KERNEL_H_ */
