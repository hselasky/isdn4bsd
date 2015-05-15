/*-
 * Copyright (c) 2015 Hans Petter Selasky. All rights reserved.
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

#ifndef _I4B_IOPROXY_H_
#define	_I4B_IOPROXY_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/kernel.h>
#include <sys/kern_mtx.h>
#include <sys/kern_thread.h>
#include <sys/queue.h>
#include <sys/eventhandler.h>
#include <sys/kern_cv.h>
#include <sys/kern_irq.h>
#include <sys/kern_sx.h>
#include <sys/kern_timer.h>
#include <sys/kern_conf.h>
#include <sys/kern_irq.h>
#include <sys/kern_cdev.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/syspublic.h>
#include <sys/mbuf.h>

#define	__KASSERT(arg, fmt) assert(arg)
#define	log(...) do { } while (0)
#define	priv_check(...) (1)
#define	bcopy(s,d,len) memcpy(d,s,len)
#define	SI_SUB_KLD	(SI_SUB_APPLICATIONS - 2)
#define	SI_SUB_PSEUDO	(SI_SUB_APPLICATIONS - 1)
#define	CAPI_AI_SOFTC_POOL_MAX	1	/* connection */
#define	I4B_CDESC_POOL_MAX 8		/* calldescriptors */
#define	I4B_GPHONE_UNIT_MAX 1		/* unit */
#define	I4B_MAX_CONTROLLERS 1
#define	I4B_MAX_CHANNELS 2
#define	IFQ_MAXLEN 8
#define	HAVE_NO_I4B_DRIVER
#define	HAVE_NO_I4B_TRACE
#define	HAVE_NO_ECHO_CANCEL

#endif					/* _I4B_IOPROXY_H_ */
