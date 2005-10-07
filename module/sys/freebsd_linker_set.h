/*-
 * Copyright (c) 1999 John D. Polstra
 * Copyright (c) 1999,2001 Peter Wemm <peter@FreeBSD.org>
 * All rights reserved.
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
 *
 * This is a lite version of "FreeBSD/src/sys/sys/linker_set.h"
 */

#ifndef __FREEBSD_SYS_LINKER_SET_H__
#define __FREEBSD_SYS_LINKER_SET_H__

#define __MAKE_SET(set, sym)					\
	static void const * const __set_##set##_sym_##sym	\
	__section("set_" #set) __used = &sym

#define TEXT_SET( set, sym)	__MAKE_SET(set, sym)
#define DATA_SET( set, sym)	__MAKE_SET(set, sym)
#define BSS_SET(  set, sym)	__MAKE_SET(set, sym)
#define ABS_SET(  set, sym)	__MAKE_SET(set, sym)
#define SET_ENTRY(set, sym)	__MAKE_SET(set, sym)

#define SET_DECLARE(set, ptype)				\
	extern ptype *__CONCAT(__start_set_,set);	\
	extern ptype *__CONCAT(__stop_set_,set)

#define SET_BEGIN(set)				\
	(&__CONCAT(__start_set_,set))
#define SET_LIMIT(set)				\
	(&__CONCAT(__stop_set_,set))

#define SET_FOREACH(pvar, set)			\
	for((pvar) = SET_BEGIN(set);		\
	    (pvar) < SET_LIMIT(set);		\
	    (pvar)++)

#define SET_ITEM(set, i)			\
	((SET_BEGIN(set))[i])

/*
 * Provide a count of the items in a set.
 */
#define SET_COUNT(set)				\
	(SET_LIMIT(set) - SET_BEGIN(set))

#endif	/* __FREEBSD_SYS_LINKER_SET_H__ */
