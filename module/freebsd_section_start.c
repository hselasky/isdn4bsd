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
 * This file defines the start of some sections
 * 
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/freebsd_compat.h>

const DECLARE_SYSCTL_DATA(bsd_sysctl_data_start[0]);
const DECLARE_MOD_DATA(bsd_module_data_start[0]);
const DECLARE_SYSINIT_DATA(bsd_sys_init_data_start[0]);
const DECLARE_SYSUNINIT_DATA(bsd_sys_uninit_data_start[0]);

/*
 * XXX manually define some functions
 * to zero:
 */

#define ZERO_DEF(what)				\
  void * what##_setup_ft(void) {		\
	return ((void *)1); /* FT_INVALID */	\
  }						\
  void what##_response_to_user(void) { }

ZERO_DEF(ibc);
ZERO_DEF(ing);
ZERO_DEF(diehl);
ZERO_DEF(tina_dd);
ZERO_DEF(amv_b1);
ZERO_DEF(i4bisppp);

