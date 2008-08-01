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

#include "ose.h"

#include <bsd_module_all.h>

static void
kproc_start(void)
{
  void (*func)(void *arg);
  void *arg;
  func = get_env(current_process(), "bsd_func_ptr");
  arg = get_env(current_process(), "bsd_func_arg");
  (func)(arg);
  return;
}

int
kproc_create(void (*func)(void *), void *arg, struct proc proc**,          
	     int flags, int pages, const char *fmt, ...)
{
	PROCESS p;

	p = create_process(OS_PRI_PROC,
	     "USBPROC", &kproc_start, 4096, 15, (OSTIME)0,
	     (PROCESS)0, (struct OS_redir_entry *) NULL, 
	     (OSVECTOR) 0, (OSUSER)0);

	*proc = (void *)p;

	set_env(p, "bsd_func_ptr", func);
	set_env(p, "bsd_func_arg", arg);

	start(p);

	return (0);
}

void
kproc_exit(int)
{
	kill_proc(current_process());
	return;
}

int
kproc_suspend(struct proc *proc, int)
{
	/* not needed */
	return (0);
}

struct thread *
curthread_sub(void)
{
	return ((void *)current_process());
}

void
sched_prio(struct thread *td, uint8_t prio)
{
  /* not implemented */
  return;
}
