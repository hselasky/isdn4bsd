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

#include <bsd_module_all.h>

extern int pthread_create(struct proc **proc, const void *attr, kproc_func_t *func, void *arg);
extern void pthread_exit(void *value_ptr);
extern struct thread *pthread_self(void);

#ifndef SIMULATOR
static void
kproc_start(void *arg)
{
#if 0
	kproc_func_t *func;
	void *arg;

	func = (void *)get_envp(current_process(), "bsd_func_ptr");
	arg = (void *)get_envp(current_process(), "bsd_func_arg");
	(func) (arg);
#endif
	printf("kproc_start: WARNING: Process %p did "
	    "not call kproc_exit()!\n", curthread);
	kproc_exit(0);
	return;
}

#endif

int
kproc_create(kproc_func_t *func, void *arg, struct proc **proc,
    int flags, int pages, const char *fmt,...)
{
#if 0
	PROCESS p;

	p = create_process(OS_PRI_PROC,
	    "USBPROC", &kproc_start, 4096, 15, (OSTIME) 0,
	    (PROCESS) 0, (struct OS_redir_entry *)NULL,
	    (OSVECTOR) 0, (OSUSER) 0);

	*proc = (void *)p;

	set_envp(p, "bsd_func_ptr", (OSADDRESS) func);
	set_envp(p, "bsd_func_arg", (OSADDRESS) arg);

	start(p);
#endif
#ifdef SIMULATOR
	if (pthread_create(proc, NULL, func, arg)) {
		printf("Failed creating process %s\n", fmt);
	}
#endif
	return (0);
}

void
kproc_exit(int error)
{
#if 0
	kill_proc(current_process());
#endif
#ifdef SIMULATOR
	pthread_exit(NULL);
#endif
	return;
}

int
kproc_suspend(struct proc *proc, int ticks)
{
	/* not needed */
	return (0);
}

struct thread *
curthread_sub(void)
{
#if 0
	return ((void *)current_process());
#endif
#ifdef SIMULATOR
	return (pthread_self());
#endif
	return (NULL);
}

void
sched_prio(struct thread *td, uint8_t prio)
{
	/* not implemented */
	return;
}
