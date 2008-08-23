/*-
 * Copyright (c) 1999 Michael Smith <msmith@freebsd.org>
 * Copyright (c) 2005 Hans Petter Selasky
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
 */

struct eventhandler_entry {
	void   *func;
	void   *arg;
	uint8_t	used;
	struct eventhandler_entry *next;
};

struct eventhandler_list {
	struct eventhandler_entry *el_root;
	struct mtx el_mtx;
};

typedef struct eventhandler_entry *eventhandler_tag;

#define	EVENTHANDLER_INVOKE(name, ...)		\
do {						\
    struct eventhandler_entry *ptr;		\
    struct eventhandler_list *list =		\
	eventhandler_get_list(#name);		\
						\
    if(list == NULL)				\
    {						\
        break;					\
    }						\
						\
    mtx_lock(&(list)->el_mtx);			\
    ptr = (list)->el_root;			\
    while(ptr)					\
    {						\
       if((ptr)->used)				\
       {					\
	   mtx_unlock(&(list)->el_mtx);		\
						\
	   ((name##_fn_type)(ptr->func))	\
		(ptr->arg,## __VA_ARGS__);	\
						\
	   mtx_lock(&(list)->el_mtx);		\
       }					\
       ptr = ptr->next;				\
    }						\
    mtx_unlock(&(list)->el_mtx);		\
} while(0)

extern struct eventhandler_list *
	eventhandler_get_list(const char *name);

extern	eventhandler_tag
eventhandler_register(struct eventhandler_list *list,
    const char *name, void *func, void *arg,
    int priority);
extern void
	eventhandler_deregister(struct eventhandler_list *list, eventhandler_tag tag);

#define	EVENTHANDLER_DECLARE(name, type)				\
typedef type name##_fn_type;						\
static __inline eventhandler_tag					\
eventhandler_register_##name(struct eventhandler_list *list,		\
			     const char *name, type func,		\
			     void *arg, int priority)			\
{									\
  return eventhandler_register(list, name, (void *)func, arg, priority);	\
}

#define	EVENTHANDLER_REGISTER(name, func, arg, pri)		\
  eventhandler_register_##name(NULL, #name, func, arg, pri)

#define	EVENTHANDLER_DEREGISTER(name, tag)			\
  eventhandler_deregister(eventhandler_get_list(#name), tag)

/* Generic priority levels */
#define	EVENTHANDLER_PRI_FIRST	0
#define	EVENTHANDLER_PRI_ANY	10000
#define	EVENTHANDLER_PRI_LAST	20000

/* Shutdown events */
typedef void (*shutdown_fn) (void *, int);

#define	SHUTDOWN_PRI_FIRST	EVENTHANDLER_PRI_FIRST
#define	SHUTDOWN_PRI_DEFAULT	EVENTHANDLER_PRI_ANY
#define	SHUTDOWN_PRI_LAST	EVENTHANDLER_PRI_LAST

EVENTHANDLER_DECLARE(shutdown_pre_sync, shutdown_fn);	/* before fs sync */
EVENTHANDLER_DECLARE(shutdown_post_sync, shutdown_fn);	/* after fs sync */
EVENTHANDLER_DECLARE(shutdown_final, shutdown_fn);

/* Low memory event */
typedef void (*vm_lowmem_handler_t)(void *, int);

#define	LOWMEM_PRI_DEFAULT	EVENTHANDLER_PRI_FIRST

EVENTHANDLER_DECLARE(vm_lowmem, vm_lowmem_handler_t);

/*
 * Process events
 * process_fork and exit handlers are called without Giant.
 * exec handlers are called with Giant, but that is by accident.
 */
struct proc;

typedef void (*exitlist_fn) (void *, struct proc *);
typedef void (*forklist_fn) (void *, struct proc *, struct proc *, int);
typedef void (*execlist_fn) (void *, struct proc *);

EVENTHANDLER_DECLARE(process_exit, exitlist_fn);
EVENTHANDLER_DECLARE(process_fork, forklist_fn);
EVENTHANDLER_DECLARE(process_exec, execlist_fn);
