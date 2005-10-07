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
 * $FreeBSD: $
 *
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/kern/subr_eventhandler.h"
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include <sys/freebsd_compat.h>

static struct eventhandler_list evh_list[16];

static void
eventhandler_init(void *arg)
{
    u_int8_t x;
    for(x = 0; x < (sizeof(evh_list) / sizeof(evh_list[0])); x++)
    {
        mtx_init(&evh_list[x].el_mtx, "eventhandler", NULL, MTX_DEF);
    }
    return;
}
SYSINIT(eventhandler_init, SI_SUB_DRIVERS, SI_ORDER_FIRST, eventhandler_init, NULL);

struct eventhandler_list *
eventhandler_get_list(const char *name)
{
    /* XXX this should be dynamic */

    if(strcmp(name, "dev_clone") == 0)
      return &evh_list[0];
    if(strcmp(name, "shutdown_pre_sync") == 0)
      return &evh_list[1];
    if(strcmp(name, "shutdown_post_sync") == 0)
      return &evh_list[2];
    if(strcmp(name, "shutdown_final") == 0)
      return &evh_list[3];
    if(strcmp(name, "vm_lowmem") == 0)
      return &evh_list[4];
    if(strcmp(name, "process_exit") == 0)
      return &evh_list[5];
    if(strcmp(name, "process_fork") == 0)
      return &evh_list[6];
    if(strcmp(name, "process_exec") == 0)
      return &evh_list[7];
    else
      printf("unknown eventhandler list: %s! (FIXME)\n", name);

    return NULL;
}

eventhandler_tag 
eventhandler_register(struct eventhandler_list *list, 
		      const char *name, void *func, void *arg, 
		      int priority)
{
    struct eventhandler_entry *ptr;
    list = eventhandler_get_list(name);

    if(list == NULL)
    {
        return NULL;
    }

    mtx_lock(&list->el_mtx);
    ptr = list->el_root;
    while(ptr)
    {
        if(ptr->used == 0)
	{
	    ptr->used = 1;
	    break;
	}
	ptr = ptr->next;
    }
    mtx_unlock(&list->el_mtx);

    if(ptr == NULL)
    {
        ptr = malloc(sizeof(*ptr), M_TEMP, M_ZERO|M_WAITOK);
    }

    if(ptr == NULL)
    {
        return NULL;
    }

    ptr->func = func;
    ptr->arg = arg;

    if(ptr->used == 0)
    {
        ptr->used = 1;

	mtx_lock(&list->el_mtx);
	ptr->next = list->el_root;
	list->el_root = ptr;
	mtx_unlock(&list->el_mtx);
    }
    return ptr;
}

void
eventhandler_deregister(struct eventhandler_list *list, eventhandler_tag tag)
{
    struct eventhandler_entry *ptr;

    if(list == NULL)
    {
        return;
    }

    mtx_lock(&list->el_mtx);
    ptr = list->el_root;
    while(ptr)
    {
        if(ptr == tag)
	{
	    ptr->used = 0;
	    break;
	}
	ptr = ptr->next;
    }
    mtx_unlock(&list->el_mtx);
    return;
}
