/*-
 * Copyright (c) 2005
 *      Hans Petter Selasky. All rights reserved.
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
 */
#include <sys/param.h>
#include <sys/sysctl.h>
#include <sys/ioctl.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/mount.h>
#include <sys/exec.h>
#include <sys/lkm.h>
#include <sys/file.h>
#include <sys/errno.h>

#if ((__NetBSD_Version__ < 300000000) || (__NetBSD_Version__ >= 400000000))
#define pci_set_powerstate __pci_set_powerstate
#define pci_get_powerstate __pci_get_powerstate
#endif

#include <dev/pci/pcivar.h>
#include <dev/pci/pcidevs.h>

#undef pci_set_powerstate
#undef pci_get_powerstate

#include <sys/freebsd_compat.h>

extern int i4b_lkmentry(struct lkm_table *, int, int);

MOD_MISC("i4b");

static int
_pci_pci_probe(struct pci_attach_args *arg)
{
    struct __device *dev;
    static struct __device dummy_pci_dev;
    static struct bsd_module_data dev_module;
    static struct __driver driver;
    int error;

    if(sizeof(dev->dev_aux_data) < sizeof(*arg))
    {
        printf("%s: match needs %d bytes!\n",
	       __FUNCTION__, (u_int32_t)sizeof(*arg));
	return 0;
    }

    dev = malloc(sizeof(*dev), M_DEVBUF, M_WAITOK);

    if(dev == NULL)
    {
        printf("%s: out of memory!\n",
	       __FUNCTION__);
	return 0;
    }

    bzero(dev, sizeof(*dev));

    /* this structure must be available
     * after attach:
     */
    bcopy(arg, &(dev->dev_aux_data), sizeof(*arg));
    arg = (void *)&(dev->dev_aux_data);

    /* create a dummy parent PCI device */

    dummy_pci_dev.dev_module = &dev_module;
    dev_module.driver = &driver;
    driver.name = "pci";
    snprintf(&dummy_pci_dev.dev_nameunit[0], 
	     sizeof(dummy_pci_dev.dev_nameunit), "pci0");

    /* initialize "dev" structure */

    dev->dev_id = arg->pa_id;
    dev->dev_pci_class = arg->pa_class;
    dev->dev_dma_tag = arg->pa_dmat;
    dev->dev_what = DEVICE_IS_PCI;
    dev->dev_parent = &dummy_pci_dev;
    TAILQ_INIT(&dev->dev_children);

    device_set_ivars(dev, arg);

    snprintf(&dev->dev_desc[0], 
	     sizeof(dev->dev_desc), "unknown");

    mtx_lock(&Giant);

    error = device_probe_and_attach(dev);

    mtx_unlock(&Giant);

    if(error)
    {
        goto done;
    }

    device_printf(dev, "<%s> @ %s\n", device_get_desc(dev),
		  device_get_nameunit(device_get_parent(dev)));

 done:
    if(error)
    {
        free(dev, M_DEVBUF);
	dev = NULL;
    }

    /* continue search */
    return 0;
}

static void
do_sysinit(const struct sysinit *start, 
	   const struct sysinit *end,
	   u_int32_t level_first, 
	   u_int32_t level_last)
{
    const struct sysinit *sys;
    u_int32_t level_next;
    u_int32_t order_first;
    u_int32_t order_next;
    u_int32_t temp1;
    u_int32_t temp2;

    if(level_last < level_first)
    {
        level_last = level_first;
    }

    while(1)
    {
        level_next = level_last;
	order_first = 0;

	while(1)
	{
	    order_next = 0xffffffff;

	    for(sys = start;
		sys < end;
		sys++)
	    {
	        temp1 = sys->subsystem;
		temp2 = sys->order;

		if((temp1 == level_first) &&
		   (temp2 == order_first))
		{
#if 0
		    printf("calling %p(%p) @ %s:%d:\n", 
			   sys->func, sys->udata, 
			   sys->file, sys->line);
#endif
		    (sys->func)(sys->udata);
		}

		if((temp1 > level_first) &&
		   (temp1 < level_next))
		{
		    level_next = temp1;
		}

		if((temp2 > order_first) &&
		   (temp2 < order_next))
		{
		    order_next = temp2;
		}
	    }

	    if(order_first == 0xffffffff)
	    {
	        break;
	    }
	    order_first = order_next;
	}

	if(level_first == level_last)
	{
	    break;
	}
	level_first = level_next;
    }
    return;
}

static int
do_module(int command)
{
    struct module_called {
      void *callback;
      struct module_called *next;
    };
    struct module_called *root = NULL;
    struct module_called *temp;
    const struct bsd_module_data *mod;
    int error = 0;

    for(mod = &bsd_module_data_start[0]; 
	mod < &bsd_module_data_end[0];
	mod++)
    {
        if(mod->callback)
	{
	    temp = root;
	    while(temp)
	    {
	        if(temp->callback == mod->callback)
		{
		    goto skip;
		}
		temp = temp->next;
	    }

	    temp = __builtin_alloca(sizeof(*temp));
	    temp->next = root;
	    temp->callback = mod->callback;
	    root = temp;

	    error = (mod->callback)(NULL, command, mod->arg);

	    if(error)
	    {
	        break;
	    }
	}
    skip: ;
    }
    return error;
}

static int
load(struct lkm_table *p, int cmd)
{
    struct device *dev;
    int error;

    printf("FreeBSD 7.x emulation layer loaded. Probing devices ...\n");

    /*
     * call initializers first
     */
    do_sysinit(&bsd_sys_init_data_start[0], 
	       &bsd_sys_init_data_end[0],
	       0, SI_SUB_DRIVERS);

    /*
     * call module loaders
     */
    error = do_module(MOD_LOAD);
    if(error)
    {
        do_sysinit(&bsd_sys_uninit_data_start[0], 
		   &bsd_sys_uninit_data_end[0],
		   0, -1);

        printf("%s: error loading module(s): %d!\n",
	       __FUNCTION__, error);
	goto done;
    }

    /*
     * scan PCI
     */
    TAILQ_FOREACH(dev, &alldevs, dv_list)
    {
        if(!strcmp(dev->dv_cfdriver->cd_name, "pci"))
	{
#if (__NetBSD_Version__ >= 500000000)
	    static const int wildcard[16] = { -1, -1 };

	    pci_enumerate_bus(device_private(dev), &wildcard[0], &_pci_pci_probe, 0);
#elif (__NetBSD_Version__ >= 300000000)
	    static const int wildcard[16] = { -1, -1 };

	    pci_enumerate_bus((void *)dev, &wildcard[0], &_pci_pci_probe, 0);
#else
	    pci_enumerate_bus((void *)dev, &_pci_pci_probe, 0);
#endif
	}
    }

    /*
     * call initializers
     */
    do_sysinit(&bsd_sys_init_data_start[0], 
	       &bsd_sys_init_data_end[0],
	       SI_SUB_DRIVERS+1, -1);

 done:

    return error;
}

/* 
 * The following function is used when we have I4B loaded
 * into the NetBSD kernel to load the I4B module.
 */
extern void i4b_load(void);

void
i4b_load(void)
{
	if (load(NULL, 0)) {
		/* ignore any errors */
	}
}

static int
unload(struct lkm_table *p, int cmd)
{
    int error = 0;

    /*
     * Sorry, this is not supported
     */
    return EINVAL;

    /*
     * call module un-loaders
     */
    error = do_module(MOD_UNLOAD);
    if(error)
    {
        printf("%s: error un-loading module: %d!\n",
	       __FUNCTION__, error);
	goto done;
    }

    /*
     * call un-initializers
     */
    do_sysinit(&bsd_sys_uninit_data_start[0], 
	       &bsd_sys_uninit_data_end[0],
	       0, -1);

    /*
     * TODO: detach devices
     */

 done:

    return error;
}

/*
 * entry point
 */
int
i4b_lkmentry(struct lkm_table *lkmtp, int cmd, int ver)
{
    DISPATCH(lkmtp, cmd, ver, load, unload, lkm_nofunc)
}

