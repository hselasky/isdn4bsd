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
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/freebsd_compat.h>

#include <machine/stdarg.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcidevs.h>

#include <dev/isa/isavar.h>

#include <dev/isapnp/isapnpreg.h>
#include <dev/isapnp/isapnpvar.h>
#include <dev/isapnp/isapnpdevs.h>

static const char * const unknown_string = "unknown";

#if 0

static u_int8_t
get_hex(u_int8_t tmp)
{
    if((tmp >= 'A') && (tmp <= 'F'))
      tmp = (tmp - 'A' + 0xA);
    else if((tmp >= 'a') && (tmp <= 'f'))
      tmp = (tmp - 'a' + 0xA);
    else if((tmp >= '0') && (tmp <= '9'))
      tmp = (tmp - '0');
    else 
      tmp = 0;

    return tmp;
}

static u_int32_t
isapnp_vendor_to_id(const u_int8_t *src)
{
    u_int32_t id =
      (get_hex(src[6]) << 24) |
      (get_hex(src[5]) << 28) |
      (get_hex(src[4]) << 16) |
      (get_hex(src[3]) << 20) |
      (((src[2] - 'A' + 1) & 0x1f) << 8) |
      (((src[1] - 'A' + 1) & 0x07) << 13) |
      (((src[1] - 'A' + 1) & 0x18) >> 3) |
      (((src[0] - 'A' + 1) & 0x3F) << 2);

    return id;
}

static int
netbsd_pnp_match(device_t dev, struct isapnp_attach_args *arg)
{
    static struct __device dummy_isa_dev;
    static struct bsd_module_data dev_module;
    static struct __driver driver;

    /* create a dummy parent ISA device */

    dummy_isa_dev.dev_module = &dev_module;
    dev_module.driver = &driver;
    driver.name = "isa";

    snprintf(&dummy_isa_dev.dev_nameunit[0], sizeof(dummy_isa_dev.dev_nameunit), 
	     "isa0");

    /* initialize "dev" structure */

    bzero((&(dev->dev_local)) + 1, sizeof(*dev) - sizeof(dev->dev_local));

    dev->dev_id = isapnp_vendor_to_id(&arg->ipa_devlogic[0]);
    dev->dev_id_sub = isapnp_vendor_to_id(&arg->ipa_devcompat[0]);
    dev->dev_what = DEVICE_IS_PNP;
    dev->dev_parent = &dummy_isa_dev;

    snprintf(&dev->dev_desc[0], sizeof(dev->dev_desc), 
	     "%s", &arg->ipa_devident[0]);

    device_set_ivars(dev, arg);

    return (device_probe_and_attach(dev) ? 1 : 0);
}
#endif

u_int32_t
pci_read_config(device_t dev, int reg, int width)
{
    u_int32_t temp;

    switch(width) {
    case 0:
      temp = 0x00;
      break;
    case 1:
      temp = 0xFF;
      break;
    case 2:
      temp = 0xFFFF;
      break;
    case 3:
      temp = 0xFFFFFF;
      break;
    default:
      temp = 0xFFFFFFFF;
      break;
    }

    if(dev && (dev->dev_what == DEVICE_IS_PCI))
    {
        struct pci_attach_args *arg = device_get_ivars(dev);

	if(arg)
	{
	    temp &= pci_conf_read(arg->pa_pc, arg->pa_tag, reg);
	}
    }
    return temp;
}

void
pci_write_config(device_t dev, int reg, u_int32_t val, int width)
{
    u_int32_t temp;

    switch(width) {
    case 0:
      temp = 0x00;
      break;
    case 1:
      temp = 0xFF;
      break;
    case 2:
      temp = 0xFFFF;
      break;
    case 3:
      temp = 0xFFFFFF;
      break;
    default:
      temp = 0xFFFFFFFF;
      break;
    }

    if(dev && (dev->dev_what == DEVICE_IS_PCI))
    {
        struct pci_attach_args *arg = device_get_ivars(dev);

	if(arg)
	{
	    val &= temp;
	    temp = ~temp;
	    val |= pci_read_config(dev, reg, 4) & temp;
	    pci_conf_write(arg->pa_pc, arg->pa_tag, reg, val);
	}
    }
    return;
}

int
pci_enable_busmaster(device_t dev)
{
    u_int32_t temp;

    if(dev && (dev->dev_what == DEVICE_IS_PCI))
    {
        struct pci_attach_args *arg = device_get_ivars(dev);

	if(arg)
	{
	    temp = pci_conf_read(arg->pa_pc, arg->pa_tag, PCI_COMMAND_STATUS_REG);
	    temp |= PCI_COMMAND_MASTER_ENABLE;
	    pci_conf_write(arg->pa_pc, arg->pa_tag, PCI_COMMAND_STATUS_REG, temp);
	}
    }
    return 0;
}

int
pci_disable_busmaster(device_t dev)
{
    u_int32_t temp;

    if(dev && (dev->dev_what == DEVICE_IS_PCI))
    {
        struct pci_attach_args *arg = device_get_ivars(dev);

	if(arg)
	{
	    temp = pci_conf_read(arg->pa_pc, arg->pa_tag, PCI_COMMAND_STATUS_REG);
	    temp &= ~PCI_COMMAND_MASTER_ENABLE;
	    pci_conf_write(arg->pa_pc, arg->pa_tag, PCI_COMMAND_STATUS_REG, temp);
	}
    }
    return 0;
}

int
pci_enable_io(device_t dev, int space)
{
    u_int32_t temp;

    if(dev && (dev->dev_what == DEVICE_IS_PCI))
    {
        struct pci_attach_args *arg = device_get_ivars(dev);

	if(arg)
	{
	    temp = pci_conf_read(arg->pa_pc, arg->pa_tag, PCI_COMMAND_STATUS_REG);

	    switch(space) {
	    case SYS_RES_IOPORT:
	        temp |= PCI_COMMAND_IO_ENABLE;
		break;
	    case SYS_RES_MEMORY:
	        temp |= PCI_COMMAND_MEM_ENABLE;
		break;
	    }
	    pci_conf_write(arg->pa_pc, arg->pa_tag, PCI_COMMAND_STATUS_REG, temp);
	}
    }
    return 0;
}

int
pci_disable_io(device_t dev, int space)
{
    u_int32_t temp;

    if(dev && (dev->dev_what == DEVICE_IS_PCI))
    {
        struct pci_attach_args *arg = device_get_ivars(dev);

        if(arg)
	{
	    temp = pci_conf_read(arg->pa_pc, arg->pa_tag, PCI_COMMAND_STATUS_REG);

	    switch(space) {
	    case SYS_RES_IOPORT:
	        temp &= ~PCI_COMMAND_IO_ENABLE;
		break;
	    case SYS_RES_MEMORY:
	        temp &= ~PCI_COMMAND_MEM_ENABLE;
		break;
	    }
	    pci_conf_write(arg->pa_pc, arg->pa_tag, PCI_COMMAND_STATUS_REG, temp);
	}
    }
    return 0;
}

static struct resource *
alloc_res(device_t dev)
{
    u_int16_t x = DEVICE_MAXRES;
    struct resource *res = &dev->dev_resources[0];

    while(x)
    {
        if(res->r_in_use ||
	   res->r_alloced)
	{
	    res++;
	    x--;
	    continue;
	}
	bzero(res, sizeof(*res));
	res->r_in_use = 1;
	return res;
    }
    return NULL;
}

static struct resource *
find_res(device_t dev, u_int8_t type, u_int32_t rid)
{
    struct resource *res = &dev->dev_resources[0];
    u_int16_t x = DEVICE_MAXRES;

    while(x)
    {
        if(res->r_in_use && !res->r_alloced &&
	   (res->r_rid == rid) &&
	   (res->r_type == type))
	{
	    break;
	}
	res++;
	x--;
    }
    if(x == 0)
      res = NULL;
    return res;
}

static void
free_res(struct resource *res)
{
    res->r_in_use = 0;
    res->r_alloced = 0;
    return;
}

struct resource *
bus_alloc_resource(device_t dev, int type, int *rid, u_int32_t start, 
		   u_int32_t end, u_int32_t count, u_int32_t flags)
{
    struct resource *res = NULL;
    bus_addr_t base;
    bus_size_t size;

    if(rid == NULL) goto error;

    if((start == 0) && (end == (u_int32_t)(-1)))
    {
      res = find_res(dev, type, rid[0]);

      if(res)
      {
	  /* already allocated */
	  goto error;
      }

      res = alloc_res(dev);
      if(res == NULL)
      {
	  /* out of memory */
	  goto error;
      }

      if(dev->dev_what == DEVICE_IS_PNP)
      {
	  struct isapnp_attach_args *arg = device_get_ivars(dev);
	  u_int32_t n;

	  if(arg == NULL)
	  {
	      goto error;
	  }

	  if(dev->dev_res_alloc == 0)
	  {
	      if(isapnp_config(arg->ipa_iot, arg->ipa_memt, arg))
	      {
		  /* resource error */
		  goto error;
	      }
	      dev->dev_res_alloc = 1;
	  }

	  n = rid[0];

	  switch(type) {
	  case SYS_RES_IOPORT:

	    if(n >= arg->ipa_nio)
	    {
	        goto error;
	    }

	    res->r_bustag = arg->ipa_iot;
	    res->r_start = arg->ipa_io[n].base;
	    res->r_end = 
	      arg->ipa_io[n].base +
	      arg->ipa_io[n].length - 1;
	    res->r_bushandle = arg->ipa_io[n].h;
	    res->r_rid = n;
	    res->r_type = SYS_RES_IOPORT;
	    break;

	  case SYS_RES_MEMORY:

	    if(n >= arg->ipa_nmem)
	    {
	        goto error;
	    }

	    res->r_bustag = arg->ipa_memt;
	    res->r_start = arg->ipa_mem[n].base;
	    res->r_end = 
	      arg->ipa_mem[n].base +
	      arg->ipa_mem[n].length - 1;
	    res->r_bushandle = arg->ipa_mem[n].h;
	    res->r_rid = n;
	    res->r_type = SYS_RES_MEMORY;
	    break;

	  case SYS_RES_IRQ:

	    if(n >= arg->ipa_nirq)
	    {
	        goto error;
	    }

	    res->r_start = arg->ipa_irq[n].num;
	    res->r_end = arg->ipa_irq[n].num;
	    res->r_intr_type = arg->ipa_irq[n].type;
	    res->r_rid = n;
	    res->r_type = SYS_RES_IRQ;
	    break;

	  case SYS_RES_DRQ:

	    if(n >= arg->ipa_ndrq)
	    {
	        goto error;
	    }

	    res->r_dmatag = arg->ipa_dmat;
	    res->r_start = arg->ipa_drq[n].num;
	    res->r_end = arg->ipa_drq[n].num;
	    res->r_rid = n;
	    res->r_type = SYS_RES_DRQ;
	    break;

	  default:
	    goto error;
	  }
      }
#if 0
      else if(dev->dev_what == DEVICE_IS_ISA)
      {

      }
#endif
      else if(dev->dev_what == DEVICE_IS_PCI)
      {
	  struct pci_attach_args *arg = device_get_ivars(dev);

	  if(arg == NULL)
	  {
	      goto error;
	  }

	  switch(type) {
	  case SYS_RES_IOPORT:
	    if(pci_mapreg_map(arg, rid[0], 
			      PCI_MAPREG_TYPE_IO, 0, 
			      &res->r_bustag, &res->r_bushandle,
			      &base, &size))
	    {
	        goto error;
	    }
	    res->r_start = base;
	    res->r_end = base + size -1;
	    res->r_rid = rid[0];
	    res->r_type = type;
	    break;

	  case SYS_RES_MEMORY:
	    if(pci_mapreg_map(arg, rid[0], 
			      (PCI_MAPREG_TYPE_MEM | PCI_MAPREG_MEM_TYPE_32BIT), 0,
			      &res->r_bustag, &res->r_bushandle,
			      &base, &size))
	    {
	        goto error;
	    }
	    res->r_start = base;
	    res->r_end = base + size -1;
	    res->r_rid = rid[0];
	    res->r_type = type;
	    break;

	  case SYS_RES_IRQ:
	    if(rid[0] != 0)
	    {
	        goto error;
	    }

	    res->r_start =
	      res->r_end =
	      pci_read_config(dev, PCI_INTERRUPT_REG, 1);
	    res->r_rid = rid[0];
	    res->r_type = type;
	    break;

	  default:
	    goto error;
	  }
      }
      else
      {
	  goto error;
      }
    }
    else
    {
      /* no support */
      goto error;
    }

    dev->dev_res_alloc_count++;
    res->r_in_use = 1;
    return res;

 error:
    if(res)
    {
      free_res(res);
      res = NULL;
    }
    return res;
}

int
bus_release_resource(device_t dev, int type, int rid, 
		     struct resource *res)
{
    int error = ENXIO;
    if(dev && res)
    {
        if(res->r_alloced)
	{
	  switch(res->r_type) {
	  case SYS_RES_IOPORT:
	  case SYS_RES_MEMORY:
	      bus_space_unmap(res->r_bustag, res->r_bushandle,
			      res->r_end - res->r_start + 1);
	      break;
	  }
	}
	free_res(res);
	error = 0;

	if(dev->dev_res_alloc_count)
	{
	    dev->dev_res_alloc_count--;
	}

	if((dev->dev_res_alloc_count == 0) &&
	   (dev->dev_res_alloc))
	{
	    if(dev->dev_what == DEVICE_IS_PNP)
	    {
	        struct isapnp_attach_args *arg = device_get_ivars(dev);

		if(arg)
		{
		    isapnp_unconfig(arg->ipa_iot, arg->ipa_memt, arg);
		}
	    }
	    dev->dev_res_alloc = 0;
	}
    }
    return error;
}

int
bus_set_resource(device_t dev, int type, int rid, u_int32_t start, 
		 u_int32_t count)
{
    u_int16_t x = DEVICE_MAXRES;
    struct resource *res = &dev->dev_resources[0];

    if(count == 0)
    {
        return EINVAL;
    }

    while(x)
    {
        if(res->r_in_use ||
	   res->r_alloced)
	{
	    if((res->r_rid == rid) &&
	       (res->r_type == type))
	    {
	        goto found;
	    }
	}
	res++;
	x--;
    }

    res = alloc_res(dev);

 found:
    if(res == NULL)
    {
        return ENOMEM;
    }

    res->r_rid = rid;
    res->r_type = type;
    res->r_start = start;
    res->r_end = start+count-1;

    return 0;
}

static int
interrupt_wrapper(void *arg)
{
    device_t dev = arg;
    (dev->dev_intr_func)(dev->dev_intr_arg);
    return 1;
}

int
bus_setup_intr(device_t dev, struct resource *res, int flags,
	       driver_intr_t *handler, void *priv, void **cookiep)
{
    if(cookiep == NULL)
    {
        return EINVAL;
    }

    if(dev->dev_intr_func)
    {
        printf("%s: sorry, only one interrupt handler "
	       "supported per unit.\n", __FUNCTION__);
	return ENXIO;
    }

    cookiep[0] = NULL;

    if(dev->dev_what == DEVICE_IS_PNP)
    {
        struct isapnp_attach_args *arg = device_get_ivars(dev);

	if(arg)
	{
	    cookiep[0] = 
	      isa_intr_establish(arg->ipa_ic, res->r_start,
				 res->r_intr_type, IPL_NET,
				 &interrupt_wrapper, dev); 
	}
    }
    else if(dev->dev_what == DEVICE_IS_ISA)
    {
        struct isa_attach_args *arg = device_get_ivars(dev);

	if(arg)
	{
	    cookiep[0] = 
	      isa_intr_establish(arg->ia_ic, res->r_start,
				 IST_EDGE, IPL_NET,
				 &interrupt_wrapper, dev);
	}
    }
    else if(dev->dev_what == DEVICE_IS_PCI)
    {
        struct pci_attach_args *arg = device_get_ivars(dev);

	if(arg)
	{
	    pci_intr_handle_t intr_h;

	    if(pci_intr_map(arg, &intr_h))
	    {
	        return ENXIO;
	    }

	    cookiep[0] = 
	      pci_intr_establish(arg->pa_pc, intr_h,
				 IPL_NET, &interrupt_wrapper, dev);
	}
    }

    if(cookiep[0] == NULL)
    {
        return ENXIO;
    }

    dev->dev_intr_func = handler;
    dev->dev_intr_arg = priv;
    return 0;
}

int
bus_teardown_intr(device_t dev, struct resource *r, void *cookie)
{
    if(cookie == NULL)
    {
        return 0;
    }

    if(dev->dev_what == DEVICE_IS_PNP)
    {
	  struct isapnp_attach_args *arg = device_get_ivars(dev);
	  if(arg)
	  {
	      isa_intr_disestablish(arg->ipa_ic, cookie);
	  }
    }
    else if(dev->dev_what == DEVICE_IS_ISA)
    {
        struct isa_attach_args *arg = device_get_ivars(dev);
	if(arg)
	{
	    isa_intr_disestablish(arg->ia_ic, cookie);
	}
    }
    else if(dev->dev_what == DEVICE_IS_PCI)
    {
        struct pci_attach_args *arg = device_get_ivars(dev);
	if(arg)
	{
	    pci_intr_disestablish(arg->pa_pc, cookie);
	}
    }

    dev->dev_intr_func = NULL;
    dev->dev_intr_arg = NULL;
    return 0;
}

const char *
device_get_nameunit(device_t dev)
{
    if(dev && dev->dev_nameunit[0])
      return &(dev->dev_nameunit[0]);
    else
      return unknown_string;
}

int
__device_printf(device_t dev, const char *fmt, ...)
{
    const char *indent;
    va_list ap;

    if(dev && dev->dev_nameunit[0])
      indent = &dev->dev_nameunit[0];
    else
      indent = unknown_string;

    printf("%s: ", indent);

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    return 0; /* XXX should return number of characters printed */
}

device_t 
device_add_child(device_t dev, const char *name, int unit)
{
    device_printf(dev, "%s not supported!\n", __FUNCTION__);
    return NULL;
}

void
device_quiet(device_t dev)
{
    dev->dev_quiet = 1;
    return;
}

const char *
device_get_desc(device_t dev)
{
    if(dev)
      return &(dev->dev_desc[0]);
    else
      return unknown_string;
}

static void
default_method(void)
{
    /* do nothing */
    return;
}

void *
device_get_method(device_t dev, const char *what)
{
    void *temp = NULL;
    const struct __device_method *mtod;

    if(dev && 
       dev->dev_module &&
       dev->dev_module->driver &&
       dev->dev_module->driver->methods)
    {
        mtod = dev->dev_module->driver->methods;
	while(mtod->desc && mtod->func)
	{
	  if(strcmp(mtod->desc, what) == 0)
	  {
	      temp = mtod->func;
	      break;
	  }
	  mtod++;
	}
    }
    if(temp == NULL)
    {
        /* maybe one should panic here */
        temp = &default_method;
	device_printf(dev, "%s: WARNING: missing method: %s!\n",
		      __FUNCTION__, what);
    }
    return temp;
}

const char *
device_get_name(device_t dev)
{
    const struct bsd_module_data *mod = dev ? dev->dev_module : NULL;

    if(mod &&
       mod->driver &&
       mod->driver->name)
      return mod->driver->name;
    else
      return unknown_string;
}

int
device_probe_and_attach(device_t dev)
{
    const struct bsd_module_data *mod;
    const u_int8_t *bus_name_parent = device_get_name(device_get_parent(dev));
    u_int16_t n;

    if(dev->dev_attached)
    {
        return 0;
    }

    /*
     * find a module for our device, if any
     */
    for(mod = &bsd_module_data_start[0]; 
	mod < &bsd_module_data_end[0];
	mod++)
    {
        if(mod->bus_name && 
	   mod->driver && 
	   mod->driver->methods &&
	   mod->devclass && 
	   mod->driver->name &&
	   (strcmp(mod->bus_name, bus_name_parent) == 0))
	{
	    for(n = 0; n < DEVCLASS_MAXUNIT; n++)
	    {
	        if(mod->devclass->dev_list[n])
		{
		    continue;
		}

		mod->devclass->dev_list[n] = dev;

		dev->dev_unit = n;
		dev->dev_module = mod;
		snprintf(&dev->dev_nameunit[0], sizeof(dev->dev_nameunit), 
			 "%s%d", mod->driver->name, n);

		if(CALL_METHOD(dev, device_probe, dev) == 0)
		{
		    if(CALL_METHOD(dev, device_attach, dev) == 0)
		    {
		        /* success */
		        dev->dev_attached = 1;
			return 0;
		    }
		}

		/* else try next driver */

		device_detach(dev);

		break;
	    }
	}
    }
    return ENODEV;
}

int
device_detach(device_t dev)
{
    const struct bsd_module_data *mod = dev->dev_module;
    int error;

    if(mod == NULL)
    {
        device_printf(dev, "invalid device: dev->dev_module == NULL!\n");
	return 0;
    }

    if(dev->dev_attached)
    {
       error = CALL_METHOD(dev, device_detach, dev);
       if(error)
       {
	   return error;
       }

       dev->dev_attached = 0;
    }

    device_set_softc(dev, NULL);
    dev->dev_softc_set = 0;

    if(devclass_get_device(mod->devclass, dev->dev_unit) == dev)
    {
        mod->devclass->dev_list[dev->dev_unit] = NULL;
    }
    else
    {
        device_printf(dev, "warning device is not present in devclass!\n");
    }

    dev->dev_module = NULL;
    return 0;
}

void
device_set_softc(device_t dev, void *softc)
{
    if(dev->dev_softc_alloc)
    {
        free(dev->dev_sc, M_DEVBUF);
        dev->dev_sc = NULL;
    }

    dev->dev_sc = softc;
    dev->dev_softc_alloc = 0;
    dev->dev_softc_set = 1;
    return;
}

void *
device_get_softc(device_t dev)
{
    const struct bsd_module_data *mod = dev->dev_module;

    if((!dev->dev_softc_set) &&
       (!dev->dev_softc_alloc) &&
       mod && 
       mod->driver &&
       mod->driver->size)
    {
        dev->dev_sc = malloc(mod->driver->size, M_DEVBUF, M_WAITOK);

	if(dev->dev_sc)
	{
	    dev->dev_softc_set = 1;
	    dev->dev_softc_alloc = 1;
	}
    }
    return dev->dev_sc;
}

int
device_delete_child(device_t dev, device_t child)
{
    device_printf(dev, "%s not supported!\n", __FUNCTION__);
    return ENXIO;
}

int
device_is_attached(device_t dev)
{
    return dev->dev_attached;
}

void
device_set_desc(device_t dev, const char * desc)
{
    snprintf(&dev->dev_desc[0], sizeof(dev->dev_desc), "%s", desc);
    return;
}

void *
devclass_get_softc(devclass_t *dc, int unit)
{
    return device_get_softc(devclass_get_device(dc,unit));
}

int 
devclass_get_maxunit(devclass_t *dc)
{
    int max_unit = 0;

    if(dc)
    {
        max_unit = DEVCLASS_MAXUNIT;
        while(max_unit--)
	{
	  if(dc->dev_list[max_unit])
	  {
	      break;
	  }
	}
	max_unit++;
    }
    return max_unit;
}

device_t
devclass_get_device(devclass_t *dc, int unit)
{
    return ((unit < 0) || (unit >= DEVCLASS_MAXUNIT) || (dc == NULL)) ? 
      NULL : dc->dev_list[unit];
}

devclass_t *
devclass_find(const char *classname)
{
    const struct bsd_module_data *mod;

    for(mod = &bsd_module_data_start[0]; 
	mod < &bsd_module_data_end[0];
	mod++)
    {
        if(mod->devclass && mod->mod_name &&
	   (strcmp(classname, mod->mod_name) == 0))
	{
	    return mod->devclass;
	}
    }
    return NULL;
}

struct usb_dma {
    bus_dma_tag_t     tag;
    bus_dmamap_t      map;
    bus_dma_segment_t seg;
    int               seg_count;
};

void *
usb_alloc_mem(u_int32_t size, u_int8_t align_power)
{
    caddr_t ptr = NULL;
    struct usb_dma temp;

    bzero(&temp, sizeof(temp));

    temp.tag = &pci_bus_dma_tag;
    temp.seg_count = 1;

    size += sizeof(struct usb_dma);

    if(bus_dmamem_alloc(temp.tag, size, (1 << align_power), 0,
			&temp.seg, 1,
			&temp.seg_count, BUS_DMA_NOWAIT))
    {
        goto done_4;
    }

    if(bus_dmamem_map(temp.tag, &temp.seg, temp.seg_count, size,
		      &ptr, BUS_DMA_NOWAIT|BUS_DMA_COHERENT))
    {
        goto done_3;
    }

    if(bus_dmamap_create(temp.tag, size, 1, size,
			 0, BUS_DMA_NOWAIT, &temp.map))
    {
        goto done_2;
    }

    if(bus_dmamap_load(temp.tag, temp.map, ptr, size, NULL, 
		       BUS_DMA_NOWAIT))
    {
        goto done_1;
    }

    size -= sizeof(temp);

    bcopy(&temp, ((u_int8_t *)ptr) + size, sizeof(temp));

    return ptr;

 done_1:
    bus_dmamap_destroy(temp.tag, temp.map);

 done_2:
    bus_dmamem_unmap(temp.tag, ptr, size);

 done_3:
    bus_dmamem_free(temp.tag, &temp.seg, temp.seg_count);

 done_4:
    return NULL;
}

u_int32_t
usb_vtophys(void *ptr, u_int32_t size)
{
    struct usb_dma *arg = (void *)(((u_int8_t *)ptr) + size);
    return arg->seg.ds_addr;
}

void
usb_free_mem(void *ptr, u_int32_t size)
{
    struct usb_dma *arg = (void *)(((u_int8_t *)ptr) + size);

    bus_dmamap_unload(arg->tag, arg->map);
    bus_dmamap_destroy(arg->tag, arg->map);
    bus_dmamem_unmap(arg->tag, ptr, size);
    bus_dmamem_free(arg->tag, &arg->seg, arg->seg_count);

    return;
}
