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
 * "module/freebsd_kern_bus.c"
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/freebsd_compat.h>

#include <machine/stdarg.h>

#if ((__NetBSD_Version__ < 300000000) || (__NetBSD_Version__ >= 400000000))
#define pci_set_powerstate __pci_set_powerstate
#define pci_get_powerstate __pci_get_powerstate
#endif

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcidevs.h>

#undef pci_set_powerstate
#undef pci_get_powerstate

#ifndef FREEBSD_NO_ISA

#include <dev/isa/isavar.h>

#include <dev/isapnp/isapnpreg.h>
#include <dev/isapnp/isapnpvar.h>
#include <dev/isapnp/isapnpdevs.h>

#endif

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
    TAILQ_INIT(&dev->dev_children);

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
	    val |= pci_read_config(dev, reg, 4) & (~temp);
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

u_int8_t
pci_get_class(device_t dev)
{
    return PCI_CLASS(dev->dev_pci_class);
}

u_int8_t
pci_get_subclass(device_t dev)
{
    return PCI_SUBCLASS(dev->dev_pci_class);
}

u_int8_t
pci_get_progif(device_t dev)
{
    return PCI_INTERFACE(dev->dev_pci_class);
}

u_int16_t
pci_get_vendor(device_t dev)
{
    return (dev->dev_id & 0xFFFF);
}

u_int16_t
pci_get_device(device_t dev)
{
    return ((dev->dev_id >> 16) & 0xFFFF);
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

#ifndef FREEBSD_NO_ISA

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
      else 
#endif
	if(dev->dev_what == DEVICE_IS_PCI)
      {
	  struct pci_attach_args *arg = device_get_ivars(dev);

	  if(arg == NULL)
	  {
	      goto error;
	  }

	  switch(type) {
	  case SYS_RES_IOPORT:
	    if ((rid[0] < PCI_MAPREG_START) || (rid[0] & 3)) {
	        printf("%s: Invalid I/O-port rid: 0x%x\n", __FUNCTION__, rid[0]);
		goto error;
	    }

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
	    if ((rid[0] < PCI_MAPREG_START) || (rid[0] & 3)) {
	        printf("%s: Invalid memory rid: 0x%x\n", __FUNCTION__, rid[0]);
		goto error;
	    }

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
#ifndef FREEBSD_NO_ISA
	    if(dev->dev_what == DEVICE_IS_PNP)
	    {
	        struct isapnp_attach_args *arg = device_get_ivars(dev);

		if(arg)
		{
		    isapnp_unconfig(arg->ipa_iot, arg->ipa_memt, arg);
		}
	    }
#endif
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

#ifndef FREEBSD_NO_ISA
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
    else 
#endif
      if(dev->dev_what == DEVICE_IS_PCI)
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

#ifndef FREEBSD_NO_ISA
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
    else
#endif
      if(dev->dev_what == DEVICE_IS_PCI)
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

int32_t
bus_generic_detach(device_t dev)
{
    device_t child;
    int error;

    if (!dev->dev_attached)
	return (EBUSY);

    TAILQ_FOREACH(child, &dev->dev_children, dev_link) {
	if ((error = device_detach(child)) != 0)
        	return (error);
    }

    return (0);
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

static u_int8_t
devclass_create(devclass_t *dc_pp)
{
    if (dc_pp == NULL) {
        return 1;
    }

    if (dc_pp[0] == NULL) {
        dc_pp[0] = malloc(sizeof(**(dc_pp)), 
			  M_DEVBUF, M_WAITOK|M_ZERO);

	if (dc_pp[0] == NULL) {
	    return 1;
	}
    }
    return 0;
}

static const struct bsd_module_data *
devclass_find_create(const char *classname)
{
    const struct bsd_module_data *mod;

    for(mod = &bsd_module_data_start[0]; 
	mod < &bsd_module_data_end[0];
	mod++)
    {
        if(mod->mod_name &&
	   (strcmp(classname, mod->mod_name) == 0))
	{
	    if(devclass_create(mod->devclass_pp))
	    {
	        continue;
	    }
	    return mod;
	}
    }
    return NULL;
}

static u_int8_t
devclass_add_device(const struct bsd_module_data *mod, device_t dev)
{
    u_int16_t n;
    const char *name = "unknown";

    if ((mod->devclass_pp == NULL) ||
	(mod->devclass_pp[0] == NULL)) {
        return 1;
    }

    if(mod->driver &&
       mod->driver->name) {
        name = mod->driver->name;
    }

    if (dev->dev_unit_manual) {

        n = dev->dev_unit;

	if ((n < DEVCLASS_MAXUNIT) &&
	    (mod->devclass_pp[0]->dev_list[n] == NULL))
	{
	    goto found;
	}

    } else {

        for(n = 0; n < DEVCLASS_MAXUNIT; n++)
	{
	    if(mod->devclass_pp[0]->dev_list[n] == NULL)
	    {
	        goto found;
	    }
	}
    }

    return 1;

 found:

    mod->devclass_pp[0]->dev_list[n] = dev;

    dev->dev_unit = n;
    dev->dev_module = mod;
    snprintf(&dev->dev_nameunit[0], 
	     sizeof(dev->dev_nameunit), "%s%d", name, n);

    return 0;
}

static void
devclass_delete_device(const struct bsd_module_data *mod, device_t dev)
{
    if (mod == NULL) {
        return;
    }

    if(devclass_get_device(mod->devclass_pp ? 
			   mod->devclass_pp[0] : NULL, dev->dev_unit) == dev)
    {
        mod->devclass_pp[0]->dev_list[dev->dev_unit] = NULL;
    }
    else
    {
        device_printf(dev, "WARNING: device is not present in devclass!\n");
    }
    dev->dev_module = NULL;
    return;
}

static device_t
make_device(device_t parent, const char *name, int unit)
{
    device_t dev = NULL;
    const struct bsd_module_data *mod = NULL;

    if (name) {

        mod = devclass_find_create(name);

	if (!mod) {

	    printf("%s:%d:%s: can't find device "
		   "class %s\n", __FILE__, __LINE__,
		   __FUNCTION__, name);
	    goto done;
        }
    }

    dev = malloc(sizeof(*dev), M_DEVBUF, M_NOWAIT|M_ZERO);

    if (dev == NULL) goto done;

    dev->dev_parent = parent;
    TAILQ_INIT(&dev->dev_children);

    if (unit != -1) {
        dev->dev_unit_manual = 1;
	dev->dev_unit = unit;
    }

    if (name) {
        dev->dev_fixed_class = 1;
	dev->dev_module = mod;

	if (devclass_add_device(mod, dev)) {
	    goto error;
	}
    }

 done:
    return dev;

 error:
    if (dev) {
        free(dev, M_DEVBUF);
    }
    return NULL;
}

device_t
device_add_child_ordered(device_t dev, int order, const char *name, int unit)
{
    device_t child;
    device_t place;

    child = make_device(dev, name, unit);
    if (child == NULL) {
        goto done;
    }
    child->dev_order = order;

    TAILQ_FOREACH(place, &dev->dev_children, dev_link) {
        if (place->dev_order > order) {
	    break;
	}
    }

    if (place) {
        /*
	 * The device 'place' is the first device whose order is
	 * greater than the new child.
	 */
        TAILQ_INSERT_BEFORE(place, child, dev_link);
    } else {
        /*
	 * The new child's order is greater or equal to the order of
	 * any existing device. Add the child to the tail of the list.
	 */
        TAILQ_INSERT_TAIL(&dev->dev_children, child, dev_link);
    }

 done:
    return child;
}

device_t 
device_add_child(device_t dev, const char *name, int unit)
{
    return device_add_child_ordered(dev, 0, name, unit);
}

int
device_delete_child(device_t dev, device_t child)
{
    int error = 0;
    device_t grandchild;

    /* remove children first */

    while ( (grandchild = TAILQ_FIRST(&child->dev_children)) ) {
        error = device_delete_child(child, grandchild);
	if (error) goto done;
    }

    error = device_detach(child);

    if (error) goto done;

    devclass_delete_device(child->dev_module, child);

    TAILQ_REMOVE(&dev->dev_children, child, dev_link);

    free(child, M_DEVBUF);

 done:
    return error;
}

int
device_get_children(device_t dev, device_t **devlistp, int *devcountp)
{
    int count;
    int n;
    device_t child;
    device_t *list;

    count = 0;
    TAILQ_FOREACH(child, &dev->dev_children, dev_link) {
	count++;
    }

    if (count == 0) {
        /* avoid zero size allocation */
        n = 1 * sizeof(device_t);
    } else {
        n = count * sizeof(device_t);
    }

    list = malloc(n, M_TEMP, M_NOWAIT|M_ZERO);
    if (!list) {
        *devlistp = NULL;
	*devcountp = 0;
	return ENOMEM;
    }

    n = 0;
    TAILQ_FOREACH(child, &dev->dev_children, dev_link) {
        if (n < count) {
	    list[n] = child;
	    n++;
	}
    }

    if (n != count) {
        printf("%s: WARNING: Number of "
	       "devices changed %d -> %d!\n",
	       __FILE__, count, n);
    }

    *devlistp = list;
    *devcountp = count;

    return 0;
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

    if(dev->dev_attached)
    {
        return 0;
    }

    if (dev->dev_fixed_class) {

        mod = dev->dev_module;

	if(CALL_METHOD(dev, device_probe, dev) <= 0)
	{
	    if(CALL_METHOD(dev, device_attach, dev) == 0)
	    {
	        /* success */
	        dev->dev_attached = 1;
		return 0;
	    }
	}

	device_detach(dev);

	goto error;
    }

    /*
     * else find a module for our device, if any
     */
    for(mod = &bsd_module_data_start[0]; 
	mod < &bsd_module_data_end[0];
	mod++)
    {
        if(mod->bus_name && 
	   mod->driver && 
	   mod->driver->methods &&
	   mod->driver->name &&
	   (strcmp(mod->bus_name, bus_name_parent) == 0))
	{
	    if (devclass_create(mod->devclass_pp)) {
	        device_printf(dev, "devclass_create_() failed!\n");
	        continue;
	    }

	    if (devclass_add_device(mod, dev)) {
	        device_printf(dev, "devclass_add_device() failed!\n");
	        continue;
	    }

	    if(CALL_METHOD(dev, device_probe, dev) <= 0)
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
	}
    }

 error:
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

    if (dev->dev_fixed_class == 0) {
        devclass_delete_device(mod, dev);
    }
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
    const struct bsd_module_data *mod;

    if (dev == NULL) {
        return NULL;
    }

    mod = dev->dev_module;

    if((!dev->dev_softc_set) &&
       (!dev->dev_softc_alloc) &&
       mod && 
       mod->driver &&
       mod->driver->size)
    {
        dev->dev_sc = malloc(mod->driver->size, 
			     M_DEVBUF, M_WAITOK|M_ZERO);

	if(dev->dev_sc)
	{
	    dev->dev_softc_set = 1;
	    dev->dev_softc_alloc = 1;
	}
    }
    return dev->dev_sc;
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

void
device_set_desc_copy(device_t dev, const char * desc)
{
    device_set_desc(dev, desc);
    return;
}

void *
devclass_get_softc(devclass_t dc, int unit)
{
    return device_get_softc(devclass_get_device(dc,unit));
}

int 
devclass_get_maxunit(devclass_t dc)
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
devclass_get_device(devclass_t dc, int unit)
{
    return ((unit < 0) || (unit >= DEVCLASS_MAXUNIT) || (dc == NULL)) ? 
      NULL : dc->dev_list[unit];
}

devclass_t 
devclass_find(const char *classname)
{
    const struct bsd_module_data *mod;

    for(mod = &bsd_module_data_start[0]; 
	mod < &bsd_module_data_end[0];
	mod++)
    {
        if(mod->devclass_pp && 
	   mod->devclass_pp[0] &&
	   mod->mod_name &&
	   (strcmp(classname, mod->mod_name) == 0))
	{
	    return mod->devclass_pp[0];
	}
    }
    return NULL;
}

#ifndef IHFC_USB_ENABLED

#ifdef __NetBSD__

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_create - allocate a DMA tag
 *
 * NOTE: If the "align" parameter has a value of 1 the DMA-tag will
 * allow multi-segment mappings. Else all mappings are single-segment.
 *------------------------------------------------------------------------*/
static void
usbd_dma_tag_create(struct usbd_dma_tag *udt,
    uint32_t size, uint32_t align)
{
	uint32_t nseg;

	if (align == 1) {
		nseg = (2 + (size / USB_PAGE_SIZE));
	} else {
		nseg = 1;
	}

	udt->p_seg = malloc(nseg * sizeof(*(udt->p_seg)),
	    M_DEVBUF, M_WAITOK | M_ZERO);

	if (udt->p_seg == NULL) {
		return;
	}
	udt->tag = udt->tag_parent->tag;
	udt->n_seg = nseg;
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_free - free a DMA tag
 *------------------------------------------------------------------------*/
static void
usbd_dma_tag_destroy(struct usbd_dma_tag *udt)
{
	free(udt->p_seg, M_DEVBUF);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_find - factored out code
 *------------------------------------------------------------------------*/
struct usbd_dma_tag *
usbd_dma_tag_find(struct usbd_dma_parent_tag *udpt,
    uint32_t size, uint32_t align)
{
	struct usbd_dma_tag *udt;
	uint8_t nudt;

	__KASSERT(align > 0, ("Invalid parameter align = 0!\n"));
	__KASSERT(size > 0, ("Invalid parameter size = 0!\n"));

	udt = udpt->utag_first;
	nudt = udpt->utag_max;

	while (nudt--) {

		if (udt->align == 0) {
			usbd_dma_tag_create(udt, size, align);
			if (udt->tag == NULL) {
				return (NULL);
			}
			udt->align = align;
			udt->size = size;
			return (udt);
		}
		if ((udt->align == align) && (udt->size == size)) {
			return (udt);
		}
		udt++;
	}
	return (NULL);
}

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_setup - initialise USB DMA tags
 *------------------------------------------------------------------------*/
void
usbd_dma_tag_setup(struct usbd_dma_parent_tag *udpt,
    struct usbd_dma_tag *udt, bus_dma_tag_t dmat,
    struct mtx *mtx, usbd_dma_callback_t *func,
    struct usbd_memory_info *info, uint8_t ndmabits,
    uint8_t nudt)
{
	bzero(udpt, sizeof(*udpt));

	/* sanity checking */
	if ((nudt == 0) ||
	    (ndmabits == 0) ||
	    (mtx == NULL)) {
		/* something is corrupt */
		return;
	}

	/* store some information */
	udpt->mtx = mtx;
	udpt->info = info;
	udpt->func = func;
	udpt->tag = dmat;
	udpt->utag_first = udt;
	udpt->utag_max = nudt;
	udpt->dma_bits = ndmabits;

	while (nudt--) {
		bzero(udt, sizeof(*udt));
		udt->tag_parent = udpt;
		udt++;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_dma_tag_unsetup - factored out code
 *------------------------------------------------------------------------*/
void
usbd_dma_tag_unsetup(struct usbd_dma_parent_tag *udpt)
{
	struct usbd_dma_tag *udt;
	uint8_t nudt;

	udt = udpt->utag_first;
	nudt = udpt->utag_max;

	while (nudt--) {

		if (udt->align) {
			/* destroy the USB DMA tag */
			usbd_dma_tag_destroy(udt);
			udt->align = 0;
		}
		udt++;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_common_mem_cb - BUS-DMA callback function
 *------------------------------------------------------------------------*/
static void
usbd_pc_common_mem_cb(struct usbd_page_cache *pc, bus_dma_segment_t *segs,
    int nseg, int error, uint8_t isload)
{
	struct usbd_dma_parent_tag *uptag;
	struct usbd_page *pg;
	uint32_t rem;
	uint8_t ext_seg;		/* extend last segment */

	uptag = pc->tag_parent;

	/*
	 * XXX There is sometimes recursive locking here.
	 * XXX We should try to find a better solution.
	 * XXX Until further the "owned" variable does
	 * XXX the trick.
	 */

	if (error) {
		goto done;
	}
	pg = pc->page_start;
	pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	rem = segs->ds_addr & (USB_PAGE_SIZE - 1);
	pc->page_offset_buf = rem;
	pc->page_offset_end += rem;
	if (nseg < ((pc->page_offset_end +
	    (USB_PAGE_SIZE - 1)) / USB_PAGE_SIZE)) {
		ext_seg = 1;
	} else {
		ext_seg = 0;
	}
	nseg--;

	while (nseg > 0) {
		nseg--;
		segs++;
		pg++;
		pg->physaddr = segs->ds_addr & ~(USB_PAGE_SIZE - 1);
	}

	/*
	 * XXX The segments we get from BUS-DMA are not aligned,
	 * XXX so we need to extend the last segment if we are
	 * XXX unaligned and cross the segment boundary!
	 */
	if (ext_seg && pc->ismultiseg) {
		(pg + 1)->physaddr = pg->physaddr + USB_PAGE_SIZE;
	}
done:
	mtx_lock(uptag->mtx);
	uptag->dma_error = (error ? 1 : 0);
	if (isload) {
		(uptag->func) (uptag);
	}
	mtx_unlock(uptag->mtx);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_alloc_mem - allocate DMA'able memory
 *
 * Returns:
 *    0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
uint8_t
usbd_pc_alloc_mem(struct usbd_page_cache *pc, struct usbd_page *pg,
    uint32_t size, uint32_t align)
{
	struct usbd_dma_parent_tag *uptag;
	struct usbd_dma_tag *utag;
	caddr_t ptr = NULL;
	bus_dmamap_t map;
	int seg_count;

	uptag = pc->tag_parent;

	if (align != 1) {
		/*
	         * The alignment must be greater or equal to the
	         * "size" else the object can be split between two
	         * memory pages and we get a problem!
	         */
		while (align < size) {
			align *= 2;
			if (align == 0) {
				goto done_5;
			}
		}
	}
	/* get the correct DMA tag */
	utag = usbd_dma_tag_find(pc->tag_parent, size, align);
	if (utag == NULL) {
		goto done_5;
	}
	if (bus_dmamem_alloc(utag->tag, size, align, 0, utag->p_seg,
	    utag->n_seg, &seg_count, BUS_DMA_WAITOK)) {
		goto done_4;
	}
	if (bus_dmamem_map(utag->tag, utag->p_seg, seg_count, size,
	   (void *)&ptr, BUS_DMA_WAITOK | BUS_DMA_COHERENT)) {
		goto done_3;
	}
	if (bus_dmamap_create(utag->tag, size, utag->n_seg, (align == 1) ?
	    USB_PAGE_SIZE : size, 0, BUS_DMA_WAITOK, &map)) {
		goto done_2;
	}
	if (bus_dmamap_load(utag->tag, map, ptr, size, NULL,
	    BUS_DMA_WAITOK)) {
		goto done_1;
	}
	pc->p_seg = malloc(seg_count * sizeof(*(pc->p_seg)),
	    M_DEVBUF, M_WAITOK | M_ZERO);
	if (pc->p_seg == NULL) {
		goto done_0;
	}
	/* store number if actual segments used */
	pc->n_seg = seg_count;

	/* make a copy of the segments */
	bcopy(utag->p_seg, pc->p_seg,
	    seg_count * sizeof(*(pc->p_seg)));

	/* setup page cache */
	pc->buffer = ptr;
	pc->page_start = pg;
	pc->page_offset_buf = 0;
	pc->page_offset_end = size;
	pc->map = map;
	pc->tag = utag->tag;
	pc->ismultiseg = (align == 1);

	usbd_pc_common_mem_cb(pc, utag->p_seg, seg_count, 0, 0);

	bzero(ptr, size);

	usbd_pc_cpu_flush(pc);

	return (0);

done_0:
	bus_dmamap_unload(utag->tag, map);
done_1:
	bus_dmamap_destroy(utag->tag, map);
done_2:
	bus_dmamem_unmap(utag->tag, ptr, size);
done_3:
	bus_dmamem_free(utag->tag, utag->p_seg, seg_count);
done_4:
	/* utag is destroyed later */
done_5:
	/* reset most of the page cache */
	pc->buffer = NULL;
	pc->page_start = NULL;
	pc->page_offset_buf = 0;
	pc->page_offset_end = 0;
	pc->map = NULL;
	pc->tag = NULL;
	pc->n_seg = 0;
	pc->p_seg = NULL;
	return (1);
}

/*------------------------------------------------------------------------*
 *	usbd_pc_free_mem - free DMA memory
 *
 * This function is NULL safe.
 *------------------------------------------------------------------------*/
void
usbd_pc_free_mem(struct usbd_page_cache *pc)
{
	if (pc && pc->buffer) {
		bus_dmamap_unload(pc->tag, pc->map);
		bus_dmamap_destroy(pc->tag, pc->map);
		bus_dmamem_unmap(pc->tag, pc->buffer,
		    pc->page_offset_end - pc->page_offset_buf);
		bus_dmamem_free(pc->tag, pc->p_seg, pc->n_seg);
		free(pc->p_seg, M_DEVBUF);
		pc->buffer = NULL;
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_cpu_invalidate - invalidate CPU cache
 *------------------------------------------------------------------------*/
void
usbd_pc_cpu_invalidate(struct usbd_page_cache *pc)
{
	uint32_t len;

	len = pc->page_offset_end - pc->page_offset_buf;

	bus_dmamap_sync(pc->tag, pc->map, 0, len,
	    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);
	return;
}

/*------------------------------------------------------------------------*
 *	usbd_pc_cpu_flush - flush CPU cache
 *------------------------------------------------------------------------*/
void
usbd_pc_cpu_flush(struct usbd_page_cache *pc)
{
	uint32_t len;

	len = pc->page_offset_end - pc->page_offset_buf;

	bus_dmamap_sync(pc->tag, pc->map, 0, len,
	    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	return;
}

/*------------------------------------------------------------------------*
 *  usbd_get_page - lookup DMA-able memory for the given offset
 *
 * NOTE: Only call this function when the "page_cache" structure has
 * been properly initialized !
 *------------------------------------------------------------------------*/
void
usbd_get_page(struct usbd_page_cache *pc, uint32_t offset,
    struct usbd_page_search *res)
{
	struct usbd_page *page;

	if (pc->page_start) {

		/* Case 1 - something has been loaded into DMA */

		if (pc->buffer) {

			/* Case 1a - Kernel Virtual Address */

			res->buffer = USBD_ADD_BYTES(pc->buffer, offset);
		}
		offset += pc->page_offset_buf;

		/* compute destination page */

		page = pc->page_start;

		if (pc->ismultiseg) {

			page += (offset / USB_PAGE_SIZE);

			offset %= USB_PAGE_SIZE;

			res->length = USB_PAGE_SIZE - offset;
			res->physaddr = page->physaddr + offset;
		} else {
			res->length = 0 - 1;
			res->physaddr = page->physaddr + offset;
		}
		if (!pc->buffer) {

			/* Case 1b - Non Kernel Virtual Address */

			res->buffer = USBD_ADD_BYTES(page->buffer, offset);
		}
	} else {

		/* Case 2 - Plain PIO */

		res->buffer = USBD_ADD_BYTES(pc->buffer, offset);
		res->length = 0 - 1;
		res->physaddr = 0;
	}
	return;
}

#endif

#endif
