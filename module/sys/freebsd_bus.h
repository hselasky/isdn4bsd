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
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/sys/bus.h"
 */

#ifndef __FREEBSD_SYS_BUS_H__
#define __FREEBSD_SYS_BUS_H__

struct __driver;
struct __devclass;
struct __device;
struct module;
struct bsd_module_data;

#define CALL_METHOD(dev, what, args...) \
  (((what##_t *)device_get_method(dev,#what))(args))

struct __device_method {
  const u_int8_t *desc;
  void * const func;
};

typedef struct __device_method device_method_t;

#define DEVMETHOD(what,func) { #what, func }

struct resource {
    u_int32_t  r_start;   /* first entry, inclusive */
    u_int32_t  r_end;     /* last entry, inclusive */
    bus_space_tag_t r_bustag;       /* bus_space tag */
    bus_space_handle_t r_bushandle; /* bus_space handle */
    bus_dma_tag_t r_dmatag;
    u_int32_t  r_rid;          /* optional rid for this resource. */
    u_int8_t   r_in_use : 1;
    u_int8_t   r_alloced : 1;
    u_int8_t   r_type;
    u_int8_t   r_intr_type;
};

#define DEVICE_MAXRES 64
#define DEVICE_IS_UNKNOWN 0
#define DEVICE_IS_PCI     1
#define DEVICE_IS_ISA     2
#define DEVICE_IS_PNP     3
#define DEVICE_IS_USB     4

struct __device {
    struct device dev_local; /* this structure must be first */
    struct resource dev_resources[DEVICE_MAXRES];

    u_int16_t dev_unit;
    void *    dev_sc;
    u_int32_t dev_id;
    u_int32_t dev_id_sub;
    u_int32_t dev_flags;
    u_int32_t dev_pci_class;
    int32_t   dev_order;

    u_int8_t  dev_what : 4;
    u_int8_t  dev_res_alloc : 1;
    u_int8_t  dev_quiet : 1;
    u_int8_t  dev_softc_set : 1;
    u_int8_t  dev_softc_alloc : 1;
    u_int8_t  dev_attached : 1;
    u_int8_t  dev_fixed_class : 1;
    u_int8_t  dev_unit_manual : 1;

    u_int8_t  dev_nameunit[64];
    u_int8_t  dev_desc[64];
    u_int16_t dev_res_alloc_count;
    void *    dev_aux;

    u_int8_t  dev_aux_data[1024];

    void    (*dev_intr_func)(void *);
    void *    dev_intr_arg;

bus_dma_tag_t dev_dma_tag;

const struct bsd_module_data *dev_module;

    TAILQ_HEAD(device_list, __device) dev_children;
    TAILQ_ENTRY(__device) dev_link;

    struct __device *dev_parent;
};

typedef struct __device * device_t;

#define DEVCLASS_MAXUNIT 64

struct __devclass {
    device_t dev_list[DEVCLASS_MAXUNIT];
};

typedef struct __devclass *devclass_t;

struct __driver {
    const u_int8_t *name;
    const struct __device_method *methods;
    u_int32_t size;
};

typedef struct __driver driver_t;

#define MOD_LOAD 1
#define MOD_UNLOAD 2

struct bsd_module_data {
    int (*callback)(struct module *, int, void *arg);
    void *arg;
    const u_int8_t *bus_name;
    const u_int8_t *mod_name;
    const u_int8_t *long_name;
    const struct __driver *driver;
    struct __devclass **devclass_pp;
};

#define DECLARE_MOD_DATA(name)				\
struct bsd_module_data name				\
  __attribute__((__section__("_bsd_module_data_start"),	\
		 __aligned__(1),__used__))

#define DRIVER_MODULE(name, busname, driver, devclass, evh, arg)        \
static const DECLARE_MOD_DATA(name##_##busname##_driver_mod) =		\
{ evh, arg, #busname, #name, #busname "/" #name, &driver, &devclass }

#define MODULE_DEPEND(args...)
#define MODULE_VERSION(args...)

typedef int  bus_child_location_str_t(device_t parent, device_t child, 
				      char *buf, size_t buflen);
typedef int  bus_child_pnpinfo_str_t(device_t parent, device_t child, 
				     char *buf, size_t buflen);
typedef void bus_driver_added_t(device_t dev, driver_t *driver);
typedef int  bus_print_child_t(device_t dev, device_t child);
typedef int  device_attach_t(device_t dev);
typedef int  device_detach_t(device_t dev);
typedef int  device_resume_t(device_t dev);
typedef int  device_shutdown_t(device_t dev);
typedef int  device_probe_t(device_t dev);
typedef int  device_suspend_t(device_t dev);
typedef void driver_intr_t(void *);

#define RF_ALLOCATED    0x0001  /* resource has been reserved */
#define RF_ACTIVE       0x0002  /* resource allocation has been activated */
#define RF_SHAREABLE    0x0004  /* resource permits contemporaneous sharing */
#define RF_TIMESHARE    0x0008  /* resource permits time-division sharing */
#define RF_WANTED       0x0010  /* somebody is waiting for this resource */
#define RF_FIRSTSHARE   0x0020  /* first in sharing list */
#define RF_PREFETCHABLE 0x0040  /* resource is prefetchable */

extern struct resource *
bus_alloc_resource(device_t dev, int type, int *rid, u_int32_t start, 
		   u_int32_t end, u_int32_t count, u_int32_t flags);

extern int
bus_release_resource(device_t dev, int type, int rid, 
		     struct resource *r);

extern int
bus_set_resource(device_t dev, int type, int rid, u_int32_t start, 
		 u_int32_t count);

extern int
bus_teardown_intr(device_t dev, struct resource *r, void *cookie);

extern int
bus_setup_intr(device_t dev, struct resource *r, int flags,
	       driver_intr_t *handler, void *arg, void **cookiep);

static __inline int
bus_generic_resume(device_t dev)
{
    return 0;
}

static __inline int
bus_generic_shutdown(device_t dev)
{
    return 0;
}

static __inline int
bus_generic_suspend(device_t dev)
{
    return 0;
}

static __inline int
bus_generic_print_child(device_t dev, device_t child)
{
    return 0;
}

static __inline void
bus_generic_driver_added(device_t dev, driver_t *driver)
{
    return;
}

static __inline struct resource *
bus_alloc_resource_any(device_t dev, int type, int *rid, u_int32_t flags)
{
    return (bus_alloc_resource(dev, type, rid, 0, 0-1, 1, flags));
}

static __inline u_int32_t
rman_get_start(struct resource *r)
{
    return (r->r_start);
}

#if 0
/*
 * There is no guarantee that memory
 * mapped I/O is directly reachable from 
 * the host memory, so this function is
 * not supported. Use "bus_space_xxx" to
 * access this memory!
 */
static __inline void *
rman_get_virtual(struct resource *r)
{
    return (r->r_virtual);
}
#endif

static __inline bus_space_tag_t
rman_get_bustag(struct resource *r)
{
    return (r->r_bustag);
}

static __inline bus_space_handle_t
rman_get_bushandle(struct resource *r)
{
    return (r->r_bushandle);
}

static __inline int
rman_get_rid(struct resource *r)
{
    return (r->r_rid);
}

static __inline u_int32_t
rman_get_size(struct resource *r)
{
    return (r->r_end - r->r_start + 1);
}

static __inline device_t
device_get_parent(device_t dev)
{
    return dev ? dev->dev_parent : NULL;
}

extern const char *
device_get_name(device_t dev);

extern const char *
device_get_nameunit(device_t dev);

#define device_printf __device_printf

extern int
__device_printf(device_t dev, const char *, ...) __printflike(2, 3);

extern device_t
device_add_child_ordered(device_t dev, int order, const char *name, int unit);

extern device_t 
device_add_child(device_t dev, const char *name, int unit);

extern void
device_quiet(device_t dev);

static __inline void
device_set_ivars(device_t dev, void *ivars)
{
    if(dev)
    {
        dev->dev_aux = ivars;
    }
    return;
}

static __inline void *
device_get_ivars(device_t dev)
{
    return dev ? dev->dev_aux : NULL;
}

static __inline u_int32_t
device_get_flags(device_t dev)
{
    return dev ? dev->dev_flags : 0;
}

static __inline void
device_set_flags(device_t dev, u_int32_t flags)
{
    if(dev)
      dev->dev_flags = flags;
    return;
}

extern const char *
device_get_desc(device_t dev);

extern int
device_probe_and_attach(device_t dev);

extern int
device_detach(device_t dev);

extern void *
device_get_softc(device_t dev);

extern void
device_set_softc(device_t dev, void *softc);

static __inline bus_dma_tag_t
device_get_dma_tag(device_t dev)
{
    return (dev) ? (dev->dev_dma_tag) : NULL;
}

extern int
device_delete_child(device_t dev, device_t child);

extern int
device_get_children(device_t dev, device_t **devlistp, int *devcountp);

extern int
device_is_attached(device_t dev);

extern void
device_set_desc(device_t dev, const char* desc);

extern void
device_set_desc_copy(device_t dev, const char* desc);

static __inline int
device_get_unit(device_t dev)
{
    return dev ? dev->dev_unit : 0;
}

extern void *
devclass_get_softc(devclass_t dc, int unit);

extern int 
devclass_get_maxunit(devclass_t dc);

extern device_t
devclass_get_device(devclass_t dc, int unit);

extern devclass_t 
devclass_find(const char *classname);

enum {
  INTR_TYPE_TTY = 1,
  INTR_TYPE_BIO = 2,
  INTR_TYPE_NET = 4,
  INTR_TYPE_CAM = 8,
  INTR_TYPE_MISC = 16,
  INTR_TYPE_CLK = 32,
  INTR_TYPE_AV = 64,
  INTR_FAST = 128,
  INTR_EXCL = 256,                /* exclusive interrupt */
  INTR_MPSAFE = 512,              /* this interrupt is SMP safe */
  INTR_ENTROPY = 1024             /* this interrupt provides entropy */
};

#define SYS_RES_IRQ     1       /* interrupt lines */
#define SYS_RES_DRQ     2       /* isa dma lines */
#define SYS_RES_MEMORY  3       /* i/o memory */
#define SYS_RES_IOPORT  4       /* i/o ports */

extern u_int32_t
pci_read_config(device_t dev, int reg, int width);

extern void
pci_write_config(device_t dev, int reg, u_int32_t val, int width);

extern int
pci_enable_busmaster(device_t dev);

extern int
pci_disable_busmaster(device_t dev);

extern int
pci_enable_io(device_t dev, int space);

extern int
pci_disable_io(device_t dev, int space);

static __inline u_int32_t
pci_get_devid(device_t dev)
{
    return dev->dev_id;
}

#include <sys/freebsd_pcireg.h>

#if 0
#define pci_get_cachelnsz(dev)   pci_read_config(dev, PCIR_CACHELNSZ, 1)
#define pci_get_class(dev)       pci_read_config(dev, PCIR_CLASS, 1)
#define pci_get_cmdreg(dev)      pci_read_config(dev, PCIR_COMMAND, 2)
#define pci_get_device(dev)      pci_read_config(dev, PCIR_DEVICE, 2)
#define pci_get_hdrtype(dev)     pci_read_config(dev, PCIR_HDRTYPE, 1)
#define pci_get_intpin(dev)      pci_read_config(dev, PCIR_INTPIN, 1)
#define pci_get_irq(dev)         pci_read_config(dev, PCIR_INTLINE, 1)
#define pci_get_lattimer(dev)    pci_read_config(dev, PCIR_LATTIMER, 1)
#define pci_get_maxlat(dev)      pci_read_config(dev, PCIR_MAXLAT, 1)
#define pci_get_mingnt(dev)      pci_read_config(dev, PCIR_MINGNT, 1)
#define pci_get_progif(dev)      pci_read_config(dev, PCIR_PROGIF, 1)
#define pci_get_revid(dev)       pci_read_config(dev, PCIR_REVID, 1)
#define pci_get_statreg(dev)     pci_read_config(dev, PCIR_STATUS, 2)
#define pci_get_subclass(dev)    pci_read_config(dev, PCIR_SUBCLASS, 1)
#define pci_get_subdevice(dev)   pci_read_config(dev, PCIR_SUBDEV_0, 2)
#define pci_get_subvendor(dev)   pci_read_config(dev, PCIR_SUBVEND_0, 2)
#define pci_get_vendor(dev)      pci_read_config(dev, PCIR_VENDOR, 2)
#else
extern u_int8_t pci_get_class(device_t dev);
extern u_int8_t pci_get_subclass(device_t dev);
extern u_int8_t pci_get_progif(device_t dev);
extern u_int16_t pci_get_vendor(device_t dev);
extern u_int16_t pci_get_device(device_t dev);
#endif

#define PCI_POWERSTATE_D0       0
#define PCI_POWERSTATE_D1       1
#define PCI_POWERSTATE_D2       2
#define PCI_POWERSTATE_D3       3
#define PCI_POWERSTATE_UNKNOWN  -1

static __inline int
pci_get_powerstate(device_t dev)
{
    device_printf(dev, "pci_get_powerstate() is not supported!\n");
    return PCI_POWERSTATE_D0;
}

static __inline int
pci_set_powerstate(device_t dev, int state)
{
    device_printf(dev, "pci_set_powerstate() is not supported!\n");
    return 0;
}

static __inline u_int32_t 
isa_get_logicalid(device_t dev)
{
    return dev->dev_id;
}

static __inline u_int32_t 
isa_get_vendorid(device_t dev)
{
    return dev->dev_id;
}

static __inline u_int32_t
isa_get_compatid(device_t dev)
{
    return dev->dev_id_sub;
}

#endif /* __FREEBSD_SYS_BUS_H__ */
