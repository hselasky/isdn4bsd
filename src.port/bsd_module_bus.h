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

struct module_version;
struct module_depend;
struct driver;
struct devclass;
struct device;
struct module;
struct module_data;
struct bus_dma_tag_t;
struct resource;

typedef struct driver driver_t;
typedef struct devclass *devclass_t;
typedef struct device *device_t;
typedef struct bus_dma_tag_t *bus_dma_tag_t;
typedef void driver_intr_t (void *);

typedef int bus_alloc_resource_t (device_t parent, device_t child, struct resource *res, int type, int *rid, uint32_t start, uint32_t end, uint32_t count, uint32_t flags);
typedef void bus_free_resource_t (device_t parent, device_t child, struct resource *res, int rid);
typedef int bus_setup_interrupt_t (device_t parent, device_t child, struct resource *r, int flags, driver_intr_t *handler, void *arg, void **cookiep);
typedef int bus_teardown_interrupt_t (device_t parent, device_t child, struct resource *r, void *cookie);
typedef int bus_child_location_str_t (device_t parent, device_t child, char *buf, size_t buflen);
typedef int bus_child_pnpinfo_str_t (device_t parent, device_t child, char *buf, size_t buflen);
typedef void bus_driver_added_t (device_t dev, driver_t *driver);
typedef int bus_print_child_t (device_t dev, device_t child);
typedef int device_attach_t (device_t dev);
typedef int device_detach_t (device_t dev);
typedef int device_resume_t (device_t dev);
typedef int device_shutdown_t (device_t dev);
typedef int device_probe_t (device_t dev);
typedef int device_suspend_t (device_t dev);

struct device_method {
	const char *desc;
	void   *const func;
};

typedef struct device_method device_method_t;

#define	DEVMETHOD(what,func) { #what, (void *)&func }

struct resource {
	uint32_t r_start;		/* first entry, inclusive */
	uint32_t r_end;			/* last entry, inclusive */
	bus_space_tag_t r_bustag;	/* bus_space tag */
	bus_space_handle_t r_bushandle;	/* bus_space handle */
	bus_dma_tag_t r_dmatag;
	uint32_t r_rid;			/* optional rid for this resource. */
	uint8_t	r_in_use:1;
	uint8_t	r_alloced:1;
	uint8_t	r_type;
	uint8_t	r_intr_type;
};

#define	DEVICE_MAXRES 64

struct device {
	struct resource dev_resources[DEVICE_MAXRES];
	TAILQ_HEAD(device_list, device) dev_children;
	TAILQ_ENTRY(device) dev_link;

	struct device *dev_parent;
	bus_dma_tag_t dev_dma_tag;
	const struct module_data *dev_module;
	void   *dev_aux_data;
	void   *dev_sc;
	void   *dev_aux;
	void    (*dev_intr_func) (void *);
	void   *dev_intr_arg;

	uint32_t dev_id;
	uint32_t dev_id_sub;
	uint32_t dev_flags;
	uint32_t dev_pci_class;
	int	dev_order;

	uint16_t dev_res_alloc_count;
	uint16_t dev_unit;

	uint8_t	dev_res_alloc:1;
	uint8_t	dev_quiet:1;
	uint8_t	dev_softc_set:1;
	uint8_t	dev_softc_alloc:1;
	uint8_t	dev_attached:1;
	uint8_t	dev_fixed_class:1;
	uint8_t	dev_unit_manual:1;

	char	dev_nameunit[64];
	char	dev_desc[64];
};

#define	DEVCLASS_MAXUNIT 64

struct devclass {
	device_t dev_list[DEVCLASS_MAXUNIT];
};

struct driver {
	const char *name;
	const struct device_method *methods;
	uint32_t size;
};

#define	MOD_LOAD 1
#define	MOD_UNLOAD 2

struct module_data {
	int     (*callback) (struct module *, int, void *arg);
	void   *arg;
	const char *bus_name;
	const char *mod_name;
	const char *long_name;
	const struct driver *driver;
	struct devclass **devclass_pp;
};

struct module_depend {
	const char *bus_name;
	const char *mod_name;
	uint8_t	minver;
	uint8_t	prefver;
	uint8_t	maxver;
};

struct module_version {
	const char *mod_name;
	uint8_t	ver;
};

#define	DRIVER_MODULE(name, busname, driver, devclass, evh, arg)        \
  const struct module_data bsd_##name##_##busname##_driver_mod =	{	\
	evh, arg, #busname, #name, #busname "/" #name,			\
	&driver, &devclass }

#define	MODULE_DEPEND(name, busname, minver, prefver, maxver)		\
  const struct module_depend bsd_##name##_##busname##_depend = {	\
	#busname, #name, minver, prefver, maxver }

#define	MODULE_VERSION(name, ver)					\
  const struct module_version bsd_##name##_##busname##_version = {	\
	#name, ver }

#define	RF_ALLOCATED    0x0001		/* resource has been reserved */
#define	RF_ACTIVE       0x0002		/* resource allocation has been
					 * activated */
#define	RF_SHAREABLE    0x0004		/* resource permits contemporaneous
					 * sharing */
#define	RF_TIMESHARE    0x0008		/* resource permits time-division
					 * sharing */
#define	RF_WANTED       0x0010		/* somebody is waiting for this
					 * resource */
#define	RF_FIRSTSHARE   0x0020		/* first in sharing list */
#define	RF_PREFETCHABLE 0x0040		/* resource is prefetchable */

struct resource *bus_alloc_resource(device_t dev, int type, int *rid, uint32_t start, uint32_t end, uint32_t count, uint32_t flags);
int	bus_release_resource(device_t dev, int type, int rid, struct resource *r);
int	bus_set_resource(device_t dev, int type, int rid, uint32_t start, uint32_t count);
int	bus_teardown_intr(device_t dev, struct resource *r, void *cookie);
int	bus_generic_detach(device_t dev);
int	bus_setup_intr(device_t dev, struct resource *r, int flags, driver_intr_t *handler, void *arg, void **cookiep);
int	bus_generic_resume(device_t dev);
int	bus_generic_shutdown(device_t dev);
int	bus_generic_suspend(device_t dev);
int	bus_generic_print_child(device_t dev, device_t child);
void	bus_generic_driver_added(device_t dev, driver_t *driver);
struct resource *bus_alloc_resource_any(device_t dev, int type, int *rid, uint32_t flags);
uint32_t rman_get_start(struct resource *r);
bus_space_tag_t rman_get_bustag(struct resource *r);
bus_space_handle_t rman_get_bushandle(struct resource *r);
int	rman_get_rid(struct resource *r);
uint32_t rman_get_size(struct resource *r);
device_t device_get_parent(device_t dev);
void   *device_get_method(device_t dev, const char *what);
const char *device_get_name(device_t dev);
const char *device_get_nameunit(device_t dev);
int	device_printf(device_t dev, const char *,...)__printflike(2, 3);
device_t device_add_child_ordered(device_t dev, int order, const char *name, int unit);
device_t device_add_child(device_t dev, const char *name, int unit);
void	device_quiet(device_t dev);
void	device_set_ivars(device_t dev, void *ivars);
void   *device_get_ivars(device_t dev);
uint32_t device_get_flags(device_t dev);
void	device_set_flags(device_t dev, uint32_t flags);
const char *device_get_desc(device_t dev);
int	device_probe_and_attach(device_t dev);
int	device_detach(device_t dev);
void   *device_get_softc(device_t dev);
void	device_set_softc(device_t dev, void *softc);
bus_dma_tag_t device_get_dma_tag(device_t dev);
int	device_delete_child(device_t dev, device_t child);
int	device_get_children(device_t dev, device_t **devlistp, int *devcountp);
int	device_is_attached(device_t dev);
void	device_set_desc(device_t dev, const char *desc);
void	device_set_desc_copy(device_t dev, const char *desc);
int	device_get_unit(device_t dev);
void   *devclass_get_softc(devclass_t dc, int unit);
int	devclass_get_maxunit(devclass_t dc);
device_t devclass_get_device(devclass_t dc, int unit);
devclass_t devclass_find(const char *classname);

enum {
	INTR_TYPE_TTY = 1,
	INTR_TYPE_BIO = 2,
	INTR_TYPE_NET = 4,
	INTR_TYPE_CAM = 8,
	INTR_TYPE_MISC = 16,
	INTR_TYPE_CLK = 32,
	INTR_TYPE_AV = 64,
	INTR_FAST = 128,
	INTR_EXCL = 256,		/* exclusive interrupt */
	INTR_MPSAFE = 512,		/* this interrupt is SMP safe */
	INTR_ENTROPY = 1024		/* this interrupt provides entropy */
};

enum {
	SYS_RES_IRQ = 1,		/* interrupt lines */
	SYS_RES_DRQ = 2,		/* isa dma lines */
	SYS_RES_MEMORY = 3,		/* I/O memory */
	SYS_RES_IOPORT = 4		/* I/O ports */
};
