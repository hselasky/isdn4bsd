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

#undef DRIVER_MODULE
#undef MODULE_DEPEND
#undef MODULE_VERSION

#define	DRIVER_MODULE(name, busname, driver, devclass, evh, arg)        \
  extern const struct module_data bsd_##name##_##busname##_driver_mod;

#define	MODULE_DEPEND(name, busname, minver, prefver, maxver)		\
  extern const struct module_depend bsd_##name##_##busname##_depend;

#define	MODULE_VERSION(name, ver)					\
  extern const struct module_version bsd_##name##_##busname##_version;

#include <bsd_module_driver.h>
#include <bsd_module_depend.h>
#include <bsd_module_version.h>

#undef DRIVER_MODULE
#undef MODULE_DEPEND
#undef MODULE_VERSION

#define	DRIVER_MODULE(name, busname, driver, devclass, evh, arg) \
  &bsd_##name##_##busname##_driver_mod,

#define	MODULE_DEPEND(name, busname, minver, prefver, maxver)	\
  &bsd_##name##_##busname##_depend,

#define	MODULE_VERSION(name, ver) \
  &bsd_##name##_##busname##_version,

static const struct module_data *module_data[] = {
#include <bsd_module_driver.h>
};

#if 0
static const struct module_depend *module_depend[] = {
#include <bsd_module_depend.h>
};

#endif

#if 0
static const struct module_version *module_version[] = {
#include <bsd_module_version.h>
};

#endif

static const char unknown_string[] = {"unknown"};

int
bus_generic_resume(device_t dev)
{
	return (0);
}

int
bus_generic_shutdown(device_t dev)
{
	return (0);
}

int
bus_generic_suspend(device_t dev)
{
	return (0);
}

int
bus_generic_print_child(device_t dev, device_t child)
{
	return (0);
}

void
bus_generic_driver_added(device_t dev, driver_t *driver)
{
	return;
}

struct resource *
bus_alloc_resource_any(device_t dev, int type, int *rid, uint32_t flags)
{
	return (bus_alloc_resource(dev, type, rid, 0, 0 - 1, 1, flags));
}

uint32_t
rman_get_start(struct resource *r)
{
	return (r->r_start);
}

bus_space_tag_t
rman_get_bustag(struct resource *r)
{
	return (r->r_bustag);
}

bus_space_handle_t
rman_get_bushandle(struct resource *r)
{
	return (r->r_bushandle);
}

int
rman_get_rid(struct resource *r)
{
	return (r->r_rid);
}

uint32_t
rman_get_size(struct resource *r)
{
	return (r->r_end - r->r_start + 1);
}

device_t
device_get_parent(device_t dev)
{
	return (dev ? dev->dev_parent : NULL);
}

void
device_set_ivars(device_t dev, void *ivars)
{
	if (dev) {
		dev->dev_aux = ivars;
	}
	return;
}

void   *
device_get_ivars(device_t dev)
{
	return (dev ? dev->dev_aux : NULL);
}

uint32_t
device_get_flags(device_t dev)
{
	return (dev ? dev->dev_flags : 0);
}

void
device_set_flags(device_t dev, uint32_t flags)
{
	if (dev)
		dev->dev_flags = flags;
	return;
}

bus_dma_tag_t
bus_get_dma_tag(device_t dev)
{
	return ((dev) ? (dev->dev_dma_tag) : NULL);
}

int
device_get_unit(device_t dev)
{
	return (dev ? dev->dev_unit : 0);
}

static struct resource *
alloc_res(device_t dev)
{
	struct resource *res = dev->dev_resources;
	struct resource *res_end = dev->dev_resources + DEVICE_MAXRES;

	while (res != res_end) {
		if (res->r_in_use ||
		    res->r_alloced) {
			res++;
			continue;
		}
		bzero(res, sizeof(*res));
		res->r_in_use = 1;
		return (res);
	}
	return (NULL);
}

static struct resource *
find_res(device_t dev, uint8_t type, uint32_t rid)
{
	struct resource *res = dev->dev_resources;
	struct resource *res_end = dev->dev_resources + DEVICE_MAXRES;

	while (res != res_end) {
		if (res->r_in_use && (!res->r_alloced) &&
		    (res->r_rid == rid) &&
		    (res->r_type == type)) {
			return (res);
		}
		res++;
	}
	return (NULL);
}

static void
free_res(struct resource *res)
{
	res->r_in_use = 0;
	res->r_alloced = 0;
	return;
}

struct resource *
bus_alloc_resource(device_t dev, int type, int *rid, uint32_t start,
    uint32_t end, uint32_t count, uint32_t flags)
{
	struct resource *res = NULL;
	int err = 0;

	if (rid == NULL)
		goto error;

	if ((start == 0) && (end == (uint32_t)(-1))) {
		res = find_res(dev, type, rid[0]);
		if (res) {
			/* already allocated */
			goto error;
		}
		res = alloc_res(dev);
		if (res == NULL) {
			/* out of memory */
			goto error;
		}
#if 0
		DEVMETHOD(bus_alloc_resource, NULL);	/* dummy */
#endif
		err = BUS_ALLOC_RESOURCE(device_get_parent(dev), dev, res,
		    type, rid, start, end, count, flags);
		if (err) {
			/* some error happened */
			goto error;
		}
	} else {
		/* no support */
		goto error;
	}

	dev->dev_res_alloc_count++;
	res->r_in_use = 1;
	return (res);

error:
	if (res) {
		free_res(res);
	}
	return (NULL);
}

int
bus_release_resource(device_t dev, int type, int rid,
    struct resource *res)
{
	int error = ENXIO;

	if (dev && res) {
		if (res->r_alloced) {
#if 0
			DEVMETHOD(bus_free_resource, NULL);	/* dummy */
#endif
			BUS_FREE_RESOURCE(device_get_parent(dev), dev, res, rid);
		}
		free_res(res);
		error = 0;

		if (dev->dev_res_alloc_count) {
			dev->dev_res_alloc_count--;
		}
		if ((dev->dev_res_alloc_count == 0) &&
		    (dev->dev_res_alloc)) {
			dev->dev_res_alloc = 0;
		}
	}
	return (error);
}

int
bus_set_resource(device_t dev, int type, int rid, uint32_t start,
    uint32_t count)
{
	struct resource *res = dev->dev_resources;
	struct resource *res_end = dev->dev_resources + DEVICE_MAXRES;

	if (count == 0) {
		return (EINVAL);
	}
	while (res != res_end) {
		if (res->r_in_use ||
		    res->r_alloced) {
			if ((res->r_rid == rid) &&
			    (res->r_type == type)) {
				goto found;
			}
		}
		res++;
	}

	res = alloc_res(dev);

found:
	if (res == NULL) {
		return (ENOMEM);
	}
	res->r_rid = rid;
	res->r_type = type;
	res->r_start = start;
	res->r_end = start + count - 1;

	return (0);
}

int
bus_setup_intr(device_t dev, struct resource *res, int flags,
    driver_filter_t *filter, driver_intr_t *handler,
    void *priv, void **cookiep)
{
	int err;

	if (cookiep == NULL) {
		return (EINVAL);
	}
	cookiep[0] = NULL;

#if 0
	DEVMETHOD(bus_setup_interrupt, NULL);	/* dummy */
#endif
	err = BUS_SETUP_INTERRUPT(device_get_parent(dev), dev,
	    res, flags, filter, handler, priv, cookiep);

	return (err);
}

int
bus_teardown_intr(device_t dev, struct resource *r, void *cookie)
{
	int err;

	if (cookie == NULL) {
		return (0);
	}
#if 0
	DEVMETHOD(bus_teardown_interrupt, NULL);	/* dummy */
#endif
	err = BUS_TEARDOWN_INTERRUPT(device_get_parent(dev), dev,
	    r, cookie);

	return (err);
}

int
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
	if (dev && dev->dev_nameunit[0])
		return (dev->dev_nameunit);
	else
		return (unknown_string);
}

int
device_printf(device_t dev, const char *fmt,...)
{
	const char *indent;
	va_list ap;

	if (dev && dev->dev_nameunit[0])
		indent = dev->dev_nameunit;
	else
		indent = unknown_string;

	printf("%s: ", indent);

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);

	/*
	 * XXX should return number of characters printed
	 */
	return (0);
}

static uint8_t
devclass_create(devclass_t *dc_pp)
{
	if (dc_pp == NULL) {
		return (1);
	}
	if (dc_pp[0] == NULL) {
		dc_pp[0] = malloc(sizeof(**(dc_pp)),
		    M_DEVBUF, M_WAITOK | M_ZERO);

		if (dc_pp[0] == NULL) {
			return (1);
		}
	}
	return (0);
}

static const struct module_data *
devclass_find_create(const char *classname)
{
	const struct module_data *mod;
	uint16_t x;

	for (x = 0; x != (sizeof(module_data) / sizeof(module_data[0])); x++) {
		mod = module_data[x];
		if (mod->mod_name &&
		    (strcasecmp(classname, mod->mod_name) == 0)) {
			if (devclass_create(mod->devclass_pp)) {
				continue;
			}
			return (mod);
		}
	}
	return (NULL);
}

static uint8_t
devclass_add_device(const struct module_data *mod, device_t dev)
{
	uint16_t n;
	const char *name;

	if ((mod->devclass_pp == NULL) ||
	    (mod->devclass_pp[0] == NULL)) {
		return (1);
	}
	if (mod->driver &&
	    mod->driver->name) {
		name = mod->driver->name;
	} else {
		name = unknown_string;
	}

	if (dev->dev_unit_manual) {

		n = dev->dev_unit;

		if ((n < DEVCLASS_MAXUNIT) &&
		    (mod->devclass_pp[0]->dev_list[n] == NULL)) {
			goto found;
		}
	} else {

		for (n = 0; n != DEVCLASS_MAXUNIT; n++) {
			if (mod->devclass_pp[0]->dev_list[n] == NULL) {
				goto found;
			}
		}
	}

	return (1);

found:

	mod->devclass_pp[0]->dev_list[n] = dev;

	dev->dev_unit = n;
	dev->dev_module = mod;
	snprintf(dev->dev_nameunit,
	    sizeof(dev->dev_nameunit), "%s%d", name, n);

	return (0);
}

static void
devclass_delete_device(const struct module_data *mod, device_t dev)
{
	if (mod == NULL) {
		return;
	}
	if (devclass_get_device(mod->devclass_pp ?
	    mod->devclass_pp[0] : NULL, dev->dev_unit) == dev) {
		mod->devclass_pp[0]->dev_list[dev->dev_unit] = NULL;
	} else {
		device_printf(dev, "WARNING: device is not "
		    "present in devclass!\n");
	}
	dev->dev_module = NULL;
	return;
}

static device_t
make_device(device_t parent, const char *name, int unit)
{
	device_t dev = NULL;
	const struct module_data *mod = NULL;

	if (name) {

		mod = devclass_find_create(name);

		if (!mod) {

			printf("%s:%d:%s: can't find device "
			    "class %s\n", __FILE__, __LINE__,
			    __FUNCTION__, name);
			goto done;
		}
	}
	dev = malloc(sizeof(*dev),
	    M_DEVBUF, M_NOWAIT | M_ZERO);

	if (dev == NULL)
		goto done;

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
	return (dev);

error:
	if (dev) {
		free(dev, M_DEVBUF);
	}
	return (NULL);
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

	if (dev == NULL) {
		/* no parent */
		goto done;
	}
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
	return (child);
}

device_t
device_add_child(device_t dev, const char *name, int unit)
{
	return (device_add_child_ordered(dev, 0, name, unit));
}

int
device_delete_child(device_t dev, device_t child)
{
	int error = 0;
	device_t grandchild;

	if (dev != device_get_parent(child)) {
		device_printf(dev, "WARNING: Trying to delete "
		    "non-parenting child!\n");
	}
	/* remove children first */

	while ((grandchild = TAILQ_FIRST(&child->dev_children))) {
		error = device_delete_child(child, grandchild);
		if (error)
			goto done;
	}

	error = device_detach(child);

	if (error)
		goto done;

	devclass_delete_device(child->dev_module, child);

	if (dev != NULL) {
		/* remove child from parent */
		TAILQ_REMOVE(&dev->dev_children, child, dev_link);
	}
	free(child, M_DEVBUF);

done:
	return (error);
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
		*devlistp = NULL;
		*devcountp = 0;
		return (EINVAL);
	} else {
		n = count * sizeof(device_t);
	}

	list = malloc(n, M_TEMP, M_NOWAIT | M_ZERO);
	if (!list) {
		*devlistp = NULL;
		*devcountp = 0;
		return (ENOMEM);
	}
	n = 0;
	TAILQ_FOREACH(child, &dev->dev_children, dev_link) {
		if (n != count) {
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

	return (0);
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
	if (dev)
		return &(dev->dev_desc[0]);
	else
		return (unknown_string);
}

static void
default_method(void)
{
	/* do nothing */
	return;
}

void   *
device_get_method(device_t dev, const char *what)
{
	void *temp = NULL;
	const struct device_method *mtod;

	if (dev &&
	    dev->dev_module &&
	    dev->dev_module->driver &&
	    dev->dev_module->driver->methods) {
		mtod = dev->dev_module->driver->methods;
		while (mtod->desc && mtod->func) {
			if (strcasecmp(mtod->desc, what) == 0) {
				temp = mtod->func;
				break;
			}
			mtod++;
		}
	}
	if (temp == NULL) {
		/* maybe one should panic here */
		temp = &default_method;
		device_printf(dev, "%s: WARNING: missing method: %s!\n",
		    __FUNCTION__, what);
	}
	return (temp);
}

const char *
device_get_name(device_t dev)
{
	const struct module_data *mod = dev ? dev->dev_module : NULL;

	if (mod &&
	    mod->driver &&
	    mod->driver->name)
		return (mod->driver->name);
	else
		return (unknown_string);
}

int
device_probe_and_attach(device_t dev)
{
	const struct module_data *mod;
	const char *bus_name_parent;
	uint16_t x;

	bus_name_parent = device_get_name(device_get_parent(dev));

	if (dev->dev_attached) {
		return (0);
	}
	if (dev->dev_fixed_class) {

		mod = dev->dev_module;
#if 0
		DEVMETHOD(device_probe, NULL);	/* dummy */
		DEVMETHOD(device_attach, NULL);	/* dummy */
#endif
		if (DEVICE_PROBE(dev) <= 0) {
			if (DEVICE_ATTACH(dev) == 0) {
				/* success */
				dev->dev_attached = 1;
				return (0);
			}
		}
		device_detach(dev);

		goto error;
	}
	/*
         * else find a module for our device, if any
         */
	for (x = 0; x != (sizeof(module_data) / sizeof(module_data[0])); x++) {
		mod = module_data[x];
		if (mod->bus_name &&
		    mod->driver &&
		    mod->driver->methods &&
		    mod->driver->name &&
		    (strcasecmp(mod->bus_name, bus_name_parent) == 0)) {
			if (devclass_create(mod->devclass_pp)) {
				device_printf(dev, "devclass_create_() "
				    "failed!\n");
				continue;
			}
			if (devclass_add_device(mod, dev)) {
				device_printf(dev, "devclass_add_device() "
				    "failed!\n");
				continue;
			}
			if (DEVICE_PROBE(dev) <= 0) {
				if (DEVICE_ATTACH(dev) == 0) {
					/* success */
					dev->dev_attached = 1;
					return (0);
				}
			}
			/* else try next driver */

			device_detach(dev);
		}
	}

error:
	return (ENODEV);
}

int
device_detach(device_t dev)
{
	const struct module_data *mod = dev->dev_module;
	int error;

#if 0
	if (mod == NULL) {
		device_printf(dev, "invalid device: "
		    "dev->dev_module == NULL!\n");
		return (0);
	}
#endif
	if (dev->dev_attached) {
#if 0
		DEVMETHOD(device_detach, NULL);	/* dummy */
#endif
		error = DEVICE_DETACH(dev);
		if (error) {
			return error;
		}
		dev->dev_attached = 0;
	}
	device_set_softc(dev, NULL);
	dev->dev_softc_set = 0;

	if (dev->dev_fixed_class == 0) {
		devclass_delete_device(mod, dev);
	}
	return (0);
}

void
device_set_softc(device_t dev, void *softc)
{
	if (dev->dev_softc_alloc) {
		free(dev->dev_sc, M_DEVBUF);
		dev->dev_sc = NULL;
	}
	dev->dev_sc = softc;
	dev->dev_softc_alloc = 0;
	dev->dev_softc_set = 1;
	return;
}

void   *
device_get_softc(device_t dev)
{
	const struct module_data *mod;

	if (dev == NULL) {
		return (NULL);
	}
	mod = dev->dev_module;

	if ((!dev->dev_softc_set) &&
	    (!dev->dev_softc_alloc) &&
	    mod &&
	    mod->driver &&
	    mod->driver->size) {
		dev->dev_sc = malloc(mod->driver->size,
		    M_DEVBUF, M_WAITOK | M_ZERO);

		if (dev->dev_sc) {
			dev->dev_softc_set = 1;
			dev->dev_softc_alloc = 1;
		}
	}
	return (dev->dev_sc);
}

int
device_is_attached(device_t dev)
{
	return (dev->dev_attached);
}

void
device_set_desc(device_t dev, const char *desc)
{
	snprintf(dev->dev_desc, sizeof(dev->dev_desc), "%s", desc);
	return;
}

void
device_set_desc_copy(device_t dev, const char *desc)
{
	device_set_desc(dev, desc);
	return;
}

void   *
devclass_get_softc(devclass_t dc, int unit)
{
	return (device_get_softc(devclass_get_device(dc, unit)));
}

int
devclass_get_maxunit(devclass_t dc)
{
	int max_unit = 0;

	if (dc) {
		max_unit = DEVCLASS_MAXUNIT;
		while (max_unit--) {
			if (dc->dev_list[max_unit]) {
				break;
			}
		}
		max_unit++;
	}
	return (max_unit);
}

device_t
devclass_get_device(devclass_t dc, int unit)
{
	return (((unit < 0) || (unit >= DEVCLASS_MAXUNIT) || (dc == NULL)) ?
	    NULL : dc->dev_list[unit]);
}

devclass_t
devclass_find(const char *classname)
{
	const struct module_data *mod;
	uint16_t x;

	for (x = 0; x != (sizeof(module_data) / sizeof(module_data[0])); x++) {
		mod = module_data[x];
		if (mod->devclass_pp &&
		    mod->devclass_pp[0] &&
		    mod->mod_name &&
		    (strcasecmp(classname, mod->mod_name) == 0)) {
			return (mod->devclass_pp[0]);
		}
	}
	return (NULL);
}

static void
device_kld_event(int what)
{
	const struct module_data *mod;
	uint16_t x;
	uint16_t y;
	device_t dev;

	for (x = 0; x != (sizeof(module_data) / sizeof(module_data[0])); x++) {
		mod = module_data[x];

		if ((what == MOD_UNLOAD) &&
		    (mod->devclass_pp != NULL)) {

			/* detach any active devices */

			for (y = 0; y != DEVCLASS_MAXUNIT; y++) {

				dev = devclass_get_device(mod->devclass_pp[0], y);

				if (dev) {
					mtx_lock(&Giant);
					if (device_is_attached(dev)) {
						if (device_detach(dev)) {
							/* ignore any errors */
						}
					}
					mtx_unlock(&Giant);
					dev->dev_module = NULL;
				}
			}
		}
		if (mod->callback) {
			if ((mod->callback) (NULL, what, mod->arg)) {
				/* ignore any errors */
				printf("WARNING: Module %s did not "
				    "accept event %d\n", mod->mod_name, what);
			}
		}
	}
	return;
}

void
devctl_queue_data(char *data)
{
	/* we just free the data */
	free(data, M_BUS);
	return;
}

static void
device_kld_load(void *arg)
{
	device_kld_event(MOD_LOAD);
	return;
}

SYSINIT(module_load, SI_SUB_KLD, SI_ORDER_FIRST, &device_kld_load, NULL);

static void
device_kld_unload(void *arg)
{
	device_kld_event(MOD_UNLOAD);
	return;
}

SYSUNINIT(module_unload, SI_SUB_KLD, SI_ORDER_FIRST, &device_kld_unload, NULL);
