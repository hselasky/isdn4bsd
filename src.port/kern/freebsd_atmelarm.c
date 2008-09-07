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

static bus_alloc_resource_t atmelarm_alloc_resource;
static bus_free_resource_t atmelarm_free_resource;
static bus_setup_interrupt_t atmelarm_setup_interrupt;
static bus_teardown_interrupt_t atmelarm_teardown_interrupt;
static int atmelarm_mod_load(struct module *, int, void *arg);

static device_method_t atmelarm_methods[] = {
	/* Device interface */
	DEVMETHOD(bus_alloc_resource, atmelarm_alloc_resource),
	DEVMETHOD(bus_free_resource, atmelarm_free_resource),
	DEVMETHOD(bus_setup_interrupt, atmelarm_setup_interrupt),
	DEVMETHOD(bus_teardown_interrupt, atmelarm_teardown_interrupt),
	DEVMETHOD(bus_print_child, bus_generic_print_child),

	{0, 0}
};

static driver_t atmelarm_driver = {
	"atmelarm",
	atmelarm_methods,
	0,
};

static devclass_t atmelarm_devclass;

DRIVER_MODULE(atmelarm, atmelarm, atmelarm_driver, atmelarm_devclass, atmelarm_mod_load, 0);

extern volatile uint8_t *UsbIoBase;

#ifdef MUSB2_DMA_ENABLED
extern uint32_t musbotg_get_dma_chan(int unit, int rid);

#endif

static int
atmelarm_alloc_resource(device_t parent, device_t child,
    struct resource *res, int type, int *rid, uint32_t start,
    uint32_t end, uint32_t count, uint32_t flags)
{
	switch (type) {
#ifdef MUSB2_DMA_ENABLED
		case SYS_RES_DRQ:
		start = musbotg_get_dma_chan(device_get_unit(child), rid[0]);
		if (start != 0) {
			res->r_start = start;
			res->r_end = start;
			res->r_bustag = NULL;
			res->r_bushandle = NULL;
			return (0);
		}
		break;
#endif
	case SYS_RES_MEMORY:
		if (rid[0] == 0) {
			res->r_start = UsbIoBase - (uint8_t *)0;
			res->r_end = (UsbIoBase - (uint8_t *)0) + 0x400;
			res->r_bustag = NULL;
			res->r_bushandle = (bus_space_handle_t)UsbIoBase;
			return (0);
		}
		break;
	case SYS_RES_IRQ:
		if (rid[0] == 0) {
			return (0);
		}
		break;
	default:
		break;
	}
	return (EINVAL);
}

static void
atmelarm_free_resource(device_t parent, device_t child,
    struct resource *res, int rid)
{
	return;
}

extern int (*UsbInterruptFilterPtr) (void *);
extern void (*UsbInterruptHandlerPtr) (void *);
extern void *UsbInterruptHandlerArg;

static int
atmelarm_setup_interrupt(device_t parent, device_t child,
    struct resource *r, int flags, driver_filter_t *filter,
    driver_intr_t *handler, void *arg, void **cookiep)
{
	if (UsbInterruptFilterPtr)
		return (EBUSY);

	if (UsbInterruptHandlerPtr)
		return (EBUSY);

	UsbInterruptHandlerArg = arg;
	UsbInterruptHandlerPtr = handler;
	UsbInterruptFilterPtr = filter;

	*cookiep = handler;

	return (0);
}

static int
atmelarm_teardown_interrupt(device_t parent, device_t child,
    struct resource *r, void *cookie)
{
	UsbInterruptHandlerPtr = NULL;
	UsbInterruptFilterPtr = NULL;
	return (0);
}

static device_t atmelarm_root;
static device_t atmelarm_usb;

static int
atmelarm_mod_load(struct module *mod, int event, void *arg)
{
	switch (event) {
		case MOD_LOAD:
		atmelarm_root = device_add_child(NULL, "atmelarm", -1);
		if (atmelarm_root) {
			atmelarm_usb = device_add_child(atmelarm_root, "musbotg", -1);
			if (atmelarm_usb) {
				if (device_probe_and_attach(atmelarm_usb)) {
					/* ignore */
					device_printf(atmelarm_usb,
					    "WARNING: Probe and attach failed!\n");
				}
			}
		}
		break;

	case MOD_UNLOAD:
		if (atmelarm_usb) {
			if (device_detach(atmelarm_usb)) {
				/* ignore */
			}
			if (device_delete_child(atmelarm_root, atmelarm_usb)) {
				/* ignore */
			}
		}
		if (atmelarm_root) {
			if (device_delete_child(NULL, atmelarm_root)) {
				/* ignore */
			}
		}
		break;

	default:
		break;
	}
	return (0);
}
