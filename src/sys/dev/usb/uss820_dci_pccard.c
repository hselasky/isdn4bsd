#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uss820_dci_pccard.c $");

/*-
 * Copyright (c) 2008 Hans Petter Selasky <hselasky@freebsd.org>
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
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/endian.h>
#include <sys/queue.h>
#include <sys/lock.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include <dev/usb/uss820_dci.h>

#include <dev/pccard/pccardreg.h>
#include <dev/pccard/pccardvar.h>

static device_probe_t uss820_pccard_probe;
static device_attach_t uss820_pccard_attach;
static device_detach_t uss820_pccard_detach;
static device_suspend_t uss820_pccard_suspend;
static device_resume_t uss820_pccard_resume;
static device_shutdown_t uss820_pccard_shutdown;

static uint8_t uss820_pccard_lookup(device_t dev);

static device_method_t uss820_dci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, uss820_pccard_probe),
	DEVMETHOD(device_attach, uss820_pccard_attach),
	DEVMETHOD(device_detach, uss820_pccard_detach),
	DEVMETHOD(device_suspend, uss820_pccard_suspend),
	DEVMETHOD(device_resume, uss820_pccard_resume),
	DEVMETHOD(device_shutdown, uss820_pccard_shutdown),

	/* Bus interface */
	DEVMETHOD(bus_print_child, bus_generic_print_child),

	{0, 0}
};

static driver_t uss820_dci_driver = {
	.name = "uss820",
	.methods = uss820_dci_methods,
	.size = sizeof(struct uss820_dci_softc),
};

static devclass_t uss820_dci_devclass;

DRIVER_MODULE(uss820, pccard, uss820_dci_driver, uss820_dci_devclass, 0, 0);
MODULE_DEPEND(uss820, usb, 1, 1, 1);

static const char *const uss820_desc = "USS820 USB Device Controller";

static int
uss820_pccard_suspend(device_t dev)
{
	struct uss820_dci_softc *sc = device_get_softc(dev);
	int err;

	err = bus_generic_suspend(dev);
	if (err == 0) {
		uss820_dci_suspend(sc);
	}
	return (err);
}

static int
uss820_pccard_resume(device_t dev)
{
	struct uss820_dci_softc *sc = device_get_softc(dev);
	int err;

	uss820_dci_resume(sc);

	err = bus_generic_resume(dev);

	return (err);
}

static int
uss820_pccard_shutdown(device_t dev)
{
	struct uss820_dci_softc *sc = device_get_softc(dev);
	int err;

	err = bus_generic_shutdown(dev);
	if (err)
		return (err);

	uss820_dci_uninit(sc);

	return (0);
}

static uint8_t
uss820_pccard_lookup(device_t dev)
{
	uint32_t prod;
	uint32_t vend;

	pccard_get_vendor(dev, &vend);
	pccard_get_product(dev, &prod);

	return (0);
}

static int
uss820_pccard_probe(device_t dev)
{
	if (uss820_pccard_lookup(dev)) {
		device_set_desc(dev, uss820_desc);
		return (0);
	}
	return (ENXIO);
}
static int
uss820_pccard_attach(device_t dev)
{
	struct uss820_dci_softc *sc = device_get_softc(dev);
	int err;
	int rid;

	if (sc == NULL) {
		return (ENXIO);
	}
	sc->sc_bus.dma_tag_parent = device_get_dma_tag(dev);

	/* get all DMA memory */

	if (usbd_bus_mem_alloc_all(&(sc->sc_bus), NULL)) {
		return (ENOMEM);
	}
	rid = 0;
	sc->sc_io_res =
	    bus_alloc_resource_any(dev, SYS_RES_IOPORT, &rid, RF_ACTIVE);

	if (!sc->sc_io_res) {
		goto error;
	}
	sc->sc_io_tag = rman_get_bustag(sc->sc_io_res);
	sc->sc_io_hdl = rman_get_bushandle(sc->sc_io_res);
	sc->sc_io_size = rman_get_size(sc->sc_io_res);

	/* multiply all addresses by 4 */
	sc->sc_reg_shift = 2;

	rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		goto error;
	}
	sc->sc_bus.bdev = device_add_child(dev, "usb", -1);
	if (!(sc->sc_bus.bdev)) {
		goto error;
	}
	device_set_ivars(sc->sc_bus.bdev, &(sc->sc_bus));
	device_set_softc(sc->sc_bus.bdev, &(sc->sc_bus));

	err = usbd_config_td_setup(&(sc->sc_config_td), sc,
	    &(sc->sc_bus.mtx), NULL, 0, 4);
	if (err) {
		device_printf(dev, "could not setup config thread!\n");
		goto error;
	}
#if (__FreeBSD_version >= 700031)
	err = bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL, (void *)uss820_dci_interrupt, sc, &(sc->sc_intr_hdl));
#else
	err = bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    (void *)uss820_dci_interrupt, sc, &(sc->sc_intr_hdl));
#endif
	if (err) {
		sc->sc_intr_hdl = NULL;
		goto error;
	}
	err = uss820_dci_init(sc);
	if (err) {
		device_printf(dev, "Init failed\n");
		goto error;
	}
	err = device_probe_and_attach(sc->sc_bus.bdev);
	if (err) {
		device_printf(dev, "USB probe and attach failed\n");
		goto error;
	}
	return (0);

error:
	uss820_pccard_detach(dev);
	return (ENXIO);
}

static int
uss820_pccard_detach(device_t dev)
{
	struct uss820_dci_softc *sc = device_get_softc(dev);
	device_t bdev;
	int err;

	if (sc->sc_bus.bdev) {
		bdev = sc->sc_bus.bdev;
		device_detach(bdev);
		device_delete_child(dev, bdev);
	}
	/* during module unload there are lots of children leftover */
	device_delete_all_children(dev);

	if (sc->sc_irq_res && sc->sc_intr_hdl) {
		/*
		 * only call at91_udp_uninit() after at91_udp_init()
		 */
		uss820_dci_uninit(sc);

		err = bus_teardown_intr(dev, sc->sc_irq_res,
		    sc->sc_intr_hdl);
		sc->sc_intr_hdl = NULL;
	}
	if (sc->sc_irq_res) {
		bus_release_resource(dev, SYS_RES_IRQ, 0,
		    sc->sc_irq_res);
		sc->sc_irq_res = NULL;
	}
	if (sc->sc_io_res) {
		bus_release_resource(dev, SYS_RES_IOPORT, 0,
		    sc->sc_io_res);
		sc->sc_io_res = NULL;
	}
	usbd_config_td_unsetup(&(sc->sc_config_td));

	usbd_bus_mem_free_all(&(sc->sc_bus), NULL);

	return (0);
}
