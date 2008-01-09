#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/usb.c,v 1.121 2007/10/20 23:23:18 julian Exp $");

/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * USB specifications and other documentation can be found at
 * http://www.usb.org/developers/docs/ and
 * http://www.usb.org/developers/devclass_docs/
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>

#include <sys/proc.h>
#include <sys/unistd.h>
#include <sys/kthread.h>
#include <sys/poll.h>
#include <sys/signalvar.h>
#include <sys/vnode.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

MALLOC_DEFINE(M_USB, "USB", "USB");
MALLOC_DEFINE(M_USBDEV, "USBdev", "USB device");
MALLOC_DEFINE(M_USBHC, "USBHC", "USB host controller");

/* define this unconditionally in case a kernel module is loaded that
 * has been compiled with debugging options.
 */
SYSCTL_NODE(_hw, OID_AUTO, usb, CTLFLAG_RW, 0, "USB debugging");

#ifdef USB_DEBUG
int	usbdebug = 0;

SYSCTL_INT(_hw_usb, OID_AUTO, debug, CTLFLAG_RW,
    &usbdebug, 0, "usb debug level");
#endif

#if ((__FreeBSD_version >= 700001) || (__FreeBSD_version == 0) || \
     ((__FreeBSD_version >= 600034) && (__FreeBSD_version < 700000)))
#define	USB_UCRED struct ucred *ucred,
#else
#define	USB_UCRED
#endif

static uint8_t usb_post_init_called = 0;

static device_probe_t usb_probe;
static device_attach_t usb_attach;
static device_detach_t usb_detach;

static int usb_dummy_open(struct cdev *dev, int oflags, int devtype, struct thread *td);
static void usb_event_thread(struct usbd_bus *bus);
static void usb_create_event_thread(struct usbd_bus *bus);
static void usb_attach_sub(device_t dev, struct usbd_bus *bus);
static void usb_post_init(void *arg);
static struct usbd_clone *usb_clone_sub(struct usbd_bus *bus);
static void usb_clone_remove(struct usbd_bus *bus);
static void usb_clone(void *arg, USB_UCRED char *name, int namelen, struct cdev **dev);
static int usb_ioctl(struct usb_cdev *dev, u_long cmd, caddr_t addr, int32_t fflags, struct thread *td);
static void usb_init(void *arg);
static void usb_uninit(void *arg);

static devclass_t usb_devclass;
static driver_t usb_driver =
{
	.name = "usb",
	.methods = (device_method_t[]){
		DEVMETHOD(device_probe, usb_probe),
		DEVMETHOD(device_attach, usb_attach),
		DEVMETHOD(device_detach, usb_detach),
		DEVMETHOD(device_suspend, bus_generic_suspend),
		DEVMETHOD(device_resume, bus_generic_resume),
		DEVMETHOD(device_shutdown, bus_generic_shutdown),
		{0, 0}
	},
	.size = 0,			/* the softc must be set by the
					 * attacher! */
};

DRIVER_MODULE(usb, ohci, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, uhci, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, ehci, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, at91_udp, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, uss820, usb_driver, usb_devclass, 0, 0);

MODULE_DEPEND(usb, usb, 1, 1, 1);
MODULE_VERSION(usb, 1);

static cdevsw_t usb_dummy_cdevsw = {
	.d_version = D_VERSION,
	.d_open = usb_dummy_open,
	.d_name = "usb_dummy_cdev",
};

static int
usb_dummy_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	return (ENODEV);
}

/*------------------------------------------------------------------------*
 *	usb_event_thread - explore the device tree from the root
 *------------------------------------------------------------------------*/
static void
usb_event_thread(struct usbd_bus *bus)
{
	int error;

	PRINTFN(2, ("\n"));

	while (1) {

		mtx_lock(&(bus->mtx));
retry:
		if (bus->needs.teardown) {
			wakeup(&(bus->bdev));
			mtx_unlock(&(bus->mtx));
			break;
		}
		if ((bus->devices[USB_ROOT_HUB_ADDR] == NULL) ||
		    (bus->devices[USB_ROOT_HUB_ADDR]->hub == NULL) ||
		    (bus->needs.explore == 0)) {

			if (bus->needs.sync) {
				bus->needs.sync = 0;
				wakeup(&(bus->needs));
			}
			bus->needs.wakeup = 1;

			error = mtx_sleep(bus, &(bus->mtx),
			    0, "usbevt", hz * 240);

			PRINTFN(2, ("woken up\n"));

			bus->needs.wakeup = 0;
			goto retry;
		}
		bus->needs.explore = 0;

		if (bus->needs.probe_attach) {
			bus->needs.probe_attach = 0;
			bus->driver_added_refcount++;
		}
		if (bus->driver_added_refcount == 0) {
			/* avoid zero, hence that is memory default */
			bus->driver_added_refcount = 1;
		}
		mtx_unlock(&(bus->mtx));

		mtx_lock(&usb_global_lock);

		/*
		 * Explore the Root USB HUB (this call can sleep,
		 * exiting usb_global_lock, which is actually Giant)
		 */
		(bus->devices[USB_ROOT_HUB_ADDR]->hub->explore)
		    (bus->devices[USB_ROOT_HUB_ADDR]);

		mtx_unlock(&usb_global_lock);
	}

	PRINTF(("exit\n"));

	usb_thread_exit(0);

	return;
}

/*------------------------------------------------------------------------*
 *	usb_needs_explore
 *
 * This functions is called when the USB event thread
 * needs to be explored.
 *------------------------------------------------------------------------*/
void
usb_needs_explore(struct usbd_bus *bus, uint8_t what)
{
	int err;

	PRINTFN(2, ("\n"));

	mtx_lock(&(bus->mtx));
	if (bus->needs.wakeup) {
		bus->needs.wakeup = 0;
		wakeup(bus);
	}
	switch (what) {
	case USB_BUS_EXPLORE_STOP:
		bus->needs.teardown = 1;
		err = mtx_sleep(&(bus->bdev), &(bus->mtx),
		    0, "usbdrain", 0);
		break;

	case USB_BUS_EXPLORE_PROBE:
		bus->needs.explore = 1;
		bus->needs.probe_attach = 1;
		break;

	case USB_BUS_EXPLORE_SYNC:
		bus->needs.sync = 1;
		err = mtx_sleep(&(bus->needs), &(bus->mtx),
		    0, "usbsync", 30 * hz);
		break;

	default:
		/* just explore */
		bus->needs.explore = 1;
		break;
	}
	mtx_unlock(&(bus->mtx));
	return;
}

/*------------------------------------------------------------------------*
 *	usb_needs_probe_and_attach
 *
 * This function is called whenever a new driver is loaded and will
 * cause that all USB busses are re-explored.
 *------------------------------------------------------------------------*/
void
usb_needs_probe_and_attach(void)
{
	struct usbd_bus *bus;
	devclass_t dc;
	device_t dev;
	uint8_t what;
	int max;

	PRINTFN(2, ("\n"));

	dc = devclass_find("usb");
	if (dc == NULL) {
		return;
	}
	/*
	 * Explore all USB busses in parallell.
	 */

	what = USB_BUS_EXPLORE_PROBE;
repeat:
	max = devclass_get_maxunit(dc);
	while (max >= 0) {
		dev = devclass_get_device(dc, max);
		if (dev) {
			bus = device_get_softc(dev);
			usb_needs_explore(bus, what);
		}
		max--;
	}

	if (what == USB_BUS_EXPLORE_PROBE) {
		what = USB_BUS_EXPLORE_SYNC;
		goto repeat;
	}
	return;
}

static void
usb_create_event_thread(struct usbd_bus *bus)
{
	if (usb_thread_create((void *)&usb_event_thread, bus,
	    &bus->event_thread, "%s", device_get_nameunit(bus->bdev))) {
		device_printf(bus->bdev, "unable to create event thread for\n");
		panic("usb_create_event_thread");
	}
	return;
}

/* called from "{ehci,ohci,uhci}_pci_attach()" */

static int
usb_probe(device_t dev)
{
	PRINTF(("\n"));
	return (0);
}

static void
usb_attach_sub(device_t dev, struct usbd_bus *bus)
{
	dev_clone_fn usb_clone_ptr = &usb_clone;
	struct usbd_device *child;
	usbd_status_t err;
	uint8_t speed;

	PRINTF(("\n"));

	mtx_assert(&usb_global_lock, MA_OWNED);

	switch (bus->usbrev) {
	case USBREV_1_0:
		speed = USB_SPEED_FULL;
		device_printf(bus->bdev, "12Mbps Full Speed USB v1.0\n");
		break;

	case USBREV_1_1:
		speed = USB_SPEED_FULL;
		device_printf(bus->bdev, "12Mbps Full Speed USB v1.1\n");
		break;

	case USBREV_2_0:
		speed = USB_SPEED_HIGH;
		device_printf(bus->bdev, "480Mbps High Speed USB v2.0\n");
		break;

	case USBREV_2_5:
		speed = USB_SPEED_VARIABLE;
		device_printf(bus->bdev, "480Mbps Wireless USB v2.5\n");
		break;

	default:
		device_printf(bus->bdev, "Unsupported USB revision!\n");
		return;
	}

	/* Allocate the Root USB device */

	child = usbd_alloc_device(bus->bdev, bus, NULL, 0, 0, 1,
	    speed, USB_MODE_HOST);
	if (child) {
		err = usbd_probe_and_attach(child,
		    USB_IFACE_INDEX_ANY);
		if (!err) {
			if (!bus->devices[USB_ROOT_HUB_ADDR]->hub) {
				err = USBD_ERR_NO_ROOT_HUB;
			}
		}
	} else {
		err = USBD_ERR_NOMEM;
	}

	if (err) {
		device_printf(bus->bdev, "Root HUB problem, error=%s\n",
		    usbd_errstr(err));
	}
	/* Make sure that the USB BUS is explored */
	bus->needs.explore = 1;

	snprintf(bus->usb_name, sizeof(bus->usb_name), "usb%u", device_get_unit(dev));

	bus->usb_clone_tag = EVENTHANDLER_REGISTER(dev_clone, usb_clone_ptr, bus, 1000);
	if (bus->usb_clone_tag == NULL) {
		device_printf(dev, "Registering clone handler failed!\n");
	}
	/* create a dummy device so that we are visible */
	bus->usb_cdev =
	    make_dev(&usb_dummy_cdevsw, device_get_unit(dev),
	    UID_ROOT, GID_OPERATOR, 0660, "%s ", bus->usb_name);

	if (bus->usb_cdev == NULL) {
		device_printf(dev, "Creating dummy device failed!\n");
	}
	/* create and start the event thread */

	usb_create_event_thread(bus);

	return;
}

static int
usb_attach(device_t dev)
{
	struct usbd_bus *bus = device_get_softc(dev);

	mtx_lock(&usb_global_lock);

	if (usb_post_init_called != 0) {
		usb_attach_sub(dev, bus);
	}
	mtx_unlock(&usb_global_lock);

	return (0);			/* return success */
}

static void
usb_post_init(void *arg)
{
	struct usbd_bus *bus;
	devclass_t dc;
	device_t dev;
	int max;
	int n;

	mtx_lock(&usb_global_lock);

	dc = devclass_find("usb");

	if (dc) {
		max = devclass_get_maxunit(dc) + 1;
		for (n = 0; n != max; n++) {
			dev = devclass_get_device(dc, n);
			if (dev) {
				bus = device_get_softc(dev);
				usb_attach_sub(dev, bus);
			}
		}
	}
	usb_post_init_called = 1;

	/* explore all USB busses in parallell */

	usb_needs_probe_and_attach();

	mtx_unlock(&usb_global_lock);

	return;
}

SYSINIT(usb_post_init, SI_SUB_KICK_SCHEDULER, SI_ORDER_ANY, usb_post_init, NULL);

static int
usb_detach(device_t dev)
{
	struct usbd_bus *bus = device_get_softc(dev);
	struct usbd_device *udev = bus->devices[USB_ROOT_HUB_ADDR];

	PRINTF(("start\n"));

	/* get rid of explore thread */
	usb_needs_explore(bus, USB_BUS_EXPLORE_STOP);

	mtx_lock(&usb_global_lock);

	/* detach children first */
	bus_generic_detach(dev);

	/*
	 * Free USB Root device, but not any sub-devices, hence they
	 * are freed by the caller of this function
	 */
	usbd_detach_device(udev, USB_IFACE_INDEX_ANY, 0);
	usbd_free_device(udev);

	mtx_unlock(&usb_global_lock);

	mtx_lock(&bus->mtx);
	if (bus->bdev == dev) {
		/*
		 * need to clear bus->bdev here so that the parent detach
		 * routine does not free this device again
		 */
		bus->bdev = NULL;
	} else {
		device_printf(dev, "unexpected bus->bdev value!\n");
	}
	mtx_unlock(&bus->mtx);

	if (bus->usb_cdev) {
		destroy_dev(bus->usb_cdev);
		bus->usb_cdev = NULL;
	}
	if (bus->usb_clone_tag) {
		EVENTHANDLER_DEREGISTER(dev_clone, bus->usb_clone_tag);
		bus->usb_clone_tag = NULL;
	}
	usb_clone_remove(bus);

	return (0);
}

static struct usbd_clone *
usb_clone_sub(struct usbd_bus *bus)
{
	struct usbd_clone *sub;
	int32_t error;
	const char *p_name[2];
	char n_name[64];

	mtx_lock(&(bus->mtx));
	sub = bus->usb_clone_root;
	mtx_unlock(&(bus->mtx));

	while (sub) {
		if (!usb_cdev_opened(&(sub->cdev))) {
			return (sub);
		}
		sub = sub->next;
	}

	sub = malloc(sizeof(*sub), M_USBDEV, M_ZERO | M_WAITOK);
	if (sub == NULL) {
		return (NULL);
	}
	mtx_lock(&(bus->mtx));
	sub->unit = bus->usb_clone_count;
	if (bus->usb_clone_count < USB_BUS_MAX_CLONES) {
		bus->usb_clone_count++;
	}
	mtx_unlock(&(bus->mtx));

	if (sub->unit >= USB_BUS_MAX_CLONES) {
		/* limit reached */
		goto error;
	}
	snprintf(n_name, sizeof(n_name), "%s.%02x", bus->usb_name, sub->unit);

	mtx_init(&(sub->mtx), "usb_clone", NULL, MTX_DEF | MTX_RECURSE);

	p_name[0] = n_name;
	p_name[1] = NULL;

	sub->cdev.sc_ioctl = &usb_ioctl;
	sub->bus = bus;

	error = usb_cdev_attach(&(sub->cdev), sub, &(sub->mtx),
	    p_name, UID_ROOT, GID_OPERATOR, 0660,
	    0, 0, 0, 0);
	if (error) {
		goto error;
	}
	/* insert sub-device into a one-way linked list */

	mtx_lock(&(bus->mtx));
	sub->next = bus->usb_clone_root;
	bus->usb_clone_root = sub;
	mtx_unlock(&(bus->mtx));
	return (sub);

error:
	free(sub, M_USBDEV);
	return (NULL);
}

static void
usb_clone_remove(struct usbd_bus *bus)
{
	struct usbd_clone *sub;
	struct usbd_clone *next;
	uint8_t done = 0;

	while (1) {

		/*
		 * first prevent any further clones from being created:
		 */
		mtx_lock(&(bus->mtx));
		bus->usb_clone_count = USB_BUS_MAX_CLONES;
		sub = bus->usb_clone_root;
		bus->usb_clone_root = NULL;
		mtx_unlock(&(bus->mtx));

		/*
		 * XXX wait for any leftover VOP_LOOKUP() calls to finish.
		 * Really we should lock some lock here to lock that problem
		 * out! --hps
		 */
		usb_delay_ms(bus, 500);

		if (sub == NULL) {
			if (done) {
				break;
			} else {
				done = 1;
			}
		} else {
			done = 0;
		}

		/* teardown all cloned devices */
		while (sub) {

			usb_cdev_detach(&(sub->cdev));

			next = sub->next;

			mtx_destroy(&(sub->mtx));

			free(sub, M_USBDEV);

			sub = next;
		}
	}
	return;
}

static void
usb_clone(void *arg, USB_UCRED char *name, int namelen, struct cdev **dev)
{
	struct usbd_bus *bus = arg;
	struct usbd_clone *sub;

	if (*dev) {
		return;
	}
	if (strcmp(name, bus->usb_name) != 0) {
		return;
	}
	sub = usb_clone_sub(bus);
	if (sub == NULL) {
		return;
	}
	*dev = sub->cdev.sc_cdev[0];

	dev_ref(*dev);
	return;
}

static int
usb_ioctl(struct usb_cdev *dev, u_long cmd, caddr_t addr,
    int32_t fflags, struct thread *td)
{
	struct usbd_clone *sub = dev->sc_priv_ptr;
	struct usbd_bus *bus = sub->bus;
	struct usbd_device *udev = 0;
	int error = 0;

	usb_cdev_unlock(dev, fflags);

	switch (cmd) {
		/* this part should be deleted */
	case USB_DISCOVER:
		break;

	case USB_REQUEST:
		{
			struct usb_ctl_request *ur = (void *)addr;
			uint16_t len = UGETW(ur->ucr_request.wLength);
			uint8_t isread = (ur->ucr_request.bmRequestType & UT_READ) ? 1 : 0;
			void *ptr = 0;

			udev = usbd_ref_device(bus, ur->ucr_addr);
			if (udev == NULL) {
				error = ENXIO;
				goto done;
			}
			PRINTF(("USB_REQUEST addr=%d len=%d\n", ur->ucr_addr, len));

			if (len != 0) {
				ptr = malloc(len, M_TEMP, M_WAITOK);
				if (ptr == NULL) {
					error = ENOMEM;
					goto ret001;
				}
				if (isread == 0) {
					error = copyin(ur->ucr_data, ptr, len);
					if (error) {
						goto ret001;
					}
				}
			}
			error = usbd_do_request_flags
			    (udev, NULL, &ur->ucr_request, ptr, ur->ucr_flags,
			    &len, USBD_DEFAULT_TIMEOUT);

			ur->ucr_actlen = len;

			if (error) {
				error = EIO;
				goto ret001;
			}
			if (len != 0) {
				if (isread) {
					error = copyout(ptr, ur->ucr_data, len);
					if (error) {
						goto ret001;
					}
				}
			}
	ret001:
			if (ptr) {
				free(ptr, M_TEMP);
			}
			usbd_unref_device(udev);
			goto done;
		}

	case USB_DEVICEINFO:
		{
			struct usb_device_info *di = (void *)addr;

			udev = usbd_ref_device(bus, di->udi_addr);
			if (udev == NULL) {
				error = ENXIO;
				goto done;
			}
			error = usbd_fill_deviceinfo(udev, di);

			usbd_unref_device(udev);
			goto done;
		}

	case USB_DEVICESTATS:
		*(struct usb_device_stats *)addr = bus->stats;
		break;

	case USB_DEVICEENUMERATE:
		{
			struct usb_device_enumerate *ude = (void *)addr;
			struct usbd_device *parent_hub;
			uint8_t old_addr;
			uint8_t buf[8];

			udev = usbd_ref_device(bus, ude->ude_addr);
			if (udev == NULL) {
				error = ENXIO;
				goto done;
			}
			old_addr = udev->address;
			parent_hub = udev->parent_hub;
			if (parent_hub == NULL) {
				error = EINVAL;
				goto ret002;
			}
			error = usbreq_reset_port(parent_hub, NULL, udev->port_no);
			if (error) {
				error = ENXIO;
				goto ret002;
			}
			/*
			 * After that the port has been reset our device
			 * should be at address zero:
			 */
			udev->address = USB_START_ADDR;

			/*
			 * It should be allowed to read some descriptors
			 * from address zero:
			 */
			error = usbreq_get_desc(udev, NULL, buf,
			    8, 8, 0, UDESC_DEVICE, 0, 0);
			if (error) {
				error = ENXIO;
				goto ret002;
			}
			/*
			 * Restore device address:
			 */
			error = usbreq_set_address(udev, NULL, old_addr);
			if (error) {
				error = ENXIO;
				goto ret002;
			}
	ret002:
			/* restore address */
			udev->address = old_addr;
			usbd_unref_device(udev);
			break;
		}

		/* ... more IOCTL's to come ! ... --hps */

	default:
		error = EINVAL;
		break;
	}
done:
	return (usb_cdev_lock(dev, fflags, error));
}

#ifndef usb_global_lock
struct mtx usb_global_lock;

#endif

static void
usb_init(void *arg)
{
#ifndef usb_global_lock
	mtx_init(&usb_global_lock, "usb_global_lock",
	    NULL, MTX_DEF | MTX_RECURSE);
#endif

	return;
}

SYSINIT(usb_init, SI_SUB_DRIVERS, SI_ORDER_FIRST, usb_init, NULL);

static void
usb_uninit(void *arg)
{
#ifndef usb_global_lock
	mtx_destroy(&usb_global_lock);
#endif
	return;
}

SYSUNINIT(usb_uninit, SI_SUB_KLD, SI_ORDER_ANY, usb_uninit, NULL);
