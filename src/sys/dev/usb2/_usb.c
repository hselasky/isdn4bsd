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

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/malloc.h>
#include <sys/bus.h> /* device_xxx() */
#include <sys/module.h>

#include <sys/uio.h> /* UIO_XXX */
#include <sys/sysctl.h> /* SYSCTL_XXX() */
#include <sys/proc.h>
#include <sys/unistd.h>
#include <sys/filio.h> /* FXXX */
#include <sys/ioccom.h> /* IOR()/IOW()/IORW() */
#include <sys/kthread.h>
#include <sys/poll.h>
#include <sys/conf.h>
#include <sys/signalvar.h>
#include <sys/vnode.h>

#include <machine/bus.h>

#include <dev/usb2/usb_port.h>
#include <dev/usb2/usb.h>
#include <dev/usb2/usb_subr.h>
#include <dev/usb2/usb_quirks.h>

__FBSDID("$FreeBSD: src/sys/dev/usb2/usb.c $");

#define DEV2UNIT(d)	(minor(d))
#define DEV2BUS(d)	(*((struct usbd_bus **)&((d)->si_drv1)))

#define USB_DEV_MINOR	255		/* event queue device */

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

/*
 * 0  - do usual exploration
 * 1  - do not use timeout exploration
 * >1 - do no exploration
 */
int	usb_noexplore = 0;
#endif

#define USB_MAX_EVENTS 100
struct usb_event_wrapper
{
	struct usb_event ue;
	TAILQ_ENTRY(usb_event_wrapper) next;
};

static TAILQ_HEAD(, usb_event_wrapper) usb_events =
	TAILQ_HEAD_INITIALIZER(usb_events);

#ifndef usb_global_lock
struct mtx usb_global_lock;
#endif

/* these variables are protected by "usb_global_lock" */
static int usb_nevents = 0;
static struct selinfo usb_selevent;
static struct proc *usb_async_proc;  /* process that wants USB SIGIO */
static int usb_dev_open = 0;

/**/
static const char * const usbrev_str[] = USBREV_STR;

/* 
 * usb_discover - explore the device tree from the root
 * 
 * usb_discover device nodes, kthread
 */
static void
usb_discover(struct usbd_bus *bus)
{
	PRINTFN(2,("\n"));

#ifdef USB_DEBUG
	if(usb_noexplore > 1)
	{
		return;
	}
#endif
	mtx_assert(&usb_global_lock, MA_OWNED);

	/* check that only one thread is exploring
	 * at a time
	 */
	while(bus->is_exploring)
	{
		bus->wait_explore = 1;

		msleep(&bus->wait_explore, &usb_global_lock, PWAIT,
		       "usb wait explore", 0);
	}

	bus->is_exploring = 1;

	while(bus->root_port.device &&
	      bus->root_port.device->hub &&
	      bus->needs_explore &&
	      (bus->wait_explore == 0))
	{
		bus->needs_explore = 0;

		/* explore the hub 
		 * (this call can sleep,
		 *  exiting usb_global_lock, 
		 *  which is actually Giant)
		 */
		(bus->root_port.device->hub->explore)
		  (bus->root_port.device);
	}

	bus->is_exploring = 0;

	if(bus->wait_explore)
	{
		bus->wait_explore = 0;
		wakeup(&bus->wait_explore);
	}
	return;
}

static void
usb_event_thread(struct usbd_bus *bus)
{
	mtx_lock(&usb_global_lock);

	while(1)
	{
		if(bus->root_port.device == 0)
		{
			break;
		}

#ifdef USB_DEBUG
		if(usb_noexplore < 2)
#endif
		{
			usb_discover(bus);
		}

#ifdef USB_DEBUG
		msleep(&bus->needs_explore, &usb_global_lock, PWAIT,
		       "usbevt", usb_noexplore ? 0 : hz * 60);
#else
		msleep(&bus->needs_explore, &usb_global_lock, PWAIT,
		       "usbevt", hz * 60);
#endif
		PRINTFN(2,("woke up\n"));
	}

	bus->event_thread = NULL;

	/* in case parent is waiting for us to exit */
	wakeup(bus);

	mtx_unlock(&usb_global_lock);

	PRINTF(("exit\n"));

	kthread_exit(0);

	return;
}

void
usb_needs_explore(struct usbd_device *udev)
{
	PRINTFN(2,("\n"));

	mtx_lock(&usb_global_lock);
	udev->bus->needs_explore = 1;
	wakeup(&udev->bus->needs_explore);
	mtx_unlock(&usb_global_lock);
	return;
}

u_int8_t usb_driver_added_refcount;

void
usb_needs_probe_and_attach(void)
{
	struct usbd_bus *bus;
	devclass_t dc;
	device_t dev;
	int max;

	PRINTFN(2,("\n"));

	mtx_lock(&usb_global_lock);

	usb_driver_added_refcount++;

	dc = devclass_find("usb");

	if(dc)
	{
	    max = devclass_get_maxunit(dc);
 	    while(max >= 0)
	    {
	        dev = devclass_get_device(dc, max);
		if(dev)
		{
		    bus = device_get_softc(dev);

		    bus->needs_explore = 1;
		    wakeup(&bus->needs_explore);
		}
		max--;
	    }
	}
	else
	{
	    printf("%s: \"usb\" devclass not present!\n",
		   __FUNCTION__);
	}
	mtx_unlock(&usb_global_lock);
	return;
}

static void
usb_create_event_thread(struct usbd_bus *bus)
{
	if(usb_kthread_create1((void*)(void*)&usb_event_thread, bus, &bus->event_thread,
			"%s", device_get_nameunit(bus->bdev)))
	{
		device_printf(bus->bdev, "unable to create event thread for\n");
		panic("usb_create_event_thread");
	}
	return;
}

static int
usb_event_get_next(struct usb_event *ue)
{
	struct usb_event_wrapper *uew;
	int err;

	mtx_lock(&usb_global_lock);

	uew = TAILQ_FIRST(&usb_events);

	if(uew == NULL)
	{
		usb_nevents = 0;
		err = 0;
	}
	else
	{
		*ue = uew->ue;

		TAILQ_REMOVE(&usb_events, uew, next);

		free(uew, M_USBDEV);

		if(usb_nevents)
		{
		   usb_nevents--;
		}
		err = 1;
	}
	mtx_unlock(&usb_global_lock);
	return (err);
}

static void
usb_event_add(int type, struct usb_event *uep)
{
	struct usb_event_wrapper *uew;
	struct timeval thetime;

	uew = malloc(sizeof *uew, M_USBDEV, M_WAITOK|M_ZERO);
	if(uew == NULL)
	{
		return;
	}
	uew->ue = *uep;
	uew->ue.ue_type = type;
	microtime(&thetime);
	TIMEVAL_TO_TIMESPEC(&thetime, &uew->ue.ue_time);

	mtx_lock(&usb_global_lock);

	if(USB_EVENT_IS_DETACH(type))
	{
		struct usb_event_wrapper *uewi, *uewi_next;

		for (uewi = TAILQ_FIRST(&usb_events);
		     uewi;
		     uewi = uewi_next)
		{
			uewi_next = TAILQ_NEXT(uewi, next);
			if(uewi->ue.u.ue_driver.ue_cookie.cookie ==
			    uep->u.ue_device.udi_cookie.cookie)
			{
				TAILQ_REMOVE(&usb_events, uewi, next);
				free(uewi, M_USBDEV);
				usb_nevents--;
				uewi_next = TAILQ_FIRST(&usb_events);
			}
		}
	}
	if(usb_nevents >= USB_MAX_EVENTS)
	{
		/* too many queued events, drop an old one */
		PRINTF(("event dropped\n"));

		struct usb_event ue;
		(void)usb_event_get_next(&ue);
	}
	TAILQ_INSERT_TAIL(&usb_events, uew, next);
	usb_nevents++;
	wakeup(&usb_events);
	selwakeuppri(&usb_selevent, PZERO);
	if(usb_async_proc != NULL)
	{
		PROC_LOCK(usb_async_proc);
		psignal(usb_async_proc, SIGIO);
		PROC_UNLOCK(usb_async_proc);
	}

	mtx_unlock(&usb_global_lock);
	return;
}

void
usbd_add_dev_event(int type, struct usbd_device *udev)
{
	struct usb_event ue;

	bzero(&ue, sizeof(ue));

	usbd_fill_deviceinfo(udev, &ue.u.ue_device,
			     USB_EVENT_IS_ATTACH(type));
	usb_event_add(type, &ue);
	return;
}

void
usbd_add_drv_event(int type, struct usbd_device *udev, struct device *dev)
{
	struct usb_event ue;

	bzero(&ue, sizeof(ue));

	ue.u.ue_driver.ue_cookie = udev->cookie;
	strncpy(ue.u.ue_driver.ue_devname, device_get_nameunit(dev),
		sizeof ue.u.ue_driver.ue_devname);
	usb_event_add(type, &ue);
	return;
}

/* XXX NOTE: caller must increase dev->refcount !! */

#if defined(__NetBSD__) || defined(__OpenBSD__)
static int
usbd_print(void *aux, const char *pnp)
{
	struct usb_attach_arg *uaa = aux;
	char devinfo[1024];

	PRINTFN(15, ("dev=%p\n", uaa->device));
	if(pnp)
	{
		if(!uaa->usegeneric)
		{
			return (QUIET);
		}
		usbd_devinfo(uaa->device, 1, devinfo);
		printf("%s, %s", devinfo, pnp);
	}
	if(uaa->port != 0)
	{
		printf(" port %d", uaa->port);
	}
	if(uaa->configno != UHUB_UNK_CONFIGURATION)
	{
		printf(" configuration %d", uaa->configno);
	}
	if(uaa->ifaceno != UHUB_UNK_INTERFACE)
	{
		printf(" interface %d", uaa->ifaceno);
	}
#if 0
	/*
	 * It gets very crowded with these locators on the attach line.
	 * They are not really needed since they are printed in the clear
	 * by each driver.
	 */
	if(uaa->vendor != UHUB_UNK_VENDOR)
	{
		printf(" vendor 0x%04x", uaa->vendor);
	}
	if(uaa->product != UHUB_UNK_PRODUCT)
	{
		printf(" product 0x%04x", uaa->product);
	}
	if(uaa->release != UHUB_UNK_RELEASE)
	{
		printf(" release 0x%04x", uaa->release);
	}
#endif
	return (UNCONF);
}

#if defined(__NetBSD__)
static int
usbd_submatch(struct device *parent, struct cfdata *cf, void *aux)
{
#elif defined(__OpenBSD__)
static int
usbd_submatch(struct device *parent, void *match, void *aux)
{
	struct cfdata *cf = match;
#endif
	struct usb_attach_arg *uaa = aux;

	PRINTFN(5,("port=%d,%d configno=%d,%d "
	    "ifaceno=%d,%d vendor=%d,%d product=%d,%d release=%d,%d\n",
	    uaa->port, cf->uhubcf_port,
	    uaa->configno, cf->uhubcf_configuration,
	    uaa->ifaceno, cf->uhubcf_interface,
	    uaa->vendor, cf->uhubcf_vendor,
	    uaa->product, cf->uhubcf_product,
	    uaa->release, cf->uhubcf_release));
	if((uaa->port != 0) &&	/* root hub has port 0, it should match */

	   (
		((uaa->port != 0) &&
		 (cf->uhubcf_port != UHUB_UNK_PORT) &&
		 (cf->uhubcf_port != uaa->port) ) ||

		((uaa->configno != UHUB_UNK_CONFIGURATION) &&
		 (cf->uhubcf_configuration != UHUB_UNK_CONFIGURATION) &&
		 (cf->uhubcf_configuration != uaa->configno)) ||

		((uaa->ifaceno != UHUB_UNK_INTERFACE) &&
		 (cf->uhubcf_interface != UHUB_UNK_INTERFACE) &&
		 (cf->uhubcf_interface != uaa->ifaceno)) ||

		((uaa->vendor != UHUB_UNK_VENDOR) &&
		 (cf->uhubcf_vendor != UHUB_UNK_VENDOR) &&
		 (cf->uhubcf_vendor != uaa->vendor)) ||

		((uaa->product != UHUB_UNK_PRODUCT) &&
		 (cf->uhubcf_product != UHUB_UNK_PRODUCT) &&
		 (cf->uhubcf_product != uaa->product)) ||

		((uaa->release != UHUB_UNK_RELEASE) &&
		 (cf->uhubcf_release != UHUB_UNK_RELEASE) &&
		 (cf->uhubcf_release != uaa->release))
		)
	   )
	{
		return 0;
	}
	if((cf->uhubcf_vendor != UHUB_UNK_VENDOR) &&
	   (cf->uhubcf_vendor == uaa->vendor) &&
	   (cf->uhubcf_product != UHUB_UNK_PRODUCT) &&
	   (cf->uhubcf_product == uaa->product))
	{
		/* we have a vendor & product locator match */
		if((cf->uhubcf_release != UHUB_UNK_RELEASE) &&
		   (cf->uhubcf_release == uaa->release))
		{
			uaa->matchlvl = UMATCH_VENDOR_PRODUCT_REV;
		}
		else
		{
			uaa->matchlvl = UMATCH_VENDOR_PRODUCT;
		}
	}
	else
	{
		uaa->matchlvl = 0;
	}

	return ((*cf->cf_attach->ca_match)(parent, cf, aux));
}
#endif


/* called from uhci_pci_attach */

USB_MATCH(usb)
{
	PRINTF(("\n"));
	return (UMATCH_GENERIC);
}

extern struct cdevsw usb_cdevsw;

static void
__usb_attach(device_t self, struct usbd_bus *bus)
{
	usbd_status err;
	u_int8_t speed;
	struct usb_event ue;

	PRINTF(("\n"));

	mtx_assert(&usb_global_lock, MA_OWNED);

	bus->root_port.power = USB_MAX_POWER;

	device_printf(bus->bdev, "USB revision %s",
		      usbrev_str[bus->usbrev]);

	switch (bus->usbrev)
	{
	case USBREV_1_0:
	case USBREV_1_1:
		speed = USB_SPEED_FULL;
		break;

	case USBREV_2_0:
		speed = USB_SPEED_HIGH;
		break;

	default:
		printf(", not supported\n");
		return;
	}

	printf("\n");

	/* make sure not to use tsleep() if we are cold booting */
	if(cold)
	{
		bus->use_polling++;
	}

	ue.u.ue_ctrlr.ue_bus = device_get_unit(bus->bdev);
	usb_event_add(USB_EVENT_CTRLR_ATTACH, &ue);

	err = usbd_new_device(bus->bdev, bus, 0, speed, 0,
			      &bus->root_port);
	if(!err)
	{
		if(bus->root_port.device->hub == NULL)
		{
			device_printf(bus->bdev, 
				      "root device is not a hub\n");
			return;
		}

		/*
		 * the USB bus is explored here so that devices, 
		 * for example the keyboard, can work during boot
		 */

		/* make sure that the bus is explored */
		bus->needs_explore = 1;

		usb_discover(bus);
	}
	else
	{
		device_printf(bus->bdev, "root hub problem, error=%s\n",
			      usbd_errstr(err));
	}

	if(cold)
	{
		bus->use_polling--;
	}

#if defined(__NetBSD__) || defined(__OpenBSD__)
	usb_kthread_create
	  ((void*)(void*)&usb_create_event_thread, bus);
#endif

#if defined(__FreeBSD__)
	usb_create_event_thread(bus);

	struct cdev *dev;

	/* the per controller devices (used for usb_discover) */
	/* XXX This is redundant now, but old usbd's will want it */
	dev = make_dev(&usb_cdevsw, device_get_unit(self), UID_ROOT, GID_OPERATOR,
		       0660, "usb%d", device_get_unit(self));

	if(dev)
	{
		DEV2BUS(dev) = bus;
	}
#endif
	return;
}

static u_int8_t usb_post_init_called = 0;

USB_ATTACH(usb)
{
	struct usbd_bus *bus = device_get_softc(self);

	mtx_lock(&usb_global_lock);

	if(usb_post_init_called != 0)
	{
		__usb_attach(self, bus);
	}

	mtx_unlock(&usb_global_lock);

	USB_ATTACH_SUCCESS_RETURN;
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

	if(dc)
	{
	    max = devclass_get_maxunit(dc);
	    for(n = 0; n <= max; n++)
	    {
	        dev = devclass_get_device(dc, n);
		if(dev)
		{
		    bus = device_get_softc(dev);

		    __usb_attach(dev, bus);
		}
	    }
	}
	else
	{
	    printf("%s: \"usb\" devclass not present!\n",
		   __FUNCTION__);
	}

	usb_post_init_called = 1;

	mtx_unlock(&usb_global_lock);

	return;
}

SYSINIT(usb_post_init, SI_SUB_PSEUDO, SI_ORDER_ANY, usb_post_init, NULL);

#if defined(__NetBSD__) || defined(__OpenBSD__)
int
usb_activate(device_ptr_t self, enum devact act)
{
	struct usbd_bus *bus = device_get_softc(self);
	struct usbd_device *udev = bus->root_port.device;
	int i, rv = 0;

	switch (act)
	{
	case DVACT_ACTIVATE:
		return (EOPNOTSUPP);

	case DVACT_DEACTIVATE:

		lock();
		bus->root_port.device = NULL;
		unlock();

		XXX subdevs is a structure;
		XXX subdevs are not zero terminated;

		if((udev != NULL) &&
		    (udev->cdesc != NULL) &&
		    (udev->subdevs != NULL))
		{
			for (i = 0; udev->subdevs[i]; i++)
			{
				rv |= config_deactivate(udev->subdevs[i]);
			}
		}
		break;
	}
	return (rv);
}
#endif

static int
usb_detach(device_t self, int flags)
{
	struct usbd_bus *bus = device_get_softc(self);
	struct usb_event ue;

	PRINTF(("start\n"));

	mtx_lock(&usb_global_lock);

	/* wait for any possible explore calls to finish */
	while(bus->is_exploring)
	{
		bus->wait_explore = 1;

		msleep(&bus->wait_explore, &usb_global_lock, PWAIT,
		       "usb wait explore", 0);
	}

	if(bus->root_port.device != NULL)
	{
		/* free device, but not sub-devices,
		 * hence they are freed by the 
		 * caller of this function
		 */
		usbd_free_device(&bus->root_port, 0);
	}

	/* kill off event thread */
	if(bus->event_thread != NULL)
	{
		wakeup(&bus->needs_explore);

		if(msleep(bus, &usb_global_lock, PWAIT, "usbdet", hz * 60))
		{
			device_printf(bus->bdev,
				      "event thread didn't die\n");
		}
		PRINTF(("event thread dead\n"));
	}

	mtx_unlock(&usb_global_lock);

	ue.u.ue_ctrlr.ue_bus = device_get_unit(bus->bdev);
	usb_event_add(USB_EVENT_CTRLR_DETACH, &ue);

	mtx_lock(&bus->mtx);
	if(bus->bdev == self)
	{
		/* need to clear bus->bdev
		 * here so that the parent
		 * detach routine does not
		 * free this device again
		 */
		bus->bdev = NULL;
	}
	else
	{
		device_printf(self, "unexpected bus->bdev value!\n");
	}
	mtx_unlock(&bus->mtx);
	return (0);
}

static int
usbopen(struct cdev *dev, int flag, int mode, struct thread *proc)
{
	int error = 0;

	mtx_lock(&usb_global_lock);

	if(DEV2UNIT(dev) == USB_DEV_MINOR)
	{
		if(usb_dev_open)
		{
			error = EBUSY;
			goto done;
		}
		usb_dev_open = 1;
		usb_async_proc = 0;
	}
	else
	{
		struct usbd_bus *bus = DEV2BUS(dev);

		if(bus->root_port.device == NULL)
		{
			/* device is beeing detached */
			error = EIO;
			goto done;
		}
	}

 done:
	mtx_unlock(&usb_global_lock);
	return (error);
}

static int
usbread(struct cdev *dev, struct uio *uio, int flag)
{
	struct usb_event ue;
	int error = 0;

	if(DEV2UNIT(dev) != USB_DEV_MINOR)
	{
		return (ENODEV);
	}

	if(uio->uio_resid != sizeof(struct usb_event))
	{
		return (EINVAL);
	}

	mtx_lock(&usb_global_lock);

	for(;;)
	{
		if(usb_event_get_next(&ue) != 0)
		{
			break;
		}
		if(flag & IO_NDELAY)
		{
			error = EWOULDBLOCK;
			break;
		}
		error = msleep(&usb_events, &usb_global_lock,
			       (PZERO|PCATCH), "usbrea", 0);
		if(error)
		{
			break;
		}
	}

	mtx_unlock(&usb_global_lock);

	if(!error)
	{
		error = uiomove((void *)&ue, uio->uio_resid, uio);
	}
	return (error);
}

static int
usbclose(struct cdev *dev, int flag, int mode, struct thread *proc)
{
	if(DEV2UNIT(dev) == USB_DEV_MINOR)
	{
		mtx_lock(&usb_global_lock);

		usb_async_proc = 0;
		usb_dev_open = 0;

		mtx_unlock(&usb_global_lock);
	}
	return (0);
}

static int
usbioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag, struct thread *p)
{
	int error = 0;

	mtx_lock(&usb_global_lock);

	if(DEV2UNIT(dev) == USB_DEV_MINOR)
	{
		switch (cmd)
		{
		case FIONBIO:
			/* all handled in the upper FS layer */
			break;

		case FIOASYNC:
			if(*(int *)data)
#if __FreeBSD_version >= 500000
				usb_async_proc = p->td_proc;
#else
				usb_async_proc = p;
#endif
			else
				usb_async_proc = 0;

			break;

		default:
			error = EINVAL;
			break;
		}
	}
	else
	{
		struct usbd_bus *bus = DEV2BUS(dev);

		if(bus->root_port.device == NULL)
		{
			/* detached */
			error = EIO;
			goto done;
		}

		switch (cmd)
		{
#if defined(__FreeBSD__)
		/* this part should be deleted */
		case USB_DISCOVER:
			break;
#endif
		case USB_REQUEST:
		{
			struct usb_ctl_request *ur = (void *)data;
			int len = UGETW(ur->ucr_request.wLength);
			struct iovec iov;
			struct uio uio;
			void *ptr = 0;
			int addr = ur->ucr_addr;
			usbd_status err;
			int error = 0;

			PRINTF(("USB_REQUEST addr=%d len=%d\n", addr, len));
			if((len < 0) ||
			   (len > 32768))
			{
				error = EINVAL;
				goto done;
			}

			if((addr < 0) || 
			   (addr >= USB_MAX_DEVICES) ||
			   (bus->devices[addr] == 0 /* might be checked by usbd_do_request_flags */))
			{
				error = EINVAL;
				goto done;
			}

			if(len != 0)
			{
				iov.iov_base = (caddr_t)ur->ucr_data;
				iov.iov_len = len;
				uio.uio_iov = &iov;
				uio.uio_iovcnt = 1;
				uio.uio_resid = len;
				uio.uio_offset = 0;
				uio.uio_segflg = UIO_USERSPACE;
				uio.uio_rw =
				  ur->ucr_request.bmRequestType & UT_READ ?
				  UIO_READ : UIO_WRITE;
				uio.uio_procp = p;
				ptr = malloc(len, M_TEMP, M_WAITOK);
				if(uio.uio_rw == UIO_WRITE)
				{
					error = uiomove(ptr, len, &uio);
					if(error)
					{
						goto ret;
					}
				}
			}
			err = usbd_do_request_flags
			  (bus->devices[addr], &ur->ucr_request, ptr,
			   ur->ucr_flags, &ur->ucr_actlen,
			   USBD_DEFAULT_TIMEOUT);
			if(err)
			{
				error = EIO;
				goto ret;
			}
			if(len != 0)
			{
				if(uio.uio_rw == UIO_READ)
				{
					error = uiomove(ptr, len, &uio);
					if(error)
					{
						goto ret;
					}
				}
			}
		ret:
			if(ptr)
			{
				free(ptr, M_TEMP);
			}
			goto done;
		}

		case USB_DEVICEINFO:
		{
			struct usb_device_info *di = (void *)data;
			int addr = di->udi_addr;

			if((addr < 1) ||
			   (addr >= USB_MAX_DEVICES) ||
			   (bus->devices[addr] == 0))
			{
				error = EINVAL;
				goto done;
			}

			error = usbd_fill_deviceinfo(bus->devices[addr], di, 1);
			goto done;
		}

		case USB_DEVICESTATS:
			*(struct usb_device_stats *)data = bus->stats;
			break;

		default:
			error = EINVAL;
			break;
		}
	}

 done:
	mtx_unlock(&usb_global_lock);
	return (error);
}

static int
usbpoll(struct cdev *dev, int events, struct thread *td)
{
	int revents, mask;
	int unit = DEV2UNIT(dev);

	if(unit == USB_DEV_MINOR)
	{
		revents = 0;
		mask = POLLIN | POLLRDNORM;

		mtx_lock(&usb_global_lock);

		if((events & mask) && (usb_nevents > 0))
		{
			revents |= events & mask;
		}
		if((revents == 0) && (events & mask))
		{
			selrecord(td, &usb_selevent);
		}

		mtx_unlock(&usb_global_lock);

		return (revents);
	}
	else
	{
#if defined(__FreeBSD__)
		return (0);	/* select/poll never wakes up - back compat */
#else
		return (ENXIO);
#endif
	}
}

struct cdevsw usb_cdevsw =
{
#ifdef D_VERSION
	.d_version =	D_VERSION,
#endif
	.d_open =	usbopen,
	.d_close =	usbclose,
	.d_read =	usbread,
	.d_ioctl =	usbioctl,
	.d_poll =	usbpoll,
	.d_name =	"usb",
};

static void
usb_init(void *arg)
{
#ifndef usb_global_lock
	mtx_init(&usb_global_lock, "usb_global_lock",
		 NULL, MTX_DEF|MTX_RECURSE);
#endif

#if defined(__FreeBSD__)
	struct cdev *dev;

	/* the device spitting out events */
	dev = make_dev(&usb_cdevsw, USB_DEV_MINOR, UID_ROOT, GID_OPERATOR,
		       0660, "usb");

	if(dev)
	{
		DEV2BUS(dev) = NULL;
	}
#endif
	return;
}

SYSINIT(usb_init, SI_SUB_DRIVERS, SI_ORDER_FIRST, usb_init, NULL);

static void
bus_dmamap_load_callback(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	((u_int32_t *)arg)[0] = segs->ds_addr;

	if(error)
	{
	    printf("%s: %s: error=%d\n",
		   __FILE__, __FUNCTION__, error);
	}
	return;
}

struct usb_dma 
{
	struct bus_dma_tag *	tag;
	struct bus_dmamap *	map;
	u_int32_t		physaddr;
};

void *
usb_alloc_mem(struct bus_dma_tag *tag, u_int32_t size, u_int8_t align_power)
{
	struct bus_dmamap *map;
	u_int32_t physaddr = 0;
	void *ptr;

	size += sizeof(struct usb_dma);

	if(tag == NULL)
	{
	  if(bus_dma_tag_create
	   ( /* parent    */NULL,
	     /* alignment */(1 << align_power),
	     /* boundary  */0,
	     /* lowaddr   */BUS_SPACE_MAXADDR_32BIT,
	     /* highaddr  */BUS_SPACE_MAXADDR,
	     /* filter    */NULL,
	     /* filterarg */NULL,
	     /* maxsize   */size,
	     /* nsegments */1,
	     /* maxsegsz  */size,
	     /* flags     */0,
#if __FreeBSD_version >= 500000
	     /* lock      */NULL,
	     /*           */NULL,
#endif
	     &tag))
	  {
		return NULL;

	  }
	}

	if(bus_dmamem_alloc
	   (tag, &ptr, (BUS_DMA_NOWAIT|BUS_DMA_COHERENT), &map))
	{
		bus_dma_tag_destroy(tag);
		return NULL;
	}

	if(bus_dmamap_load
	   (tag, map, ptr, size, &bus_dmamap_load_callback, 
	    &physaddr, (BUS_DMA_NOWAIT|BUS_DMA_COHERENT)))
	{
		bus_dmamem_free(tag, ptr, map);
		bus_dma_tag_destroy(tag);
		return NULL;
	}

	size -= sizeof(struct usb_dma);

	((struct usb_dma *)(((u_int8_t *)ptr) + size))->tag = tag;
	((struct usb_dma *)(((u_int8_t *)ptr) + size))->map = map;
	((struct usb_dma *)(((u_int8_t *)ptr) + size))->physaddr = physaddr;

#ifdef USB_DEBUG
	if(usbdebug > 14)
	{
	    printf("%s: %p, %d bytes\n", 
		   __FUNCTION__, ptr, size);
	}
#endif
	return ptr;
}

u_int32_t
usb_vtophys(void *ptr, u_int32_t size)
{
	register u_int32_t temp = 
	  ((struct usb_dma *)(((u_int8_t *)ptr) + size))->physaddr;

#ifdef USB_DEBUG
	if(usbdebug > 14)
	{
	    printf("%s: %p, physaddr = 0x%08x\n", 
		   __FUNCTION__, ptr, temp);
	}
#endif
  	return temp;
}

void
usb_free_mem(void *ptr, u_int32_t size)
{
	struct bus_dma_tag *tag = 
	  ((struct usb_dma *)(((u_int8_t *)ptr) + size))->tag;
	struct bus_dmamap *map =
	  ((struct usb_dma *)(((u_int8_t *)ptr) + size))->map;

	bus_dmamap_unload(tag, map);

	bus_dmamem_free(tag, ptr, map);

	bus_dma_tag_destroy(tag);

#ifdef USB_DEBUG
	if(usbdebug > 14)
	{
	    printf("%s: %p, %d bytes\n", 
		   __FUNCTION__, ptr, size);
	}
#endif
	return;
}

static devclass_t usb_devclass;
static driver_t usb_driver =
{
	.name    = "usb",
	.methods = (device_method_t [])
	{
	  DEVMETHOD(device_probe, usb_match),
	  DEVMETHOD(device_attach, usb_attach),
	  DEVMETHOD(device_detach, usb_detach),
	  DEVMETHOD(device_suspend, bus_generic_suspend),
	  DEVMETHOD(device_resume, bus_generic_resume),
	  DEVMETHOD(device_shutdown, bus_generic_shutdown),
	  {0,0}
	},
	.size    = 0, /* the softc must be set by the attacher! */
};

DRIVER_MODULE(usb, ohci, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, uhci, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, ehci, usb_driver, usb_devclass, 0, 0);

MODULE_DEPEND(usb, usb, 1, 1, 1);
MODULE_VERSION(usb, 1);
