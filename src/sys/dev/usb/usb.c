#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/usb.c,v 1.112 2007/05/12 05:53:53 brueffer Exp $");

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

#include <sys/uio.h> /* UIO_XXX */
#include <sys/proc.h>
#include <sys/unistd.h>
#include <sys/filio.h> /* FXXX */
#include <sys/ioccom.h> /* IOR()/IOW()/IORW() */
#include <sys/kthread.h>
#include <sys/poll.h>
#include <sys/signalvar.h>
#include <sys/vnode.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>

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

static device_probe_t usb_probe;
static device_attach_t usb_attach;
static device_detach_t usb_detach;

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

		mtx_sleep(&bus->wait_explore, &usb_global_lock, PWAIT,
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
		mtx_sleep(&bus->needs_explore, &usb_global_lock, PWAIT,
		       "usbevt", usb_noexplore ? 0 : hz * 60);
#else
		mtx_sleep(&bus->needs_explore, &usb_global_lock, PWAIT,
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
usbd_add_drv_event(int type, struct usbd_device *udev, device_t dev)
{
	struct usb_event ue;

	bzero(&ue, sizeof(ue));

	ue.u.ue_driver.ue_cookie = udev->cookie;
	strncpy(ue.u.ue_driver.ue_devname, device_get_nameunit(dev),
		sizeof ue.u.ue_driver.ue_devname);
	usb_event_add(type, &ue);
	return;
}

/* called from uhci_pci_attach */

static int
usb_probe(device_t dev)
{
	PRINTF(("\n"));
	return (UMATCH_GENERIC);
}

extern cdevsw_t usb_cdevsw;

static void
__usb_attach(device_t dev, struct usbd_bus *bus)
{
	usbd_status err;
	u_int8_t speed;
	struct usb_event ue;
	struct cdev *cdev;

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

	usb_create_event_thread(bus);

	/* the per controller devices (used for usb_discover) */
	/* XXX This is redundant now, but old usbd's will want it */
	cdev = make_dev(&usb_cdevsw, device_get_unit(dev), UID_ROOT, GID_OPERATOR,
			0660, "usb%d", device_get_unit(dev));

	if(cdev)
	{
		DEV2BUS(cdev) = bus;
	}
	return;
}

static u_int8_t usb_post_init_called = 0;

static int
usb_attach(device_t dev)
{
	struct usbd_bus *bus = device_get_softc(dev);

	mtx_lock(&usb_global_lock);

	if(usb_post_init_called != 0)
	{
		__usb_attach(dev, bus);
	}

	mtx_unlock(&usb_global_lock);

	return 0; /* return success */
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

static int
usb_detach(device_t dev)
{
	struct usbd_bus *bus = device_get_softc(dev);
	struct usb_event ue;

	PRINTF(("start\n"));

	mtx_lock(&usb_global_lock);

	/* wait for any possible explore calls to finish */
	while(bus->is_exploring)
	{
		bus->wait_explore = 1;

		mtx_sleep(&bus->wait_explore, &usb_global_lock, PWAIT,
		       "usb wait explore", 0);
	}

	/* detach children first */
	bus_generic_detach(dev);

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

		if(mtx_sleep(bus, &usb_global_lock, PWAIT, "usbdet", hz * 60))
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
	if(bus->bdev == dev)
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
		device_printf(dev, "unexpected bus->bdev value!\n");
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
		error = mtx_sleep(&usb_events, &usb_global_lock,
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
				usb_async_proc = p->td_proc;
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
		/* this part should be deleted */
		case USB_DISCOVER:
			break;
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
			   (addr >= USB_MAX_DEVICES))
			{
				error = EINVAL;
				goto done;
			}

			if (bus->devices[addr] == 0)
			{
				error = ENXIO;
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
		/* select/poll never wakes up - back compat */
		return 0;
	}
}

cdevsw_t usb_cdevsw = {
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
	struct cdev *cdev;

#ifndef usb_global_lock
	mtx_init(&usb_global_lock, "usb_global_lock",
		 NULL, MTX_DEF|MTX_RECURSE);
#endif
	/* the device spitting out events */
	cdev = make_dev(&usb_cdevsw, USB_DEV_MINOR, UID_ROOT, 
			GID_OPERATOR, 0660, "usb");

	if(cdev)
	{
		DEV2BUS(cdev) = NULL;
	}
	return;
}

SYSINIT(usb_init, SI_SUB_DRIVERS, SI_ORDER_FIRST, usb_init, NULL);

static devclass_t usb_devclass;
static driver_t usb_driver =
{
	.name    = "usb",
	.methods = (device_method_t [])
	{
	  DEVMETHOD(device_probe, usb_probe),
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

