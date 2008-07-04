/*-
 * Copyright (c) 2006 Hans Petter Selasky. All rights reserved.
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
 *
 *
 * usb2_dev.c - An abstraction layer for creating devices under /dev/...
 */

#include <dev/usb2/include/usb2_standard.h>
#include <dev/usb2/include/usb2_ioctl.h>
#include <dev/usb2/include/usb2_defs.h>
#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_error.h>

#define	USB_DEBUG_VAR usb2_fifo_debug

#include <dev/usb2/core/usb2_core.h>
#include <dev/usb2/core/usb2_mbuf.h>
#include <dev/usb2/core/usb2_dev.h>
#include <dev/usb2/core/usb2_process.h>
#include <dev/usb2/core/usb2_device.h>
#include <dev/usb2/core/usb2_debug.h>
#include <dev/usb2/core/usb2_busdma.h>
#include <dev/usb2/core/usb2_generic.h>
#include <dev/usb2/core/usb2_dynamic.h>

#include <dev/usb2/controller/usb2_controller.h>
#include <dev/usb2/controller/usb2_bus.h>

#include <sys/filio.h>
#include <sys/ttycom.h>
#include <sys/syscallsubr.h>

#ifdef USB_DEBUG
static int usb2_fifo_debug = 0;

SYSCTL_NODE(_hw_usb2, OID_AUTO, dev, CTLFLAG_RW, 0, "USB device");
SYSCTL_INT(_hw_usb2_dev, OID_AUTO, debug, CTLFLAG_RW,
    &usb2_fifo_debug, 0, "Debug Level");
#endif

#if ((__FreeBSD_version >= 700001) || (__FreeBSD_version == 0) || \
     ((__FreeBSD_version >= 600034) && (__FreeBSD_version < 700000)))
#define	USB_UCRED struct ucred *ucred,
#else
#define	USB_UCRED
#endif

/* prototypes */

static uint32_t usb2_path_convert_one(const char **pp);
static uint32_t usb2_path_convert(const char *path);
static uint8_t usb2_match_perm(struct usb2_perm *psystem, struct usb2_perm *puser);
static struct usb2_fifo *usb2_fifo_alloc(uint8_t fifo_index);
static int usb2_fifo_open(struct usb2_fifo *f, struct file *fp, struct thread *td, int fflags);
static void usb2_fifo_close(struct usb2_fifo *f, struct thread *td, int fflags);
static void usb2_dev_init(void *arg);
static void usb2_dev_init_post(void *arg);
static void usb2_dev_uninit(void *arg);
static int usb2_fifo_uiomove(struct usb2_fifo *f, void *cp, int n, struct uio *uio);
static void usb2_fifo_wakeup(struct usb2_fifo *f);
static void usb2_fifo_check_methods(struct usb2_fifo_methods *pm);
static void usb2_clone(void *arg, USB_UCRED char *name, int namelen, struct cdev **dev);

static d_fdopen_t usb2_fdopen;
static d_close_t usb2_close;

static fo_rdwr_t usb2_read_f;
static fo_rdwr_t usb2_write_f;
static fo_truncate_t usb2_truncate_f;
static fo_ioctl_t usb2_ioctl_f;
static fo_poll_t usb2_poll_f;
static fo_kqfilter_t usb2_kqfilter_f;
static fo_stat_t usb2_stat_f;
static fo_close_t usb2_close_f;

static usb2_fifo_open_t usb2_fifo_dummy_open;
static usb2_fifo_close_t usb2_fifo_dummy_close;
static usb2_fifo_ioctl_t usb2_fifo_dummy_ioctl;
static usb2_fifo_cmd_t usb2_fifo_dummy_cmd;

static struct usb2_perm usb2_perm = {
	.uid = UID_ROOT,
	.gid = GID_OPERATOR,
	.mode = 0660,
	.active = 1,
};

static struct cdevsw usb2_devsw = {
	.d_version = D_VERSION,
	.d_fdopen = usb2_fdopen,
	.d_close = usb2_close,
	.d_name = "usb",
	.d_flags = D_TRACKCLOSE,
};

static struct fileops usb2_ops_f = {
	.fo_read = usb2_read_f,
	.fo_write = usb2_write_f,
	.fo_truncate = usb2_truncate_f,
	.fo_ioctl = usb2_ioctl_f,
	.fo_poll = usb2_poll_f,
	.fo_kqfilter = usb2_kqfilter_f,
	.fo_stat = usb2_stat_f,
	.fo_close = usb2_close_f,
	.fo_flags = DFLAG_PASSABLE | DFLAG_SEEKABLE
};

static const dev_clone_fn usb2_clone_ptr = &usb2_clone;
static struct cdev *usb2_dev;
static uint32_t usb2_last_devloc = 0 - 1;
static eventhandler_tag usb2_clone_tag;
static void *usb2_old_f_data;
static struct fileops *usb2_old_f_ops;

struct mtx usb2_ref_lock;

static uint32_t
usb2_path_convert_one(const char **pp)
{
	const char *ptr;
	uint32_t temp = 0;

	ptr = *pp;

	while ((*ptr >= '0') && (*ptr <= '9')) {
		temp *= 10;
		temp += (*ptr - '0');
		if (temp >= 1000000) {
			/* catch overflow early */
			return (0 - 1);
		}
		ptr++;
	}

	if (*ptr == '.') {
		/* skip dot */
		ptr++;
	}
	*pp = ptr;

	return (temp);
}

/*------------------------------------------------------------------------*
 *	usb2_path_convert
 *
 * Path format: "/dev/usb<bus>.<dev>.<iface>.<fifo>"
 *
 * Returns: Path converted into numerical format.
 *------------------------------------------------------------------------*/
static uint32_t
usb2_path_convert(const char *path)
{
	uint32_t temp;
	uint32_t devloc;

	devloc = 0;

	temp = usb2_path_convert_one(&path);

	if (temp >= USB_BUS_MAX) {
		return (0 - 1);
	}
	devloc += temp;

	temp = usb2_path_convert_one(&path);

	if (temp >= USB_DEV_MAX) {
		return (0 - 1);
	}
	devloc += (temp * USB_BUS_MAX);

	temp = usb2_path_convert_one(&path);

	if (temp >= USB_IFACE_MAX) {
		return (0 - 1);
	}
	devloc += (temp * USB_DEV_MAX * USB_BUS_MAX);

	temp = usb2_path_convert_one(&path);

	if (temp >= USB_FIFO_MAX) {
		return (0 - 1);
	}
	devloc += (temp * USB_IFACE_MAX * USB_DEV_MAX * USB_BUS_MAX);

	return (devloc);
}

/*------------------------------------------------------------------------*
 *	usb2_set_iface_perm
 *
 * This function will set the interface permissions.
 *------------------------------------------------------------------------*/
void
usb2_set_iface_perm(struct usb2_device *udev, uint8_t iface_index,
    uint32_t uid, uint32_t gid, uint16_t mode)
{
	struct usb2_interface *iface;

	iface = usb2_get_iface(udev, iface_index);
	if (iface && iface->idesc) {
		mtx_lock(&usb2_ref_lock);
		iface->perm.uid = uid;
		iface->perm.gid = gid;
		iface->perm.mode = mode;
		iface->perm.active = 1;
		mtx_unlock(&usb2_ref_lock);

	}
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_match_perm
 *
 * This function will compare two permission structures and see if
 * they are matching.
 *
 * Return values:
 * 0: Permissions are not matching.
 * Else: Permissions are matching.
 *------------------------------------------------------------------------*/
static uint8_t
usb2_match_perm(struct usb2_perm *psystem, struct usb2_perm *puser)
{
	uint16_t mode;

	if (psystem->active && puser->active) {
		/* continue */
	} else {
		return (0);		/* no access */
	}

	/* get the mode differences with regard to the bits that are set */
	mode = ((psystem->mode ^ puser->mode) & puser->mode);

	if ((psystem->uid == puser->uid) && ((mode & 0700) == 0)) {
		return (1);		/* allow access */
	} else if ((psystem->gid == puser->gid) && ((mode & 0070) == 0)) {
		return (1);		/* allow access */
	} else if ((mode & 0007) == 0) {
		return (1);		/* allow access */
	}
	return (0);			/* deny access */
}

/*------------------------------------------------------------------------*
 *	usb2_ref_device
 *
 * This function is used to atomically refer an USB device by its
 * device location. If this function returns success the USB device
 * will not dissappear until the USB device is unreferenced.
 *
 * Return values:
 *  0: Success, refcount incremented on the given USB device.
 *  Else: Failure.
 *------------------------------------------------------------------------*/
usb2_error_t
usb2_ref_device(struct file *fp, struct usb2_location *ploc, uint32_t devloc)
{
	int fflags;

	if (fp) {
		/* get the device location */
		devloc = USB_P2U(fp->f_data);
		fflags = fp->f_flag;
		/* only ref FIFO */
		ploc->is_uref = 0;
	} else {
		/* only ref device */
		fflags = 0;
		ploc->is_uref = 1;
	}

	if (devloc > (USB_BUS_MAX * USB_DEV_MAX *
	    USB_EP_MAX * USB_IFACE_MAX)) {
		/* invalid device location */
		return (USB_ERR_INVAL);
	}
	/* store device location */
	ploc->devloc = devloc;
	ploc->bus_index = devloc % USB_BUS_MAX;
	ploc->dev_index = (devloc / USB_BUS_MAX) % USB_DEV_MAX;
	ploc->iface_index = (devloc / (USB_BUS_MAX *
	    USB_DEV_MAX)) % USB_IFACE_MAX;
	ploc->ep_index = (devloc / (USB_BUS_MAX * USB_DEV_MAX *
	    USB_IFACE_MAX)) % USB_EP_MAX;

	mtx_lock(&usb2_ref_lock);
	ploc->bus = devclass_get_softc(usb2_devclass_ptr, ploc->bus_index);
	if (ploc->bus == NULL) {
		DPRINTF(1, "no bus\n");
		goto error;
	}
	if (ploc->dev_index >= ploc->bus->devices_max) {
		DPRINTF(1, "invalid dev index\n");
		goto error;
	}
	ploc->udev = ploc->bus->devices[ploc->dev_index];
	if (ploc->udev == NULL) {
		DPRINTF(1, "no device\n");
		goto error;
	}
	if (ploc->udev->refcount == USB_DEV_REF_MAX) {
		DPRINTF(1, "no dev ref\n");
		goto error;
	}
	ploc->iface = usb2_get_iface(ploc->udev, ploc->iface_index);
	if (ploc->ep_index != 0) {
		/* non control endpoint - we need an interface */
		if (ploc->iface == NULL) {
			DPRINTF(1, "no iface\n");
			goto error;
		}
		if (ploc->iface->idesc == NULL) {
			DPRINTF(1, "no idesc\n");
			goto error;
		}
	}
	/* check TX FIFO */

	ploc->txfifo = ploc->udev->fifo[(2 * ploc->ep_index) + USB_FIFO_TX];
	if ((ploc->txfifo != NULL) &&
	    (ploc->txfifo->refcount != USB_FIFO_REF_MAX) &&
	    (ploc->txfifo->curr_file == fp)) {
		ploc->is_write = 1;	/* ref */
	} else {
		if (fflags & FWRITE) {
			goto error;
		}
		ploc->is_write = 0;	/* no ref */
		ploc->txfifo = NULL;	/* no access */
	}

	/* check RX FIFO */

	ploc->rxfifo = ploc->udev->fifo[(2 * ploc->ep_index) + USB_FIFO_RX];
	if ((ploc->rxfifo != NULL) &&
	    (ploc->rxfifo->refcount != USB_FIFO_REF_MAX) &&
	    (ploc->rxfifo->curr_file == fp)) {
		ploc->is_read = 1;	/* ref */
	} else {
		if (fflags & FREAD) {
			goto error;
		}
		ploc->is_read = 0;	/* no ref */
		ploc->rxfifo = NULL;	/* no access */
	}

	/* when everything is OK we increment the refcounts */

	if (ploc->is_uref) {
		DPRINTF(1, "ref udev\n");
		ploc->udev->refcount++;
	}
	if (ploc->is_write) {
		DPRINTF(1, "ref write\n");
		ploc->txfifo->refcount++;
	}
	if (ploc->is_read) {
		DPRINTF(1, "ref read\n");
		ploc->rxfifo->refcount++;
	}
	mtx_unlock(&usb2_ref_lock);

	if (ploc->is_uref) {
		/*
		 * We are about to alter the bus-state. Apply the
		 * required locks.
		 */
		sx_xlock(ploc->udev->default_sx + 1);
		mtx_lock(&Giant);	/* XXX */
	}
	return (0);

error:
	mtx_unlock(&usb2_ref_lock);
	DPRINTF(1, "fail\n");
	return (USB_ERR_INVAL);
}

/*------------------------------------------------------------------------*
 *	usb2_unref_device
 *
 * This function will release the reference count by one unit for the
 * given USB device.
 *------------------------------------------------------------------------*/
void
usb2_unref_device(struct usb2_location *ploc)
{
	if (ploc->is_uref) {
		mtx_unlock(&Giant);	/* XXX */
		sx_unlock(ploc->udev->default_sx + 1);
	}
	mtx_lock(&usb2_ref_lock);
	if (ploc->is_read) {
		if (--(ploc->rxfifo->refcount) == 0) {
			cv_signal(&(ploc->rxfifo->cv_drain));
		}
	}
	if (ploc->is_write) {
		if (--(ploc->txfifo->refcount) == 0) {
			cv_signal(&(ploc->txfifo->cv_drain));
		}
	}
	if (ploc->is_uref) {
		if (--(ploc->udev->refcount) == 0) {
			cv_signal(ploc->udev->default_cv + 1);
		}
	}
	mtx_unlock(&usb2_ref_lock);
	return;
}

static struct usb2_fifo *
usb2_fifo_alloc(uint8_t fifo_index)
{
	struct usb2_fifo *f;

	f = malloc(sizeof(*f), M_USBDEV, M_WAITOK | M_ZERO);

	if (f) {
		cv_init(&f->cv_io, "FIFO-IO");
		cv_init(&f->cv_drain, "FIFO-DRAIN");
		f->refcount = 1;
		f->fifo_index = fifo_index;
	}
	return (f);
}

void
usb2_fifo_free(struct usb2_fifo *f)
{
	uint8_t n;

	if (f == NULL) {
		/* be NULL safe */
		return;
	}
	/* destroy symlink devices, if any */
	for (n = 0; n != 2; n++) {
		if (f->symlink[n]) {
			destroy_dev(f->symlink[n]);
			f->symlink[n] = NULL;
		}
	}
	mtx_lock(&usb2_ref_lock);

	/* delink ourselves to stop calls from userland */
	if ((f->fifo_index < USB_FIFO_MAX) &&
	    (f->udev != NULL)) {
		f->udev->fifo[f->fifo_index] = NULL;
	} else {
		DPRINTF(-1, "USB FIFO %p has not been linked!\n", f);
	}

	/* decrease refcount */
	f->refcount--;
	/* prevent any write flush */
	f->flag_iserror = 1;
	/* need to wait until all callers have exited */
	while (f->refcount != 0) {
		mtx_lock(f->priv_mtx);
		/* get I/O thread out of any sleep state */
		if (f->flag_sleeping) {
			f->flag_sleeping = 0;
			cv_broadcast(&f->cv_io);
		}
		mtx_unlock(f->priv_mtx);

		/* wait for sync */
		cv_wait(&f->cv_drain, &usb2_ref_lock);
	}
	mtx_unlock(&usb2_ref_lock);

	/* take care of closing the device here, if any */
	usb2_fifo_close(f, curthread, 0);

	cv_destroy(&f->cv_io);
	cv_destroy(&f->cv_drain);

	free(f, M_USBDEV);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_check
 *------------------------------------------------------------------------*/
static void
usb2_fifo_check(struct usb2_location *ploc, int fflags)
{
	struct usb2_device *udev;
	struct usb2_pipe *pipe;
	struct usb2_fifo *f;
	uint8_t ep_index;
	uint8_t ep_dir;

	udev = ploc->udev;
	ep_index = ploc->ep_index;
	if (fflags & FREAD) {
		f = ploc->rxfifo;
	} else if (fflags & FWRITE) {
		f = ploc->txfifo;
	} else {
		/* nothing to do */
		return;
	}

	if (f) {
		/* nothing do to - we already have a FIFO */
		DPRINTF(1, "has FIFO\n");
		return;
	}
	if (ep_index >= 16) {
		/* nothing to do - these are virtual endpoints */
		DPRINTF(1, "VEP\n");
		return;
	}
	/* automatically create a generic endpoint */
	if (ep_index == 0) {
		pipe = &(udev->default_pipe);
	} else {
		if (fflags & FREAD) {
			if (udev->flags.usb2_mode == USB_MODE_HOST) {
				ep_dir = UE_DIR_IN;
			} else {
				ep_dir = UE_DIR_OUT;
			}
		} else {
			if (udev->flags.usb2_mode == USB_MODE_HOST) {
				ep_dir = UE_DIR_OUT;
			} else {
				ep_dir = UE_DIR_IN;
			}
		}
		pipe = usb2_get_pipe_by_addr(udev, ep_index | ep_dir);
		if (pipe == NULL) {
			/* if the pipe does not exist then return */
			return;
		}
		if (pipe->edesc == NULL) {
			/* invalid pipe */
			return;
		}
		if (pipe->iface_index != ploc->iface_index) {
			/*
			 * Permissions violation - trying to access a
			 * pipe that does not belong to the interface.
			 */
			return;
		}
	}
	if (fflags & FREAD) {
		f = usb2_fifo_alloc((2 * ep_index) + USB_FIFO_RX);
	} else {
		f = usb2_fifo_alloc((2 * ep_index) + USB_FIFO_TX);
	}
	if (f == NULL) {
		/* could not create FIFO */
		return;
	}
	/* update some fields */
	f->priv_mtx = udev->default_mtx;
	f->priv_sc0 = pipe;
	f->unit = 0;
	f->methods = &usb2_ugen_methods;
	f->iface_index = ploc->iface_index;
	f->udev = udev;

	/* check the methods */
	usb2_fifo_check_methods(f->methods);

	/* register our FIFO */
	mtx_lock(&usb2_ref_lock);
	if (fflags & FREAD) {
		udev->fifo[(2 * ep_index) + USB_FIFO_RX] = f;
	} else {
		udev->fifo[(2 * ep_index) + USB_FIFO_TX] = f;
	}
	mtx_unlock(&usb2_ref_lock);

	/* we are done */
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_open
 *
 * Returns:
 * 0: Success
 * Else: Failure
 *------------------------------------------------------------------------*/
static int
usb2_fifo_open(struct usb2_fifo *f, struct file *fp, struct thread *td,
    int fflags)
{
	int err;

	if (f == NULL) {
		/* no FIFO there */
		DPRINTF(1, "no FIFO\n");
		return (ENXIO);
	}
	/* remove FWRITE and FREAD flags */
	fflags &= ~(FWRITE | FREAD);

	/* set correct file flags */
	if ((f->fifo_index & 1) == USB_FIFO_TX) {
		fflags |= FWRITE;
	} else {
		fflags |= FREAD;
	}

	/* check if we are already opened */
	/* we don't need any locks when checking this variable */
	if (f->curr_file) {
		err = EBUSY;
		goto done;
	}
	/* call open method */
	err = (f->methods->f_open) (f, fflags, td);
	if (err) {
		goto done;
	}
	mtx_lock(f->priv_mtx);

	/* reset sleep flag */
	f->flag_sleeping = 0;

	/* reset error flag */
	f->flag_iserror = 0;

	/* reset select flag */
	f->flag_isselect = 0;

	/* reset flushing flag */
	f->flag_flushing = 0;

	/* reset ASYNC proc flag */
	f->async_p = NULL;

	/* set which file we belong to */
	f->curr_file = fp;

	/* reset queue */
	usb2_fifo_reset(f);

	mtx_unlock(f->priv_mtx);
done:
	return (err);
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_reset
 *------------------------------------------------------------------------*/
void
usb2_fifo_reset(struct usb2_fifo *f)
{
	struct usb2_mbuf *m;

	if (f == NULL) {
		return;
	}
	while (1) {
		USB_IF_DEQUEUE(&(f->used_q), m);
		if (m) {
			USB_IF_ENQUEUE(&(f->free_q), m);
		} else {
			break;
		}
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_close
 *------------------------------------------------------------------------*/
static void
usb2_fifo_close(struct usb2_fifo *f, struct thread *td, int fflags)
{
	int err;

	/* check if we are not opened */
	if (!f->curr_file) {
		/* nothing to do - already closed */
		return;
	}
	mtx_lock(f->priv_mtx);

	/* clear current file flag */
	f->curr_file = NULL;

	/* check if we are selected */
	if (f->flag_isselect) {
		selwakeup(&(f->selinfo));
		f->flag_isselect = 0;
	}
	/* check if a thread wants SIGIO */
	if (f->async_p != NULL) {
		PROC_LOCK(f->async_p);
		psignal(f->async_p, SIGIO);
		PROC_UNLOCK(f->async_p);
		f->async_p = NULL;
	}
	/* remove FWRITE and FREAD flags */
	fflags &= ~(FWRITE | FREAD);

	/* flush written data, if any */
	if ((f->fifo_index & 1) == USB_FIFO_TX) {

		if (!f->flag_iserror) {

			/* set flushing flag */
			f->flag_flushing = 1;

			/* start write transfer, if not already started */
			(f->methods->f_start_write) (f);

			/* check if flushed already */
			while (f->flag_flushing &&
			    (!f->flag_iserror)) {
				/* wait until all data has been written */
				f->flag_sleeping = 1;
				err = cv_wait_sig(&(f->cv_io), f->priv_mtx);
				if (err) {
					DPRINTF(0, "signal received\n");
					break;
				}
			}
		}
		fflags |= FWRITE;

		/* stop write transfer, if not already stopped */
		(f->methods->f_stop_write) (f);
	} else {
		fflags |= FREAD;

		/* stop write transfer, if not already stopped */
		(f->methods->f_stop_read) (f);
	}

	/* check if we are sleeping */
	if (f->flag_sleeping) {
		DPRINTF(1, "Sleeping at close!\n");
	}
	mtx_unlock(f->priv_mtx);

	/* call close method */
	(f->methods->f_close) (f, fflags, td);

	DPRINTF(0, "closed\n");
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_fdopen - cdev callback
 *------------------------------------------------------------------------*/
static int
usb2_fdopen(struct cdev *dev, int xxx_oflags, struct thread *td, struct file *fp)
{
	struct usb2_location loc;
	struct usb2_perm perm;
	uint32_t devloc;
	int err;
	int fflags;

	DPRINTF(1, "oflags=0x%08x\n", xxx_oflags);

	bzero(&perm, sizeof(perm));

	devloc = usb2_last_devloc;
	usb2_last_devloc = (0 - 1);	/* reset "usb2_devloc" */

	if (fp == NULL) {
		DPRINTF(1, "fp == NULL\n");
		return (ENXIO);
	}
	if (usb2_old_f_data != fp->f_data) {
		if (usb2_old_f_data != NULL) {
			DPRINTF(-1, "File data mismatch!\n");
			return (ENXIO);
		}
		usb2_old_f_data = fp->f_data;
	}
	if (usb2_old_f_ops != fp->f_ops) {
		if (usb2_old_f_ops != NULL) {
			DPRINTF(-1, "File ops mismatch!\n");
			return (ENXIO);
		}
		usb2_old_f_ops = fp->f_ops;
	}
	fflags = fp->f_flag;
	DPRINTF(1, "fflags=0x%08x\n", fflags);

	if (!(fflags & (FREAD | FWRITE))) {
		/* should not happen */
		return (EPERM);
	}
	if (devloc == (uint32_t)(0 - 1)) {
		/* tried to open /dev/usb */
		DPRINTF(1, "no devloc\n");
		return (ENXIO);
	}
	err = usb2_ref_device(NULL, &loc, devloc);
	if (err) {
		DPRINTF(1, "cannot ref device\n");
		return (ENXIO);
	}
	/* create a permissions mask */
	perm.uid = td->td_ucred->cr_ruid;
	perm.gid = td->td_ucred->cr_rgid;
	perm.mode = 0;
	if (fflags & FREAD)
		perm.mode |= 0444;
	if (fflags & FWRITE)
		perm.mode |= 0222;
	perm.active = 1;

	mtx_lock(loc.udev->default_mtx);

	/* scan down the permissions tree */

	if ((fflags & FWRITE) && (loc.txfifo != NULL) &&
	    (loc.txfifo->iface_index != loc.iface_index)) {
		/* the FIFO does not belong to the specified interface */
		err = EPERM;
	} else if ((fflags & FREAD) && (loc.rxfifo != NULL) &&
	    (loc.rxfifo->iface_index != loc.iface_index)) {
		/* the FIFO does not belong to the specified interface */
		err = EPERM;
	} else if ((loc.ep_index != 0) && loc.iface &&
	    usb2_match_perm(&perm, &loc.iface->perm)) {
		/* we got access through the interface */
		err = 0;
	} else if (loc.udev && usb2_match_perm(&perm, &loc.udev->perm)) {
		/* we got access through the device */
		err = 0;
	} else if (loc.bus && usb2_match_perm(&perm, &loc.bus->perm)) {
		/* we got access through the USB bus */
		err = 0;
	} else if (usb2_match_perm(&perm, &usb2_perm)) {
		/* we got general access */
		err = 0;
	} else {
		/* no access */
		err = EPERM;
	}

	mtx_unlock(loc.udev->default_mtx);

	/* check for error */
	if (err) {
		usb2_unref_device(&loc);
		return (err);
	}
	usb2_fifo_check(&loc, fflags & FREAD);
	usb2_fifo_check(&loc, fflags & FWRITE);
	usb2_unref_device(&loc);

	/* try to refer the device and associated FIFOs again */
	err = usb2_ref_device(NULL, &loc, devloc);
	if (err) {
		return (ENXIO);
	}
	if (fflags & FREAD) {
		err = usb2_fifo_open(loc.rxfifo, fp, td,
		    fflags);
		if (err) {
			DPRINTF(1, "read open failed\n");
			usb2_unref_device(&loc);
			return (err);
		}
	}
	if (fflags & FWRITE) {
		err = usb2_fifo_open(loc.txfifo, fp, td,
		    fflags);
		if (err) {
			DPRINTF(1, "write open failed\n");
			if (fflags & FREAD) {
				usb2_fifo_close(loc.rxfifo, td,
				    fflags);
			}
			usb2_unref_device(&loc);
			return (err);
		}
	}
	/*
	 * Magic: Take over the file so that we get all the callbacks
	 * directly and don't have to create another device:
	 */
	fp->f_ops = &usb2_ops_f;
	fp->f_data = ((uint8_t *)0) + loc.devloc;

	usb2_unref_device(&loc);

	DPRINTF(1, "error=%d\n", err);

	return (err);
}

/*------------------------------------------------------------------------*
 *	usb2_close - cdev callback
 *------------------------------------------------------------------------*/
static int
usb2_close(struct cdev *dev, int flag, int mode, struct thread *p)
{
	return (0);			/* nothing to do */
}

/*------------------------------------------------------------------------*
 *      usb2_clone - cdev callback
 *
 * This function is the kernel clone callback for "/dev/usbX.Y".
 *------------------------------------------------------------------------*/
static void
usb2_clone(void *arg, USB_UCRED char *name, int namelen, struct cdev **dev)
{
	if (*dev) {
		/* someone else has created a device */
		return;
	}
	if (strncmp(name, USB_DEVICE_NAME,
	    sizeof(USB_DEVICE_NAME) - 1)) {
		return;
	}
	if (usb2_last_devloc != (uint32_t)(0 - 1)) {
		/*
		 * XXX can we assume that the clone and open operation is
		 * atomic ?
		 */
		DPRINTF(1, "Clone race!\n");
	}
	usb2_last_devloc = usb2_path_convert(name +
	    sizeof(USB_DEVICE_NAME) - 1);

	if (usb2_last_devloc == (uint32_t)(0 - 1)) {
		/* invalid location */
		return;
	}
	dev_ref(usb2_dev);
	*dev = usb2_dev;
	return;
}

static void
usb2_dev_init(void *arg)
{
	mtx_init(&usb2_ref_lock, "USB ref mutex", NULL, MTX_DEF);
	return;
}

SYSINIT(usb2_dev_init, SI_SUB_KLD, SI_ORDER_FIRST, usb2_dev_init, NULL);

static void
usb2_dev_init_post(void *arg)
{
	/* create a dummy device so that we are visible */
	usb2_dev = make_dev(&usb2_devsw, 0, UID_ROOT, GID_OPERATOR,
	    0000, USB_DEVICE_NAME " ");
	if (usb2_dev == NULL) {
		DPRINTF(-1, "Could not create usb bus device!\n");
	}
	usb2_clone_tag = EVENTHANDLER_REGISTER(dev_clone, usb2_clone_ptr, NULL, 1000);
	if (usb2_clone_tag == NULL) {
		DPRINTF(-1, "Registering clone handler failed!\n");
	}
	return;
}

SYSINIT(usb2_dev_init_post, SI_SUB_KICK_SCHEDULER, SI_ORDER_FIRST, usb2_dev_init_post, NULL);

static void
usb2_dev_uninit(void *arg)
{
	if (usb2_clone_tag) {
		EVENTHANDLER_DEREGISTER(dev_clone, usb2_clone_tag);
		usb2_clone_tag = NULL;
	}
	if (usb2_dev) {
		destroy_dev(usb2_dev);
		usb2_dev = NULL;
	}
	mtx_destroy(&usb2_ref_lock);
	return;
}

SYSUNINIT(usb2_dev_uninit, SI_SUB_KICK_SCHEDULER, SI_ORDER_ANY, usb2_dev_uninit, NULL);

static int
usb2_close_f(struct file *fp, struct thread *td)
{
	struct usb2_location loc;
	int fflags;
	int err;

	fflags = fp->f_flag;

	DPRINTF(1, "fflags=%u\n", fflags);

	err = usb2_ref_device(fp, &loc, 0);;

	/* restore some file variables */
	fp->f_ops = usb2_old_f_ops;
	fp->f_data = usb2_old_f_data;

	/* check for error */
	if (err) {
		DPRINTF(1, "could not ref\n");
		goto done;
	}
	if (fflags & FREAD) {
		usb2_fifo_close(loc.rxfifo, td, fflags);
	}
	if (fflags & FWRITE) {
		usb2_fifo_close(loc.txfifo, td, fflags);
	}
	usb2_unref_device(&loc);

done:
	/* call old close method */
	return (fp->f_ops->fo_close) (fp, td);
}

static int
usb2_ioctl_f_sub(struct usb2_fifo *f, u_long cmd, void *addr, struct thread *td)
{
	int error = 0;

	switch (cmd) {
	case FIONBIO:
		/* handled by upper FS layer */
		break;

	case FIOASYNC:
		if (*(int *)addr) {
			if (f->async_p != NULL) {
				error = EBUSY;
				break;
			}
#if defined(__NetBSD__)
			f->async_p = td;
#else
			f->async_p = td->td_proc;
#endif
		} else {
			f->async_p = NULL;
		}
		break;

		/* XXX this is not the most general solution */
	case TIOCSPGRP:
		if (f->async_p == NULL) {
			error = EINVAL;
			break;
		}
		if (*(int *)addr != f->async_p->p_pgid) {
			error = EPERM;
			break;
		}
		break;
	default:
		return (ENOTTY);
	}
	return (error);
}

static int
usb2_ioctl_f(struct file *fp, u_long cmd, void *addr,
    struct ucred *cred, struct thread *td)
{
	struct usb2_location loc;
	int fflags;
	int err_rx;
	int err_tx;
	int err;
	uint8_t is_common = 0;

	err = usb2_ref_device(fp, &loc, 0);;
	if (err) {
		return (ENXIO);
	}
	fflags = fp->f_flag;

	DPRINTF(1, "fflags=%u, cmd=0x%lx\n", fflags, cmd);

	if (fflags & FREAD) {
		if (fflags & FWRITE) {
			/*
			 * Automagically figure out if we have an IOCTL that
			 * should not be replicated to both FIFOs:
			 */
			if ((loc.rxfifo->priv_sc0 ==
			    loc.txfifo->priv_sc0) &&
			    (loc.rxfifo->priv_sc1 ==
			    loc.txfifo->priv_sc1) &&
			    (loc.rxfifo->methods ==
			    loc.txfifo->methods)) {
				is_common = 1;
			}
		}
		err_rx = usb2_ioctl_f_sub(loc.rxfifo, cmd, addr, td);
		if (err_rx == ENOTTY) {
			err_rx = (loc.rxfifo->methods->f_ioctl) (
			    loc.rxfifo, cmd, addr,
			    is_common ? fflags : (fflags & ~FWRITE), td);
		}
	} else {
		err_rx = 0;
	}
	if (fflags & FWRITE) {
		err_tx = usb2_ioctl_f_sub(loc.txfifo, cmd, addr, td);
		if (err_tx == ENOTTY) {
			if (is_common)
				err_tx = 0;	/* already handled this IOCTL */
			else
				err_tx = (loc.txfifo->methods->f_ioctl) (
				    loc.txfifo, cmd, addr, fflags & ~FREAD, td);
		}
	} else {
		err_tx = 0;
	}

	if (err_rx) {
		err = err_rx;
	} else if (err_tx) {
		err = err_tx;
	} else {
		err = 0;		/* no error */
	}
	usb2_unref_device(&loc);
	return (err);
}

/* ARGSUSED */
static int
usb2_kqfilter_f(struct file *fp, struct knote *kn)
{
	return (ENXIO);
}


/* ARGSUSED */
static int
usb2_poll_f(struct file *fp, int events, struct ucred *cred, struct thread *td)
{
	struct usb2_location loc;
	struct usb2_fifo *f;
	struct usb2_mbuf *m;
	int fflags;
	int revents;

	revents = usb2_ref_device(fp, &loc, 0);;
	if (revents) {
		return (POLLHUP);
	}
	fflags = fp->f_flag;
	f = loc.txfifo;

	if ((events & (POLLOUT | POLLWRNORM)) &&
	    (fflags & FWRITE)) {

		mtx_lock(f->priv_mtx);

		/* check if any packets are available */
		USB_IF_POLL(&(f->free_q), m);

		if (f->flag_iserror || m) {
			revents |= events & (POLLOUT | POLLWRNORM);
		} else {
			f->flag_isselect = 1;
			selrecord(td, &(f->selinfo));
		}

		mtx_unlock(f->priv_mtx);
	}
	f = loc.rxfifo;

	if ((events & (POLLIN | POLLRDNORM)) &&
	    (fflags & FREAD)) {

		mtx_lock(f->priv_mtx);

		/* check if any packets are available */
		USB_IF_POLL(&(f->used_q), m);

		if (f->flag_iserror || m) {
			revents |= events & (POLLIN | POLLRDNORM);
		} else {
			f->flag_isselect = 1;
			selrecord(td, &(f->selinfo));

			/* start reading data */
			(f->methods->f_start_read) (f);
		}

		mtx_unlock(f->priv_mtx);
	}
	usb2_unref_device(&loc);
	return (revents);
}

/* ARGSUSED */
static int
usb2_read_f(struct file *fp, struct uio *uio, struct ucred *cred, int flags, struct thread *td)
{
	struct usb2_location loc;
	struct usb2_fifo *f;
	struct usb2_mbuf *m;
	int fflags;
	int resid;
	int io_len;
	int err;
	uint8_t tr_data = 0;

	DPRINTF(1, "\n");

	fflags = fp->f_flag & (O_NONBLOCK | O_DIRECT | FREAD | FWRITE);
	if (fflags & O_DIRECT)
		fflags |= IO_DIRECT;

	err = usb2_ref_device(fp, &loc, 0);
	if (err) {
		return (ENXIO);
	}
	f = loc.rxfifo;
	if (f == NULL) {
		/* should not happen */
		return (EPERM);
	}
	resid = uio->uio_resid;

	if ((flags & FOF_OFFSET) == 0)
		uio->uio_offset = fp->f_offset;

	mtx_lock(f->priv_mtx);

	if (f->flag_iserror) {
		err = EIO;
		goto done;
	}
	/* XXX TODO: support IO-vectors */

	while (uio->uio_resid > 0) {

		USB_IF_DEQUEUE(&(f->used_q), m);

		if (m == NULL) {

			/* start read transfer, if not already started */

			(f->methods->f_start_read) (f);

			if (fflags & O_NONBLOCK) {
				if (tr_data) {
					/* return length before error */
					break;
				}
				err = EWOULDBLOCK;
				break;
			}
			err = usb2_fifo_wait(f);
			if (err) {
				break;
			}
			continue;
		} else {
			tr_data = 1;
		}

		io_len = min(m->cur_data_len, uio->uio_resid);

		DPRINTF(1, "transfer %d bytes from %p\n",
		    io_len, m->cur_data_ptr);

		err = usb2_fifo_uiomove(f,
		    m->cur_data_ptr, io_len, uio);

		m->cur_data_len -= io_len;
		m->cur_data_ptr += io_len;

		if (m->cur_data_len == 0) {

			uint8_t last_packet;

			last_packet = m->last_packet;

			USB_IF_ENQUEUE(&(f->free_q), m);

			if (last_packet) {
				/* keep framing */
				break;
			}
		} else {
			USB_IF_PREPEND(&(f->used_q), m);
		}

		if (err) {
			break;
		}
	}
done:
	mtx_unlock(f->priv_mtx);

	usb2_unref_device(&loc);

	if ((flags & FOF_OFFSET) == 0)
		fp->f_offset = uio->uio_offset;
	fp->f_nextoff = uio->uio_offset;
	return (err);
}

static int
usb2_stat_f(struct file *fp, struct stat *sb, struct ucred *cred, struct thread *td)
{
	return (vnops.fo_stat(fp, sb, cred, td));
}

static int
usb2_truncate_f(struct file *fp, off_t length, struct ucred *cred, struct thread *td)
{
	return (vnops.fo_truncate(fp, length, cred, td));
}

/* ARGSUSED */
static int
usb2_write_f(struct file *fp, struct uio *uio, struct ucred *cred, int flags, struct thread *td)
{
	struct usb2_location loc;
	struct usb2_fifo *f;
	struct usb2_mbuf *m;
	int fflags;
	int resid;
	int io_len;
	int err;
	uint8_t tr_data = 0;

	DPRINTF(1, "\n");

	fflags = fp->f_flag & (O_NONBLOCK | O_DIRECT |
	    FREAD | FWRITE | O_FSYNC);
	if (fflags & O_DIRECT)
		fflags |= IO_DIRECT;

	err = usb2_ref_device(fp, &loc, 0);
	if (err) {
		return (ENXIO);
	}
	f = loc.txfifo;
	if (f == NULL) {
		/* should not happen */
		usb2_unref_device(&loc);
		return (EPERM);
	}
	resid = uio->uio_resid;

	if ((flags & FOF_OFFSET) == 0)
		uio->uio_offset = fp->f_offset;

	mtx_lock(f->priv_mtx);

	if (f->flag_iserror) {
		err = EIO;
		goto done;
	}
	/* XXX TODO: support IO-vectors */

	while (uio->uio_resid > 0) {

		USB_IF_DEQUEUE(&(f->free_q), m);

		if (m == NULL) {

			if (fflags & O_NONBLOCK) {
				if (tr_data) {
					/* return length before error */
					break;
				}
				err = EWOULDBLOCK;
				break;
			}
			err = usb2_fifo_wait(f);
			if (err) {
				break;
			}
			continue;
		} else {
			tr_data = 1;
		}

		USB_MBUF_RESET(m);

		io_len = min(m->cur_data_len, uio->uio_resid);

		m->cur_data_len = io_len;

		DPRINTF(1, "transfer %d bytes to %p\n",
		    io_len, m->cur_data_ptr);

		err = usb2_fifo_uiomove(f,
		    m->cur_data_ptr, io_len, uio);

		if (err) {
			USB_IF_ENQUEUE(&(f->free_q), m);
			break;
		} else {
			USB_IF_ENQUEUE(&(f->used_q), m);
			(f->methods->f_start_write) (f);
		}
	}
done:
	mtx_unlock(f->priv_mtx);

	usb2_unref_device(&loc);

	if ((flags & FOF_OFFSET) == 0)
		fp->f_offset = uio->uio_offset;
	fp->f_nextoff = uio->uio_offset;

	return (err);
}

static int
usb2_fifo_uiomove(struct usb2_fifo *f, void *cp,
    int n, struct uio *uio)
{
	int error;

	mtx_unlock(f->priv_mtx);

	/*
	 * "uiomove()" can sleep so one needs to make a wrapper,
	 * exiting the mutex and checking things:
	 */
	error = uiomove(cp, n, uio);

	mtx_lock(f->priv_mtx);

	return (error);
}

int
usb2_fifo_wait(struct usb2_fifo *f)
{
	int err;

	mtx_assert(f->priv_mtx, MA_OWNED);

	if (f->flag_iserror) {
		/* we are gone */
		return (EIO);
	}
	f->flag_sleeping = 1;

	err = cv_wait_sig(&(f->cv_io), f->priv_mtx);

	if (f->flag_iserror) {
		/* we are gone */
		err = EIO;
	}
	return (err);
}

void
usb2_fifo_signal(struct usb2_fifo *f)
{
	if (f->flag_sleeping) {
		f->flag_sleeping = 0;
		cv_broadcast(&(f->cv_io));
	}
	return;
}

static void
usb2_fifo_wakeup(struct usb2_fifo *f)
{
	usb2_fifo_signal(f);

	if (f->flag_isselect) {
		selwakeup(&(f->selinfo));
		f->flag_isselect = 0;
	}
	if (f->async_p != NULL) {
		PROC_LOCK(f->async_p);
		psignal(f->async_p, SIGIO);
		PROC_UNLOCK(f->async_p);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_opened
 *
 * Returns:
 * 0: FIFO not opened.
 * Else: FIFO is opened.
 *------------------------------------------------------------------------*/
uint8_t
usb2_fifo_opened(struct usb2_fifo *f)
{
	uint8_t temp;
	uint8_t do_unlock;

	if (f == NULL) {
		return (0);		/* be NULL safe */
	}
	if (mtx_owned(f->priv_mtx)) {
		do_unlock = 0;
	} else {
		do_unlock = 1;
		mtx_lock(f->priv_mtx);
	}
	temp = f->curr_file ? 1 : 0;
	if (do_unlock) {
		mtx_unlock(f->priv_mtx);
	}
	return (temp);
}


static int
usb2_fifo_dummy_open(struct usb2_fifo *fifo,
    int fflags, struct thread *td)
{
	return (0);
}

static void
usb2_fifo_dummy_close(struct usb2_fifo *fifo,
    int fflags, struct thread *td)
{
	return;
}

static int
usb2_fifo_dummy_ioctl(struct usb2_fifo *fifo, u_long cmd, void *addr,
    int fflags, struct thread *td)
{
	return (ENOTTY);
}

static void
usb2_fifo_dummy_cmd(struct usb2_fifo *fifo)
{
	fifo->flag_flushing = 0;	/* not flushing */
	return;
}

static void
usb2_fifo_check_methods(struct usb2_fifo_methods *pm)
{
	/* check that all callback functions are OK */

	if (pm->f_open == NULL)
		pm->f_open = &usb2_fifo_dummy_open;

	if (pm->f_close == NULL)
		pm->f_close = &usb2_fifo_dummy_close;

	if (pm->f_ioctl == NULL)
		pm->f_ioctl = &usb2_fifo_dummy_ioctl;

	if (pm->f_start_read == NULL)
		pm->f_start_read = &usb2_fifo_dummy_cmd;

	if (pm->f_stop_read == NULL)
		pm->f_stop_read = &usb2_fifo_dummy_cmd;

	if (pm->f_start_write == NULL)
		pm->f_start_write = &usb2_fifo_dummy_cmd;

	if (pm->f_stop_write == NULL)
		pm->f_stop_write = &usb2_fifo_dummy_cmd;

	return;
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_attach
 *
 * The following function will create a duplex FIFO.
 *
 * Return values:
 * 0: Success.
 * Else: Failure.
 *------------------------------------------------------------------------*/
int
usb2_fifo_attach(struct usb2_device *udev, void *priv_sc,
    struct mtx *priv_mtx, struct usb2_fifo_methods *pm,
    struct usb2_fifo_sc *f_sc, uint16_t unit, uint16_t subunit,
    uint8_t iface_index)
{
	struct usb2_fifo *f_tx;
	struct usb2_fifo *f_rx;
	char buf[32];
	char src[32];
	uint8_t n;

	f_sc->fp[USB_FIFO_TX] = NULL;
	f_sc->fp[USB_FIFO_RX] = NULL;

	if (pm == NULL)
		return (EINVAL);

	if (priv_mtx == NULL)
		priv_mtx = &Giant;

	/* search for a free FIFO slot */

	for (n = USB_EP_MAX;; n += 2) {

		if (n == USB_FIFO_MAX) {
			return (ENOMEM);
		}
		if ((udev->fifo[n + USB_FIFO_TX] == NULL) &&
		    (udev->fifo[n + USB_FIFO_RX] == NULL)) {
			break;
		}
	}

	f_tx = usb2_fifo_alloc(n + USB_FIFO_TX);	/* write FIFO */
	f_rx = usb2_fifo_alloc(n + USB_FIFO_RX);	/* read FIFO */

	if ((f_tx == NULL) || (f_rx == NULL)) {
		usb2_fifo_free(f_tx);
		usb2_fifo_free(f_rx);
		return (ENOMEM);
	}
	/* initialise FIFO structures */

	f_tx->priv_mtx = priv_mtx;
	f_tx->priv_sc0 = priv_sc;
	f_tx->unit = unit;
	f_tx->iface_index = iface_index;
	f_tx->methods = pm;
	f_tx->udev = udev;

	f_rx->priv_mtx = priv_mtx;
	f_rx->priv_sc0 = priv_sc;
	f_rx->unit = unit;
	f_rx->iface_index = iface_index;
	f_rx->methods = pm;
	f_rx->udev = udev;

	/* check the methods */
	usb2_fifo_check_methods(pm);

	f_sc->fp[USB_FIFO_TX] = f_tx;
	f_sc->fp[USB_FIFO_RX] = f_rx;

	mtx_lock(&usb2_ref_lock);
	udev->fifo[n + USB_FIFO_TX] = f_tx;
	udev->fifo[n + USB_FIFO_RX] = f_rx;
	mtx_unlock(&usb2_ref_lock);

	if (snprintf(src, sizeof(src),
	    USB_DEVICE_NAME "%u.%u.%u.%u",
	    device_get_unit(udev->bus->bdev),
	    udev->device_index,
	    iface_index,
	    n / 2)) {
		/* ignore */
	}
	for (n = 0; n != 4; n++) {

		if (pm->basename[n] == NULL) {
			continue;
		}
		if (subunit == 0xFFFF) {
			if (snprintf(buf, sizeof(buf),
			    "%s%u%s", pm->basename[n],
			    unit, pm->postfix[n] ?
			    pm->postfix[n] : "")) {
				/* ignore */
			}
		} else {
			if (snprintf(buf, sizeof(buf),
			    "%s%u.%u%s", pm->basename[n],
			    unit, subunit, pm->postfix[n] ?
			    pm->postfix[n] : "")) {
				/* ignore */
			}
		}

		/*
		 * Distribute the symbolic links into two FIFO structures:
		 */
		if (n & 1) {
			f_rx->symlink[n / 2] =
			    make_dev_symlink(src, "%s", buf);
		} else {
			f_tx->symlink[n / 2] =
			    make_dev_symlink(src, "%s", buf);
		}
		DPRINTF(0, "Symlink: %s -> %s\n", buf, src);
	}

	DPRINTF(1, "attached %p/%p\n", f_tx, f_rx);
	return (0);
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_alloc_buffer
 *
 * Return values:
 * 0: Success
 * Else failure
 *------------------------------------------------------------------------*/
int
usb2_fifo_alloc_buffer(struct usb2_fifo *f, uint32_t bufsize,
    uint16_t nbuf)
{
	usb2_fifo_free_buffer(f);

	/* allocate an endpoint */
	f->free_q.ifq_maxlen = nbuf;
	f->used_q.ifq_maxlen = nbuf;

	f->queue_data = usb2_alloc_mbufs(
	    M_USBDEV, &(f->free_q), bufsize, nbuf);

	if ((f->queue_data == NULL) && bufsize && nbuf) {
		return (ENOMEM);
	}
	return (0);			/* success */
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_free_buffer
 *
 * This function will free the buffers associated with a FIFO. This
 * function can be called multiple times in a row.
 *------------------------------------------------------------------------*/
void
usb2_fifo_free_buffer(struct usb2_fifo *f)
{
	if (f->queue_data) {
		/* free old buffer */
		free(f->queue_data, M_USBDEV);
		f->queue_data = NULL;
	}
	/* reset queues */

	bzero(&(f->free_q), sizeof(f->free_q));
	bzero(&(f->used_q), sizeof(f->used_q));
	return;
}

void
usb2_fifo_detach(struct usb2_fifo_sc *f_sc)
{
	if (f_sc == NULL) {
		return;
	}
	usb2_fifo_free(f_sc->fp[USB_FIFO_TX]);
	usb2_fifo_free(f_sc->fp[USB_FIFO_RX]);

	f_sc->fp[USB_FIFO_TX] = NULL;
	f_sc->fp[USB_FIFO_RX] = NULL;

	DPRINTF(1, "detached %p\n", f_sc);

	return;
}

uint32_t
usb2_fifo_put_bytes_max(struct usb2_fifo *f)
{
	struct usb2_mbuf *m;
	uint32_t len;

	USB_IF_POLL(&(f->free_q), m);

	if (m) {
		len = m->max_data_len;
	} else {
		len = 0;
	}
	return (len);
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_put_data
 *
 * what:
 *  0 - normal operation
 *  1 - set last packet flag to enforce framing
 *------------------------------------------------------------------------*/
void
usb2_fifo_put_data(struct usb2_fifo *f, struct usb2_page_cache *pc,
    uint32_t offset, uint32_t len, uint8_t what)
{
	struct usb2_mbuf *m;
	uint32_t io_len;

	while (len || (what == 1)) {

		USB_IF_DEQUEUE(&(f->free_q), m);

		if (m) {
			USB_MBUF_RESET(m);

			io_len = min(len, m->cur_data_len);

			usb2_copy_out(pc, offset, m->cur_data_ptr, io_len);

			m->cur_data_len = io_len;
			offset += io_len;
			len -= io_len;

			if ((len == 0) && (what == 1)) {
				m->last_packet = 1;
			}
			USB_IF_ENQUEUE(&(f->used_q), m);

			usb2_fifo_wakeup(f);

			if ((len == 0) || (what == 1)) {
				break;
			}
		} else {
			break;
		}
	}
	return;
}

void
usb2_fifo_put_data_linear(struct usb2_fifo *f, void *ptr,
    uint32_t len, uint8_t what)
{
	struct usb2_mbuf *m;
	uint32_t io_len;

	while (len || (what == 1)) {

		USB_IF_DEQUEUE(&(f->free_q), m);

		if (m) {
			USB_MBUF_RESET(m);

			io_len = min(len, m->cur_data_len);

			bcopy(ptr, m->cur_data_ptr, io_len);

			m->cur_data_len = io_len;
			ptr = USB_ADD_BYTES(ptr, io_len);
			len -= io_len;

			if ((len == 0) && (what == 1)) {
				m->last_packet = 1;
			}
			USB_IF_ENQUEUE(&(f->used_q), m);

			usb2_fifo_wakeup(f);

			if ((len == 0) || (what == 1)) {
				break;
			}
		} else {
			break;
		}
	}
	return;
}

void
usb2_fifo_put_data_error(struct usb2_fifo *f)
{
	f->flag_iserror = 1;
	usb2_fifo_wakeup(f);
	return;
}

/*------------------------------------------------------------------------*
 *	usb2_fifo_get_data
 *
 * what:
 *  0 - normal operation
 *  1 - only get one "usb2_mbuf"
 *
 * returns:
 *  0 - no more data
 *  1 - data in buffer
 *------------------------------------------------------------------------*/
uint8_t
usb2_fifo_get_data(struct usb2_fifo *f, struct usb2_page_cache *pc,
    uint32_t offset, uint32_t len, uint32_t *actlen,
    uint8_t what)
{
	struct usb2_mbuf *m;
	uint32_t io_len;
	uint8_t tr_data = 0;

	actlen[0] = 0;

	while (1) {

		USB_IF_DEQUEUE(&(f->used_q), m);

		if (m) {

			tr_data = 1;

			io_len = min(len, m->cur_data_len);

			usb2_copy_in(pc, offset, m->cur_data_ptr, io_len);

			len -= io_len;
			offset += io_len;
			actlen[0] += io_len;
			m->cur_data_ptr += io_len;
			m->cur_data_len -= io_len;

			if ((m->cur_data_len == 0) || (what == 1)) {
				USB_IF_ENQUEUE(&(f->free_q), m);

				usb2_fifo_wakeup(f);

				if (what == 1) {
					break;
				}
			} else {
				USB_IF_PREPEND(&(f->used_q), m);
			}
		} else {

			if (tr_data) {
				/* wait for data to be written out */
				break;
			}
			if (f->flag_flushing) {
				f->flag_flushing = 0;
				usb2_fifo_wakeup(f);
			}
			break;
		}
		if (len == 0) {
			break;
		}
	}
	return (tr_data);
}

uint8_t
usb2_fifo_get_data_linear(struct usb2_fifo *f, void *ptr,
    uint32_t len, uint32_t *actlen, uint8_t what)
{
	struct usb2_mbuf *m;
	uint32_t io_len;
	uint8_t tr_data = 0;

	actlen[0] = 0;

	while (1) {

		USB_IF_DEQUEUE(&(f->used_q), m);

		if (m) {

			tr_data = 1;

			io_len = min(len, m->cur_data_len);

			bcopy(m->cur_data_ptr, ptr, io_len);

			len -= io_len;
			ptr = USB_ADD_BYTES(ptr, io_len);
			actlen[0] += io_len;
			m->cur_data_ptr += io_len;
			m->cur_data_len -= io_len;

			if ((m->cur_data_len == 0) || (what == 1)) {
				USB_IF_ENQUEUE(&(f->free_q), m);

				usb2_fifo_wakeup(f);

				if (what == 1) {
					break;
				}
			} else {
				USB_IF_PREPEND(&(f->used_q), m);
			}
		} else {

			if (tr_data) {
				/* wait for data to be written out */
				break;
			}
			if (f->flag_flushing) {
				f->flag_flushing = 0;
				usb2_fifo_wakeup(f);
			}
			break;
		}
		if (len == 0) {
			break;
		}
	}
	return (tr_data);
}

void
usb2_fifo_get_data_error(struct usb2_fifo *f)
{
	f->flag_iserror = 1;
	usb2_fifo_wakeup(f);
	return;
}