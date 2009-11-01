/*-
 * Copyright (c) 1999-2002 Poul-Henning Kamp
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
 *
 * This file comes from "FreeBSD/src/sys/kern/kern_conf.c"
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/lock.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>
#include <sys/conf.h>
#include <sys/vnode.h>
#include <sys/queue.h>
#include <sys/poll.h>
#include <sys/tty.h>
#include <sys/ucred.h>
#include <machine/stdarg.h>

#include <sys/freebsd_compat.h>

/* Built at compile time from sys/conf/majors */

static struct mtx devmtx;

/* Define a dead_cdevsw for use when devices leave unexpectedly. */

static struct __cdevsw dead_cdevsw = {
	.d_version = D_VERSION,
	.d_name = "dead",
};

static void destroy_devl(struct cdev *dev);
static void dev_check_cdevsw(struct __cdevsw *);

static void
dev_lock_init(void *arg)
{
	mtx_init(&devmtx, "cdev", NULL, MTX_DEF | MTX_RECURSE);
}

SYSINIT(dev_lock_init, SI_SUB_DONE, SI_ORDER_ANY, dev_lock_init, NULL);

void
dev_lock(void)
{
	mtx_lock(&devmtx);
}

void
dev_unlock(void)
{
	mtx_unlock(&devmtx);
}

void
dev_ref(struct cdev *dev)
{
	dev_lock();
	dev->si_refcount++;
	dev_unlock();
}

void
dev_rel(struct cdev *dev)
{
	int flag = 0;
	int i;

	mtx_assert(&devmtx, MA_NOTOWNED);

	dev_lock();
	dev->si_refcount--;
	flag = (dev->si_refcount == 0);
	dev_unlock();

	if (flag) {
		i = dev->si_inode;
		dev->si_inode = 0;
		if (i != 0)
			*devfs_itod(i) = NULL;
		free(dev, M_DEVBUF);
	}
}

struct __cdevsw *
dev_ref_cdevsw(struct cdev *dev)
{
	struct __cdevsw *csw;

	dev_lock();
	csw = dev->si_devsw;
	if (csw != NULL)
		dev_ref(dev);
	dev_unlock();
	return (csw);
}

static int
__nullop(void)
{
	return (0);
}

static int
__enodev(void)
{
	return (ENODEV);
}

static void
no_strategy(struct bio *bp)
{
}


/* Default methods if driver does not specify method */

#define	null_open	(d_open_t *)&__nullop
#define	null_close	(d_close_t *)&__nullop
#define	no_dump		(dumper_t *)&__enodev
#define	no_read		(d_read_t *)&__enodev
#define	no_write	(d_write_t *)&__enodev
#define	no_ioctl	(d_ioctl_t *)&__enodev
#define	no_mmap		(d_mmap_t *)&__enodev
#define	no_kqfilter	(d_kqfilter_t *)&__enodev

static int
no_poll(struct cdev *dev __unused, int events, struct thread *td __unused)
{
	/*
	 * Return true for read/write.  If the user asked for something
	 * special, return POLLNVAL, so that clients have a way of
	 * determining reliably whether or not the extended
	 * functionality is present without hard-coding knowledge
	 * of specific filesystem implementations.
	 * Stay in sync with vop_nopoll().
	 */
	if (events & ~POLLSTANDARD)
		return (POLLNVAL);

	return (events & (POLLIN | POLLOUT | POLLRDNORM | POLLWRNORM));
}

/*
 * struct cdev * and u_dev_t primitives
 */

int
minor(struct cdev *x)
{
	if (x == NULL)
		return NODEV;
	return (x->si_drv0 & MAXMINOR);
}

int
dev2unit(struct cdev *x)
{
	if (x == NULL)
		return NODEV;
	return (minor2unit(minor(x)));
}

u_int
minor2unit(u_int _minor)
{

	__KASSERT((_minor & ~MAXMINOR) == 0, ("Illegal minor %x", _minor));
	return ((_minor & 0xff) | ((_minor >> 8) & 0xffff00));
}

int
unit2minor(int unit)
{

	__KASSERT(unit <= 0xffffff, ("Invalid unit (%d) in unit2minor", unit));
	return ((unit & 0xff) | ((unit << 8) & ~0xffff));
}

static struct cdev *
allocdev(struct __cdevsw *csw, int minornr)
{
	struct cdev *si;

	dev_lock();
	dev_check_cdevsw(csw);
	dev_unlock();

	si = malloc(sizeof(*si), M_DEVBUF, M_ZERO | M_WAITOK);
	if (si == NULL)
		return (NULL);

	si->si_name = si->__si_namebuf;
	si->si_drv0 = (dev_t)minornr;
	si->si_devsw = csw;
	si->si_refcount = 1;
	LIST_INIT(&si->si_children);
	LIST_INIT(&si->si_alist);

	return (si);
}

int
uminor(dev_t dev)
{
	return (dev & MAXMINOR);
}

int
umajor(dev_t dev)
{
	return ((dev & ~MAXMINOR) >> 8);
}

static void
dev_check_cdevsw(struct __cdevsw *devsw)
{
	if (devsw->d_open == NULL)
		devsw->d_open = null_open;
	if (devsw->d_close == NULL)
		devsw->d_close = null_close;
	if (devsw->d_read == NULL)
		devsw->d_read = no_read;
	if (devsw->d_write == NULL)
		devsw->d_write = no_write;
	if (devsw->d_ioctl == NULL)
		devsw->d_ioctl = no_ioctl;
	if (devsw->d_poll == NULL)
		devsw->d_poll = no_poll;
	if (devsw->d_mmap == NULL)
		devsw->d_mmap = no_mmap;
	if (devsw->d_strategy == NULL)
		devsw->d_strategy = no_strategy;
	if (devsw->d_kqfilter == NULL)
		devsw->d_kqfilter = no_kqfilter;
}

struct cdev *
make_dev(struct __cdevsw *devsw, int minornr, uid_t uid, gid_t gid, int mode,
    const char *fmt,...)
{
	struct cdev *dev;
	va_list ap;
	int i;

	__KASSERT((minornr & ~MAXMINOR) == 0,
	    ("Invalid minor (0x%x) in make_dev", minornr));

	dev = allocdev(devsw, minornr);

	dev_lock();

	va_start(ap, fmt);
#if 0
	i = vsnrprintf(dev->__si_namebuf, sizeof dev->__si_namebuf, 32, fmt, ap);
#else
	i = vsnprintf(dev->__si_namebuf, sizeof(dev->__si_namebuf), fmt, ap);
#endif
	va_end(ap);

	if (i > (sizeof(dev->__si_namebuf) - 1)) {
		printf("WARNING: Device name truncated! (%s)\n",
		    dev->__si_namebuf);
	}
	dev->si_flags |= SI_NAMED;
	dev->si_uid = uid;
	dev->si_gid = gid;
	dev->si_mode = mode;

	devfs_create(dev);

	dev_unlock();

	return (dev);
}

static void
dev_dependsl(struct cdev *pdev, struct cdev *cdev)
{
	cdev->si_parent = pdev;
	cdev->si_flags |= SI_CHILD;
	LIST_INSERT_HEAD(&pdev->si_children, cdev, si_siblings);
}

struct cdev *
make_dev_alias(struct cdev *pdev, const char *fmt,...)
{
	struct cdev *dev;
	va_list ap;
	int i = 0;

	dev = allocdev(&dead_cdevsw, 0);
	if (dev == NULL)
		return (NULL);

	dev_lock();

	dev->si_flags |= SI_ALIAS | SI_NAMED;

	va_start(ap, fmt);
#if 0
	i = vsnrprintf(dev->__si_namebuf, sizeof dev->__si_namebuf, 32, fmt, ap);
#else
	i = vsnprintf(dev->__si_namebuf, sizeof(dev->__si_namebuf), fmt, ap);
#endif
	va_end(ap);

	if (i > (sizeof(dev->__si_namebuf) - 1)) {
		printf("WARNING: Device name truncated! (%s)\n",
		    dev->__si_namebuf);
	}
	dev_dependsl(pdev, dev);

	devfs_create(dev);

	dev_unlock();

	return (dev);
}

static void
destroy_devl(struct cdev *dev)
{
	struct __cdevsw *csw;

	mtx_assert(&devmtx, MA_OWNED);

	__KASSERT(dev->si_flags & SI_NAMED,
	    ("WARNING: Driver mistake: destroy_dev on %d\n", minor(dev)));

	devfs_destroy(dev);

	/* If we are a child, remove us from the parents list */
	if (dev->si_flags & SI_CHILD) {
		LIST_REMOVE(dev, si_siblings);
	}
	/* Clear some flags */
	dev->si_flags &= ~(SI_NAMED | SI_ALIAS | SI_CHILD);

	csw = dev->si_devsw;
	dev->si_devsw = NULL;

	/* Kill any children devices */
	while (!LIST_EMPTY(&dev->si_children))
		destroy_devl(LIST_FIRST(&dev->si_children));

	dev_unlock();

	dev_rel(dev);

	dev_lock();
}

void
destroy_dev(struct cdev *dev)
{
	dev_lock();
	destroy_devl(dev);
	dev_unlock();
}

const char *
devtoname(struct cdev *dev)
{
	struct __cdevsw *csw;
	char *p;
	int mynor;

	if (dev == NULL) {
		return "unknown";
	}
	if (dev->si_name[0] == '#' || dev->si_name[0] == '\0') {
		p = dev->si_name;
		csw = dev_ref_cdevsw(dev);
		if (csw != NULL) {
			sprintf(p, "(%s)", csw->d_name);
			dev_rel(dev);
		}
		p += strlen(p);
		mynor = minor(dev);
		if (mynor < 0 || mynor > 255)
			sprintf(p, "/%#x", (u_int)mynor);
		else
			sprintf(p, "/%d", mynor);
	}
	return (dev->si_name);
}

int
dev_stdclone(char *name, char **namep, const char *stem, int *unit)
{
	int u, i;

	i = strlen(stem);
	if (bcmp(stem, name, i) != 0)
		return (0);
	if (!isdigit(name[i]))
		return (0);
	u = 0;
	if (name[i] == '0' && isdigit(name[i + 1]))
		return (0);
	while (isdigit(name[i])) {
		u *= 10;
		u += name[i++] - '0';
	}
	if (u > 0xffffff)
		return (0);
	*unit = u;
	if (namep)
		*namep = &name[i];
	if (name[i])
		return (2);
	return (1);
}

void
termioschars(struct termios *t)
{
	bcopy(&ttydefchars, t->c_cc, sizeof(t->c_cc));
}
