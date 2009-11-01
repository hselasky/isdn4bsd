/*-
 * Copyright (c) 2000,2004
 *	Poul-Henning Kamp.  All rights reserved.
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
 * This is a lite version of "FreeBSD/src/sys/fs/devfs/devfs_devs.c"
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/dirent.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/vnode.h>

#include <sys/freebsd_compat.h>

static struct cdev *devfs_inot[NDEVFSINO];

static int devfs_inode;
static unsigned int devfs_generation;

static struct devfs_dirent *devfs_find(struct devfs_dirent *dd, const char *name, int namelen);

void
devfs_timestamp(struct timespec *ts)
{
	struct timeval tv;

	microtime(&tv);
	TIMEVAL_TO_TIMESPEC(&tv, ts);
}

struct devfs_dirent **
devfs_itode(struct devfs_mount *dm, int inode)
{
	if (inode < 0)
		return (NULL);
	if (inode < NDEVFSINO)
		return (&dm->dm_dirent[inode]);
	return (NULL);
}

struct cdev **
devfs_itod(int inode)
{
	if (inode < 0)
		return (NULL);
	if (inode < NDEVFSINO)
		return (&devfs_inot[inode]);
	return (NULL);
}

static struct devfs_dirent *
devfs_find(struct devfs_dirent *dd, const char *name, int namelen)
{
	struct devfs_dirent *de;

	TAILQ_FOREACH(de, &dd->de_dlist, de_list) {
		if (namelen != de->de_dirent->d_namlen)
			continue;
		if (bcmp(name, de->de_dirent->d_name, namelen) != 0)
			continue;
		break;
	}
	return (de);
}

struct devfs_dirent *
devfs_newdirent(char *name, int namelen)
{
	int i;
	struct devfs_dirent *de;

#if (__NetBSD_Version__ < 400000000)
#define	_DIRENT_RECLEN(pde, namlen) \
	(sizeof(*(pde)) - sizeof((pde)->d_name) + (((namlen) + 1 + 3) &~ 3))
#endif
	i = sizeof(*de) + _DIRENT_RECLEN(de->de_dirent, namelen);
	MALLOC(de, struct devfs_dirent *, i, M_DEVFS, M_WAITOK | M_ZERO);
	de->de_dirent = (struct dirent *)(de + 1);
	de->de_dirent->d_namlen = namelen;
	de->de_dirent->d_reclen = _DIRENT_RECLEN(de->de_dirent, namelen);
	bcopy(name, de->de_dirent->d_name, namelen);
	de->de_dirent->d_name[namelen] = '\0';
	devfs_timestamp(&de->de_ctime);
	de->de_mtime = de->de_atime = de->de_ctime;
	de->de_links = 1;
	return (de);
}

#undef DECONST
#define	DECONST(arg) ((void *)(((const char *)arg) - ((const char *)0)))

struct devfs_dirent *
devfs_vmkdir(char *name, int namelen, struct devfs_dirent *dotdot)
{
	struct devfs_dirent *dd;
	struct devfs_dirent *de;

	dd = devfs_newdirent(name, namelen);

	TAILQ_INIT(&dd->de_dlist);

	dd->de_dirent->d_type = DT_DIR;
	dd->de_mode = 0555;
	dd->de_links = 2;
	dd->de_dir = dd;

	de = devfs_newdirent(DECONST("."), 1);
	de->de_dirent->d_type = DT_DIR;
	de->de_dir = dd;
	de->de_flags |= DE_DOT;
	TAILQ_INSERT_TAIL(&dd->de_dlist, de, de_list);

	de = devfs_newdirent(DECONST(".."), 2);
	de->de_dirent->d_type = DT_DIR;
	if (dotdot == NULL)
		de->de_dir = dd;
	else
		de->de_dir = dotdot;
	de->de_flags |= DE_DOTDOT;
	TAILQ_INSERT_TAIL(&dd->de_dlist, de, de_list);

	return (dd);
}

static void
devfs_delete(struct devfs_dirent *dd, struct devfs_dirent *de)
{

	if (de->de_symlink) {
		FREE(de->de_symlink, M_DEVFS);
		de->de_symlink = NULL;
	}
	if (de->de_vnode)
		de->de_vnode->v_data = NULL;

	TAILQ_REMOVE(&dd->de_dlist, de, de_list);

	FREE(de, M_DEVFS);
}

void
devfs_purge(struct devfs_dirent *dd)
{
	struct devfs_dirent *de;

	if (dd == NULL)
		return;

	for (;;) {
		de = TAILQ_FIRST(&dd->de_dlist);
		if (de == NULL)
			break;
		devfs_delete(dd, de);
	}

	FREE(dd, M_DEVFS);
}

int
devfs_populate(struct devfs_mount *dm)
{
	struct cdev *dev;
	struct cdev *pdev;
	struct __cdevsw *csw;
	struct devfs_dirent *dd;
	struct devfs_dirent *de;
	struct devfs_dirent **dep;
	char *q;
	char *s;
	int i;
	int j;

	if (dm->dm_generation == devfs_generation)
		return (0);
	dm->dm_generation = devfs_generation;

	for (i = 0; i != NDEVFSINO; i++) {
		dev = *devfs_itod(i);
		if (dev == NULL)
			continue;
		csw = dev_ref_cdevsw(dev);
		dep = devfs_itode(dm, i);
		de = *dep;

		if (csw == NULL) {
			/* device is gone */
			if (de != NULL) {
				dd = de->de_dir;
				*dep = NULL;
				devfs_delete(dd, de);
				dev_rel(dev);
			}
			continue;
		}
		if (de != NULL) {
			/* already have a directory entry */
			dev_rel(dev);
			continue;
		}
		dd = dm->dm_basedir;
		s = dev->si_name;
		for (;;) {
			for (q = s; *q != '/' && *q != '\0'; q++)
				continue;
			if (*q != '/')
				break;
			de = devfs_find(dd, s, q - s);
			if (de == NULL) {
				de = devfs_vmkdir(s, q - s, dd);
				de->de_inode = dm->dm_inode++;
				TAILQ_INSERT_TAIL(&dd->de_dlist, de, de_list);
				dd->de_links++;
			}
			s = q + 1;
			dd = de;
		}
		de = devfs_newdirent(s, q - s);
		if (dev->si_flags & SI_ALIAS) {
			de->de_inode = dm->dm_inode++;
			de->de_uid = 0;
			de->de_gid = 0;
			de->de_mode = 0755;
			de->de_dirent->d_type = DT_LNK;
			pdev = dev->si_parent;
			j = strlen(pdev->si_name) + 1;
			MALLOC(de->de_symlink, char *, j, M_DEVFS, M_WAITOK);
			bcopy(pdev->si_name, de->de_symlink, j);
		} else {
			de->de_inode = i;
			de->de_uid = dev->si_uid;
			de->de_gid = dev->si_gid;
			de->de_mode = dev->si_mode;
			de->de_dirent->d_type = DT_FIFO;
		}
		*dep = de;

		de->de_dir = dd;

		TAILQ_INSERT_TAIL(&dd->de_dlist, de, de_list);
	}

	return (0);
}

/*
 * devfs_create() and devfs_destroy() are called from kern_conf.c and
 * in both cases the devlock() mutex is held, so no further locking
 * is necesary and no sleeping allowed.
 */

void
devfs_create(struct cdev *dev)
{
	int i;
	int j = NDEVFSINO;

	i = devfs_inode;

	while (j--) {

		if (i < 3 || i >= NDEVFSINO)
			i = 3;

		if (devfs_inot[i] == NULL) {
			dev->si_inode = i;
			devfs_inot[i] = dev;
			devfs_generation++;
			break;
		}
		i++;
	}

	devfs_inode = i;
}

void
devfs_destroy(struct cdev *dev)
{
	devfs_generation++;
}
