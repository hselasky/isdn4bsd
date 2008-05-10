/*-
 * Copyright (c) 2005
 *      Hans Petter Selasky. All rights reserved.
 * Copyright (c) 2000-2004
 *	Poul-Henning Kamp.  All rights reserved.
 * Copyright (c) 1989, 1992-1993, 1995
 *	The Regents of the University of California.  All rights reserved.
 * Copyright (c) 
 *       Jan-Simon Pendry. All rights reserved.
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
 * This file is a lite version of "FreeBSD/src/sys/fs/devfs/devfs_vnops.c"
 *
 * On NetBSD the locking rules are defined in: /usr/src/sys/kern/vnode_if.src
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
#include <sys/fcntl.h>

#include <sys/freebsd_compat.h>

typedef int (__vnodeop_t)(void *);

static __vnodeop_t **devfs_vnodeop_p;

#if (__NetBSD_Version__ >= 400000000)
#define a_p a_l->l_proc
#define cn_proc cn_lwp->l_proc
#define BSD_VOP_ACCESS(a,b,c,d0,d1) \
  VOP_ACCESS(a,b,c,d1)
#else
#define BSD_VOP_ACCESS(a,b,c,d0,d1) \
  VOP_ACCESS(a,b,c,d0)
#endif

/*
 * helper functions
 */
int
devfs_allocv(struct devfs_dirent *de, struct mount *mp, 
	     struct vnode **vpp, struct proc *td)
{
	struct vnode *vp;
	struct cdev *dev;
	int error;

	__KASSERT(td == curthread, ("%s: td != curthread", __FUNCTION__));

loop:
	vp = de->de_vnode;
	if(vp != NULL)
	{
	    if(vget(vp, LK_EXCLUSIVE))
	    {
	        goto loop;
	    }
	    vpp[0] = vp;
	    return 0;
	}

	if(de->de_dirent->d_type == DT_CHR)
	{
	    dev = devfs_itod(de->de_inode)[0];
	    if(dev == NULL)
	    {
	        return ENOENT;
	    }
	}
	else
	{
	    dev = NULL;
	}

	error = getnewvnode(VT_NON, mp, devfs_vnodeop_p, &vp);
	if(error)
	{
	    printf("%s: ERROR: %d: failed to allocate new vnode!\n",
		   __FUNCTION__, error);
	    return error;
	}

	if(de->de_dirent->d_type == DT_CHR)
	{
	    vp->v_type = VCHR;
	    VI_LOCK(vp);
	    dev_lock();
	    dev_refl(dev);
	    de->de_dev = dev; /* XXX lock */
	    LIST_INSERT_HEAD(&dev->si_alist, de, de_alias);
	    dev->si_usecount += vp->v_usecount;
	    dev_unlock();
	    VI_UNLOCK(vp);
#if 0
	    vp->v_op = &devfs_specops;
#endif
	}
	else if(de->de_dirent->d_type == DT_DIR)
	{
	    vp->v_type = VDIR;
	}
	else if(de->de_dirent->d_type == DT_LNK)
	{
	    vp->v_type = VLNK;
	}
	else
	{
	    vp->v_type = VBAD;
	}

	vp->v_data = de;
	de->de_vnode = vp;
	vn_lock(vp, LK_EXCLUSIVE | LK_RETRY);

	vpp[0] = vp;
	return 0;
}

static int
devfs_fp_check(struct vnode *vp, struct cdev **devp, struct __cdevsw **dswp)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct devfs_dirent *de = vp->v_data;

	if(de == NULL)
	{
	    return ENXIO;
	}

	devp[0] = de->de_dev;

	if(devp[0] == NULL)
	{
	    return ENXIO;
	}

	dswp[0] = dev_refthread(devp[0]);
	if(dswp[0] == NULL)
	{
	    return ENXIO;
	}

	return (0);
}

static u_int32_t
devfs_random(void)
{
	/* make sure that people don't make assumptions about device
	 * major/minor numbers in userspace
	 */
	return (boottime.tv_sec & 0x0f0f);
}

dev_t
dev2udev(struct cdev *dev)
{
	if(dev == NULL)
	{
	    return (NODEV);
	}
	return (dev->si_inode ^ devfs_random());
}

/*
 * vnode operations
 */

#ifndef DFLTPHYS
#define DFLTPHYS        (64 * 1024)     /* default max raw I/O transfer size */
#endif

static int
devfs_open(struct vop_open_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	struct proc *td = ap->a_p;
	struct __cdevsw *dsw;
	struct cdev *dev;
	int error;

	if(vp->v_type == VDIR)
	{
	    return 0;
	}

	if(vp->v_type != VCHR)
	{
	    return ENXIO;
        }

	error = devfs_fp_check(vp, &dev, &dsw);
	if(error)
	{
	    goto done;
	}

	/* make this field valid before any I/O in d_open */
	if(dev->si_iosize_max == 0)
	{
	    dev->si_iosize_max = DFLTPHYS;
	}

	/* XXX: Special casing of ttys for deadfs.  Probably redundant. */
	if(dsw->d_flags & D_TTY)
	{
	    vp->v_flag |= VISTTY;
	}

	dev->si_file_flags &= ~(O_NONBLOCK | O_DIRECT);

	VOP_UNLOCK(vp, 0);

	if(dsw->d_flags & D_NEEDGIANT)
	{
	    mtx_lock(&Giant);
	}

	if(dsw->d_fdopen)
	{
#if 0
	    error = dsw->d_fdopen(dev, ap->a_mode, td, ap->a_fdidx);
#else
	    printf("%s: d_fdopen is not supported!\n",
		   __FUNCTION__);

	    error = dsw->d_open(dev, ap->a_mode, S_IFCHR, td);
#endif
	}
	else
	{
	    error = dsw->d_open(dev, ap->a_mode, S_IFCHR, td);
	}

	if(dsw->d_flags & D_NEEDGIANT)
	{
	    mtx_unlock(&Giant);
	}

	vn_lock(vp, LK_EXCLUSIVE | LK_RETRY);

	dev_relthread(dev);

	if(error)
	{
	    goto done;
	}

 done:
	return (error);
}

static int
devfs_read(struct vop_read_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	int error;

	if(ap->a_vp->v_type == VDIR)
	{
	    error = VOP_READDIR(ap->a_vp, ap->a_uio, ap->a_cred, 
				NULL, NULL, NULL);
	}
	else if(ap->a_vp->v_type == VCHR)
	{
	    struct __cdevsw *dsw;
	    struct cdev *dev;
	    struct uio *uio;
	    int ioflag;
	    int resid;

	    error = devfs_fp_check(ap->a_vp, &dev, &dsw);
	    if(error)
	    {
	        goto done;
	    }

	    uio = ap->a_uio;
	    resid = uio->uio_resid;

	    ioflag = dev->si_file_flags & (O_NONBLOCK | O_DIRECT);
	    if(ioflag & O_DIRECT)
	    {
	        ioflag |= IO_DIRECT;
	    }

	    if(ap->a_ioflag & FNONBLOCK)
	    {
	        ioflag |= O_NONBLOCK;
	    }

	    VOP_UNLOCK(ap->a_vp, 0);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_lock(&Giant);
	    }

	    error = dsw->d_read(dev, uio, ioflag);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_unlock(&Giant);
	    }

	    vn_lock(ap->a_vp, LK_EXCLUSIVE | LK_RETRY);

	    dev_relthread(dev);

	    if((uio->uio_resid != resid) || 
	       ((error == 0) && (resid != 0)))
	    {
	        devfs_timestamp(&dev->si_atime);
	    }
	}
	else
	{
	    error = ENXIO;
	}

 done:
	return error;
}

static int
devfs_write(struct vop_write_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	int error;

	if(ap->a_vp->v_type == VCHR)
	{
	    struct uio *uio;
	    struct cdev *dev;
	    struct __cdevsw *dsw;
	    int ioflag;
	    int resid;

	    error = devfs_fp_check(ap->a_vp, &dev, &dsw);
	    if(error)
	    {
	        goto done;
	    }

	    uio = ap->a_uio;
	    resid = uio->uio_resid;

	    ioflag = dev->si_file_flags & (O_NONBLOCK | O_DIRECT);
	    if(ioflag & O_DIRECT)
	    {
	        ioflag |= IO_DIRECT;
	    }

	    if(ap->a_ioflag & FNONBLOCK)
	    {
	        ioflag |= O_NONBLOCK;
	    }

	    VOP_UNLOCK(ap->a_vp, 0);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_lock(&Giant);
	    }

	    error = dsw->d_write(dev, uio, ioflag);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_unlock(&Giant);
	    }

 	    vn_lock(ap->a_vp, LK_EXCLUSIVE | LK_RETRY);

	    dev_relthread(dev);

	    if((uio->uio_resid != resid) || 
	       ((error == 0) && (resid != 0)))
	    {
		devfs_timestamp(&dev->si_ctime);
		dev->si_mtime = dev->si_ctime;
	    }
	}
	else
	{
	    error = ENXIO;
	}
 done:
	return error;
}

static int
devfs_ioctl(struct vop_ioctl_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	int error;

	if(ap->a_vp->v_type == VCHR)
	{
	    struct cdev *dev;
	    struct __cdevsw *dsw;
	    u_int32_t cmd = ap->a_command;
	    void *data = ap->a_data;

	    error = devfs_fp_check(ap->a_vp, &dev, &dsw);
	    if(error)
	    {
	        goto done;
	    }

	    if(cmd == FIONBIO)
	    {
	        if(*(int *)data)
		{
		    dev->si_file_flags |= O_NONBLOCK;
		}
		else
		{
		    dev->si_file_flags &= ~O_NONBLOCK;
		}

		/* this IOCTL is also forwarded to the driver */
	    }
	    else if(cmd == FIODTYPE)
	    {
	        *(int *)data = (dsw->d_flags & D_TYPEMASK);
		dev_relthread(dev);
		error = 0;
		goto done;
	    }
	    else if(cmd == FIODGNAME)
	    {
	        struct fiodgname_arg *fgn = data;
	        u_int8_t buf[128];
		error = snprintf(&buf[0], sizeof(buf), "%s", devtoname(dev));
		dev_relthread(dev);

		if((error < 0) || 
		   (error >= (sizeof(buf)-1)))
		{
		    error = EINVAL;
		    goto done;
		}

		error++;
		if(error > fgn->len)
		{
		    error = EINVAL;
		    goto done;
		}

		error = copyout(&buf[0], fgn->buf, error);
		goto done;
	    }

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_lock(&Giant);
	    }

	    error = dsw->d_ioctl(dev, cmd, data, ap->a_fflag, ap->a_p);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_unlock(&Giant);
	    }

	    dev_relthread(dev);
#if 0
	    if(error == ENOIOCTL)
	    {
	        error = ENOTTY;
	    }
#endif
	}
	else
	{
	    error = ENXIO;
	}
 done:
	return error;
}

static int
devfs_poll(struct vop_poll_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	int error;

	if(ap->a_vp->v_type == VCHR)
	{
	    struct cdev *dev;
	    struct __cdevsw *dsw;

	    error = devfs_fp_check(ap->a_vp, &dev, &dsw);
	    if(error)
	    {
	        goto done;
	    }

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_lock(&Giant);
	    }

	    error = dsw->d_poll(dev, ap->a_events, ap->a_p);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_unlock(&Giant);
  	    }

	    dev_relthread(dev);
	}
	else
	{
	    error = ENXIO;
	}
 done:
	return error;
}

static int
devfs_kqfilter(struct vop_kqfilter_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	int error;

	if(ap->a_vp->v_type == VCHR)
	{
	    struct cdev *dev;
	    struct __cdevsw *dsw;

	    error = devfs_fp_check(ap->a_vp, &dev, &dsw);
	    if(error)
	    {
	        goto done;
	    }

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_lock(&Giant);
	    }

	    error = dsw->d_kqfilter(dev, ap->a_kn);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_unlock(&Giant);
	    }

	    dev_relthread(dev);
	}
	else
	{
	    error = ENXIO;
	}
 done:
	return error;
}

static int
devfs_close(struct vop_close_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	int error;

	if(ap->a_vp->v_type == VCHR)
	{
	    struct cdev *dev;
	    struct __cdevsw *dsw;

	    error = devfs_fp_check(ap->a_vp, &dev, &dsw);
	    if(error)
	    {
	        goto done;
	    }

	    VOP_UNLOCK(ap->a_vp, 0);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_lock(&Giant);
	    }

	    error = dsw->d_close(dev, ap->a_fflag, S_IFCHR, ap->a_p);

	    if(dsw->d_flags & D_NEEDGIANT)
	    {
	        mtx_unlock(&Giant);
	    }

	    vn_lock(ap->a_vp, LK_EXCLUSIVE | LK_RETRY);

#ifdef DEVFS_DEBUG
	    if(count_dev(dev) > 1)
	    {
	        printf("%s: %s: WARNING: device, %s, is still referenced!\n",
		       __FILE__, __FUNCTION__, devtoname(dev));
	    }
#endif

	    dev_relthread(dev);
	}
	else
	{
	    error = ENXIO;
	}
 done:
	return error;
}

static int
devfs_access(struct vop_access_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	struct devfs_dirent *de;
	int error;

	de = vp->v_data;
	if(vp->v_type == VDIR)
	{
	    de = de->de_dir;
	}

	error = vaccess
	  (vp->v_type, de->de_mode, de->de_uid, de->de_gid,
	   ap->a_mode, ap->a_cred);

	if(error == 0) 
	{
	    goto done;
	}

	if(error != EACCES)
	{
	    goto done;
	}

	/* We do, however, allow access to the controlling terminal */
	if(!(ap->a_p->p_flag & P_CONTROLT))
	{
	    goto done;
	}

	if(ap->a_p->p_session->s_ttyvp == de->de_vnode)
	{
	    error = 0;
	}
 done:
	return (error);
}

static void
fix_time(struct timespec *tv)
{
	if(tv->tv_sec == 0)
	{
	    tv->tv_sec = boottime.tv_sec;
	    tv->tv_nsec = boottime.tv_usec * 1000;
	}
	return;
}

static int
devfs_getattr(struct vop_getattr_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	struct vattr *vap = ap->a_vap;
	struct devfs_dirent *de = vp->v_data;
	struct cdev *dev;

	if(vp->v_type == VDIR)
	{
	    de = de->de_dir;
	}

	bzero(vap, sizeof(*vap));

	vattr_null(vap);
	vap->va_uid = de->de_uid;
	vap->va_gid = de->de_gid;
	vap->va_mode = de->de_mode;

	if(vp->v_type == VLNK)
	    vap->va_size = strlen(de->de_symlink);
	else if(vp->v_type == VDIR)
	    vap->va_size = vap->va_bytes = DEV_BSIZE;
	else
	    vap->va_size = 0;

	if(vp->v_type != VDIR)
	    vap->va_bytes = 0;

	vap->va_blocksize = DEV_BSIZE;
	vap->va_type = vp->v_type;

	if(vp->v_type != VCHR)
	{
		fix_time(&de->de_atime);
		fix_time(&de->de_mtime);
		fix_time(&de->de_ctime);

		vap->va_atime = de->de_atime;
		vap->va_mtime = de->de_mtime;
		vap->va_ctime = de->de_ctime;
	}
	else
	{
		dev = de->de_dev;

		fix_time(&dev->si_atime);
		fix_time(&dev->si_mtime);
		fix_time(&dev->si_ctime);

		vap->va_atime = dev->si_atime;
		vap->va_mtime = dev->si_mtime;
		vap->va_ctime = dev->si_ctime;

		vap->va_rdev = dev->si_inode ^ devfs_random();
	}

	fix_time(&vap->va_birthtime);

	vap->va_gen = 0;
	vap->va_flags = 0;
	vap->va_nlink = de->de_links;
	vap->va_fileid = de->de_inode;
#if (__NetBSD_Version__ >= 300000000)
	vap->va_fsid = vp->v_mount->mnt_stat.f_fsidx.__fsid_val[0];
#else
	vap->va_fsid = vp->v_mount->mnt_stat.f_fsid.val[0];
#endif
	return 0;
}

static int
devfs_setattr(struct vop_setattr_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct devfs_dirent *de;
	struct cdev *dev;
	struct vattr *vap;
	struct vnode *vp;
	uid_t uid;
	gid_t gid;
	int error;
	u_int8_t c;

	vap = ap->a_vap;
	vp = ap->a_vp;
	if((vap->va_type != VNON) ||
	   (vap->va_nlink != VNOVAL) ||
	   (vap->va_fsid != VNOVAL) ||
	   (vap->va_fileid != VNOVAL) ||
	   (vap->va_blocksize != VNOVAL) ||
	   ((vap->va_flags != VNOVAL) && (vap->va_flags != 0)) ||
	   (vap->va_rdev != VNOVAL) ||
#if 0
	   (vap->va_size != VNOVAL) ||
#endif
	   (vap->va_bytes != VNOVAL) ||
	   (vap->va_gen != VNOVAL))
	{
	    error = EINVAL;
	    goto done;
	}

	de = vp->v_data;
	dev = de->de_dev;

	if(vp->v_type == VDIR)
	{
	    de = de->de_dir;
	}

	error = 0;
	c = 0;

	if(vap->va_uid == (uid_t)VNOVAL)
	    uid = de->de_uid;
	else
	    uid = vap->va_uid;

	if(vap->va_gid == (gid_t)VNOVAL)
	    gid = de->de_gid;
	else
	    gid = vap->va_gid;

	if((uid != de->de_uid) || 
	   (gid != de->de_gid))
	{
	    if(((kauth_cred_getuid(ap->a_cred) != de->de_uid) || 
		(uid != de->de_uid) ||
		((gid != de->de_gid) && !groupmember(gid, ap->a_cred))) &&
	       (error = suser(ap->a_p)))
	    {
	        goto done;
	    }
	    de->de_uid = uid;
	    de->de_gid = gid;
	    c = 1;
	}

	if(vap->va_mode != (mode_t)VNOVAL)
	{
	    if((kauth_cred_getuid(ap->a_cred) != de->de_uid) &&
	       (error = suser(ap->a_p)))
	    {
	        goto done;
	    }
	    de->de_mode = vap->va_mode;
	    c = 1;
	}

	if((vap->va_atime.tv_sec != VNOVAL) || 
	   (vap->va_mtime.tv_sec != VNOVAL))
	{
	    if(((vap->va_vaflags & VA_UTIMES_NULL) == 0) &&
	       (error = BSD_VOP_ACCESS(vp, VWRITE, ap->a_cred, ap->a_p, ap->a_l)))
	    {
	        goto done;
	    }

	    if(vap->va_atime.tv_sec != VNOVAL)
	    {
	        if(vp->v_type == VCHR)
		    dev->si_atime = vap->va_atime;
		else
		    de->de_atime = vap->va_atime;
	    }

	    if(vap->va_mtime.tv_sec != VNOVAL)
	    {
	        if(vp->v_type == VCHR)
		  dev->si_mtime = vap->va_mtime;
		else
		  de->de_mtime = vap->va_mtime;
	    }
	    c = 1;
	}

	if(c)
	{
	    if(vp->v_type == VCHR)
	        devfs_timestamp(&dev->si_ctime);
	    else
	        devfs_timestamp(&de->de_mtime);
	}

 done:
	return error;
}

/*
 * Construct the fully qualified path name relative to the mountpoint
 */
static char *
devfs_fqpn(char *buf, int len, struct vnode *dvp, struct componentname *cnp)
{
	struct devfs_dirent *de, *dd;
	struct devfs_mount *dmp;

	dmp = VFSTODEVFS(dvp->v_mount);
	dd = dvp->v_data;
	len--;
	buf[len] = '\0';
	len -= cnp->cn_namelen;
	if(len < 0)
	{
	    return (NULL);
	}

	bcopy(cnp->cn_nameptr, buf + len, cnp->cn_namelen);
	de = dd;
	while(de != dmp->dm_basedir)
	{
	    len--;
	    if(len < 0)
	    {
	        return (NULL);
	    }

	    buf[len] = '/';
	    len -= de->de_dirent->d_namlen;
	    if(len < 0)
	    {
	        return (NULL);
	    }

	    bcopy(de->de_dirent->d_name, buf + len,
		  de->de_dirent->d_namlen);
	    de = TAILQ_FIRST(&de->de_dlist);	/* "." */
	    de = TAILQ_NEXT(de, de_list);	/* ".." */
	    de = de->de_dir;
	}
	return (buf + len);
}

#undef DECONST
#define DECONST(arg) ((void *)(((const char *)arg) - ((const char *)0)))

static int
__devfs_lookup(struct vop_lookup_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct componentname *cnp = ap->a_cnp;
	struct vnode **vpp = ap->a_vpp;
	struct vnode *dvp = ap->a_dvp;
	struct devfs_dirent **dde;
	struct devfs_dirent *de;
	struct devfs_dirent *dd;
	struct devfs_mount *dmp;
	struct cdev *cdev;
	struct proc *td = cnp->cn_proc;
	int error;
	int flags = cnp->cn_flags;
	int nameiop = cnp->cn_nameiop;
	u_int8_t specname[SPECNAMELEN + 1];
	u_int8_t *pname;
	u_int16_t name_len;
#if (__NetBSD_Version__ < 400000000)
	u_int8_t unlock = (!(flags & ISLASTCN)) || (!(flags & LOCKPARENT));
#endif

	pname = DECONST(cnp->cn_nameptr);
	dmp = VFSTODEVFS(dvp->v_mount);
	dd = dvp->v_data;
	vpp[0] = NULLVP;

	if((flags & ISLASTCN) && (nameiop == RENAME))
	{
	    error = EOPNOTSUPP;
	    goto done;
	}

	if(dvp->v_type != VDIR)
	{
	    error = ENOTDIR;
	    goto done;
	}

	if((flags & ISDOTDOT) && (dvp->v_flag & VROOT))
	{
	    error = EIO;
	    goto done;
	}

	error = BSD_VOP_ACCESS(dvp, VEXEC, cnp->cn_cred, td, cnp->cn_lwp);
	if(error)
	{
	    goto done;
	}

	if((cnp->cn_namelen == 1) && (pname[0] == '.'))
	{
	    if((flags & ISLASTCN) && (nameiop != LOOKUP))
	    {
	        error = EINVAL;
		goto done;
	    }

	    vpp[0] = dvp;
	    VREF(dvp);
#if (__NetBSD_Version__ < 400000000)
	    unlock = 0;
#endif
	    goto done;
	}

	if(flags & ISDOTDOT)
	{
	    if((flags & ISLASTCN) && (nameiop != LOOKUP))
	    {
	        error = EINVAL;
		goto done;
	    }

	    VOP_UNLOCK(dvp, 0);

            de = TAILQ_FIRST(&dd->de_dlist);    /* "." */
            de = TAILQ_NEXT(de, de_list);       /* ".." */
	    de = de->de_dir;
	    error = devfs_allocv(de, dvp->v_mount, vpp, td);

#if (__NetBSD_Version__ < 400000000)
	    if((error == 0) && unlock)
	    {
	        unlock = 0;
		cnp->cn_flags |= PDIRUNLOCK;
		goto done;
	    }
#endif
	    vn_lock(dvp, LK_EXCLUSIVE | LK_RETRY);
	    goto done;
	}

	devfs_populate(dmp);
	dd = dvp->v_data;
	TAILQ_FOREACH(de, &dd->de_dlist, de_list)
	{
	    if(cnp->cn_namelen != de->de_dirent->d_namlen)
	    {
	        continue;
	    }
	    if(bcmp(cnp->cn_nameptr, de->de_dirent->d_name,
		     de->de_dirent->d_namlen))
	    {
	        continue;
	    }
	    if(de->de_flags & DE_WHITEOUT)
	    {
	        goto notfound;
	    }
	    goto found;
	}

	if(nameiop == DELETE)
	{
	    goto notfound;
	}

	/*
	 * OK, we didn't have an entry for the name we were asked for
	 * so we try to see if anybody can create it on demand.
	 */
	pname = devfs_fqpn(specname, sizeof(specname), dvp, cnp);
	if(pname == NULL)
	{
	    goto notfound;
	}

	cdev = NULL;
	name_len = strlen(pname);
#if (__NetBSD_Version__ < 400000000)
	EVENTHANDLER_INVOKE(dev_clone, td->p_ucred, pname, 
			    name_len, &cdev);
#else
	EVENTHANDLER_INVOKE(dev_clone, NULL /* XXX not supported */, pname, 
			    name_len, &cdev);
#endif
	if(cdev == NULL)
	{
	    goto notfound;
	}

	devfs_populate(dmp);

	dde = devfs_itode(dmp, cdev->si_inode);
	dev_rel(cdev);

	if((dde == NULL) || 
	   (dde[0] == NULL) ||
	   (dde[0] == DE_DELETED))
	{
	    goto notfound;
	}

	if((dde[0])->de_flags & DE_WHITEOUT)
	{
	    goto notfound;
	}

	de = dde[0];
	goto found;

notfound:

	if(((nameiop == CREATE) || (nameiop == RENAME)) &&
#if (__NetBSD_Version__ < 400000000)
	   (flags & (LOCKPARENT | WANTPARENT)) && 
#endif
	   (flags & ISLASTCN))
	{
	    error = BSD_VOP_ACCESS(dvp, VWRITE, 
	        cnp->cn_cred, td, cnp->cn_lwp);
	    if (error) {
		goto done;
	    }

	    cnp->cn_flags |= SAVENAME;
	    error = EJUSTRETURN;
	    goto done;
	}

	error = ENOENT;
	goto done;

 found:

	if((cnp->cn_nameiop == DELETE) && (flags & ISLASTCN))
	{
		error = BSD_VOP_ACCESS(dvp, VWRITE, cnp->cn_cred, td, cnp->cn_lwp);
		if(error)
		{
		    goto done;
		}

		if(vpp[0] == dvp)
		{
			VREF(dvp);
#if (__NetBSD_Version__ < 400000000)
			unlock = 0;
#endif
			goto done;
		}
	}

	error = devfs_allocv(de, dvp->v_mount, vpp, td);

 done:
#if (__NetBSD_Version__ < 400000000)
	if((error == 0) && unlock)
	{
	    VOP_UNLOCK(dvp, 0);
	    cnp->cn_flags |= PDIRUNLOCK;
	}
#endif
	return error;
}

static int
devfs_lookup(struct vop_lookup_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct devfs_mount *dmp;
	int error;

	dmp = VFSTODEVFS(ap->a_dvp->v_mount);
	__lockmgr(&dmp->dm_lock, LK_SHARED, NULL, curthread);
	error = __devfs_lookup(ap);
	__lockmgr(&dmp->dm_lock, LK_RELEASE, NULL, curthread);
	return error;
}

static int
devfs_mknod(struct vop_mknod_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct componentname *cnp;
	struct devfs_dirent *dd;
	struct devfs_dirent *de;
	struct devfs_mount *dmp;
	struct vnode **vpp;
	struct vnode *dvp;
	struct proc *td;
	int error;

	dvp = ap->a_dvp;
	VOP_UNLOCK(dvp, 0);

	/*
	 * The only type of node we should be creating here is a
	 * character device, for anything else return EOPNOTSUPP.
	 */
	if(ap->a_vap->va_type != VCHR)
	{
	    error = EOPNOTSUPP;
	    goto done;
	}

	dmp = VFSTODEVFS(dvp->v_mount);
	__lockmgr(&dmp->dm_lock, LK_EXCLUSIVE, 0, curthread);

	cnp = ap->a_cnp;
	vpp = ap->a_vpp;
	td = cnp->cn_proc;
	dd = dvp->v_data;

	error = ENOENT;
	TAILQ_FOREACH(de, &dd->de_dlist, de_list)
	{
		if(cnp->cn_namelen != de->de_dirent->d_namlen)
		{
		    continue;
		}
		if(bcmp(cnp->cn_nameptr, de->de_dirent->d_name,
			de->de_dirent->d_namlen))
		{
		    continue;
		}
		if(de->de_flags & DE_WHITEOUT)
		{
			break;
		}
		goto notfound;
	}
	if(de == NULL)
	{
	    goto notfound;
	}
	de->de_flags &= ~DE_WHITEOUT;
	error = devfs_allocv(de, dvp->v_mount, vpp, td);

 notfound:
	__lockmgr(&dmp->dm_lock, LK_RELEASE, 0, curthread);

 done:
	return error;
}

static int
devfs_pathconf(struct vop_pathconf_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
 	switch (ap->a_name) {
	case _PC_NAME_MAX:
		ap->a_retval[0] = NAME_MAX;
		break;
	case _PC_PATH_MAX:
		ap->a_retval[0] = PATH_MAX;
		break;
#if 0
	case _PC_MAC_PRESENT:
		ap->a_retval[0] = 0;
		break;
#endif
	case _PC_LINK_MAX:
		ap->a_retval[0] = LINK_MAX;
		break;
 	case _PC_PIPE_BUF:
		ap->a_retval[0] = PIPE_BUF;
		break;
 	case _PC_CHOWN_RESTRICTED:
		ap->a_retval[0] = 1;
		break;
 	case _PC_NO_TRUNC:
		ap->a_retval[0] = 1;
		break;
 	case _PC_SYNC_IO:
		ap->a_retval[0] = 1;
		break;
	default:
		return EINVAL;
	}
	return 0;
}

static int
devfs_readdir(struct vop_readdir_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct devfs_mount *dmp;
	struct devfs_dirent *dd;
	struct devfs_dirent *de;
	struct dirent *dps;
	struct dirent *dpe;
	struct dirent *dp;
	struct uio *uio;
	off_t *cookiebuf;
	off_t *cookiep;
	off_t oldoff;
	off_t off;
	int error;
	int ncookies = 0;

	if(ap->a_vp->v_type != VDIR)
	{
	    return ENOTDIR;
	}

	uio = ap->a_uio;
	if(uio->uio_offset < 0)
	{
	    return EINVAL;
	}

	dmp = VFSTODEVFS(ap->a_vp->v_mount);
	__lockmgr(&dmp->dm_lock, LK_SHARED, 0, curthread);
	devfs_populate(dmp);
	error = 0;
	de = ap->a_vp->v_data;
	off = 0;
	oldoff = uio->uio_offset;
	TAILQ_FOREACH(dd, &de->de_dlist, de_list)
	{
	    if(dd->de_flags & DE_WHITEOUT)
	    {
	        continue;
	    }
	    if(dd->de_dirent->d_type == DT_DIR)
	    {
	        de = dd->de_dir;
	    }
	    else
	    {
	        de = dd;
	    }
	    dp = dd->de_dirent;
	    if(dp->d_reclen > uio->uio_resid)
	    {
	        break;
	    }
	    dp->d_fileno = de->de_inode;
	    if(off >= uio->uio_offset)
	    {
	        ncookies++;
		error = uiomove(dp, dp->d_reclen, uio);
		if(error)
		{
		    break;
		}
	    }
	    off += dp->d_reclen;
	}

	if((!error) && (ap->a_ncookies != NULL) && (ap->a_cookies != NULL))
	{
	    MALLOC(cookiebuf, off_t *, ncookies * sizeof(u_long), 
		   M_DEVFS, M_WAITOK);

	    cookiep = cookiebuf;
	    dps = (struct dirent *)((char *)uio->uio_iov->iov_base -
				    (uio->uio_offset - oldoff));
	    dpe = (struct dirent *) uio->uio_iov->iov_base;
	    for(dp = dps;
		dp < dpe;
		dp = (struct dirent *)((caddr_t) dp + dp->d_reclen))
	    {
	        oldoff += dp->d_reclen;
		*cookiep++ = (off_t) oldoff;
	    }
	    *ap->a_ncookies = ncookies;
	    *ap->a_cookies = cookiebuf;
	}
	__lockmgr(&dmp->dm_lock, LK_RELEASE, 0, curthread);
	uio->uio_offset = off;
	return error;
}

static int
devfs_readlink(struct vop_readlink_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct devfs_dirent *de;
	int error;

	de = ap->a_vp->v_data;
	error = uiomove(de->de_symlink, strlen(de->de_symlink), ap->a_uio);
	return error;
}

static int
devfs_reclaim(struct vop_reclaim_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	struct devfs_dirent *de;
	struct cdev *dev;

	de = vp->v_data;
	if(de != NULL)
	{
	    de->de_vnode = NULL;
	}
	vp->v_data = NULL;
	cache_purge(vp);
#if 0
	vnode_destroy_vobject(vp);
#endif

	dev = de->de_dev; /* XXX */
	de->de_dev = NULL;

	if(dev == NULL)
	{
	    goto done;
	}

	dev_lock();
	if(de != NULL)
	{
	    LIST_REMOVE(de, de_alias);
	}
	dev->si_usecount -= vp->v_usecount;
	dev_unlock();
	dev_rel(dev);

 done:
	return (0);
}

static int
devfs_remove(struct vop_remove_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	struct devfs_dirent *dd;
	struct devfs_dirent *de;
	struct devfs_mount *dmp = VFSTODEVFS(vp->v_mount);

	__lockmgr(&dmp->dm_lock, LK_EXCLUSIVE, 0, curthread);
	dd = ap->a_dvp->v_data;
	de = vp->v_data;
	if(de->de_dirent->d_type == DT_LNK)
	{
	    TAILQ_REMOVE(&dd->de_dlist, de, de_list);
	    if(de->de_vnode)
	    {
	        de->de_vnode->v_data = NULL;
	    }
	    FREE(de, M_DEVFS);
	}
	else
	{
	    de->de_flags |= DE_WHITEOUT;
	}
	__lockmgr(&dmp->dm_lock, LK_RELEASE, 0, curthread);

	VOP_UNLOCK(vp, 0);
	VOP_UNLOCK(ap->a_dvp, 0);
	return (0);
}

/*
 * Revoke is called on a tty when a terminal session ends.  The vnode
 * is orphaned by setting v_op to deadfs so we need to let go of it
 * as well so that we create a new one next time around.
 */
static int
devfs_revoke(struct vop_revoke_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	struct cdev *dev;
	struct devfs_dirent *de;

	__KASSERT((ap->a_flags & REVOKEALL) != 0,
		  ("%s: not REVOKEALL !", __FUNCTION__));

	de = vp->v_data;
	dev = de->de_dev;

	while(1)
	{
	    dev_lock();
	    de = LIST_FIRST(&dev->si_alist);
	    dev_unlock();
	    if(de == NULL)
	    {
	        break;
	    }
	    vgone(de->de_vnode);
	}
	return (0);
}

static int
devfs_symlink(struct vop_symlink_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct devfs_dirent *dd;
	struct devfs_dirent *de;
	struct devfs_mount *dmp;
	struct proc *td;
	int i, error;
#if 0
	VOP_UNLOCK(ap->a_tdvp, 0);
#endif

	td = ap->a_cnp->cn_proc;
	__KASSERT(td == curthread, ("%s: td != curthread", __FUNCTION__));
	error = suser(td);
	if(error)
	{
	    goto done;
	}
	dmp = VFSTODEVFS(ap->a_dvp->v_mount);
	dd = ap->a_dvp->v_data;
	de = devfs_newdirent(DECONST(ap->a_cnp->cn_nameptr), 
			     ap->a_cnp->cn_namelen);
	de->de_uid = 0;
	de->de_gid = 0;
	de->de_mode = 0755;
	de->de_inode = dmp->dm_inode++;
	de->de_dirent->d_type = DT_LNK;
	i = strlen(ap->a_target) + 1;
	MALLOC(de->de_symlink, char *, i, M_DEVFS, M_WAITOK);
	bcopy(ap->a_target, de->de_symlink, i);
	__lockmgr(&dmp->dm_lock, LK_EXCLUSIVE, 0, td);
	TAILQ_INSERT_TAIL(&dd->de_dlist, de, de_list);
	devfs_allocv(de, ap->a_dvp->v_mount, ap->a_vpp, td);
	__lockmgr(&dmp->dm_lock, LK_RELEASE, 0, td);

 done:

	return error;
}

/*
 * FreeBSD is moving from IO_XXX to O_XXX for device
 * drivers. To not break old drivers, the following
 * assertions must be met:
 */

#if ((O_NONBLOCK != IO_NDELAY) || !defined(O_NONBLOCK) || !defined(IO_NDELAY))
#error "Assertion not met!"
#endif

#if ((O_FSYNC != IO_SYNC) || !defined(O_FSYNC) || !defined(IO_SYNC))
#error "Assertion not met!"
#endif

static int
devfs_fsync(struct vop_fsync_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return 0;
}

static int
devfs_print(struct vop_print_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct devfs_dirent *de = ap->a_vp->v_data;
	struct cdev *dev = de->de_dev;
	printf("\tdev %s\n", devtoname(dev));
	return (0);
}

static int
devfs_advlock(struct vop_advlock_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return ((ap->a_flags & F_FLOCK) ? EOPNOTSUPP : EINVAL);
}

static int
devfs_lock(struct vop_lock_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	return (lockmgr(&vp->v_lock, ap->a_flags, &vp->v_interlock));
}

static int
devfs_unlock(struct vop_unlock_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	return (lockmgr(&vp->v_lock, ap->a_flags | LK_RELEASE,
			&vp->v_interlock));
}

static int
devfs_islocked(struct vop_islocked_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;

	return (lockstatus(&vp->v_lock));
}

static int
devfs_inactive(struct vop_inactive_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_vp;
	VOP_UNLOCK(vp, 0);
	vgone(vp);
	return 0;
}

static int
_devfs_create(struct vop_create_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	struct vnode *vp = ap->a_dvp;

	VOP_UNLOCK(vp, 0);

	return EOPNOTSUPP;
}

static int
devfs_lease(struct vop_lease_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

static int
devfs_fcntl(struct vop_fcntl_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

static int
devfs_mmap(struct vop_mmap_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

static int
devfs_seek(struct vop_seek_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return (ap->a_newoff < 0) ? EINVAL : 0;
}

static int
devfs_link(struct vop_link_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	VOP_UNLOCK(ap->a_dvp, 0);
	return EOPNOTSUPP;
}

static int
devfs_rename(struct vop_rename_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	VOP_UNLOCK(ap->a_tdvp, 0);
	if(ap->a_tvp)
	{
	    VOP_UNLOCK(ap->a_tvp, 0);
	}
	return EOPNOTSUPP;
}

static int
devfs_mkdir(struct vop_mkdir_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	VOP_UNLOCK(ap->a_dvp, 0);
	return EOPNOTSUPP;
}

static int
devfs_rmdir(struct vop_rmdir_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	VOP_UNLOCK(ap->a_dvp, 0);
	VOP_UNLOCK(ap->a_vp, 0);
	return EOPNOTSUPP;
}

static int
devfs_abortop(struct vop_abortop_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return 0;
}

static int
devfs_bmap(struct vop_bmap_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

static int
devfs_strategy(struct vop_strategy_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

#if (__NetBSD_Version__ < 400000000)
static int
devfs_blkatoff(struct vop_blkatoff_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

static int
devfs_valloc(struct vop_valloc_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return 0;
}

static int
devfs_vfree(struct vop_vfree_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

static int
devfs_truncate(struct vop_truncate_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

static int
devfs_update(struct vop_update_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return 0;
}
#endif

static int
devfs_bwrite(struct vop_bwrite_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return EOPNOTSUPP;
}

extern int genfs_getpages(void *);
extern int genfs_putpages(void *);

static int
devfs_getpages(struct vop_getpages_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return genfs_getpages(ap);
}

static int
devfs_putpages(struct vop_putpages_args *ap)
{
#ifdef DEVFS_DEBUG
	printf("%s\n", __FUNCTION__);
#endif
	return genfs_putpages(ap);
}

/*
 * devfs export table
 */

typedef int (* const vop_t)(void *);

static const struct vnodeopv_entry_desc 
devfs_vnodeop_entries[] = 
{
  { &vop_default_desc      , (vop_t)&vn_default_error },

  /*
   * basic vnode operations:
   */

  { &vop_open_desc         , (vop_t)&devfs_open },
  { &vop_read_desc         , (vop_t)&devfs_read },
  { &vop_write_desc        , (vop_t)&devfs_write },
  { &vop_ioctl_desc        , (vop_t)&devfs_ioctl },
  { &vop_poll_desc         , (vop_t)&devfs_poll },
  { &vop_kqfilter_desc     , (vop_t)&devfs_kqfilter },
  { &vop_close_desc        , (vop_t)&devfs_close },

  /*
   * extended vnode operations:
   */

  { &vop_access_desc       , (vop_t)&devfs_access },
  { &vop_getattr_desc      , (vop_t)&devfs_getattr },
  { &vop_setattr_desc      , (vop_t)&devfs_setattr },
  { &vop_lookup_desc       , (vop_t)&devfs_lookup },
  { &vop_mknod_desc        , (vop_t)&devfs_mknod },
  { &vop_pathconf_desc     , (vop_t)&devfs_pathconf },
  { &vop_readdir_desc      , (vop_t)&devfs_readdir },
  { &vop_readlink_desc     , (vop_t)&devfs_readlink },
  { &vop_reclaim_desc      , (vop_t)&devfs_reclaim },
  { &vop_remove_desc       , (vop_t)&devfs_remove },
  { &vop_revoke_desc       , (vop_t)&devfs_revoke },
  { &vop_symlink_desc      , (vop_t)&devfs_symlink },
  { &vop_fsync_desc        , (vop_t)&devfs_fsync },
  { &vop_print_desc        , (vop_t)&devfs_print },
  { &vop_advlock_desc      , (vop_t)&devfs_advlock },

  { &vop_lock_desc         , (vop_t)&devfs_lock },
  { &vop_unlock_desc       , (vop_t)&devfs_unlock },
  { &vop_islocked_desc     , (vop_t)&devfs_islocked },
  { &vop_inactive_desc     , (vop_t)&devfs_inactive },

  /*
   * various vnode operations:
   */

  { &vop_create_desc       , (vop_t)&_devfs_create },
  { &vop_lease_desc        , (vop_t)&devfs_lease },
  { &vop_fcntl_desc        , (vop_t)&devfs_fcntl },
  { &vop_mmap_desc         , (vop_t)&devfs_mmap },
  { &vop_seek_desc         , (vop_t)&devfs_seek },
  { &vop_link_desc         , (vop_t)&devfs_link },
  { &vop_rename_desc       , (vop_t)&devfs_rename },
  { &vop_mkdir_desc        , (vop_t)&devfs_mkdir },
  { &vop_rmdir_desc        , (vop_t)&devfs_rmdir },
  { &vop_abortop_desc      , (vop_t)&devfs_abortop },
  { &vop_bmap_desc         , (vop_t)&devfs_bmap },
  { &vop_strategy_desc     , (vop_t)&devfs_strategy },
#if (__NetBSD_Version__ < 400000000)
  { &vop_blkatoff_desc     , (vop_t)&devfs_blkatoff },
  { &vop_valloc_desc       , (vop_t)&devfs_valloc },
  { &vop_vfree_desc        , (vop_t)&devfs_vfree },
  { &vop_truncate_desc     , (vop_t)&devfs_truncate },
  { &vop_update_desc       , (vop_t)&devfs_update },
#endif
  { &vop_bwrite_desc       , (vop_t)&devfs_bwrite },
  { &vop_getpages_desc     , (vop_t)&devfs_getpages },
  { &vop_putpages_desc     , (vop_t)&devfs_putpages },

  { NULL, NULL }
};

const struct vnodeopv_desc
devfs_vnodeop_opv_desc =
  { &devfs_vnodeop_p, &devfs_vnodeop_entries[0] };

/*
 * Not implemented:
 *
 * vop_stat_desc
 * vop_mmap_desc
 */
