/*-
 * Copyright (c) 2005
 *      Hans Petter Selasky. All rights reserved.
 * Copyright (c) 2000
 *	Poul-Henning Kamp.  All rights reserved.
 * Copyright (c) 1992, 1993, 1995
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
 * This file is a lite version of "FreeBSD/src/sys/fs/devfs/devfs_vfsops.c"
 *
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

static int
devfs_root(struct mount *mp, /* int flags, */
	   struct vnode **vpp)
{
	struct thread *td = curthread; /* XXX */
	struct devfs_mount *dmp;
	struct vnode *vp;
	int error;

	dmp = VFSTODEVFS(mp);
	error = devfs_allocv(dmp->dm_rootdir, mp, &vp, td);
	if(error)
	{
	    goto done;
	}
#if (__NetBSD_Version__ >= 500000000)
	vp->v_vflag |= VV_ROOT;
#else
	vp->v_flag |= VROOT;
#endif
	*vpp = vp;

 done:
	return error;
}

#undef DECONST
#define DECONST(arg) ((void *)(((const char *)arg) - ((const char *)0)))

#if (__NetBSD_Version__ >= 500000000)
static int
devfs_mount(struct mount *mp, const char *path, void *data,
	    size_t *data_len)
#elif (__NetBSD_Version__ >= 400000000)
static int
devfs_mount(struct mount *mp, const char *path, void *data,
	    struct nameidata *ndp, struct lwp *l)
#else
static int
devfs_mount(struct mount *mp, const char *path, void *data,
	    struct nameidata *ndp, struct thread *td)
#endif
{
	struct devfs_mount *fmp;
	struct vnode *rvp;
	int error = 0;

	if(mp->mnt_flag & (MNT_UPDATE | MNT_ROOTFS))
	{
	    return EOPNOTSUPP;
	}

	MALLOC(fmp, struct devfs_mount *, sizeof(struct devfs_mount),
	       M_DEVFS, M_WAITOK | M_ZERO);

	MALLOC(fmp->dm_dirent, struct devfs_dirent **,
	       sizeof(struct devfs_dirent *) * NDEVFSINO,
	       M_DEVFS, M_WAITOK | M_ZERO);

	lockinit(&fmp->dm_lock, PVFS, "devfs", 0, 0);

	mp->mnt_flag |= MNT_LOCAL;
	mp->mnt_data = fmp;
#if 0
	mp->mnt_stat.f_namemax = MAXNAMLEN;
#endif

	fmp->dm_mount = mp;

	vfs_getnewfsid(mp);

	fmp->dm_inode = DEVFSINOMOUNT;

	fmp->dm_rootdir = devfs_vmkdir(DECONST("(root)"), 6, NULL);
	fmp->dm_rootdir->de_inode = 2;
	fmp->dm_basedir = fmp->dm_rootdir;

#if 0
	devfs_rules_newmount(fmp, td);
#endif

	error = devfs_root(mp, /* LK_EXCLUSIVE, */ &rvp /* , td */);
	if(error)
	{
	    goto done;
	}

	VOP_UNLOCK(rvp, 0);

	fmp->dm_root_vnode = rvp;

#if (__NetBSD_Version__ >= 500000000)
	error = set_statvfs_info(path, UIO_USERSPACE, "devfs", UIO_SYSSPACE,
	    mp->mnt_op->vfs_name, mp, l);
#elif (__NetBSD_Version__ >= 400000000)
	error = set_statfs_info
	  (path, UIO_USERSPACE, "devfs", UIO_SYSSPACE, mp, l);
#else
	error = set_statfs_info
	  (path, UIO_USERSPACE, "devfs", UIO_SYSSPACE, mp, td);
#endif
	if(error)
	{
	    goto done;
	}
#if 0
	vfs_mountedfrom(mp, "devfs");
#endif

 done:
	if(error)
	{
	    mp->mnt_data = NULL;
	    lockdestroy(&fmp->dm_lock);
	    FREE(fmp, M_DEVFS);
	}
	return error;
}

#if (__NetBSD_Version__ >= 500000000)
static int
devfs_unmount(struct mount *mp, int mntflags)
#elif (__NetBSD_Version__ >= 400000000)
static int
devfs_unmount(struct mount *mp, int mntflags, 
	      struct lwp *l)
#else
static int
devfs_unmount(struct mount *mp, int mntflags, 
	      struct thread *td)
#endif
{
	struct devfs_mount *fmp;
	int flags = 0;
	int error;

	if((mntflags & MNT_FORCE))
	{
	    flags |= FORCECLOSE;
	}

	fmp = VFSTODEVFS(mp);

#if 1
	/* drop extra reference to root vnode */

	if(fmp->dm_root_vnode)
	{
	    vrele(fmp->dm_root_vnode);
	}
#endif

	/* flush vnodes */

	error = vflush(mp, fmp->dm_root_vnode, flags);
	if(error)
	{
	    goto done;
	}

	devfs_purge(fmp->dm_rootdir);
	mp->mnt_data = NULL;
	lockdestroy(&fmp->dm_lock);
	free(fmp->dm_dirent, M_DEVFS);
	free(fmp, M_DEVFS);

 done:
	return error;
}

#if (__NetBSD_Version__ >= 500000000)
static int
devfs_statfs(struct mount *mp, struct statfs *sbp)
#elif (__NetBSD_Version__ >= 400000000)
static int
devfs_statfs(struct mount *mp, struct statfs *sbp, struct lwp *l)
#else
static int
devfs_statfs(struct mount *mp, struct statfs *sbp, struct thread *td)
#endif
{
#if (__NetBSD_Version__ < 300000000)
	sbp->f_flags = 0;
#endif
	sbp->f_bsize = DEV_BSIZE;
	sbp->f_iosize = DEV_BSIZE;
	sbp->f_blocks = 2;		/* 1K to keep df happy */
	sbp->f_bfree = 0;
	sbp->f_bavail = 0;
	sbp->f_files = 0;
	sbp->f_ffree = 0;

	copy_statfs_info(sbp, mp);
	return 0;
}

static void
devfs_init(void)
{
	return;
}

static void
devfs_reinit(void)
{
	return;
}

static void
devfs_done(void)
{
	return;
}

#if (__NetBSD_Version__ >= 500000000)
static int
devfs_start(struct mount *a, int b)
#elif (__NetBSD_Version__ >= 400000000)
static int
devfs_start(struct mount *a, int b, struct lwp *l)
#else
static int
devfs_start(struct mount *a, int b, struct proc *c)
#endif
{
	return 0;
}

#if (__NetBSD_Version__ >= 500000000)
static int
devfs_quotactl(struct mount *a, int b, uid_t c, void *)
#elif (__NetBSD_Version__ < 300000000)
static int
devfs_quotactl(struct mount *a, int b, uid_t c, caddr_t d,
	       struct proc *e)
#elif (__NetBSD_Version__ >= 400000000)
static int
devfs_quotactl(struct mount *a, int b, uid_t c, void *d,
	       struct lwp *l)
#else
static int
devfs_quotactl(struct mount *a, int b, uid_t c, void *d,
	       struct proc *e)
#endif
{
	return ENOTSUP;
}

#if (__NetBSD_Version__ >= 500000000)
static int
devfs_sync(struct mount *a, int b, kauth_cred_t kcred)
#elif (__NetBSD_Version__ >= 400000000)
static int
devfs_sync(struct mount *a, int b, kauth_cred_t kcred,
	   struct lwp *l)
#else
static int
devfs_sync(struct mount *a, int b, struct ucred * c,
	   struct proc *d)
#endif
{
	return 0;
}

static int
devfs_vget(struct mount *a, ino_t b, struct vnode **c)
{
	return ENOTSUP;
} 

static int
devfs_fhtovp(struct mount *a, struct fid *b,
	     struct vnode **c)
{
	return ENOTSUP;
} 

#if (__NetBSD_Version__ >= 400000000)
static int
devfs_vptofh(struct vnode *a, struct fid *b, size_t *ps)
#else
static int
devfs_vptofh(struct vnode *a, struct fid *b)
#endif
{
	return ENOTSUP;
} 

#if (__NetBSD_Version__ < 400000000)
static int
devfs_checkexp(struct mount *a, struct mbuf *b, int *c,
	       struct ucred **d)
{
	return ENOTSUP;
}
#endif

static const struct vnodeopv_desc * const 
devfs_vnodeopv_descs[] = 
{
	&devfs_vnodeop_opv_desc,
	NULL,
};

static struct vfsops devfs_vfsops = 
{
  .vfs_name           = MOUNT_DEVFS,
  .vfs_mount          = &devfs_mount,
  .vfs_unmount        = &devfs_unmount,
  .vfs_root           = &devfs_root,
  .vfs_statfs         = &devfs_statfs,
  .vfs_opv_descs      = &devfs_vnodeopv_descs[0],

  /*
   * not used
   */
  .vfs_init           = &devfs_init,
  .vfs_reinit         = &devfs_reinit,
  .vfs_done           = &devfs_done,
  .vfs_start          = &devfs_start,
  .vfs_quotactl       = &devfs_quotactl,
  .vfs_sync           = &devfs_sync,
  .vfs_vget           = &devfs_vget,
  .vfs_fhtovp         = &devfs_fhtovp,
  .vfs_vptofh         = &devfs_vptofh,
#if (__NetBSD_Version__ < 400000000)
  .vfs_checkexp       = &devfs_checkexp,
#endif
};

static void
devfs_sysinit(void *arg)
{
	int error;

	error = vfs_attach(&devfs_vfsops);

	if(error)
	{
	    printf("%s: VFS attach failed, error=%d!\n",
		   __FUNCTION__, error);
	    goto done;
	}

 done:
	return;
}

SYSINIT(devfs_sysinit, SI_SUB_DRIVERS, SI_ORDER_FIRST, 
	devfs_sysinit, NULL);

static void
devfs_sysuninit(void *arg)
{
	vfs_detach(&devfs_vfsops);

	return;
}

SYSUNINIT(devfs_sysuninit, SI_SUB_DRIVERS, SI_ORDER_FIRST, 
	  devfs_sysuninit, NULL);

