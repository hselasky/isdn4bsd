/*-
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 * Copyright (c) 2000
 *	Poul-Henning Kamp.  All rights reserved.
 * Copyright (c) 2002
 *	Dima Dorfman.  All rights reserved.
 *
 * This code is derived from software donated to Berkeley by
 * Jan-Simon Pendry.
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
 * This file is a lite version of "FreeBSD/src/sys/fs/devfs/devfs.h"
 */

#ifndef __FREEBSD_FS_DEVFS_DEVFS_H__
#define	__FREEBSD_FS_DEVFS_DEVFS_H__

#define	DEVFS_MAGIC	0xdb0a087a

/*
 * Identifiers.  The ruleset and rule numbers are 16-bit values.  The
 * "rule ID" is a combination of the ruleset and rule number; it
 * should be able to univocally describe a rule in the system.  In
 * this implementation, the upper 16 bits of the rule ID is the
 * ruleset number; the lower 16 bits, the rule number within the
 * aforementioned ruleset.
 */
typedef uint16_t devfs_rnum;
typedef uint16_t devfs_rsnum;
typedef uint32_t devfs_rid;

/*
 * Identifier manipulators.
 */
#define	rid2rsn(rid)	((rid) >> 16)
#define	rid2rn(rid)	((rid) & 0xffff)
#define	mkrid(rsn, rn)	((rn) | ((rsn) << 16))

/*
 * Plain DEVFS rule.  This gets shared between kernel and userland
 * verbatim, so it shouldn't contain any pointers or other kernel- or
 * userland-specific values.
 */
struct devfs_rule {
	uint32_t dr_magic;			/* Magic number. */
	devfs_rid dr_id;			/* Identifier. */

	/*
	 * Conditions under which this rule should be applied.  These
	 * are ANDed together since OR can be simulated by using
	 * multiple rules.  dr_icond determines which of the other
	 * variables we should process.
	 */
	int	dr_icond;
#define	DRC_DSWFLAGS	0x001
#define	DRC_PATHPTRN	0x002
	int	dr_dswflags;			/* cdevsw flags to match. */
#define	DEVFS_MAXPTRNLEN	200
	char	dr_pathptrn[DEVFS_MAXPTRNLEN];	/* Pattern to match path. */

	/*
	 * Things to change.  dr_iacts determines which of the other
	 * variables we should process.
	 */
	int	dr_iacts;
#define	DRA_BACTS	0x001
#define	DRA_UID		0x002
#define	DRA_GID		0x004
#define	DRA_MODE	0x008
#define	DRA_INCSET	0x010
	int	dr_bacts;			/* Boolean (on/off) action. */
#define	DRB_HIDE	0x001			/* Hide entry (DE_WHITEOUT). */
#define	DRB_UNHIDE	0x002			/* Unhide entry. */
	uid_t	dr_uid;
	gid_t	dr_gid;
	mode_t	dr_mode;
	devfs_rsnum dr_incset;			/* Included ruleset. */
};

/*
 * Rule-related ioctls.
 */
#define	DEVFSIO_RADD		_IOWR('D', 0, struct devfs_rule)
#define	DEVFSIO_RDEL		_IOW( 'D', 1, devfs_rid)
#define	DEVFSIO_RAPPLY		_IOW( 'D', 2, struct devfs_rule)
#define	DEVFSIO_RAPPLYID	_IOW( 'D', 3, devfs_rid)
#define	DEVFSIO_RGETNEXT       	_IOWR('D', 4, struct devfs_rule)

#define	DEVFSIO_SUSE		_IOW( 'D', 10, devfs_rsnum)
#define	DEVFSIO_SAPPLY		_IOW( 'D', 11, devfs_rsnum)
#define	DEVFSIO_SGETNEXT	_IOWR('D', 12, devfs_rsnum)

/* XXX: DEVFSIO_RS_GET_INFO for refcount, active if any, etc. */

#ifdef _KERNEL

/*
 * These are default sizes for the DEVFS inode table and the overflow
 * table.  If the default table overflows we allocate the overflow 
 * table, the size of which can also be set with a sysctl.  If the
 * overflow table fills you're toast.
 */
#ifndef NDEVFSINO
#define NDEVFSINO 1024
#endif

#ifndef NDEVFSOVERFLOW
#define NDEVFSOVERFLOW 32768
#endif

/*
 * This is the first "per mount" inode, these are used for directories
 * and symlinks and the like.  Must be larger than the number of "true"
 * device nodes and symlinks.  It is.
 */
#define DEVFSINOMOUNT	0x2000000

#ifdef MALLOC_DECLARE
MALLOC_DECLARE(M_DEVFS);
#endif

struct cdev;

struct devfs_dirent {
	int			de_inode;
	int			de_flags;
#define	DE_WHITEOUT	0x1
#define	DE_DOT		0x2
#define	DE_DOTDOT	0x4
	struct dirent 		*de_dirent;
	TAILQ_ENTRY(devfs_dirent) de_list;
	TAILQ_HEAD(, devfs_dirent) de_dlist;
	LIST_ENTRY(devfs_dirent) de_alias;
	struct devfs_dirent *   de_dir;
	int			de_links;
	mode_t			de_mode;
	uid_t			de_uid;
	gid_t			de_gid;
	struct label *          de_label;
	struct timespec 	de_atime;
	struct timespec 	de_mtime;
	struct timespec 	de_ctime;
	struct vnode *          de_vnode;
	char *                  de_symlink;
	struct cdev *           de_dev;
};

struct devfs_mount {
	struct mount *          dm_mount;
	struct devfs_dirent *   dm_rootdir;
	struct devfs_dirent *   dm_basedir;
	unsigned		dm_generation;
	struct devfs_dirent **  dm_dirent;
	struct devfs_dirent **  dm_overflow;
	int			dm_inode;
	struct lock		dm_lock;
	devfs_rsnum		dm_ruleset;
	struct vnode *          dm_root_vnode;
};

/*
 * This is what we fill in dm_dirent[N] for a deleted entry.
 */
#define DE_DELETED ((struct devfs_dirent *)sizeof(struct devfs_dirent))

#define VFSTODEVFS(mp)	((struct devfs_mount *)((mp)->mnt_data))

extern void 
  devfs_rules_apply(struct devfs_mount *dm, struct devfs_dirent *de);
extern int
  devfs_rules_ioctl(struct mount *mp, u_long cmd, caddr_t data, struct thread *td);
extern void 
  devfs_rules_newmount(struct devfs_mount *dm, struct thread *td);
extern int
  devfs_allocv(struct devfs_dirent *de, struct mount *mp, struct vnode **vpp, 
	       struct thread *td);
extern struct cdev **
  devfs_itod(int inode);
extern struct devfs_dirent **
  devfs_itode(struct devfs_mount *dm, int inode);
extern int
  devfs_populate(struct devfs_mount *dm);
extern struct devfs_dirent *
  devfs_newdirent(char *name, int namelen);
extern void
  devfs_purge(struct devfs_dirent *dd);
extern struct devfs_dirent *
  devfs_vmkdir (char *name, int namelen, 
		struct devfs_dirent *dotdot);
extern const struct vnodeopv_desc 
  devfs_vnodeop_opv_desc;
extern void
  devfs_timestamp(struct timespec *ts);

#undef minor
#undef major
#undef makedev

extern int 
  minor(struct cdev *x);
extern dev_t 
  dev2udev(struct cdev *x);
extern int 
  uminor(dev_t dev);
extern int 
  umajor(dev_t dev);
extern const char *
  devtoname(struct cdev *cdev);

#define MOUNT_DEVFS "devfs"
#define M_DEVFS M_TEMP
#define S_IFCHR  0020000                /* character special */

#define O_DIRECT 0
#define IO_DIRECT 0

#define VI_LOCK(vp) simple_lock(&(vp)->v_interlock)
#define VI_UNLOCK(vp) simple_unlock(&(vp)->v_interlock)

#define FIODTYPE        _IOR('f', 122, int)     /* get d_flags type part */
struct fiodgname_arg {
   int     len;
   void *  buf;
};
#define FIODGNAME       _IOW('f', 120, struct fiodgname_arg) /* get dev. name */

#endif /* _KERNEL */

#endif /* !__FREEBSD_FS_DEVFS_DEVFS_H__ */
