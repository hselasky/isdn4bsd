/*-
 * Copyright (c) 1982, 1986, 1989, 1990, 1993
 *      The Regents of the University of California.  All rights reserved.
 * Copyright (c) 1997 Peter Wemm. All rights reserved.
 * Copyright (c) 2000
 *      Poul-Henning Kamp.  All rights reserved.
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

struct ucred;
struct cdev;
struct file;
struct stat;
struct knote;
struct selinfo;

#define	FREAD		0x0001
#define	FWRITE		0x0002
#define	O_NONBLOCK	0x0004		/* no delay */
#define	O_APPEND	0x0008		/* set append mode */
#define	O_ASYNC		0x0040		/* signal pgrp when data ready */
#define	O_FSYNC		0x0080		/* synchronous writes */
#define	O_SYNC		0x0080		/* POSIX synonym for O_FSYNC */
#define	O_DIRECT	0x00010000
#define	IO_DIRECT	0x0100		/* attempt to bypass buffer cache */
#define	FOF_OFFSET	1		/* Use the offset in uio argument */

enum uio_rw {
	UIO_READ,
	UIO_WRITE
};

/* Segment flag values. */
enum uio_seg {
	UIO_USERSPACE,			/* from user data space */
	UIO_SYSSPACE,			/* from system space */
	UIO_NOCOPY			/* don't copy, already in object */
};

struct iovec {
	void   *iov_base;		/* Base address. */
	size_t	iov_len;		/* Length. */
};

struct uio {
	struct iovec *uio_iov;		/* scatter/gather list */
	int	uio_iovcnt;		/* length of scatter/gather list */
	off_t	uio_offset;		/* offset in target object */
	int	uio_resid;		/* remaining bytes to process */
	enum uio_seg uio_segflg;	/* address space */
	enum uio_rw uio_rw;		/* operation */
	struct thread *uio_td;		/* owner */
};

typedef int d_open_t (struct cdev *dev, int oflags, int devtype, struct thread *td);
typedef int d_fdopen_t (struct cdev *dev, int oflags, struct thread *td, struct file *fp);
typedef int d_close_t (struct cdev *dev, int fflag, int devtype, struct thread *td);
typedef int d_ioctl_t (struct cdev *dev, u_long cmd, caddr_t data, int fflag, struct thread *td);

#define	D_TRACKCLOSE    0x00000000	/* track all closes */
#define	D_VERSION       0x00000000

/*
 * Character device switch table
 */
struct cdevsw {
	int	d_version;
	uint32_t d_flags;
	const char *d_name;
	d_open_t *d_open;
	d_fdopen_t *d_fdopen;
	d_close_t *d_close;
	d_ioctl_t *d_ioctl;
};

typedef int fo_rdwr_t (struct file *fp, struct uio *uio, struct ucred *active_cred, int flags, struct thread *td);

#define	FOF_OFFSET      1		/* Use the offset in uio argument */
typedef int fo_truncate_t (struct file *fp, off_t length, struct ucred *active_cred, struct thread *td);
typedef int fo_ioctl_t (struct file *fp, u_long com, void *data, struct ucred *active_cred, struct thread *td);
typedef int fo_poll_t (struct file *fp, int events, struct ucred *active_cred, struct thread *td);
typedef int fo_kqfilter_t (struct file *fp, struct knote *kn);
typedef int fo_stat_t (struct file *fp, struct stat *sb, struct ucred *active_cred, struct thread *td);
typedef int fo_close_t (struct file *fp, struct thread *td);
typedef int fo_flags_t;

struct fileops {
	fo_rdwr_t *fo_read;
	fo_rdwr_t *fo_write;
	fo_truncate_t *fo_truncate;
	fo_ioctl_t *fo_ioctl;
	fo_poll_t *fo_poll;
	fo_kqfilter_t *fo_kqfilter;
	fo_stat_t *fo_stat;
	fo_close_t *fo_close;
	fo_flags_t fo_flags;		/* DFLAG_* below */
};

#define	DFLAG_PASSABLE  0x01		/* may be passed via unix sockets. */
#define	DFLAG_SEEKABLE  0x02		/* seekable / nonsequential */

#define	POLLIN          0x0001		/* any readable data available */
#define	POLLPRI         0x0002		/* OOB/Urgent readable data */
#define	POLLOUT         0x0004		/* file descriptor is writeable */
#define	POLLRDNORM      0x0040		/* non-OOB/URG data available */
#define	POLLWRNORM      POLLOUT		/* no write type differentiation */
#define	POLLRDBAND      0x0080		/* OOB/Urgent readable data */
#define	POLLWRBAND      0x0100		/* OOB/Urgent data can be written */

/*
 * These events are set if they occur regardless of whether they were
 * requested.
 */
#define	POLLERR         0x0008		/* some poll error occurred */
#define	POLLHUP         0x0010		/* file descriptor was "hung up" */
#define	POLLNVAL        0x0020		/* requested events "invalid" */

struct file {
	void   *f_data;			/* file descriptor specific data */
	struct fileops *f_ops;		/* File operations */
	struct ucred *f_cred;		/* associated credentials. */
	struct vnode *f_vnode;		/* NULL or applicable vnode */
	short	f_type;			/* descriptor type */
	short	f_vnread_flags;		/* (f) Sleep lock for f_offset */
	volatile uint32_t f_flag;	/* see fcntl.h */
	volatile uint32_t f_count;	/* reference count */
	/*
         *  DTYPE_VNODE specific fields.
         */
	int	f_seqcount;		/* Count of sequential accesses. */
	off_t	f_nextoff;		/* next expected read/write offset. */
	/*
         *  DFLAG_SEEKABLE specific fields
         */
	off_t	f_offset;
	/*
         * Mandatory Access control information.
         */
	void   *f_label;		/* Place-holder for MAC label. */
};

/* Generic file-descriptor ioctl's. */
#define	FIONREAD        _IOR('f', 127, int)	/* get # bytes to read */
#define	FIONBIO         _IOW('f', 126, int)	/* set/clear non-blocking i/o */
#define	FIOASYNC        _IOW('f', 125, int)	/* set/clear async i/o */
#define	TIOCSPGRP       _IOW('t', 118, int)	/* set pgrp of tty */
#define	TIOCGPGRP       _IOR('t', 119, int)	/* get pgrp of tty */

#define	UID_ROOT        0
#define	UID_BIN         3
#define	UID_UUCP        66

#define	GID_WHEEL       0
#define	GID_KMEM        2
#define	GID_TTY         4
#define	GID_OPERATOR    5
#define	GID_BIN         7
#define	GID_GAMES       13
#define	GID_DIALER      68

typedef void (*dev_clone_fn) (void *arg, struct ucred *ucred, char *name, int namelen, struct cdev **dev);

EVENTHANDLER_DECLARE(dev_clone, dev_clone_fn);

/* dummies */

void	selrecord(struct thread *td, struct selinfo *sip);
void	selwakeup(struct selinfo *sip);
void	dev_ref(struct cdev *dev);
struct cdev *make_dev(struct cdevsw *_devsw, int _minor, uid_t _uid, gid_t _gid, int _perms, const char *_fmt,...)__printflike(6, 7);
void	destroy_dev(struct cdev *_dev);
