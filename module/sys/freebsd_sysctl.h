/*-
 * Copyright (c) 1989, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Mike Karels at Berkeley Software Design, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
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
 *
 * This is a lite version of "FreeBSD/src/sys/sys/sysctl.h"
 */

#ifndef __FREEBSD_SYS_SYSCTL_H__
#define	__FREEBSD_SYS_SYSCTL_H__

#define CTLTYPE		0xf	/* Mask for the type */
#define	CTLTYPE_NODE	1	/* name is a node */
#define	CTLTYPE_INT	2	/* name describes an integer */
#define	CTLTYPE_STRING	3	/* name describes a string */
#define	CTLTYPE_QUAD	4	/* name describes a 64-bit number */
#define	CTLTYPE_STRUCT	5	/* name describes a structure */
#define	CTLTYPE_OPAQUE	CTLTYPE_STRUCT /* name describes a structure */
#define	CTLTYPE_UINT	6	/* name describes an unsigned integer */
#define	CTLTYPE_LONG	7	/* name describes a long */
#define	CTLTYPE_ULONG	8	/* name describes an unsigned long */

#define CTLFLAG_RD	0x80000000	/* Allow reads of variable */
#define CTLFLAG_WR	0x40000000	/* Allow writes to the variable */
#define CTLFLAG_RW	(CTLFLAG_RD|CTLFLAG_WR)
#define CTLFLAG_NOLOCK	0
#define CTLFLAG_ANYBODY	0
#define CTLFLAG_SECURE	0
#define CTLFLAG_PRISON	0
#define CTLFLAG_DYN	0
#define CTLFLAG_SKIP	0
#define CTLMASK_SECURE	0
#define CTLFLAG_TUN	0x20000000
#define CTLFLAG_RDTUN	(CTLFLAG_RD|CTLFLAG_TUN)
#define CTLFLAG_SECURE1	0
#define CTLFLAG_SECURE2	0
#define CTLFLAG_SECURE3	0

#define OID_AUTO	(-1)
#define CTL_AUTO_START	0x100

#ifdef _KERNEL
#define SYSCTL_HANDLER_ARGS				\
  struct sysctl_oid *oidp, void *arg1, int arg2,	\
  struct sysctl_req *req

/* definitions for sysctl_req 'lock' member */
#define REQ_UNLOCKED	0	/* not locked and not wired */
#define REQ_LOCKED	1	/* locked and not wired */
#define REQ_WIRED	2	/* locked and wired */

/*
 * This describes the access space for a sysctl request.  This is needed
 * so that we can use the interface from the kernel or from user-space.
 */
struct sysctl_req {
    struct thread   *td;            /* used for access checking */
    int             lock;           /* locking/wiring state */
    void            *oldptr;
    size_t          oldlen;
    size_t          oldidx;
    int             (*oldfunc)(struct sysctl_req *, const void *, size_t);
    void            *newptr;
    size_t          newlen;
    size_t          newidx;
    int             (*newfunc)(struct sysctl_req *, void *, size_t);
    size_t          validlen;
    int             flags;
};

SLIST_HEAD(sysctl_oid_list, sysctl_oid);

/*
 * This describes one "oid" in the MIB tree.  Potentially more nodes can
 * be hidden behind it, expanded by the handler.
 */
struct sysctl_oid {
	struct sysctl_oid_list *oid_parent;
	SLIST_ENTRY(sysctl_oid) oid_link;
	int		oid_number;
	u_int		oid_kind;
	void		*oid_arg1;
	int		oid_arg2;
	const char	*oid_name;
	int 		(*oid_handler)(SYSCTL_HANDLER_ARGS);
	const char	*oid_fmt;
	int		oid_refcnt;
	const char	*oid_descr;
};

#define SYSCTL_IN(r, p, l) (r->newfunc)(r, p, l)
#define SYSCTL_OUT(r, p, l) (r->oldfunc)(r, p, l)

extern int sysctl_handle_int(SYSCTL_HANDLER_ARGS);
extern int sysctl_handle_long(SYSCTL_HANDLER_ARGS);
extern int sysctl_handle_intptr(SYSCTL_HANDLER_ARGS);
extern int sysctl_handle_string(SYSCTL_HANDLER_ARGS);
extern int sysctl_handle_opaque(SYSCTL_HANDLER_ARGS);

/*
 * These functions are used to add/remove an oid from the mib.
 */
extern void sysctl_register_oid(struct sysctl_oid *oidp);
extern void sysctl_unregister_oid(struct sysctl_oid *oidp);

/* Declare a static oid to allow child oids to be added to it. */
#define SYSCTL_DECL(name)					\
	extern struct sysctl_oid_list sysctl_##name##_children

/* Hide these in macros */
#define	SYSCTL_CHILDREN(oid_ptr) \
  ((struct sysctl_oid_list *)(oid_ptr)->oid_arg1)

#define	SYSCTL_CHILDREN_SET(oid_ptr, val) \
  (oid_ptr)->oid_arg1 = (val)

#define	SYSCTL_STATIC_CHILDREN(oid_name) \
  (&sysctl_##oid_name##_children)

/* === Structs and macros related to context handling === */

/* All dynamically created sysctls can be tracked in a context list. */
struct sysctl_ctx_entry {
	struct sysctl_oid *entry;
	TAILQ_ENTRY(sysctl_ctx_entry) link;
};

TAILQ_HEAD(sysctl_ctx_list, sysctl_ctx_entry);

#define SYSCTL_NODE_CHILDREN(parent, name) \
	sysctl_##parent##_##name##_children

#define DECLARE_SYSCTL_DATA(name)				\
	struct sysctl_oid name					\
	__attribute__((__section__("_bsd_sysctl_data_start"),	\
		       __aligned__(1),__used__))

/* This constructs a "raw" MIB oid. */
#define SYSCTL_OID(parent, nbr, name, kind, a1, a2, handler, fmt, descr) \
	static const DECLARE_SYSCTL_DATA(sysctl__##parent##_##name) =	\
	{	&sysctl_##parent##_children, { 0 },			\
		nbr, kind, a1, a2, #name, handler, fmt, 0, descr }

#define SYSCTL_ADD_OID(ctx, parent, nbr, name, kind, a1, a2, handler, fmt, descr) \
	sysctl_add_oid(ctx, parent, nbr, name, kind, a1, a2, handler, fmt, descr)

/* This constructs a node from which other oids can hang. */
#define SYSCTL_NODE(parent, nbr, name, access, handler, descr)		    \
	struct sysctl_oid_list SYSCTL_NODE_CHILDREN(parent, name);	    \
	SYSCTL_OID(parent, nbr, name, CTLTYPE_NODE|(access),		    \
		   (void*)&SYSCTL_NODE_CHILDREN(parent, name), 0, handler, \
		   "N", descr)

#define SYSCTL_ADD_NODE(ctx, parent, nbr, name, access, handler, descr)	    \
	sysctl_add_oid(ctx, parent, nbr, name, CTLTYPE_NODE|(access),	    \
	0, 0, handler, "N", descr)

/* Oid for a string.  len can be 0 to indicate '\0' termination. */
#define SYSCTL_STRING(parent, nbr, name, access, arg, len, descr) \
	SYSCTL_OID(parent, nbr, name, CTLTYPE_STRING|(access), \
		arg, len, sysctl_handle_string, "A", descr)

#define SYSCTL_ADD_STRING(ctx, parent, nbr, name, access, arg, len, descr)  \
	sysctl_add_oid(ctx, parent, nbr, name, CTLTYPE_STRING|(access),	    \
	arg, len, sysctl_handle_string, "A", descr)

/* Oid for an int.  If ptr is NULL, val is returned. */
#define SYSCTL_INT(parent, nbr, name, access, ptr, val, descr) \
	SYSCTL_OID(parent, nbr, name, CTLTYPE_INT|(access), \
		ptr, val, sysctl_handle_int, "I", descr)

#define SYSCTL_ADD_INT(ctx, parent, nbr, name, access, ptr, val, descr)	    \
	sysctl_add_oid(ctx, parent, nbr, name, CTLTYPE_INT|(access),	    \
	ptr, val, sysctl_handle_int, "I", descr)

/* Oid for an unsigned int.  If ptr is NULL, val is returned. */
#define SYSCTL_UINT(parent, nbr, name, access, ptr, val, descr) \
	SYSCTL_OID(parent, nbr, name, CTLTYPE_UINT|(access), \
		ptr, val, sysctl_handle_int, "IU", descr)

#define SYSCTL_ADD_UINT(ctx, parent, nbr, name, access, ptr, val, descr)    \
	sysctl_add_oid(ctx, parent, nbr, name, CTLTYPE_UINT|(access),	    \
	ptr, val, sysctl_handle_int, "IU", descr)

/* Oid for a long.  The pointer must be non NULL. */
#define SYSCTL_LONG(parent, nbr, name, access, ptr, val, descr) \
	SYSCTL_OID(parent, nbr, name, CTLTYPE_LONG|(access), \
		ptr, val, sysctl_handle_long, "L", descr)

#define SYSCTL_ADD_LONG(ctx, parent, nbr, name, access, ptr, descr)	    \
	sysctl_add_oid(ctx, parent, nbr, name, CTLTYPE_LONG|(access),	    \
	ptr, 0, sysctl_handle_long, "L", descr)

/* Oid for an unsigned long.  The pointer must be non NULL. */
#define SYSCTL_ULONG(parent, nbr, name, access, ptr, val, descr) \
	SYSCTL_OID(parent, nbr, name, CTLTYPE_ULONG|(access), \
		ptr, val, sysctl_handle_long, "LU", descr)

#define SYSCTL_ADD_ULONG(ctx, parent, nbr, name, access, ptr, descr)	    \
	sysctl_add_oid(ctx, parent, nbr, name, CTLTYPE_ULONG|(access),	    \
	ptr, 0, sysctl_handle_long, "LU", descr)

/* Oid for an opaque object.  Specified by a pointer and a length. */
#define SYSCTL_OPAQUE(parent, nbr, name, access, ptr, len, fmt, descr) \
	SYSCTL_OID(parent, nbr, name, CTLTYPE_OPAQUE|(access), \
		ptr, len, sysctl_handle_opaque, fmt, descr)

#define SYSCTL_ADD_OPAQUE(ctx, parent, nbr, name, access, ptr, len, fmt, descr)\
	sysctl_add_oid(ctx, parent, nbr, name, CTLTYPE_OPAQUE|(access),	    \
	ptr, len, sysctl_handle_opaque, fmt, descr)

/* Oid for a struct.  Specified by a pointer and a type. */
#define SYSCTL_STRUCT(parent, nbr, name, access, ptr, type, descr) \
	SYSCTL_OID(parent, nbr, name, CTLTYPE_OPAQUE|(access), \
		ptr, sizeof(struct type), sysctl_handle_opaque, \
		"S," #type, descr)

#define SYSCTL_ADD_STRUCT(ctx, parent, nbr, name, access, ptr, type, descr) \
	sysctl_add_oid(ctx, parent, nbr, name, CTLTYPE_OPAQUE|(access),	    \
	ptr, sizeof(struct type), sysctl_handle_opaque, "S," #type, descr)

/* Oid for a procedure.  Specified by a pointer and an arg. */
#define SYSCTL_PROC(parent, nbr, name, access, ptr, arg, handler, fmt, descr) \
	SYSCTL_OID(parent, nbr, name, (access), \
		ptr, arg, handler, fmt, descr)

#define SYSCTL_ADD_PROC(ctx,parent,nbr,name,access,ptr,arg,handler,fmt,descr) \
	sysctl_add_oid(ctx,parent,nbr,name,(access),			\
		       ptr, arg, handler, fmt, descr)

/*
 * Declare some common oids.
 */
extern struct sysctl_oid_list sysctl__children;
SYSCTL_DECL(_kern);
SYSCTL_DECL(_sysctl);
SYSCTL_DECL(_vm);
SYSCTL_DECL(_vfs);
SYSCTL_DECL(_net);
SYSCTL_DECL(_debug);
SYSCTL_DECL(_debug_sizeof);
SYSCTL_DECL(_hw);
SYSCTL_DECL(_hw_bus);
SYSCTL_DECL(_machdep);
SYSCTL_DECL(_user);
SYSCTL_DECL(_compat);

/* Dynamic oid handling */
extern struct sysctl_oid *
sysctl_add_oid(struct sysctl_ctx_list *clist,
	       struct sysctl_oid_list *parent, int nbr, const char *name,
	       int kind, void *arg1, int arg2,
	       int (*handler) (SYSCTL_HANDLER_ARGS),
	       const char *fmt, const char *descr);
extern int
sysctl_move_oid(struct sysctl_oid *oidp,
		struct sysctl_oid_list *parent);
extern int
sysctl_remove_oid(struct sysctl_oid *oidp, int del, int recurse);
extern int
sysctl_ctx_init(struct sysctl_ctx_list *clist);
extern int
sysctl_ctx_free(struct sysctl_ctx_list *clist);
extern struct sysctl_ctx_entry *
sysctl_ctx_entry_add(struct sysctl_ctx_list *clist,
		     struct sysctl_oid *oidp);
extern struct sysctl_ctx_entry *
sysctl_ctx_entry_find(struct sysctl_ctx_list *clist,
		      struct sysctl_oid *oidp);
extern int 
sysctl_ctx_entry_del(struct sysctl_ctx_list *clist,
		     struct sysctl_oid *oidp);

extern int 
kernel_sysctl(struct thread *td, int *name, u_int namelen, void *old,
	      size_t *oldlenp, void *new, size_t newlen,
	      size_t *retval, int flags);
extern int 
kernel_sysctlbyname(struct thread *td, char *name,
		    void *old, size_t *oldlenp, void *new, size_t newlen,
		    size_t *retval, int flags);
extern int 
userland_sysctl(struct thread *td, int *name, u_int namelen, void *old,
		size_t *oldlenp, int inkernel, void *new, size_t newlen,
		size_t *retval, int flags);
extern int 
sysctl_find_oid(int *name, u_int namelen, struct sysctl_oid **noid,
		int *nindx, struct sysctl_req *req);
extern int 
sysctl_wire_old_buffer(struct sysctl_req *req, size_t len);

#endif	/* _KERNEL */

#endif	/* __FREEBSD_SYS_SYSCTL_H__ */
