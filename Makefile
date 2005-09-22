#
#
# .include "../Makefile.inc"
#

MAINTAINER=hselasky@c2i.net

.PATH:  ${.CURDIR}/i4b/driver ${.CURDIR}/i4b/layer4 \
	${.CURDIR}/i4b/layer1/ihfc2 ${.CURDIR}/i4b/dss1 \
	${.CURDIR}/i4b/layer1

KMOD=     i4b
MAN=
CFLAGS+=  -Wall
OSNAME!=  uname

# FIX:
# i4b/layer4/i4b_drivers.awk
#
SRCS=

.if (${OSNAME} != "FreeBSD")
SRCS+= \
freebsd_section_start.c \
freebsd_kern_bus.c \
freebsd_i4b_mod.c freebsd_devfs_devs.c freebsd_devfs_vfsops.c \
freebsd_devfs_vnops.c freebsd_kern_mutex.c freebsd_kern_conf.c \
freebsd_subr_eventhandler.c freebsd_kern_sysctl.c freebsd_if.c
.else
SRCS+= \
bus_if.h device_if.h vnode_if.h opt_bus.h pci_if.h
# opt_usb.h usb_if.h usb_if.c
.endif

SRCS+=\
i4b_tel.c  i4b_rbch.c i4b_trace.c \
i4b_ctl.c i4b_i4bdrv.c i4b_capidrv.c i4b_mbuf.c \
i4b_l4mgmt.c i4b_l4.c i4b_ihfc2_drv.c i4b_ihfc2_pnp.c \
i4b_ihfc2_dev.c i4b_ihfc2_l1if.c i4b_hdlc.c dss1_l2fsm.c \
i4b_l1.c

#
# Netgraph is not supported on NetBSD:
#
# i4b_ing.c
#

#
# these files must always
# be last !
#
SRCS+=  i4b_ihfc2_end.c
.if (${OSNAME} != "FreeBSD")
SRCS+=  freebsd_section_end.c
.endif

.if ((${OSNAME} == "OpenBSD") || (${OSNAME} == "MirBSD"))
.include <bsd.lkm.mk>
.else
.include <bsd.kmod.mk>
.endif
