#
# Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
#
# .include "../Makefile.inc"
#

MAINTAINER=hselasky@c2i.net

I4B_DIR=${.CURDIR}/../src/sys/i4b
USB_DIR=${.CURDIR}/../src/sys/dev/usb

.PATH:  ${I4B_DIR}/driver ${I4B_DIR}/layer4 \
	${I4B_DIR}/layer1/ihfc2 ${I4B_DIR}/dss1 \
	${I4B_DIR}/layer1 ${USB_DIR}

KMOD=     i4b
MAN=
CFLAGS+=  -Wall -I${.CURDIR}/../src/sys
#
# NOTE: only enable the following flags
# if they are in the kernel:
#
#CFLAGS+= -DDEBUG -g
#CFLAGS+= -DDIAGNOSTIC
CFLAGS+= -DMULTIPROCESSOR
#
# Uncomment to enable USB:
#
#WITH_USB=
#
#
OSNAME!=  uname
CPUNAME!= uname -m

.if (${CPUNAME} == "sparc64")
CFLAGS+=-DFREEBSD_NO_ISA
.endif

# FIX:
# i4b/layer4/i4b_drivers.awk
#
SRCS=

.if (${OSNAME} != "FreeBSD")
CFLAGS+=-include ${.CURDIR}/fbsdid.h
SRCS+= \
freebsd_section_start.c \
freebsd_kern_bus.c \
freebsd_i4b_mod.c freebsd_devfs_devs.c freebsd_devfs_vfsops.c \
freebsd_devfs_vnops.c freebsd_kern_mutex.c freebsd_kern_conf.c \
freebsd_subr_eventhandler.c freebsd_kern_sysctl.c freebsd_if.c \
freebsd_kern_timeout.c freebsd_kern_mbuf.c

.if defined(WITH_USB)
#
# Generic EHCI / UHCI / OHCI USB controller driver
#
SRCS+= \
ehci.c ehci_pci.c ohci.c ohci_pci.c uhci.c uhci_pci.c \
ugen.c uhub.c usb.c usb_requests.c usb_subr.c usb_transfer.c \
usb_quirks.c

CFLAGS+= -DIHFC_USB_ENABLED
.endif

.else
SRCS+= \
bus_if.h device_if.h vnode_if.h opt_bus.h pci_if.h
# opt_usb.h usb_if.h usb_if.c
.endif

SRCS+=\
i4b_tel.c i4b_ipr.c i4b_isppp.c i4b_rbch.c i4b_trace.c \
i4b_ctl.c i4b_i4bdrv.c i4b_capidrv.c i4b_mbuf.c \
i4b_l4mgmt.c i4b_l4.c i4b_ihfc2_drv.c i4b_ihfc2_pnp.c \
i4b_ihfc2_dev.c i4b_ihfc2_l1if.c i4b_hdlc.c dss1_l2fsm.c \
i4b_l1.c i4b_echo_cancel.c i4b_dtmf.c i4b_convert_xlaw.c

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
