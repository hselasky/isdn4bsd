#
# Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
#
# Temporary module to compile I4B with the second
# generation FreeBSD USB stack. Currently FreeBSD only.
#

MAINTAINER=hselasky@c2i.net

I4B_DIR=${.CURDIR}/../src/sys/i4b
USB_DIR=${.CURDIR}/../src/sys/dev/usb2/core

.PATH:  ${I4B_DIR}/driver ${I4B_DIR}/layer4 \
	${I4B_DIR}/layer1/ihfc3 ${I4B_DIR}/dss1 \
	${I4B_DIR}/layer1 ${USB_DIR}

KMOD=     i4b
MAN=
CFLAGS+=  -Wall -I${.CURDIR}/../src/sys

OSNAME!=  uname
CPUNAME!= uname -m

.if (${CPUNAME} == "amd64")
CFLAGS+=-DFREEBSD_NO_ISA
.endif

.if (${CPUNAME} == "sparc64")
CFLAGS+=-DFREEBSD_NO_ISA
.endif

SRCS=

CFLAGS+= -DIHFC_USB_ENABLED

SRCS+= bus_if.h device_if.h vnode_if.h opt_bus.h pci_if.h isa_if.h
SRCS+= freebsd_fix.c opt_usb.h
SRCS+= usb2_if.h

.if defined(WITH_SPPP)
SRCS+= i4b_isppp.c
.endif

SRCS+=\
i4b_tel.c i4b_ipr.c i4b_rbch.c i4b_trace.c \
i4b_ctl.c i4b_i4bdrv.c i4b_capidrv.c i4b_mbuf.c \
i4b_l4mgmt.c i4b_l4.c i4b_ihfc2_drv.c i4b_ihfc2_pnp.c \
i4b_ihfc2_dev.c i4b_ihfc2_l1if.c i4b_hdlc.c dss1_l2fsm.c \
i4b_l1.c i4b_echo_cancel.c i4b_dtmf.c i4b_convert_xlaw.c \
usb2_config_td.c

SRCS+=  i4b_ihfc2_end.c
.include <bsd.kmod.mk>

