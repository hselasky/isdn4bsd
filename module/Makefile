#
# Copyright (c) 2012 Hans Petter Selasky. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
#
# Makefile for I4B kernel module
#

I4B_DIR=${.CURDIR}/../src/sys/i4b

.PATH:  \
	${I4B_DIR}/dss1 \
	${I4B_DIR}/layer1 \
	${I4B_DIR}/layer1/ihfc3 \
	${I4B_DIR}/layer1/iloop \
	${I4B_DIR}/layer1/yealink \
	${I4B_DIR}/layer4 \
	${I4B_DIR}/driver

.include "../Makefile.sub"

KMODNAME?=i4b
KMOD=${KMODNAME}
MAN=

.if defined(MACHINE_CPUARCH)
.if (${MACHINE_CPUARCH} != "i386")
CFLAGS+=-DFREEBSD_NO_ISA
.endif
.endif

.if defined(MACHINE_ARCH)
.if (${MACHINE_ARCH} != "i386")
CFLAGS+=-DFREEBSD_NO_ISA
.endif
.endif

SRCS=

SRCS+= bus_if.h
SRCS+= device_if.h
SRCS+= isa_if.h
SRCS+= opt_bus.h
SRCS+= opt_usb.h
SRCS+= pci_if.h
SRCS+= usb_if.h
SRCS+= vnode_if.h

SRCS+= dss1_l2fsm.c
SRCS+= i4b_capidrv.c
SRCS+= i4b_convert_xlaw.c
SRCS+= i4b_ctl.c
SRCS+= i4b_dtmf.c
SRCS+= i4b_echo_cancel.c
SRCS+= i4b_hdlc.c
SRCS+= i4b_i4bdrv.c
SRCS+= i4b_l1.c
SRCS+= i4b_l4.c
SRCS+= i4b_l4mgmt.c
SRCS+= i4b_mbuf.c
SRCS+= usb2_config_td.c

.if defined(HAVE_ISDN_HFC_DRIVER)
SRCS+= i4b_ihfc2_drv.c
SRCS+= i4b_ihfc2_pnp.c
SRCS+= i4b_ihfc2_dev.c
SRCS+= i4b_ihfc2_l1if.c
SRCS+= i4b_ihfc2_end.c
.endif

.if defined(HAVE_ISDN_LOOP_DRIVER)
SRCS+= iloop.c
.endif

.if !defined(HAVE_YEALINK_DRIVER)
SRCS+= yealink.c
SRCS+= dss1_lite.c dss1_lite_l1if.c
.endif

.if defined(HAVE_SPPP_DRIVER)
SRCS+= i4b_isppp.c
.endif

.if defined(HAVE_TEL_DRIVER)
SRCS+= i4b_tel.c
.endif

.if defined(HAVE_IPR_DRIVER)
.if defined(HAVE_IPR_VJ)
CFLAGS+= -DIPR_VJ
.endif
SRCS+= i4b_ipr.c
.endif

.if defined(HAVE_RBCH_DRIVER)
SRCS+= i4b_rbch.c
.endif

.if defined(HAVE_TRACE_DRIVER)
SRCS+= i4b_trace.c
.endif

.if defined(HAVE_ING_DRIVER)
SRCS+= i4b_ing.c
.endif

.include <bsd.kmod.mk>
