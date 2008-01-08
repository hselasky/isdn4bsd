# $FreeBSD: src/sys/modules/uchcom/Makefile,v 1.1 2007/10/18 10:51:06 takawata Exp $

S=	${.CURDIR}/../..
.PATH: $S/dev/usb

KMOD=	uchcom
SRCS=	uchcom.c opt_usb.h device_if.h bus_if.h usb_if.h vnode_if.h usbdevs.h

.include <bsd.kmod.mk>
