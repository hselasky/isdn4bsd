# $FreeBSD: src/sys/modules/uslcom/Makefile,v 1.2 2008/03/27 20:38:03 antoine Exp $

S=	${.CURDIR}/../..
.PATH: $S/dev/usb

KMOD=	uslcom
SRCS=	uslcom.c opt_usb.h \
	vnode_if.h device_if.h bus_if.h usbdevs.h

.include <bsd.kmod.mk>
