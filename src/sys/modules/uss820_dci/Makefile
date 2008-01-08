# $FreeBSD: 

S=	${.CURDIR}/../..

.PATH: $S/dev/usb

KMOD=	uss820_dci

SRCS=  bus_if.h usb_if.h vnode_if.h   device_if.h  usb_if.c \
    card_if.h opt_usb.h

SRCS+=	uss820_dci_pccard.c uss820_dci.c uss820_dci.h 

.include <bsd.kmod.mk>

