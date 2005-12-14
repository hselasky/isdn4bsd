# $FreeBSD: src/sys/modules/usb/Makefile,v 1.15 2004/07/12 21:59:06 imp Exp $ 

MAINTAINER=	n_hibma@freebsd.org

S=	${.CURDIR}/../..

.PATH: $S/dev/usb $S/dev/usb2 $S/pci

KMOD=	usb
SRCS=	bus_if.h device_if.h usb_if.h usb_if.c \
	vnode_if.h \
	opt_usb.h \
	hid.c hid.h usbhid.h \
	usb_quirks.c ../usb/usb_quirks.h \
	usb_ethersubr.c usbdevs.h \
	_uhub.c \
	_usb.c ../usb2/usb.h \
	_usb_requests.c \
	_usb_subr.c ../usb2/usb_subr.h \
	_usb_transfer.c \
	../usb2/usb_port.h

SRCS+=	_uhci_pci.c _uhci.c ../usb2/uhci.h
SRCS+=	_ohci_pci.c _ohci.c ../usb2/ohci.h
SRCS+=	_ehci_pci.c _ehci.c ../usb2/ehci.h
SRCS+=	opt_bus.h pci_if.h

.include <bsd.kmod.mk>
