# $FreeBSD: src/sys/modules/usb/Makefile,v 1.18 2005/06/04 10:58:39 schweikh Exp $

S=	${.CURDIR}/../..

.PATH: $S/dev/usb $S/pci

KMOD=	usb
SRCS=	bus_if.h device_if.h usb_if.h usb_if.c vnode_if.h \
	opt_usb.h \
	uhub.c \
	usbdevs.h usbdevs_data.h \
	usb.c usb.h usb_port.h \
	usb_cdev.c \
	usb_hid.c usb_hid.h \
	usb_quirks.c usb_quirks.h \
	usb_requests.c \
	usb_subr.c usb_subr.h \
	usb_transfer.c \
	usb_compat_linux.c \

SRCS+=	uhci_pci.c uhci.c uhci.h 
SRCS+=	ohci_pci.c ohci.c ohci.h 
SRCS+=	ehci_pci.c ehci.c ehci.h 
SRCS+=	opt_bus.h pci_if.h

.include <bsd.kmod.mk>
