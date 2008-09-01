#
# $FreeBSD$
#

S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/misc

KMOD=  usb2_misc
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h opt_bus.h opt_netgraph.h

SRCS+= usb2_misc.c
SRCS+= ufm2.c
SRCS+= udbp2.c

.include <bsd.kmod.mk>
