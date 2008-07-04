S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/ndis

KMOD=  usb2_ndis
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h opt_bus.h

SRCS+= usb2_ndis.c
SRCS+= if_ndis_usb2.c

.include <bsd.kmod.mk>
