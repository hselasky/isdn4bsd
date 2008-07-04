S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/quirk

KMOD=  usb2_quirk
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h opt_usb.h opt_bus.h
SRCS+= usb2_quirk.c

.include <bsd.kmod.mk>
