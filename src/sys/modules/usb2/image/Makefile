S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/image

KMOD=  usb2_image
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h opt_bus.h 

SRCS+= usb2_image.c
SRCS+= uscanner2.c

.include <bsd.kmod.mk>
