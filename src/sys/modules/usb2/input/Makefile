#
# $FreeBSD$
#

S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/input

KMOD=  usb2_input
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h opt_bus.h opt_compat.h opt_kbd.h opt_ukbd.h
SRCS+= uhid2.c
SRCS+= ukbd2.c
SRCS+= ums2.c
SRCS+= usb2_input.c

.include <bsd.kmod.mk>

