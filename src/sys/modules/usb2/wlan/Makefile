#
# $FreeBSD$
#

S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/wlan

KMOD=  usb2_wlan
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h miibus_if.h opt_inet.h opt_bus.h
SRCS+= if_rum2.c
SRCS+= if_ural2.c
SRCS+= if_zyd2.c
SRCS+= usb2_wlan.c

.include <bsd.kmod.mk>
