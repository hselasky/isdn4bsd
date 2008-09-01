#
# $FreeBSD$
#

S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/ethernet

KMOD=  usb2_ethernet
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h miibus_if.h opt_inet.h opt_bus.h
SRCS+= if_aue2.c
SRCS+= if_axe2.c
SRCS+= if_cdce2.c
SRCS+= if_cue2.c
SRCS+= if_kue2.c
SRCS+= if_rue2.c
SRCS+= if_udav2.c
SRCS+= usb2_ethernet.c

.include <bsd.kmod.mk>

