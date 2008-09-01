#
# $FreeBSD$
#

S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/template

KMOD=  usb2_template
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h opt_usb.h opt_bus.h
SRCS+= usb2_template.c
SRCS+= usb2_template_cdce.c
SRCS+= usb2_template_msc.c
SRCS+= usb2_template_mtp.c

.include <bsd.kmod.mk>
