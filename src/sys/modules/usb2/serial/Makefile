#
# $FreeBSD$
#

S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/serial

KMOD=  usb2_serial
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h opt_bus.h opt_compat.h
SRCS+= usb2_serial.c

SRCS+= uark2.c
SRCS+= ubsa2.c
SRCS+= ubser2.c
SRCS+= uchcom2.c
SRCS+= ucycom2.c
SRCS+= ufoma2.c
SRCS+= uftdi2.c
SRCS+= ugensa2.c
SRCS+= uipaq2.c
SRCS+= ulpt2.c
SRCS+= umct2.c
SRCS+= umodem2.c
SRCS+= umoscom2.c
SRCS+= uplcom2.c
SRCS+= uvisor2.c
SRCS+= uvscom2.c

.include <bsd.kmod.mk>
