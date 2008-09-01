#
# $FreeBSD$
#

S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/storage

KMOD=  usb2_storage
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h opt_bus.h opt_cam.h opt_ata.h ata_if.h

SRCS+= usb2_storage.c

#
# NOTE: ATA-USB is not compatible with UMASS
#

.if defined(HAVE_ATAUSB)
SRCS+= ata-usb2.c
.else
SRCS+= umass2.c
.endif
SRCS+= urio2.c
SRCS+= ustorage2_fs.c

.include <bsd.kmod.mk>
