#
# USB system test Makefile
#

.if !defined(SYSDIR)
.if exists(${.CURDIR}/.sysdir)
SYSDIR!=cat .sysdir
.else
SYSDIR="${.CURDIR}/../src/sys"
.endif
.endif

#
# Enable simulator build by default
#
SIMULATOR=YES

.PATH: \
	${SYSDIR}/dev/usb \
	${SYSDIR}/dev/usb/template \
	${SYSDIR}/dev/usb/storage \
	${SYSDIR}/dev/usb/controller \
	${.CURDIR} \
	${.CURDIR}/modules/mtp \
	${.CURDIR}/kern \
	${.CURDIR}/libkern

PROG= usb2kernel
MAN=  # no manual page at the moment
SRCS= 
# CFLAGS += -ffreestanding -nostdlib

CFLAGS += -O -O2 -g -Wall \
 -include ./bsd_module_all.h \
 -nostdinc -I . -I ${SYSDIR} -I /usr/include

.if defined(SIMULATOR)
CFLAGS += -DSIMULATOR
CFLAGS += -DUSB_HW_DRIVER_NAME=\"atmegadci\"
LDFLAGS += -lpthread
.else
CFLAGS += -DUSB_HW_DRIVER_NAME=\"musbotg\"
.endif

.include "Makefile.inc"

.include <bsd.prog.mk>
