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
CFLAGS += -O -O2 -g -ffreestanding -Wall \
 -include ./bsd_module_all.h \
 -nostdlib -nostdinc -I . -I ${SYSDIR} -I /usr/include

.if defined(SIMULATOR)
CFLAGS += -DSIMULATOR
LDFLAGS += -lc -lc_r -lpthread
.endif

.include "Makefile.inc"

.include <bsd.prog.mk>
