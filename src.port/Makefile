.PATH: \
	${.CURDIR} \
	${.CURDIR}/../src/sys/dev/usb2/core \
	${.CURDIR}/../src/sys/dev/usb2/template \
	${.CURDIR}/../src/sys/dev/usb2/storage \
	${.CURDIR}/../src/sys/dev/usb2/controller \
	${.CURDIR}/kern \
	${.CURDIR}/libkern

PROG= usb2kernel
MAN=  # no manual page at the moment
SRCS= 
CFLAGS += -ffreestanding -Wall -include ./bsd_module_all.h -nostdlib -nostdinc -I . -I ../src/sys

.include "Makefile.inc"

.include <bsd.prog.mk>

