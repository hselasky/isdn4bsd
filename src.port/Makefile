.PATH: \
	${.CURDIR} \
	${.CURDIR}/../src/sys/dev/usb2/core \
	${.CURDIR}/../src/sys/dev/usb2/template \
	${.CURDIR}/../src/sys/dev/usb2/storage \
	${.CURDIR}/../src/sys/dev/usb2/controller \
	${.CURDIR}/modules/mtp \
	${.CURDIR}/kern \
	${.CURDIR}/libkern

PROG= usb2kernel
MAN=  # no manual page at the moment
SRCS= 
CFLAGS += -O2 -g -ffreestanding -Wall -include ./bsd_module_all.h -nostdlib -nostdinc -I . -I ../src/sys
SIGBASED_IO=

.include "Makefile.inc"

.include <bsd.prog.mk>
