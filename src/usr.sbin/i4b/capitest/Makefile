# $FreeBSD: $

PROG=  capitest
MAN=   capitest.8
SRCS=  main.c

DPADD+= ${LIBCAPI20}
LDADD+= -lcapi20

DPADD+= ${LIBM}
LDADD+= -lm

.include "../Makefile.sub"
.include <bsd.prog.mk>

