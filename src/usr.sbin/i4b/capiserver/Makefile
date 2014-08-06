# $FreeBSD: $

PROG=  capiserver
MAN=   capiserver.8
SRCS=  capiserver.c
PTHREAD_LIBS?=  -lpthread

LDFLAGS+= ${PTHREAD_LIBS}

.include "../Makefile.sub"
.include <bsd.prog.mk>
