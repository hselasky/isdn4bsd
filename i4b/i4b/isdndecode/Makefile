# $FreeBSD: src/usr.sbin/i4b/isdndecode/Makefile,v 1.5 2001/07/20 06:19:48 obrien Exp $

PROG=	isdndecode
MAN=	isdndecode.8
SRCS=	main.c layer1.c layer2.c layer3.c layer3_subr.c facility.c
CFLAGS+= -I. -I${.CURDIR}/../../../sys/i4b/dss1 -Wall

.include <bsd.prog.mk>

