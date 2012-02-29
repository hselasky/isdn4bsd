# $FreeBSD: src/usr.sbin/i4b/isdntrace/Makefile,v 1.7 2001/07/20 06:19:49 obrien Exp $

PROG=	isdntrace
MAN=	isdntrace.8
SRCS=	q921.c q931.c q931_util.c q932_fac.c 1tr6.c trace.c unknownl3.c
CFLAGS+= -I${.CURDIR} -I${.CURDIR}/../../../sys/i4b/dss1 -Wall

.include "../Makefile.sub"
.include <bsd.prog.mk>
