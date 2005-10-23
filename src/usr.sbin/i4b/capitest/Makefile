# $FreeBSD: $

PROG=  capitest
MAN=   capitest.8
SRCS=  main.c

DPADD+= ${LIBCAPI20}
LDADD+=	-lcapi20

install:
	@${ECHO} "capitest is not installed automatically"

.include <bsd.prog.mk>
