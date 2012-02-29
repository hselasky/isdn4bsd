# $FreeBSD: src/usr.sbin/i4b/isdnmonitor/Makefile,v 1.7 2003/04/04 17:49:14 obrien Exp $

PROG=	isdnmonitor
MAN=	isdnmonitor.8
SRCS=	main.c curses.c

# compile debug support
CFLAGS+= -DDEBUG

# avoid wacky merging of string constants from
# source code with compile-time timestamp
CFLAGS+= -fno-merge-constants

DPADD=	${LIBCURSES}
LDADD=	-lcurses

.include "../Makefile.sub"
.include <bsd.prog.mk>
