# $FreeBSD: src/usr.sbin/i4b/isdnd/Makefile,v 1.12 2003/04/04 17:49:14 obrien Exp $

PROG=	isdnd
MAN=	isdnd.rc.5 isdnd.rates.5 isdnd.acct.5 isdnd.8
SRCS=	rc_parse.y rc_scan.l main.c rc_config.c log.c curses.c \
	rates.c fsm.c support.c exec.c monitor.c alias.c \
	y.tab.h holiday.c

# compile debug support
CFLAGS+= -DDEBUG

# avoid wacky merging of string constants from
# source code with compile-time timestamp
CFLAGS+= -fno-merge-constants

# enable rtprio usage
CFLAGS+= -DUSE_RTPRIO

CFLAGS+= -I. -I${.CURDIR}/../isdnmonitor -I${.CURDIR}/../isdntel -I${.CURDIR}

.include "${.CURDIR}/../Makefile.inc"

.if !defined(I4B_WITHOUT_CURSES)
CFLAGS+= -DUSE_CURSES
DPADD=	${LIBCURSES}
LDADD=	-lcurses
.endif

.if defined(I4B_EXTERNAL_MONITOR)
CFLAGS+= -DI4B_EXTERNAL_MONITOR
.if defined(I4B_NOTCPIP_MONITOR)
CFLAGS+= -DI4B_NOTCPIP_MONITOR
.endif
.endif

.include <bsd.prog.mk>