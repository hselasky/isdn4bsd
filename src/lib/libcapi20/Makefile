#
# $FreeBSD: $
#
# Makefile for shared CAPI access library
#

LIB=		capi20
SHLIB_MAJOR=	2
SHLIB_MINOR=	0
CFLAGS+=	-Wall 

DPADD+=         ${LIBMD}
LDADD+=         -lmd

SRCS=		capilib.c bintec.c

MKLINT=		no

NOGCCERROR=

MAN=		capi20.3
MLINKS+=	capi20.3 capi.3
MLINKS+=	capi20.3 capi20_be_alloc_bintec.3
MLINKS+=	capi20.3 capi20_be_alloc_i4b.3
MLINKS+=	capi20.3 capi20_be_free.3
MLINKS+=	capi20.3 capi20_register.3
MLINKS+=	capi20.3 capi20_release.3
MLINKS+=	capi20.3 capi20_put_message.3
MLINKS+=	capi20.3 capi20_get_message.3
MLINKS+=	capi20.3 capi20_wait_for_message.3
MLINKS+=	capi20.3 capi20_get_manufacturer.3
MLINKS+=	capi20.3 capi20_get_version.3
MLINKS+=	capi20.3 capi20_get_serial_number.3
MLINKS+=	capi20.3 capi20_get_profile.3
MLINKS+=	capi20.3 capi20_is_installed.3
MLINKS+=	capi20.3 capi20_fileno.3
MLINKS+=	capi20.3 capi20_encode.3
MLINKS+=	capi20.3 capi20_decode.3
MLINKS+=	capi20.3 capi20_get_errstr.3

.include <bsd.lib.mk>
