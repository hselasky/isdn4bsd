#
# Copyright (c) 2012 Hans Petter Selasky. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# ISDN4BSD toplevel Makefile
#

VERSION=2.0.1

KMODNAME?=i4b

.if defined(HAVE_KMOD) || defined(HAVE_ALL)
SUBDIR+= module
.endif

.if defined(HAVE_UTILS) || defined(HAVE_ALL) || defined(HAVE_MAN)
SUBDIR+= src/usr.sbin
.endif

.include <bsd.subdir.mk>

CONFIG=${.CURDIR}/Makefile.sub
CAPI20=${.CURDIR}/src/sys/i4b/include/capi20.h

cleanconfig:
	rm -f ${CONFIG} ${CAPI20}

configure: cleanconfig

	touch ${CONFIG}
	ln -s ${INCLUDEDIR}/capi20.h ${CAPI20}

	echo "KMODDIR=${KMODDIR}" >> ${CONFIG}
	echo "KMODNAME=${KMODNAME}" >> ${CONFIG}
	echo "BINDIR=${BINDIR}" >> ${CONFIG}
	echo "INCLUDEDIR=${INCLUDEDIR}" >> ${CONFIG}
	echo "MANDIR=${MANDIR}" >> ${CONFIG}
	echo "CFLAGS+= -I${.CURDIR}/src/sys" >> ${CONFIG}
	echo "CFLAGS+= -D_GNU_SOURCE" >> ${CONFIG}
	echo "CFLAGS+= -L${LIBDIR}" >> ${CONFIG}
	echo "MKLINT=no" >> ${CONFIG}
	echo "WARNS=3" >> ${CONFIG}
	echo "NO_WERROR=" >> ${CONFIG}
	echo "NOGCCERROR=" >> ${CONFIG}
	echo "NO_PROFILE=" >> ${CONFIG}
	echo "WERROR=" >> ${CONFIG}

.if defined(HAVE_DEBUG) || defined(HAVE_ALL)
	echo "HAVE_DEBUG=1" >> ${CONFIG}
	echo "CFLAGS+= -g" >> ${CONFIG}
.endif
.if defined(HAVE_CAPIMONITOR) || defined(HAVE_ALL)
	echo "HAVE_CAPIMONITOR=capimonitor" >> ${CONFIG}
.endif
.if defined(HAVE_CAPITEST) || defined(HAVE_ALL)
	echo "HAVE_CAPITEST=capitest" >> ${CONFIG}
.endif
.if defined(HAVE_DTMFDECODE) || defined(HAVE_ALL)
	echo "HAVE_DTMFDECODE=dtmfdecode" >> ${CONFIG}
.endif
.if defined(HAVE_G711CONV) || defined(HAVE_ALL)
	echo "HAVE_G711CONV=g711conv" >> ${CONFIG}
.endif
.if defined(HAVE_ISDNCONFIG) || defined(HAVE_ALL)
	echo "HAVE_ISDNCONFIG=isdnconfig" >> ${CONFIG}
.endif
.if defined(HAVE_ISDNDEBUG) || defined(HAVE_ALL)
	echo "HAVE_ISDNDEBUG=isdndebug" >> ${CONFIG}
.endif
.if defined(HAVE_ISDNDECODE) || defined(HAVE_ALL)
	echo "HAVE_ISDNDECODE=isdndecode" >> ${CONFIG}
.endif
.if defined(HAVE_ISDND) || defined(HAVE_ALL)
	echo "HAVE_ISDND=isdnd" >> ${CONFIG}
.endif
.if defined(HAVE_ISDNMONITOR) || defined(HAVE_ALL)
	echo "HAVE_ISDNMONITOR=isdnmonitor" >> ${CONFIG}
.endif
.if defined(HAVE_ISDNTELCTL) || defined(HAVE_ALL)
	echo "HAVE_ISDNTELCTL=isdntelctl" >> ${CONFIG}
.endif
.if defined(HAVE_ISDNTEL) || defined(HAVE_ALL)
	echo "HAVE_ISDNTEL=isdntel" >> ${CONFIG}
.endif
.if defined(HAVE_ISDNTEST) || defined(HAVE_ALL)
	echo "HAVE_ISDNTEST=isdntest" >> ${CONFIG}
.endif
.if defined(HAVE_ISDNTRACE) || defined(HAVE_ALL)
	echo "HAVE_ISDNTRACE=isdntrace" >> ${CONFIG}
.endif
.if defined(HAVE_MAN) || defined(HAVE_ALL)
	echo "HAVE_MAN=man" >> ${CONFIG}
.endif
.if defined(HAVE_NO_MAN)
	echo "MK_MAN=no" >> ${CONFIG}
.endif
.if defined(HAVE_MOUNT_DEVFS) || defined(HAVE_ALL)
	echo "HAVE_MOUNT_DEVFS=mount_devfs" >> ${CONFIG}
.endif
.if defined(I4B_EXTERNAL_MONITOR)
	echo "I4B_EXTERNAL_MONITOR=1" >> ${CONFIG}
.endif
.if defined(I4B_WITHOUT_CURSES)
	echo "I4B_WITHOUT_CURSES=1" >> ${CONFIG}
.endif
.if defined(HAVE_IPR_VJ)
	echo "HAVE_IPR_VJ=1" >> ${CONFIG}
.endif
.if defined(I4B_NOTCPIP_MONITOR)
	echo "I4B_NOTCPIP_MONITOR=1" >> ${CONFIG}
.endif
.if defined(HAVE_ISDN_HFC_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_ISDN_HFC_DRIVER=1" >> ${CONFIG}
.endif
.if defined(HAVE_ISDN_LOOP_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_ISDN_LOOP_DRIVER=1" >> ${CONFIG}
.endif
.if defined(HAVE_YEALINK_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_YEALINK_DRIVER=1" >> ${CONFIG}
.endif
.if defined(HAVE_SPPP_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_SPPP_DRIVER=1" >> ${CONFIG}
.endif
.if defined(HAVE_TEL_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_TEL_DRIVER=1" >> ${CONFIG}
.endif
.if defined(HAVE_IPR_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_IPR_DRIVER=1" >> ${CONFIG}
.endif
.if defined(HAVE_ING_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_ING_DRIVER=1" >> ${CONFIG}
.endif
.if defined(HAVE_RBCH_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_RBCH_DRIVER=1" >> ${CONFIG}
.endif
.if defined(HAVE_TRACE_DRIVER) || defined(HAVE_ALL)
	echo "HAVE_TRACE_DRIVER=1" >> ${CONFIG}
.endif

package:

	make configure HAVE_ALL=YES
	make clean cleandepend HAVE_ALL=YES
	make cleanconfig

	tar -cvf temp.tar --exclude="*~" --exclude="*#" \
		--exclude=".svn" --exclude="*.orig" --exclude="*.rej" \
		--exclude="temp" \
		Makefile Makefile.sub README.TXT examples module/Makefile src

	rm -rf isdn4bsd-${VERSION}

	mkdir isdn4bsd-${VERSION}

	tar -xvf temp.tar -C isdn4bsd-${VERSION}

	rm -rf temp.tar

	tar -jcvf isdn4bsd-${VERSION}.tar.bz2 isdn4bsd-${VERSION}

