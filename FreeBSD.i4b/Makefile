VERSION=1_6_1
DATE=June 2006

FILTER= grep -v "temp"   | \
	grep -v "~"      | \
	grep -v "\#"     | \
	grep -v "\.orig" | \
	grep -v "/\.svn" 

#
# If you want to install the
# driver in another place, just
# change the "S" variable below:
#

.if !defined(S)
S       = /usr/src
.endif
INCLUDE = /usr/include
TEMP    = tempfile
DIFF    = new_ihfc_${VERSION}.diff.bz2
PACKAGE = new_ihfc_${VERSION}.tar.bz2
BACKUP  = i4b_system_backup_${VERSION}.tar.bz2
N       =\n
SITE    = http://www.turbocat.net/~hselasky/isdn4bsd/releases
ARCH   := uname -m
CONF_DIR=${S}/sys/${ARCH}/conf

.if !exists(${CONF_DIR}/GENERIC)
CONF_DIR=${S}/sys/i386/conf
.endif

REMOVE_FILES=\
${S}/sys/i386/include/i4b*h \
${INCLUDE}/machine/i4b*h \
${INCLUDE}/dev/usb2/*h \
${INCLUDE}/i4b/include/*h \

I4B_FILES=\
${S}/lib/libcapi20 \
${S}/sys/i4b \
${S}/usr.sbin/i4b \

#
# some of these files and directories
# might need to be updated one day:
#
#${S}/etc/rc.d/isdnd \
#${S}/etc/rc.isdn \
#${S}/etc/isdn \
#${S}/share/examples/isdn \
#${S}/share/examples/isdn/FAQ \
#${S}/share/examples/isdn/Overview \
#${S}/share/isdn \
#${S}/share/doc/en_US.ISO_8859-1/books/handbook/isdn.html
#

USB_FILES=\
${S}/sys/modules/usb/Makefile \
${S}/sys/modules/ugen/Makefile \
${S}/sys/modules/ums/Makefile \
${S}/sys/modules/ukbd/Makefile \
${S}/sys/modules/ulpt/Makefile \
${S}/sys/modules/uhid/Makefile \
${S}/sys/tools/usbdevs2h.awk \
${S}/sys/dev/usb/usb.h \
${S}/sys/dev/usb/usb_port.h \
${S}/sys/dev/usb/usbdi.h \
${S}/sys/dev/usb/usbdi_util.h \
${S}/sys/dev/usb/usbdivar.h \
${S}/sys/dev/usb/hid.h \
${S}/sys/dev/usb/usbhid.h \
${S}/sys/dev/usb2 \

USB_MODULES=\
${S}/sys/modules/aue \
${S}/sys/modules/axe \
${S}/sys/modules/cdce \
${S}/sys/modules/cue \
${S}/sys/modules/if_ndis \
${S}/sys/modules/kue \
${S}/sys/modules/ndis \
${S}/sys/modules/netgraph/bluetooth/ubtbcmfw \
${S}/sys/modules/netgraph/bluetooth/ubt \
${S}/sys/modules/rue \
${S}/sys/modules/sound/driver/uaudio \
${S}/sys/modules/ubsa \
${S}/sys/modules/ubser \
${S}/sys/modules/ucom \
${S}/sys/modules/ucycom \
${S}/sys/modules/udav \
${S}/sys/modules/udbp \
${S}/sys/modules/ufm \
${S}/sys/modules/uftdi \
${S}/sys/modules/ugen \
${S}/sys/modules/uhid \
${S}/sys/modules/ukbd \
${S}/sys/modules/ulpt \
${S}/sys/modules/umass \
${S}/sys/modules/umct \
${S}/sys/modules/umodem \
${S}/sys/modules/ums \
${S}/sys/modules/uplcom \
${S}/sys/modules/ural \
${S}/sys/modules/urio \
${S}/sys/modules/usb \
${S}/sys/modules/uscanner \
${S}/sys/modules/uvisor \
${S}/sys/modules/uvscom \

CONFIG_FILES=\
${S}/sys/conf/files \
${S}/sys/conf/files.pc98 \
${S}/sys/conf/files.i386 \

HEADER_FIX_FILES=\
${S}/usr.sbin/ppp/i4b.c \
${S}/share/examples/isdn/v21/v21modem.c \
${S}/share/examples/isdn/contrib/hplay.c \
${S}/share/examples/isdn/contrib/isdntelmux.c \
${S}/share/examples/isdn/i4brunppp/i4brunppp.c \

DIFF_FILES=\
${HEADER_FIX_FILES} \
${S}/sys/i386/include/bus_at386.h \
${S}/sys/conf/kmod.mk \
${S}/sys/dev/sound/usb/uaudio.c \
${S}/sys/dev/sound/pcm/sound.c \
${S}/sys/dev/sound/pcm/vchan.c \
${S}/sys/dev/sound/pcm/channel.c \
${S}/sys/dev/sound/pcm/channel.h \
${S}/sys/dev/sound/pcm/dsp.c \
${S}/sys/dev/sound/isa/sb16.c \
${S}/sys/dev/sound/isa/sb8.c \
${S}/sys/dev/usb/if_ural.c \
${S}/sys/dev/usb/ufoma.c \
${S}/sys/dev/usb/ums.c \
${S}/sys/dev/usb/udbp.c \
${S}/sys/dev/usb/ukbd.c \
${S}/sys/dev/usb/umass.c \
${S}/sys/dev/usb/umodem.c \
${S}/sys/dev/usb/uvisor.c \
${S}/sys/dev/usb/ucycom.c \
${S}/sys/dev/usb/hid.c \
${S}/sys/dev/usb/usb_quirks.c \
${S}/sys/dev/usb/usbdevs \
${S}/etc/mtree/BSD.include.dist \
${S}/include/Makefile \
${S}/usr.sbin/Makefile.inc \
${S}/lib/Makefile \

BACKUP_FILES=\
${REMOVE_FILES} \
${I4B_FILES} \
${USB_FILES} \
${CONFIG_FILES} \
${DIFF_FILES} \

PACKAGE_FILES=`find ${I4B_FILES} ${USB_FILES} | ${FILTER}`

FILES_APPEND='\
${N}\# \
${N}\# ISDN4BSD system files \
${N}i4b_drivers.o                       optional i4b dependency \"$$S/i4b/layer4/i4b_drivers.awk Makefile\" compile-with \"$${AWK} -f $${.ALLSRC} | $${CC} -c -o $${.TARGET} $${ASM_CFLAGS} $${WERROR} -\" no-implicit-rule clean \"i4b_drivers.o\" \
${N}i4b/driver/i4b_trace.c              optional i4btrc \
${N}i4b/driver/i4b_rbch.c               optional i4brbch \
${N}i4b/driver/i4b_tel.c                optional i4btel \
${N}i4b/driver/i4b_ing.c                optional i4bing \
${N}i4b/driver/i4b_ipr.c                optional i4bipr \
${N}i4b/driver/i4b_isppp.c              optional i4bisppp \
${N}i4b/driver/i4b_ctl.c                optional i4bctl \
${N}net/slcompress.c                    optional i4bipr \
${N}net/slcompress.c                    optional i4bisppp \
${N}net/if_spppsubr.c                   optional i4bisppp \
${N}net/if_spppfr.c                     optional i4bisppp \
${N}i4b/dss1/dss1_l2fsm.c               optional i4b \
${N}i4b/dss1/dss1_l2fsm.c               optional i4bdss1 \
${N}i4b/layer4/i4b_capidrv.c            optional i4b \
${N}i4b/layer4/i4b_i4bdrv.c             optional i4b \
${N}i4b/layer4/i4b_l4.c                 optional i4b \
${N}i4b/layer4/i4b_l4mgmt.c             optional i4b \
${N}i4b/layer4/i4b_mbuf.c               optional i4b \
${N}i4b/layer1/i4b_l1.c                 optional i4b \
${N}i4b/layer1/i4b_echo_cancel.c        optional i4b \
${N}i4b/layer1/i4b_convert_xlaw.c       optional i4b \
${N}i4b/layer1/i4b_hdlc.c               optional i4b \
${N}\# \
${N}\# ISDN4BSD driver files \
${N}i4b/layer1/ihfc2/i4b_ihfc2_l1if.c   optional ihfc \
${N}i4b/layer1/ihfc2/i4b_ihfc2_pnp.c    optional ihfc \
${N}i4b/layer1/ihfc2/i4b_ihfc2_drv.c    optional ihfc \
${N}i4b/layer1/ihfc2/i4b_ihfc2_dev.c    optional ihfc \
${N}i4b/layer1/ihfc2/i4b_ihfc2_end.c    optional ihfc \
${N}\# \
${N}\# USB support \
${N}dev/usb2/_uhci.c            optional usb \
${N}dev/usb2/_uhci_pci.c        optional usb \
${N}dev/usb2/_uhci.c            optional uhci \
${N}dev/usb2/_uhci_pci.c        optional uhci \
${N}dev/usb2/_ohci.c            optional usb \
${N}dev/usb2/_ohci_pci.c        optional usb \
${N}dev/usb2/_ohci.c            optional ohci \
${N}dev/usb2/_ohci_pci.c        optional ohci \
${N}dev/usb2/_ehci.c            optional ehci \
${N}dev/usb2/_ehci_pci.c        optional ehci \
${N}dev/usb2/_uhub.c            optional usb \
${N}dev/usb2/_usb.c             optional usb \
${N}dev/usb2/_usb_requests.c    optional usb \
${N}dev/usb2/_usb_subr.c        optional usb \
${N}dev/usb2/_usb_transfer.c    optional usb \
${N}dev/usb2/_usb_cdev.c        optional usb \
${N}dev/usb2/_usb_hid.c         optional usb \
${N}dev/usb2/_ugen.c            optional ugen \
${N}dev/usb2/_ulpt.c            optional ulpt \
${N}dev/usb2/_ums.c             optional ums \
${N}dev/usb2/_ukbd.c            optional ukbd \
${N}dev/usb2/_uhid.c            optional uhid \
${N}usbdevs.h                   optional usb dependency \"$$S/tools/usbdevs2h.awk $$S/dev/usb/usbdevs\" compile-with \"$${AWK} -f $$S/tools/usbdevs2h.awk $$S/dev/usb/usbdevs\" no-obj no-implicit-rule before-depend clean \"usbdevs.h\" \
${N}'

FILES_I386_PC98_APPEND=''

CHANNEL_H_APPEND='\
${N}void chn_rel(struct cdev *dev, struct pcm_channel *rdch, struct pcm_channel *wrch, u_int32_t prio);\
${N}int chn_get(struct cdev *dev, struct pcm_channel **rdch, struct pcm_channel **wrch, u_int32_t prio);\
${N}'

#
# "ше" are substituted for 
# "\n" and "\t", respectivly
#

M=ше
BSD_INCLUDE_DIST_APPEND_1="${M}usb2${M}.."
BSD_INCLUDE_DIST_APPEND_2="еi4b${M}  include${M}  ..${M}..ш.."

HELP='\#\
${N}\# ISDN4BSD v.${VERSION:C/_/./g} for FreeBSD 5/6/7 - ${DATE} \
${N}\# \
${N}\# source directory:      ${S} \
${N}\# backup directory:      ${.CURDIR} \
${N}\# targets:               help fetch all install install_mod deinstall clean \
${N}\# \
${N}\# \
${N}\# 1. Install this package:\
${N}\#    \"make install\" \
${N}\# \
${N}\# 2. Recompile and install all modules \
${N}\#    that are depending on this package: \
${N}\#    \"make install_mod\" \
${N}\# \
${N}\# 3. Make sure that the following lines are  \
${N}\#    in your kernel configuration file, \
${N}\#    for example \"${CONF_DIR}/custom\": \
${N}\# \
${N}\# I4B section \
${N}\# \
${N}options IPR_VJ \
${N}device \"i4bdss1\" \
${N}device \"i4b\" \
${N}device \"i4btrc\" \
${N}device \"i4bctl\" \
${N}device \"i4brbch\" \
${N}device \"i4btel\" \
${N}device \"i4bipr\" \
${N}device \"i4bisppp\" \
${N}\# \
${N}\# If you need more than 8 units please \
${N}\# edit \"${S}/sys/i4b/include/i4b_global.h\", \
${N}\# until further. \
${N}\# \
${N}\
${N}device ihfc  \
${N}device usb \
${N}device pcm \
${N}\#or device sound \
${N}\#if device pcm does not exist \
${N}\
${N}\# \
${N}\# 4. Recompile and install the kernel:  \
${N}\#    \"cd ${S} && make buildkernel installkernel  \
${N}\#    KERNCONF=custom -DNO_MODULES -DNOCLEAN -DNO_CLEAN\" \
${N}\# \
${N}\#    The second time you run make  \
${N}\#    you can add -DNO_KERNELDEPEND  \
${N}\#    to speed up the compilation. \
${N}\# \
${N}\#    If the system does not boot, you \
${N}\#    might want to compile the kernel without \
${N}\#    \"-DNO_MODULES\" \
${N}\# \
${N}\# 5. Write down the kernel backup location, \
${N}\#    \"${.CURDIR}/kernel\", and reboot. \
${N}\# \
${N}\
${N}\# \
${N}\# In case of trouble you can send an e-mail to: \
${N}\# hselasky@c2i.net \
${N}\
${N}\# Type \"make help\" to display this message again \
${N}'

help:
	@printf ${HELP}

fetch:
	fetch ${SITE}/FreeBSD/${VERSION}/${DIFF}
	fetch ${SITE}/FreeBSD/${VERSION}/${PACKAGE}

backup:
	@echo ""
	@echo "Installing sources to \"${S}\""
	@echo "Installing headers to \"${INCLUDE}\""
	@echo ""
	@echo "Kernel backup location is \"${.CURDIR}/kernel\""
	@echo ""

	@sleep 10

.if exists(${BACKUP}.dont.remove)

	@echo "Backup already exists ... continuing"

.else

	@echo ""
	@echo "Backup in progress ..."
	@echo ""

#
# "/etc/isdn/answer" needs the
# following directory:
#

	mkdir /var/isdn                || echo -n

#
# make directory tree
#
	mkdir ${S}                     || echo -n
	mkdir ${S}/lib                 || echo -n
	mkdir ${S}/lib/libcapi20       || echo -n
	mkdir ${S}/usr.sbin            || echo -n
	mkdir ${S}/usr.sbin/i4b        || echo -n
	mkdir ${S}/usr.sbin/ppp        || echo -n
	mkdir ${S}/sys                 || echo -n
	mkdir ${S}/sys/i4b             || echo -n
	mkdir ${S}/sys/dev             || echo -n
	mkdir ${S}/sys/dev/sound       || echo -n
	mkdir ${S}/sys/dev/sound/isa   || echo -n
	mkdir ${S}/sys/dev/sound/pcm   || echo -n
	mkdir ${S}/sys/dev/sound/usb   || echo -n
	mkdir ${S}/sys/dev/usb         || echo -n
	mkdir ${S}/sys/dev/usb2        || echo -n
	mkdir ${S}/sys/modules         || echo -n
	mkdir ${S}/sys/modules/ugen    || echo -n
	mkdir ${S}/sys/modules/uhid    || echo -n
	mkdir ${S}/sys/modules/ukbd    || echo -n
	mkdir ${S}/sys/modules/ulpt    || echo -n
	mkdir ${S}/sys/modules/ums     || echo -n
	mkdir ${S}/sys/modules/usb     || echo -n
	mkdir ${S}/sys/conf            || echo -n
	mkdir ${S}/sys/i386            || echo -n
	mkdir ${S}/sys/i386/include    || echo -n
	mkdir ${S}/sys/tools           || echo -n
	mkdir ${S}/etc                 || echo -n
	mkdir ${S}/etc/mtree           || echo -n
	mkdir ${S}/include             || echo -n
	mkdir ${S}/share               || echo -n
	mkdir ${S}/share/examples      || echo -n
	mkdir ${S}/share/examples/isdn || echo -n
	mkdir ${S}/share/examples/isdn/v21 || echo -n
	mkdir ${S}/share/examples/isdn/contrib || echo -n
	mkdir ${S}/share/examples/isdn/i4brunppp || echo -n

	mkdir ${INCLUDE}               || echo -n
	mkdir ${INCLUDE}/i4b           || echo -n
	mkdir ${INCLUDE}/i4b/include   || echo -n
	mkdir ${INCLUDE}/dev           || echo -n
	mkdir ${INCLUDE}/dev/usb2      || echo -n
	mkdir ${INCLUDE}/machine       || echo -n

#
# create missing files
#

.for FILE in \
${S}/sys/i386/include/i4b_ioctl.h \
${INCLUDE}/i4b/include/i4b_ioctl.h \
${INCLUDE}/dev/usb2/usb.h \
${INCLUDE}/machine/i4b_ioctl.h \
${I4B_FILES} \
${USB_FILES} \
${CONFIG_FILES} \
${DIFF_FILES}
	@ [ -f ${FILE} ] || [ -d ${FILE} ] || touch ${FILE} || echo -n
.endfor

#
# try to cleanup the source tree
# before it is backed up
#

	(cd ${S}/usr.sbin/i4b && make clean) || echo -n
	(cd ${S}/lib/libcapi20 && make clean) || echo -n

#
# do backup
#

	cp -v /boot/kernel/kernel .

	tar -jcvf ${BACKUP} -C/ ${BACKUP_FILES}

	@touch ${BACKUP}.dont.remove

	sync
.endif


update_pcm:

	sed -e s/getchns/chn_get/g \
	    -e s/relchns/chn_rel/g \
	    ${S}/sys/dev/sound/pcm/dsp.c \
	| sed -e s/"chn_get(struct cdev \*"/"chn_get(dev_t "/g \
	      -e s/"chn_rel(struct cdev \*"/"chn_rel(dev_t "/g \
	> ${TEMP}

	cat ${TEMP} > ${S}/sys/dev/sound/pcm/dsp.c

	sed -e s/"MTX_DEF)"/"MTX_DEF|MTX_RECURSE)"/g \
	    -e s/"d->flags = 0;"/""/g \
	    ${S}/sys/dev/sound/pcm/sound.c \
	> ${TEMP}

	cat ${TEMP} > ${S}/sys/dev/sound/pcm/sound.c

	cat ${S}/sys/dev/sound/pcm/channel.h \
	| grep -v " chn_rel(" \
	| grep -v " chn_get(" \
	| cat - \
	> ${TEMP}

	@printf ${CHANNEL_H_APPEND} >> ${TEMP}

	cat ${TEMP} > ${S}/sys/dev/sound/pcm/channel.h


update_config:

#
# cleanup
#

.for FILE in ${CONFIG_FILES}
	cat ${FILE} \
	| grep -v "ihfc | isic | itjc | iwic" \
	| grep -v " i4b" \
	| grep -v " new_usb" \
	| grep -v "i4b/" \
	| grep -v "dev/usb2" \
	| grep -v "dev/usb/ehci.c" \
	| grep -v "dev/usb/ehci_pci.c" \
	| grep -v "dev/usb/ohci.c" \
	| grep -v "dev/usb/ohci_pci.c" \
	| grep -v "dev/usb/uhci.c" \
	| grep -v "dev/usb/uhci_pci.c" \
	| grep -v "dev/usb/uhub.c" \
	| grep -v "dev/usb/usb.c" \
	| grep -v "dev/usb/usb_mem.c" \
	| grep -v "dev/usb/usb_subr.c" \
	| grep -v "dev/usb/usbdi.c" \
	| grep -v "dev/usb/usbdi_util.c" \
	| grep -v "dev/usb/ugen.c" \
	| grep -v "dev/usb/ums.c" \
	| grep -v "dev/usb/ukbd.c" \
	| grep -v "dev/usb/uhid.c" \
	| grep -v "dev/usb/ulpt.c" \
	| grep -v "dev/usb/hid.c" \
	| sed -e s/"S\/dev\/usb\/usbdevs -h"/"S\/dev\/usb\/usbdevs"/g \
	| sed -e s/"S\/dev\/usb\/usbdevs -d"/"S\/dev\/usb\/usbdevs"/g \
	> ${TEMP}
	cat ${TEMP} > ${FILE}
.endfor

#
# update
#
	@printf ${FILES_APPEND}           >> ${S}/sys/conf/files
#
#	@printf ${FILES_I386_PC98_APPEND} >> ${S}/sys/conf/files.pc98
#	@printf ${FILES_I386_PC98_APPEND} >> ${S}/sys/conf/files.i386
#

all:
build:
	@echo "All done."

install: ${DIFF} ${PACKAGE} help backup update_config update_pcm 

# cleanup

	rm -f -rv ${I4B_FILES} ${USB_FILES} ${REMOVE_FILES} || echo -n

# install package

	tar -opjxmvf ${PACKAGE} -C${S}

# install header files

	cp -v ${S}/sys/i4b/include/*h ${INCLUDE}/i4b/include/ || echo -n
	cp -v ${S}/sys/dev/usb2/*h ${INCLUDE}/dev/usb2/ || echo -n 

#
# touch some files so that the kernel is rebuilt
# properly
#
	touch \
	${S}/sys/dev/usb/usbdevs \
	${S}/sys/dev/usb2/*c || echo -n

# patch 1
	@echo "These patches are not critical. Don't worry if they fail:"

	(bzcat ${DIFF} | patch -N -l -d ${S}) || echo -n

# patch 2
	sed -e s/"dev_t dev"/"struct cdev \*dev"/g \
		${S}/sys/dev/sound/pcm/dsp.c \
	> ${TEMP}

	cat ${TEMP} > ${S}/sys/dev/sound/pcm/dsp.c

# patch 3
	cat ${S}/include/Makefile \
	| sed -e s/" dev\/usb2 i4b\/include "//g \
	| sed -e s/" dev\/usb2 "//g \
	| sed -e s/" dev\/usb "/" dev\/usb dev\/usb2 i4b\/include "/g \
	> ${TEMP}
	cat ${TEMP} > ${S}/include/Makefile

# patch 4
	cat ${S}/sys/conf/kmod.mk \
	| sed -e s/"@\/dev\/usb\/usbdevs -h"/"@\/dev\/usb\/usbdevs"/g \
	| sed -e s/"@\/dev\/usb\/usbdevs -d"/"@\/dev\/usb\/usbdevs"/g \
	> ${TEMP}
	cat ${TEMP} > ${S}/sys/conf/kmod.mk

# patch 5
.if !exists(${S}/sys/net/if_spppfr.c)
	touch ${S}/sys/net/if_spppfr.c || echo -n
.endif

# patch 6 (obsolete)

# patch 7
#
# remove lines
# starting with a TAB, then
# hook in some directories
#
	cat ${S}/etc/mtree/BSD.include.dist \
	| grep -v "^	" \
	| sed -e s/"^    dev *$$"/"    dev"${BSD_INCLUDE_DIST_APPEND_1}/g \
	| sed -e s/"^\.\. *$$"/${BSD_INCLUDE_DIST_APPEND_2}/g \
	| tr "ше" "\n\t" \
	| cat - \
	> ${TEMP}

	cat ${TEMP} > ${S}/etc/mtree/BSD.include.dist

# patch 8
#
#
	cat ${S}/lib/Makefile \
	| grep -v "^SUBDIR+= libcapi20" \
	| grep -v "^.include <bsd.subdir.mk>" \
	| cat - \
	> ${TEMP}

	echo "SUBDIR+= libcapi20" >> ${TEMP}
	echo ".include <bsd.subdir.mk>" >> ${TEMP}

	cat ${TEMP} > ${S}/lib/Makefile

# patch 9
#
#
	[ -f ${S}/usr.sbin/Makefile.inc ] || \
	(echo "BINDIR?=/usr/sbin" >> ${S}/usr.sbin/Makefile.inc)

# patch 10
#
# update some #include <... statements

.for FILE in ${HEADER_FIX_FILES}
	cat ${FILE} \
	| sed -e "s/<machine\/i4b_/<i4b\/include\/i4b_/g" \
	| sed -e "s/<netisdn\/i4b_/<i4b\/include\/i4b_/g" \
	| sed -e "s/<i4b\/i4b_/<i4b\/include\/i4b_/g" \
	> ${TEMP}
	cat ${TEMP} > ${FILE}
.endfor

#
# final install
#
	cd ${S}/lib/libcapi20 && make cleandepend clean depend all install

	cd ${S}/usr.sbin/i4b && make cleandepend clean depend all install

	man ihfc

uninstall: deinstall

deinstall: ${BACKUP}

	@echo ""
	@echo "Deinstalling sources from \"${S}\""
	@echo "Deinstalling headers from \"${INCLUDE}\""
	@echo ""

	@sleep 10

	rm -f -rv ${I4B_FILES} ${USB_FILES} ${REMOVE_FILES} || echo -n

	tar -jxvf ${BACKUP} -C/
#
# make sure a new backup is made at next
# install
#
	rm -f ${BACKUP}.dont.remove || echo -n

clean:
	rm -f ${TEMP} || echo -n

diffs: clean
	touch ${TEMP}

.for FILE in ${DIFF_FILES:C/${S}\///g}
.if exists(${S}.diff/${FILE}.ref)
	@((( cd ${S}.diff && diff -b -cr ${FILE}.ref ${FILE} ) >> ${TEMP}) \
	&& echo "\"${S}.diff/${FILE}\" does not differ!") || echo -n
.else
	@echo "\"${S}.diff/${FILE}.ref\" does not exist!"
.endif
.endfor

	bzip2 ${TEMP} && mv ${TEMP}.bz2 ${DIFF}

package: diffs
	@cd ${S}/lib/libcapi20 && make clean cleandepend

	@cd ${S}/usr.sbin/i4b && make clean cleandepend

	(cd ${S} && tar --norecurse -cvf - ${PACKAGE_FILES:C/${S}\///g}) > ${TEMP}

	bzip2 ${TEMP} && mv ${TEMP}.bz2 ${PACKAGE}

list_files:
	cd ${S} && ls -l ${PACKAGE_FILES:C/${S}\///}

print_all:
	@printf ${FILES_APPEND}
	@printf ${FILES_I386_PC98_APPEND}
	@printf ${CHANNEL_H_APPEND}
	@printf '${BSD_INCLUDE_DIST_APPEND_1}'
	@printf '${BSD_INCLUDE_DIST_APPEND_2}'

backdate:
	touch -t 200011111111 ${S}/sys/i386/include/bus_at386.h

install_mod:
.for FILE in ${USB_MODULES}
	(cd ${FILE} && make depend all install clean) || echo -n
.endfor

#-HPS
