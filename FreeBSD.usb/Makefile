VERSION=1_6_2
DATE=November 2006

FILTER= grep -v "temp"   |\
	grep -v "~"      |\
	grep -v "\#"     |\
	grep -v "\.orig" |\
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
DIFF    = new_usb_${VERSION}.diff.bz2
PACKAGE = new_usb_${VERSION}.tar.bz2
BACKUP  = usb_system_backup_${VERSION}.tar.bz2
N       =\n
SITE    = http://www.turbocat.net/~hselasky/usb4bsd/releases
ARCH   := uname -m
CONF_DIR=${S}/sys/${ARCH}/conf

.if !exists(${CONF_DIR}/GENERIC)
CONF_DIR=${S}/sys/i386/conf
.endif

REMOVE_FILES=\
${INCLUDE}/dev/usb/*h \

UAUDIO_FILES=\
${S}/sys/dev/sound/usb/uaudio.c \
${S}/sys/dev/sound/usb/uaudio.h \
${S}/sys/dev/sound/usb/uaudio_pcm.c \
${S}/sys/dev/sound/usb/uaudioreg.h \

#
#${S}/sys/dev/sound/pcm/mixer.c \
#${S}/sys/dev/sound/pcm/mixer.h \
#

USB_FILES=\
${S}/sys/modules/usb/Makefile \
${S}/sys/modules/ugen/Makefile \
${S}/sys/modules/ums/Makefile \
${S}/sys/modules/ukbd/Makefile \
${S}/sys/modules/ulpt/Makefile \
${S}/sys/modules/uhid/Makefile \
${S}/sys/modules/ubser/Makefile \
${S}/sys/tools/usbdevs2h.awk\
${S}/sys/dev/ata/ata-usb.c \
${S}/sys/dev/if_ndis/if_ndis_usb.c \
${S}/sys/dev/usb \
${S}/sys/netgraph/bluetooth/drivers/ubt/TODO \
${S}/sys/netgraph/bluetooth/drivers/ubt/ng_ubt.c \
${S}/sys/netgraph/bluetooth/drivers/ubt/ng_ubt_var.h \
${S}/sys/netgraph/bluetooth/drivers/ubtbcmfw/ubtbcmfw.c \

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

DIFF_FILES=\
${UAUDIO_FILES} \
${S}/sys/conf/kmod.mk \

BACKUP_FILES=\
${REMOVE_FILES} \
${USB_FILES} \
${CONFIG_FILES} \
${DIFF_FILES} \

PACKAGE_FILES=`find ${USB_FILES} | ${FILTER}`

FILES_APPEND='\
${N}\# \
${N}\# USB support \
${N}dev/usb/usb.c             optional usb \
${N}dev/usb/usb_requests.c    optional usb \
${N}dev/usb/usb_subr.c        optional usb \
${N}dev/usb/usb_transfer.c    optional usb \
${N}dev/usb/usb_cdev.c        optional usb \
${N}dev/usb/usb_hid.c         optional usb \
${N}usbdevs.h                 optional usb dependency \"$$S/tools/usbdevs2h.awk $$S/dev/usb/usbdevs\" compile-with \"$${AWK} -f $$S/tools/usbdevs2h.awk $$S/dev/usb/usbdevs\" no-obj no-implicit-rule before-depend clean \"usbdevs.h\" \
${N}'

HELP='\#\
${N}\# USB driver for FreeBSD 6/7 - ${DATE} \
${N}\# \
${N}\# source directory:      ${S} \
${N}\# backup directory:      ${.CURDIR} \
${N}\# targets:               help fetch all install install_mod deinstall clean \
${N}\# \
${N}\# \
${N}\# 0. If you installed an older \
${N}\#    version of this package, try \
${N}\#    to deinstall it first or do a \
${N}\#    cvsup of your kernel source code. \
${N}\# \
${N}\# 1. Install this package:\
${N}\#    \"make install\" \
${N}\# \
${N}\# 2. Make sure that the following lines are \
${N}\#    in your kernel configuration file, \
${N}\#    for example \"${CONF_DIR}/custom\": \
${N}\
${N}device usb \
${N}\
${N}\# \
${N}\# 3. Recompile and install the kernel: \
${N}\#    \"cd ${S} && make buildkernel installkernel \
${N}\#    KERNCONF=custom -DNOCLEAN -DNO_CLEAN\" \
${N}\# \
${N}\#    The second time you run make \
${N}\#    you can add -DNO_KERNELDEPEND \
${N}\# \
${N}\# 4. Write down the kernel backup location, \
${N}\#    \"${.CURDIR}/kernel\", and reboot. \
${N}\# \
${N}\
${N}\# \
${N}\# In case of trouble you can send an e-mail to: \
${N}\# hselasky@c2i.net \
${N}\
${N}\# type \"make help\" to display this message again \
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
# make directory tree
#
	mkdir ${S}                     || echo -n
	mkdir ${S}/sys                 || echo -n
	mkdir ${S}/sys/dev             || echo -n
	mkdir ${S}/sys/dev/if_ndis     || echo -n
	mkdir ${S}/sys/dev/sound       || echo -n
	mkdir ${S}/sys/dev/sound/pcm   || echo -n
	mkdir ${S}/sys/dev/sound/usb   || echo -n
	mkdir ${S}/sys/dev/ata         || echo -n
	mkdir ${S}/sys/dev/usb         || echo -n
	mkdir ${S}/sys/modules         || echo -n
	mkdir ${S}/sys/modules/ubser   || echo -n
	mkdir ${S}/sys/modules/ugen    || echo -n
	mkdir ${S}/sys/modules/uhid    || echo -n
	mkdir ${S}/sys/modules/ukbd    || echo -n
	mkdir ${S}/sys/modules/ulpt    || echo -n
	mkdir ${S}/sys/modules/ums     || echo -n
	mkdir ${S}/sys/modules/usb     || echo -n
	mkdir ${S}/sys/netgraph        || echo -n
	mkdir ${S}/sys/netgraph/bluetooth  || echo -n
	mkdir ${S}/sys/netgraph/bluetooth/drivers  || echo -n
	mkdir ${S}/sys/netgraph/bluetooth/drivers/ubt  || echo -n
	mkdir ${S}/sys/netgraph/bluetooth/drivers/ubtbcmfw  || echo -n
	mkdir ${S}/sys/conf            || echo -n
	mkdir ${S}/sys/i386            || echo -n
	mkdir ${S}/sys/i386/include    || echo -n
	mkdir ${S}/sys/tools           || echo -n
	mkdir ${S}/etc                 || echo -n
	mkdir ${S}/etc/mtree           || echo -n
	mkdir ${S}/include             || echo -n

	mkdir ${INCLUDE}               || echo -n 
	mkdir ${INCLUDE}/dev           || echo -n 
	mkdir ${INCLUDE}/dev/usb       || echo -n

#
# create missing files
#

.for FILE in \
${INCLUDE}/dev/usb/usb.h \
${USB_FILES} \
${CONFIG_FILES} \
${DIFF_FILES}
	@ [ -f ${FILE} ] || [ -d ${FILE} ] || touch ${FILE} || echo -n
.endfor

#
# do backup
#

	cp -v /boot/kernel/kernel .

	tar -jcvf ${BACKUP} -C/ ${BACKUP_FILES}

	@touch  ${BACKUP}.dont.remove

	sync
.endif


update_config:

#
# cleanup
#

.for FILE in ${CONFIG_FILES}
	cat ${FILE} \
	| grep -v " new_usb" \
	| grep -v "dev/usb2" \
	| grep -v "dev/usb2" \
	| grep -v "dev/usb/usb.c" \
	| grep -v "dev/usb/usb_mem.c" \
	| grep -v "dev/usb/usb_requests.c" \
	| grep -v "dev/usb/usb_subr.c" \
	| grep -v "dev/usb/usb_transfer.c" \
	| grep -v "dev/usb/usb_cdev.c" \
	| grep -v "dev/usb/usb_hid.c" \
	| grep -v "dev/usb/usb_ethersubr.c" \
	| grep -v "dev/usb/usbdi.c" \
	| grep -v "dev/usb/usbdi_util.c" \
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

all:
build:
	@echo "All done."

install: ${DIFF} ${PACKAGE} help backup update_config 

# cleanup

	rm -f -rv ${USB_FILES} ${REMOVE_FILES} || echo -n

# install package

	tar -opjxmvf ${PACKAGE} -C${S}

# install header files

	cp -v ${S}/sys/dev/usb/*h ${INCLUDE}/dev/usb/ || echo -n 

#
# touch some files so that the kernel is rebuilt
# properly
#
	touch \
	${S}/sys/dev/usb/usbdevs \
	${S}/sys/dev/usb/*c || echo -n

# patch 1
	@echo "These patches are not critical. Don't worry if they fail:"

	(bzcat ${DIFF} | patch -N -l -d ${S}) || echo -n

# patch 2

# patch 3 (obsolete)

# patch 4
	cat ${S}/sys/conf/kmod.mk \
	| sed -e s/"@\/dev\/usb\/usbdevs -h"/"@\/dev\/usb\/usbdevs"/g \
	| sed -e s/"@\/dev\/usb\/usbdevs -d"/"@\/dev\/usb\/usbdevs"/g \
	> ${TEMP}
	cat ${TEMP} > ${S}/sys/conf/kmod.mk

# patch 5

# patch 6 (obsolete)

# patch 7 (obsolete)

# patch 8

# patch 9

# patch 10

# patch 11
	rm -f ${UAUDIO_FILES} || echo -n
	touch ${UAUDIO_FILES} || echo -n

# done

uninstall: deinstall

deinstall: ${BACKUP}

	@echo ""
	@echo "Deinstalling sources from \"${S}\""
	@echo "Deinstalling headers from \"${INCLUDE}\""
	@echo ""

	@sleep 10

	rm -f -rv ${USB_FILES} ${REMOVE_FILES} || echo -n

	tar -jxvf ${BACKUP} -C/
#
# make sure a new backup is made at next
# install
#
	rm -f ${BACKUP}.dont.remove || echo -n

clean:
	rm -f ${TEMP}

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

sorry:
	@echo "The system is currently under reconstruction."
	@echo "Please check out again in some hours!"
	@exit 1

package: diffs

	(cd ${S}; tar --norecurse -cvf - ${PACKAGE_FILES:C/${S}\///g}) > ${TEMP}

	bzip2 ${TEMP} && mv ${TEMP}.bz2 ${PACKAGE}

list_files:
	cd ${S} && ls -l ${PACKAGE_FILES:C/${S}\///}

print_all:
	@printf ${FILES_APPEND}

install_mod:
.for FILE in ${USB_MODULES}
	(cd ${FILE} && make depend all install clean) || echo -n
.endfor

#-HPS

