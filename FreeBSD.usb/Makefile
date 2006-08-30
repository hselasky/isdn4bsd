VERSION=1_6_1_r2
DATE=August 2006

FILTER= grep -v "temp"   |\
	grep -v "~"      |\
	grep -v "\#"     |\
	grep -v "\.orig" |\
	grep -v "/\.svn
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
${INCLUDE}/dev/usb2/*h \

USB_FILES=\
${S}/sys/modules/usb/Makefile \
${S}/sys/modules/ugen/Makefile \
${S}/sys/modules/ums/Makefile \
${S}/sys/modules/ukbd/Makefile \
${S}/sys/modules/ulpt/Makefile \
${S}/sys/modules/uhid/Makefile \
${S}/sys/tools/usbdevs2h.awk\
${S}/sys/dev/ata/ata-usb.c \
${S}/sys/dev/usb/usb.h \
${S}/sys/dev/usb/usb_port.h \
${S}/sys/dev/usb/usbdi.h \
${S}/sys/dev/usb/usbdi_util.h \
${S}/sys/dev/usb/usbdivar.h \
${S}/sys/dev/usb/hid.h \
${S}/sys/dev/usb/usbhid.h \
${S}/sys/dev/usb/if_aue.c \
${S}/sys/dev/usb/if_auereg.h \
${S}/sys/dev/usb/if_axe.c \
${S}/sys/dev/usb/if_axereg.h \
${S}/sys/dev/usb/if_cdce.c \
${S}/sys/dev/usb/if_cdcereg.h \
${S}/sys/dev/usb/if_cue.c \
${S}/sys/dev/usb/if_cuereg.h \
${S}/sys/dev/usb/if_kue.c \
${S}/sys/dev/usb/if_kuefw.h \
${S}/sys/dev/usb/if_kuereg.h \
${S}/sys/dev/usb/if_rue.c \
${S}/sys/dev/usb/if_ruereg.h \
${S}/sys/dev/usb/if_udav.c \
${S}/sys/dev/usb/if_udavreg.h \
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

DIFF_FILES=\
${S}/sys/conf/kmod.mk \
${S}/sys/dev/sound/usb/uaudio.c \
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

BACKUP_FILES=\
${REMOVE_FILES} \
${USB_FILES} \
${CONFIG_FILES} \
${DIFF_FILES} \

PACKAGE_FILES=`find ${USB_FILES} | ${FILTER}`

FILES_APPEND='\
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

#
# "ше" are substituted for 
# "\n" and "\t", respectivly
#

M=ше
BSD_INCLUDE_DIST_APPEND_1="${M}usb2${M}.."

HELP='\#\
${N}\# USB driver for FreeBSD 5/6/7 - ${DATE} \
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
${N}\# 3. Make sure that the following lines are \
${N}\#    in your kernel configuration file, \
${N}\#    for example \"${CONF_DIR}/custom\": \
${N}\
${N}device usb \
${N}\
${N}\# \
${N}\# 4. Recompile and install the kernel: \
${N}\#    \"cd ${S} && make buildkernel installkernel \
${N}\#    KERNCONF=custom -DNO_MODULES -DNOCLEAN -DNO_CLEAN\" \
${N}\# \
${N}\#    The second time you run make \
${N}\#    you can add -DNO_KERNELDEPEND \
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
	mkdir ${S}/sys/dev/sound       || echo -n
	mkdir ${S}/sys/dev/sound/usb   || echo -n
	mkdir ${S}/sys/dev/ata         || echo -n
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

	mkdir ${INCLUDE}               || echo -n 
	mkdir ${INCLUDE}/dev           || echo -n 
	mkdir ${INCLUDE}/dev/usb2      || echo -n

#
# create missing files
#

.for FILE in \
${INCLUDE}/dev/usb2/usb.h \
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
	| grep -v "dev/usb/ehci.c" \
	| grep -v "dev/usb/ehci_pci.c" \
	| grep -v "dev/usb/ohci.c" \
	| grep -v "dev/usb/ohci_pci.c" \
	| grep -v "dev/usb/ugen.c" \
	| grep -v "dev/usb/uhci.c" \
	| grep -v "dev/usb/uhci_pci.c" \
	| grep -v "dev/usb/uhub.c" \
	| grep -v "dev/usb/usb.c" \
	| grep -v "dev/usb/usb_mem.c" \
	| grep -v "dev/usb/usb_subr.c" \
	| grep -v "dev/usb/usbdi.c" \
	| grep -v "dev/usb/usbdi_util.c" \
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

all:
build:
	@echo "All done."

install: ${DIFF} ${PACKAGE} help backup update_config 

# cleanup

	rm -f -rv ${USB_FILES} ${REMOVE_FILES} || echo -n

# install package

	tar -opjxmvf ${PACKAGE} -C${S}

# install header files

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

# patch 3
	cat ${S}/include/Makefile \
	| sed -e s/" dev\/usb2 "//g \
	| sed -e s/" dev\/usb "/" dev\/usb dev\/usb2 "/g \
	> ${TEMP}
	cat ${TEMP} > ${S}/include/Makefile

# patch 4
	cat ${S}/sys/conf/kmod.mk \
	| sed -e s/"@\/dev\/usb\/usbdevs -h"/"@\/dev\/usb\/usbdevs"/g \
	| sed -e s/"@\/dev\/usb\/usbdevs -d"/"@\/dev\/usb\/usbdevs"/g \
	> ${TEMP}
	cat ${TEMP} > ${S}/sys/conf/kmod.mk

# patch 5

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
	| tr "ше" "\n\t" \
	| cat - \
	> ${TEMP}

	cat ${TEMP} > ${S}/etc/mtree/BSD.include.dist

# patch 8

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

package: diffs

	(cd ${S}; tar --norecurse -cvf - ${PACKAGE_FILES:C/${S}\///g}) > ${TEMP}

	bzip2 ${TEMP} && mv ${TEMP}.bz2 ${PACKAGE}

list_files:
	cd ${S} && ls -l ${PACKAGE_FILES:C/${S}\///}

print_all:
	@printf ${FILES_APPEND}
	@printf '${BSD_INCLUDE_DIST_APPEND_1}'
	@printf '${BSD_INCLUDE_DIST_APPEND_2}'

backdate:
	touch -t 200011111111 ${S}/sys/i386/include/bus_at386.h

install_mod:
.for FILE in ${USB_MODULES}
	(cd ${FILE} && make depend all install clean) || echo -n
.endfor

#-HPS

