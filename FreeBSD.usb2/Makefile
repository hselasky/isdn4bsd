DATE=February 2009

.if !defined(S)
S       = /usr/src
.endif
.if !defined(INCLUDE)
INCLUDE = /usr/include
.endif

all:
	@echo "USB2 branch. Makefile last updated ${DATE}."
	@echo ""
	@echo "The following installation will link the USB2 files "
	@echo "into your kernel sources and includes. You need to have "
	@echo "FreeBSD-7/8 checked out in $S. You can override this "
	@echo "location with S=${S} and INCLUDE=${INCLUDE}."
	@[ -d ${S}/sys/dev ] || (echo "${S}/sys/dev does not exist" ; exit )
	@[ -d ${S}/sys/modules ] || (echo "${S}/sys/modules does not exist" ; exit )
	@[ -d ${INCLUDE}/dev ] || (echo "${INCLUDE}/dev" ; exit )
	@echo ""
	@echo "You have to apply the following patch manually:"
	@echo "${S}/sys/conf/kmod.mk"
	@echo "@@ -331,6 +331,7 @@"
	@echo "        dev/sound/pcm/feeder_if.m dev/sound/pcm/mixer_if.m \\"
	@echo "        dev/sound/midi/mpu_if.m dev/sound/midi/mpufoi_if.m \\"
	@echo "        dev/sound/midi/synth_if.m dev/usb/usb_if.m isa/isa_if.m \\"
	@echo "+       dev/usb2/core/usb2_if.m \\"
	@echo "        kern/bus_if.m kern/cpufreq_if.m kern/device_if.m kern/serdev_if.m \\"
	@echo "        libkern/iconv_converter_if.m opencrypto/cryptodev_if.m \\"
	@echo "        pc98/pc98/canbus_if.m"
	@echo ""
	@echo "Done."

install:
	@echo "Installing USB2 branch to ${S} and ${INCLUDE}. "
	@echo ""
	@echo "Waiting 10 seconds ..."
	@echo ""
	@sleep 10
	@echo "Cleaning up previous installation ..."
	@echo ""
	@rm -r -v -f ${S}/sys/dev/usb2
	@rm -r -v -f ${S}/sys/modules/usb2
	@rm -r -v -f ${INCLUDE}/dev/usb2

	@echo ""
	@echo "Copying files ..."
	@echo ""

	tar --exclude=".svn" --exclude="@" --exclude="*~" \
	--exclude="*#" -C ${.CURDIR}/../src \
	-cvf temp.tar sys/modules/usb2 sys/dev/usb2

	tar -mxvf temp.tar -C ${S}

	@echo "Installing header files ..."
	mkdir ${INCLUDE}/dev/usb2
	mkdir ${INCLUDE}/dev/usb2/include
	cp -v ${.CURDIR}/../src/sys/dev/usb2/include/*h ${INCLUDE}/dev/usb2/include/

	@echo ""
	cd ${.CURDIR}/../src/lib/libusb20 && make WARNS=4 clean cleandepend depend all install
	cd ${.CURDIR}/../src/lib/libusbhid && make WARNS=4 clean cleandepend depend all install
	cd ${.CURDIR}/../src/usr.sbin/usbconfig && make BINDIR=/usr/sbin clean cleandepend depend all install
	cd ${.CURDIR}/../src/usr.bin/usbhidctl && make BINDIR=/usr/bin clean cleandepend depend all install
	cd ${.CURDIR}/../src/usr.bin/usbhidaction && make BINDIR=/usr/bin clean cleandepend depend all install

	@echo ""
	@echo "NOTE: -k option is used here. Please check "
	@echo "any compile failures."
	@echo ""
	@echo "NOTE: Buildworld will not install USB2 !"
	@echo ""

	@echo "1) Please execute manually: "
	@echo "make -C ${S}/sys/modules/usb2 clean cleandepend depend all install"

	@echo "2) Please remove any USB drivers from your kernel "
	@echo "and update your loader.conf manually:"
	@echo ""
	@echo "cat >> /boot/loader.conf << EOF"
	@echo ""
	@echo "usb2_core_load=YES"
	@echo "usb2_controller_load=YES"
	@echo "usb2_controller_ehci_load=YES"
	@echo "usb2_controller_ohci_load=YES"
	@echo "usb2_controller_uhci_load=YES"
	@echo "usb2_input_load=YES"
	@echo "usb2_input_kbd_load=YES"
	@echo "usb2_input_ms_load=YES"
	@echo "EOF"
	@echo ""
	@echo "USB utilities installed: usbconfig"
	@echo ""
	@echo "3) Need libusb-0.1.12_2 support? Please see: http://wiki.freebsd.org/USB"
	@echo ""
	@echo "Have fun! --hps"
	@echo ""

clean:
	rm -f temp.tar

install_usr:
	@echo "Installing header files ..."
	cp -v ${.CURDIR}/../src/sys/dev/usb2/include/*h ${INCLUDE}/dev/usb2/include/

	cd ${.CURDIR}/../src/lib/libusb20 && make WARNS=4 clean cleandepend depend all install
	cd ${.CURDIR}/../src/lib/libusbhid && make WARNS=4 clean cleandepend depend all install
	cd ${.CURDIR}/../src/usr.sbin/usbconfig && make BINDIR=/usr/sbin clean cleandepend depend all install
	cd ${.CURDIR}/../src/usr.bin/usbhidctl && make BINDIR=/usr/bin clean cleandepend depend all install
	cd ${.CURDIR}/../src/usr.bin/usbhidaction && make BINDIR=/usr/bin clean cleandepend depend all install
