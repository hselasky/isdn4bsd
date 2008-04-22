#
# Makefile for automatic installation on NetBSD
#

S=${.CURDIR}/../src

all:
	@echo "Done."

install:

#
# install header files
#

	rm -r "/usr/include/i4b" || echo -n
	mkdir "/usr/include/i4b"
	cp -r "${S}/sys/i4b/include" "/usr/include/i4b/"

#
# compile and install kernel module, libraries and ISDN utilities
#

	cd ${S}/lib/libcapi20 ; make depend
	cd ${S}/lib/libcapi20 ; make
	cd ${S}/lib/libcapi20 ; make install

	cd ${S}/../ ; make depend
	cd ${S}/../ ; make
	cd ${S}/../ ; make install

#
# By default USB support is not compiled. Be sure
# to disable all USB devices in the kernel first.
# You might have to build a new kernel. Then compile
# the I4B module like this:
#
# cd module
# make -DWITH_USB clean
# make -DWITH_USB depend
# make -DWITH_USB all
# make -DWITH_USB install
# cd ..
#

#
# load the driver
#

load:
	${S}/../examples/isdnstart.sh

package: all

