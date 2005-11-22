Welcome to the new ISDN4BSD driver.

This file should give you some help on how to install this driver, until an 
automatic installer is present.  

FEATURES


SUPPORTED HARDWARE

At the writing moment only PCI ISDN adapters for Basic Rate and Primary Rate, are 
supported. See "man ihfc" after install.  


HOW TO INSTALL AS ROOT

#
# install header files
#

rm -r -i "/usr/include/i4b"
mkdir "/usr/include/i4b"
cp -r "src/sys/i4b/include" "/usr/include/i4b"

#
# compile and install kernel module, libraries and ISDN utilities
#

cd src/lib/libcapi20
make depend
make
make install

cd ../../../
make depend
make
make install

#
# The rest of the install instructions from here
# and downwards, apply to NetBSD only.
#

#
# load the driver (NetBSD only)
#

modload -o i4b.t /usr/lkm/i4b.o

#
# mount new /dev (NetBSD only)
#

rehash
sync

rm /dev/i4b*
rm /dev/ihfc*
rm /dev/capi20

#
# TODO: mirror /dev/dev/* in /dev/
#

ln -s /dev/dev/i4b /dev/i4b
ln -s /dev/dev/i4bctl /dev/i4bctl
ln -s /dev/dev/capi20 /dev/capi20
ln -s /dev/dev/i4btrc0 /dev/i4btrc0
ln -s /dev/dev/i4btrc1 /dev/i4btrc1
ln -s /dev/dev/i4btrc7 /dev/i4btrc7

rm -r "/dev/dev"
mkdir "/dev/dev"
mount_devfs "/dev/dev"

HOW TO SET NT-MODE:

See "man isdnconfig" or "man isdnd.rc".

USING WITH ASTERISK

  First you have to install the ISDN4BSD module. Then get chan_capi [1], install it 
by using "gmake all" and then "gmake install", and create a "capi.conf" file like the 
one in the "examples" directory.


RESOURCES

 [1] chan_capi (when one is asked for password, just press enter)
cvs -d:pserver:anonymous@cvs.sourceforge.net:/cvsroot/chan-capi login
cvs -z3 -d:pserver:anonymous@cvs.sourceforge.net:/cvsroot/chan-capi co -P chan_capi 

