



		  Welcome to the new ISDN4BSD driver

                                    .  .   .    .     .



This file should give you some help on how to install this driver.



SUPPORTED OPERATING SYSTEMS
===========================

 - FreeBSD-5.3
 - FreeBSD-5.4
 - FreeBSD-5-current
 - FreeBSD-6-current
 - FreeBSD-7-current

 - NetBSD 2.x.x
 - NetBSD 3.x.x



FEATURES
========

  - support for EuroISDN, DSS1 (point to point and point to multipoint)
  - support for overlap sending and dial in digits
  - support for early B-channel connection when dialing out
  - support for Basic Rate Interface, 2xB, in software and hardware
  - support for Primary Rate Interface, 30xB, in software and hardware
  - driver uses a per controller mutex to protect data and chip access
  - direct access to D-channel and B-channels
  - full support for NT-mode in software and hardware **
  - cloneable "/dev/i4b"
  - new "CAPI 2.0" application interface
  - new "DSS1" protocol
  - new "isdnphone" that uses a kernel soundbridge
  - new "isdntrace"
  - new "isdndecode"
  - new "isdnd"



SUPPORTED HARDWARE
==================

See "man ihfc" after install.  

NOTE: On NetBSD only PCI ISDN adapters for Basic Rate and Primary Rate, 
are supported.

A short list of chips supported:
IPAC / ISAC / HSCX / HFC / WINBOND / OHCI / UHCI / EHCI


HOW TO INSTALL AS ROOT ON FREEBSD
=================================

cd FreeBSD.i4b
make S=../src package
make install

#
# Type "make help" for more help.
#
# If you have the source tree in a directory different from "/usr/src",
# simply use "make S=/usr/my_src_directory install". The only files that will 
# not get installed there is a copy of some header files, which are always
# installed in "/usr/include/..."
#


HOW TO INSTALL AS ROOT ON NETBSD
================================

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
# load the driver
#

modload -o i4b.t /usr/lkm/i4b.o

#
# mount new /dev
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



HOW TO SET NT-MODE
==================

See "man isdnconfig" or "man isdnd.rc".

#
# If one has "isdnd" running, then NT-mode can be set
# by adding some lines to "/etc/isdn/isdnd.rc", under
# the "SYSTEM section":
#

controller # 0
# uncomment the next line to
# enable NT-mode for controller 0:
#
# driver_type = DRVR_DSS1_NT
#
controller # 1
controller # 2
controller # n

#
# USB controllers start at n = 7 and decrements
#
# NOTE: NT-mode will only be set when "isdnd" is
# started or "isdnconfig" is executed. When one plugs
# an USB ISDN adapter, TE-mode will be selected by
# default. This might change in the future.
#


USING WITH ASTERISK, http://www.asterisk.org
============================================

On FreeBSD asterisk can be installed from "/usr/ports/net/asterisk".
After that one has to install the ISDN4BSD module. Then get chan_capi
[1], install it by running "gmake all" and then "gmake install", and
create a "capi.conf" file like the one in the "examples" directory. 
Then put something sensible in your "extensions.conf" and it should 
work.


RESOURCES
=========

 [1] chan_capi (when one is asked for password, just press enter)
cvs -d:pserver:anonymous@cvs.sourceforge.net:/cvsroot/chan-capi login
cvs -z3 -d:pserver:anonymous@cvs.sourceforge.net:/cvsroot/chan-capi co -P chan_capi 


NOTES:
======

All I4B header files were moved from "/usr/include/machine/" and 
"/usr/src/sys/i386/include", to "/usr/include/i4b/include" and 
"/sys/i4b/include", including "capi20.h".

The ISDN4BSD CAPI library is 99% compatible with the Linux CAPI library. 
To make a Linux CAPI application compile on FreeBSD one only needs to include 
the file "/usr/include/i4b/include/capi20.h" and make a few one-line patches. 
See "man capi" for how to resolve those one-liners. 
There is no need to install "liblinuxcapi" !

ISDN4BSD controller 0, is CAPI controller 0. If ones application cannot handle
that, one can add a few lines of code that switches "0" to "MAX_CONTROLLER" in 
incoming frames. ISDN4BSD will accept both "0" and "MAX_CONTROLLER" 
as controller "0".

See "/usr/src/usr.sbin/i4b/capitest/main.c" for an example on how to take full 
advantage of the ISDN4BSD CAPI 2.0 implementation.

See "/usr/src/usr.sbin/i4b/isdntest/main.c" if one wants to stay updated on the 
I4B API.

As a consequence of the changes I have made, one can now run "isdntest" while 
"isdnd" is running. I have plans to make the device allocation dynamic, so 
that applications can allocate devices like "/dev/i4btelX" dynamically, 
without need for configuration. Until further all the "counts" have been put in 
"/sys/i4b/include/i4b_global.h". If one needs more than 8 units please edit 
that file.

The driver now supports all cards that are supported by the official version of 
I4B except active CAPI controllers and one "Frizt" card. If one has got 
an ISDN device the chances should be good that this driver will support it.


ISDN4BSD FORUM
==============

freebsd-isdn@freebsd.org


--HPS
