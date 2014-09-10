



		  Welcome to the new ISDN4BSD driver

                                    .  .   .    .     .



This file should give you some help on how to install this driver.



OFFICIALLY SUPPORTED OPERATING SYSTEMS
======================================

 - FreeBSD-8-stable
 - FreeBSD-9-stable
 - FreeBSD-10-stable
 - FreeBSD-11-current

FEATURES
========

  - support for 512-tap FFT Echo Cancelling
  - support for EuroISDN, DSS1 (point to point and point to multipoint)
  - support for overlap sending and dial in digits
  - support for hold and retrieve in NT-mode
  - support for early B-channel connection when dialing out
  - support for Basic Rate Interface, 2xB, in software and hardware
  - support for Primary Rate Interface, 30xB, in software and hardware
  - driver uses a per controller mutex to protect data and chip access
  - direct access to D-channel and B-channels through /dev/ihfcX.Y
  - full support for NT-mode in software and hardware **
  - new "CAPI 2.0" application interface
  - new "DSS1" protocol
  - new "isdntrace"
  - new "isdndecode"
  - new "isdnd"
  - new "capiserver"

** not all hardware can run NT-mode

SUPPORTED HARDWARE
==================

See "man ihfc" after install.  

A short list of chips supported:
IPAC / ISAC / HSCX / HFC / WINBOND


HOW TO INSTALL AS ROOT
======================

#
# Ensure that libcapi is installed
#

make configure HAVE_ALL=YES KMODDIR=/boot/modules \
    LIBDIR=/usr/local/lib BINDIR=/usr/local/bin \
    INCLUDEDIR=/usr/local/include \
    MANDIR=/usr/local/man/man

make depend all HAVE_ALL=YES
make install HAVE_ALL=YES

HOW TO SET NT-MODE
==================

See "man isdnconfig"

#
# USB controllers start at n = 63 and decrements
#
# NOTE: NT-mode will only be set when "isdnconfig" is executed.
# When one plugs an USB ISDN adapter, TE-mode will be selected by
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

NOTES
=====

If more than 8 units needs to be supported please edit "src/sys/i4b/include/i4b_global.h"

ISDN4BSD FORUM
==============

freebsd-isdn@freebsd.org

--HPS

