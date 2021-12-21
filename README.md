<IMG SRC="https://raw.githubusercontent.com/hselasky/isdn4bsd/main/www/isdn4bsd.jpg"></IMG>
<IMG SRC="https://raw.githubusercontent.com/hselasky/isdn4bsd/main/www/isdn4bsd.gif"></IMG>

This file should give you some help on how to install this driver.

# OFFICIALLY SUPPORTED OPERATING SYSTEMS

 - FreeBSD-8-stable
 - FreeBSD-9-stable
 - FreeBSD-10-stable
 - FreeBSD-11-stable
 - FreeBSD-12-stable
 - FreeBSD-13-stable

# FEATURES
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

# SUPPORTED HARDWARE

See "man ihfc" after install.  

Basic chips supported:

IPAC / ISAC / HSCX / HFC / WINBOND

<UL>
<LI> HFC - based devices (all works, recommended)
<UL>
<LI>    AcerISDN P10
<LI>    Asuscom ISDNLink P-IN100-ST-D2
<LI>    Bewan ISDN USB TA
<LI>    Bewan Modem RNIS USB
<LI>    Billion ISDN tiny USB modem
<LI>    Billion USB TA 2
<LI>    DrayTec ISDN USB
<LI>    DrayTek USB ISDN TA (MiniVigor)
<LI>    HFC-2BDS0 ISA/PnP/PCI/USB ISDN
<LI>    HFC-E1 PCI ISDN
<LI>    HFC-4S PCI ISDN
<LI>    HFC-8S PCI ISDN
<LI>    Motorola MC145575
<LI>    OliTec ISDN USB
<LI>    OliTec Modem RNIS USB V2
<LI>    Stollmann USB TA
<LI>    Teles S0/16.3c PnP
<LI>    Telewell
<LI>    Trust ISDN
<LI>    Twister ISDN TA
<LI>    Xplorer 500
<LI>    Zoltrix Speedier 128K PCI
</UL>

<LI> ISAC - based devices (all should work, not recommended)
<UL>
<LI>    Asuscom ISDNlink 128K ISA
<LI>    AVM A1
<LI>    AVM Fritz!Card
<LI>    AVM Fritz!Card PnP
<LI>    AVM Fritz!Card PCI
<LI>    AVM Fritz!Card PCI version 2 (** NOTE)
<LI>    Compaq Microcom 610 (Compaq series PSB2222I)
<LI>    Creatix ISDN-S0 P&P
<LI>    Creatix S0/16 PnP
<LI>    Dr.Neuhaus Niccy Go@
<LI>    Dynalink IS64PH
<LI>    ELSA MicroLink ISDN/PCI
<LI>    ELSA QuickStep 1000pro ISA
<LI>    ELSA QuickStep 1000pro PCI
<LI>    Eicon.Diehl DIVA 2.0 ISA PnP
<LI>    Eicon.Diehl DIVA 2.02 ISA PnP
<LI>    ITK ix1 Micro V3.0
<LI>    MultiTech MT128SA
<LI>    Sedlbauer Win Speed
<LI>    Siemens I-Surf 2.0 PnP
<LI>    Teles S0/16.3 PnP
</UL>

<LI> Winbond (all works, not recommended)
<UL>
<LI> Asuscom ISDNLink P-IN100-ST-D
<LI> Asuscom ISDNLink TA-280-ST-W (USB)
<LI> Dynalink IS64PPH
<LI> Planet PCI ISDN Adapter (IA128P-STDV)
</UL>

<LI> Tiger300/320 - based devices (all works, not recommended)
<UL>
<LI>    NETjet-S PCI ISDN
<LI>    Teles PCI-TJ
</UL>
</UL>

<P> ** NOTE: Some versions of the "AVM Fritz!Card PCI version 2" card do not work in voice mode!

<A HREF="http://www.openvox.com.cn">
<IMG SRC="https://raw.githubusercontent.com/hselasky/isdn4bsd/main/www/OpenVoxSmall.png" WIDTH="200" HEIGHT="60" ALT="OpenVox Logo">
</A>

# HOW TO INSTALL AS ROOT
<PRE>
#
# Ensure that libcapi is installed
#

make configure HAVE_ALL=YES KMODDIR=/boot/modules \
    LIBDIR=/usr/local/lib BINDIR=/usr/local/bin \
    INCLUDEDIR=/usr/local/include \
    MANDIR=/usr/local/man/man

make -m /usr/src/share/mk SYSDIR=/usr/src/sys depend all HAVE_ALL=YES
make -m /usr/src/share/mk SYSDIR=/usr/src/sys install HAVE_ALL=YES

</PRE>

# HOW TO SET NT-MODE

<PRE>
See "man isdnconfig"

#
# USB controllers start at n = 63 and decrements
#
# NOTE: NT-mode will only be set when "isdnconfig" is executed.
# When one plugs an USB ISDN adapter, TE-mode will be selected by
# default. This might change in the future.
#
</PRE>

# USING WITH ASTERISK, http://www.asterisk.org

On FreeBSD asterisk can be installed from "/usr/ports/net/asterisk".
After that one has to install the ISDN4BSD module. Then get chan_capi
[1], install it by running "gmake all" and then "gmake install", and
create a "capi.conf" file like the one in the "examples" directory. 
Then put something sensible in your "extensions.conf" and it should 
work.

# NOTES

If more than 8 units needs to be supported please edit "src/sys/i4b/include/i4b_global.h"

# ISDN4BSD FORUM

freebsd-isdn@freebsd.org

<IMG SRC="https://raw.githubusercontent.com/hselasky/isdn4bsd/main/www/modem.gif"></IMG>

<A HREF="mailto:hps&#x40;selasky.org">--HPS</A>

