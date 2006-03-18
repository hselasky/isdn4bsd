#!/bin/sh

#
# This is an example startup shell 
# for ISDN4BSD on NetBSD.
# Don't forget to "chmod +x isdnstart.sh"
#

[ `uname` != "NetBSD" ] && echo "Operating system mismatch!" && exit

#
# only execute once
#
modstat -n i4b > /dev/null && echo "ISDN4BSD already loaded (skipping init)" && exit

echo "Starting ISDN4BSD ..."
modload -o i4b.t /usr/lkm/i4b.o

rm -f /dev/i4b*
rm -f /dev/ihfc*
rm -f /dev/capi20

#
# TODO: mirror /dev/dev/* in /dev/
#

ln -s /dev/dev/i4b /dev/i4b
ln -s /dev/dev/i4bctl /dev/i4bctl
ln -s /dev/dev/capi20 /dev/capi20
ln -s /dev/dev/i4btrc0 /dev/i4btrc0
ln -s /dev/dev/i4btrc1 /dev/i4btrc1
ln -s /dev/dev/i4btrc2 /dev/i4btrc2
ln -s /dev/dev/i4btrc3 /dev/i4btrc3
ln -s /dev/dev/i4btrc4 /dev/i4btrc4
ln -s /dev/dev/i4btrc5 /dev/i4btrc5
ln -s /dev/dev/i4btrc6 /dev/i4btrc6
ln -s /dev/dev/i4btrc7 /dev/i4btrc7

rm -f -r "/dev/dev"
mkdir "/dev/dev"
mount_devfs "/dev/dev"

#
# set NT-mode for controller 1 and 2
#
# HINT: One can do all the configuration
#       in one line, like this:
#
# isdnconfig -u 1 -p DRVR_DSS1_NT \
#            -u 2 -p DRVR_DSS1_NT 
#

