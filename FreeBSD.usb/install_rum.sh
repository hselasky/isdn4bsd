#!/bin/sh
echo "Installing if_rum for FreeBSD 7-current ..."
cp -i ../src/sys/dev/usb/if_rum*[ch] /sys/dev/usb/
echo "Done."
