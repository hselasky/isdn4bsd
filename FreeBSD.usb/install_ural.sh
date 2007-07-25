#!/bin/sh
echo "Installing if_ural for FreeBSD 7-current ..."
cp -i ../src/sys/dev/usb/if_ural*[ch] /sys/dev/usb/
echo "Done."
