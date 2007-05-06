#!/bin/sh
echo "Installing uaudio ..."
cp -i ../src/sys/dev/sound/usb/uaudio*[ch] /sys/dev/sound/usb/
cp -i ../src/sys/dev/sound/pcm/mixer*[ch] /sys/dev/sound/pcm/
echo "Done."
