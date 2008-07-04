S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/sound

KMOD=  usb2_sound
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h
SRCS+= opt_usb.h opt_bus.h feeder_if.h channel_if.h
SRCS+= mixer_if.h

SRCS+= usb2_sound.c
SRCS+= uaudio2.c
SRCS+= uaudio2_pcm.c

.include <bsd.kmod.mk>
