#!/bin/sh
FILES=" \
arpa/inet.h \
ctype.h \
curses.h \
dev/pci/pcireg.h \
dev/pci/pcivar.h \
dev/sound/pcm/sound.h \
dirent.h \
err.h \
errno.h \
fcntl.h \
fs/devfs/devfs.h \
isa/isa_common.h \
isa/isavar.h \
locale.h \
machine/bus.h \
machine/clock.h \
machine/resource.h \
math.h \
ncurses.h \
net/bpf.h \
net/if.h \
net/if_sppp.h \
net/if_types.h \
net/if_var.h \
net/netisr.h \
net/slcompress.h        \
netdb.h \
netgraph/netgraph.h \
netgraph/ng_message.h \
netgraph/ng_parse.h \
netinet/in.h \
netinet/in_systm.h \
netinet/in_var.h \
netinet/ip.h \
netinet/ip_icmp.h \
netinet/tcp.h \
netinet/udp.h \
osreldate.h \
paths.h \
poll.h \
regex.h \
signal.h \
stdarg.h \
stdio.h \
stdlib.h \
string.h \
strings.h \
sys/bus.h \
sys/cdefs.h \
sys/conf.h \
sys/dir.h \
sys/endian.h \
sys/errno.h \
sys/fcntl.h \
sys/file.h \
sys/filio.h \
sys/ioccom.h \
sys/ioctl.h \
sys/kernel.h \
sys/lock.h \
sys/lockmgr.h \
sys/malloc.h \
sys/mbuf.h \
sys/mman.h \
sys/module.h \
sys/mutex.h \
sys/param.h \
sys/poll.h \
sys/proc.h \
sys/queue.h \
sys/rman.h \
sys/rtprio.h \
sys/selinfo.h \
sys/socket.h \
sys/sockio.h \
sys/soundcard.h \
sys/stat.h \
sys/syslog.h \
sys/systm.h \
sys/time.h \
sys/tty.h \
sys/types.h \
sys/uio.h \
sys/un.h \
sys/unistd.h \
sys/wait.h \
sysexits.h \
syslog.h \
time.h \
unistd.h \
dev/usb2/usb.h \
dev/usb2/usb_port.h \
dev/usb2/usb_subr.h \
vm/pmap.h \
vm/vm.h \
"

# [ -d include ] || mkdir include
# [ -d include/dev ] || mkdir include/dev
# [ -d include/dev/usb2 ] || mkdir include/dev/usb2
# [ -d include/dev/pci ] || mkdir include/dev/pci
# [ -d include/dev/sound ] || mkdir include/dev/sound
# [ -d include/dev/sound/pcm/ ] || mkdir include/dev/sound/pcm/
# [ -d include/fs ] || mkdir include/fs
# [ -d include/fs/devfs ] || mkdir include/fs/devfs
# [ -d include/isa ] || mkdir include/isa
# [ -d include/net ] || mkdir include/net
# [ -d include/netgraph ] || mkdir include/netgraph
# [ -d include/netinet ] || mkdir include/netinet
# [ -d include/sys ] || mkdir include/sys
# [ -d include/vm ] || mkdir include/vm
# [ -d include/machine ] || mkdir include/machine

for FF in $FILES
do
F=include/$FF

[ -f /usr/include/$FF ] || [ -f /mnt3/src/sys/$FF ] || echo "$FF does not exist!"


# rm -f $F

# echo "#if defined(__FreeBSD__)" 
# echo "#include <$FF>" 
# echo "#elif defined(__NetBSD__)"
# echo "#include <$FF>"
# echo "#else"
# echo "#include <$FF>"
# echo "#endif"

done
