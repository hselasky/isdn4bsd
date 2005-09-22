#!/bin/sh
FILES="\
dev/sound/pcm/sound.h \
fs/devfs/devfs.h \
isa/isa_common.h \
isa/isavar.h \
machine/clock.h \
machine/resource.h \
net/if_var.h \
netgraph/netgraph.h \
netgraph/ng_message.h \
netgraph/ng_parse.h \
sys/bus.h \
sys/lockmgr.h \
sys/module.h \
sys/mutex.h \
sys/rman.h \
sys/rtprio.h \
sys/selinfo.h \
vm/pmap.h \
vm/vm.h \
"

for F in `find /mnt3/src/sys/i4b/* | grep -v "~" | grep -v "#"`
do

([ -f $F ] && (

echo "File $F"

for S in $FILES
do
cat $F | grep -a $S
done

echo -n

)) || echo "File $F does not exist"

done