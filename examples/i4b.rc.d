#!/bin/sh
#
# PROVIDE: i4b
# REQUIRE: DAEMON
# KEYWORD: shutdown
#
# You will need to set some variables in /etc/rc.conf to start isdn:
#
# i4b=YES
#
# You will also need to edit "/etc/rc.d/asterisk" to make it
# require the I4B rc startup file.
#

if [ -f /etc/rc.subr ]
then
        . /etc/rc.subr
fi

name="i4b"
rcvar=$name
command="/usr/local/sbin/i4b_postinstall"

