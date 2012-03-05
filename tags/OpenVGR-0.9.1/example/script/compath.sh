#!/bin/bash
source /usr/local/share/rtshell/shell_support
nameserver=/`grep nameservers rtc.conf | sed '/#/d' | \
             sed 's/ //g' | sed 's/corba.nameservers://g' | \
             awk '{ print $1 }' -`/
rtcwd $nameserver
rtfind . -i $1 | sed "s/$1//g"
