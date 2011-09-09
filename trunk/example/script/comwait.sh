#!/bin/bash
source /usr/local/share/rtshell/shell_support
nameserver=/`grep nameservers rtc.conf | sed '/#/d' | \
             sed 's/ //g' | sed 's/corba.nameservers://g' | \
             awk '{ print $1 }' -`/
rtcwd $nameserver
for ((cnt=0; cnt < 10; ++cnt)); do
  found=`rtfind . -i $1`
  if test -n "$found"; then
    echo "$1 launched."
    break
  fi
  sleep 1
done
