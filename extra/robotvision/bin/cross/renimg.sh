#!/bin/bash
srcdir=$1
base=testin
for ((cnum=0; cnum < 3; cnum++));  do
  inputfiles=$srcdir/cap*$cnum.png
  num=0
  for i in $inputfiles; do
    pnum=`printf %02d $num`
    echo $i $base$pnum$cnum.png
    ln -sf $i $base$pnum$cnum.png
    num=`expr $num + 1`
  done
done
