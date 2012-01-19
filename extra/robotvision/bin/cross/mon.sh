#!/bin/sh
inputfile=`ls -1 testin000.*`
ext=${inputfile##*0.}
#echo $ext
inputfiles=testin*0.$ext
for i in $inputfiles; do
  base=`basename $i 0.$ext`
  num=${base##testin}
  files="out$num"0".$ext out$num"1".$ext out$num"2".$ext out$num"t".$ext"
  echo $files
  montage -tile 3x1 -geometry 512x384 $files
done
