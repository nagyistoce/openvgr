#!/bin/sh
if [ "$1" = "test" ]; then
  rm -f template.*
  ln -s ../samples/template.png .
  ./renimg.sh ../samples
  cd ../matrix
  rm -f camera_calib.yaml 3d.txt
  ln -s ../samples/camera_calib.yaml .
  ln -s ../samples/3d.txt .
  cd ../cross
else
  datadir=$1
  if [ -z "$datadir" ]; then
    echo "data directory ?"
    read datadir
  fi
  rm -f template.*
  ln -s $datadir/template.png .
  ./renimg.sh $datadir
  cd ../matrix
  rm -f camera_calib.yaml 3d.txt
  ln -s $datadir/camera_calib.yaml .
  ln -s $datadir/3d.txt .
  cd ../cross
fi
