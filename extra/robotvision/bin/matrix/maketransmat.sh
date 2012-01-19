#!/bin/sh
# クロスマークのカメラ系３次元位置をもとめて mat 形式で crosspos.mat に出力
lnum=`cat Cdata.0 | wc -l`
echo $lnum 3 > crosspos.mat
paste Cdata.0 Cdata.1 | \
awk '{ printf( "./cr2xyz -i camera_calib.yaml %f %f %f %f\n", $1, $2, $3, $4 ) }' - | sh >> crosspos.mat
# ロボット座標系での対応位置を mat 形式で 3d.mat に出力
echo $lnum 3 > 3d.mat
cat 3d.txt >> 3d.mat
./transmat -i crosspos.mat -j 3d.mat -o matrix.txt
rm crosspos.mat 3d.mat
