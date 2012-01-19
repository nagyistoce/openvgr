#!/bin/sh
#
# クロスマーカー検出スクリプト
#
# 使用方法の出力
usage()
{
  echo "Usage: ./cr.sh [-bthr #] [-lmin #] [-gmax #] [-smin #] [-disp]"
  exit 1
}
# 入力画像 testin*[012].* から template.* に似たクロスマークを検出する
# 結果は Cdata.[012] ファイルに出力される
getCrossData()
{
    # cnext : 入力ファイル名に含まれるカメラ番号と拡張子
	cnext=$cnum.$ext
    # 入力ファイル名
	inputfiles=testin*$cnext
    # 出力ファイル名
	output=Cdata.$cnum
    # ３次元座標とクロス点の対応データ作成
    cat /dev/null > $output
	for i in $inputfiles; do
		base=`basename $i $cnext`
		num=${base##testin}
        # クロス点画素座標の検出
		cr=`./findx -i $i -t template.$ext -o out$num$cnext $opts`
		echo $cr >> $output
	done
}
# メイン処理
# findx に対するスクリプトのオプションデフォルト値　環境に合わせて変更すること
#opts="-bthr 80 -lmin 5 -gmax 2 -smin 23 -xelo"
BTHR=80 # 二値化閾値
LMIN=5  # 最短線分長
GMAX=2  # 線分とぎれ許容長
SMIN=23 # 線分傾き最小値
INFO=-xelo # 実行情報
# スクリプト実行時に与えられるオプションを取得する
GETOPT=`getopt -a -q -o h -l bthr:,lmin:,gmax:,smin:,disp -- "$@"`
if [ $? != 0 ]; then
  usage
fi
eval set -- "$GETOPT"
#echo "$@"
while true
  do
    case $1 in
      --bthr) BTHR=$2; shift 2  ;;
      --lmin) LMIN=$2; shift 2  ;;
      --gmax) GMAX=$2; shift 2  ;;
      --smin) SMIN=$2; shift 2  ;;
      --disp) INFO="-xelo -disp";
              shift ;;
      -h)     usage             ;;
      --)     shift ; break     ;;
      *)      usage             ;;
    esac
  done
# findx オプション設定
opts="-bthr $BTHR -lmin $LMIN -gmax $GMAX -smin $SMIN $INFO"
echo $opts
# 使用する画像フォーマットを調べる
firstfile=`ls -1 testin000.*`
ext=${firstfile##*.}
# クロスマーク位置検出の実行
cnum=0 # カメラ番号０の画像を処理
getCrossData
cnum=1 # カメラ番号１の画像を処理
getCrossData
cnum=2 # カメラ番号２の画像を処理
getCrossData
