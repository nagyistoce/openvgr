#!/bin/sh
TERMOPT=--window-with-profile="Default"
# コンポーネント起動
gnome-terminal -t "Recognition Progress" -e ./substill/comup.sh --geometry 80x24+0+530 $TERMOPT &
# コンポーネントの起動を待つ　待ち時間は必要に応じて変更する
sleep 1
# 接続
./substill/comcon.sh
lnum=`wc -l testlist.txt | awk '{ print $1 }' -`
i=0
# 認識実行ループ
while [ $i -lt $lnum ]
do
  # 認識パラメータの取得
  i=`expr $i + 1`
  line=`sed -n "$i p" testlist.txt`
  # 活性化して認識
  ./substill/comact.sh $line
  # 認識時間が少しかかるので待つ
  sleep 1
  echo -n "hit enter-key after checking the result."
  read dummy
  # 次の認識のために一旦不活性化
  ./substill/comdeact.sh
done
# コンポーネント終了
./substill/comdown.sh
