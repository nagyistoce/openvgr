#!/bin/bash
source /usr/local/share/rtshell/shell_support
# rtshell でのコンポーネントパスを取得
# 全コンポーネントが同じ場所にあること
comppath=`./compath.sh Recognition0.rtc`
rtcwd $comppath
# モデルリストファイルの設定
rtconf Recognition0.rtc set RecogModelListPath model/ModelList.txt
rtconf RecognitionResultViewer0.rtc set RecogModelListPath model/ModelList.txt
# 認識パラメータファイルの設定
rtconf Recognition0.rtc set RecogParameterFilePath $1
# モデルIDの設定
rtconf SetModelID0.rtc set ModelID   $2
# キャリブレーションデータディレクトリの設定
rtconf SendImage0.rtc set CalibDir   $3
# キャリブレーションデータファイルの設定
rtconf SendImage0.rtc set CalibFile  $4
# 入力画像ディレクトリの設定
rtconf SendImage0.rtc set ImageDir   $5
# 入力画像ファイルの設定
rtconf SendImage0.rtc set ImageFile0 $6
rtconf SendImage0.rtc set ImageFile1 $7
rtconf SendImage0.rtc set ImageFile2 $8
# 入力エラーコードの設定
rtconf SendImage0.rtc set ErrorCode  $9
# 点群数の設定
rtconf SendImage0.rtc set OutputPointNum ${10}
# デバッグ情報出力指定
#rtconf Recognition0.rtc set DebugText 1
#rtconf Recognition0.rtc set DebugDisplay 1
# コンポーネントの活性化
rtact RecognitionResultViewer0.rtc
rtact Recognition0.rtc
rtact SendImage0.rtc
rtact SetModelID0.rtc
