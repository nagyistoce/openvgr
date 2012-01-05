#!/bin/bash
# コンポーネント起動
TERMOPT=--window-with-profile="Default"
./Measure3DComp &
./MultiCameraComp &
./MultiDispComp &
./RecognitionResultViewerComp &
./RecognitionComp &

if test -r /usr/bin/gnome-terminal
then
  gnome-terminal -t "SetModelID" -e ./SetModelIDComp --geometry 80x24+0+530 $TERMOPT &
else
  echo "gnome-terminal がありません"
  echo "スクリプトを継続する場合は"
  echo "別ターミナルで SetModelIDComp を実行し"
  echo "その後 enter キーを押してください"
  read dummy
fi

# コンポーネントの起動完了を待つ
./comwait.sh MultiCamera0.rtc
./comwait.sh MultiDisp0.rtc
./comwait.sh Measure3D0.rtc
./comwait.sh SetModelID0.rtc
./comwait.sh Recognition0.rtc
./comwait.sh RecognitionResultViewer0.rtc

# rtshell でのコンポーネントパスを取得
# 全コンポーネントが同じ場所にあること
source /usr/local/share/rtshell/shell_support
compath=`./compath.sh Recognition0.rtc`
rtcwd $compath

# コンポーネント接続
rtcon MultiCamera0.rtc:images MultiDisp0.rtc:images
rtcon MultiCamera0.rtc:images Measure3D0.rtc:ImageIn
rtcon MultiCamera0.rtc:control Measure3D0.rtc:CameraCapture
rtcon Measure3D0.rtc:Data3DOut Recognition0.rtc:Stereo3DIn
rtcon Measure3D0.rtc:Reconstruct3D Recognition0.rtc:Reconstruct3D
rtcon SetModelID0.rtc:Recognition Recognition0.rtc:Recognition
rtcon Recognition0.rtc:RecognitionResultViewer RecognitionResultViewer0.rtc:RecognitionResultViewer

# カメラキャリブレーションデータ設定
rtconf MultiCamera0.rtc set camera_calib_file ../../build/camera_calib.yaml
# カメラ動作パラメータ設定
rtconf MultiCamera0.rtc set camera_setting_file ../../build/ieee1394board.0
# 認識パラメータ設定
rtconf Recognition0.rtc set RecogParameterFilePath ../dataset/param/w02.txt
# モデルリストファイルの設定
rtconf Recognition0.rtc set RecogModelListPath model/ModelList.txt
rtconf RecognitionResultViewer0.rtc set RecogModelListPath model/ModelList.txt
# キャプチャ画像の保存を設定
#rtconf MultiDisp0.rtc set image_save_mode 1
if [ "$1" = "debug" ]
then
  rtconf Recognition0.rtc set DebugText 1
  rtconf Recognition0.rtc set DebugDisplay 1
fi

# コンポーネントの活性化
# SetModelID は最後に活性化すること
rtact Measure3D0.rtc
rtact MultiDisp0.rtc
rtact MultiCamera0.rtc
rtact RecognitionResultViewer0.rtc
rtact Recognition0.rtc
rtact SetModelID0.rtc

# 認識完了待ち
sleep 5
# 結果確認待ち
echo "<< Hit enter-key to end >>"
read dummy

# コンポーネントの不活性化
rtdeact Measure3D0.rtc
rtdeact MultiDisp0.rtc
rtdeact MultiCamera0.rtc
rtdeact RecognitionResultViewer0.rtc
rtdeact Recognition0.rtc
rtdeact SetModelID0.rtc

# コンポーネント終了
rtdis MultiCamera0.rtc
rtdis Measure3D0.rtc
rtdis RecognitionResultViewer0.rtc
rtdis Recognition0.rtc
rtexit MultiDisp0.rtc
rtexit Measure3D0.rtc
rtexit MultiCamera0.rtc
rtexit RecognitionResultViewer0.rtc
rtexit Recognition0.rtc
rtdel SetModelID0.rtc
pkill -f SetModelIDComp
