#!/bin/bash
# コンポーネントの起動を待つ
./comwait.sh RecognitionResultViewer0.rtc
./comwait.sh Recognition0.rtc
./comwait.sh SendImage0.rtc
./comwait.sh SetModelID0.rtc
source /usr/local/share/rtshell/shell_support
# rtshell でのコンポーネントパスを取得
# 全コンポーネントが同じ場所にあること
comppath=`./compath.sh Recognition0.rtc`
rtcwd $comppath
rtcon SendImage0.rtc:Stereo3D Recognition0.rtc:Stereo3DIn
rtcon SendImage0.rtc:Reconstruct3D Recognition0.rtc:Reconstruct3D
rtcon SetModelID0.rtc:Recognition Recognition0.rtc:Recognition
rtcon Recognition0.rtc:RecognitionResultViewer RecognitionResultViewer0.rtc:RecognitionResultViewer
