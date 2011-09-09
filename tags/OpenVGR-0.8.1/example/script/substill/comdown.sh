#!/bin/bash
source /usr/local/share/rtshell/shell_support
# rtshell でのコンポーネントパスを取得
# 全コンポーネントが同じ場所にあること
comppath=`./compath.sh Recognition0.rtc`
rtcwd $comppath
rtdis SendImage0.rtc
rtdis RecognitionResultViewer0.rtc
rtdis Recognition0.rtc
rtexit RecognitionResultViewer0.rtc
rtexit Recognition0.rtc
rtexit SendImage0.rtc
rtexit SetModelID0.rtc
