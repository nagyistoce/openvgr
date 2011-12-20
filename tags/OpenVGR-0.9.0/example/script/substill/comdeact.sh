#!/bin/bash
source /usr/local/share/rtshell/shell_support
# rtshell でのコンポーネントパスを取得
# 全コンポーネントが同じ場所にあること
comppath=`./compath.sh Recognition0.rtc`
rtcwd $comppath
rtdeact RecognitionResultViewer0.rtc
rtdeact Recognition0.rtc
rtdeact SendImage0.rtc
rtdeact SetModelID0.rtc
