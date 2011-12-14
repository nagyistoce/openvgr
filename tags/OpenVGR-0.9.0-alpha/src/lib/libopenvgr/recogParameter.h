/*
 recogParameter.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
/*!
 * @file recogParameter.h
 * @brief 認識パラメータ設定関連
 */

#ifndef _RECOG_PARAMETER_H
#define _RECOG_PARAMETER_H

#include "parameters.h"

// Parameters 構造体にデフォルト値をセットする。
extern void setDefaultRecogParameter(Parameters& param);

// ファイルから Parameters 構造体に設定値を読み込む。
extern int loadRecogParameter(char* path, Parameters& param);

// Parameters 構造体にデバッグ用パラメータを設定する。
extern int loadDebugParameter(int text, int image, int display, Parameters& param);

#endif // _RECOG_PARAMETER_H
