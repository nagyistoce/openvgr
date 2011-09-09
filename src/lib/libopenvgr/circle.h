/*
 circle.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file circle.h
 * @brief 3次元円特徴生成関連関数
 * @date \$Date::                            $
 */

#ifndef _CIRCLE_H
#define _CIRCLE_H

#include "extractFeature.h"
#include "stereo.h"

// 二次元楕円のステレオ対応から三次元空間中の真円を推定・復元する
void EllipseToCircle(StereoPairing pairing,    // ステレオペア情報
		     CalibParam calib,         // キャリブレーションデータ
		     StereoData& stereo,       // ステレオ対応情報（復元真円情報を含む）
		     unsigned char* edgeL,     // 評価用エッジ画像（左）
		     unsigned char* edgeR,     // 評価用エッジ画像（右）
		     Parameters parameters);   // 全パラメータ

#endif // _CIRCLE_H
