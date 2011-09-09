/*
 common.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file common.h
 * @brief 各種の共通定義
 * @date \$Date::                            $
 */

#ifndef _COMMON_H
#define _COMMON_H
#include <math.h>
#include "visionErrorCode.h"

#define VISION_EPS	1.0e-10 // 0と見做す大きさ

#define VISIBLE   1
#define INVISIBLE 0

typedef struct Data_2D          // ２次元位置
{
  double col;
  double row;
} Data_2D;

typedef enum StereoPairing     // ステレオの画像の組み合わせ指定
{
  DBL_LR,
  DBL_LV,
  DBL_RV,
  TBL_OR,
  TBL_AND
} StereoPairing;

#define USE_DISTANCETRANSFORM // Hausdorff distance を用いた対応点評価

#endif // _COMMON_H
