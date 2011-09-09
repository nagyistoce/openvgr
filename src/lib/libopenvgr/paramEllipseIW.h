/* -*- coding: utf-8 -*-
 paramEllipseIW.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

/*
  尺取虫方式で楕円計算する関数のための
  ParamEllipseIW 構造体
  外部から与える楕円計算を制御するパラメータ

  Condition: ELLIPSE_CONDITION_MEAN or ELLIPSE_CONDITION_MAX
             ELLIPSE_CONDITION_MEANのとき、誤差の平均値を評価に使う
             ELLIPSE_CONDITION_MAXのとき、誤差の最大値を評価に使う
  MinLength: 評価の対象にする点列の最小数
  MinShortRad: 短径がこの値より小さい楕円は失敗とみなす
  ThMeanError: ELLIPSE_CONDITION_MEANのときに使う閾値(0.5くらい？）
  ThMaxError: ELLIPSE_CONDITION_MAXのときに使う閾値(2.0くらい？）
  Offset_Mode: MODE_OFFSET_STATIC or MODE_OFFSET_DYNAMIC
              MODE_OFFSET_STATICのとき、Trackの重心を使う
	      MODE_OFFSET_DYNAMIC のとき、各点列の重心をオフセットにする
 */

#ifndef _PARAM_ELLIPSE_IW_H_

#define _PARAM_ELLIPSE_IW_H_

#define  ELLIPSE_CONDITION_MEAN (0)
#define  ELLIPSE_CONDITION_MAX (1)

#define ELLIPSE_OFFSET_STATIC	(0)
#define ELLIPSE_OFFSET_DYNAMIC	(1)

// 最小値
#define MINIMUM_MIN_LENGTH	(5)
#define MINIMUM_MIN_SHORT_RAD	(0.0)
#define MINIMUM_TH_MEAN_ERROR	(0.0)
#define MINIMUM_TH_MAX_ERROR	(0.0)

// 推奨初期値
#define	DEF_PARAME_CONDITION	(ELLIPSE_CONDITION_MEAN)
#define	DEF_PARAME_MIN_LENGTH	(20)
#define	DEF_PARAME_MIN_SHORT_RAD	(2.0)
#define	DEF_PARAME_TH_MEAN_ERROR	(0.5)
#define	DEF_PARAME_TH_MAX_ERROR		(2.0)
#define	DEF_PARAME_OFFSET_MODE		(ELLIPSE_OFFSET_DYNAMIC)


typedef struct _param_ellipse_IW_{
  int		Condition; 
  int		MinLength;   // default = 20
  double	MinShortRad; // default = 2.0
  double	ThMeanError;
  double	ThMaxError;
  int		OffsetMode;
}ParamEllipseIW;

#endif
