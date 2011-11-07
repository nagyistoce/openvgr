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
  MinShortRadPrev: 短径がこの値より小さい楕円は失敗とみなす（距離計算前）通常は変更しない
  MinShortRadPost: 短径がこの値より小さい楕円は失敗とみなす（距離計算後）
  ThMeanError: ELLIPSE_CONDITION_MEANのときに使う閾値(0.5くらい？）
  ThMaxError: ELLIPSE_CONDITION_MAXのときに使う閾値(2.0くらい？）
  Offset_Mode: MODE_OFFSET_STATIC or MODE_OFFSET_DYNAMIC
              MODE_OFFSET_STATICのとき、Trackの重心を使う
	      MODE_OFFSET_DYNAMIC のとき、各点列の重心をオフセットにする
 */

#ifndef _PARAM_ELLIPSE_IW_H_

#define _PARAM_ELLIPSE_IW_H_

enum paramEllipseIW_Ellipse_with_line_key
  {
    ENABLE_ELLIPSE_NONE, // 0
    ENABLE_ELLIPSE_WITH_LINE, // 1
    ENABLE_ELLIPSE_WITHOUT_LINE // 2
  };

enum paramEllipseIW_Old_Merge_func_key
  {
    DISABLE_OLD_MERGE_FUNC, // 0
    ENABLE_OLD_MERGE_FUNC // 1
  };

//#define  ELLIPSE_CONDITION_MEAN (0)
//#define  ELLIPSE_CONDITION_MAX (1)

enum paramEllipseIW_ErrCond_key
  {
    ELLIPSE_CONDITION_MEAN, // 0
    ELLIPSE_CONDITION_MAX   // 1
  };

//#define ELLIPSE_OFFSET_STATIC	(0)
//#define ELLIPSE_OFFSET_DYNAMIC	(1)
enum paramEllipseIW_OffsetMode_key
  {
    ELLIPSE_OFFSET_STATIC,	
    ELLIPSE_OFFSET_DYNAMIC
  };

// 最小値
#define MINIMUM_MIN_LENGTH	(5)
#define MINIMUM_MIN_SHORT_RAD	(0.0)
#define MINIMUM_TH_MEAN_ERROR	(0.0)
#define MINIMUM_TH_MAX_ERROR	(0.0)

// 推奨初期値
#define	DEF_PARAME_CONDITION	(ELLIPSE_CONDITION_MEAN)
#define	DEF_PARAME_MIN_LENGTH	(20)
//#define	DEF_PARAME_POST_MIN_LENGTH	(100)
#define	DEF_PARAME_POST_MIN_LENGTH	(50)
//#define	DEF_PARAME_MIN_SHORT_RAD_PREV	(2.0)
#define	DEF_PARAME_MIN_SHORT_RAD_PREV	(1.0)
#define	DEF_PARAME_MIN_SHORT_RAD_POST	(2.0)
//#define	DEF_PARAME_TH_MEAN_ERROR	(0.5)
#define	DEF_PARAME_TH_MEAN_ERROR	(0.4)
#define	DEF_PARAME_TH_MAX_ERROR		(2.0)
//#define DEF_PARAME_TH_MEAN_ERROR_MERGING	(0.55)
#define DEF_PARAME_TH_MEAN_ERROR_MERGING	(0.44)
#define DEF_PARAME_TH_MAX_ERROR_MERGING	(2.2)
//#define DEF_PARAME_MIN_DETERMINANT	(1e-6)
#define DEF_PARAME_MIN_DETERMINANT	(1e-3)
//#define DEF_PARAME_MAX_EIGEN_VALUE_RATIO	(1000.0)
#define	DEF_PARAME_OFFSET_MODE		(ELLIPSE_OFFSET_DYNAMIC)
#define DEF_PARAME_SW_LINE_ELLIPSE	(ENABLE_ELLIPSE_WITH_LINE|\
                                         ENABLE_ELLIPSE_WITHOUT_LINE)
#define DEF_PARAME_SW_OLD_MERGE_FUNC	(ENABLE_OLD_MERGE_FUNC)

typedef struct _param_ellipse_IW_{
  int		Condition;
  int		MinLength;   // default = 20
  int		PostMinLength;   // default = 100
  double	MinShortRadPrev; 
  double	MinShortRadPost; // default = 2.0
  double	ThMeanError;
  double	ThMaxError;
  double	ThMeanErrorMerging; // used in merging
  double	ThMaxErrorMerging; // used in merging
  double	MinDeterminant; 
  //  double	MaxEigenValueRatio;
  int		OffsetMode;
  int           SwLineEllipse;
  int           SwOldMergeFunc;
}ParamEllipseIW;

#endif
