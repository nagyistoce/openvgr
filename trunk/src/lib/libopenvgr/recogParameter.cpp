/*
 recogParameter.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "parameters.h"
#include "visionErrorCode.h"

#ifndef MAX_PATH
#define MAX_PATH 256
#endif

//
//! Parameters 構造体にデフォルト値をセットする。
//
void
setDefaultRecogParameter(Parameters& param)
{
  // パラメータ領域のクリア
  memset(&param, 0x0, sizeof(Parameters));

  param.pairing = DBL_LR;       // ステレオペア設定
  param.outputCandNum = 20;     // 出力候補数

  // 認識パラメータの設定

  param.feature2D.edgeDetectFunction = 0;       // エッジ検出アルゴリズム
  param.feature2D.edgeStrength = 5.0;    // 検出するエッジの最低微分強度

  param.feature2D.maxErrorofLineFit = 0.5;
  // 直線を当てはめるときの最大誤差
  param.feature2D.maxErrorofConicFit = 0.8;
  // 二次曲線を当てはめるときの最大誤差
  param.feature2D.overlapRatioLine = 0.7;
  // 直線、双曲線の特徴点を抽出する区間の重複可能な最大比率
  param.feature2D.overlapRatioCircle = 0.8;
  // 楕円の特徴点を抽出する区間の重複可能な最大比率

  param.feature2D.min_length_line = 15.0;         // 直線の最小長さ
  param.feature2D.max_distance_end_points = 10.0; // 端点間距離の閾値

  // 以下は固定
  param.feature2D.minFragment = 10;
  // 検出するエッジの最低外周長
  param.feature2D.max_length_delete_line = 30.0;
  // 楕円検出直前に削除する直線の最大長さ
  param.feature2D.min_radian_hyperbola = 15.0 / 180.0 * M_PI;
  // 双曲線のなす角度閾値 (0, 180 度に近いものを除去する。)
  param.feature2D.min_length_hyperbola_data = 2.0;
  // 双曲線での中心からデータまでの距離の閾値
  param.feature2D.min_length_hyperbola_vector = 10.0;
  // 双曲線での中心から端点までの距離の閾値
  param.feature2D.min_length_ellipse_axis = 10.0;       // 楕円の軸長の閾値
  param.feature2D.min_filling_ellipse = 0.2;    // 楕円の充填率の閾値
  param.feature2D.max_flatness_ellipse = 4.0;
  // 楕円の偏平率（長軸/短軸)
  param.feature2D.max_distance_ellipse_grouping = 10.0;
  // 中心距離判定閾値
  param.feature2D.min_distance_ellipse_pairing = 0.0;
  // 中心距離判定閾値
  param.feature2D.max_distance_ellipse_pairing = 100.0;
  // 中心距離判定閾値
  param.feature2D.min_length_ellipse_axisS = 5.0;       // 楕円の軸長の閾値
  param.feature2D.min_length_ellipse_axisL = 10.0;      // 楕円の軸長の閾値
  param.feature2D.max_length_ellipse_axisL = 50.0;      // 楕円の軸長の閾値

  param.stereo.rdif = 5;        // 対応円半径許容差
  param.stereo.ndif = 10;       // 対応円法線角度許容差

  param.stereo.ethr = 5.0;      // ステレオ対応誤差

  param.stereo.amin = 85;       // 頂点 線分の成す角度最小値
  param.stereo.amax = 95;       // 頂点 線分の成す角度最小値

#if 0
  param.stereo.lmin = 10;       // 頂点と端点の距離最小値
  param.stereo.lmax = 200;      // 頂点と端点の距離最大値
#endif

  param.stereo.depn = 500;      // カメラからの距離(near)
  param.stereo.depf = 2000;     // カメラからの距離(far)

  double tolerance1 = 5.0;      // 頂点の角度差の許容割合
  double tolerance2 = 10.0;     // 円の半径の差の許容割合
  double pdist = 3.0;           // モデルサンプル点間隔(mm)

  param.match.tolerance1 = tolerance1;
  param.match.tolerance2 = tolerance2;
  param.match.pdist = pdist;

  param.paramEIW.Condition = DEF_PARAME_CONDITION;
  param.paramEIW.MinLength = DEF_PARAME_MIN_LENGTH;
  param.paramEIW.MinShortRadPrev = DEF_PARAME_MIN_SHORT_RAD_PREV;
  param.paramEIW.MinShortRadPost = DEF_PARAME_MIN_SHORT_RAD_POST;
  param.paramEIW.ThMeanError = DEF_PARAME_TH_MEAN_ERROR;
  param.paramEIW.ThMaxError = DEF_PARAME_TH_MAX_ERROR;
  param.paramEIW.OffsetMode = DEF_PARAME_OFFSET_MODE;
}

enum paramKey
{
  eStereoPair,
  eOutputCandNum,

  eEdgeDetectFunction,
  eEdgeStrength,
  eMaxErrorOfLineFit,
  eMaxErrorOfConicFit,
  eOverlapRatioLine,
  eOverlapRatioCircle,
  eMinLengthLine2D,
  eHDMax,

  eDepN,
  eDepF,
  eAMin,
  eAMax,
#if 0
  eLMin,
  eLMax,
#endif
  eStereoError,

  eIwCondition,
  eIwMinLength,
  eIwMinShortRadPrev,
  eIwMinShortRadPost,
  eIwThMeanError,
  eIwThMaxError,
  eIwOffsetMode,

  eParamSentinel
};

static const char* paramKeyString[] = {
  "StereoPair",
  "OutputCandNum",

  "EdgeDetectFunction",
  "EdgeStrength",
  "MaxErrorOfLineFit",
  "MaxErrorOfConicFit",
  "OverlapRatioLine",
  "OverlapRatioCircle",
  "MinLengthLine2D",
  "HDMax",

  "DepN",
  "DepF",
  "AMin",
  "AMax",
#if 0
  "LMin",
  "LMax",
#endif
  "StereoError",

  "IW_Condition",
  "IW_MinLength",
  "IW_MinShortRadPrev",
  "IW_MinShortRadPost",
  "IW_ThMeanError",
  "IW_ThMaxError",
  "IW_OffsetMode"
};

//
//! ファイルから、Parameters 構造体に設定値を読み込む。
//
// フォーマット：
//
//   パラメータ名: value
//
int
loadRecogParameter(char* path, Parameters& param)
{
  if ((path == NULL) || (strlen(path) == 0))
    {
      return VISION_PARAM_ERROR;
    }

  FILE* fp = fopen(path, "r");
  if (fp == NULL)
    {
      return VISION_FILE_OPEN_ERROR;
    }

  setDefaultRecogParameter(param);

  char buffer[MAX_PATH];
  char* p;
  int i = 0;

  while (fgets(buffer, MAX_PATH, fp))
    {
      if ((buffer[0] == '\n') || (buffer[0] == '#'))
        {
          // 空白行か、'#' で始まる行は SKIP。
          continue;
        }

      for (i = 0; i < eParamSentinel; i++)
        {
          if (strncmp(buffer,
		      paramKeyString[i], strlen(paramKeyString[i])) != 0)
            {
              continue;
            }
          p = &(buffer[strlen(paramKeyString[i])]);
          while ((*p != '\0') && ((*p < '0') || (*p > '9')) && (*p != '-'))
            {
              // 数字以外はスキップ
              p++;
            }
          if (*p == '\0')
            {
              continue;
            }

          switch (i)
            {
            case eStereoPair:
              // ステレオペア設定
              param.pairing = (StereoPairing) atoi(p);
              break;
            case eOutputCandNum:
              // 出力候補数
              param.outputCandNum = atoi(p);
              break;

            case eEdgeDetectFunction:
              // エッジ検出アルゴリズム
              param.feature2D.edgeDetectFunction = atoi(p);
              break;
            case eEdgeStrength:
              // 検出するエッジの最低微分強度
              param.feature2D.edgeStrength = atof(p);
              break;
            case eMaxErrorOfLineFit:
              // 直線を当てはめるときの最大誤差
              param.feature2D.maxErrorofLineFit = atof(p);
              break;
            case eMaxErrorOfConicFit:
              // 二次曲線を当てはめるときの最大誤差
              param.feature2D.maxErrorofConicFit = atof(p);
              break;
            case eOverlapRatioLine:
              // 特徴点を抽出する区間の重複可能な最大比率
              param.feature2D.overlapRatioLine = atof(p);
              break;
            case eOverlapRatioCircle:
              // 特徴点を抽出する区間の重複可能な最大比率
              param.feature2D.overlapRatioCircle = atof(p);
              break;
            case eMinLengthLine2D:
              // エッジ画像上での頂点構成直線最小長
              param.feature2D.min_length_line = atof(p);
              break;
            case eHDMax:
              // 頂点生成時の端点間距離最大値
              param.feature2D.max_distance_end_points = atof(p);
              break;

            case eDepN:
              // カメラからの距離(near)
              param.stereo.depn = atoi(p);
              break;
            case eDepF:
              // カメラからの距離(far)
              param.stereo.depf = atoi(p);
              break;
            case eAMin:
              // 頂点 線分の成す角度最小値
              param.stereo.amin = atoi(p);
              break;
            case eAMax:
              // 頂点 線分の成す角度最小値
              param.stereo.amax = atoi(p);
              break;
#if 0
            case eLMin:
              // 頂点と端点の距離最小値
              param.stereo.lmin = atoi(p);
              break;
            case eLMax:
              // 頂点と端点の距離最大値
              param.stereo.lmax = atoi(p);
              break;
#endif
            case eStereoError:
              // ステレオ対応誤差
              param.stereo.ethr = atof(p);
              break;

            case eIwCondition:
              param.paramEIW.Condition = (atoi(p) != 0 ? ELLIPSE_CONDITION_MAX : ELLIPSE_CONDITION_MEAN);
              break;
            case eIwMinLength:
              param.paramEIW.MinLength = atoi(p);
              break;
            case eIwMinShortRadPrev:
              param.paramEIW.MinShortRadPrev = atof(p);
              break;
            case eIwMinShortRadPost:
              param.paramEIW.MinShortRadPost = atof(p);
              break;
            case eIwThMeanError:
              param.paramEIW.ThMeanError = atof(p);
              break;
            case eIwThMaxError:
              param.paramEIW.ThMaxError = atof(p);
              break;
            case eIwOffsetMode:
              param.paramEIW.OffsetMode = (atoi(p) != 0 ? ELLIPSE_OFFSET_STATIC : ELLIPSE_OFFSET_DYNAMIC);
              break;

            default:
              fclose(fp);
              return VISION_FILE_FORMAT_ERROR;
            }

          break;
        }
    }

  fclose(fp);

  return 0;
}

int
loadDebugParameter(int text, int image, int display, Parameters& param)
{
  if ( text )
    {
      param.dbgtext = 1;
    }

  if( display )
    {
      param.dbgdisp = 1;
      // 表示のために画像生成は必須
      param.dbgimag = 1;
    }
  else if( image )
    {
      param.dbgimag = 1;
    }

  return 0; // 今は 0 しか返さない
}
