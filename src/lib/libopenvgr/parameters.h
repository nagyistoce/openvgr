/*
 parameters.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file parameters.h
 * @brief 処理パラメータ設定関連関数
 * @date \$Date::                            $
 */

#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#include "common.h"
#include "paramEllipseIW.h"

//! ２次元特徴抽出用パラメータ
typedef struct ParametersFeature2D
{
  //! エッジ検出アルゴリズム (0: Sobel3x3 1: Sobel 5x5)
  int edgeDetectFunction;
  //! 検出するエッジの最低微分強度
  double edgeStrength;
  //! 検出するエッジの最低外周長（画素）
  int minFragment;
  //! 直線をあてはめる時の最大誤差（画素）
  double maxErrorofLineFit;
  //! 同一の線分と見なす最大端点距離（画素）
  double max_distance_similar_line;
  //! 二次曲線をあてはめる時の最大誤差（画素）
  double maxErrorofConicFit;
  //! 直線、双曲線の特徴点を抽出する区間の重複可能な最大比率（0.0から1.0)
  double overlapRatioLine;
  //! 楕円の特徴点を抽出する区間の重複可能な最大比率（0.0から1.0)
  double overlapRatioCircle;
  //! 削除する直線の最大の長さ（画素）
  double max_length_delete_line;
  //! 双曲線のなす角度閾値（0,180度に近いものを除去する）（ラジアン）
  double min_radian_hyperbola;
  //! 双曲線での中心からデータまでの距離の閾値（画素）
  double min_length_hyperbola_data;
  //! 双曲線での中心から端点までの距離の閾値（画素）
  double min_length_hyperbola_vector;
  //! 楕円の軸長の閾値（画素）
  double min_length_ellipse_axis;
  //! 楕円の充填率の閾値（0.0から1.0）
  double min_filling_ellipse;
  //! 楕円の偏平率（長軸/短軸)
  double max_flatness_ellipse;
  //! 端点間距離の閾値（画素）
  double max_distance_end_points;
  //! 直線の長さの閾値（画素）
  double min_length_line;
  //! 中心距離判定閾値（画素）
  double max_distance_ellipse_grouping;
  //! 中心距離判定閾値（画素）
  double min_distance_ellipse_pairing;
  //! 中心距離判定閾値（画素）
  double max_distance_ellipse_pairing;
  //! 楕円の軸長の閾値（画素）
  double min_length_ellipse_axisS;
  //! 楕円の軸長の閾値（画素）
  double min_length_ellipse_axisL;
  //! 楕円の軸長の閾値（画素）
  double max_length_ellipse_axisL;
  //! 検出しない特徴のフラグ
  int no_search_features;
} ParametersFeature2D;

//! ステレオ対応処理用パラメータ
typedef struct ParmetersStereo
{
  //! 対応誤差閾値（mm）
  double ethr;
  //! 半径許容差（mm）
  double rdif;
  //! 左右法線角度許容差（度）
  double ndif;
  //! 円中心・頂点位置奥行開始（mm）
  double depn;
  //! 円中心・頂点位置奥行終了（mm）
  double depf;
  //! 頂点 角度最小値（度）
  double amin;
  //! 頂点 角度最大値（度）
  double amax;
#if 0
  //! 頂点 線分最小値（mm）
  double lmin;
  //! 頂点 線分最大値（mm）
  double lmax;
#endif
} ParametersStereo;

//! 認識用パラメータ
typedef struct ParametersMatch
{
  //! 頂点の角度差の許容割合(%)
  double tolerance1;
  //! 円の半径の差の許容割合(%)
  double tolerance2;
  //! モデルサンプル点間隔（mm）
  double pdist;
} ParametersMatch;

//! 全パラメータ
typedef struct Parameters
{
  ParametersFeature2D feature2D;
  ParametersStereo stereo;
  ParametersMatch match;
  StereoPairing pairing;
  ParamEllipseIW paramEIW;

  //! 出力候補数
  int outputCandNum;

  //! 画像サイズ横（画素）
  int colsize;
  //! 画像サイズ縦（画素）
  int rowsize;
  //! 画像サイズ総画素数
  int imgsize;
  //! デバッグテキスト生成
  int dbgtext;
  //! デバッグ画像生成
  int dbgimag;
  //! デバッグ画像表示
  int dbgdisp;
} Parameters;

#endif // _PARAMETERS_H
