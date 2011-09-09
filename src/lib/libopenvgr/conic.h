/*
 conic.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file conic.h
 * @brief 二次曲線特徴抽出関連関数
 * @date \$Date::                            $
 */

#ifndef _CONIC_H
#define _CONIC_H

#include <math.h>
#include "parameters.h"

// Conic のタイプ定義
typedef enum ConicType
{
  ConicType_Unknown,
  ConicType_Line,
  ConicType_Ellipse,
  ConicType_Hyperbola,
  ConicType_Parabola
} ConicType;

// ２次曲線当てはめ係数行列をクリアする
void clearConicSum(double sum[5][5]);  // ２次曲線当てはめ係数行列

// 1点を２次曲線当てはめ係数行列に加える
void addConicSum(double sum[5][5],     // ２次曲線当てはめ係数行列
                 int* point,           // 輪郭点座標
                 double* offset);      // 楕円係数計算時のオフセット

// 1点を２次曲線当てはめから引く
void subConicSum(double sum[5][5],     // ２次曲線当てはめ係数行列
                 int* point,           // 輪郭点座標
                 double* offset);      // 楕円係数計算時のオフセット

// ２次曲線の係数と、特定の点からの距離を求める
double distanceConic(double coef[6],   // 二次曲線係数
                     int* point);      // 輪郭点座標

// ２次曲線の係数から、そのタイプを調べる
ConicType getConicType(double coef[6]);

// ２次曲線の係数から、その性質を調べる
void getConicProperty(double coef[6],    // 二次曲線係数
                      ConicType* type,   // 二次曲線属性
                      double center[2],  // 楕円中心
                      double axis[2][2], // 楕円回転行列
                      double* Laxis,     // 楕円の長軸の長さ (画素)
                      double* Saxis);    // 楕円の短軸の長さ (画素)

// 二次曲線当てはめの微分係数行列から２次曲線係数を計算する
// 戻り値：固有値の数
int fitConic(double sum[5][5],           // 二次曲線当てはめの微分係数行列
             double coef[3][6],          // 二次曲線係数
             double* offset);            // 楕円係数計算時のオフセット

// 二次曲線当てはめ
// 戻り値：二次曲線の分類
ConicType fitConicAny(double retcoef[6],       // 二次曲線係数
                      double* retError,        // エラーコード
                      double sum[5][5],        // 二次曲線当てはめの微分係数行列
                      int* point,              // 当てはめる輪郭の点列
                      const int nPoint,        // 当てはめる輪郭の点数
                      const int start,         // 当てはめる輪郭点列の始点
                      const int end,           // 当てはめる輪郭点列の終点
                      Parameters parameters,   // 全パラメータ
                      int line_detect_flag,    // 直線検出フラグ
                      double* offset);         // 楕円係数計算時のオフセット

#endif // _CONIC_H
