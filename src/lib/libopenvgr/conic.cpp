/*
 conic.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file conic.cpp
 * @brief 二次曲線特徴抽出関連関数
 * @date \$Date::                            $
 */
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "vectorutil.h"
#include "parameters.h"
#include "conic.h"

// 小さい方を前に入れ替える
inline static void
comprSwapDouble(double *a, double *b)
{
  if (*a > *b)
    {
      const double swap = *a;
      *a = *b;
      *b = swap;
    }
}

// 小さい順に並べ替える
static void
sortRoot(double root[3], const int nRoot)
{
  if (nRoot <= 1)
    {                           // do nothing
      return;
    }
  else if (nRoot == 2)
    {
      comprSwapDouble(&(root[0]), &(root[1]));
      return;
    }
  else if (nRoot == 3)
    {
      comprSwapDouble(&(root[1]), &(root[2]));
      comprSwapDouble(&(root[0]), &(root[1]));
      comprSwapDouble(&(root[1]), &(root[2]));
    }
  else
    {
      return;
    }
}

// 3次方程式を解いて、3x3の実対称行列の固有値と固有ベクトルを求める
static int
eigen33(double e[3], double ev[3][3], double m[3][3], const double rankDiag)
{
  double coef[4];
  coef[3] = -1.0;
  coef[2] = m[0][0] + m[1][1] + m[2][2];
  coef[1] = -m[0][0] * m[1][1] - m[1][1] * m[2][2] - m[2][2] * m[0][0]
    + m[1][2] * m[2][1] + m[0][2] * m[2][0] + m[0][1] * m[1][0];
  coef[0] = m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0]
    + m[0][2] * m[1][0] * m[2][1] - m[0][0] * m[1][2] * m[2][1]
    - m[0][1] * m[1][0] * m[2][2] - m[0][2] * m[1][1] * m[2][0];

  CvMat vCoef, vE, vEv, vM;
  vCoef = cvMat(4, 1, CV_64FC1, coef);
  vE = cvMat(3, 1, CV_64FC1, e);
  vEv = cvMat(3, 3, CV_64FC1, ev);
  vM = cvMat(3, 3, CV_64FC1, m);

  // ３次方程式を解く
  const int nRoot = cvSolveCubic(&vCoef, &vE);
  // 出てきた解が実数解
  sortRoot(e, nRoot);
  cvEigenVV(&vM, &vEv, &vE, rankDiag);

  return nRoot;
}

// 直線のあてはめ
static void
fitLine(double center[2], double direction[2], double sum[5][5], double offset[2])
{
  if (sum[0][0] == 0.0)
    {
      center[0] = 0.0;
      center[1] = 0.0;
      direction[0] = 0.0;
      direction[1] = 0.0;
      return;
    }

  center[0] = sum[1][0] / sum[0][0];
  center[1] = sum[0][1] / sum[0][0];
  const double M20 = sum[2][0] - pow(center[0], 2) * sum[0][0];
  const double M11 = sum[1][1] - center[0] * center[1] * sum[0][0];
  const double M02 = sum[0][2] - pow(center[1], 2) * sum[0][0];

  const double theta = atan2(M11 * 2.0, M20 - M02) / 2.0;
  direction[0] = cos(theta);
  direction[1] = sin(theta);
  center[0] += offset[0];
  center[1] += offset[1];
  return;
}


// ２次曲線の係数と、特定の点からのおおよその距離を求める
// 簡易計算法なので厳密ではない
// 楕円の場合は外側にあるとプラス
// 楕円の場合は内側にあるとマイナス
static double
distanceSideConic(double coef[6], int* point)
{
  const double x = point[0];
  const double y = point[1];

  double D = coef[0] * x*x + coef[1] * x*y + coef[2] * y*y + coef[3] * x + coef[4] * y + coef[5];
  if (coef[0] < 0.0)
    {
      D *= -1.0;
    }
  const double dx = 2.0 * coef[0] * x + coef[1] * y + coef[3];
  const double dy = 2.0 * coef[2] * y + coef[1] * x + coef[4];
  const double inc = sqrt(pow(dx, 2) + pow(dy, 2));

  if (inc > 0.0)
    {
      if (inc < VISION_EPS)
        {
          return D;
        }
      return D / inc;
    }
  else
    {
      return DBL_MAX;
    }
}

// データ点から２次曲線までの距離の評価：距離の平均
static int
checkConicInConic2(double* maxError, double coef[6],
                   int* point, const int nPoint, const int start,
                   const int end, Parameters parameters)
{
  *maxError = 0.0;
  double error = 0.0;

  int p, pp, pend;
  double dist;
  int nn = 0;
  if (end < start)
    {
      pend = end + nPoint;
    }
  else
    {
      pend = end;
    }
  for (p = start; p <= pend; p++)
    {
      pp = p % nPoint;
      dist = distanceConic(coef, &point[pp * 2]);
      if (dist >= 0.0)
        {
          error += dist;
          nn++;
        }
      if (dist >= parameters.feature2D.maxErrorofConicFit * 3.0)
        {                       // はずれ値をチェック
          *maxError = dist;
          return 0;
        }
    }
  *maxError = error / (double) nn;

  return 1;                     // すべての点の誤差は一定範囲内であった
}


// すべての点から直線までの距離が一定範囲内かどうかのテスト
static int
checkConicInLine(double* maxError, double center[2], double direction[2],
                 int* point, const int nPoint, const int start,
                 const int end, const double errorLine)
{
  const double errorLine2 = pow(errorLine, 2); // 計算しやすいように２乗しておく
  double maxError2 = 0.0;
  double dx, dy, error2 = 0.0;

  *maxError = 0.0;
  if ((center[0] == 0.0) && (center[1] == 0.0) &&
      (direction[0] == 0.0) && (direction[1] == 0.0))
    {
      return 0;
    }

  int p, pp, pend;
  if (end < start)
    {
      pend = end + nPoint;
    }
  else
    {
      pend = end;
    }
  for (p = start; p <= pend; p++)
    {
      pp = p % nPoint;
      dx = (double) point[pp * 2] - center[0];
      dy = (double) point[pp * 2 + 1] - center[1];
      error2 = pow(dx, 2) + pow(dy, 2) - pow(dx * direction[0] + dy * direction[1], 2);
      if (error2 >= errorLine2)
        {
          *maxError = sqrt(error2);
          return 0;             // 誤差は大きい
        }
      if (maxError2 < error2)
        {
          maxError2 = error2;
        }
    }
  *maxError = sqrt(maxError2);

  return 1;                     // すべての点の誤差は一定範囲内であった
}

// ２次曲線当てはめ係数行列をクリアする
void
clearConicSum(double sum[5][5])        // ２次曲線当てはめ係数行列
{
  int ix, iy;
  for (ix = 0; ix < 5; ix++)
    {
      for (iy = 0; iy < 5; iy++)
        {
          sum[ix][iy] = 0.0;
        }
    }
  return;
}

// 1点を２次曲線当てはめ係数行列に加える
void
addConicSum(double sum[5][5],  // ２次曲線当てはめ係数行列。sum[ix][iy] = x^{ix} y^{iy}, ix + iy < 5
            int* point,        // 輪郭点座標
            double* offset)    // 重心のオフセット
{
  const double x = point[0] - offset[0];
  const double y = point[1] - offset[1];

  int ix, iy;
  double X, Y;
  for (ix = 0, X = 1.0; ix < 5; ix++, X *= x)
    {
      for (iy = 0, Y = X; iy < 5 - ix; iy++, Y *= y)
        {
          sum[ix][iy] += Y;
        }
    }

  return;
}

// 1点を２次曲線当てはめ係数行列から引く
void
subConicSum(double sum[5][5],  // ２次曲線当てはめ係数行列。sum[ix][iy] = x^{ix} y^{iy}, ix + iy < 5
            int* point,        // 輪郭点座標
            double* offset)    // 重心のオフセット
{
  const double x = point[0] - offset[0];
  const double y = point[1] - offset[1];

  int ix, iy;
  double X, Y;
  for (ix = 0, X = 1.0; ix < 5; ix++, X *= x)
    {
      for (iy = 0, Y = X; iy < 5 - ix; iy++, Y *= y)
        {
          sum[ix][iy] -= Y;
        }
    }
  return;
}

// ２次曲線の係数と、特定の点からの距離を求める
double
distanceConic(double coef[6],  // 二次曲線係数
              int* point)      // 輪郭点座標
{
  // 簡易計算法なので厳密ではない
  return fabs(distanceSideConic(coef, point));        // 絶対距離
}

// ２次曲線の係数から、そのタイプを調べる
ConicType
getConicType(double coef[6])
{
  const double D = 4.0 * coef[0] * coef[2] - coef[1] * coef[1];
  const double norm2 = 4.0 * (coef[0]*coef[0] + coef[2]*coef[2]) + coef[1]*coef[1];

  // ここで楕円か、双曲線か、放物線か調べる
  if (D > VISION_EPS * norm2)
    {
      return ConicType_Ellipse; // ELLIPSE
    }
  else if (D < -VISION_EPS * norm2)
    {
      return ConicType_Hyperbola;       // HYPERBOLA
    }
  else
    {
      return ConicType_Parabola;        // PARABOLA
    }
}

// ２次曲線の係数から、その性質を調べる
void
getConicProperty(double coef[6],       // 二次曲線係数
                 ConicType* type,      // 二次曲線属性
                 double center[2],     // 楕円中心
                 double axis[2][2],    // 楕円回転行列
                 double* Laxis,        // 楕円の長軸の長さ
                 double* Saxis)        // 楕円の短軸の長さ
{
  const double D = 4.0 * coef[0] * coef[2] - coef[1] * coef[1];
  double F;
  int i;

  if (*type != ConicType_Ellipse)
    {
      return;
    }

  // 中心を求める 放物線は適用外なので注意！
  center[0] = -(2.0 * coef[2] * coef[3] - coef[1] * coef[4]) / D;
  center[1] = -(2.0 * coef[0] * coef[4] - coef[1] * coef[3]) / D;

  // 中心を移動した場合の F は
  F = (coef[0] * pow(center[0], 2) + coef[1] * center[0] * center[1]
       + coef[2] * pow(center[1], 2) + coef[3] * center[0] + coef[4] * center[1] + coef[5]);
  // これにより a(x+xc)^2 + b(x+xc)(y+yc) + c(y+yc)^2 + F = 0 の形になる

  // 各軸をx,y軸に一致させた場合のA, Cを求める
  double a[2][2] = { {coef[0], coef[1] / 2.0}, {coef[1] / 2.0, coef[2]} };
  double e[2] = {0, 0}, ev[2][2] ={{1, 0}, {0, 1}};
  double A, C;

  eigenM22(e, ev, a, VISION_EPS);
  if (e[0] * e[1] < VISION_EPS) // 楕円でない場合
    {
      return;
    }

  if (e[0] >= 0.0) // 2個の固有ベクトルが正：ev[1]が長軸の方向
    {
      for (i = 0; i < 2; ++i)
        {
          axis[0][i] =  ev[1][i];
          axis[1][i] = -ev[0][i];
        }
      A = e[1];
      C = e[0];
    }
  else
    {              // 2個の固有ベクトルが負：ev[0]が長軸の方向
      for (i = 0; i < 2; ++i)
        {
          axis[0][i] =  ev[0][i];
          axis[1][i] =  ev[1][i];
        }
      A = e[0];
      C = e[1];
    }

  // F による規格化
  if (F != 0.0)
    {
      A /= -(F);
      C /= -(F);
      // これで A x^2 + C y^2 = 1 の標準形
      if ((A > 0.0) && (C > 0.0))
        {                       // 楕円
          if (A <= C)
            {
              *Laxis = sqrt(1.0 / A);
              *Saxis = sqrt(1.0 / C);
            }
          else
            {
              *Laxis = sqrt(1.0 / C);
              *Saxis = sqrt(1.0 / A);
            }
          return;
        }
      else if ((A <= 0.0) && (C <= 0.0))
        {                       // 虚円もしくは放物線
          *Laxis = 0.0;
          *Saxis = 0.0;
          return;
        }
      else
        {
          if (A > C)
            {
              *Laxis = sqrt(1.0 / A);
              *Saxis = sqrt(-1.0 / C);
              return;
            }
          else
            {
              *Laxis = sqrt(1.0 / C);
              *Saxis = sqrt(-1.0 / A);
              return;
            }
        }
    }
  // これは A x^2 + C y^2 = 0 の形式
  *Laxis = 0.0;
  *Saxis = 0.0;
  return;
}

// 二次曲線当てはめの微分係数行列から２次曲線係数を計算する
// 戻り値：固有値の数
int
fitConic(double sum[5][5],     // 二次曲線当てはめの微分係数行列
         double coef[3][6],    // 二次曲線係数
         double* offset)       // 重心のオフセット
{
  double P00[3][3] = {
    {sum[4][0], sum[3][1], sum[2][2]},
    {sum[3][1], sum[2][2], sum[1][3]},
    {sum[2][2], sum[1][3], sum[0][4]}
  };
  double P01[3][3] = {
    {sum[3][0], sum[2][1], sum[2][0]},
    {sum[2][1], sum[1][2], sum[1][1]},
    {sum[1][2], sum[0][3], sum[0][2]}
  };
  double P10[3][3] = {
    {sum[3][0], sum[2][1], sum[1][2]},
    {sum[2][1], sum[1][2], sum[0][3]},
    {sum[2][0], sum[1][1], sum[0][2]}
  };
  double P11[3][3] = {
    {sum[2][0], sum[1][1], sum[1][0]},
    {sum[1][1], sum[0][2], sum[0][1]},
    {sum[1][0], sum[0][1], sum[0][0]}
  };


  double TMP1[3][3];
  CvMat vP10 = cvMat(3, 3, CV_64FC1, P10);
  CvMat vP11 = cvMat(3, 3, CV_64FC1, P11);
  CvMat vTMP1 = cvMat(3, 3, CV_64FC1, TMP1);
  double ret = cvSolve(&vP11, &vP10, &vTMP1);
  if (ret == 0)
    {
      return -1;
    }

  double TMP2[3][3];
  mulM33(P01, TMP1, TMP2);
  double TMP3[3][3];
  subM33(P00, TMP2, TMP3);
  double e[3];                  // eigen value
  double ev[3][3];              // eigen vectors
  // 固有値の計算
  const int nEigen = eigen33(e, ev, TMP3, VISION_EPS);

  int i;
  double newcoef3;
  double newcoef4;
  double newcoef5;
  for (i = 0; i < nEigen; i++)
    {
      copyV3(ev[i], coef[i]);
      mulM33V3(TMP1, ev[i], &(coef[i][3]));
      mulV3S(-1.0, &(coef[i][3]), &(coef[i][3]));

      // オフセットを考慮した計算
      newcoef3 = -2.0 * coef[i][0] * offset[0] - coef[i][1] * offset[1] + coef[i][3];
      newcoef4 = -coef[i][1] * offset[0] - 2.0 * coef[i][2] * offset[1] + coef[i][4];
      newcoef5 = coef[i][0] * offset[0] * offset[0] + coef[i][1] * offset[0] * offset[1]
                 + coef[i][2] * offset[1] * offset[1] - coef[i][3] * offset[0]
                 - coef[i][4] * offset[1] + coef[i][5];

      coef[i][3] = newcoef3; 
      coef[i][4] = newcoef4;
      coef[i][5] = newcoef5;
    }
  return nEigen;
}


// 二次曲線当てはめ
// 戻り値：二次曲線の分類
ConicType
fitConicAny(double retcoef[6],         // 二次曲線係数
            double* retError,          // エラーコード
            double sum[5][5],          // 二次曲線当てはめの微分係数行列
            int* point,                // 当てはめる輪郭の点列
            const int nPoint,          // 当てはめる輪郭の点数
            const int start,           // 当てはめる輪郭点列の始点
            const int end,             // 当てはめる輪郭点列の終点
            Parameters parameters,     // 全パラメータ
            int line_detect_flag,      // 直線検出フラグ
            double* offset)            // 重心のオフセット
{
  *retError = 0.0;
  if (sum[0][0] <= 2.0)
    {
      return ConicType_Unknown; // 当てはまらなかったことにする
    }

  // 直線にあてはめてみる
  double center[2];
  double direction[2];
  double error;
  double coef[3][6];
  int nConic;
  double minError;
  int c, minC;

  if (line_detect_flag)
    {
      fitLine(center, direction, sum, offset);
      if (checkConicInLine(&error, center, direction, point, nPoint, start, end,
                           parameters.feature2D.maxErrorofLineFit) == 1)
        {
          // 直線でも十分にあてはまってしまった
          retcoef[0] = retcoef[1] = retcoef[2] = 0.0;
          retcoef[3] = direction[1];
          retcoef[4] = -direction[0];
          retcoef[5] = direction[0] * center[1] - direction[1] * center[0];
          *retError = error;
          return ConicType_Line;
        }
      return ConicType_Unknown;
    }

  nConic = fitConic(sum, coef, offset);

  if (nConic <= 0)
    {
      retcoef[0] = retcoef[1] = retcoef[2] = retcoef[3] = retcoef[4] = retcoef[5] = 0.0;
      return ConicType_Unknown;
    }

  // 解の中から妥当なものを選択する
  minError = DBL_MAX;
  minC = -1;
  for (c = 0; c < nConic; c++)
    {
      error = 0.0;
      if (checkConicInConic2(&error, coef[c], point, nPoint, start, end, parameters) == 1)
        {
          if (minError > error)
            {                   // よりよくあてはまったものがみつかった
              minC = c;
              minError = error;
            }
        }
    }

  if (minC == -1)
    {                           // 誤差範囲内であてはまる曲線がなかった
      retcoef[0] = retcoef[1] = retcoef[2] = retcoef[3] = retcoef[4] = retcoef[5] = 0.0;
      return ConicType_Unknown;
    }
  else
    {
      memcpy(retcoef, coef[minC], sizeof(double) * 6);
      *retError = minError;
      return getConicType(retcoef);
    }
}
