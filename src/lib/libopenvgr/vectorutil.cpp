/*
 vectorutil.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file vectorutil.cpp
 * @brief ベクトル処理、行列処理　ユーティリティ関数
 * @date \$Date::                            $
 */
#include <cv.h>

#include "vectorutil.h"
#include "common.h"
#include "quaternion.h"

// 実数値のゼロチェック
int
isZero(double value)
{
  // 数値の絶対値がしきい値よりも小さければ零と見なす
  if (fabs(value) < VISION_EPS)
    {
      // 零とみなせる場合
      return 1;
    }
  else
    {
      // 非零とみなせる場合
      return 0;
    }
}


//--------------------
// ２次元ベクトル演算
//--------------------

// コピー
// out == in でも動作する
void
copyV2(V2 in, V2 out)
{
  const int dimension = 2;
  int d;
  for (d = 0; d < dimension; d++)
    {
      out[d] = in[d];
    }
  return;
}

// スカラー積
// out == in でも動作する
void
mulV2S(V2 in, const double s, V2 out)
{
  const int dimension = 2;

  int d;
  for (d = 0; d < dimension; d++)
    {
      out[d] = in[d] * s;
    }
  return;
}

// ノルム
double
getNormV2(V2 in)
{
  return sqrt(in[0] * in[0] + in[1] * in[1]);
}

// 規格化
// out == in でも動作する
void
normalizeV2(V2 in, V2 out)
{
  double norm = getNormV2(in);
  if (isZero(norm))
    {
      copyV2(in, out);
    }
  else
    {
      mulV2S(in, 1.0 / norm, out);
    }
  return;
}

// ２点間の距離
double
getDistanceV2(V2 in1, V2 in2)
{
  V2 sub;
  int i;

  for (i = 0; i < 2; i++)
    {
      sub[i] = in1[i] - in2[i];
    }

  return getNormV2(sub);
}

// ベクトルのなす角（ラジアン）
double 
getAngle2D(double vec1[2], double vec2[2])
{
  double dotproduct;
  double norm;

  dotproduct = vec1[0] * vec2[0] + vec1[1] * vec2[1];
  norm = sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1]) * 
    sqrt(vec2[0] * vec2[0] + vec2[1] * vec2[1]);

  dotproduct = dotproduct / norm;
  if(dotproduct > 1.0)
    {
      return(0.0);
    }
  else if(dotproduct < -1.0)
    {
      return(M_PI);
    }
  else
    {
      return(acos(dotproduct));
    }
}

//--------------------
// ３次元ベクトル演算
//--------------------

// ゼロセット
void
zeroV3(V3 out)
{
  const int dimension = 3;

  int d;
  for (d = 0; d < dimension; d++)
    {
      out[d] = 0.0;
    }
  return;
}


// コピー
// out == in でも動作する
void
copyV3(V3 in, V3 out)
{
  const int dimension = 3;
  int d;
  for (d = 0; d < dimension; d++)
    {
      out[d] = in[d];
    }
  return;
}

// 加算
void
addV3(V3 in1, V3 in2, V3 out)
{
  const int dimension = 3;

  int d;
  for (d = 0; d < dimension; d++)
    {
      out[d] = in1[d] + in2[d];
    }
  return;
}

// 減算
void
subV3(V3 in1, V3 in2, V3 out)
{
  const int dimension = 3;

  int d;
  for (d = 0; d < dimension; d++)
    {
      out[d] = in1[d] - in2[d];
    }
  return;
}

// スカラー積
// out == in でも動作する
void
mulV3S(const double s, V3 in, V3 out)
{
  const int dimension = 3;
  int d;
  for (d = 0; d < dimension; d++)
    {
      out[d] = in[d] * s;
    }
  return;
}

// 規格化
// out == in でも動作する
void
normalizeV3(V3 in, V3 out)
{
  double norm = getNormV3(in);
  if (isZero(norm))
    {
      copyV3(in, out);
    }
  else
    {
      mulV3S(1.0 / norm, in, out);
    }
  return;
}

// ノルム
double
getNormV3(V3 in)
{
  return sqrt(in[0] * in[0] + in[1] * in[1] + in[2] * in[2]);
}

// 内積
double
getInnerProductV3(V3 in1, V3 in2)
{
  return in1[0] * in2[0] + in1[1] * in2[1] + in1[2] * in2[2];
}

// クロス積
void
getCrossProductV3(V3 in1, V3 in2, V3 out)
{
  out[0] = in1[1] * in2[2] - in1[2] * in2[1];
  out[1] = in1[2] * in2[0] - in1[0] * in2[2];
  out[2] = in1[0] * in2[1] - in1[1] * in2[0];
  return;
}

// ２点間の距離
double
getDistanceV3(V3 in1, V3 in2)
{
  V3 sub;
  int i;

  for (i = 0; i < 3; i++)
    {
      sub[i] = in1[i] - in2[i];
    }

  return getNormV3(sub);
}

//----------------
// ３ｘ３配列演算
//----------------

// 減算
void
subM33(M33 in1, M33 in2, M33 out)
{
  const int size = 3;
  int col, row;
  for (row = 0; row < size; row++)
    {
      for (col = 0; col < size; col++)
        {
          out[row][col] = in1[row][col] - in2[row][col];
        }
    }
  return;
}


// 転置
void
transposeM33(M33 in, M33 out)
{
  int col, row;
  const int size = 3;
  for (col = 0; col < size; col++)
    {
      out[col][col] = in[col][col];
      for (row = col + 1; row < size; row++)
        {
          const double U = in[row][col];
          const double L = in[col][row];
          out[col][row] = U;
          out[row][col] = L;
        }
    }
  return;
}

// マトリクスxマトリクス積
void
mulM33(M33 in1, M33 in2, M33 out)
{
  const int size = 3;

  int row;
  for (row = 0; row < size; row++)
    {
      int col;
      for (col = 0; col < size; col++)
        {
          double sum = 0.0;
          int k;
          for (k = 0; k < size; k++)
            {
              sum += (in1[row][k]) * (in2[k][col]);
            }
          out[row][col] = sum;
        }
    }
  return;
}

// マトリクスxベクトル積
// out と in2 は同一であってはいけない
void
mulM33V3(M33 in1, V3 in2, V3 out)
{
  const int size = 3;

  int row;
  for (row = 0; row < size; row++)
    {
      double sum = 0.0;
      int k;
      for (k = 0; k < size; k++)
        {
          sum += (in1[row][k]) * (in2[k]);
        }
      out[row] = sum;
    }
  return;
}

// 逆行列
int
inverseM33(M33 in, M33 out)
{
  int col, row;

  // 各要素は、下位の２x２行列の判別式から求められる
  for (row = 0; row < 3; row++)
    {
      const int row1 = (row + 1) % 3;
      const int row2 = (row + 2) % 3;
      for (col = 0; col < 3; col++)
        {
          const int col1 = (col + 1) % 3;
          const int col2 = (col + 2) % 3;

          out[col][row] = (in[row1][col1] * in[row2][col2] - in[row1][col2] * in[row2][col1]);
        }
    }

  // 逆行列の判別値を求める
  double det = 0.0;
  for (col = 0; col < 3; col++)
    {
      det += in[0][col] * out[col][0];
    }

  if (isZero(det))
    {                           // 判別式がいきなりゼロに近ければ、ランク落ちで計算不能
      return -1;                // 計算できなかった
    }

  // 規格化する
  for (row = 0; row < 3; row++)
    {
      for (col = 0; col < 3; col++)
        {
          out[row][col] /= det;
        }
    }

  return 1;                     // 計算できた
}

// tail : 始点 , head : 終点 , data : データ格納領域, vec : 方向ベクトル
void
getDirectionVector(double tail[3], double head[3], double data[3], CvMat* vec)
{
  data[0] = head[0] - tail[0];
  data[1] = head[1] - tail[1];
  data[2] = head[2] - tail[2];
  *vec = cvMat(3, 1, CV_64FC1, data);
  cvNormalize(vec, vec);
  return;
}

// axis : 基準軸 , normal : 法線, dir : 法線に対する直交ベクトル
int
getOrthogonalDir(double axis[3], double normal[3], double dir[3])
{
  CvMat avec, nvec, dvec;
  double norm;
  // axis -> avec : 基準軸
  avec = cvMat(1, 3, CV_64FC1, axis);
  // normal -> nvec : 法線
  nvec = cvMat(1, 3, CV_64FC1, normal);
  // dir -> dvec : 法線に対する直交ベクトル
  dvec = cvMat(1, 3, CV_64FC1, dir);
  // avec と nvec の両方に直交するベクトルをもとめる
  cvCrossProduct(&avec, &nvec, &dvec);
  norm = cvNorm(&dvec);
  // 零ベクトルだったらエラー終了
  if (isZero(norm))
    {
      return -1;
    }
  // 単位方向ベクトルにする
  cvNormalize(&dvec, &dvec);
  return 0;
}

// axis : 基準軸 , normal : 法線, dir : 法線に対する直交ベクトル
int
getOrthogonalDir(CvMat* axis, CvMat* normal, CvMat* dir)
{
  double norm;
  // axis と normal に直交するベクトルを dir に返す
  cvCrossProduct(axis, normal, dir);
  norm = cvNorm(dir);
  // 外積ベクトルの大きさが零なら無効
  if (isZero(norm))
    {
      return -1;
    }
  cvNormalize(dir, dir);
  return 0;
}

// 任意軸周りの回転を表すクォータニオンを求める
void
quaternion_rotation(quaternion_t q, const double radian, double axis[3])
{
  double norm = 0.0;
  double cosine = 0.0, sine = 0.0;
  int i;

  quat_re(q) = quat_im(q, 0) = quat_im(q, 1) = quat_im(q, 2) = 0.0;

  norm = getNormV3(axis);
  if (norm <= 0)
    {
      return;
    }

  for (i = 0; i < 3; ++i)
    {
      axis[i] /= norm;
    }

  cosine = cos(radian / 2.0);
  sine = sin(radian / 2.0);

  quat_re(q) = cosine;
  for (i = 0; i < 3; ++i)
    {
      quat_im(q, i) = sine * axis[i];
    }
  return;
}
