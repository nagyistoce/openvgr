/*
 findmatrix.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
#include <cxtypes.h>
#include <cxcore.h>

#include "findmatrix.h"
#include "quaternion.h"

// 回転を求める
static double findRotation(
    double rotation[3][3],
    CvMat* before_rotated,
    CvMat* after_rotated,
    const double *weight,
    const int nSample )
{
  double matN[4][4];
  double eigen_value[4];
  double eigen_vector[4][4];
  double weight_sum = 0.0;
  double maxE;
  CvMat eigenvalue, eigenvector;
  CvMat MatN;
  int i, j, k;

  MatN = cvMat( 4, 4, CV_64FC1, matN );
  cvSetZero( &MatN );

  for ( i = 0; i < nSample; i++ )
    {
      double w = weight ? weight[i] : 1.0;
      double s[3][3];

      weight_sum += w;

      for ( j = 0; j < 3; ++j )
        {
          for ( k = 0; k < 3; ++k )
            {
              s[j][k] = cvmGet(before_rotated, i, j) * cvmGet(after_rotated, i, k);
            }
        }

      matN[0][0] += w * (s[0][0] + s[1][1] + s[2][2]);
      matN[0][1] += w * (s[1][2] - s[2][1]);
      matN[0][2] += w * (s[2][0] - s[0][2]);
      matN[0][3] += w * (s[0][1] - s[1][0]);

      //matN[1][0] = matN[0][1];
      matN[1][1] += w * (s[0][0] - s[1][1] - s[2][2]);
      matN[1][2] += w * (s[0][1] + s[1][0]);
      matN[1][3] += w * (s[0][2] + s[2][0]);

      //matN[2][0] = matN[0][2];
      //matN[2][1] = matN[1][2];
      matN[2][2] += w * (-s[0][0] + s[1][1] - s[2][2]);
      matN[2][3] += w * (s[1][2] + s[2][1]);

      //matN[3][0] = matN[0][3];
      //matN[3][1] = matN[1][3];
      //matN[3][2] = matN[2][3];
      matN[3][3] += w * (-s[0][0] - s[1][1] + s[2][2]);
    }

  // 対称な要素をコピー
  for ( i = 1; i < 4; ++i )
    {
      for ( j = 0; j < i; ++j )
        {
          matN[i][j] = matN[j][i];
        }
    }

  eigenvalue  = cvMat( 4, 1, CV_64FC1, eigen_value );
  eigenvector = cvMat( 4, 4, CV_64FC1, eigen_vector );
  cvEigenVV( &MatN, &eigenvector, &eigenvalue );
  maxE = eigen_value[0];

  quaternion_t q;
  double R[3*3];

  // 要素を代入
  quat_re(q) = eigen_vector[0][0];
  for (i = 0; i < 3; ++i)
    {
      quat_im(q, i) = eigen_vector[0][i+1];
    }

  // 回転行列に変換して結果を代入
  quat_R_from_q(R, 3, q);
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          rotation[i][j] = R[i + 3*j];
        }
    }

  return maxE/weight_sum;
}

static void getDataCenter(
    CvMat* center,
    CvMat* before_centered,
    const double *weight,
    const int nSample )
{
  double sum[3] = {0.0, 0.0, 0.0};
  double wdt[3];
  double tWeight = 0.0;
  CvMat data;
  CvMat wdata;
  CvMat Sum;
  int i;

  if ( nSample <= 0 )
    {
      return;
    }

  Sum  = cvMat( 1, 3, CV_64FC1, sum );

  if ( weight == NULL )
    { // 重みなし
      for ( i = 0; i < nSample; i++ )
        {
          cvGetRow( before_centered, &data, i );
          cvAdd( &Sum, &data, &Sum );
          tWeight += 1.0;
        }
    }
  else
    { // 重みあり
      wdata = cvMat( 1, 3, CV_64FC1, wdt );
      for ( i = 0; i < nSample; i++ )
        {
          cvGetRow( before_centered, &data, i );
          cvScale( &data, &wdata, weight[i] );
          cvAdd( &Sum, &wdata, &Sum );
          tWeight += weight[i];
        }
    }

  if ( tWeight != 0.0 )
    {
      cvScale( &Sum, center, 1.0/tWeight );
    }
}

static void moveCenter(
    CvMat* centered,
    CvMat* center,
    CvMat* before_centered,
    const double *weight,
    const int nSample )
{
  CvMat src, dst;
  int i;

  if ( nSample <= 0 )
    {
      return;
    }

  getDataCenter( center, before_centered, weight, nSample );

  for ( i = 0; i < nSample; i++ )
    {
      cvGetRow( before_centered, &src, i );
      cvGetRow( centered, &dst, i );
      cvSub( &src, center, &dst );
    }
}

// 3次元空間中の点について変換前の点と変換後の点の対応付けから回転と移動を求める
void findTransformationMatrixForPairedPoints(
    double rotation[3][3], // 回転行列
    double translation[3], // 移動ベクトル
    CvMat* before_moved,   // 変換前の点座標配列
    CvMat* after_moved,    // 変換後の点座標配列
    const double *weight,  // 各データの重み配列
    int nSample            // サンプル数
  )
{
  CvMat* before_rotated;
  CvMat* after_rotated;
  CvMat before_gc;
  CvMat after_gc;
  CvMat after_rot;
  double bgc[3];
  double agc[3];
  double arot[3];
  CvMat Rmat;
  CvMat Tvec;

  // 移動後の回転を求めるための領域の確保
  before_rotated = cvCreateMat( nSample, 3, CV_64FC1 );
  after_rotated  = cvCreateMat( nSample, 3, CV_64FC1 );

  // 回転前と回転後の重心をそれぞれ求めて移動する
  before_gc = cvMat( 1, 3, CV_64FC1, bgc );
  moveCenter( before_rotated, &before_gc, before_moved, weight, nSample );
  after_gc  = cvMat( 1, 3, CV_64FC1, agc );
  moveCenter( after_rotated,  &after_gc,  after_moved,  weight, nSample );

  // 重心が一致する標本間での回転を求める
  findRotation( rotation, before_rotated, after_rotated, weight, nSample );

  // 重心の差は回転前の重心を回転させて, 回転後の重心から引いたものになる
  Rmat = cvMat( 3, 3, CV_64FC1, rotation );
  before_gc = cvMat( 3, 1, CV_64FC1, bgc );
  after_rot = cvMat( 3, 1, CV_64FC1, arot );
  cvMatMul( &Rmat, &before_gc, &after_rot );
  after_gc  = cvMat( 3, 1, CV_64FC1, agc );
  Tvec = cvMat( 3, 1, CV_64FC1, translation );
  cvSub( &after_gc, &after_rot, &Tvec );

  cvReleaseMat( &before_rotated );
  cvReleaseMat( &after_rotated );
  return;
}
