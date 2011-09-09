/*
 circle.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file circle.cpp
 * @brief 3次元円特徴生成関連関数
 * @date \$Date::                            $
 */
#include "stereo.h"
#include "vectorutil.h"
#include "debugutil.h"
#include "modelpoints.h"

#include "correspondence.hpp"
#include "extractFeature.hpp"

#ifdef CIRCLE_DEBUG
static void
printMat(const char* message, const cv::Mat& M)
{
  fprintf(stderr, "%s\n", message);

  for (int row = 0; row < M.rows; ++row)
    {
      for (int col = 0; col < M.cols; ++col)
        {
          fprintf(stderr, "%f ", M.at<double>(row, col));
        }
      fprintf(stderr, "\n");
    }

  return;
}
#endif

// 画像平面上での楕円長軸方向算出
static void
calc_normal_on_image_plane(CameraParam* camera_param, const Feature2D_old* feature, double normal[3])
{
  Data_2D image[2], plane[2];
  double v[2], norm = 0.0;
  int i;

  image[0].col = -feature->axis[0] * feature->ev[0][0] + feature->center[0];
  image[0].row = -feature->axis[0] * feature->ev[0][1] + feature->center[1];

  image[1].col =  feature->axis[0] * feature->ev[0][0] + feature->center[0];
  image[1].row =  feature->axis[0] * feature->ev[0][1] + feature->center[1];

  for (i = 0; i < 2; ++i)
    {
      backprojectPoint(&plane[i], image[i], camera_param);
    }

  v[0] = plane[1].col - plane[0].col;
  v[1] = plane[1].row - plane[0].row;
  norm = sqrt(v[0]*v[0] + v[1]*v[1]);

  for (i = 0; i < 2; ++i)
    {
      normal[i] = v[i] / norm;
    }
  normal[2] = 0.0;
}

// 二次元楕円の形状から三次元空間中の真円の半径と法線の推定値を計算する
#if 0
static double
CircleNormal(CameraParam* cameraParam, Feature2D_old* feature, double center[3],
             double normal[2][3])
{
  double cvec[3] = { 0.0, 0.0, 0.0 };
  double rvec[3], pvec[3], qvec[3];
  double cc[3], cn[2][3];
  double cost, sint;
  double f, d, radius;
  int i, j;

  // 楕円長軸と平行で画像平面上にあるベクトルを得る
  calc_normal_on_image_plane(cameraParam, feature, rvec);

  // 円中心座標をカメラ座標系に変換
  mulM33V3(cameraParam->Rotation, center, cvec);
  addV3(cvec, cameraParam->Translation, cvec);

  // 楕円長軸ベクトルと円中心ベクトルに直交するベクトル qvec を求める
  getCrossProductV3(rvec, cvec, qvec);
  // qvec を単位ベクトル化する
  normalizeV3(qvec, qvec);
  // rvec と qvec に直交する pvec を求める
  getCrossProductV3(rvec, qvec, pvec);

  // 円面の傾きθに対する cosθと sinθを求める
  cost = feature->axis[1] / feature->axis[0];
  sint = sin(acos(cost));

  // カメラ座標系での法線ベクトルの算出 ２つの解がある
  for (i = 0; i < 3; i++)
    {
      cn[0][i] = cost * pvec[i] + sint * qvec[i];
      cn[1][i] = cost * pvec[i] - sint * qvec[i];
    }

  //fprintf(stderr, "cn:\n% f % f\n% f % f\n", cn[0][0], cn[0][1], cn[1][0], cn[1][1]);

  // 法線をワールド座標系に変換
  for (i = 0; i < 3; i++)
    {
      normal[0][i] = normal[1][i] = 0;
      for (j = 0; j < 3; j++)
        {
          normal[0][i] += cameraParam->Rotation[j][i] * cn[0][j];
          normal[1][i] += cameraParam->Rotation[j][i] * cn[1][j];
        }
    }

  d = 0.0;
  for (i = 0; i < 3; i++)
    {
      cc[i] = center[i] - cameraParam->Position[i];
      d += cameraParam->Rotation[i][2] * cc[i];
    }
  d = fabs(d);
  f = cameraParam->intrinsicMatrix[0][0];           // 焦点距離
  radius = d * feature->axis[0] / f;
  return radius;
}
#else
// Reference:
// Kenichi Katanani and Wu Liu, "3D Interpretation of Conics and Orthogonality", 
// CVGIP: Image Understanding, Vol.58, No.3, pp.286-301, 1993
static double
CircleNormal(CameraParam* cameraParam, Feature2D_old* feature, double center[3],
             double normal[2][3])
{
  cv::Mat Q(3, 3, CV_64FC1), eigenvalues(3, 1, CV_64FC1), eigenvectors(3, 3, CV_64FC1);
  cv::Mat n1(1, 3, CV_64FC1), n2(1, 3, CV_64FC1);
  double e[3];
  double det;
  double lambda1, lambda2;
  double newcoef[6];
  const double t_x = -cameraParam->intrinsicMatrix[0][2], t_y = -cameraParam->intrinsicMatrix[1][2];
  const double f = cameraParam->intrinsicMatrix[0][0];
  double val, d, cc[3], radius;
  int i, j;

  // 原点を光学中心に移動
  newcoef[0] = feature->coef[0];
  newcoef[1] = feature->coef[1];
  newcoef[2] = feature->coef[2];
  newcoef[3] = -2.0 * feature->coef[0] * t_x - feature->coef[1] * t_y + feature->coef[3];
  newcoef[4] = -feature->coef[1] * t_x - 2.0 * feature->coef[2] * t_y + feature->coef[4];
  newcoef[5] = feature->coef[0] * t_x * t_x + feature->coef[1] * t_x * t_y +
    feature->coef[2] * t_y * t_y - feature->coef[3] * t_x - feature->coef[4] * t_y +
    feature->coef[5];

  Q.at<double>(0, 0) = newcoef[0];
  Q.at<double>(0, 1) = newcoef[1] / 2;
  Q.at<double>(1, 0) = Q.at<double>(0, 1);
  Q.at<double>(1, 1) = newcoef[2];
  Q.at<double>(2, 0) = newcoef[3] / 2 / f;
  Q.at<double>(0, 2) = Q.at<double>(2, 0);
  Q.at<double>(1, 2) = newcoef[4] / 2 / f;
  Q.at<double>(2, 1) = Q.at<double>(1, 2);
  Q.at<double>(2, 2) = newcoef[5] / f / f;

  // 行列式が -1 になるように正規化
  det = cv::determinant(Q);
  val = -cbrt(det);
  Q /= val;

  cv::eigen(Q, eigenvalues, eigenvectors);
  e[0] = eigenvalues.at<double>(0, 0);
  e[1] = eigenvalues.at<double>(1, 0);
  e[2] = eigenvalues.at<double>(2, 0);

  // 法線ベクトルの算出
  lambda1 = sqrt((e[0] - e[1]) / (e[0] - e[2]));
  lambda2 = sqrt((e[1] - e[2]) / (e[0] - e[2]));
  n1 = lambda1 * eigenvectors.row(0) + lambda2 * eigenvectors.row(2);
  n2 = lambda1 * eigenvectors.row(0) - lambda2 * eigenvectors.row(2);

  // 単位ベクトルへの正規化
  n1 = n1 / sqrt(n1.dot(n1));
  n2 = n2 / sqrt(n2.dot(n2));

  // 法線をワールド系に変換
  for (i = 0; i < 3; i++)
    {
      normal[0][i] = normal[1][i] = 0;
      for (j = 0; j < 3; j++)
        {
          normal[0][i] += cameraParam->Rotation[j][i] * n1.at<double>(0, j);
          normal[1][i] += cameraParam->Rotation[j][i] * n2.at<double>(0, j);
        }
    }

  // 支持平面までの距離
  d = 0.0;
  for (i = 0; i < 3; i++)
    {
      cc[i] = center[i] - cameraParam->Position[i];
      d += normal[0][i] * cc[i];
    }
  d = fabs(d);

  // 半径
  radius = d / sqrt(e[1] * e[1] * e[1]);

  return radius;
}
#endif

//! 画像上の楕円の長軸・短軸に対応する3次元単位ベクトルの算出
void
calc_3d_axes_of_circle(double major_axis[3],   //!< 長軸
                       double minor_axis[3],   //!< 短軸
                       const double normal[3], //!< 3次元円の法線
                       const CameraParam *cp)  //!< カメラパラメータ
{
  double norm = 0.0;

  // 長軸の方向をカメラの奥行き方向と法線方向の外積で決める
  if (cp != NULL)
    {
      getCrossProductV3(const_cast<double*>(cp->Rotation[2]), const_cast<double*>(normal), major_axis);
      norm = getNormV3(major_axis);
      if (norm >= VISION_EPS)
        {
          mulV3S(1.0/norm, major_axis, major_axis);
          getCrossProductV3(const_cast<double*>(normal), major_axis, minor_axis);
        }
      else
        {
          copyV3(const_cast<double*>(cp->Rotation[0]), major_axis);
          copyV3(const_cast<double*>(cp->Rotation[1]), minor_axis);
        }
    }
  else // cp == NULL の時はカメラの姿勢＝単位行列とする
    {
      major_axis[0] = -normal[1];
      major_axis[1] =  normal[0];
      major_axis[2] = 0.0;

      norm = getNormV3(major_axis);
      if (norm >= VISION_EPS)
        {
          mulV3S(1.0/norm, major_axis, major_axis);
          getCrossProductV3(const_cast<double*>(normal), major_axis, minor_axis);
        }
      else
        {
          major_axis[0] = 1.0;
          major_axis[1] = 0.0;
          major_axis[2] = 0.0;

          minor_axis[0] = 0.0;
          minor_axis[1] = normal[2] > 0.0 ? 1.0 : -1.0;
          minor_axis[2] = 0.0;
        }
    }
}

// 真円を２次元エッジ画像に投影して評価する
static int
evalCircleOnEdge(unsigned char* edge, CameraParam* cameraParam,
		 double radius, double normal[3], double center[3],
		 Parameters parameters)
{
  double in_xyz3d[3];           // 円上の点（開始は３次元円周上の一点）
  int iCountOnEdge = 0;
  int rowsize = parameters.rowsize;
  int colsize = parameters.colsize;
  Data_2D iPos;
  int ndiv = 360; // 分割数
  int d;
  int col, row;

  double axis[2][3];

  calc_3d_axes_of_circle(axis[0], axis[1], normal, NULL);
  for (d = 0; d < ndiv; ++d)
    {
      int i;
      double theta = (double)d / (double)(ndiv - 1) * M_PI * 2.0;
      for (i = 0; i < 3; ++i)
        {
          in_xyz3d[i] = radius * (axis[0][i] * cos(theta) + axis[1][i] * sin(theta)) + center[i];
        }

      projectXYZ2LR(&iPos, in_xyz3d, cameraParam);

      col = (int) floor(iPos.col + 0.5);
      row = (int) floor(iPos.row + 0.5);
      if (col >= 0 && col < colsize && row >= 0 && row < rowsize)
        {
          if (edge[row * colsize + col] > 1)
            {                       // 楕円に使用されたエッジは２
              iCountOnEdge++;
            }
        }
    }

  return iCountOnEdge;
}

// 二次元楕円のステレオ対応から三次元空間中の真円を推定・復元する
void
EllipseToCircle(StereoPairing pairing,  // ステレオペア情報
                CalibParam calib,       // キャリブレーションデータ
                StereoData& stereo,     // ステレオ対応情報（復元真円情報を含む）
                unsigned char* edgeL,   // 評価用エッジ画像（左）
                unsigned char* edgeR,   // 評価用エッジ画像（右）
                Parameters parameters)  // 全パラメータ
{
  double ethr, rthr, nthr;
  double nL[2][3], rL;
  double nR[2][3], rR;
  double wcenter[3] = { 0 };
  double wnormal[3] = { 0 };
  double radius;
  double ip[4], max;
  double diff;
  int i, j, k, n;
  int maxL, maxR;
  int iCountOnEdge, iMaxCountOnEdge, iMaxN;
  StereoConic* conic;
  CircleCandidate* circle;

  CameraParam* camParamL;
  CameraParam* camParamR;

  int count = 0;

  ethr = parameters.stereo.ethr;
  rthr = parameters.stereo.rdif;
  nthr = parameters.stereo.ndif;

  switch (pairing)
    {
    case DBL_LR:
      camParamL = &calib.CameraL;
      camParamR = &calib.CameraR;
      break;

    case DBL_LV:
      camParamL = &calib.CameraL;
      camParamR = &calib.CameraV;
      break;

    case DBL_RV:
      camParamL = &calib.CameraR;
      camParamR = &calib.CameraV;
      break;

    default:
      camParamL = &calib.CameraL;
      camParamR = &calib.CameraR;
      break;
    }

  for (k = 0; k < stereo.numOfconics; k++)
    {
      conic = &stereo.conics[k];
      // 楕円でないときは次へ
      if ((conic->valid != 1) || (conic->type != ConicType_Ellipse))
        {
          continue;
        }

      // 円情報
      circle = &(stereo.conics[k].work.circle);

      // 円中心座標をカメラ系からワールド系に変換
      copyV3(conic->center, wcenter);
      // 円の半径と法線を求める．法線は２つずつ得られる
      rL = CircleNormal(camParamL, conic->featureL, wcenter, nL);
      rR = CircleNormal(camParamR, conic->featureR, wcenter, nR);

      // 半径差が大きいときは誤対応
      if (fabs(rL - rR) > rthr)
        {
          circle->valid = -3;
          continue;
        }

      // 左右半径の平均を仮の半径とする
      radius = (rL + rR) / 2.0;

      // 法線ベクトルの向きを比較するために左右２個ずつのベクトルの
      // 組み合わせで内積をとり，最大になるものを見つける
      max = -DBL_MAX;
      maxL = maxR = 0;
      for (i = 0, n = 0; i < 2; i++)
        {
          for (j = 0; j < 2; j++, n++)
            {
              ip[n] = getInnerProductV3(nL[i], nR[j]);
              if (ip[n] > max)
                {
                  max = ip[n];
                  maxL = i;
                  maxR = j;
                }
            }
        }

      // 法線の向きが異なるときは誤対応
      if (max < 0)
        {
          circle->valid = -2;
          continue;
        }

      // １に近いものは同じ方向とする
      diff = acos(max) * 180 / M_PI;
      if (diff > nthr)
        {
          circle->valid = -1;
          continue;
        }

      // ここまでくれば正しい対応とみなす

      // 左右法線の平均をとる
      for (n = 0; n < 3; n++)
        {
          wnormal[n] = (nL[maxL][n] + nR[maxR][n]) / 2.0;
        }

      // 単位ベクトル化
      normalizeV3(wnormal, wnormal);

      // 3つの法線（Ｌ，Ｒ，平均）からもっとも良さそうな法線を選択する
      // 3次元円を2次元エッジ画像に投影して、円上のエッジ点をカウントし、もっとも多いのを選択
      iMaxN = 0;
      normalizeV3(nL[maxL], nL[maxL]);
      normalizeV3(nR[maxR], nR[maxR]);
      iMaxCountOnEdge =
        evalCircleOnEdge(edgeL, camParamL, radius, wnormal, wcenter, parameters)
        + evalCircleOnEdge(edgeR, camParamR, radius, wnormal, wcenter, parameters);
      iCountOnEdge =
        evalCircleOnEdge(edgeL, camParamL, rL, nL[maxL], wcenter, parameters)
        + evalCircleOnEdge(edgeR, camParamR, rL, nL[maxL], wcenter, parameters);
      if (iCountOnEdge > iMaxCountOnEdge)
        {
          iMaxN = 1;
          iMaxCountOnEdge = iCountOnEdge;
        }
      iCountOnEdge =
        evalCircleOnEdge(edgeL, camParamL, rR, nR[maxR], wcenter, parameters)
        + evalCircleOnEdge(edgeR, camParamR, rR, nR[maxR], wcenter, parameters);
      if (iCountOnEdge > iMaxCountOnEdge)
        {
          iMaxN = 2;
          iMaxCountOnEdge = iCountOnEdge;
        }
      // 半径・法線・中心を保存する
      // ワールド系の出力の場合はこのまま．カメラ系が必要なら変換すること
      copyV3(wcenter, circle->center);
      switch (iMaxN)
        {
        case 1:
          circle->radius = rL;
          copyV3(nL[maxL], circle->normal);
          break;

        case 2:
          circle->radius = rR;
          copyV3(nR[maxR], circle->normal);
          break;

        case 0:
          /* fall through */
        default:
          circle->radius = radius;
          copyV3(wnormal, circle->normal);
          break;
        }
      // 円データの有効フラグをたてる
      circle->valid = 1;
      count++;
    }

  if ( parameters.dbgimag )
    {
      // 円中心の３次元復元結果画像（左画像）の表示・保存
      drawStereoCircles( edgeL, stereo, pairing, parameters, camParamL );
    }

  if ( parameters.dbgtext )
    {
      // 円の保存
      printStereoCircles( stereo, pairing );
    }

  return;
}

// conic のパラメータを対称行列にセットする
static void
set_conic_parameter(const ovgr::EllipseFeature* f,
                    cv::Mat* C)
{
  const double *coef = const_cast<double *>(f->coef);

  C->at<double>(0, 0) = coef[0];
  C->at<double>(0, 1) = coef[1] / 2;
  C->at<double>(1, 0) = C->at<double>(0, 1);
  C->at<double>(1, 1) = coef[2];
  C->at<double>(2, 0) = coef[3] / 2;
  C->at<double>(0, 2) = C->at<double>(2, 0);
  C->at<double>(1, 2) = coef[4] / 2;
  C->at<double>(2, 1) = C->at<double>(1, 2);
  C->at<double>(2, 2) = coef[5];

#ifdef CIRCLE_DEBUG
  printMat("C", *C);
#endif

  return;
}

// 特性方程式の係数の計算 I[0] L^2 + I[1] L + I[2] = 0
static void
calc_coeffs_delta(double I[3],
                  const cv::Mat& Q1,
                  const cv::Mat& Q2)
{
  double a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34, a41, a42, a43, a44;
  double b11, b12, b13, b14, b21, b22, b23, b24, b31, b32, b33, b34, b41, b42, b43, b44;
  a11 = Q1.at<double>(0, 0);
  a12 = Q1.at<double>(0, 1);
  a13 = Q1.at<double>(0, 2);
  a14 = Q1.at<double>(0, 3);
  a21 = Q1.at<double>(1, 0);
  a22 = Q1.at<double>(1, 1);
  a23 = Q1.at<double>(1, 2);
  a24 = Q1.at<double>(1, 3);
  a31 = Q1.at<double>(2, 0);
  a32 = Q1.at<double>(2, 1);
  a33 = Q1.at<double>(2, 2);
  a34 = Q1.at<double>(2, 3);
  a41 = Q1.at<double>(3, 0);
  a42 = Q1.at<double>(3, 1);
  a43 = Q1.at<double>(3, 2);
  a44 = Q1.at<double>(3, 3);

  b11 = Q2.at<double>(0, 0);
  b12 = Q2.at<double>(0, 1);
  b13 = Q2.at<double>(0, 2);
  b14 = Q2.at<double>(0, 3);
  b21 = Q2.at<double>(1, 0);
  b22 = Q2.at<double>(1, 1);
  b23 = Q2.at<double>(1, 2);
  b24 = Q2.at<double>(1, 3);
  b31 = Q2.at<double>(2, 0);
  b32 = Q2.at<double>(2, 1);
  b33 = Q2.at<double>(2, 2);
  b34 = Q2.at<double>(2, 3);
  b41 = Q2.at<double>(3, 0);
  b42 = Q2.at<double>(3, 1);
  b43 = Q2.at<double>(3, 2);
  b44 = Q2.at<double>(3, 3);

  I[0] =
    a11 * b22 * b33 * b44 - a12 * b21 * b33 * b44 - a21 * b12 * b33 * b44 +
    a22 * b11 * b33 * b44 - a11 * b23 * b32 * b44 + a13 * b21 * b32 * b44 +
    a21 * b13 * b32 * b44 - a23 * b11 * b32 * b44 + a12 * b23 * b31 * b44 -
    a13 * b22 * b31 * b44 - a22 * b13 * b31 * b44 + a23 * b12 * b31 * b44 +
    a31 * b12 * b23 * b44 - a32 * b11 * b23 * b44 - a31 * b13 * b22 * b44 +
    a33 * b11 * b22 * b44 + a32 * b13 * b21 * b44 - a33 * b12 * b21 * b44 -
    a11 * b22 * b34 * b43 + a12 * b21 * b34 * b43 + a21 * b12 * b34 * b43 -
    a22 * b11 * b34 * b43 + a11 * b24 * b32 * b43 - a14 * b21 * b32 * b43 -
    a21 * b14 * b32 * b43 + a24 * b11 * b32 * b43 - a12 * b24 * b31 * b43 +
    a14 * b22 * b31 * b43 + a22 * b14 * b31 * b43 - a24 * b12 * b31 * b43 -
    a31 * b12 * b24 * b43 + a32 * b11 * b24 * b43 + a31 * b14 * b22 * b43 -
    a34 * b11 * b22 * b43 - a32 * b14 * b21 * b43 + a34 * b12 * b21 * b43 +
    a11 * b23 * b34 * b42 - a13 * b21 * b34 * b42 - a21 * b13 * b34 * b42 +
    a23 * b11 * b34 * b42 - a11 * b24 * b33 * b42 + a14 * b21 * b33 * b42 +
    a21 * b14 * b33 * b42 - a24 * b11 * b33 * b42 + a13 * b24 * b31 * b42 -
    a14 * b23 * b31 * b42 - a23 * b14 * b31 * b42 + a24 * b13 * b31 * b42 +
    a31 * b13 * b24 * b42 - a33 * b11 * b24 * b42 - a31 * b14 * b23 * b42 +
    a34 * b11 * b23 * b42 + a33 * b14 * b21 * b42 - a34 * b13 * b21 * b42 -
    a12 * b23 * b34 * b41 + a13 * b22 * b34 * b41 + a22 * b13 * b34 * b41 -
    a23 * b12 * b34 * b41 + a12 * b24 * b33 * b41 - a14 * b22 * b33 * b41 -
    a22 * b14 * b33 * b41 + a24 * b12 * b33 * b41 - a13 * b24 * b32 * b41 +
    a14 * b23 * b32 * b41 + a23 * b14 * b32 * b41 - a24 * b13 * b32 * b41 -
    a32 * b13 * b24 * b41 + a33 * b12 * b24 * b41 + a32 * b14 * b23 * b41 -
    a34 * b12 * b23 * b41 - a33 * b14 * b22 * b41 + a34 * b13 * b22 * b41 -
    a41 * b12 * b23 * b34 + a42 * b11 * b23 * b34 + a41 * b13 * b22 * b34 -
    a43 * b11 * b22 * b34 - a42 * b13 * b21 * b34 + a43 * b12 * b21 * b34 +
    a41 * b12 * b24 * b33 - a42 * b11 * b24 * b33 - a41 * b14 * b22 * b33 +
    a44 * b11 * b22 * b33 + a42 * b14 * b21 * b33 - a44 * b12 * b21 * b33 -
    a41 * b13 * b24 * b32 + a43 * b11 * b24 * b32 + a41 * b14 * b23 * b32 -
    a44 * b11 * b23 * b32 - a43 * b14 * b21 * b32 + a44 * b13 * b21 * b32 +
    a42 * b13 * b24 * b31 - a43 * b12 * b24 * b31 - a42 * b14 * b23 * b31 +
    a44 * b12 * b23 * b31 + a43 * b14 * b22 * b31 - a44 * b13 * b22 * b31;

  I[1] =
    a11 * a22 * b33 * b44 - a12 * a21 * b33 * b44 - a11 * a23 * b32 * b44 +
    a13 * a21 * b32 * b44 + a12 * a23 * b31 * b44 - a13 * a22 * b31 * b44 -
    a11 * a32 * b23 * b44 + a12 * a31 * b23 * b44 + a11 * a33 * b22 * b44 -
    a13 * a31 * b22 * b44 - a12 * a33 * b21 * b44 + a13 * a32 * b21 * b44 +
    a21 * a32 * b13 * b44 - a22 * a31 * b13 * b44 - a21 * a33 * b12 * b44 +
    a23 * a31 * b12 * b44 + a22 * a33 * b11 * b44 - a23 * a32 * b11 * b44 -
    a11 * a22 * b34 * b43 + a12 * a21 * b34 * b43 + a11 * a24 * b32 * b43 -
    a14 * a21 * b32 * b43 - a12 * a24 * b31 * b43 + a14 * a22 * b31 * b43 +
    a11 * a32 * b24 * b43 - a12 * a31 * b24 * b43 - a11 * a34 * b22 * b43 +
    a14 * a31 * b22 * b43 + a12 * a34 * b21 * b43 - a14 * a32 * b21 * b43 -
    a21 * a32 * b14 * b43 + a22 * a31 * b14 * b43 + a21 * a34 * b12 * b43 -
    a24 * a31 * b12 * b43 - a22 * a34 * b11 * b43 + a24 * a32 * b11 * b43 +
    a11 * a23 * b34 * b42 - a13 * a21 * b34 * b42 - a11 * a24 * b33 * b42 +
    a14 * a21 * b33 * b42 + a13 * a24 * b31 * b42 - a14 * a23 * b31 * b42 -
    a11 * a33 * b24 * b42 + a13 * a31 * b24 * b42 + a11 * a34 * b23 * b42 -
    a14 * a31 * b23 * b42 - a13 * a34 * b21 * b42 + a14 * a33 * b21 * b42 +
    a21 * a33 * b14 * b42 - a23 * a31 * b14 * b42 - a21 * a34 * b13 * b42 +
    a24 * a31 * b13 * b42 + a23 * a34 * b11 * b42 - a24 * a33 * b11 * b42 -
    a12 * a23 * b34 * b41 + a13 * a22 * b34 * b41 + a12 * a24 * b33 * b41 -
    a14 * a22 * b33 * b41 - a13 * a24 * b32 * b41 + a14 * a23 * b32 * b41 +
    a12 * a33 * b24 * b41 - a13 * a32 * b24 * b41 - a12 * a34 * b23 * b41 +
    a14 * a32 * b23 * b41 + a13 * a34 * b22 * b41 - a14 * a33 * b22 * b41 -
    a22 * a33 * b14 * b41 + a23 * a32 * b14 * b41 + a22 * a34 * b13 * b41 -
    a24 * a32 * b13 * b41 - a23 * a34 * b12 * b41 + a24 * a33 * b12 * b41 +
    a11 * a42 * b23 * b34 - a12 * a41 * b23 * b34 - a11 * a43 * b22 * b34 +
    a13 * a41 * b22 * b34 + a12 * a43 * b21 * b34 - a13 * a42 * b21 * b34 -
    a21 * a42 * b13 * b34 + a22 * a41 * b13 * b34 + a21 * a43 * b12 * b34 -
    a23 * a41 * b12 * b34 - a22 * a43 * b11 * b34 + a23 * a42 * b11 * b34 -
    a11 * a42 * b24 * b33 + a12 * a41 * b24 * b33 + a11 * a44 * b22 * b33 -
    a14 * a41 * b22 * b33 - a12 * a44 * b21 * b33 + a14 * a42 * b21 * b33 +
    a21 * a42 * b14 * b33 - a22 * a41 * b14 * b33 - a21 * a44 * b12 * b33 +
    a24 * a41 * b12 * b33 + a22 * a44 * b11 * b33 - a24 * a42 * b11 * b33 +
    a11 * a43 * b24 * b32 - a13 * a41 * b24 * b32 - a11 * a44 * b23 * b32 +
    a14 * a41 * b23 * b32 + a13 * a44 * b21 * b32 - a14 * a43 * b21 * b32 -
    a21 * a43 * b14 * b32 + a23 * a41 * b14 * b32 + a21 * a44 * b13 * b32 -
    a24 * a41 * b13 * b32 - a23 * a44 * b11 * b32 + a24 * a43 * b11 * b32 -
    a12 * a43 * b24 * b31 + a13 * a42 * b24 * b31 + a12 * a44 * b23 * b31 -
    a14 * a42 * b23 * b31 - a13 * a44 * b22 * b31 + a14 * a43 * b22 * b31 +
    a22 * a43 * b14 * b31 - a23 * a42 * b14 * b31 - a22 * a44 * b13 * b31 +
    a24 * a42 * b13 * b31 + a23 * a44 * b12 * b31 - a24 * a43 * b12 * b31 +
    a31 * a42 * b13 * b24 - a32 * a41 * b13 * b24 - a31 * a43 * b12 * b24 +
    a33 * a41 * b12 * b24 + a32 * a43 * b11 * b24 - a33 * a42 * b11 * b24 -
    a31 * a42 * b14 * b23 + a32 * a41 * b14 * b23 + a31 * a44 * b12 * b23 -
    a34 * a41 * b12 * b23 - a32 * a44 * b11 * b23 + a34 * a42 * b11 * b23 +
    a31 * a43 * b14 * b22 - a33 * a41 * b14 * b22 - a31 * a44 * b13 * b22 +
    a34 * a41 * b13 * b22 + a33 * a44 * b11 * b22 - a34 * a43 * b11 * b22 -
    a32 * a43 * b14 * b21 + a33 * a42 * b14 * b21 + a32 * a44 * b13 * b21 -
    a34 * a42 * b13 * b21 - a33 * a44 * b12 * b21 + a34 * a43 * b12 * b21;

  I[2] =
    a11 * a22 * a33 * b44 - a12 * a21 * a33 * b44 - a11 * a23 * a32 * b44 +
    a13 * a21 * a32 * b44 + a12 * a23 * a31 * b44 - a13 * a22 * a31 * b44 -
    a11 * a22 * a34 * b43 + a12 * a21 * a34 * b43 + a11 * a24 * a32 * b43 -
    a14 * a21 * a32 * b43 - a12 * a24 * a31 * b43 + a14 * a22 * a31 * b43 +
    a11 * a23 * a34 * b42 - a13 * a21 * a34 * b42 - a11 * a24 * a33 * b42 +
    a14 * a21 * a33 * b42 + a13 * a24 * a31 * b42 - a14 * a23 * a31 * b42 -
    a12 * a23 * a34 * b41 + a13 * a22 * a34 * b41 + a12 * a24 * a33 * b41 -
    a14 * a22 * a33 * b41 - a13 * a24 * a32 * b41 + a14 * a23 * a32 * b41 -
    a11 * a22 * a43 * b34 + a12 * a21 * a43 * b34 + a11 * a23 * a42 * b34 -
    a13 * a21 * a42 * b34 - a12 * a23 * a41 * b34 + a13 * a22 * a41 * b34 +
    a11 * a22 * a44 * b33 - a12 * a21 * a44 * b33 - a11 * a24 * a42 * b33 +
    a14 * a21 * a42 * b33 + a12 * a24 * a41 * b33 - a14 * a22 * a41 * b33 -
    a11 * a23 * a44 * b32 + a13 * a21 * a44 * b32 + a11 * a24 * a43 * b32 -
    a14 * a21 * a43 * b32 - a13 * a24 * a41 * b32 + a14 * a23 * a41 * b32 +
    a12 * a23 * a44 * b31 - a13 * a22 * a44 * b31 - a12 * a24 * a43 * b31 +
    a14 * a22 * a43 * b31 + a13 * a24 * a42 * b31 - a14 * a23 * a42 * b31 +
    a11 * a32 * a43 * b24 - a12 * a31 * a43 * b24 - a11 * a33 * a42 * b24 +
    a13 * a31 * a42 * b24 + a12 * a33 * a41 * b24 - a13 * a32 * a41 * b24 -
    a11 * a32 * a44 * b23 + a12 * a31 * a44 * b23 + a11 * a34 * a42 * b23 -
    a14 * a31 * a42 * b23 - a12 * a34 * a41 * b23 + a14 * a32 * a41 * b23 +
    a11 * a33 * a44 * b22 - a13 * a31 * a44 * b22 - a11 * a34 * a43 * b22 +
    a14 * a31 * a43 * b22 + a13 * a34 * a41 * b22 - a14 * a33 * a41 * b22 -
    a12 * a33 * a44 * b21 + a13 * a32 * a44 * b21 + a12 * a34 * a43 * b21 -
    a14 * a32 * a43 * b21 - a13 * a34 * a42 * b21 + a14 * a33 * a42 * b21 -
    a21 * a32 * a43 * b14 + a22 * a31 * a43 * b14 + a21 * a33 * a42 * b14 -
    a23 * a31 * a42 * b14 - a22 * a33 * a41 * b14 + a23 * a32 * a41 * b14 +
    a21 * a32 * a44 * b13 - a22 * a31 * a44 * b13 - a21 * a34 * a42 * b13 +
    a24 * a31 * a42 * b13 + a22 * a34 * a41 * b13 - a24 * a32 * a41 * b13 -
    a21 * a33 * a44 * b12 + a23 * a31 * a44 * b12 + a21 * a34 * a43 * b12 -
    a24 * a31 * a43 * b12 - a23 * a34 * a41 * b12 + a24 * a33 * a41 * b12 +
    a22 * a33 * a44 * b11 - a23 * a32 * a44 * b11 - a22 * a34 * a43 * b11 +
    a24 * a32 * a43 * b11 + a23 * a34 * a42 * b11 - a24 * a33 * a42 * b11;

  return;
}

// 平面の方程式の抽出
static cv::Mat
extract_plane(const cv::Mat& eigenvector1, 
              const cv::Mat& eigenvector2, 
              const double mu[2], 
              const CameraParam* camParam1,
              const CameraParam* camParam2)
{
  cv::Mat planePairs(4, 2, CV_64FC1);
  cv::Mat cameraPos(4, 2, CV_64FC1);
  int plane_index = -1; // どちらの平面か

  // 平面の計算
  for (size_t i = 0; i < 4; ++i)
    {
      double val1, val2;
      val1 = mu[0] * eigenvector1.at<double>(i, 0);
      val2 = mu[1] * eigenvector2.at<double>(i, 0);

      planePairs.at<double>(i, 0) = val1 + val2;
      planePairs.at<double>(i, 1) = val1 - val2;
    }

  // カメラ位置の取得
  for (size_t i = 0; i < 3; ++i)
    {
      cameraPos.at<double>(i, 0) = camParam1->Position[i];
      cameraPos.at<double>(i, 1) = camParam2->Position[i];
    }
  cameraPos.at<double>(3, 0) = 1.0;
  cameraPos.at<double>(3, 1) = 1.0;

#ifdef CIRCLE_DEBUG
  printMat("pos1, 2", cameraPos);
  printMat("plane", planePairs);
#endif

  // どちらの平面か、内積で判断する
  if (planePairs.col(0).dot(cameraPos.col(0)) * 
      planePairs.col(0).dot(cameraPos.col(1)) > 0)
    {
      plane_index = 0;
    }
  else
    {
      plane_index = 1;
    }

#ifdef CIRCLE_DEBUG
  for (size_t i = 0; i < 2; i++)
    {
      fprintf(stderr, "%f\n", planePairs.col(i).dot(cameraPos.col(0)) * 
              planePairs.col(i).dot(cameraPos.col(1)));
    }
#endif

  return planePairs.col(plane_index);
}

// z = 0 をplaneへ変換する行列の計算
static cv::Mat
calc_z_to_plane_transform(const cv::Mat& plane)
{
  cv::Mat R = cv::Mat::zeros(4, 4, CV_64FC1);
  cv::Mat paxis(3, 1, CV_64FC1);
  cv::Mat zaxis(3, 1, CV_64FC1);
  
  // 平面の法線
  paxis.at<double>(0, 0) = plane.at<double>(0, 0);
  paxis.at<double>(1, 0) = plane.at<double>(1, 0);
  paxis.at<double>(2, 0) = plane.at<double>(2, 0);
  // 正規化
  double paxis_norm = sqrt(paxis.dot(paxis));
  paxis /= paxis_norm;

  // z 軸
  zaxis.at<double>(0, 0) = 0.0;
  zaxis.at<double>(1, 0) = 0.0;
  zaxis.at<double>(2, 0) = 1.0;
  // 外積をつかって回転軸の計算
  // z軸を面の法線へ
  cv::Mat raxis(3, 1, CV_64FC1);
  raxis = zaxis.cross(paxis);

  // 平面の法線とz軸のなす角度
  double rad = acos(paxis.at<double>(2, 0));

  // (0,0,1)を面の法線へ回転する行列の計算
  quaternion_t q;
  double cR[9];
  double r_axis[3];
  r_axis[0] = raxis.at<double>(0, 0);
  r_axis[1] = raxis.at<double>(1, 0);
  r_axis[2] = raxis.at<double>(2, 0);

  quaternion_rotation(q, rad, r_axis);
  quat_R_from_q(cR, 3, q);
  // 原点から平面までの距離
  double plane_dist = plane.at<double>(3, 0) / paxis_norm;
  
  // 行列の生成
  for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
        {
          R.at<double>(row, col) = cR[col * 3 + row];
        }
      R.at<double>(row, 3) = -plane_dist * paxis.at<double>(row, 0);
    }
  R.at<double>(3, 3) = 1.0;

  return R;
}

// Feature2D_old 特徴量への変換
static void
convert_to_Feature2D(const cv::Mat Q, 
                     ::Feature2D_old* f)
{
  double new_coef[6];

  new_coef[0] = Q.at<double>(0, 0);
  new_coef[1] = Q.at<double>(0, 1) * 2;
  new_coef[2] = Q.at<double>(1, 1);
  new_coef[3] = Q.at<double>(0, 3) * 2;
  new_coef[4] = Q.at<double>(1, 3) * 2;
  new_coef[5] = Q.at<double>(3, 3);
  // 楕円にしないと getConicProperty が動作しない
  f->type = ConicType_Ellipse;

  getConicProperty(new_coef, &f->type, f->center, f->ev, &f->axis[0], &f->axis[1]);

  return;
}

// CircleCandidate 特徴量への変換
static void
set_to_CircleCandidate(const ::Feature2D_old& f,
                       const cv::Mat& plane,
                       const cv::Mat& R, // z = 0 を平面へ変換する行列
                       ::CircleCandidate* circle)
{
  cv::Mat f_center(4, 1, CV_64FC1);
  f_center.at<double>(0, 0) = f.center[0];
  f_center.at<double>(1, 0) = f.center[1];
  f_center.at<double>(2, 0) = 0.0;
  f_center.at<double>(3, 0) = 1.0;
  cv::Mat center = R * f_center;

  double norm = plane.at<double>(0, 0) * plane.at<double>(0, 0) +
    plane.at<double>(1, 0) * plane.at<double>(1, 0) + 
    plane.at<double>(2, 0) * plane.at<double>(2, 0);
  norm = sqrt(norm);

  // 法線
  circle->normal[0] = plane.at<double>(0, 0) / norm;
  circle->normal[1] = plane.at<double>(1, 0) / norm;
  circle->normal[2] = plane.at<double>(2, 0) / norm;
  // 半径、長軸と短軸の平均
  circle->radius = (f.axis[0] + f.axis[1]) / 2;
  // 中心
  circle->center[0] = center.at<double>(0, 0);
  circle->center[1] = center.at<double>(1, 0);
  circle->center[2] = center.at<double>(2, 0);

  return;
}

// 特性方程式の係数の計算 mu^2 + c[0] mu + c[1] = 0
// Faddeev-Leverrier Method を利用
static void
calc_coeffs_theta(double coeffs[2], 
                  const cv::Mat& Q)
{
  cv::Mat B1, B2;
  cv::Mat I = cv::Mat::eye(Q.rows, Q.cols, CV_64FC1);
  double p[2];
  cv::Scalar tr;

  B1 = Q.clone();
  tr = cv::trace(B1);
  p[0] = tr[0];

  B2 = Q * (B1 - p[0] * I);
  tr = cv::trace(B2);
  p[1] = tr[0] / 2;

  // 符号の反転
  coeffs[0] = -p[0];
  coeffs[1] = -p[1];

  return;
}

// ある固有値に対応する固有ベクトルの計算
static void
calc_eigenvector(const double mu, 
                 const cv::Mat& Q, 
                 cv::Mat* eigenvec)
{
  cv::Mat M = cv::Mat::zeros(Q.rows, Q.cols, CV_64FC1);
  cv::Mat I = cv::Mat::eye(Q.rows, Q.cols, CV_64FC1);

  M = Q - mu * I;
  cv::SVD::solveZ(M, *eigenvec);

  return;
}

// 3次元空間楕円復元
// Reference:
// Long Quan, "Conic Reconstruction and Correspondence From Two Views",
// IEEE Trans. on PAMI, Vol.18, No.2, pp.151-160, 1996
static int
reconstruct_ellipse(const ovgr::EllipseFeature* f1, 
                    const ovgr::EllipseFeature* f2, 
                    const cv::Mat& P1, 
                    const cv::Mat& P2,
                    const CameraParam* camParam1,
                    const CameraParam* camParam2,
                    ::CircleCandidate* ccandidate,
                    const Parameters& parameters)
{
  cv::Mat C1(3, 3, CV_64FC1), C2(3, 3, CV_64FC1); // 画像上での楕円
  cv::Mat Q1(4, 4, CV_64FC1), Q2(4, 4, CV_64FC1); // cone
  double lambda;
  cv::Mat Q(4, 4, CV_64FC1); // Q1 + lambda Q2
  double I[3];
  const double rthr = parameters.stereo.rdif;

  // conic のパラメータを行列にセット
  set_conic_parameter(f1, &C1);
  set_conic_parameter(f2, &C2);
  // cone
  Q1 = P1.t() * C1 * P1;
  Q2 = P2.t() * C2 * P2;

  // 特性方程式の係数の計算 I[0] L^2 + I[1] L + I[2] = 0
  calc_coeffs_delta(I, Q1, Q2);
#ifdef CIRCLE_DEBUG
  fprintf(stderr, "%e %f\n", I[1] * I[1] - 4 * I[0] * I[2], I[1] * I[1] / (I[0] * I[2]));
#endif
  // 係数比が大きい場合は誤対応
  if (I[1] * I[1] / (I[0] * I[2]) - 4.0 > 0.01)
    {
      return -1;
    }
  lambda = - 0.5 * I[1] / I[0];

  // Q = Q1 + lambda Q2
  Q = Q1 + lambda * Q2;

  // 固有値の計算
  double mu[2];
  double Imu[2];
  double det;
  // 特性方程式の係数の計算 mu^2 + Imu[0] mu + Imu[1] = 0
  calc_coeffs_theta(Imu, Q);
  det = Imu[0] * Imu[0] - 4 * Imu[1];
  if (det < 0)
    {
      return -1;
    }
  mu[0] = (- Imu[0] + sqrt(det)) / 2;
  mu[1] = (- Imu[0] - sqrt(det)) / 2;

  // 解が同符号の場合は不適
  if (mu[0] * mu[1] > 0)
    {
      return -1;
    }

  // 固有ベクトルの計算
  cv::Mat eigenvector1(4, 1, CV_64FC1);
  cv::Mat eigenvector2(4, 1, CV_64FC1);
  calc_eigenvector(mu[0], Q, &eigenvector1);
  calc_eigenvector(mu[1], Q, &eigenvector2);

  // mu[0] > 0, mu[1] < 0 
  mu[0] = sqrt(mu[0]);
  mu[1] = sqrt(-mu[1]);

  // cone が交差する平面の抽出
  cv::Mat plane = extract_plane(eigenvector1, eigenvector2, mu, camParam1, camParam2);

  // z = 0 を平面へ変換する行列の計算
  const cv::Mat R = calc_z_to_plane_transform(plane);
  // cone と平面の交線
  const cv::Mat Qz = R.t() * Q1 * R;

  // Feature2D_old 特徴量への変換
  ::Feature2D_old f;
  memset(&f, 0, sizeof(Feature2D_old));
  convert_to_Feature2D(Qz, &f);

#ifdef CIRCLE_DEBUG
  fprintf(stderr, "a %f b %f\n", f.axis[0], f.axis[1]);
  fprintf(stderr, "center %f %f\n", f.center[0], f.center[1]);
#endif

  // 長軸と短軸の長さが大きく異なる場合
  if (fabs(f.axis[0] - f.axis[1]) > rthr)
    {
      return -1;
    }

  // CircleCandidate 特徴量への変換
  set_to_CircleCandidate(f, plane, R, ccandidate);

  return 0;
}


// P = A [R | T] の計算
static cv::Mat
calc_projection_matrix(const CameraParam* camParam)
{
  const cv::Mat cvA(3, 3, CV_64FC1, const_cast<double(*)[3]>(camParam->intrinsicMatrix));
  const cv::Mat cvR(3, 3, CV_64FC1, const_cast<double(*)[3]>(camParam->Rotation));
  const cv::Mat cvT(3, 1, CV_64FC1, const_cast<double*>(camParam->Translation));
  cv::Mat P(3, 4, CV_64FC1);
  cv::Mat RT(3, 4, CV_64FC1);
  
  for (size_t r = 0; r < 3; ++r)
    {
      for(size_t c = 0; c < 3; ++c)
        {
          RT.at<double>(r, c) = cvR.at<double>(r, c);
        }
      RT.at<double>(r, 3) = cvT.at<double>(r, 0);
    }

  P = cvA * RT;
#if 0
  // P[2,3] = 1.0 となるようにスケーリング
  double scale = P.at<double>(2, 3);
  if (scale != 0.0)
    {
      P /= scale;
    }
#endif
  return P;
}

// 二次元楕円のステレオ対応から三次元空間中の真円を推定・復元する
void
reconstruct_ellipse2D_to_circle3D(std::vector<const ovgr::Features2D*>& feature,
                                  const ovgr::CorrespondingSet& cs,
                                  const CameraParam* camParam[3],
                                  const unsigned char* edge[3],
                                  Features3D* scene,
                                  const Parameters& parameters)  // 全パラメータ
{
  double ethr, depn, depf, rthr, nthr;
  double nL[2][3], rL;
  double nR[2][3], rR;
  double wcenter[3] = { 0 };
  double wnormal[3] = { 0 };
  double radius;
  double ip[4], max;
  double diff;
  int maxL, maxR;
  int iCountOnEdge, iMaxCountOnEdge, iMaxN;

  int count = 0;

  ethr = parameters.stereo.ethr;
  depn = parameters.stereo.depn >= 0.0 ? parameters.stereo.depn : 0.0;
  depf = parameters.stereo.depf >= 0.0 ? parameters.stereo.depf : -1.0;
  rthr = parameters.stereo.rdif;
  nthr = parameters.stereo.ndif;

  std::vector<Features2D_old*> old_Features2D(feature.size());
  std::vector<CircleCandidate> ccandidates; // 3次元円復元結果格納用
  std::vector<size_t> nv(feature.size()); // 頂点数

  // 旧特徴への変換、頂点数の計算
  for (size_t i = 0; i < feature.size(); ++i)
    {
      old_Features2D[i] = ovgr::create_old_features_from_new_one(*feature[i]);      
      nv[i] = feature[i]->vertex.size();
    }

  std::vector<cv::Mat> P; // 射影行列
  for (size_t i = 0; i < feature.size(); ++i)
    {
      const cv::Mat cvP = calc_projection_matrix(camParam[i]);
      P.push_back(cvP);
    }
#ifdef CIRCLE_DEBUG
  for (size_t i = 0; i < feature.size(); ++i)
    {
      printMat("P", P[i]);
    }
#endif

  for (ovgr::feature_list_t::const_iterator it = cs.ellipse.begin(); it != cs.ellipse.end(); ++it)
    {
      std::vector<Feature2D_old*> old_f(it->size());
      std::vector<const ovgr::EllipseFeature*> new_f(it->size());
      std::vector<int> c_index(it->size()); // カメラのインデックス
      size_t n = 0;
      
      for (size_t c = 0; c < it->size(); ++c)
        {
          // データが存在していたら
          if ((*it)[c] != -1)
            {
              c_index[n] = c;
              new_f[n] = &(feature[c]->ellipse[(*it)[c]]);
              // 旧特徴は、頂点、円と並んでいる
              old_f[n] = &(old_Features2D[c]->feature[nv[c] + (*it)[c]]);
              n++;
            }
        }

      CircleCandidate ccandidate = {0}; // 円候補
#if 0
      // 楕円の対応による3次元復元
      int ret;
      ret = reconstruct_ellipse(new_f[0], new_f[1], P[c_index[0]], P[c_index[1]], 
                                camParam[c_index[0]], camParam[c_index[1]], &ccandidate, parameters);
      if (ret < 0)
        {
          continue;
        }
#ifdef CIRCLE_DEBUG
      fprintf(stderr, "New Normal, radius, center\n");
      fprintf(stderr, "%f %f %f, %f, %f %f %f\n", 
              ccandidate.normal[0], ccandidate.normal[1], ccandidate.normal[2],
              ccandidate.radius,
              ccandidate.center[0], ccandidate.center[1], ccandidate.center[2]);
#endif
#else
      // 中心の復元
      Data_2D posL, posR;
#if 0
      posL.col = old_f[0]->center[0];
      posL.row = old_f[0]->center[1];
      posR.col = old_f[1]->center[0];
      posR.row = old_f[1]->center[1];
#else
      posL.col = new_f[0]->center[0];
      posL.row = new_f[0]->center[1];
      posR.col = new_f[1]->center[0];
      posR.row = new_f[1]->center[1];
#endif
      double error = calculateLR2XYZ(wcenter, posL, posR, 
                                     const_cast<CameraParam*>(camParam[c_index[0]]), 
                                     const_cast<CameraParam*>(camParam[c_index[1]]));
      // ここでの判定は将来的には必要ないはず
      // テスト段階では前候補がここにくるので判定が必要
      if(error > ethr)
        {
          continue;
        }

      // 一つ目のカメラと復元した頂点の距離を求める
      double dep = camParam[c_index[0]]->Translation[2];
      for (int i = 0; i < 3; ++i)
        {
          dep += camParam[c_index[0]]->Rotation[2][i] * wcenter[i];
        }
      //fprintf(stderr, "dep: %f [%f, %f]\n", dep, depn, depf);

      // dep が[depn, depf]の範囲外であれば不採用
      if (dep < depn || (depf > 0.0 && depf < dep))
        {
          continue;
        }

      // 円の半径と法線を求める．法線は２つずつ得られる
      rL = CircleNormal(const_cast<CameraParam*>(camParam[c_index[0]]), old_f[0], wcenter, nL);
      rR = CircleNormal(const_cast<CameraParam*>(camParam[c_index[1]]), old_f[1], wcenter, nR);

      // 半径差が大きいときは誤対応
      if (fabs(rL - rR) > rthr)
        {
          continue;
        }

      // 左右半径の平均を仮の半径とする
      radius = (rL + rR) / 2.0;

      // 法線ベクトルの向きを比較するために左右２個ずつのベクトルの
      // 組み合わせで内積をとり，最大になるものを見つける
      max = -DBL_MAX;
      maxL = maxR = 0;
      for (int j = 0, n = 0; j < 2; j++)
        {
          for (int k = 0; k < 2; k++, n++)
            {
              ip[n] = getInnerProductV3(nL[j], nR[k]);
              if (ip[n] > max)
                {
                  max = ip[n];
                  maxL = j;
                  maxR = k;
                }
            }
        }

      // 法線の向きが異なるときは誤対応
      if (max < 0)
        {
          continue;
        }

      // １に近いものは同じ方向とする
      diff = acos(max) * 180 / M_PI;
      if (diff > nthr)
        {
          continue;
        }

      // ここまでくれば正しい対応とみなす
      // 左右法線の平均をとる
      for (n = 0; n < 3; n++)
        {
          wnormal[n] = (nL[maxL][n] + nR[maxR][n]) / 2.0;
        }

      // 単位ベクトル化
      normalizeV3(wnormal, wnormal);
      
      // 3つの法線（Ｌ，Ｒ，平均）からもっとも良さそうな法線を選択する
      // 3次元円を2次元エッジ画像に投影して、円上のエッジ点をカウントし、もっとも多いのを選択
      iMaxN = 0;
      normalizeV3(nL[maxL], nL[maxL]);
      normalizeV3(nR[maxR], nR[maxR]);
      iMaxCountOnEdge =
        evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[0]]), 
                         const_cast<CameraParam*>(camParam[c_index[0]]), 
                         radius, wnormal, wcenter, parameters)
        + evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[1]]), 
                           const_cast<CameraParam*>(camParam[c_index[1]]), 
                           radius, wnormal, wcenter, parameters);
      iCountOnEdge =
        evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[0]]), 
                         const_cast<CameraParam*>(camParam[c_index[0]]), 
                         rL, nL[maxL], wcenter, parameters)
        + evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[1]]), 
                           const_cast<CameraParam*>(camParam[c_index[1]]), 
                           rL, nL[maxL], wcenter, parameters);
      if (iCountOnEdge > iMaxCountOnEdge)
        {
          iMaxN = 1;
          iMaxCountOnEdge = iCountOnEdge;
        }
      iCountOnEdge =
        evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[0]]), 
                         const_cast<CameraParam*>(camParam[c_index[0]]), 
                         rR, nR[maxR], wcenter, parameters)
        + evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[1]]), 
                           const_cast<CameraParam*>(camParam[c_index[1]]), 
                           rR, nR[maxR], wcenter, parameters);
      if (iCountOnEdge > iMaxCountOnEdge)
        {
          iMaxN = 2;
          iMaxCountOnEdge = iCountOnEdge;
        }
      // 半径・法線・中心を保存する
      // ワールド系の出力の場合はこのまま．カメラ系が必要なら変換すること
      copyV3(wcenter, ccandidate.center);
      switch (iMaxN)
        {
        case 1:
          ccandidate.radius = rL;
          copyV3(nL[maxL], ccandidate.normal);
          break;

        case 2:
          ccandidate.radius = rR;
          copyV3(nR[maxR], ccandidate.normal);
          break;

        case 0:
          /* fall through */
        default:
          ccandidate.radius = radius;
          copyV3(wnormal, ccandidate.normal);
          break;
        }
      count++;

#ifdef CIRCLE_DEBUG
      fprintf(stderr, "Old Normal, radius, center\n");
      fprintf(stderr, "%f %f %f, %f, %f %f %f\n", 
              ccandidate.normal[0], ccandidate.normal[1], ccandidate.normal[2],
              ccandidate.radius,
              ccandidate.center[0], ccandidate.center[1], ccandidate.center[2]);
#endif
#endif
      // 復元結果として追加
      ccandidates.push_back(ccandidate);
    }

  // 旧3次元特徴に代入
  set_circle_to_OldFeature3D(ccandidates, scene);

  fprintf(stderr, "%d\n", ccandidates.size());

  if ( parameters.dbgimag )
    {
      for (size_t i = 0; i < feature.size(); ++i)
        {
          // 円の３次元復元結果画像の表示・保存
          drawCircleCandidate(edge[i], ccandidates, i, parameters, camParam[i]);
        }
    }

  // メモリの開放
  for (size_t i = 0; i < feature.size(); ++i)
    {
      destructFeatures(old_Features2D[i]);
    }

  return;
}
