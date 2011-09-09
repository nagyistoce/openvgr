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

// 画像平面上での楕円長軸方向算出
static void
calc_normal_on_image_plane(CameraParam* camera_param, const Feature2D* feature, double normal[3])
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
      undistortPosition(&plane[i], image[i], camera_param);
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
static double
CircleNormal(CameraParam* cameraParam, Feature2D* feature, double center[3],
             double normal[2][3])
{
  double cvec[3] = { 0.0, 0.0, 0.0 };
  double rvec[3], pvec[3], qvec[3];
  double cc[3], cn[2][3];
  double cost, sint;
  double f, d, radius;
  int i, j;

  // 楕円長軸と平行で画像平面上にあるベクトルを得る
#if 0  // cup in mixshapeAが通らないのでコメントアウト(r323)
  calc_normal_on_image_plane(cameraParam, feature, rvec);
#else
  rvec[0] = feature->ev[0][0];
  rvec[1] = feature->ev[0][1];
  rvec[2] = 0.0;                // ３次元化して扱う
#endif

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
          normal[0][i] += cameraParam->rRotation[i][j] * cn[0][j];
          normal[1][i] += cameraParam->rRotation[i][j] * cn[1][j];
        }
    }

  d = 0.0;
  for (i = 0; i < 3; i++)
    {
      cc[i] = center[i] - cameraParam->Position[i];
      d += cameraParam->rRotation[2][i] * cc[i];
    }
  d = fabs(d);
  f = cameraParam->fx;           // 焦点距離
  radius = d * feature->axis[0] / f;
  return radius;
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
  double step = 2.0 * M_PI / ndiv;
  double genP[3];
  quaternion_t q;
  int sts, d;
  int col, row;

  // 円の始点座標計算
  sts = getPointOnCircle(normal, radius, genP);
  // 始点計算ができないときは対応点なし
  if (sts)
    {
      return 0;
    }

  // 回転を表す単位クォータニオンの計算
  quaternion_rotation(q, -step, normal);
  for (d = 0; d < ndiv; d++)
    {
      // 円中心からの法線を回転軸にして円の点列座標を計算する
      quat_rot(genP, q, genP);
      addV3(genP, center, in_xyz3d);

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
