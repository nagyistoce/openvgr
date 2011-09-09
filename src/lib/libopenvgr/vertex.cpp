/*
 vertex.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file vertex.cpp
 * @brief 3次元頂点特徴生成関連関数
 * @date \$Date::                            $
 */
#include "stereo.h"
#include "vectorutil.h"
#include "debugutil.h"

// 頂点データの有効無効フラグを比較する
static int
compareValidData(const void* cc1, const void* cc2)
{
  const StereoConic* c1 = (const StereoConic*) cc1;
  const StereoConic* c2 = (const StereoConic*) cc2;
  if (c2->work.vertex.valid == c1->work.vertex.valid)
    {
      return 0;
    }
  else if (c2->work.vertex.valid > c1->work.vertex.valid)
    {
      return 1;
    }
  else
    {
      return -1;
    }
}

// 基礎行列の計算
// x1^T E x2 = 0
// x2 = R x1 + t
static void
calcEssentialMatrix(M33 E, CameraParam* cp1, CameraParam* cp2)
{
  double Rt[3][3], t[3], tx[3][3];
  int i, j;

  /* Rt = R1 * R2^T */
  mulM33(cp1->Rotation, cp2->rRotation, Rt);

  /* t = R2 * (-R1^T * t1) + t2 */
  for (i = 0; i < 3; ++i)
    {
      t[i] = cp2->Translation[i];
      for (j = 0; j < 3; ++j)
        {
          t[i] += cp2->Rotation[i][j] * cp1->Position[j];
        }
    }

  /* tx * v = t x v */
  tx[0][0] = 0.0;
  tx[0][1] = -t[2];
  tx[0][2] =  t[1];

  tx[1][0] =  t[2];
  tx[1][1] = 0.0;
  tx[1][2] = -t[0];

  tx[2][0] = -t[1];
  tx[2][1] =  t[0];
  tx[2][2] = 0.0;

  /* E = R^T * tx */
  mulM33(Rt, tx, E);
}

// 注目端点の対応点を決定する
// 　注目端点の対応画像上でのエピポーラ線と
// 　対応頂点とその端点を結ぶ直線の交点を対応点とする
static void
correspondStereoPoints(double* thisPoint,
                       CameraParam* thisSideCameraParam,
                       CameraParam* otherSideCameraParam,
                       double* otherSidePoint1,
                       double* otherSidePoint2,
                       double* corrPoint,
                       Parameters parameters)
{
  Data_2D iPos, icPos, icPosS, icPosE;

  double E[3][3], el[3]; // エピポーラ線の係数(el[0] * x + el[1] * y + el[2] = 0)
  double pl[3], cp[3];
  int i;

  // 注目点の歪補正
  iPos.col = thisPoint[0];
  iPos.row = thisPoint[1];
  undistortPosition(&icPos, iPos, thisSideCameraParam);

  calcEssentialMatrix(E, otherSideCameraParam, thisSideCameraParam);
  // エピポーラ線を求める
  for (i = 0; i < 3; ++i)
    {
      el[i] = E[i][0] * icPos.col + E[i][1] * icPos.row + E[i][2];
    }
  normalizeV3(el, el);
  //fprintf(stderr, "% f % f % f\n", el[0], el[1], el[2]);

  // Point1->Point2ベクトルとエピポーラ線の交点の計算
  // Point1->Point2の方程式
  // 歪み補正後に計算する
  iPos.col = otherSidePoint1[0];
  iPos.row = otherSidePoint1[1];
  undistortPosition(&icPosS, iPos, otherSideCameraParam);
  iPos.col = otherSidePoint2[0];
  iPos.row = otherSidePoint2[1];
  undistortPosition(&icPosE, iPos, otherSideCameraParam);

  pl[0] =  icPosS.row - icPosE.row;
  pl[1] = -icPosS.col + icPosE.col;
  pl[2] = icPosS.col*icPosE.row - icPosS.row*icPosE.col;

  getCrossProductV3(el, pl, cp);

  if (fabs(cp[2]) >= getNormV3(cp) * VISION_EPS)
    {
      icPos.col = cp[0] / cp[2];
      icPos.row = cp[1] / cp[2];
    }
  else // 無限遠点
    {
      icPos.col = cp[0] / VISION_EPS;
      icPos.row = cp[1] / VISION_EPS;
    }
  distortPosition(&iPos, icPos, otherSideCameraParam);
  corrPoint[0] = iPos.col;
  corrPoint[1] = iPos.row;
  return;
}

// 注目点の対応点が、注目点によるエピポーラ線のどちら側かを判定する
static int
pointAnySideByEpi(double* thisPoint,
                  double* otherSidePoint,
                  CameraParam* thisSideCameraParam,
                  CameraParam* otherSideCameraParam,
                  Parameters parameters)
{
  double ret;
  Data_2D iPos, icPos;

  double E[3][3], el[3]; // エピポーラ線の係数(el[0] * x + el[1] * y + el[2] = 0)
  int i;

  // 注目点の歪補正
  iPos.col = thisPoint[0];
  iPos.row = thisPoint[1];
  undistortPosition(&icPos, iPos, thisSideCameraParam);

  calcEssentialMatrix(E, otherSideCameraParam, thisSideCameraParam);
  // エピポーラ線を求める
  for (i = 0; i < 3; ++i)
    {
      el[i] = E[i][0] * icPos.col + E[i][1] * icPos.row + E[i][2];
    }
  normalizeV3(el, el);

  // y軸方向が正になるようにする
  if (el[1] < 0.0)
    {
      for (i = 0; i < 3; ++i)
        {
          el[i] = -el[i];
        }
    }

  // 点のエピポーラ線との関係の判定
  iPos.col = otherSidePoint[0];
  iPos.row = otherSidePoint[1];
  undistortPosition(&icPos, iPos, otherSideCameraParam);

  ret = (el[0] * icPos.col + el[1] * icPos.row + el[2]) / sqrt(el[0]*el[0] + el[1]*el[1]);

  if (ret > VISION_EPS)
    {
      return 1;                 // 上
    }
  else if (ret < -VISION_EPS)
    {
      return 2;                 // 下
    }
  else
    {
      return 0;                 // 同
    }
  return -1;
}

// 二次元双曲線データから三次元頂点データを生成
void
HyperbolaToVertex(StereoPairing pairing,       // ステレオペアリング情報
                  CalibParam calib,            // キャリブレーションデータ
                  StereoData& stereo,          // ステレオ対応データ
                  Features2D* left,            // 左画像の２次元特徴データ
                  Features2D* right,           // 右画像の２次元特徴データ
                  unsigned char* edgeL,        // 左画像のエッジ画像
                  unsigned char* edgeR,        // 右画像のエッジ画像
                  Parameters parameters)       // 全パラメータ
{
  double ethr, depn, depf, amin, amax, lmin, lmax;
  int numOfHyperbolas;
  int numOfVertices;
  int i;

  CameraParam* camParamL;
  CameraParam* camParamR;

  StereoConic* conic;
  VertexCandidate* vertex;
  Feature2D* leftFeature;
  Feature2D* rightFeature;
  double Ls[2], Le[2], Lc[2], Rs[2], Re[2], Rc[2];      // 双曲線データ点の端点、中心点の2次元座標
  double dx, dy, dz, dx2, dy2, dz2, Llen, Rlen;
  double Ls2Rs[2], Le2Re[2], Ls2Re[2], Le2Rs[2];
  double Rs2Ls[2], Re2Le[2], Rs2Le[2], Re2Ls[2];
  double LsLen, LeLen, RsLen, ReLen;
  int LsByEPI, LeByEPI, RsByEPI, ReByEPI;       // 各端点がエピポーラ線のどちら側かの判定結果
  int LsLeFlag, RsReFlag;

  double xyz[3], error;
  double vangle;
  Data_2D posL, posR;

  // 使用するパラメータの設定
  ethr = parameters.stereo.ethr;
  depn = parameters.stereo.depn;
  depf = parameters.stereo.depf;
  amin = parameters.stereo.amin;
  amax = parameters.stereo.amax;
  lmin = parameters.stereo.lmin;
  lmax = parameters.stereo.lmax;

  numOfVertices = 0;
  numOfHyperbolas = 0;

  // ステレオペア指定に合わせて使用するキャリブレーションデータを設定
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

  // ステレオ対応データから双曲線を選別する
  for (i = 0; i < stereo.numOfconics; i++)
    {
      conic = &(stereo.conics[i]);
      // 双曲線でないときは次へ
      if (conic->valid != 1 || conic->type != ConicType_Hyperbola)
        {
          continue;
        }

      // 頂点情報
      vertex = &(stereo.conics[i].work.vertex);
      copyV3(conic->center, vertex->position);

      leftFeature = conic->featureL;
      rightFeature = conic->featureR;
      Ls[0] = leftFeature->startSPoint[0];
      Ls[1] = leftFeature->startSPoint[1];
      Le[0] = leftFeature->endSPoint[0];
      Le[1] = leftFeature->endSPoint[1];
      Lc[0] = leftFeature->center[0];
      Lc[1] = leftFeature->center[1];
      Rs[0] = rightFeature->startSPoint[0];
      Rs[1] = rightFeature->startSPoint[1];
      Re[0] = rightFeature->endSPoint[0];
      Re[1] = rightFeature->endSPoint[1];
      Rc[0] = rightFeature->center[0];
      Rc[1] = rightFeature->center[1];
      dx = Ls[0] - Lc[0];
      dy = Ls[1] - Lc[1];
      LsLen = sqrt(dx * dx + dy * dy);
      dx = Le[0] - Lc[0];
      dy = Le[1] - Lc[1];
      LeLen = sqrt(dx * dx + dy * dy);
      dx = Rs[0] - Rc[0];
      dy = Rs[1] - Rc[1];
      RsLen = sqrt(dx * dx + dy * dy);
      dx = Re[0] - Rc[0];
      dy = Re[1] - Rc[1];
      ReLen = sqrt(dx * dx + dy * dy);

      // Ls,Le,Rs,Reの対応関係を求める
      // Lcのエピポーラ線に対して、Rs, Reがどちら側にあるか判定する
      // 0:エピポーラ線上 1:エピポーラ線より上 2:エピポーラ線より下 -1:エラー
      RsByEPI = pointAnySideByEpi(Lc, Rs, camParamL, camParamR, parameters);
      ReByEPI = pointAnySideByEpi(Lc, Re, camParamL, camParamR, parameters);
      if (RsByEPI == -1 || ReByEPI == -1)
        {
          return;
        }
      if (RsByEPI == 0 || ReByEPI == 0)
        {                       // エピポーラ線に線分が平行なため求まらない
          vertex->valid = -3;
          continue;
        }
      LsByEPI = pointAnySideByEpi(Rc, Ls, camParamR, camParamL, parameters);
      LeByEPI = pointAnySideByEpi(Rc, Le, camParamR, camParamL, parameters);
      if (LsByEPI == -1 || LeByEPI == -1)
        {
          return;
        }
      if (LsByEPI == 0 || LeByEPI == 0)
        {                       // エピポーラ線に線分が平行なため求まらない
          vertex->valid = -3;
          continue;
        }
      if ((RsByEPI == ReByEPI && LsByEPI != LeByEPI)    // 左右の結果が異なる場合は違う特徴
          || (RsByEPI != ReByEPI && LsByEPI == LeByEPI))
        {
          vertex->valid = -3;
          continue;
        }
      // Ls,Le、Rs,Leがそれぞれ左右どちらにあるか外積で確認する
      LsLeFlag = ((Ls[0] * Le[1] - Ls[1] * Le[0]) > 0.0 ? 1 : 0);
      RsReFlag = ((Rs[0] * Re[1] - Rs[1] * Re[0]) > 0.0 ? 1 : 0);
      // 左右のペアを決めて復元する
      if ((RsByEPI != ReByEPI   // 端点が別の側にある
           && RsByEPI == LsByEPI)       // Ls,Rsが同じ側にある
          || (RsByEPI == ReByEPI        // 端点が同じ側にある
              && LsLeFlag == RsReFlag))
        {                       // LsLeの左右関係とRsReの左右関係が同じ
          // Ls,Rsで復元する
          if (LsLen > RsLen)
            {                   // 長い方を基準にする
              correspondStereoPoints(Ls, camParamL, camParamR, Rc, Rs, Ls2Rs, parameters);
              // Ls, Ls2Rsの組み合わせの3次元ベクトル
              posL.col = Ls[0];
              posL.row = Ls[1];
              posR.col = Ls2Rs[0];
              posR.row = Ls2Rs[1];
              error = calculateLR2XYZ(xyz, posL, posR, camParamL, camParamR);
            }
          else
            {
              correspondStereoPoints(Rs, camParamR, camParamL, Lc, Ls, Rs2Ls, parameters);
              // Rs2Ls, Rsの組み合わせの3次元ベクトル
              posL.col = Rs2Ls[0];
              posL.row = Rs2Ls[1];
              posR.col = Rs[0];
              posR.row = Rs[1];
              error = calculateLR2XYZ(xyz, posL, posR, camParamL, camParamR);
            }

          if (error > ethr)
            {
              vertex->valid = -3;
              continue;
            }
          copyV3(xyz, vertex->endpoint1);

          // Le,Reで復元する
          if (LeLen > ReLen)
            {                   // 長い方を基準にする
              correspondStereoPoints(Le, camParamL, camParamR,Rc, Re, Le2Re, parameters);
              // Le, Le2Reの組み合わせの3次元ベクトル
              posL.col = Le[0];
              posL.row = Le[1];
              posR.col = Le2Re[0];
              posR.row = Le2Re[1];
              error = calculateLR2XYZ(xyz, posL, posR, camParamL, camParamR);
            }
          else
            {
              correspondStereoPoints(Re, camParamR, camParamL,Lc, Le, Re2Le, parameters);
              // Re2Le, Reの組み合わせの3次元ベクトル
              posL.col = Re2Le[0];
              posL.row = Re2Le[1];
              posR.col = Re[0];
              posR.row = Re[1];
              error = calculateLR2XYZ(xyz, posL, posR, camParamL, camParamR);
            }
          if (error > ethr)
            {
              vertex->valid = -3;
              continue;
            }
          copyV3(xyz, vertex->endpoint2);
        }
      else
        {
          if ((RsByEPI != ReByEPI       // 端点が別の側にある
               && RsByEPI == LeByEPI)   // Ls,Reが同じ側にある
              || (RsByEPI == ReByEPI    // 端点が同じ側にある
                  && LsLeFlag != RsReFlag))
            {                   // LsLeの左右関係とRsReの左右関係が異なる
              // Ls,Reで復元する
              if (LsLen > ReLen)
                {               // 長い方を基準にする
                  correspondStereoPoints(Ls, camParamL, camParamR, Rc, Re, Ls2Re, parameters);
                  // Ls, Ls2Reの組み合わせの3次元ベクトル
                  posL.col = Ls[0];
                  posL.row = Ls[1];
                  posR.col = Ls2Re[0];
                  posR.row = Ls2Re[1];
                  error = calculateLR2XYZ(xyz, posL, posR, camParamL, camParamR);
                }
              else
                {
                  correspondStereoPoints(Re, camParamR, camParamL, Lc, Ls, Re2Ls, parameters);
                  // Re2Ls, Reの組み合わせの3次元ベクトル
                  posL.col = Re2Ls[0];
                  posL.row = Re2Ls[1];
                  posR.col = Re[0];
                  posR.row = Re[1];
                  error = calculateLR2XYZ(xyz, posL, posR, camParamL, camParamR);
                }

              if (error > ethr)
                {
                  vertex->valid = -3;
                  continue;
                }
              copyV3(xyz, vertex->endpoint1);
              // Le,Rsで復元する
              if (LeLen > RsLen)
                {               // 長い方を基準にする
                  correspondStereoPoints(Le, camParamL, camParamR, Rc, Rs, Le2Rs, parameters);
                  // Le, Le2Rsの組み合わせの3次元ベクトル
                  posL.col = Le[0];
                  posL.row = Le[1];
                  posR.col = Le2Rs[0];
                  posR.row = Le2Rs[1];
                  error = calculateLR2XYZ(xyz, posL, posR, camParamL, camParamR);
                }
              else
                {
                  correspondStereoPoints(Rs, camParamR, camParamL, Lc, Le, Rs2Le, parameters);
                  // Rs2Le, Rsの組み合わせの3次元ベクトル
                  posL.col = Rs2Le[0];
                  posL.row = Rs2Le[1];
                  posR.col = Rs[0];
                  posR.row = Rs[1];
                  error = calculateLR2XYZ(xyz, posL, posR, camParamL, camParamR);
                }
              if (error > ethr)
                {
                  vertex->valid = -3;
                  continue;
                }
              copyV3(xyz, vertex->endpoint2);
            }
        }

      // 頂点を構成する線分の成す角による絞り込み
      // ２直線のベクトルの成す角度の計算
      dx = vertex->endpoint1[0] - vertex->position[0];
      dy = vertex->endpoint1[1] - vertex->position[1];
      dz = vertex->endpoint1[2] - vertex->position[2];
      dx2 = vertex->endpoint2[0] - vertex->position[0];
      dy2 = vertex->endpoint2[1] - vertex->position[1];
      dz2 = vertex->endpoint2[2] - vertex->position[2];
      Llen = sqrt(dx * dx + dy * dy + dz * dz);
      Rlen = sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
      // 単位ベクトル、長さ情報を保存
      vertex->vector1[0] = dx / Llen;
      vertex->vector1[1] = dy / Llen;
      vertex->vector1[2] = dz / Llen;
      vertex->vector2[0] = dx2 / Rlen;
      vertex->vector2[1] = dy2 / Rlen;
      vertex->vector2[2] = dz2 / Rlen;
      vertex->len1 = Llen;
      vertex->len2 = Rlen;
      vangle = acos((dx * dx2 + dy * dy2 + dz * dz2) / (Llen * Rlen));
      // ラジアンを度に変換
      vangle = (vangle / M_PI) * 180.0;
      vertex->angle = vangle;
      // パラメータで指定された範囲外の角度になるものは除外する
      if (vangle >= amin && vangle <= amax)
        {
          // 線分長さが範囲外のものは除外する
          if (Llen >= lmin && Llen <= lmax && Rlen >= lmin && Rlen <= lmax)
            {
              vertex->valid = 1;
              continue;
            }
          else
            {
              vertex->valid = -1;
              continue;
            }
        }
      else
        {
          vertex->valid = -2;
          continue;
        }
    }

  // 使用可能なデータを配列の先頭に集める
  qsort(stereo.conics, stereo.numOfconics, sizeof(StereoConic), compareValidData);

  if ( parameters.dbgimag )
    {
      // 頂点の３次元復元結果画像（左画像）の表示・保存
      // ３次元復元結果変数：stereo
      drawStereoVertices( edgeL, stereo, pairing, parameters, camParamL );
    }

  if ( parameters.dbgtext )
    {
      // 頂点を構成する2ベクトルの保存
      // 出力結果 position, endpoint1, position, endpoint2 .....
      printStereoVertices( stereo, pairing );
    }

  return;
}
