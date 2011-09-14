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
#include <iostream>
#include "stereo.h"
#include "vectorutil.h"
#include "debugutil.h"
#include "vertex.h"
#include "rtvcm.h"

#include "local.h"
#include "geometry.hpp"

// 画像内かどうかの判定
inline int
isValidPixelPosition(double col, double row, const Parameters& parameters)
{
  if (col < 0 || col > parameters.colsize || row < 0 || row > parameters.rowsize)
    {
      return 0;
    }
  else
    {
      return 1;
    }
}

// 頂点データの有効無効フラグを比較する
static int
compareValidData(const void* cc1, const void* cc2)
{
  const StereoConic* c1 = (const StereoConic*) cc1;
  const StereoConic* c2 = (const StereoConic*) cc2;

  if (c1->type != ConicType_Hyperbola ||
      c2->type != ConicType_Hyperbola)
    {
      return -1;
    }

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
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          int k;

          Rt[i][j] = 0.0;
          for (k = 0; k < 3; ++k)
            {
              Rt[i][j] += cp1->Rotation[i][k] * cp2->Rotation[j][k];
            }
        }
    }
              
  /* t = R2 * (-R1^T * t1) + t2 */
  for (i = 0; i < 3; ++i)
    {
      t[i] = cp2->Translation[i];
      for (j = 0; j < 3; ++j)
        {
          t[i] -= Rt[j][i] * cp1->Translation[j];
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
  backprojectPoint(&icPos, iPos, thisSideCameraParam);

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
  backprojectPoint(&icPosS, iPos, otherSideCameraParam);

  iPos.col = otherSidePoint2[0];
  iPos.row = otherSidePoint2[1];
  backprojectPoint(&icPosE, iPos, otherSideCameraParam);

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
  projectPoint(&iPos, icPos, otherSideCameraParam);

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
  backprojectPoint(&icPos, iPos, thisSideCameraParam);

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
  backprojectPoint(&icPos, iPos, otherSideCameraParam);

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
}

// 二次元双曲線データから三次元頂点データを生成
void
HyperbolaToVertex(StereoPairing pairing,       // ステレオペアリング情報
                  CalibParam calib,            // キャリブレーションデータ
                  StereoData& stereo,          // ステレオ対応データ
                  Features2D_old* left,        // 左画像の２次元特徴データ
                  Features2D_old* right,       // 右画像の２次元特徴データ
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
  Feature2D_old* leftFeature;
  Feature2D_old* rightFeature;
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
      // 0:エピポーラ線上 1:エピポーラ線より上 2:エピポーラ線より下
      RsByEPI = pointAnySideByEpi(Lc, Rs, camParamL, camParamR, parameters);
      ReByEPI = pointAnySideByEpi(Lc, Re, camParamL, camParamR, parameters);
      if (RsByEPI == 0 || ReByEPI == 0)
        {                       // エピポーラ線に線分が平行なため求まらない
          vertex->valid = -3;
          continue;
        }
      LsByEPI = pointAnySideByEpi(Rc, Ls, camParamR, camParamL, parameters);
      LeByEPI = pointAnySideByEpi(Rc, Le, camParamR, camParamL, parameters);
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
              if (isValidPixelPosition(Ls2Rs[0], Ls2Rs[1], parameters) == 0)
                {
                  vertex->valid = -3;
                  continue;
                }
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
              if (isValidPixelPosition(Rs2Ls[0], Rs2Ls[1], parameters) == 0)
                {
                  vertex->valid = -3;
                  continue;
                }
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
              if (isValidPixelPosition(Le2Re[0], Le2Re[1], parameters) == 0)
                {
                  vertex->valid = -3;
                  continue;
                }
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
              if (isValidPixelPosition(Re2Le[0], Re2Le[1], parameters) == 0)
                {
                  vertex->valid = -3;
                  continue;
                }
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
                  if (isValidPixelPosition(Ls2Re[0], Ls2Re[1], parameters) == 0)
                    {
                      vertex->valid = -3;
                      continue;
                    }
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
                  if (isValidPixelPosition(Re2Ls[0], Re2Ls[1], parameters) == 0)
                    {
                      vertex->valid = -3;
                      continue;
                    }
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
                  if (isValidPixelPosition(Le2Rs[0], Le2Rs[1], parameters) == 0)
                    {
                      vertex->valid = -3;
                      continue;
                    }
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
                  if (isValidPixelPosition(Rs2Le[0], Rs2Le[1], parameters) == 0)
                    {
                      vertex->valid = -3;
                      continue;
                    }
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

static int
reconstruct_vertex3D(::Vertex vertex[2],
                     const ovgr::VertexFeature& vf1, const ovgr::VertexFeature& vf2,
                     const cv::Mat& F, const bool inv,
                     const CameraParam& cp1, const CameraParam& cp2)
{
  int num = 0;

  const ovgr::Point2D* pos[2][2] = {{&vf1.start, &vf1.end}, {&vf2.start, &vf2.end}};
  const ovgr::Point2D* mid[2] = {&vf1.mid, &vf2.mid};

  Data_2D pos_temp[2] = {{vf1.mid.x, vf1.mid.y},
                         {vf2.mid.x, vf2.mid.y}};
  double vpos[3];
  double error = 0.0;

  // 頂点の3次元位置を復元
  error = calculateLR2XYZ(vpos, pos_temp[0], pos_temp[1],
                  const_cast<CameraParam*>(&cp1), const_cast<CameraParam*>(&cp2));
  //fprintf(stderr, "error: % 10.3g\n", error);

  for (int pair = 0; pair < 2; ++pair)
    {
      Data_2D corres_pos[2][2];
      bool is_valid = true;

      // 対応点を求める
      for (int i = 0; i < 2; ++i)
        {
          const int pi = (pair + i) % 2;

          double epi_line[3];
          double cross_pos[2][3];
          
          // 第1カメラ画像上の点に対応する第2カメラ画像上の点
          calc_epipolar_line(epi_line, F, *pos[0][i], inv);
          s_cross3(cross_pos[1], epi_line, vf2.line_coef[pi]);
          s_normalize(cross_pos[1]);

          // 第2カメラ画像上の点に対応する第1カメラ画像上の点
          calc_epipolar_line(epi_line, F, *pos[1][pi], !inv);
          s_cross3(cross_pos[0], epi_line, vf1.line_coef[i]);
          s_normalize(cross_pos[0]);

          // 観測点と上で計算した点のうち、頂点に近い方(＝短い線分)を選ぶ
          bool use_projected[2] = {false, false};
          for (int j = 0; j < 2; ++j)
            {
              const int p = (j == 0) ? i : pi;

              const double dot = (pos[j][p]->x - mid[j]->x) * (cross_pos[j][0] / cross_pos[j][2] - mid[j]->x) + (pos[j][p]->y - mid[j]->y) * (cross_pos[j][1] / cross_pos[j][2] - mid[j]->y);
              const double norm2 = pow(pos[j][p]->x - mid[j]->x, 2) + pow(pos[j][p]->y - mid[j]->y, 2);
              const double ratio = dot / norm2;

              if (ratio < VISION_EPS || fabs(cross_pos[j][2]) < VISION_EPS)
                {
                  is_valid = false;
                  break;
                }

              if (ratio <= 1.0)
                {
                  corres_pos[j][i].col = cross_pos[j][0] / cross_pos[j][2];
                  corres_pos[j][i].row = cross_pos[j][1] / cross_pos[j][2];
                  use_projected[j] = true;
                }
              else
                {
                  corres_pos[j][i].col = pos[j][p]->x;
                  corres_pos[j][i].row = pos[j][p]->y;
                  use_projected[j] = false;
                }
            }
          if (use_projected[0] == use_projected[1])
            {
              is_valid = false;
            }

          if (!is_valid)
            {
              break;
            }
        }
      if (!is_valid)
        {
          continue;
        }

#if 0
      printf("% f % f % f % f\n", corres_pos[0][0].col, corres_pos[0][0].row,
             corres_pos[1][0].col, corres_pos[1][0].row);
      printf("% f % f % f % f\n", vf1.mid.x, vf1.mid.y, vf2.mid.x, vf2.mid.y);
      printf("% f % f % f % f\n", corres_pos[0][1].col, corres_pos[0][1].row,
             corres_pos[1][1].col, corres_pos[1][1].row);
      printf("\n");
#endif

      // 3次元端点位置
      error = calculateLR2XYZ(vertex[num].endpoint1, corres_pos[0][0], corres_pos[1][0],
                      const_cast<CameraParam*>(&cp1), const_cast<CameraParam*>(&cp2));
      //fprintf(stderr, "error: % 10.3g ", error);

      error = calculateLR2XYZ(vertex[num].endpoint2, corres_pos[0][1], corres_pos[1][1],
                      const_cast<CameraParam*>(&cp1), const_cast<CameraParam*>(&cp2));
      //fprintf(stderr, "% 10.3g\n", error);

      for (int i = 0; i < 3; ++i)
        {
          vertex[num].endpoint1[i] -= vpos[i]; // 頂点が原点
          vertex[num].endpoint2[i] -= vpos[i];
        }
#if 0
      printf("% f % f % f\n", vertex[num].endpoint1[0], vertex[num].endpoint1[1], vertex[num].endpoint1[2]);
      printf("% f % f % f\n", vertex[num].endpoint2[0], vertex[num].endpoint2[1], vertex[num].endpoint2[2]);
      printf("\n");
#endif

      double vangle;
      CvMat vec1, vec2;
      CvMat normal, bisector, perpendicular;

      // 頂点を構成する線分の単位方向ベクトルを求める
      double position[3] = {0.0, 0.0, 0.0};
      getDirectionVector(position, vertex[num].endpoint1, vertex[num].direction1, &vec1);
      getDirectionVector(position, vertex[num].endpoint2, vertex[num].direction2, &vec2);

      // 頂点を構成する線分の成す角を求める
      vangle = cvDotProduct(&vec1, &vec2);
      vangle = (acos(vangle) / M_PI) * 180.0;
      vertex[num].angle = vangle;

      // 以下の３つのベクトルを用いて姿勢を表す行列をつくる
      // 頂点の法線を求める
      normal = cvMat(3, 1, CV_64FC1, vertex[num].orientation[2]);
      cvCrossProduct(&vec1, &vec2, &normal);
      // 頂点を構成する線分が成す角の２等分線（単位方向ベクトルの中線）を求める
      bisector = cvMat(3, 1, CV_64FC1, vertex[num].orientation[1]);
      cvAdd(&vec1, &vec2, &bisector);
      cvNormalize(&bisector, &bisector);
      // 頂点の法線と中線の両方に直交する軸の方向を求める
      perpendicular = cvMat(3, 1, CV_64FC1, vertex[num].orientation[0]);
      cvCrossProduct(&bisector, &normal, &perpendicular);

      // 3次元頂点位置
      for (int i = 0; i < 3; ++i)
        {
          vertex[num].orientation[3][i] = vpos[i];
        }
      vertex[num].orientation[3][3] = 1.0;

      vertex[num].n = -1;
      vertex[num].side = M3DF_FRONT;

      vertex[num].label = M3DF_LABEL_NONE;
      vertex[num].numOfTracePoints = 0;
      vertex[num].tracepoints = NULL;
      vertex[num].transformed = NULL;
      vertex[num].projected = NULL;

      ++num;
    }

  //fprintf(stderr, "# num: %d\n", num);
  return num;
}

void 
reconstruct_hyperbola_to_vertex3D(const std::vector<const ovgr::Features2D*> feature,
                                  const ovgr::CorrespondingSet& cs,
                                  const CameraParam* camParam[3],
                                  const unsigned char* edge[3],
                                  Features3D* scene,
                                  const Parameters& parameters)
{
  // 使用するパラメータの設定
  const double ethr = parameters.stereo.ethr;
  const double depn = parameters.stereo.depn >= 0.0 ? parameters.stereo.depn : 0.0;
  const double depf = parameters.stereo.depf >= 0.0 ? parameters.stereo.depf : -1.0;
  const double amin = parameters.stereo.amin;
  const double amax = parameters.stereo.amax;
  //const double lmin = parameters.stereo.lmin;
  //const double lmax = parameters.stereo.lmax;

  // カメラの各組み合わせに対するF行列を計算
  std::vector<cv::Mat> F;
  if (feature.size() < 3)
    {
      F.resize(1);
      F[0] = ovgr::calc_fundamental_matrix(*camParam[0], *camParam[1]);
    }
  else
    {
      F.resize(3);

      for (int i = 0; i < 2; ++i)
        {
          for (int j = i+1; j < 3; ++j)
            {
              F[i + (j-1)*j/2] = ovgr::calc_fundamental_matrix(*camParam[i], *camParam[j]);
            }
        }
    }

  // 対応候補（頂点の組）について繰り返し
  std::vector< ::Vertex> scene_temp; // 頂点復元結果格納用
  for (ovgr::feature_list_t::const_iterator it = cs.vertex.begin(); it != cs.vertex.end(); ++it)
    {
      std::vector<Feature2D_old*> old_f(it->size());
      std::vector<const ovgr::VertexFeature*> new_f(it->size());
      std::vector<int> c_index(it->size(), -1); // カメラのインデックス
      size_t n = 0;

      for (size_t c = 0; c < it->size(); ++c)
        {
          // -1 はカメラ c における対応候補が見つからなかったことを示す
          if ((*it)[c] != -1)
            {
              c_index[n] = c;
              new_f[n] = &(feature[c]->vertex[(*it)[c]]);
              n++;
            }
        }

      // 頂点情報
      Data_2D pos[2] = {{new_f[0]->mid.x, new_f[0]->mid.y},
                        {new_f[1]->mid.x, new_f[1]->mid.y}};
      double pos3D[3] = {0.0, 0.0, 0.0};
      double error = calculateLR2XYZ(pos3D, pos[0], pos[1],
                                     const_cast<CameraParam*>(camParam[c_index[0]]), 
                                     const_cast<CameraParam*>(camParam[c_index[1]]));
      if (error > ethr)
        {
          continue;
        }

      // 一つ目のカメラと復元した頂点の距離を求める
      double dep = camParam[c_index[0]]->Translation[2];
      for (int i = 0; i < 3; ++i)
        {
          dep += camParam[c_index[0]]->Rotation[2][i] * pos3D[i];
        }
      //fprintf(stderr, "dep: %f [%f, %f]\n", dep, depn, depf);

      // dep が[depn, depf]の範囲外であれば不採用
      if (dep < depn || (depf > 0.0 && depf < dep))
        {
          continue;
        }

      // F行列を求める
#if 0
      bool inv = false;
      cv::Mat Fmat = ovgr::calc_fundamental_matrix(*camParam[c_index[0]], *camParam[c_index[1]]);

#else
      bool inv = c_index[0] > c_index[1];
      cv::Mat Fmat;
      if (!inv)
        {
          Fmat = F[c_index[0] + (c_index[1]-1)*c_index[1]/2];
        }
      else
        {
          Fmat = F[c_index[1] + (c_index[0]-1)*c_index[0]/2];
        }
#endif

      // 対応する2次元頂点の組から3次元の頂点特徴を生成
      ::Vertex vertex[2];
      int num_vertex = 0;
      memset(vertex, 0, sizeof(::Vertex) * 2);
      num_vertex = reconstruct_vertex3D(vertex, *new_f[0], *new_f[1], Fmat, inv,
                                        *camParam[c_index[0]], *camParam[c_index[1]]);
      for (int i = 0; i < num_vertex; ++i)
        {
          if (vertex[i].angle < amin || amax < vertex[i].angle)
            {
              continue;
            }

          scene_temp.push_back(vertex[i]);
        }
    } /* for (it) */

  // 旧3次元特徴へ代入
  // 表裏の特徴をつくるため元の2倍の領域を確保する
  scene->numOfVertices = scene_temp.size() * 2;

  scene->Vertices = (Vertex*) calloc(scene->numOfVertices, sizeof(Vertex));
  if (scene->Vertices == NULL)
    {
      return;
    }

  for (size_t i = 0; i < scene_temp.size(); ++i)
    {
      scene_temp[i].n = i; // 通し番号

      // 表の特徴をコピー
      memcpy(&scene->Vertices[2*i], &scene_temp[i], sizeof(Vertex));

      // 裏の特徴を作成
      reverseVertex(scene->Vertices[2*i], scene->Vertices[2*i + 1]);
    }

#if 0
  for (int i = 0; i < scene->numOfVertices; ++i)
    {
      Data_2D pos;

      projectXYZ2LR(&pos, scene->Vertices[i].endpoint1, const_cast<CameraParam*>(camParam[0]));
      printf("% f % f\n", pos.col, pos.row);

      projectXYZ2LR(&pos, scene->Vertices[i].endpoint2, const_cast<CameraParam*>(camParam[0]));
      printf("% f % f\n", pos.col, pos.row);

      printf("\n");
    }
#endif
}
