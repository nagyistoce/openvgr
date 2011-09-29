/*
 RecognitionKernel.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date$
*/
#include <vector>

#include <stdio.h>
#include "recogImage.h"
#include "stereo.h"
#include "circle.h"
#include "vertex.h"
#include "extractEdge.h"
#include "extractFeature_old.h"
#include "extractFeature.hpp"
#include "visionErrorCode.h"

#include <sys/time.h>

static unsigned long
GetTickCount()
{
  struct timeval t;
  double msec;
  gettimeofday(&t, NULL);
  msec = (double) (t.tv_sec * 1.0E3 + t.tv_usec * 1.0E-3);
  return (unsigned long) msec;
}

static void
freeEdgeMemory(uchar* edgL, uchar* edgR, uchar* edgV)
{
  free(edgL);
  free(edgR);
  free(edgV);
}

static void
freeFeatures2D(Features2D_old* L, Features2D_old* R, Features2D_old* V)
{
  destructFeatures(L);
  destructFeatures(R);
  destructFeatures(V);
}

//
//! 三次元物体認識の実行
//
//! 画像データから 三次元特徴を抽出し、モデルとマッチングを行い、
//! 結果を返す。
//
//! 引数:
//!   RecogImage** image  :       カメラ画像
//!   CalibParam&  calib  :       カメラキャリブレーションデータ
//!   Features3D&  model  :       対象物体モデル
//!   Parameters&  param  :       認識パラメータ
//
Match3Dresults
RecognitionKernel(RecogImage** image,
                  CalibParam&  calib,
                  Features3D&  model,
                  Parameters&  param)
{
  Match3Dresults Match = {0};
  int imageNum = calib.numOfCameras;
  int colsize = image[0]->colsize;
  int rowsize = image[0]->rowsize;

  param.colsize = colsize;
  param.rowsize = rowsize;
  param.imgsize = colsize * rowsize;

  // 画像の歪みを補正する
  undistortImage(image[0], &calib.CameraL, image[0], &calib.CameraL);
  undistortImage(image[1], &calib.CameraR, image[1], &calib.CameraR);
  if (imageNum > 2)
    {
      undistortImage(image[2], &calib.CameraV, image[2], &calib.CameraV);
    }

  unsigned char* edgeL = (unsigned char*) calloc(colsize * rowsize, sizeof(unsigned char));
  unsigned char* edgeR = (unsigned char*) calloc(colsize * rowsize, sizeof(unsigned char));
  if (edgeL == NULL || edgeR == NULL)
    {
      freeEdgeMemory(edgeL, edgeR, NULL);
      Match.error = VISION_MALLOC_ERROR;
      return Match;
    }

  unsigned char* edgeV = NULL;
  if (imageNum > 2)
    {
      edgeV = (unsigned char*) calloc(colsize * rowsize, sizeof(unsigned char));
      if (edgeV == NULL)
        {
          freeEdgeMemory(edgeL, edgeR, NULL);
          Match.error = VISION_MALLOC_ERROR;
          return Match;
        }
    }

  model.calib = &calib;
  model.edge[0] = edgeL;
  model.edge[1] = edgeR;
  if (imageNum > 2)
    {
      model.edge[2] = edgeV;
    }

  // 処理時間計測
  unsigned long stime, etime, dtime1, dtime2, dtime3;

  stime = GetTickCount();

  Features2D_old* features0 = NULL;
  Features2D_old* features1 = NULL;
  Features2D_old* features2 = NULL;

  ovgr::Features2D f0, f1, f2;
  ovgr::CorrespondingPair cp_bi, cp_LR, cp_RV, cp_VL;
  std::vector<const ovgr::CorrespondingPair*> cps(1);
  ovgr::CorrespondingSet cs;
  ovgr::CorrespondenceThresholds cpThs;

  const uchar* edges[3] = {0};
  const CameraParam* camParam[3] = {0};
  std::vector<const ovgr::Features2D*> features(2);

  StereoPairing pairing = param.pairing;

  // 画像が 2 枚の時は、強制的に DBL_LR にする。
  if (imageNum == 2)
    {
      pairing = DBL_LR;
    }

  // 特徴データ識別 ID 番号
  param.feature2D.id = 0;

  // 二次元特徴の抽出
  switch (pairing)
    {
    case DBL_LR:
      features0 = ImageToFeature2D_old(image[0]->pixel, edgeL, param, model);
      f0 = ovgr::create_new_features_from_old_one(features0, image[0]->pixel, &param);
      param.feature2D.id = 1;
      features1 = ImageToFeature2D_old(image[1]->pixel, edgeR, param, model);
      f1 = ovgr::create_new_features_from_old_one(features1, image[1]->pixel, &param);
      camParam[0] = &calib.CameraL;
      camParam[1] = &calib.CameraR;
      edges[0] = edgeL;
      edges[1] = edgeR;
      break;

    case DBL_LV:
      features0 = ImageToFeature2D_old(image[2]->pixel, edgeV, param, model);
      f0 = ovgr::create_new_features_from_old_one(features0, image[2]->pixel, &param);
      param.feature2D.id = 1;
      features1 = ImageToFeature2D_old(image[0]->pixel, edgeL, param, model);
      f1 = ovgr::create_new_features_from_old_one(features1, image[0]->pixel, &param);
      camParam[0] = &calib.CameraV;
      camParam[1] = &calib.CameraL;
      edges[0] = edgeV;
      edges[1] = edgeL;
      break;

    case DBL_RV:
      features0 = ImageToFeature2D_old(image[1]->pixel, edgeR, param, model);
      f0 = ovgr::create_new_features_from_old_one(features0, image[1]->pixel, &param);
      param.feature2D.id = 1;
      features1 = ImageToFeature2D_old(image[2]->pixel, edgeV, param, model);
      f1 = ovgr::create_new_features_from_old_one(features1, image[2]->pixel, &param);
      camParam[0] = &calib.CameraR;
      camParam[1] = &calib.CameraV;
      edges[0] = edgeR;
      edges[1] = edgeV;
      break;

    case TBL_OR:
    case TBL_AND:
      features0 = ImageToFeature2D_old(image[0]->pixel, edgeL, param, model);
      f0 = ovgr::create_new_features_from_old_one(features0, image[0]->pixel, &param);
      param.feature2D.id = 1;
      features1 = ImageToFeature2D_old(image[1]->pixel, edgeR, param, model);
      f1 = ovgr::create_new_features_from_old_one(features1, image[1]->pixel, &param);
      param.feature2D.id = 2;
      features2 = ImageToFeature2D_old(image[2]->pixel, edgeV, param, model);
      f2 = ovgr::create_new_features_from_old_one(features2, image[2]->pixel, &param);
      camParam[0] = &calib.CameraL;        
      camParam[1] = &calib.CameraR;
      camParam[2] = &calib.CameraV;
      edges[0] = edgeL;
      edges[1] = edgeR;
      edges[2] = edgeV;
      break;

    default:
      freeEdgeMemory(edgeL, edgeR, edgeV);
      Match.error = VISION_PARAM_ERROR;
      return Match;
    }

#ifdef RECOGNITION_TEST
    printf( "RecognitionKernel:RECOGNITION_TEST:StereoPair=%d\n", pairing);
    fflush(stdout);
#endif

  // 2D 処理時間計測
  etime = GetTickCount();
  dtime1 = etime - stime;
  stime = etime;

  if (features0 == NULL || features1 == NULL)
    {
      freeFeatures2D(features0, features1, features2);
      freeEdgeMemory(edgeL, edgeR, edgeV);
      Match.error = VISION_MALLOC_ERROR;
      return Match;
    }
  else if (pairing == TBL_OR || pairing == TBL_AND)
    {
      if (features2 == NULL)
        {
          freeFeatures2D(features0, features1, features2);
          freeEdgeMemory(edgeL, edgeR, edgeV);
          Match.error = VISION_MALLOC_ERROR;
          return Match;
        }
    }

  // 各ペアで、二次元特徴のすべての組み合わせでステレオ対応を試みる。

  if (pairing != TBL_OR && pairing != TBL_AND)
    {
      cp_bi = make_corresponding_pairs(f0, *camParam[0], f1, *camParam[1], cpThs);
      cps[0] = &cp_bi;
      features[0] = &f0;
      features[1] = &f1;
    }
  else
    {
      cp_LR = make_corresponding_pairs(f0, calib.CameraL, f1, calib.CameraR, cpThs);
      cp_RV = make_corresponding_pairs(f1, calib.CameraR, f2, calib.CameraV, cpThs);
      cp_VL = make_corresponding_pairs(f2, calib.CameraV, f0, calib.CameraL, cpThs);
      cps.resize(3);
      cps[0] = &cp_LR;
      cps[1] = &cp_RV;
      cps[2] = &cp_VL;
      features.resize(3);
      features[0] = &f0;
      features[1] = &f1;
      features[2] = &f2;
    }

  if (pairing != TBL_AND)
    {
      // ２眼と３眼ＯＲ
      cs = ovgr::filter_corresponding_set(cps, ovgr::CorresOr);
    }
  else
    {
      // ３眼ＡＮＤ
      cs = ovgr::filter_corresponding_set(cps, ovgr::CorresAnd);
    }

  printf("# vertex: %d, ellipse: %d\n", cs.vertex.size(), cs.ellipse.size());

  Features3D scene = { 0 };

  // 二次元頂点のステレオデータから三次元頂点情報の復元
  if (model.numOfVertices > 0)
    {
      reconstruct_hyperbola_to_vertex3D(features, cs, camParam, edges, &scene, param);
    }

  // 二次元楕円のステレオデータから三次元真円情報の復元
  if (model.numOfCircles > 0)
    {
      reconstruct_ellipse2D_to_circle3D(features, cs, camParam, edges, &scene, param);
    }

  // 3D 処理時間計測
  etime = GetTickCount();
  dtime2 = etime - stime;

  // 二次元特徴メモリ解放
  freeFeatures2D(features0, features1, features2);

  // 三次元特徴とモデルデータのマッチングを行う。
  stime = GetTickCount();

  // シーンとモデルデータを照合して認識を実行する。
  Match = matchFeatures3D(scene, model, param);

  etime = GetTickCount();
  dtime3 = etime - stime;

  // 認識時間計測
  printf("認識時間： %lu msec. (2D: %lu + 3D: %lu + 認識: %lu)\n",
	 dtime1 + dtime2 + dtime3, dtime1, dtime2, dtime3);
  fflush(stdout);

  freeEdgeMemory(edgeL, edgeR, edgeV);
  freeFeatures3D(&scene);

  return Match;
}
