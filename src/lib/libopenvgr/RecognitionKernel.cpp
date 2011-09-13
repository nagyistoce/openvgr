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
#if 0
#include <fpu_control.h>
#endif

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
  int imageNum = param.imagnum;
  int colsize = image[0]->colsize;
  int rowsize = image[0]->rowsize;

  param.colsize = colsize;
  param.rowsize = rowsize;
  param.imgsize = colsize * rowsize;

  // 画像の歪みを補正する
  undistortImage(image[0], image[0], &calib.CameraL);
  undistortImage(image[1], image[1], &calib.CameraR);
  if (imageNum > 2)
    {
      undistortImage(image[2], image[2], &calib.CameraV);
    }

  unsigned char* edgeL = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
  unsigned char* edgeR = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
  if (edgeL == NULL || edgeR == NULL)
    {
      freeEdgeMemory(edgeL, edgeR, NULL);
      Match.error = VISION_MALLOC_ERROR;
      return Match;
    }

  unsigned char* edgeV = NULL;
  if (imageNum > 2)
    {
      edgeV = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
      if (edgeV == NULL)
        {
          freeEdgeMemory(edgeL, edgeR, NULL);
          Match.error = VISION_MALLOC_ERROR;
          return Match;
        }
    }

  model.calib = &calib;
  model.image[0] = image[0]->pixel;
  model.image[1] = image[1]->pixel;
  model.edge[0] = edgeL;
  model.edge[1] = edgeR;
  model.trace_pdist = param.match.interval;
  model.trace_search = param.match.search;
  model.trace_edge = param.match.edge;
  if (imageNum > 2)
    {
      model.image[2] = image[2]->pixel;
      model.edge[2] = edgeV;
    }

  // 処理時間計測
  unsigned long stime, etime, dtime1, dtime2, dtime3;

  stime = GetTickCount();

  // 個々の画像から、二次元特徴の抽出を行う。
  Features2D_old* featuresL = NULL;
  Features2D_old* featuresR = NULL;
  Features2D_old* featuresV = NULL;

  ovgr::Features2D fL, fR, fV;
  ovgr::CorrespondingPair cp_LR, cp_RV, cp_VL;
  std::vector<const ovgr::CorrespondingPair*> cps;
  ovgr::CorrespondingSet cs;
  ovgr::CorrespondenceThresholds cpThs;

  // 画像が 2 枚の時は、強制的に DBL_LR にする。
  StereoPairing pairing = param.pairing;
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
      featuresL = ImageToFeature2D_old(image[0]->pixel, edgeL, param, model);
      fL = ovgr::create_new_features_from_old_one(featuresL, image[0]->pixel, &param);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D_old(image[1]->pixel, edgeR, param, model);
      fR = ovgr::create_new_features_from_old_one(featuresR, image[1]->pixel, &param);
      break;

    case DBL_LV:
      featuresL = ImageToFeature2D_old(image[0]->pixel, edgeL, param, model);
      fL = ovgr::create_new_features_from_old_one(featuresL, image[0]->pixel, &param);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D_old(image[2]->pixel, edgeR, param, model);
      fV = ovgr::create_new_features_from_old_one(featuresR, image[2]->pixel, &param);
      break;

    case DBL_RV:
      featuresL = ImageToFeature2D_old(image[1]->pixel, edgeL, param, model);
      fR = ovgr::create_new_features_from_old_one(featuresL, image[1]->pixel, &param);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D_old(image[2]->pixel, edgeR, param, model);
      fV = ovgr::create_new_features_from_old_one(featuresR, image[2]->pixel, &param);
      break;

    default:
      featuresL = ImageToFeature2D_old(image[0]->pixel, edgeL, param, model);
      fL = ovgr::create_new_features_from_old_one(featuresL, image[0]->pixel, &param);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D_old(image[1]->pixel, edgeR, param, model);
      fR = ovgr::create_new_features_from_old_one(featuresR, image[1]->pixel, &param);
      if (imageNum == 3)
        {
          param.feature2D.id = 2;
          featuresV = ImageToFeature2D_old(image[2]->pixel, edgeV, param, model);
          fV = ovgr::create_new_features_from_old_one(featuresV, image[2]->pixel, &param);
        }
      break;
    }

  // 2D 処理時間計測
  etime = GetTickCount();
  dtime1 = etime - stime;
  stime = etime;

  if (featuresL == NULL || featuresR == NULL)
    {
      freeFeatures2D(featuresL, featuresR, featuresV);
      freeEdgeMemory(edgeL, edgeR, edgeV);
      Match.error = VISION_MALLOC_ERROR;
      return Match;
    }
  else if (pairing == TBL_OR || pairing == TBL_AND)
    {
      if (featuresV == NULL)
        {
          freeFeatures2D(featuresL, featuresR, featuresV);
          freeEdgeMemory(edgeL, edgeR, edgeV);
          Match.error = VISION_MALLOC_ERROR;
          return Match;
        }
    }

  // 各ペアで、二次元特徴のすべての組み合わせでステレオ対応を試みる。
  Features3D scene = { 0 };

  switch (pairing)
    {
    case DBL_LR:
      cp_LR = make_corresponding_pairs(fL, calib.CameraL, fR, calib.CameraR, cpThs);
      break;
    case DBL_LV:
      cp_VL = make_corresponding_pairs(fV, calib.CameraV, fL, calib.CameraL, cpThs);
      break;
    case DBL_RV:
      cp_RV = make_corresponding_pairs(fR, calib.CameraR, fV, calib.CameraV, cpThs);
      break;
    case TBL_OR:
    case TBL_AND:
      cp_LR = make_corresponding_pairs(fL, calib.CameraL, fR, calib.CameraR, cpThs);
      cp_RV = make_corresponding_pairs(fR, calib.CameraR, fV, calib.CameraV, cpThs);
      cp_VL = make_corresponding_pairs(fV, calib.CameraV, fL, calib.CameraL, cpThs);
      break;
    }

  if (pairing != TBL_OR && pairing != TBL_AND)
    {
      cps.resize(1);
    }
  else
    {
      cps.resize(3);
    }

  switch (pairing)
    {
    case TBL_OR:
      cps[0] = &cp_LR;
      cps[1] = &cp_RV;
      cps[2] = &cp_VL;
      cs = ovgr::filter_corresponding_set(cps, ovgr::CorresOr);
      break;
    case TBL_AND:
      cps[0] = &cp_LR;
      cps[1] = &cp_RV;
      cps[2] = &cp_VL;        
      cs = ovgr::filter_corresponding_set(cps, ovgr::CorresAnd);
      break;
    case DBL_LV:
      cps[0] = &cp_VL;
      cs = ovgr::filter_corresponding_set(cps);
      break;
    case DBL_RV:
      cps[0] = &cp_RV;
      cs = ovgr::filter_corresponding_set(cps);
      break;
    case DBL_LR:
    default:
      cps[0] = &cp_LR;
      cs = ovgr::filter_corresponding_set(cps);
      break;
    }

  printf("# vertex: %d, ellipse: %d\n", cs.vertex.size(), cs.ellipse.size());

  {
    const uchar* edges[3] = {0};
    const CameraParam* camParam[3] = {0};
    std::vector<const ovgr::Features2D*> features;
    if(pairing != TBL_OR && pairing != TBL_AND)
      {
        features.resize(2);
      }
    else
      {
        features.resize(3);
      }

    switch (pairing)
      {
      case DBL_LV:
        edges[0] = edgeV;
        edges[1] = edgeL;
        camParam[0] = &calib.CameraV;
        camParam[1] = &calib.CameraL;
        features[0] = &fV;
        features[1] = &fL;
        break;
      case DBL_RV:
        edges[0] = edgeR;
        edges[1] = edgeV;
        camParam[0] = &calib.CameraR;
        camParam[1] = &calib.CameraV;
        features[0] = &fR;
        features[1] = &fV;
        break;
      case TBL_OR:
      case TBL_AND:
        edges[0] = edgeL;
        edges[1] = edgeR;
        edges[2] = edgeV;
        camParam[0] = &calib.CameraL;        
        camParam[1] = &calib.CameraR;
        camParam[2] = &calib.CameraV;
        features[0] = &fL;
        features[1] = &fR;
        features[2] = &fV;
        break;
      case DBL_LR:
      default:
        edges[0] = edgeL;
        edges[1] = edgeR;
        camParam[0] = &calib.CameraL;
        camParam[1] = &calib.CameraR;
        features[0] = &fL;
        features[1] = &fR;
        break;
      }

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
  }

  // 3D 処理時間計測
  etime = GetTickCount();
  dtime2 = etime - stime;

  // 二次元特徴メモリ解放
  freeFeatures2D(featuresL, featuresR, featuresV);

  // 三次元特徴とモデルデータのマッチングを行う。
  stime = GetTickCount();

  // シーンとモデルデータを照合して認識を実行する。
  Match = matchFeatures3d(scene, model, edgeL, edgeR, edgeV, param);

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
