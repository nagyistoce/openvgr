/*
 execute3DRecognition.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <vector>

#include "execute3DRecognition.h"
#include "imageUtil.h"
#include "modelFileio.h"
#include "calibUtil.h"
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

unsigned long
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
//! 3 次元物体認識の実行
//
//! 画像データから 3 次元特徴を抽出し、指定モデルファイルとマッチングを行い、
//! 結果を返す。
//
//! 引数:
//!   Img::TimedMultiCameraImage& frame  :       カメラ画像とキャリブレーションデータ
//!   char* modelFilePath                :       モデルファイル名
//!   Parameters& param                  :       認識パラメータ
//!   TimedRecognitionResult& result     :       認識結果
//
int
execute3DRecognition(Img::TimedMultiCameraImage& frame, int modelID,
                     char* modelFilePath, Parameters& param,
                     TimedRecognitionResult& result)
{
#if 0
  // 計算精度の設定
  unsigned short cw;
  _FPU_GETCW(cw);
  cw = (cw & ~0x0300) | _FPU_DOUBLE;    // 53bit 精度
  _FPU_SETCW(cw);
#endif

  // 現在時刻をセット
  struct timeval currentTime;

  gettimeofday(&currentTime, NULL);
  result.tm.sec = currentTime.tv_sec;
  result.tm.nsec = currentTime.tv_usec;
  result.data.length(0);

  if (modelFilePath == NULL)
    {
      // 指定されたモデルデータファイルが存在しない。
      fprintf(stderr, "モデルファイルがありません\n");
      fflush(stderr);
      return VISION_NO_MODEL_FILE;
    }

  if (frame.error_code != 0)
    {
      // 画像にエラーコードがセットされている。
      return frame.error_code;
    }

  int imageNum = frame.data.image_seq.length();
  if (imageNum < 2)
    {
      // 画像がない場合終了。
      return VISION_INPUT_NOIMAGE;
    }
  else if (imageNum > 3)
    {
      imageNum = 3;
    }

  // 画像の正当性をチェック
  if ((frame.data.image_seq[0].image.width == 0) ||
      (frame.data.image_seq[0].image.height == 0))
    {
      return VISION_ILLEGAL_IMAGE_SIZE;
    }

  int i;
  for (i = 1; i < imageNum; i++)
    {
      if ((frame.data.image_seq[i].image.width == 0) ||
          (frame.data.image_seq[i].image.height == 0))
        {
          // 画像の幅または高さが 0
          return VISION_ILLEGAL_IMAGE_SIZE;
        }

      if ((frame.data.image_seq[i].image.width != frame.data.image_seq[0].image.width)
          || (frame.data.image_seq[i].image.height != frame.data.image_seq[0].image.height))
        {
          // 画像のサイズが同一でない
          return VISION_DIFF_IMAGE_SIZE;
        }

      if (frame.data.image_seq[i].image.format != frame.data.image_seq[0].image.format)
        {
          // 画像のカラーモデルが同一でない
          return VISION_DIFF_IMAGE_COLOR_MODEL;
        }
    }

  // TimedMultiCameraImage から、RecogImage 構造体に変換する。
  RecogImage** recogImage = convertTimedMultiCameraImageToRecogImage(frame);
  if (recogImage == NULL)
    {
      // 画像の変換に失敗。
      return VISION_MALLOC_ERROR;
    }

  int colsize = recogImage[0]->colsize;
  int rowsize = recogImage[0]->rowsize;

  param.colsize = colsize;
  param.rowsize = rowsize;
  param.imgsize = colsize * rowsize;

#ifdef _DEBUG
  param.imgL = recogImage[0];
  param.imgR = recogImage[1];
  param.imgV = NULL;
  if (imageNum > 2)
    {
      param.imgV = recogImage[2];
    }
#endif

  Features3D model = { 0 };

  // モデルデータファイルを読み込む。
  int ret = loadModelFile(modelFilePath, model);
  if (ret != 0)
    {
      freeConvertedRecogImage(recogImage, imageNum);
      return VISION_NO_MODEL_FILE;
    }

  // キャリブレーションデータを Camera 構造体に変換する。
  CalibParam calib = { 0 };
  calib.numOfCameras = imageNum;
  calib.colsize = colsize;
  calib.rowsize = rowsize;

  setCalibFromCameraImage(frame.data.image_seq[0], calib.CameraL);
  setCalibFromCameraImage(frame.data.image_seq[1], calib.CameraR);
  if (imageNum > 2)
    {
      setCalibFromCameraImage(frame.data.image_seq[2], calib.CameraV);
    }

  // 画像の歪みを補正する
  undistortImage(recogImage[0], recogImage[0], &calib.CameraL);
  undistortImage(recogImage[1], recogImage[1], &calib.CameraR);
  if (imageNum > 2)
    {
      undistortImage(recogImage[2], recogImage[2], &calib.CameraV);
    }

  // 3 次元特徴を抽出する。

  unsigned char* edgeL = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
  unsigned char* edgeR = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
  if (edgeL == NULL || edgeR == NULL)
    {
      freeConvertedRecogImage(recogImage, imageNum);
      freeEdgeMemory(edgeL, edgeR, NULL);
      return VISION_MALLOC_ERROR;
    }

  unsigned char* edgeV = NULL;
  //if (imageNum > 2)
    {
      edgeV = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
      if (edgeV == NULL)
        {
          freeConvertedRecogImage(recogImage, imageNum);
          freeEdgeMemory(edgeL, edgeR, NULL);
          return VISION_MALLOC_ERROR;
        }
    }

  model.calib = &calib;
  model.image[0] = recogImage[0]->pixel;
  model.image[1] = recogImage[1]->pixel;
  model.edge[0] = edgeL;
  model.edge[1] = edgeR;
  model.trace_pdist = param.match.interval;
  model.trace_search = param.match.search;
  model.trace_edge = param.match.edge;
  //if (imageNum > 2)
    {
      model.image[2] = recogImage[2]->pixel;
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

  switch (pairing)
    {
    case DBL_LR:
      featuresL = ImageToFeature2D_old(recogImage[0]->pixel, edgeL, param, model);
      fL = ovgr::create_new_features_from_old_one(featuresL, recogImage[0]->pixel, &param);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D_old(recogImage[1]->pixel, edgeR, param, model);
      fR = ovgr::create_new_features_from_old_one(featuresR, recogImage[1]->pixel, &param);
      break;

    case DBL_LV:
      featuresL = ImageToFeature2D_old(recogImage[0]->pixel, edgeL, param, model);
      fL = ovgr::create_new_features_from_old_one(featuresL, recogImage[0]->pixel, &param);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D_old(recogImage[2]->pixel, edgeR, param, model);
      fV = ovgr::create_new_features_from_old_one(featuresR, recogImage[2]->pixel, &param);
      break;

    case DBL_RV:
      featuresL = ImageToFeature2D_old(recogImage[1]->pixel, edgeL, param, model);
      fR = ovgr::create_new_features_from_old_one(featuresL, recogImage[1]->pixel, &param);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D_old(recogImage[2]->pixel, edgeR, param, model);
      fV = ovgr::create_new_features_from_old_one(featuresR, recogImage[2]->pixel, &param);
      break;

    default:
      featuresL = ImageToFeature2D_old(recogImage[0]->pixel, edgeL, param, model);
      fL = ovgr::create_new_features_from_old_one(featuresL, recogImage[0]->pixel, &param);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D_old(recogImage[1]->pixel, edgeR, param, model);
      fR = ovgr::create_new_features_from_old_one(featuresR, recogImage[1]->pixel, &param);
      if (imageNum == 3)
        {
          param.feature2D.id = 2;
          featuresV = ImageToFeature2D_old(recogImage[2]->pixel, edgeV, param, model);
          fV = ovgr::create_new_features_from_old_one(featuresV, recogImage[2]->pixel, &param);
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
      freeConvertedRecogImage(recogImage, imageNum);
      freeEdgeMemory(edgeL, edgeR, edgeV);
      return VISION_MALLOC_ERROR;
    }
  else if (pairing == TBL_OR || pairing == TBL_AND)
    {
      if (featuresV == NULL)
        {
          freeFeatures2D(featuresL, featuresR, featuresV);
          freeConvertedRecogImage(recogImage, imageNum);
          freeEdgeMemory(edgeL, edgeR, edgeV);
          return VISION_MALLOC_ERROR;
        }
    }

  // ステレオ処理パラメータの設定

  // 各ペアで、2次元特徴のすべての組み合わせでステレオ対応を試みる。
#if 1
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
#else
  StereoData stereo, stereoLV, stereoRV;

  switch (pairing)
    {
    case DBL_LR:
      /* fall through */
    case DBL_LV:
      /* fall through */
    case DBL_RV:
      stereo = StereoCorrespondence(pairing, calib, featuresL, featuresR, param);
      break;

    case TBL_OR:
      stereo = StereoCorrespondence(DBL_LR, calib, featuresL, featuresR, param);
      stereoLV = StereoCorrespondence(DBL_LV, calib, featuresL, featuresV, param);
      stereoRV = StereoCorrespondence(DBL_RV, calib, featuresR, featuresV, param);
      break;

    case TBL_AND:
      stereo = StereoCorrespondence(DBL_LR, calib, featuresL, featuresR, param);
      stereoLV = StereoCorrespondence(DBL_LV, calib, featuresL, featuresV, param);
      break;

    default:
      stereo = StereoCorrespondence(DBL_LR, calib, featuresL, featuresR, param);
    }

  // 二次元双曲線のステレオデータから三次元頂点情報の復元
  if (model.numOfVertices > 0)
    {
      switch (pairing)
        {
        case DBL_LR:
          /* fall through */
        case DBL_LV:
          /* fall through */
        case DBL_RV:
          HyperbolaToVertex(pairing, calib, stereo, featuresL, featuresR, edgeL, edgeR, param);
          break;

        case TBL_OR:
          HyperbolaToVertex(DBL_LR, calib, stereo, featuresL, featuresR, edgeL, edgeR, param);
          HyperbolaToVertex(DBL_RV, calib, stereoRV, featuresR, featuresV, edgeR, edgeV, param);
          HyperbolaToVertex(DBL_LV, calib, stereoLV, featuresL, featuresV, edgeL, edgeV, param);
          break;

        case TBL_AND:
          HyperbolaToVertex(DBL_LR, calib, stereo, featuresL, featuresR, edgeL, edgeR, param);
          HyperbolaToVertex(DBL_LV, calib, stereoLV, featuresL, featuresV, edgeL, edgeV, param);
          break;

        default:
          HyperbolaToVertex(DBL_LR, calib, stereo, featuresL, featuresR, edgeL, edgeR, param);
        }
    }

  // 二次元楕円のステレオデータから三次元真円情報の復元
  if (model.numOfCircles > 0)
    {
      switch (pairing)
        {
        case DBL_LR:
          /* fall through */
        case DBL_LV:
          /* fall through */
        case DBL_RV:
          EllipseToCircle(pairing, calib, stereo, edgeL, edgeR, param);
          break;

        case TBL_OR:
          EllipseToCircle(DBL_LR, calib, stereo, edgeL, edgeR, param);
          EllipseToCircle(DBL_RV, calib, stereoRV, edgeR, edgeV, param);
          EllipseToCircle(DBL_LV, calib, stereoLV, edgeL, edgeV, param);
          break;

        case TBL_AND:
          EllipseToCircle(DBL_LR, calib, stereo, edgeL, edgeR, param);
          EllipseToCircle(DBL_LV, calib, stereoLV, edgeL, edgeV, param);
          break;

        default:
          EllipseToCircle(DBL_LR, calib, stereo, edgeL, edgeR, param);
        }
    }

  Features3D scene = { 0 };
  bool status;

  // 三次元特徴を認識用特徴データに変換する。
  switch (pairing)
    {
    case DBL_LR:
      /* fall through */
    case DBL_LV:
      /* fall through */
    case DBL_RV:
      status = setFeature3D(stereo, scene);
      break;

    case TBL_OR:
      status = setFeature3D_TBLOR(stereo, stereoLV, stereoRV, scene);
      break;

    case TBL_AND:
      status = setFeature3D_TBLAND(stereo, stereoLV, scene);
      break;

    default:
      status = setFeature3D(stereo, scene);
    }
#endif

  // 3D 処理時間計測
  etime = GetTickCount();
  dtime2 = etime - stime;

#if 0
  // 特徴抽出用変数のクリア
  freeStereoData(&stereo);
  if (pairing == TBL_OR)
    {
      freeStereoData(&stereoRV);
      freeStereoData(&stereoLV);
    }
  else if (pairing == TBL_AND)
    {
      freeStereoData(&stereoLV);
    }

  freeFeatures2D(featuresL, featuresR, featuresV);

  if (status == false)
    {
      freeConvertedRecogImage(recogImage, imageNum);
      freeEdgeMemory(edgeL, edgeR, edgeV);
      return VISION_MALLOC_ERROR;
    }
#else
  freeFeatures2D(featuresL, featuresR, featuresV);
#endif

  // 3 次元特徴とモデルデータのマッチングを行う。
  stime = GetTickCount();

  // シーンとモデルデータを照合して認識を実行する。
  Match3Dresults Match
    = matchFeatures3d(scene, model, edgeL, edgeR, edgeV, param);

  etime = GetTickCount();
  dtime3 = etime - stime;

  // 認識時間計測
  printf("認識時間： %lu msec. (2D: %lu + 3D: %lu + 認識: %lu)\n",
	 dtime1 + dtime2 + dtime3, dtime1, dtime2, dtime3);
  fflush(stdout);

  freeConvertedRecogImage(recogImage, imageNum);
  freeEdgeMemory(edgeL, edgeR, edgeV);

  freeFeatures3D(&scene);
  freeFeatures3D(&model);

  // 認識結果を TimedRecognitionResult 形式に変換する。
  int candNum = Match.numOfResults;
  if (candNum > param.outputCandNum)
    {
      candNum = param.outputCandNum;
    }

  result.data.length(candNum * RecogResultElementNum);
  int count = 0;
  int j, k;

  for (i = 0; i < candNum; i++)
    {
      result.data[count] = (double) frame.data.camera_set_id;
      count++;
      result.data[count] = (double) modelID;
      count++;
      result.data[count] = (double) i;  // Candidate No.
      count++;
      result.data[count] = (double) 0;  // Coordinate System No.
      count++;
      if (Match.Results != NULL)
        {
          result.data[count] = Match.Results[i].score;
        }
      else
        {
          result.data[count] = 0.0;
        }
      count++;
      result.data[count] = (double) Match.error;        // errorNo
      count++;
      result.data[count] = 0.0; // Reserve 1
      count++;
      result.data[count] = 0.0; // Reserve 2
      count++;

      if (Match.numOfResults == 0)
        {
          break;
        }

      for (j = 0; j < 3; j++)
        {
          for (k = 0; k < 4; k++)
            {
              result.data[count] = Match.Results[i].mat[j][k];
              count++;
            }
        }
    }

  // 現在時刻をセット
  gettimeofday(&currentTime, NULL);
  result.tm.sec = currentTime.tv_sec;
  result.tm.nsec = currentTime.tv_usec;

  freeMatch3Dresults(&Match);

  printf("Cand num = %d, Error = %d\n", candNum, Match.error);
  fflush(stdout);

  return 0;
}
