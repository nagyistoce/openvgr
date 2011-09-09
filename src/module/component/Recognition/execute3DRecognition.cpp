/*
 execute3DRecognition.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include "execute3DRecognition.h"
#include "imageUtil.h"
#include "modelFileio.h"
#include "calibUtil.h"
#include "stereo.h"
#include "circle.h"
#include "vertex.h"
#include "extractEdge.h"
#include "extractFeature.h"
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

//
//! 3 次元物体認識の実行
//
//! 画像データから 3 次元特徴を抽出し、指定モデルファイルとマッチングを行い、
//! 結果を返す。
//
//! 引数:
//!   Img::TImedMultiCameraImage& frame  :       カメラ画像とキャリブレーションデータ
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

  // 3 次元特徴を抽出する。

  unsigned char* edgeL = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
  if (edgeL == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  unsigned char* edgeR = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
  if (edgeR == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  unsigned char* edgeV = NULL;
  if (imageNum > 2)
    {
      edgeV = (unsigned char*) malloc(colsize * rowsize * sizeof(unsigned char));
      if (edgeV == NULL)
        {
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
  if (imageNum > 2)
    {
      model.image[2] = recogImage[2]->pixel;
      model.edge[2] = edgeV;
    }

  // 処理時間計測
  unsigned long stime, etime, dtime1, dtime2, dtime3;

  stime = GetTickCount();

  // 個々の画像から、二次元特徴の抽出を行う。
  Features2D* featuresL;
  Features2D* featuresR;
  Features2D* featuresV = NULL;

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
      featuresL = ImageToFeature2D(recogImage[0]->pixel, edgeL, param, model);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D(recogImage[1]->pixel, edgeR, param, model);
      break;

    case DBL_LV:
      featuresL = ImageToFeature2D(recogImage[0]->pixel, edgeL, param, model);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D(recogImage[2]->pixel, edgeR, param, model);
      break;

    case DBL_RV:
      featuresL = ImageToFeature2D(recogImage[1]->pixel, edgeL, param, model);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D(recogImage[2]->pixel, edgeR, param, model);
      break;

    default:
      featuresL = ImageToFeature2D(recogImage[0]->pixel, edgeL, param, model);
      param.feature2D.id = 1;
      featuresR = ImageToFeature2D(recogImage[1]->pixel, edgeR, param, model);
      if (imageNum == 3)
        {
          param.feature2D.id = 2;
          featuresV = ImageToFeature2D(recogImage[2]->pixel, edgeV, param, model);
        }
      break;
    }

  // 2D 処理時間計測
  etime = GetTickCount();
  dtime1 = etime - stime;
  stime = etime;

  // ステレオ処理パラメータの設定

  // 各ペアで、2次元特徴のすべての組み合わせでステレオ対応を試みる。
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

  // 3D 処理時間計測
  etime = GetTickCount();
  dtime2 = etime - stime;

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
  destructFeatures(featuresL);
  destructFeatures(featuresR);
  if (featuresV != NULL)
    {
      destructFeatures(featuresV);
    }

  if (status == false)
    {
      for (i = 0; i < imageNum; i++)
        {
          destructImage(*(recogImage + i));
        }
      free(recogImage);

      free(edgeL);
      free(edgeR);
      if (edgeV != NULL)
        {
          free(edgeV);
        }

      return VISION_MALLOC_ERROR;
    }

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

  for (i = 0; i < imageNum; i++)
    {
      destructImage(*(recogImage + i));
    }
  free(recogImage);

  free(edgeL);
  free(edgeR);
  if (edgeV != NULL)
    {
      free(edgeV);
    }

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
