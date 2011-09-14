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
#include "RecognitionKernel.h"
#include "visionErrorCode.h"
#if 0
#include <fpu_control.h>
#endif

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

  // 認識を実行する。
  Match3Dresults Match = RecognitionKernel(recogImage, calib, model, param);

  freeConvertedRecogImage(recogImage, imageNum);
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

  return Match.error;
}
