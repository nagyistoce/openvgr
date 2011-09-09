/*
 modelFileio.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include "modelFileio.h"
#include "rtvcm.h"
#include "visionErrorCode.h"

//
//! モデルデータをファイルから読み込む。
//
int
loadModelFile(char* path, Features3D& model)
{
  if (path == NULL)
    {
      return VISION_PARAM_ERROR;
    }

  RTVCM rtvcm = { 0 };
  int ret;

  // モデル RTVCM データを読み込む。
  ret = readRTVCModel(path, rtvcm);
  if (ret != 0)
    {
      return ret;
    }

  // モデル RTVCM データを特徴データ構造体に変換する。
  ret = convertRTVCMtoFeatures3D(rtvcm, model);
  if (ret != 0)
    {
      freeRTVCM(rtvcm);

      return ret;
    }

  freeRTVCM(rtvcm);

  return 0;
}
