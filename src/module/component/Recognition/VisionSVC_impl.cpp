/* -*-C++-*-
 VisionSVC_impl.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file  VisionSVC_impl.cpp
 * @brief Service implementation code of Vision.idl
 * @date \$Date::                            $
 */

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include "VisionSVC_impl.h"

/*
 * implementational code for IDL interface Img::CameraCaptureService
 */
CameraCaptureServiceSVC_impl::CameraCaptureServiceSVC_impl()
{
}

CameraCaptureServiceSVC_impl::~CameraCaptureServiceSVC_impl()
{
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void
CameraCaptureServiceSVC_impl::take_one_frame()
{
}

// End of implementational code

/*
 * implementational code for IDL interface Reconstruct3DService
 */
Reconstruct3DServiceSVC_impl::Reconstruct3DServiceSVC_impl()
{
}

Reconstruct3DServiceSVC_impl::~Reconstruct3DServiceSVC_impl()
{
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void
Reconstruct3DServiceSVC_impl::reconstruct()
{
}

// End of implementational code

/*
 * implementational code for IDL interface RecognitionService
 */
RecognitionServiceSVC_impl::RecognitionServiceSVC_impl()
  : m_modelID(0),
    m_modelList(NULL),
    m_modelIDUpdateFlag(false)
{
  setDefaultRecogParameter(m_recogParameter);
}

RecognitionServiceSVC_impl::~RecognitionServiceSVC_impl()
{
}

/*
 * Methods corresponding to IDL attributes and operations
 */
CORBA::Long RecognitionServiceSVC_impl::getModelID()
{
  return m_modelID;
}

//
// 認識するモデル ID のセットと認識実行のための距離計測データ要求
//
void
RecognitionServiceSVC_impl::setModelID(CORBA::Long ModelID)
{
  printf("Accept ModelID: %ld\n", ModelID);

  if (ModelID < 0)
    {
      return;
    }

  m_modelID = ModelID;
  m_modelIDUpdateFlag = true;

  int ret = 0;

  // 認識パラメータの読み込み
  if (m_recogParamPath.size() > 0)
    {
      ret = loadRecogParameter();
      if (ret != 0)
        {
          fprintf(stderr, "認識パラメータファイルの読み込みに失敗しました。[%d]\n", ret);
          return;
        }
    }

  // デバッグ用パラメータを設定する。
  ret = loadDebugParameter();
  if (ret != 0)
    {
      fprintf(stderr, "デバッグ用パラメータの設定に失敗しました。[%d]\n", ret);
      return;
    }

}

//
// モデルファイル一覧のセット
//
void
RecognitionServiceSVC_impl::setModelList(ModelFileInfo* info)
{
  m_modelList = info;
}

//
// 現在セットされている認識モデル ID のモデルファイル名を返す。
//
char*
RecognitionServiceSVC_impl::getModelFilePath()
{
  if (m_modelList == NULL)
    {
      return NULL;
    }

  if (m_modelList->modelNum == 0)
    {
      return NULL;
    }

  int i;

  for (i = 0; i < m_modelList->modelNum; i++)
    {
      if (m_modelList->model[i].id == m_modelID)
        {
          return m_modelList->model[i].path;
        }
    }

  return NULL;
}

//
// 認識設定ファイルのパスを設定する。
//
void
RecognitionServiceSVC_impl::setRecogParameterPath(const char* path)
{
  m_recogParamPath = path;
}

//
// デバッグパラメータをメンバ変数に設定する
//
void
RecognitionServiceSVC_impl::setDebugParameter(int text, int image, int display)
{
  m_DebugText = text;
  m_DebugImage = image;
  m_DebugDisplay = display;
}

//
// 認識パラメータをファイルから読み込む
//
int
RecognitionServiceSVC_impl::loadRecogParameter()
{
  int ret =::loadRecogParameter(const_cast<char*>(m_recogParamPath.c_str()), m_recogParameter);
  return ret;
}

//
// デバッグパラメータをパラメータ構造体に設定する
//
int
RecognitionServiceSVC_impl::loadDebugParameter()
{
  int ret =::loadDebugParameter(m_DebugText, m_DebugImage, m_DebugDisplay, m_recogParameter);
  return ret;
}

//
// 現在の認識パラメータを取得する。
//
void
RecognitionServiceSVC_impl::getCurrentRecogParameter(Parameters* param)
{
  *param = m_recogParameter;
}

//
// モデル ID が更新されたかどうかのフラグを取得する。
//
bool
RecognitionServiceSVC_impl::getModelIDUpdateFlag()
{
  return m_modelIDUpdateFlag;
}

//
// モデル ID 更新フラグのリセット
//
void
RecognitionServiceSVC_impl::resetModelIDUpdateFlag()
{
  m_modelIDUpdateFlag = false;
}

// End of implementational code

/*
 * implementational code for IDL interface RecognitionResultViewerService
 */
RecognitionResultViewerServiceSVC_impl::RecognitionResultViewerServiceSVC_impl()
{
}

RecognitionResultViewerServiceSVC_impl::~RecognitionResultViewerServiceSVC_impl()
{
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void
RecognitionResultViewerServiceSVC_impl::display(
  const Img::TimedMultiCameraImage& frame, const TimedRecognitionResult& pos)
{
}

// End of implementational code
