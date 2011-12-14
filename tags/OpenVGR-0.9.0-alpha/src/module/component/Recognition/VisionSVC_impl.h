/*
 VisionSVC_impl.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*-C++-*-
/*!
 * @file  VisionSVC_impl.h
 * @brief Service implementation header of Vision.idl
 */

#include <rtm/CorbaPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include "ImgSkel.h"

#include <string>

#include "VisionSkel.h"
#include "match3Dfeature.h"
#include "modelListFileIO.h"
#include "recogParameter.h"
#include "recogResult.h"

#ifndef VISIONSVC_IMPL_H
#define VISIONSVC_IMPL_H

/*!
 * @class CameraCaptureServiceSVC_impl
 * Example class implementing IDL interface Img::CameraCaptureService
 */
class CameraCaptureServiceSVC_impl:
  public virtual POA_Img::CameraCaptureService,
  public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~CameraCaptureServiceSVC_impl();

public:
  /*!
   * @brief standard constructor
   */
  CameraCaptureServiceSVC_impl();
  /*!
   * @brief destructor
   */
  virtual ~CameraCaptureServiceSVC_impl();

  // attributes and operations
  void take_one_frame();

};

/*!
 * @class Reconstruct3DServiceSVC_impl
 * Example class implementing IDL interface Reconstruct3DService
 */
class Reconstruct3DServiceSVC_impl:
  public virtual POA_Reconstruct3DService,
  public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~Reconstruct3DServiceSVC_impl();

public:
  /*!
   * @brief standard constructor
   */
  Reconstruct3DServiceSVC_impl();
  /*!
   * @brief destructor
   */
  virtual ~Reconstruct3DServiceSVC_impl();

  // attributes and operations
  void reconstruct();

};

/*!
 * @class RecognitionServiceSVC_impl
 * Example class implementing IDL interface RecognitionService
 */
class RecognitionServiceSVC_impl:
  public virtual POA_RecognitionService,
  public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~RecognitionServiceSVC_impl();

  //! 認識に使用するモデル ID
  CORBA::Long m_modelID;

  //! モデル ID とモデルファイルのパスのリスト
  ModelFileInfo* m_modelList;

  //! 認識パラメータファイルのパス
  std::string m_recogParamPath;

  //! デバッグ用テキスト情報出力スイッチ
  int m_DebugText;

  //! デバッグ用画像情報出力スイッチ
  int m_DebugImage;

  //! デバッグ用画像情報表示スイッチ
  int m_DebugDisplay;

  //! 現在の認識パラメータ
  Parameters m_recogParameter;

  //! モデル ID が更新されたかどうかのフラグ
  //! (setModelID() が呼び出されたかどうかのフラグ)
  bool m_modelIDUpdateFlag;

public:
  /*!
   * @brief standard constructor
   */
  RecognitionServiceSVC_impl();
  /*!
   * @brief destructor
   */
  virtual ~RecognitionServiceSVC_impl();

  // attributes and operations

  //! 現在設定されているモデル ID を返す。
  CORBA::Long getModelID();

  //! モデル ID を設定し、認識用画像を要求する。
  void setModelID(CORBA::Long ModelID);

  //! モデルファイル一覧のセット
  void setModelList(ModelFileInfo* info);

  //! 現在セットされている認識モデル ID のモデルファイル名を返す。
  char* getModelFilePath();

  //! 認識設定ファイルのパスを設定する。
  void setRecogParameterPath(const char* path);

  //! デバッグ用パラメータを設定する。
  void setDebugParameter(int text, int image, int display);

  //! 認識設定ファイルから認識設定を読み込んで設定を適用する。
  int loadRecogParameter();

  //! デバッグ用パラメータ設定を適用する。
  int loadDebugParameter();

  //! 現在の認識パラメータを取得する。
  void getCurrentRecogParameter(Parameters* param);

  //! モデル ID が更新されたかどうかのフラグを取得する。
  bool getModelIDUpdateFlag();

  //! モデル ID 更新フラグのリセット
  void resetModelIDUpdateFlag();
};

/*!
 * @class RecognitionResultViewerServiceSVC_impl
 * Example class implementing IDL interface RecognitionResultViewerService
 */
class RecognitionResultViewerServiceSVC_impl:
  public virtual POA_RecognitionResultViewerService,
  public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~RecognitionResultViewerServiceSVC_impl();

public:
  /*!
   * @brief standard constructor
   */
  RecognitionResultViewerServiceSVC_impl();
  /*!
   * @brief destructor
   */
  virtual ~RecognitionResultViewerServiceSVC_impl();

  // attributes and operations
  void display(const Img::TimedMultiCameraImage& frame,
	       const TimedRecognitionResult& pos);

};

#endif // VISIONSVC_IMPL_H
