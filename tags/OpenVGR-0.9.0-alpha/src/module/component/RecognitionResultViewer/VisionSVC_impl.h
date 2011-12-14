/*
 VisionSVC_impl.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*-C++-*-
/*!
 * @file  VisionSVC_impl.h
 * @brief Service implementation header of vision.idl
 */

#include <pthread.h>
#include "VisionSkel.h"
#include "match3Dfeature.h"
#include "modelListFileIO.h"
#include "recogResult.h"

#ifndef VISIONSVC_IMPL_H
#define VISIONSVC_IMPL_H

/*
 * @class Reconstruct3DServiceSVC_impl
 * Example class implementing IDL interface Reconstruct3DService
 */
class Reconstruct3DServiceSVC_impl
  : public virtual POA_Reconstruct3DService,
    public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~Reconstruct3DServiceSVC_impl();

public:
  /*
   * @brief standard constructor
   */
  Reconstruct3DServiceSVC_impl();
  /*
   * @brief destructor
   */
  virtual ~Reconstruct3DServiceSVC_impl();

  // attributes and operations
  void reconstruct();
};

/*
 * @class RecognitionServiceSVC_impl
 * Example class implementing IDL interface RecognitionService
 */
class RecognitionServiceSVC_impl
  : public virtual POA_RecognitionService,
    public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~RecognitionServiceSVC_impl();

public:
  /*
   * @brief standard constructor
   */
  RecognitionServiceSVC_impl();
  /*
   * @brief destructor
   */
  virtual ~RecognitionServiceSVC_impl();

  // attributes and operations
  CORBA::Long getModelID();
  void setModelID(CORBA::Long ModelID);
};

/*!
 * @class RecognitionResultViewerServiceSVC_impl
 * Example class implementing IDL interface RecognitionResultViewerService
 */
class RecognitionResultViewerServiceSVC_impl
  : public virtual POA_RecognitionResultViewerService,
    public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~RecognitionResultViewerServiceSVC_impl();

  //! モデル ID とモデルファイルのパスのリスト
  ModelFileInfo* m_modelList;

  //! 表示用画像領域
  IplImage* m_displayImage;
  //! 拡大縮小表示用画像領域
  IplImage* m_zoomImage;

  //! OpenCV のウインドウ名
  char m_displayWindowName[256];

  //! ウインドウをつくるかどうかのフラグ
  bool m_WindowFlag;

  //! 画面の解像度 (幅)
  int m_screenWidth;
  //! 画面の解像度 (縦)
  int m_screenHeight;

  //! 表示バッファ同期
  pthread_mutex_t m_mutex_image;
  pthread_mutex_t m_mutex_flag;

  //! キャリブレーションデータを保存(デバッグ用)
  void saveDisplayData(const Img::TimedMultiCameraImage& frame, const TimedRecognitionResult& pos);

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
  //! 認識結果の表示
  void display(const Img::TimedMultiCameraImage& frame, const TimedRecognitionResult& pos);

  //! 表示画像を表示
  void displayWindow();

  //! モデルファイル一覧のセット
  void setModelList(ModelFileInfo* info);

  //! 画面の解像度を設定する
  void setScreenSize(const int width, const int height);

  //! 表示フラグ値の設定
  void setWindowFlag(bool val);

  //! 表示フラグ値の取得
  bool getWindowFlag();
};


#endif // VISIONSVC_IMPL_H
