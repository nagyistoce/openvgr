/*
 VisionSVC_impl.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*-C++-*-
/*!
 * @file  VisionSVC_impl.h
 * @brief Service implementation header of Vision.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ImgSkel.h"

#include "VisionSkel.h"

#include <cv.h>

#ifndef VISIONSVC_IMPL_H
#define VISIONSVC_IMPL_H

/*!
 * @class CameraCaptureServiceSVC_impl
 * Example class implementing IDL interface Img::CameraCaptureService
 */
class CameraCaptureServiceSVC_impl
  : public virtual POA_Img::CameraCaptureService,
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

struct CameraParameter
{
  cv::Mat intr;
  cv::Mat dist;
  cv::Mat ext;
};

/*!
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

  std::vector<CameraParameter> m_params;

  // 画像データを送信するかどうかのフラグ。
  bool* m_readySendImageFlag;

  // 画像データの読み込みバッファ
  TimedStereo3D* m_stereo3DData;

  // 読み込む画像の枚数
  int m_readImageNum;

  // 出力する点群数
  int m_outputPointNum;

  // 出力するエラーコード
  std::string m_errorCode;

  // キャリブレーションファイル名
  std::string m_calibFilename;

  // 入力画像ファイル名 0
  std::string m_imageFilename0;

  // 入力画像ファイル名 1
  std::string m_imageFilename1;

  // 入力画像ファイル名 2
  std::string m_imageFilename2;

  // 画像ファイルの読み込み
  int loadImage();

  // キャリブレーションデータの読み込み
  int loadCalibrationData(char* path);
  int loadCalibrationDataByOpenCV(char* path);
  int loadMultiCalibData(char *path);

  void copy_camera_params(Img::CameraImage* dst, const CameraParameter& param);

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

  // 画像データを送信するかどうかのフラグをセットする。
  void setSendImageFlag(bool* flag);

  // TimedStereo3D 構造体をセットする。
  void setStereo3DData(TimedStereo3D* data);

#if 0
  // 読み込んだ画像データを返す。
  TimedStereo3D* getStereo3DData();
#endif

  // 読み込む画像枚数をセットする。
  void setReadImageNum(int num);
  // 出力点群数をセットする。
  void setOutputPointNum(int num);
  // 出力エラーコードをセットする。
  void setErrorCode(std::string& code);
  // キャリブレーションファイル名をセットする。
  void setCalibFilename(std::string& code);
  // 画像ファイル名をセットする。
  void setImageFilename(std::string& fn0, std::string& fn1, std::string& fn2);
};

/*!
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
  /*!
   * @brief standard constructor
   */
  RecognitionServiceSVC_impl();
  /*!
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
  void display(const Img::TimedMultiCameraImage& frame, TimedRecognitionResult pos);
};



#endif // VISIONSVC_IMPL_H
