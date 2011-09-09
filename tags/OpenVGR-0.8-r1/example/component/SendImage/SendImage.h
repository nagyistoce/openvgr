/*
 SendImage.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  SendImage.h
 * @brief Send Image To Recognition Component for test
 * @date  $Date:: 2011-06-21 19:32:30 +0900 #$
 */

#ifndef SENDIMAGE_H
#define SENDIMAGE_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "VisionSVC_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/*!
 * @class SendImage
 * @brief Send Image To Recognition Component for test
 *
 * テスト用に、3 次元距離計測データが要求されたら画像デ
 * ータを返す。
 *
 */
class SendImage: public RTC::DataFlowComponentBase
{
public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  SendImage (RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~SendImage ();

  // <rtc-template block="public_attribute">

  // </rtc-template>

  // <rtc-template block="public_operation">

  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  virtual RTC::ReturnCode_t onInitialize ();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  virtual RTC::ReturnCode_t onActivated (RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  virtual RTC::ReturnCode_t onExecute (RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


protected:
  // <rtc-template block="protected_attribute">

  // </rtc-template>

  // <rtc-template block="protected_operation">

  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 読み込む画像枚数
   * - Name: ReadImageNum
   * - DefaultValue: 3
   */
  short m_readImageNum;

  /*!
   * 出力する点群数
   * - Name: OutputPointNum
   * - DefaultValue: -1
   */
  int m_outputPointNum;

  /*!
   * 出力するエラーコード
   * - Name: ErrorCode
   * - DefaultValue: ""
   */
  std::string m_errorCode;

  /*!
   * キャリブレーションデータディレクトリ
   * - Name: CalibDir
   * - DefaultValue: ""
   */
  std::string m_calibDir;

  /*!
   * キャリブレーションファイル名
   * - Name: CalibFilename
   * - DefaultValue: ""
   */
  std::string m_calibFile;

  /*!
   * 入力画像ディレクトリ
   * - Name: ImageDir
   * - DefaultValue: ""
   */
  std::string m_imageDir;

  /*!
   * 入力画像ファイル名 0 ～ 2
   * - Name: ImageFile[012]
   * - DefaultValue: ""
   */
  std::string m_imageFile0;
  std::string m_imageFile1;
  std::string m_imageFile2;

  // DataInPort declaration
  // <rtc-template block="inport_declare">

  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedStereo3D m_stereo3DOut;
  /*!
   * 3 次元距離計測データを出力する。(ここでは、画像のみ)
   */
  OutPort<TimedStereo3D> m_stereo3DOutOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   * 3 次元計測データが送られてくるので、画像を返す。
   */
  RTC::CorbaPort m_Reconstruct3DPort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   * 3 次元距離計測データが要求されるので、画像を返す。
   */
  Reconstruct3DServiceSVC_impl m_Reconstruct3D;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>

private:
  // <rtc-template block="private_attribute">

  // 画像を読み込んだフラグ
  bool m_readySendImageFlag;

  // </rtc-template>

  // <rtc-template block="private_operation">

  // </rtc-template>

};


extern
  "C"
{
  DLL_EXPORT void SendImageInit (RTC::Manager* manager);
};

#endif // SENDIMAGE_H
