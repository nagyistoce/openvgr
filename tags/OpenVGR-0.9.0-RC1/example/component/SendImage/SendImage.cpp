/*
 SendImage.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  SendImage.cpp
 * @brief Send Image To Recognition Component for test
 * @date  $Date:: 2011-06-21 19:32:30 +0900 #$
 */

#include "SendImage.h"

using namespace std;

// Module specification
// <rtc-template block="module_spec">
static const char *sendimage_spec[] = {
  "implementation_id", "SendImage",
  "type_name", "SendImage",
  "description", "Send Image To Recognition Component for test",
  "version", "1.0.0",
  "vendor", "AIST",
  "category", "Test Component",
  "activity_type", "PERIODIC",
  "kind", "DataFlowComponent",
  "max_instance", "0",
  "language", "C++",
  "lang_type", "compile",
  // Configuration variables
  "conf.default.ReadImageNum", "3",
  "conf.default.OutputPointNum", "-1",
  "conf.default.ErrorCode", "",
  "conf.default.CalibDir", "",
  "conf.default.CalibFile", "",
  "conf.default.ImageDir", "",
  "conf.default.ImageFile0", "",
  "conf.default.ImageFile1", "",
  "conf.default.ImageFile2", "",

  ""
};

// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SendImage::SendImage(RTC::Manager* manager)
  // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_stereo3DOutOut("Stereo3D", m_stereo3DOut),
    m_Reconstruct3DPort("Reconstruct3D")
    // </rtc-template>
{
  m_readySendImageFlag = false;

  m_Reconstruct3D.setSendImageFlag(&m_readySendImageFlag);
  m_Reconstruct3D.setStereo3DData(&m_stereo3DOut);
}

/*!
 * @brief destructor
 */
SendImage::~SendImage()
{
}



RTC::ReturnCode_t SendImage::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  addOutPort("Stereo3D", m_stereo3DOutOut);

  // Set service provider to Ports
  m_Reconstruct3DPort.registerProvider("Reconstruct3D", "Reconstruct3DService", m_Reconstruct3D);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_Reconstruct3DPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("ReadImageNum", m_readImageNum, "3");
  bindParameter("OutputPointNum", m_outputPointNum, "-1");
  bindParameter("ErrorCode", m_errorCode, "");
  bindParameter("CalibDir",  m_calibDir, "");
  bindParameter("CalibFile", m_calibFile, "");
  bindParameter("ImageDir",  m_imageDir, "");
  bindParameter("ImageFile0", m_imageFile0, "");
  bindParameter("ImageFile1", m_imageFile1, "");
  bindParameter("ImageFile2", m_imageFile2, "");

  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SendImage::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SendImage::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SendImage::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SendImage::onActivated(RTC::UniqueId ec_id)
{
  // 画像を読み込む枚数をセットする。
  m_Reconstruct3D.setReadImageNum(m_readImageNum);
  // 出力する点群数をセットする。
  m_Reconstruct3D.setOutputPointNum(m_outputPointNum);
  // 出力エラーコードをセットする。
  m_Reconstruct3D.setErrorCode(m_errorCode);

  // キャリブレーションファイル名をセットする。
  string calibFile;

  if(m_calibDir != "")
    {
      calibFile = m_calibDir + "/" + m_calibFile;
    }
  else
    {
      calibFile = m_calibFile;
    }

  cout << calibFile << endl;
  m_Reconstruct3D.setCalibFilename(calibFile);

  // 画像ファイル名をセットする。
  string imageFile0, imageFile1, imageFile2;

  if (m_imageDir != "")
    {
      imageFile0 = m_imageDir + "/" + m_imageFile0;
      imageFile1 = m_imageDir + "/" + m_imageFile1;
      imageFile2 = m_imageDir + "/" + m_imageFile2;
    }
  else
    {
      imageFile0 = m_imageFile0;
      imageFile1 = m_imageFile1;
      imageFile2 = m_imageFile2;
    }

  cout << imageFile0 << endl;
  cout << imageFile1 << endl;
  cout << imageFile2 << endl;

  m_Reconstruct3D.setImageFilename(imageFile0, imageFile1, imageFile2);

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SendImage::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SendImage::onExecute(RTC::UniqueId ec_id)
{
  if (m_readySendImageFlag == true)
    {
      // 読み込んだ画像データを送信する
//    TimedStereo3D *data = m_Reconstruct3D.getStereo3DData();
//    m_stereo3DOut = *data;

      m_stereo3DOutOut.write();

      m_readySendImageFlag = false;
    }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SendImage::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SendImage::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SendImage::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SendImage::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SendImage::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
  void
  SendImageInit(RTC::Manager* manager)
  {
    coil::Properties profile(sendimage_spec);
    manager->registerFactory(profile, RTC::Create<SendImage>, RTC::Delete<SendImage>);
  }
};
