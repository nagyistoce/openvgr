/*
 SetModelID.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  SetModelID.cpp
 * @brief Set ModelID To Recognition Component for test
 * @date  $Date:: 2011-06-21 19:32:30 +0900 #$
 */

#include "SetModelID.h"

using namespace std;

// Module specification
// <rtc-template block="module_spec">
static const char *setmodelid_spec[] = {
  "implementation_id", "SetModelID",
  "type_name", "SetModelID",
  "description", "Set ModelID To Recognition Component for test",
  "version", "1.0.0",
  "vendor", "AIST.",
  "category", "Test Component",
  "activity_type", "PERIODIC",
  "kind", "DataFlowComponent",
  "max_instance", "0",
  "language", "C++",
  "lang_type", "compile",
  // Configuration variables
  "conf.default.ModelID", "",

  ""
};

// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SetModelID::SetModelID (RTC::Manager * manager)
  // <rtc-template block="initializer">
:RTC::DataFlowComponentBase (manager), m_RecognitionPort ("Recognition")
  // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SetModelID::~SetModelID ()
{
}



RTC::ReturnCode_t SetModelID::onInitialize ()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports
  m_RecognitionPort.registerConsumer ("recogPort", "RecognitionService",
				      m_Recognition);

  // Set CORBA Service Ports
  addPort (m_RecognitionPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter ("ModelID", m_modelID, "");

  return RTC::RTC_OK;
}


RTC::ReturnCode_t SetModelID::onFinalize ()
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SetModelID::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SetModelID::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SetModelID::onActivated(RTC::UniqueId ec_id)
{
  // コンフィグレーションパラメータでモデルIDがセットされていたら
  // 一回だけ送る
  if (m_modelID != "")
    {
      long modelID = -1;
      // Model ID を RecognitionService に送る。
      cout << "モデル ID : " << m_modelID << endl;
      modelID = atoi (m_modelID.c_str ());
      if (modelID >= 0)
        {
          m_Recognition->setModelID (modelID);
        }
    }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t SetModelID::onDeactivated(RTC::UniqueId ec_id)
{
  // コンフィグレーションパラメータをクリアする
  m_modelID = "";
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SetModelID::onExecute (RTC::UniqueId ec_id)
{
  // コンフィグレーションパラメータでモデルIDがセットされないときは従来の動作
  // セットされていたらここでは何もしない
  if (m_modelID == "")
    {
      // Model ID を入力して、RecognitionService に送る。
      string buffer;
      long modelID = -1;
      cout << "モデル ID を入力してください。：";
      cin >> buffer;
      modelID = atoi (buffer.c_str ());
      if (modelID < 0)
        {
          exit ();
        }
      m_Recognition->setModelID (modelID);
    }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SetModelID::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SetModelID::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SetModelID::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SetModelID::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SetModelID::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern
  "C"
{

  void
  SetModelIDInit (RTC::Manager * manager)
  {
    coil::Properties profile (setmodelid_spec);
    manager->
    registerFactory (profile,
		     RTC::Create < SetModelID >, RTC::Delete < SetModelID >);
  }

};
