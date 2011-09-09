/*
 MultiCameraTrigger.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  MultiCameraTrigger.cpp
 * @brief trigger MultiCamera component
 * @date $Date::                            $
 */

#include "MultiCameraTrigger.h"

// Module specification
// <rtc-template block="module_spec">
static const char* multicameratrigger_spec[] =
  {
    "implementation_id", "MultiCameraTrigger",
    "type_name",         "MultiCameraTrigger",
    "description",       "trigger MultiCamera component",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    "exec_cxt.periodic.rate", "10.0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
MultiCameraTrigger::MultiCameraTrigger(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_controlPort("control")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
MultiCameraTrigger::~MultiCameraTrigger()
{
}



RTC::ReturnCode_t MultiCameraTrigger::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_controlPort.registerConsumer("CameraCaptureService", "Img::CameraCaptureService", m_cmd);
  
  // Set CORBA Service Ports
  registerPort(m_controlPort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MultiCameraTrigger::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCameraTrigger::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCameraTrigger::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCameraTrigger::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCameraTrigger::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MultiCameraTrigger::onExecute(RTC::UniqueId ec_id)
{
  if (::CORBA::is_nil(m_cmd.getObject())) {
    return RTC::RTC_ERROR;
  }

  m_cmd->take_one_frame();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MultiCameraTrigger::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCameraTrigger::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCameraTrigger::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCameraTrigger::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCameraTrigger::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void MultiCameraTriggerInit(RTC::Manager* manager)
  {
    coil::Properties profile(multicameratrigger_spec);
    manager->registerFactory(profile,
                             RTC::Create<MultiCameraTrigger>,
                             RTC::Delete<MultiCameraTrigger>);
  }
  
};


