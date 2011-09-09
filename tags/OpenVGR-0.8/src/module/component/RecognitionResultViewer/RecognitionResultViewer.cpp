/*
 RecognitionResultViewer.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  RecognitionResultViewer.cpp
 * @brief Image Viewer for Result of Recognition
 * @date \$Date::                            $
 */
#include "RecognitionResultViewer.h"

#include <gtk/gtk.h>
#include <gdk/gdk.h>

// Module specification
// <rtc-template block="module_spec">
static const char* recognitionresultviewer_spec[] = {
  "implementation_id", "RecognitionResultViewer",
  "type_name", "RecognitionResultViewer",
  "description", "Image Viewer for Result of Recognition",
  "version", "1.0.0",
  "vendor", "AIST",
  "category", "Recongition Result Viewer",
  "activity_type", "PERIODIC",
  "kind", "DataFlowComponent",
  "max_instance", "0",
  "language", "C++",
  "lang_type", "compile",
  // Configuration variables
  "conf.default.RecogModelListPath", "modelList.txt",
  // Widget
  "conf.__widget__.RecogModelListPath", "text",
  // Constraints
  ""
};

// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RecognitionResultViewer::RecognitionResultViewer(RTC::Manager* manager)
  :
  // <rtc-template block="initializer">
  RTC::DataFlowComponentBase(manager),
  m_RecognitionResultViewerPort("RecognitionResultViewer")
  // </rtc-template>
{
  m_modelList.modelNum = 0;
  m_modelList.model = NULL;
}

/*!
 * @brief destructor
 */
RecognitionResultViewer::~RecognitionResultViewer()
{
  // モデルファイル一覧のクリア
  if (m_modelList.model != NULL)
    {
      clearModelFileInfo(&m_modelList);
    }
}

//
// 画面の解像度を取得する。
//
void
getScreenSize(int& width, int& height)
{
  int argc = 0;
  gtk_init(&argc, NULL);

  GdkScreen* screen;

  screen = gdk_screen_get_default();

  width = gdk_screen_get_width(screen);
  height = gdk_screen_get_height(screen);
  return;
}


RTC::ReturnCode_t RecognitionResultViewer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer

  // Set service provider to Ports
  m_RecognitionResultViewerPort.registerProvider("RecognitionResultViewer",
                                                 "RecognitionResultViewerService",
                                                 m_RecognitionResultViewer);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_RecognitionResultViewerPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("RecogModelListPath", m_recogModelListPath, "modelList.txt");

  // </rtc-template>

  m_RecognitionResultViewer.setModelList(&m_modelList);

  int screenWidth = 1024;
  int screenHeight = 768;

  // 画面の解像度を取得する。
  getScreenSize(screenWidth, screenHeight);

  m_RecognitionResultViewer.setScreenSize(screenWidth, screenHeight);

  printf("Screen = %d, %d\n", screenWidth, screenHeight);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t RecognitionResultViewer::onFinalize()
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RecognitionResultViewer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RecognitionResultViewer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RecognitionResultViewer::onActivated(RTC::UniqueId ec_id)
{
  // モデルファイル一覧の読み込み。
  // モデル ID とモデルファイル名の組を読み込んで、保持しておく。
  int ret = loadModelListFile((char*) m_recogModelListPath.c_str(), &m_modelList);
  if (ret != 0)
    {
      m_RecognitionResultViewer.setModelList(NULL);
      return RTC::BAD_PARAMETER;
    }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t RecognitionResultViewer::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RecognitionResultViewer::onExecute(RTC::UniqueId ec_id)
{
  m_RecognitionResultViewer.displayWindow();
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RecognitionResultViewer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RecognitionResultViewer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RecognitionResultViewer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RecognitionResultViewer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RecognitionResultViewer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C" {
  void
  RecognitionResultViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(recognitionresultviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create < RecognitionResultViewer >,
                             RTC::Delete < RecognitionResultViewer >);
    return;
  }
}
