/*
 Measure3D.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  Measure3D.cpp
 * @brief Calculate 3D from stereo images
 * @date  $Date:: 2011-06-21 19:32:30 +0900 #$
 */

//OpenCV header file include
#include<cv.h>
#include<cxcore.h>
#include<highgui.h>

#include "Measure3D.h"
#include "measure3d_scene.h"

// Module specification
// <rtc-template block="module_spec">
static const char* measure3d_spec[] =
  {
    "implementation_id", "Measure3D",
    "type_name",         "Measure3D",
    "description",       "Calculate 3D from stereo images",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Measure3D::Measure3D(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_img_inIn("ImageIn", m_img_in),
    m_Data3DOutOut("Data3DOut", m_Data3DOut),
    m_Reconstruct3DPort("Reconstruct3D"),
    m_CameraCapturePort("CameraCapture"),
    m_Reconstruct3DService(*this)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Measure3D::~Measure3D()
{
}



RTC::ReturnCode_t Measure3D::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("ImageIn", m_img_inIn);
  
  // Set OutPort buffer
  addOutPort("Data3DOut", m_Data3DOutOut);
  
  // Set service provider to Ports
  m_Reconstruct3DPort.registerProvider("Reconstruct3D", "Reconstruct3DService", m_Reconstruct3DService);
  
  // Set service consumers to Ports
  m_CameraCapturePort.registerConsumer("CameraCaptureService", "Img::CameraCaptureService", m_CaptureCameraService);
  
  // Set CORBA Service Ports
  addPort(m_Reconstruct3DPort);
  addPort(m_CameraCapturePort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Measure3D::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Measure3D::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Measure3D::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


static void
release_work3D(Work3D *work)
{
  int	i;

  for(i = 0; i < NIMAGE; i++)
    {
      cvReleaseMat(&(work->cmat[i]));
      cvReleaseMat(&(work->dcoef[i]));
      cvReleaseMat(&(work->Ri[i]));
      cvReleaseMat(&(work->Pi[i]));
    }
  cvReleaseMat(&(work->R));
  cvReleaseMat(&(work->T));
  cvReleaseMat(&(work->Q));

  cvReleaseMat(&(work->state.preFilteredImg0));
  cvReleaseMat(&(work->state.preFilteredImg1));
  cvReleaseMat(&(work->state.slidingSumBuf));
  cvReleaseMat(&(work->state.dbmin));
  cvReleaseMat(&(work->state.dbmax));

  return;
}

static void
create_work3D(Work3D *work)
{
  int	i;

  for(i = 0; i < NIMAGE; i++)
    {
      work->cmat[i] = cvCreateMat(3,3,CV_64FC1);
      work->dcoef[i] = cvCreateMat(NDCOEF,1,CV_64FC1);
      work->Ri[i] = cvCreateMat(3,3,CV_64FC1);
      work->Pi[i] = cvCreateMat(3,4,CV_64FC1);;
    }
  work->R = cvCreateMat(3,3,CV_64FC1);
  work->T = cvCreateMat(3,1,CV_64FC1);
  work->Q = cvCreateMat(4,4,CV_64FC1);

  return;
}

RTC::ReturnCode_t Measure3D::onActivated(RTC::UniqueId ec_id)
{
  char config_fname[] = "measure3d_config.d";

  // read params for SAD
  /*fprintf(stderr, "MEMO:read_measure3d_config()\n");*/
  if (read_measure3d_config(config_fname, &work.state))
    {
      fprintf(stderr, "Error in reading config file \"%s\". Aborted\n",
	      config_fname);
      return RTC::RTC_ERROR;
    }
  
  /*fprintf(stderr, "MEMO:create_work()\n");*/
  create_work3D(&work);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t Measure3D::onDeactivated(RTC::UniqueId ec_id)
{
  release_work3D(&work);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t Measure3D::onExecute(RTC::UniqueId ec_id)
{
  if (m_img_inIn.isNew())
    {
      int state = 0;

      m_img_inIn.read();

      // pass Img and get Stereo3D
      state = measure3d_scene(&m_img_in, &m_Data3DOut.data, &work);
      if (state != 0)
        {
          return RTC::RTC_ERROR;
        }
      setTimestamp(m_Data3DOut);

      m_Data3DOutOut.write();
    }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Measure3D::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Measure3D::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Measure3D::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Measure3D::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Measure3D::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void Measure3DInit(RTC::Manager* manager)
  {
    coil::Properties profile(measure3d_spec);
    manager->registerFactory(profile,
                             RTC::Create<Measure3D>,
                             RTC::Delete<Measure3D>);
  }
  
};


