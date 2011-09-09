/*
 MultiCamera.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  MultiCamera.cpp
 * @brief Output simultaneously captured images
 * @date $Date::                           $
 */

#include "MultiCamera.h"

#include <cstdio>
#include <iostream>
#include <pthread.h>
#include <cv.h>
#include <capture.h>

// Module specification
// <rtc-template block="module_spec">
static const char* multicamera_spec[] =
  {
    "implementation_id", "MultiCamera",
    "type_name",         "MultiCamera",
    "description",       "Output simultaneously captured images",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    "exec_cxt.periodic.rate", "100.0",
    // Configuration variables
    "conf.default.camera_setting_file", "ieee1394board.0",
    "conf.default.camera_calib_file", "camera_calib.yaml",
    "conf.default.camera_set_id", "1",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
MultiCamera::MultiCamera(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_imagesOut("images", m_images),
    m_controlPort("control"),
    m_cmd(*this),
    m_continuous_mode(false),
    m_frames(NULL)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
MultiCamera::~MultiCamera()
{
}



RTC::ReturnCode_t MultiCamera::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  registerOutPort("images", m_imagesOut);
  
  // Set service provider to Ports
  m_controlPort.registerProvider("CameraCaptureService", "Img::CameraCaptureService", m_cmd);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  registerPort(m_controlPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("camera_setting_file", m_camera_setting_file, "ieee1394board.0");
  bindParameter("camera_calib_file", m_camera_calib_file, "camera_calib.yaml");
  bindParameter("camera_set_id", m_camera_set_id, "1");
  
  // </rtc-template>

  /* initialize m_cap */
  m_cap.num_cameras = 0;
  m_cap.dc1394_cxt  = NULL;
  m_cap.camera_list = NULL;
  m_cap.num_active  = 0;
  m_cap.cameras     = NULL;

  pthread_rwlock_init(&m_rwlock_frames, NULL);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t MultiCamera::onFinalize()
{
  //std::cerr << __FUNCTION__ << " called." << std::endl;
  pthread_rwlock_wrlock(&m_rwlock_frames);

  capture_destroy_frames(&m_cap, &m_frames);
  capture_final(&m_cap);

  pthread_rwlock_unlock(&m_rwlock_frames);

  pthread_rwlock_destroy(&m_rwlock_frames);
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MultiCamera::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCamera::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MultiCamera::onActivated(RTC::UniqueId ec_id)
{
  /* setup camera structures */
  int status;
  status = capture_init(&m_cap);
  if (status != CAPTURE_SUCCESS) {
    return RTC::RTC_ERROR;
  }

  status = capture_setup(&m_cap, m_camera_setting_file.c_str());
  if (status != CAPTURE_SUCCESS) {
    std::cerr << "capture_setup() failed." << std::endl;
    return RTC::RTC_ERROR;
  }

  cv::FileStorage fs;
  int ncamera;
  if (fs.open(m_camera_calib_file, cv::FileStorage::READ) == true) {
      ncamera = fs["num_cameras"];
      //std::cerr << "ncamera: " << ncamera << std::endl;

      /* read camera parameters */
      m_params.resize(ncamera);
      char key[32];
      for (int i = 0; i < ncamera; ++i) {
        snprintf(key, sizeof (key), "camera%d_intr", i);
        cv::read(fs[key], m_params[i].intr);

        snprintf(key, sizeof (key), "camera%d_dist", i);
        cv::read(fs[key], m_params[i].dist);

        snprintf(key, sizeof (key), "camera%d_ext", i);
        cv::read(fs[key], m_params[i].ext);
      }
  }
  else {
    std::cerr << "WARNING: could not open '" << m_camera_calib_file << "'\n" << std::endl;

    ncamera = m_cap.num_active;

    m_params.resize(ncamera);
    for (int i = 0; i < ncamera; ++i) {
      m_params[i].intr = cv::Mat::zeros(3, 3, CV_64F);
      m_params[i].dist = cv::Mat::zeros(5, 1, CV_64F);
      m_params[i].ext  = cv::Mat::eye(4, 4, CV_64F);
    }
  }

  if (ncamera != m_cap.num_active) {
    std::cerr << "numbers of cameras in calibration data and camera setting are different." << std::endl;
    return RTC::RTC_ERROR;
  }

  /* allocate frame memories */
  pthread_rwlock_wrlock(&m_rwlock_frames);
  status = capture_create_frames(&m_cap, &m_frames);
  pthread_rwlock_unlock(&m_rwlock_frames);
  if (status != CAPTURE_SUCCESS) {
    std::cerr << "capture_create_frames() failed." << std::endl;
    return RTC::RTC_ERROR;
  }

  return RTC::RTC_OK;
}


RTC::ReturnCode_t MultiCamera::onDeactivated(RTC::UniqueId ec_id)
{
  pthread_rwlock_wrlock(&m_rwlock_frames);

  capture_destroy_frames(&m_cap, &m_frames);
  capture_final(&m_cap);

  pthread_rwlock_unlock(&m_rwlock_frames);
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MultiCamera::onExecute(RTC::UniqueId ec_id)
{
  capture();

  if (m_continuous_mode) {
    m_imagesOut.write();
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MultiCamera::onAborting(RTC::UniqueId ec_id)
{
  pthread_rwlock_wrlock(&m_rwlock_frames);

  capture_destroy_frames(&m_cap, &m_frames);
  capture_final(&m_cap);

  pthread_rwlock_unlock(&m_rwlock_frames);
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MultiCamera::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MultiCamera::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MultiCamera::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiCamera::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void MultiCamera::capture()
{
  pthread_rwlock_wrlock(&m_rwlock_frames);

  capture_multi(&m_cap, m_frames);
  set_data();

  pthread_rwlock_unlock(&m_rwlock_frames);
}

void MultiCamera::set_data()
{
  m_images.error_code = 0;

  m_images.data.camera_set_id = m_camera_set_id;

  m_images.data.image_seq.length(m_cap.num_active);
  for (int i = 0; i < m_cap.num_active; ++i) {
    Img::CameraImage& cimg = m_images.data.image_seq[i];

    /* timestamp */
    cimg.captured_time.sec  = static_cast< ::CORBA::ULong>(m_frames[i].timestamp / 1000000);
    cimg.captured_time.nsec = static_cast< ::CORBA::ULong>(m_frames[i].timestamp % 1000000) * 1000;

    copy_frame(&cimg.image, m_frames[i]);
    copy_camera_params(&cimg, m_params[i]);
  }

  setTimestamp(m_images);
}

void MultiCamera::copy_frame(Img::ImageData* dst, const capture_frame_t& frame)
{
  int byte_per_pixel = 0, size;

  /* image size */
  dst->width  = frame.width;
  dst->height = frame.height;

  /* format */
  switch (frame.format) {
  case CAPTURE_FRAME_FORMAT_GRAY:
    dst->format = Img::CF_GRAY;
    byte_per_pixel = 1;
    break;

  case CAPTURE_FRAME_FORMAT_RGB:
    dst->format = Img::CF_RGB;
    byte_per_pixel = 3;
    break;

  case CAPTURE_FRAME_FORMAT_UNKNOWN:
  default:
    dst->format = Img::CF_UNKNOWN;
    break;
  }

  /* pixel data */
  size = frame.width * frame.height * byte_per_pixel;
  dst->raw_data.length(size);
  if (byte_per_pixel < 1) {
    return;
  }

  for (int i = 0; i < size; ++i) {
    dst->raw_data[i] = reinterpret_cast<unsigned char*>(frame.raw_data)[i];
  }
}

void MultiCamera::copy_camera_params(Img::CameraImage* dst, const CameraParameter& param)
{
  /* intrinsic */
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j <= i && j < 2; ++j) {
      dst->intrinsic.matrix_element[i*(i+1)/2+j] = param.intr.at<double>(j, i) / param.intr.at<double>(2, 2);
    }
  }

  /* distortion */
  cv::Size s = param.dist.size();
  if (s.width == 1) {
    dst->intrinsic.distortion_coefficient.length(s.height);
    for (int i = 0; i < s.height; ++i) {
      dst->intrinsic.distortion_coefficient[i] = param.dist.at<double>(i, 0);
    }
  }
  else if (s.height == 1) {
    dst->intrinsic.distortion_coefficient.length(s.width);
    for (int i = 0; i < s.width; ++i) {
      dst->intrinsic.distortion_coefficient[i] = param.dist.at<double>(0, i);
    }
  }
  else {
    dst->intrinsic.distortion_coefficient.length(4);
    for (int i = 0; i < 4; ++i) {
      dst->intrinsic.distortion_coefficient[i] = 0.0;
    }
  }

  /* extrinsic */
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      dst->extrinsic[i][j] = param.ext.at<double>(i, j);
    }
  }
}

extern "C"
{
 
  void MultiCameraInit(RTC::Manager* manager)
  {
    coil::Properties profile(multicamera_spec);
    manager->registerFactory(profile,
                             RTC::Create<MultiCamera>,
                             RTC::Delete<MultiCamera>);
  }
  
};
