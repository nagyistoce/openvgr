/*
 MultiDisp.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  MultiDisp.cpp
 * @brief Display multiple images
 * @date $Date::                           $
 */

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include <cstdio>
#include <cv.h>
#include <highgui.h>

#include "MultiDisp.h"

// Module specification
// <rtc-template block="module_spec">
static const char* multidisp_spec[] =
  {
    "implementation_id", "MultiDisp",
    "type_name",         "MultiDisp",
    "description",       "Display multiple images",
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
    "conf.default.image_dir", "images",
    "conf.default.image_save_mode", "0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
MultiDisp::MultiDisp(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_imagesIn("images", m_images),
    m_num_windows(0)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
MultiDisp::~MultiDisp()
{
}



RTC::ReturnCode_t MultiDisp::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  registerInPort("images", m_imagesIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("image_dir", m_image_dir, "images");
  bindParameter("image_save_mode", m_image_save_mode, "0");
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MultiDisp::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiDisp::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiDisp::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MultiDisp::onActivated(RTC::UniqueId ec_id)
{
  if (m_image_save_mode != 0 && check_image_dir() == false) {
    return RTC::RTC_ERROR;
  }
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t MultiDisp::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t MultiDisp::onExecute(RTC::UniqueId ec_id)
{
  if (m_imagesIn.isNew()) {
    m_imagesIn.read();

    if (m_images.error_code != 0) {
      return RTC::RTC_OK;
    }

    int nimage = m_images.data.image_seq.length();
    char name[32];
    if (m_num_windows < nimage) {
      for (int i = m_num_windows; i < nimage; ++i) {
        snprintf(name, sizeof (name), "image %d", i);
        cv::namedWindow(name, CV_WINDOW_AUTOSIZE);
      }
      m_num_windows = nimage;
    }

    m_cv_images.clear();
    for (int i = 0; i < nimage; ++i) {
      Img::ImageData& idat = m_images.data.image_seq[i].image;
      cv::Mat image;

      switch (idat.format) {
      case CF_GRAY:
        image.create(idat.height, idat.width, CV_8UC1);
        for (int j = 0; j < idat.height; ++j) {
          unsigned char* row = image.ptr(j);
          for (int k = 0; k < idat.width; ++k) {
            row[k] = idat.raw_data[j*idat.width + k];
          }
        }
        break;

      case CF_RGB:
        image.create(idat.height, idat.width, CV_8UC3);
        for (int j = 0; j < idat.height; ++j) {
          int widthbytes = idat.width * 3;
          int rowhead = j * widthbytes;
          unsigned char* row = image.ptr(j);
          for (int k = 0; k < widthbytes; k+=3) {
            row[k+0] = idat.raw_data[rowhead+k+2];
            row[k+1] = idat.raw_data[rowhead+k+1];
            row[k+2] = idat.raw_data[rowhead+k+0];
          }
        }
        break;

      case CF_UNKNOWN:
      default:
        continue;
      }

      m_cv_images.push_back(image);
      snprintf(name, sizeof (name), "image %d", i);
      cv::imshow(name, image);
    }

    if (m_image_save_mode != 0) {
      save_image();
    }
  }

  cv::waitKey(10);
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MultiDisp::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiDisp::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiDisp::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiDisp::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MultiDisp::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

bool MultiDisp::check_image_dir()
{
  struct stat st_buf;

  /* quick return if m_image_dir is empty string */
  if (m_image_dir.size() < 1) {
    fprintf(stderr, "invalid image_dir (%s)\n", m_image_dir.c_str());
    return false;
  }

  /* check the status of dir */
  if (stat(m_image_dir.c_str(), &st_buf) != 0) {
    int err = errno;
    if (err != ENOENT) {
      fprintf(stderr, "invalid image_dir (%s): %s", m_image_dir.c_str(), strerror(err));
      return false;
    }

    /* try to create the directory */
    if (mkdir(m_image_dir.c_str(), S_IRWXU) != 0) {
      perror("mkdir");
      return false;
    }
    stat(m_image_dir.c_str(), &st_buf);
  }

  if ((st_buf.st_mode & (S_IFDIR | S_IWUSR)) != (S_IFDIR | S_IWUSR)) {
    fprintf(stderr, "%s is not a valid directory\n", m_image_dir.c_str());
    return false;
  }

  return true;
}

void MultiDisp::save_image()
{
  struct timeval current;
  char timestamp[16];
  cvmat_seq_t::iterator csi;

  gettimeofday(&current, NULL);
  strftime(timestamp, sizeof(timestamp), "%Y%m%d-%H%M%S", localtime(&current.tv_sec));

  csi = m_cv_images.begin();
  for (unsigned int i = 0; i < m_cv_images.size(); ++i) {
    char buf[512];

    snprintf(buf, sizeof(buf), "%s/img-%s.%06ld_%d.png", m_image_dir.c_str(), timestamp, current.tv_usec, i);
    buf[sizeof(buf)-1] = '\0';

    cv::imwrite(buf, *csi);
    ++csi;
  }
}


extern "C"
{
 
  void MultiDispInit(RTC::Manager* manager)
  {
    coil::Properties profile(multidisp_spec);
    manager->registerFactory(profile,
                             RTC::Create<MultiDisp>,
                             RTC::Delete<MultiDisp>);
  }
  
};


