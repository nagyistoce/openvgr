#include <iostream>
#include <cstdio>

#include <cv.h>
#include "capture.hpp"

Capture::Capture ()
  : m_ieee1394b_mode (true),
    m_camera_setting_file ("./ieee1394board.0"),
    m_camera_calib_file ("./calib_camera.yaml")
{
  /* setup camera structures */
  int status;
  status = capture_init (&m_cap, 0);
  if (status != CAPTURE_SUCCESS) {
    throw Capture::Exception ();
  }

  //m_cap.prefer_one_shot = 1;
  m_cap.prefer_bmode = m_ieee1394b_mode;
  m_cap.drop_frames = 1;

  status = capture_setup (&m_cap, m_camera_setting_file.c_str());
  if (status != CAPTURE_SUCCESS) {
    std::cerr << "capture_setup() failed." << std::endl;
    throw Capture::Exception ();
  }

  cv::FileStorage fs;
  int ncamera;
  if (fs.open (m_camera_calib_file, cv::FileStorage::READ) == true) {
      ncamera = fs["num_cameras"];
      //std::cerr << "ncamera: " << ncamera << std::endl;

      /* read camera parameters */
      m_params.resize (ncamera);
      char key[32];
      for (int i = 0; i < ncamera; ++i) {
        snprintf (key, sizeof (key), "camera%d_intr", i);
        cv::read (fs[key], m_params[i].intr);

        snprintf (key, sizeof (key), "camera%d_dist", i);
        cv::read (fs[key], m_params[i].dist);

        snprintf (key, sizeof (key), "camera%d_ext", i);
        cv::read (fs[key], m_params[i].ext);
      }
  }
  else {
    std::cerr << "WARNING: could not open '" << m_camera_calib_file << "'\n" << std::endl;

    ncamera = m_cap.num_active;

    m_params.resize (ncamera);
    for (int i = 0; i < ncamera; ++i) {
      m_params[i].intr = cv::Mat::zeros (3, 3, CV_64F);
      m_params[i].dist = cv::Mat::zeros (5, 1, CV_64F);
      m_params[i].ext  = cv::Mat::eye (4, 4, CV_64F);
    }
  }

  if (ncamera != m_cap.num_active) {
    std::cerr << "numbers of cameras in calibration data and camera setting are different." << std::endl;
    throw Capture::Exception ();
  }

  /* allocate frame memories */
  status = capture_create_frames (&m_cap, &m_frames);
  if (status != CAPTURE_SUCCESS) {
    std::cerr << "capture_create_frames() failed." << std::endl;
    throw Capture::Exception ();
  }
}

Capture::~Capture ()
{
  std::cerr << "called: " << __FUNCTION__ << std::endl;
  capture_destroy_frames (&m_cap, &m_frames);
  capture_final (&m_cap);
}

int
Capture::num_cameras () const
{
  return m_cap.num_active;
}

int
Capture::width (const size_t index) const
{
  return m_frames[index].width;
}

int
Capture::height (const size_t index) const
{
  return m_frames[index].height;
}

capture_frame_t*
Capture::capture ()
{
  capture_multi (&m_cap, m_frames);

  return m_frames;
}
