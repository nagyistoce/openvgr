#ifndef CAPTURE_HPP
#define CAPTURE_HPP

#include <vector>
#include <string>

#include <cv.h>
#include <capture.h>

struct CameraParameter
{
  cv::Mat intr;
  cv::Mat dist;
  cv::Mat ext;
};

class Capture
{
  capture_t m_cap;
  capture_frame_t* m_frames;

  bool m_ieee1394b_mode;
  std::string m_camera_setting_file;
  std::string m_camera_calib_file;

public:
  typedef std::vector<CameraParameter> CameraParameter_set;
  CameraParameter_set m_params;

  class Exception
  {
    std::string m_message;

  public:
    Exception (const std::string& msg = "no reason") : m_message (msg) {}
    const std::string& message() const { return m_message; }
  };

  Capture ();
  virtual ~Capture ();

  int num_cameras () const;
  int width (const size_t index = 0) const;
  int height (const size_t index = 0) const;

  capture_frame_t* capture ();
};

#endif /* CAPTURE_HPP */
