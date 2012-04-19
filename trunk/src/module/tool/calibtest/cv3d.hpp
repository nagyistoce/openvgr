#ifndef CV3D_HPP
#define CV3D_HPP

#include <cv.h>
#include <highgui.h>

#include "globj.hpp"
#include "capture.hpp"

namespace globj
{
  class DepthMap : virtual public GlObject
  {
    class DefaultDepthFunc
    {
      cv::StereoBM m_sbm;

    public:
      DefaultDepthFunc ()
        : m_sbm (cv::StereoBM::NORMALIZED_RESPONSE, 16 * 10, 21)
      {
        m_sbm.state->trySmallerWindows = 1;
      }

      void operator() (const cv::Mat& left, const cv::Mat& right, cv::Mat& disparity)
      {
        cv::Mat temp;
        m_sbm (left, right, temp);
        temp.convertTo (disparity, CV_32F, 1.0 / 16.0);
      }
    };

  protected:
    cv::Mat m_left_map1, m_left_map2, m_right_map1, m_right_map2, m_Q;
    cv::Mat m_left, m_right, m_depth;

  public:
    DepthMap ()
    {
      hide ();
    }
    DepthMap (const CameraParameter& left_param, const CameraParameter& right_param, const cv::Size& size)
    {
      hide ();
      init (left_param, right_param, size);
    }

    void init (const CameraParameter& left_param, const CameraParameter& right_param, const cv::Size& size);

    cv::Mat compute (const cv::Mat& left, const cv::Mat& right)
    {
      return compute (left, right, DefaultDepthFunc ());
    }

    template <class DepthFunc>
    cv::Mat compute (const cv::Mat& left, const cv::Mat& right, DepthFunc depth_func = cv::StereoBM ());

    void draw_obj ();
  };
}

template <class DepthFunc>
cv::Mat
globj::DepthMap::compute (const cv::Mat& left, const cv::Mat& right, DepthFunc depth_func)
{
  cv::Mat left_rectified, right_rectified;
  cv::Mat disparity;

  m_left = left.clone ();
  m_right = right.clone ();

  cv::remap (left, left_rectified, m_left_map1, m_left_map2, cv::INTER_LINEAR);
  cv::remap (right, right_rectified, m_right_map1, m_right_map2, cv::INTER_LINEAR);

  depth_func (left_rectified, right_rectified, disparity);

  cv::reprojectImageTo3D (disparity, m_depth, m_Q, true);

#if 0
  cv::namedWindow("left", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("right", CV_WINDOW_AUTOSIZE);
  cv::imshow("left", left_rectified);
  cv::imshow("right", right_rectified);
  cv::waitKey(10);
#endif

#if 0
  double minVal, maxVal;
  cv::minMaxLoc (disparity, &minVal, &maxVal);
  //printf("extreme values: %f %f\n", minVal, maxVal);

  cv::namedWindow("disparity", CV_WINDOW_AUTOSIZE);
  cv::Mat img;
  disparity.convertTo (img, CV_8UC1, 255.0 / (maxVal - minVal));
  cv::imshow("disparity", img);
  cv::waitKey(10);
#endif

  show ();

  return m_depth;
}

#endif /* CV3D_HPP */
