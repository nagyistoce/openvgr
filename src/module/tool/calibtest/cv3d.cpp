#include <GL/gl.h>
#include "cv3d.hpp"

using namespace globj;

#if 0
static void
print_mat (const cv::Mat& m)
{
  for (int i = 0; i < m.size ().height; ++i)
    {
      for (int j = 0; j < m.size ().width; ++j)
        {
          printf ("% f ", m.at<double> (i, j));
        }
      printf ("\n");
    }
  printf ("\n");
}
#endif

void
DepthMap::init (const CameraParameter& left_param, const CameraParameter& right_param, const cv::Size& size)
{
  cv::Mat left_R = left_param.ext (cv::Rect (0, 0, 3, 3));
  cv::Mat left_t = left_param.ext (cv::Rect (3, 0, 1, 3));

  cv::Mat right_R = right_param.ext (cv::Rect (0, 0, 3, 3));
  cv::Mat right_t = right_param.ext (cv::Rect (3, 0, 1, 3));

  cv::Mat R = right_R * left_R.t ();
  cv::Mat t = right_t - R * left_t;

  cv::Mat R1, R2, P1, P2;

  //printf ("size: %dx%d\n", size.width, size.height);

#if 0
  print_mat (left_R);
  print_mat (left_t);

  print_mat (right_R);
  print_mat (right_t);

  print_mat (R);
  print_mat (t);
#endif

  cv::stereoRectify (left_param.intr, left_param.dist, right_param.intr, right_param.dist, size, R, t, R1, R2, P1, P2, m_Q, 0);

#if !defined(CV_MAJOR_VERSION) || (CV_MAJOR_VERSION < 2) || (CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION < 4)
  /* As of OpenCV-2.0, the matrix Q contains wrong value.. */
  m_Q.at<double>(3, 2) *= -1.0;
#endif

#if 0
  print_mat (R1);
  print_mat (R2);

  print_mat (P1);
  print_mat (P2);

  print_mat (m_Q);
#endif

  cv::initUndistortRectifyMap (left_param.intr, left_param.dist,
                               R1, P1, size, CV_16SC2, m_left_map1, m_left_map2);

  cv::initUndistortRectifyMap (right_param.intr, right_param.dist,
                               R2, P2, size, CV_16SC2, m_right_map1, m_right_map2);
}

void
DepthMap::draw_obj ()
{
  cv::Size size = m_depth.size ();

  //printf ("size: %dx%d\n", size.width, size.height);

  glBegin (GL_POINTS);
  for (int h = 0; h < size.height; ++h)
    {
      cv::Vec3f *row = m_depth.ptr<cv::Vec3f> (h);
      char *img = m_left.ptr<char> (h);

      for (int w = 0; w < size.width; ++w)
        {
          if (row[w][2] < 0.0)
            {
              continue;
            }

          glColor3f ((double)img[w] / 255.0, (double)img[w] / 255.0, (double)img[w] / 255.0);
          glVertex3f (row[w][0], row[w][1], row[w][2]);
        }
    }
  glEnd ();
}
