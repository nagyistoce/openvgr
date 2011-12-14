/* -*- mode: C++; coding: utf-8 -*-
 geometry.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <cstdio>
#include <cv.h>

#include "constants.hpp"
#include "geometry.hpp"
#include "calib.h"

#include "local.h"

void
ovgr::calc_crossing_point(double p[3], const double l1[3], const double l2[3])
{
  s_cross3(p, l1, l2);
  s_normalize(p);
}

cv::Mat
ovgr::calc_homography(const std::vector<Array<double, 3> >& point1,
                      const std::vector<Array<double, 3> >& point2,
                      const std::vector<Array<double, 3> >& line1,
                      const std::vector<Array<double, 3> >& line2)
{
  const size_t npoint = point1.size();
  const size_t nline = line1.size();

  if (npoint > point2.size() || nline > line2.size() || npoint + nline < 4)
    {
      return cv::Mat::eye(3, 3, CV_64FC1);
    }

  cv::Mat M = cv::Mat::zeros(3 * (npoint + nline), 9, CV_64FC1);

  // 対応点
  for (size_t i = 0; i < npoint; ++i)
    {
      for (size_t j = 0; j < 3; ++j)
        {
          M.at<double>(3*i  , 3*1 + j) -= point2[i][2] * point1[i][j];
          M.at<double>(3*i  , 3*2 + j) += point2[i][1] * point1[i][j];

          M.at<double>(3*i+1, 3*0 + j) += point2[i][2] * point1[i][j];
          M.at<double>(3*i+1, 3*2 + j) -= point2[i][0] * point1[i][j];

          M.at<double>(3*i+2, 3*0 + j) -= point2[i][1] * point1[i][j];
          M.at<double>(3*i+2, 3*1 + j) += point2[i][0] * point1[i][j];
        }
    }

  // 対応線
  for (size_t i = 0; i < nline; ++i)
    {
      for (size_t j = 0; j < 3; ++j)
        {
          M.at<double>(3*(npoint + i)  , 3*j + 1) += line1[i][2] * line2[i][j];
          M.at<double>(3*(npoint + i)  , 3*j + 2) -= line1[i][1] * line2[i][j];

          M.at<double>(3*(npoint + i)+1, 3*j + 0) -= line1[i][2] * line2[i][j];
          M.at<double>(3*(npoint + i)+1, 3*j + 2) += line1[i][0] * line2[i][j];

          M.at<double>(3*(npoint + i)+2, 3*j + 0) += line1[i][1] * line2[i][j];
          M.at<double>(3*(npoint + i)+2, 3*j + 1) -= line1[i][0] * line2[i][j];
        }
    }

  for (int i = 0; i < M.rows; ++i)
    {
      for (int j = 0; j < M.cols; ++j)
        {
          printf("% 10.3g ", M.at<double>(i, j));
        }
      printf("\n");
    }
  printf("\n\n");

  cv::SVD svd(M, cv::SVD::MODIFY_A | (M.rows < 9 ? cv::SVD::FULL_UV : 0));
  cv::Mat H(3, 3, CV_64FC1);

  for (size_t i = 0; i < 3; ++i)
    {
      for (size_t j = 0; j < 3; ++j)
        {
          const double elem = svd.vt.at<double>(8, 3*i + j);
          H.at<double>(i, j) = elem;
        }
    }
  double det = cv::determinant(H);

  for (size_t i = 0; i < 9; ++i)
    {
      printf("% 10.3g ", svd.w.at<double>(i, 0));
    }
  printf("\n\n");

#if 1
  H /= cbrt(det);
#else
  for (size_t i = 0; i < 3; ++i)
    {
      for (size_t j = 0; j < 3; ++j)
        {
          H.at<double>(i, j) /= cbrt(det);
        }
    }
#endif

  return H;
}

cv::Mat
ovgr::calc_essential_matrix(const CameraParam& cp1, const CameraParam& cp2)
{
  double tx[3][3];
  const cv::Mat cvR1(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp1.Rotation));
  const cv::Mat cvR2(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp2.Rotation));
  const cv::Mat cvt1(3, 1, CV_64FC1, const_cast<double*>(cp1.Translation));
  const cv::Mat cvt2(3, 1, CV_64FC1, const_cast<double*>(cp2.Translation));
  cv::Mat cvRR(3, 3, CV_64FC1), cvt(3, 1, CV_64FC1);

  cvRR = cvR2 * cvR1.t();
  cvt = cvt2 - cvRR * cvt1;

  tx[0][0] = 0.0;
  tx[0][1] = -cvt.at<double>(2, 0);
  tx[0][2] =  cvt.at<double>(1, 0);

  tx[1][0] = -tx[0][1];
  tx[1][1] = 0.0;
  tx[1][2] = -cvt.at<double>(0, 0);

  tx[2][0] = -tx[0][2];
  tx[2][1] = -tx[1][2];
  tx[2][2] = 0.0;

  return cv::Mat(3, 3, CV_64FC1, tx) * cvRR;
}

cv::Mat
ovgr::calc_fundamental_matrix(const CameraParam& cp1, const CameraParam& cp2)
{
  cv::Mat E = calc_essential_matrix(cp1, cp2);
  const cv::Mat A1(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp1.intrinsicMatrix));
  const cv::Mat A2(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp2.intrinsicMatrix));

  return A2.t().inv() * E * A1.inv();
}

/* l = E x */
void
ovgr::calc_epipolar_line(double line_coef[3], const cv::Mat& E, const Point2D& point, const bool inv)
{
  double vec[3] = {point.x, point.y, 1.0};
  cv::Mat p(3, 1, CV_64FC1, vec), l(3, 1, CV_64FC1);

  if (inv == false)
    {
      l = E * p;
    }
  else
    {
      l = E.t() * p;
    }

  double norm = sqrt(pow(l.at<double>(0, 0), 2) + pow(l.at<double>(1, 0), 2));
  if (norm > epsilon)
    {
      for (int i = 0; i < 3; ++i)
        {
          line_coef[i] = l.at<double>(i, 0) / norm;
        }
    }
  else
    {
      /* 無限遠の直線 */
      line_coef[0] = line_coef[1] = 0.0;
      line_coef[2] = 1.0;
    }
}

/* e: epipole on the first image (cp1) */
void
ovgr::calc_epipole(double e[3], const CameraParam& cp1, const CameraParam& cp2)
{
  const cv::Mat A(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp1.intrinsicMatrix));
  const cv::Mat R1(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp1.Rotation));
  const cv::Mat t1(3, 1, CV_64FC1, const_cast<double*>(cp1.Translation));
  const cv::Mat R2(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp2.Rotation));
  const cv::Mat t2(3, 1, CV_64FC1, const_cast<double*>(cp2.Translation));

  cv::Mat epipole = A * (-R1 * (R2.t() * t2) + t1);
  double norm = sqrt(epipole.dot(epipole));

  for (int i = 0; i < 3; ++i)
    {
      e[i] = epipole.at<double>(i, 0) / norm;
    }
}

// x2 = H_inf x1
cv::Mat
ovgr::calc_infinite_homography(const CameraParam& cp1, const CameraParam& cp2)
{
  const cv::Mat A1(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp1.intrinsicMatrix));
  const cv::Mat R1(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp1.Rotation));
  const cv::Mat A2(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp2.intrinsicMatrix));
  const cv::Mat R2(3, 3, CV_64FC1, const_cast<double(*)[3]>(cp2.Rotation));

  cv::Mat R = R2 * R1.t();
  return A2 * R * A1.inv();
}

double
ovgr::distance_from_line(const double line_coef[3], const Point2D& point)
{
  double norm = sqrt(line_coef[0]*line_coef[0] + line_coef[1]*line_coef[1]);

  if (norm < epsilon)
    {
      return HUGE_VAL;
    }

  return (line_coef[0] * point.x + line_coef[1] * point.y + line_coef[2]) / norm;
}

void
ovgr::calc_line_joining_points(double line_coef[3], const double p1[3], const double p2[3])
{
  for (int i = 0; i < 3; ++i)
    {
      line_coef[i] = p1[(i+1) % 3] * p2[(i+2) % 3] - p1[(i+2) % 3] * p2[(i+1) % 3];
    }

  double norm = sqrt(line_coef[0]*line_coef[0] + line_coef[1]*line_coef[1]);

  if (norm > epsilon)
    {
      for (int i = 0; i < 3; ++i)
        {
          line_coef[i] /= norm;
        }
    }
  else
    {
      line_coef[0] = line_coef[1] = 0.0;
      line_coef[2] = 1.0;
    }
}
