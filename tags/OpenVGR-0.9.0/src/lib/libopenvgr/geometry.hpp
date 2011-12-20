/* -*- mode: C++; coding: utf-8 -*-
 geometry.hpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#ifndef _GEOMETRY_HPP
#define _GEOMETRY_HPP

#include <cv.h>
#include "calib.h"

namespace ovgr
{
  template <typename T, int N>
  struct Array
  {
    T elem[N];

    typedef T data_type;
    T operator[](size_t i) const { return elem[i]; }
    T& operator[](size_t i) { return elem[i]; }
  };

  struct Point2D
  {
    double x;
    double y;

    Point2D() : x(0), y(0) {}
    Point2D(const double _x, const double _y) : x(_x), y(_y) {}

    template <class T>
    Point2D(const cv::Point_<T> point) : x(point.x), y(point.y) {}

    template <class T>
    operator cv::Point_<T>() const { return cv::Point_<T>(x, y); }
  };

  void calc_crossing_point(double p[3], const double l1[3], const double l2[3]);

  // p2 = H p1, H^T l2 = l1
  cv::Mat calc_homography(const std::vector<Array<double, 3> >& point1, const std::vector<Array<double, 3> >& point2, const std::vector<Array<double, 3> >& line1 = std::vector<Array<double, 3> >(), const std::vector<Array<double, 3> >& line2 = std::vector<Array<double, 3> >());

  cv::Mat calc_essential_matrix(const CameraParam& cp1, const CameraParam& cp2);
  cv::Mat calc_fundamental_matrix(const CameraParam& cp1, const CameraParam& cp2);

  /* l = E x */
  void calc_epipolar_line(double line_coef[3], const cv::Mat& E, const Point2D& point, const bool inv = false);

  /* e: epipole on the first image (cp1) */
  void calc_epipole(double e[3], const CameraParam& cp1, const CameraParam& cp2);

  // x2 = H_inf x1
  cv::Mat calc_infinite_homography(const CameraParam& cp1, const CameraParam& cp2);

  double distance_from_line(const double line_coef[3], const Point2D& point);
  void calc_line_joining_points(double line_coef[3], const double p1[3], const double p2[3]);
}

#endif /* _GEOMETRY_HPP */
