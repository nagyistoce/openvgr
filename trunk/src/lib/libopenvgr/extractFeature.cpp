/* -*- coding: utf-8 -*-
 extractFeature.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <list>

#include <cv.h>
#include <highgui.h>

#include "parameters.h"
#include "match3Dfeature.h"
#include "conic.h"
#include "extractFeature_old.h"
#include "extractEdge.h"

#include "extractFeature.hpp"
#include "correspondence.hpp"
using namespace ovgr;

typedef std::vector<cv::Point> Points;
typedef std::vector<Points> PointSet;
typedef std::vector<bool> support_index_t;

template <class LF>
static void
draw_LineFeatures(cv::Mat& img, const LF& lf)
{
  cv::namedWindow("detected lines", CV_WINDOW_AUTOSIZE);

  for (typename LF::const_iterator lfi = lf.begin(); lfi != lf.end(); ++lfi)
    {
      cv::line(img, cv::Point(lfi->start.x, lfi->start.y), cv::Point(lfi->end.x, lfi->end.y), CV_RGB(0, 0, 255));
      cv::circle(img, cv::Point(lfi->start.x, lfi->start.y), 1, CV_RGB(0, 255, 0), -1);
      cv::circle(img, cv::Point(lfi->end.x, lfi->end.y), 1, CV_RGB(255, 0, 0), -1);
    }

  cv::imshow("detected lines", img);
  cv::waitKey(-1);
}

static int
intersectLineSegment2(const double coef1[3], const double coef2[3], double out[2])
{
  double p[3];

  for (int i = 0; i < 3; ++i)
    {
      p[i] = coef1[(i+1) % 3] * coef2[(i+2) % 3] - coef1[(i+2) % 3] * coef2[(i+1) % 3];
    }

  // 平行な場合
  if (fabs(p[2]) <= sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]) * VISION_EPS)
    {
      return 0;
    }

  out[0] = p[0] / p[2];
  out[1] = p[1] / p[2];

  return 1;
}

template <class T, class U>
inline static double
distance_points(const cv::Point_<T>& p1, const cv::Point_<U>& p2)
{
  return sqrt(pow((double)p1.x - (double)p2.x, 2)
              + pow((double)p1.y - (double)p2.y, 2));
}

inline static double
angle_of_lines(const double coef1[3], const double coef2[3])
{
  double dot = coef1[0]*coef2[0] + coef1[1]*coef2[1];
  double norm[2] = {sqrt(coef1[0]*coef1[0] + coef1[1]*coef1[1]),
                    sqrt(coef2[0]*coef2[0] + coef2[1]*coef2[1])};

  return acos(dot / norm[0] / norm[1]);
}

// 楕円の係数から描画用外接矩形を求める
cv::RotatedRect
ovgr::calc_ellipse_rr(const double coeff[6])
{
  double ev[3], axis[2], t[2], det;
  double D = sqrt((coeff[0] - coeff[2]) * (coeff[0] - coeff[2]) + coeff[1] * coeff[1]);

  det = 4.0*coeff[0]*coeff[2] - coeff[1]*coeff[1];
  if (fabs(det) < 1.0e-12)
    {
      return cv::RotatedRect();
    }

  t[0] = (-2.0 * coeff[2] * coeff[3] +       coeff[1] * coeff[4]) / det;
  t[1] = (       coeff[1] * coeff[3] - 2.0 * coeff[0] * coeff[4]) / det;

  //printf("% f % f\n% f % f\n", coeff[0], coeff[1]/2.0, coeff[1]/2.0, coeff[2]);

  if (coeff[0] + coeff[2] >= 0.0)
    {
      ev[0] = (coeff[0] + coeff[2] + D) / 2.0;
      ev[1] = det / 4.0 / ev[0];
      ev[2] = coeff[0] * t[0]*t[0] + coeff[1] * t[0]*t[1] + coeff[2] * t[1]*t[1] + coeff[3] * t[0] + coeff[4] * t[1] + coeff[5];

      //axis[0] = coeff[1];
      //axis[1] = -(coeff[0] - coeff[2]) - D;
      axis[0] = (coeff[0] - coeff[2]) + D;
      axis[1] = coeff[1];
    }
  else
    {
      ev[0] = -((coeff[0] + coeff[2] - D) / 2.0);
      ev[1] = det / 4.0 / ev[0];
      ev[2] = -(coeff[0] * t[0]*t[0] + coeff[1] * t[0]*t[1] + coeff[2] * t[1]*t[1] + coeff[3] * t[0] + coeff[4] * t[1] + coeff[5]);

      //axis[0] = -coeff[1];
      //axis[1] = (coeff[0] - coeff[2]) - D;
      axis[0] = -(coeff[0] - coeff[2]) + D;
      axis[1] = -coeff[1];
    }

#if 0
  {
    double norm = sqrt(axis[0]*axis[0] + axis[1]*axis[1]);
    if (norm > 1.0e-24)
      {
        axis[0] /= norm;
        axis[1] /= norm;
      }
  }
#endif

#if 0
  printf("t: % f % f\n", t[0], t[1]);
  printf("ev: %f %f %f\n", ev[0], ev[1], ev[2]);
  printf("axis: % f % f\n", axis[0], axis[1]);
  printf("\n");
#endif

  double theta = atan2(axis[1], axis[0]) * 180.0 / M_PI;
  if (theta < 0.0)
    {
      theta += 180.0;
    }

  return cv::RotatedRect(cv::Point2f(t[0], t[1]),
                         cv::Size2f(2.0 / sqrt(ev[0] / fabs(ev[2])), 2.0 / sqrt(ev[1] / fabs(ev[2]))),
                         -theta);
}

/* [0, max_index)の範囲でn個の値をランダムに選ぶ */
static void
choose_index(size_t *index, const size_t n, const size_t max_index)
{
  if (n > max_index)
    {
      return;
    }

  if (n == max_index)
    {
      for (size_t i = 0; i < n; ++i)
        {
          index[i] = i;
        }
      return;
    }

  size_t curr = 0;
  do
    {
      index[curr] = ((double)rand() * (double)max_index) / ((double)RAND_MAX + 1.0);

      size_t i;
      for (i = 0; i < curr; ++i)
        {
          if (index[curr] == index[i])
            {
              break;
            }
        }
      if (i == curr)
        {
          ++curr;
        }
    }
  while (curr < n);
}

/* 2点を通る直線を求める */
static void
compute_line_coef(double coef[3], const cv::Point& p1, const cv::Point& p2)
{
  double norm = 0.0;
  int i;

  coef[0] = p1.y - p2.y;
  coef[1] = p2.x - p1.x;
  coef[2] = p1.x * p2.y - p1.y * p2.x;

  norm = sqrt(coef[0]*coef[0] + coef[1]*coef[1]);

  for (i = 0; i < 3; ++i)
    {
      coef[i] /= norm;
    }
}

/* 点群によく当てはまる直線を求める */
static bool
search_line_ransac(double coef[3], support_index_t* support, const Points& points, const int num_trial = 100, const size_t min_points = 10, const double thres = 2.0)
{
  int num_support = 0;
  bool found = false;

  for (int i = 0; i < 3; ++i)
    {
      coef[i] = 0.0;
    }

  //printf("length: %4u\n", points.size());
  for (int trial = 0; trial < num_trial; ++trial)
    {
      size_t index[2];
      double coef_curr[3];
      support_index_t support_curr(points.size());
      int i, num;

      /* 2点を適当にとってくる */
      choose_index(index, 2, points.size());
      //printf("index[%4u]: %3d %3d\n", psi->size(), index[0], index[1]);

      /* 直線の係数を求める */
      compute_line_coef(coef_curr, points[index[0]], points[index[1]]);

      /* 距離が閾値未満の点(=サポートする点)を数える */
      num = 0;
      for (i = 0; i < (int)points.size(); ++i)
        {
          double distance = 0.0;

          distance = fabs(coef_curr[0] * points[i].x + coef_curr[1] * points[i].y + coef_curr[2]);
          if (distance < thres)
            {
              support_curr[i] = true;
              ++num;
            }
          else
            {
              support_curr[i] = false;
            }
        }

      /* サポート点が多ければ更新 */
      if (num > num_support)
        {
          num_support = num;
          *support = support_curr;

          for (i = 0; i < 3; ++i)
            {
              coef[i] = coef_curr[i];
            }
        }
    }

  /* サポート点の総数が閾値より大きければ成功 */
  if (num_support > (int)min_points)
    {
      //printf("num: %d\n", num_support);
      //printf("coef: % g % g % g\n", coef[0], coef[1], coef[2]);

      found = true;
    }

  return found;
}

/* サポートする点列で閾値以上のものを取り出す */
template <class T>
static std::vector<T>
extract_supported_points(Feature2D* feature, const Points& points, support_index_t& support, const size_t min_points = 10)
{
  std::vector<T> extracted_points;
  const size_t length = support.size();

  size_t start = 0;
  /* 最初の切れ目を探す */
  while (start < length && support[start] == true)
    {
      ++start;
    }
  if (start < length)
    {
      size_t i = (start + 1) % length;
      while (i != start)
        {
          /* サポートする始めの点まで飛ばす */
          while (i != start && support[i] == false)
            {
              i = (i+1) % length;
            }

          /* 連続するサポート点を数える */
          size_t num = 0;
          while (i + num < length + start && support[(i + num) % length] == true)
            {
              ++num;
            }

          /* サポート点の数が閾値以上なら点列に加える */
          if (num >= min_points)
            {
              Segment2D segment;
              for (size_t j = 0; j < num; ++j)
                {
                  const size_t index = (i + j) % length;
                  T point(cv::Point_<typename T::value_type>(points[index].x, points[index].y));

                  extracted_points.push_back(point);
                  segment.push_back(points[index]);
                }
              feature->segment.push_back(segment);
            }
          /* 閾値未満ならサポート点とみなさない */
          else
            {
              for (size_t j = 0; j < num; ++j)
                {
                  support[(i + j) % length] = false;
                }
            }

          i = (i + num) % length;
        }
    }
  else /* 1周全部サポート点のとき */
    {
      Segment2D segment;

      extracted_points.resize(length);
      for (size_t i = 0; i < length; ++i)
        {
          T point(cv::Point_<typename T::value_type>(points[i].x, points[i].y));
          extracted_points[i] = point;
          segment.push_back(points[i]);
        }
      feature->segment.push_back(segment);
    }

  return extracted_points;
}

/* 当てはまった直線の係数を最適化し、ほかのパラメータも決める */
static bool
create_LineFeature(LineFeature2D* feature, const Points& points, support_index_t& support, const size_t min_points = 10)
{
  std::vector<cv::Point3f> points_for_fitting;
  const size_t length = support.size();

  if (length < min_points)
    {
      for (size_t i = 0; i < length; ++i)
        {
          support[i] = false;
        }
      feature->coef[0] = feature->coef[1] = feature->coef[2] = 0.0;
      return false;
    }

  points_for_fitting = extract_supported_points<cv::Point3f>(feature, points, support, min_points);

  if (points_for_fitting.size() < min_points)
    {
      feature->coef[0] = feature->coef[1] = feature->coef[2] = 0.0;
      return false;
    }

  /* 抽出した点列に当てはまる直線を求める */
  cv::Vec6f line;
  cv::fitLine(cv::Mat(points_for_fitting), line, CV_DIST_L2, 0, 0.01, 0.01);
  feature->coef[0] = -line[1];
  feature->coef[1] =  line[0];
  feature->coef[2] = -(feature->coef[0] * line[3] + feature->coef[1] * line[4]);

  /* 誤差 */
  feature->error = 0.0;
  for (size_t i = 0; i < points_for_fitting.size(); ++i)
    {
      const cv::Point3f& p = points_for_fitting[i];
      feature->error += fabs(feature->coef[0] * p.x + feature->coef[1] * p.y + feature->coef[2]);
    }
  feature->error /= static_cast<double>(points_for_fitting.size());

  /* 当てはめた線分の始点・終点を求める */
  double l0 = line[0] * feature->segment[0][0].x + line[1] * feature->segment[0][1].y;
  double l[2] = {l0, l0};

  feature->start = feature->segment[0][0];
  feature->end = feature->segment[0][0];
  for (size_t i = 0; i < feature->segment.size(); ++i)
    {
      Segment2D& s = feature->segment[i];
      for (size_t j = 0; j < s.size(); ++j)
        {
          double lj = line[0] * s[j].x + line[1] * s[j].y;

          if (lj < l[0])
            {
              feature->start = s[j];
              l[0] = lj;
            }
          if (l[1] < lj)
            {
              feature->end = s[j];
              l[1] = lj;
            }
        }
    }

  double lo = line[0] * line[3] + line[1] * line[4];
  feature->start.x = (l[0] - lo)*line[0] + line[3];
  feature->start.y = (l[0] - lo)*line[1] + line[4];

  feature->end.x = (l[1] - lo)*line[0] + line[3];
  feature->end.y = (l[1] - lo)*line[1] + line[4];

  feature->length = sqrt(pow(feature->end.x - feature->start.x, 2)
                         + pow(feature->end.y - feature->start.y, 2));

  return true;
}

/* 点列の集合から直線を抽出する */
static std::list<LineFeature2D>
extract_lines(PointSet& contours, const double thres = 1.0, const size_t min_points = 10, const int num_trial = 50)
{
  std::list<LineFeature2D> line_features;

  for (size_t ci = 0; ci < contours.size(); ++ci)
    {
      bool found = false;

      if (contours[ci].size() < min_points)
        {
          continue;
        }

      support_index_t support(contours[ci].size());
      double coef[3];
      LineFeature2D feature;

      /* よく当てはまる直線を探す */
      found = search_line_ransac(coef, &support, contours[ci], num_trial, min_points, thres);

      /* 直線特徴を作成して追加する */
      if (found == true && create_LineFeature(&feature, contours[ci], support, min_points) == true)
        {
          //printf("coef_rev: % g % g % g\n", feature.coef[0], feature.coef[1], feature.coef[2]);
          line_features.push_back(feature);

          /* 当てはまらなかった点列を抽出して戻す */
          size_t index = 0;
          while (index < contours[ci].size())
            {
              while (index < contours[ci].size() && support[index] == true)
                {
                  ++index;
                }

              Points points;
              while (index < contours[ci].size() && support[index] == false)
                {
                  points.push_back(contours[ci][index]);
                  ++index;
                }

              if (points.size() >= min_points)
                {
                  contours.push_back(points);
                }
            }
        }
    }

  return line_features;
}

/* 線分特徴から頂点特徴を生成する */
static std::list<VertexFeature>
construct_vertex_features(const std::list<LineFeature2D>& lf)
{
  const double length_ratio = 0.5;
  const double angle_thres = 30.0 * M_PI / 180;

  std::list<VertexFeature> vf;

  for (std::list<LineFeature2D>::const_iterator i = lf.begin(); i != lf.end(); ++i)
    {
      std::list<LineFeature2D>::const_iterator j = i;
      for (++j; j != lf.end(); ++j)
        {
          double angle = angle_of_lines(i->coef, j->coef);

          if (fabs(angle - M_PI / 2.0) > M_PI/2.0 - angle_thres)
            {
              continue;
            }

          double cross_pos[2];
          if (intersectLineSegment2(i->coef, j->coef, cross_pos) == 0)
            {
              continue;
            }

          double distance[2][2];
          cv::Point2d cp(cross_pos[0], cross_pos[1]);
          int min_index[2];

          distance[0][0] = distance_points((cv::Point2d)i->start, cp);
          distance[0][1] = distance_points((cv::Point2d)i->end, cp);

          distance[1][0] = distance_points((cv::Point2d)j->start, cp);
          distance[1][1] = distance_points((cv::Point2d)j->end, cp);

          for (int k = 0; k < 2; ++k)
            {
              min_index[k] = distance[k][0] < distance[k][1] ? 0 : 1;
            }

          if (distance[0][min_index[0]] > i->length * length_ratio
              || distance[1][min_index[1]] > j->length * length_ratio)
            {
              continue;
            }

          VertexFeature feature;

          feature.segment.resize(i->segment.size() + j->segment.size());
          for (size_t k = 0; k < i->segment.size(); ++k)
            {
              feature.segment[k] = i->segment[k];
            }
          for (size_t k = 0; k < j->segment.size(); ++k)
            {
              feature.segment[k + i->segment.size()] = j->segment[k];
            }
          feature.error = (i->error * (double)i->segment.size() + j->error * (double)j->segment.size()) / (double)feature.segment.size();

          for (int k = 0; k < 3; ++k)
            {
              for (int l = 0; l <= k; ++l)
                {
                  feature.coef[k*(k+1)/2 + l] = i->coef[k] * j->coef[l] + i->coef[l] * j->coef[k];
                }
              feature.line_coef[0][k] = i->coef[k];
              feature.line_coef[1][k] = j->coef[k];
            }

          feature.length[0] = distance[0][1 - min_index[0]];
          feature.length[1] = distance[1][1 - min_index[1]];

          feature.start = (min_index[0] == 1 ? i->start : i->end);
          feature.mid   = cp;
          feature.end   = (min_index[1] == 1 ? j->start : j->end);

          vf.push_back(feature);
        }
    }

  return vf;
}

/* 楕円の係数を求める */
static double
compute_ellipse_coef(double coef[6], const Points& points)
{
  const double min_width = 10.0;
  double error = 0.0;

  cv::RotatedRect rr = fitEllipse(cv::Mat(points));

  if (rr.size.width < min_width || rr.size.height < min_width)
    {
      return -1.0;
    }

  /* 外接矩形から楕円の係数を計算 */
  double angle = rr.angle;

  double a2 = rr.size.width * rr.size.width / 4.0;
  double b2 = rr.size.height * rr.size.height / 4.0;
  double center[2] = {rr.center.x, rr.center.y};
  double c = cos(angle * M_PI / 180.0), s = -sin(angle * M_PI / 180.0);
  double c2 = c*c, s2 = 1.0 - c2;

  coef[0] = c2/a2 + s2/b2;
  coef[1] = 2.0* c*s * (b2 - a2) / a2 / b2;
  coef[2] = s2/a2 + c2/b2;
  coef[3] = -2.0 * coef[0]*center[0] - coef[1]*center[1];
  coef[4] = -coef[1]*center[0] - 2.0 * coef[2]*center[1];
  coef[5] = coef[0] * center[0]*center[0] + coef[1] * center[0]*center[1] + coef[2] * center[1]*center[1] - 1.0;

#if 0
  if (rr.size.width > 0 && rr.size.height > 0)
    {
      printf("*** ellipse ***\n");
      printf("rr: %f x %f, % f % f, theta: % f\n", rr.size.width, rr.size.height, rr.center.x, rr.center.y, rr.angle);
      printf("a: % f, b: % f\n", sqrt(a2), sqrt(b2));
      printf("center: % f % f\n", center[0], center[1]);
      s_ellipse_prop(coef);
    }
#endif

  return error;
}

/* 点群によく当てはまる楕円を探す */
static bool
search_ellipse_ransac(double coef[6], support_index_t *support, const Points& points, const int num_trial = 100, const size_t min_points = 10, const double thres = 2.0)
#if 0
{
  int num_support = 0;
  bool found = false;

  for (int i = 0; i < 6; ++i)
    {
      coef[i] = 0.0;
    }

  for (int trial = 0; trial < num_trial; ++trial)
    {
      const int num_candidate = 5;
      int index[num_candidate];
      double coef_curr[6];
      support_index_t support_curr(points.size());
      int i, num;

      /* ランダムに点を選ぶ */
      Points candidate(num_candidate);
      choose_index(index, num_candidate, (int)(points.size()));
      //printf("index[%4u]: %3d %3d\n", psi->size(), index[0], index[1]);

      for (int i = 0; i < num_candidate; ++i)
        {
          candidate[i] = points[index[i]];
        }

      /* 楕円の係数を求める */
      if (compute_ellipse_coef(coef_curr, candidate) < 0)
        {
          continue;
        }

      /* 距離が閾値より小さい点の数を数える */
      num = 0;
      for (i = 0; i < (int)points.size(); ++i)
        {
          double distance = 0.0;
          int point[2] = {points[i].x, points[i].y};

          distance = distanceConic(coef_curr, point);
          if (distance < thres)
            {
              support_curr[i] = true;
              ++num;
            }
          else
            {
              support_curr[i] = false;
            }
        }

      /* サポート点が増えれば更新 */
      if (num > num_support)
        {
          num_support = num;
          *support = support_curr;

          for (i = 0; i < 6; ++i)
            {
              coef[i] = coef_curr[i];
            }
        }
    }

  /* サポート点の総数が閾値を越えれば成功 */
  if (num_support > (int)min_points)
    {
      //printf("num: %d\n", num_support);
      //printf("coef: % g % g % g % g % g % g\n", coef[0], coef[1], coef[2], coef[3], coef[4], coef[5]);

      found = true;
    }

  return found;
}
#else
{
  int num_support = 0;
  bool found = false;

  for (int i = 0; i < 6; ++i)
    {
      coef[i] = 0.0;
    }

  for (int trial = 0; trial < num_trial; ++trial)
    {
      size_t index[2], num_points = 0;
      double coef_curr[6];
      support_index_t support_curr(points.size());
      int i, num;

      /* ランダムに2点を選ぶ */
      do
        {
          choose_index(index, 2, points.size());
          //printf("index[%4u]: %3d %3d\n", psi->size(), index[0], index[1]);

          if (index[0] < index[1])
            {
              num_points = index[1] - index[0] + 1;
            }
          else
            {
              num_points = points.size() - (index[0] - index[1] + 1) + 2;
            }
        }
      while (num_points < 6); /* required for execution of cv::fitEllipse() */

      Points candidate(num_points);
      if (index[0] < index[1])
        {
          for (size_t i = 0; i < candidate.size(); ++i)
            {
              candidate[i] = points[index[0] + i];
            }
        }
      else
        {
          size_t n = 0;
          for (size_t i = index[0]; i < points.size(); ++i)
            {
              candidate[n++] = cv::Point(0, i);
            }
          for (size_t i = 0; i <= index[1]; ++i)
            {
              candidate[n++] = points[i];
            }
          continue;
        }

      /* 楕円の係数を求める */
      if (compute_ellipse_coef(coef_curr, candidate) < 0)
        {
          continue;
        }

      /* 距離が閾値より小さい点の数を数える */
      num = 0;
      for (i = 0; i < (int)points.size(); ++i)
        {
          double distance = 0.0;
          int point[2] = {points[i].x, points[i].y};

          distance = distanceConic(coef_curr, point);
          if (distance < thres)
            {
              support_curr[i] = true;
              ++num;
            }
          else
            {
              support_curr[i] = false;
            }
        }

      /* サポート点が増えれば更新 */
      if (num > num_support)
        {
          num_support = num;
          *support = support_curr;

          for (i = 0; i < 6; ++i)
            {
              coef[i] = coef_curr[i];
            }
        }
    }

  /* サポート点の総数が閾値を越えれば成功 */
  if (num_support > (int)min_points)
    {
      //printf("num: %d\n", num_support);
      //printf("coef: % g % g % g % g % g % g\n", coef[0], coef[1], coef[2], coef[3], coef[4], coef[5]);

      found = true;
    }

  return found;
}
#endif

/* 楕円の係数を最適化し、その他のパラメータも求める */
static bool
create_EllipseFeature(EllipseFeature* feature, const Points& points, support_index_t& support, const size_t min_points = 10)
{
  Points points_for_fitting;
  const size_t length = support.size();

  if (length < min_points)
    {
      feature->coef[0] = feature->coef[1] = feature->coef[2]
        = feature->coef[3] = feature->coef[4] = feature->coef[5] = 0.0;
      return false;
    }

  points_for_fitting = extract_supported_points<cv::Point>(feature, points, support, min_points);

  if (points_for_fitting.size() < min_points)
    {
      feature->coef[0] = feature->coef[1] = feature->coef[2]
        = feature->coef[3] = feature->coef[4] = feature->coef[5] = 0.0;
      return false;
    }

  compute_ellipse_coef(feature->coef, points_for_fitting);

  /* 誤差 */
  feature->error = 0.0;
  for (size_t i = 0; i < points_for_fitting.size(); ++i)
    {
      int point[2] = {points_for_fitting[i].x, points_for_fitting[i].y};
      feature->error += distanceConic(feature->coef, point);
    }
  feature->error /= static_cast<double>(points_for_fitting.size());

  /* その他のパラメータ */
  cv::RotatedRect rr = calc_ellipse_rr(feature->coef);
  feature->center[0] = rr.center.x;
  feature->center[1] = rr.center.y;
  feature->axis[0] = rr.size.width / 2.0;
  feature->axis[1] = rr.size.height / 2.0;
  feature->theta = -rr.angle * M_PI / 180.0; /* OpenCV-2.0描画関数の仕様で符号が反転 */

  return true;
}

/* 点列の集合によく当てはまる楕円を求める */
static std::list<EllipseFeature>
extract_ellipses(PointSet& contours, const double thres = 1.5, const size_t min_points = 10, const int num_trial = 100)
{
  std::list<EllipseFeature> ellipse_features;

  for (size_t ci = 0; ci < contours.size(); ++ci)
    {
      bool found = false;

      if (contours[ci].size() < min_points)
        {
          continue;
        }

      support_index_t support(contours[ci].size());
      double coef[6];
      EllipseFeature feature;

      /* 良く当てはまる楕円を探す */
      found = search_ellipse_ransac(coef, &support, contours[ci], num_trial, min_points, thres);

      /* 楕円特徴を生成して追加 */
      if (found == true && create_EllipseFeature(&feature, contours[ci], support, min_points) == true)
        {
          //printf("coef_rev: % g % g % g % g % g % g\n", feature.coef[0], feature.coef[1], feature.coef[2], feature.coef[3], feature.coef[4], feature.coef[5]);
          ellipse_features.push_back(feature);

          /* 当てはまらなかった点列を抽出して戻す */
          size_t index = 0;
          while (index < contours[ci].size())
            {
              while (index < contours[ci].size() && support[index] == true)
                {
                  ++index;
                }

              Points points;
              while (index < contours[ci].size() && support[index] == false)
                {
                  points.push_back(contours[ci][index]);
                  ++index;
                }

              if (points.size() >= min_points)
                {
                  contours.push_back(points);
                }
            }
        }
    }

  return ellipse_features;
}

/* エッジ画像から頂点特徴と楕円特徴を抽出する */
Features2D
ovgr::extractFeatures(const unsigned char* edge,   // エッジ画像
                      const Parameters& parameters, // 全パラメータ
                      const Features3D& model)      // モデルの３次元特徴データ
{
  Features2D features;

  const double line_thres = parameters.feature2D.maxErrorofLineFit;
  const double ellipse_thres = parameters.feature2D.maxErrorofConicFit;
  const int min_points = parameters.feature2D.minFragment;
  const int no_search = parameters.feature2D.no_search_features;

  const int colsize = parameters.colsize;
  const int rowsize = parameters.rowsize;

  const cv::Mat edge_img(rowsize, colsize, CV_8UC1, const_cast<unsigned char*>(edge));
  cv::Mat temp_img;
  PointSet contours, contours_temp;

  temp_img = edge_img.clone();
  cv::findContours(temp_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  /* 直線を探す */
  if (model.numOfVertices > 0 && !(no_search & NO_SEARCH_VERTEX))
    {
      contours_temp = contours;
      std::list<LineFeature2D> lf = extract_lines(contours_temp, line_thres, min_points, 50);

      //cv::cvtColor(edge_img * 255, temp_img, CV_GRAY2RGB);
      //draw_LineFeatures(temp_img, lf);

      /* 直線から頂点特徴をつくる */
      std::list<VertexFeature> vf = construct_vertex_features(lf);

      /* 特徴を配列にコピー */
      features.vertex.resize(vf.size());
      std::list<VertexFeature>::iterator vi = vf.begin();
      for (size_t i = 0; i < features.vertex.size(); ++i)
        {
          features.vertex[i] = *vi;
          ++vi;
        }
    }

  /* 楕円を探す */
  if (model.numOfCircles > 0 && !(no_search & NO_SEARCH_ELLIPSE))
    {
      contours_temp = contours;
      std::list<EllipseFeature> ef = extract_ellipses(contours_temp, ellipse_thres, min_points, 100);

      /* 特徴を配列にコピー */
      features.ellipse.resize(ef.size());
      std::list<EllipseFeature>::iterator ei = ef.begin();
      for (size_t i = 0; i < features.ellipse.size(); ++i)
        {
          features.ellipse[i] = *ei;
          ++ei;
        }
    }

  return features;
}

// 画像から2次元特徴を抽出
Features2D
ovgr::ImageToFeature2D(unsigned char* src, unsigned char* edge,
                       const Parameters& parameters, const Features3D& model)
{
  const int no_search = parameters.feature2D.no_search_features;

  //extractEdge_new(edge, src, EEsearchedLarge, parameters);
  extractEdge(edge, src, EEsearchedLarge, parameters);

  // 特徴点を抽出する
  ovgr::Features2D features = ovgr::extractFeatures(edge, parameters, model);

#if 1
  cv::Mat gray(parameters.rowsize, parameters.colsize, CV_8UC1, src), color;
  //cv::imwrite("edge.png", cv::Mat(parameters.rowsize, parameters.colsize, CV_8UC1, edge) * 255);

  if (model.numOfVertices > 0 && !(no_search & NO_SEARCH_VERTEX))
    {
      cv::cvtColor(gray, color, CV_GRAY2RGB);
      ovgr::draw_VertexFeatures(color, features.vertex);
      //cv::imwrite("vertex.png", color);
    }

  if (model.numOfCircles > 0 && !(!(no_search & NO_SEARCH_ELLIPSE)))
    {
      cv::cvtColor(gray, color, CV_GRAY2RGB);
      ovgr::draw_EllipseFeatures(color, features.ellipse);
      //cv::imwrite("ellipse.png", color);
    }
#endif

  return features;
}

static int
calc_side_of_face(const Point2D& p1, const Point2D& p2, const Point2D& p3)
{
  double cross[3], det;

  cross[0] = p1.y - p2.y;
  cross[1] = p2.x - p1.x;
  cross[2] = p1.x * p2.y - p1.y * p2.x;

  det = cross[0] * p3.x + cross[1] * p3.y + cross[2];

  return det > 0.0 ? 1 : -1;
}

Features2D_old*
ovgr::create_old_features_from_new_one(const Features2D& features)
{
  ::Features2D_old* old_features(NULL);
  size_t num_features(features.vertex.size() + features.ellipse.size());

  /* 本体のメモリを確保 */
  old_features = (::Features2D_old*) calloc(1, sizeof(::Features2D_old));
  if (old_features == NULL)
    {
      goto error_old_features;
    }

  /* 各特徴量のメモリを確保 */
  old_features->feature = (::Feature2D_old*) calloc(num_features, sizeof(::Feature2D_old));
  if (old_features->feature == NULL)
    {
      goto error_feature;
    }
  old_features->nAlloc = num_features;
  old_features->nFeature = num_features;

  /* 点列のメモリを確保 */
  old_features->track = (::Track*) calloc(num_features, sizeof(::Track)); /* 特徴の数と同じ */
  if (old_features->track == NULL)
    {
      goto error_track;
    }
  old_features->nTrack = num_features;

  /* 点列をコピー */
  for (size_t i = 0; i < features.vertex.size(); ++i)
    {
      ::Track* t = &old_features->track[i];
      const VertexFeature& vf = features.vertex[i];

      size_t n = 0;
      for (size_t j = 0; j < vf.segment.size(); ++j)
        {
          n += vf.segment[j].size();
        }

      if(n > 0)
        {
          t->Point = (int*) calloc(2*n, sizeof(int));
          if (t->Point == NULL)
            {
              while (i > 0)
                {
                  --i;
                  free(old_features->track[i].Point);
                  old_features->track[i].Point = NULL;
                }
              goto error_point;
            }
          t->nPoint = n;
          t->offset[0] = t->offset[1] = 0.0;
        }

      n = 0;
      for (size_t j = 0; j < vf.segment.size(); ++j)
        {
          const Segment2D& s = vf.segment[j];
          for (size_t k = 0; k < s.size(); ++k)
            {
              t->Point[2*(n + k)]     = s[k].x;
              t->Point[2*(n + k) + 1] = s[k].y;
            }

          n += s.size();
        }
    }

  for (size_t i = 0; i < features.ellipse.size(); ++i)
    {
      const size_t nv = features.vertex.size();
      ::Track* t = &old_features->track[i + nv];
      const EllipseFeature& ef = features.ellipse[i];

      size_t n = 0;
      for (size_t j = 0; j < ef.segment.size(); ++j)
        {
          n += ef.segment[j].size();
        }

      if(n > 0){
        t->Point = (int*) calloc(2*n, sizeof(int));
        if (t->Point == NULL)
          {
            size_t j = i + nv;
            while (j > 0)
              {
                --j;
                free(old_features->track[j].Point);
                old_features->track[j].Point = NULL;
              }
            goto error_point;
          }
        t->nPoint = n;
        t->offset[0] = t->offset[1] = 0.0;
      }
      n = 0;
      for (size_t j = 0; j < ef.segment.size(); ++j)
        {
          const Segment2D& s = ef.segment[j];
          for (size_t k = 0; k < s.size(); ++k)
            {
              t->Point[2*(n + k)]     = s[k].x;
              t->Point[2*(n + k) + 1] = s[k].y;
            }

          n += s.size();
        }
    }

  /* 頂点特徴をコピー */
  for (size_t i = 0; i < features.vertex.size(); ++i)
    {
      ::Feature2D_old* f = &old_features->feature[i];
      const VertexFeature& vf = features.vertex[i];

      f->type = ConicType_Hyperbola;
      for (size_t j = 0; j < 6; ++j)
        {
          f->coef[j] = vf.coef[j];
        }

      f->center[0] = vf.mid.x;
      f->center[1] = vf.mid.y;

      f->startPoint[0] = vf.start.x;
      f->startPoint[1] = vf.start.y;

      f->endPoint[0] = vf.end.x;
      f->endPoint[1] = vf.end.y;

      f->start = 0;
      f->end = vf.segment.size() - 1;
      f->all = vf.segment.size();

      f->startSPoint[0] = vf.start.x;
      f->startSPoint[1] = vf.start.y;

      f->middleSPoint[0] = vf.mid.x;
      f->middleSPoint[1] = vf.mid.y;

      f->endSPoint[0] = vf.end.x;
      f->endSPoint[1] = vf.end.y;

      f->direction[0] =  vf.line_coef[0][1];
      f->direction[1] = -vf.line_coef[0][0];

      f->nPoints = vf.segment.size();
      f->nTrack = i;
      f->error = vf.error;

      f->lineLength = vf.length[0];
      f->lineLength1 = vf.length[0];
      f->lineLength2 = vf.length[1];
      f->lineAngle = acos(vf.line_coef[0][0]*vf.line_coef[1][0] + vf.line_coef[0][1]*vf.line_coef[1][1]);

      if (calc_side_of_face(vf.start, vf.mid, vf.end) < 1)
        {
          f->startSPoint[0] = vf.end.x;
          f->startSPoint[1] = vf.end.y;

          f->endSPoint[0] = vf.start.x;
          f->endSPoint[1] = vf.start.y;

          f->lineLength1 = vf.length[1];
          f->lineLength2 = vf.length[0];
        }
    }

  /* 楕円特徴をコピー */
  for (size_t i = 0; i < features.ellipse.size(); ++i)
    {
      const size_t nv = features.vertex.size();
      ::Feature2D_old* f = &old_features->feature[i + nv];
      ::Track* t = &old_features->track[i + nv];
      const EllipseFeature& ef = features.ellipse[i];

      for (size_t j = 0; j < 6; ++j)
        {
          f->coef[j] = ef.coef[j];
        }

      // 楕円とする
      f->type = ConicType_Ellipse;
      getConicProperty(const_cast<double*>(ef.coef), &f->type, f->center, f->ev, &f->axis[0], &f->axis[1]);

      if(t->nPoint == 0)
        {
          continue;
        }

      f->error = ef.error;
      f->start = 0;
      f->end = ef.segment.size() - 1;
      f->all = ef.segment.size();
      f->nTrack = i + nv;
      f->startSPoint[0] = t->Point[0];
      f->startSPoint[1] = t->Point[1];
      f->middleSPoint[0] = t->Point[ef.segment.size()];
      f->middleSPoint[1] = t->Point[ef.segment.size() + 1];
      f->endSPoint[0] = t->Point[2*ef.segment.size()];
      f->endSPoint[1] = t->Point[2*ef.segment.size() + 1];
    }

  return old_features;


  /* エラー処理 */
 error_point:
  free(old_features->track);
 error_track:
  free(old_features->feature);
 error_feature:
  free(old_features);
 error_old_features:
  return NULL;
}

// 旧特徴量を新特徴量に変換
ovgr::Features2D
ovgr::create_new_features_from_old_one(const ::Features2D_old* old_features, unsigned char *img, const Parameters* parameters)
{
  Features2D features;

  if (old_features == NULL)
    {
      return features;
    }

  for (int i = 0; i < old_features->nFeature; ++i)
    {
      ::Feature2D_old* f = &old_features->feature[i];
      ::Track*         t = (f->nTrack >= 0 ? &old_features->track[f->nTrack] : NULL);

      if (f->type != ConicType_Hyperbola && f->type != ConicType_Ellipse)
        {
          //fprintf(stderr, "type: %d\n", f->type);
          continue;
        }

      switch (f->type)
        {
        case ConicType_Hyperbola:
          {
            VertexFeature vf;

            for (int j = 0; j < 6; ++j)
              {
                vf.coef[j] = f->coef[j];
              }
            vf.error = f->error;

            vf.start = Point2D(f->startSPoint[0], f->startSPoint[1]);
            vf.mid   = Point2D(f->center[0], f->center[1]);
            vf.end   = Point2D(f->endSPoint[0], f->endSPoint[1]);

            compute_line_coef(vf.line_coef[0], vf.start, vf.mid);
            compute_line_coef(vf.line_coef[1], vf.end, vf.mid);

            vf.length[0] = f->lineLength1;
            vf.length[1] = f->lineLength2;

            if (t != NULL && t->Point != NULL)
              {
                vf.segment.resize(1);
#if 0
                if (f->start <= f->end)
                  {
                    for (int j = f->start; j <= f->end; ++j)
                      {
                        Point2D pos(t->Point[2*j], t->Point[2*j + 1]);
                        vf.segment[0].push_back(pos);
                      }
                  }
                else
                  {
                    for (int j = f->start-1; j < f->start+1; ++j)
                      {
                        Point2D pos(t->Point[2*j], t->Point[2*j + 1]);
                        vf.segment[0].push_back(pos);
                      }
                    for (int j = 0; j <= f->end; ++j)
                      {
                        Point2D pos(t->Point[2*j], t->Point[2*j + 1]);
                        vf.segment[0].push_back(pos);
                      }
                  }
#else
                int istart = f->start;
                int iend = f->end;
                if (istart > iend)
                  {
                    iend += f->all;
                  }
                for (int j = istart; j < iend; j++)
                  {
                    int jj =  (j % t->nPoint) * 2;
                    Point2D pos(t->Point[jj], t->Point[jj + 1]);
                    vf.segment[0].push_back(pos);
                  }
#endif
              }

            features.vertex.push_back(vf);
          }
          break;

        case ConicType_Ellipse:
          {
            EllipseFeature ef;

            for (int j = 0; j < 6; ++j)
              {
                ef.coef[j] = f->coef[j];
              }
            ef.error = f->error;

            ef.center[0] = f->center[0];
            ef.center[1] = f->center[1];

            ef.axis[0] = f->axis[0];
            ef.axis[1] = f->axis[1];

            ef.theta = atan2(f->ev[0][1], f->ev[0][0]);

            if (t != NULL && t->Point != NULL)
              {
                ef.segment.resize(1);
#if 0
                if (f->start <= f->end)
                  {
                    for (int j = f->start; j <= f->end; ++j)
                      {
                        Point2D pos(t->Point[2*j], t->Point[2*j + 1]);
                        ef.segment[0].push_back(pos);
                      }
                  }
                else
                  {
                    for (int j = f->start; j < f->all; ++j)
                      {
                        Point2D pos(t->Point[2*j], t->Point[2*j + 1]);
                        ef.segment[0].push_back(pos);
                      }
                    for (int j = 0; j <= f->end; ++j)
                      {
                        Point2D pos(t->Point[2*j], t->Point[2*j + 1]);
                        ef.segment[0].push_back(pos);
                      }
                  }
#else
                int istart = f->start;
                int iend = f->end;
                if (istart > iend)
                  {
                    iend += f->all;
                  }
                for (int j = istart; j < iend; j++)
                  {
                    int jj =  ((j + f->all) % t->nPoint) * 2;
                    Point2D pos(t->Point[jj], t->Point[jj + 1]);
                    ef.segment[0].push_back(pos);
                  }
#endif
              }

            features.ellipse.push_back(ef);
          }
          break;

        default:
          continue;
        }
    }


  //fprintf(stderr, "vertex : % d\n", features.vertex.size());
  //fprintf(stderr, "ellipse: % d\n", features.ellipse.size());
#if 0
#  ifdef _OPENMP
#    pragma omp critical
#  endif
  if (img != NULL && parameters != NULL)
    {
      cv::Mat gimg(parameters->rowsize, parameters->colsize, CV_8UC1, img), cimg;
      cv::cvtColor(gimg, cimg, CV_GRAY2RGB);

      if (features.vertex.size() > 0)
        {
          draw_VertexFeatures(cimg, features.vertex);
        }

      if (features.ellipse.size() > 0)
        {
          draw_EllipseFeatures(cimg, features.ellipse);
        }
    }
#endif

  return features;
}
