/* -*- coding: utf-8 -*-
 extractFeature.hpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#ifndef _EXTRACT_FEATURE_HPP
#define _EXTRACT_FEATURE_HPP

#include <vector>

#include <cv.h>
#include <highgui.h>

#include "parameters.h"
#include "match3Dfeature.h"
#include "extractFeature_old.h"

#include "geometry.hpp"

namespace ovgr
{
  typedef std::vector<Point2D> Segment2D;

  struct Feature2D
  {
    std::vector<Segment2D> segment; //!< 元になった点列
    double error;                   //!< 当てはめ平均誤差[pixel]
  };

  struct LineFeature2D : public Feature2D
  {
    //! coef[0]*x + coef[1]*y + coef[2] = 0
    double coef[3];
    Point2D start, end; //!< 始点・終点
    double length;      //!< 線分の長さ
  };

  struct ConicFeature2D : public Feature2D
  {
    //! coef[0]*x^2 + 2*coef[1]*x*y + coef[2]*y^2 + 2*coef[3]*x + 2*coef[4]*y + coef[5] = 0
    double coef[6];
  };

  struct VertexFeature : public ConicFeature2D
  {
    double line_coef[2][3];  //!< 各線分の係数
    Point2D start, mid, end; //!< 始点・中点・終点
    double length[2];        //!< 線分の長さ
  };

  struct EllipseFeature : public ConicFeature2D
  {
    double center[2];   //!< 中心
    double axis[2];     //!< 長軸、短軸の長さ
    double theta;       //!< x軸に対する回転角
  };

  struct Features2D
  {
    std::vector<VertexFeature> vertex;
    std::vector<EllipseFeature> ellipse;
  };

  // 特徴点を取り出す
  Features2D
  extractFeatures(const unsigned char* edge,   // エッジ画像
                  const Parameters& parameters, // 全パラメータ
                  const Features3D& model);     // モデルの３次元特徴データ

  // 画像から2次元特徴を抽出
  Features2D ImageToFeature2D(unsigned char* src, unsigned char* edge,
                              const Parameters& parameters, const Features3D& model);

  // 新特徴量を旧特徴量に変換
  Features2D_old* create_old_features_from_new_one(const Features2D& features);

  // 旧特徴量を新特徴量に変換
  Features2D create_new_features_from_old_one(const Features2D_old* old_features, unsigned char *img = NULL, const Parameters* parameters = NULL);

  // 楕円の係数から描画用外接矩形を求める
  cv::RotatedRect
  calc_ellipse_rr(const double coeff[6]);

  // 頂点特徴の描画
  template <class VF>
  void
  draw_VertexFeatures(cv::Mat& img, const VF& vf)
  {
    const char window_name[] = "detected vertexes";

    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    
    for (typename VF::const_iterator vfi = vf.begin(); vfi != vf.end(); ++vfi)
      {
        cv::line(img, vfi->start, vfi->mid, CV_RGB(0, 0, 255), 1, CV_AA);
        cv::line(img, vfi->mid, vfi->end, CV_RGB(0, 0, 255), 1, CV_AA);
      }

    for (typename VF::const_iterator vfi = vf.begin(); vfi != vf.end(); ++vfi)
      {
        cv::circle(img, vfi->start, 1, CV_RGB(0, 255, 0), -1, CV_AA);
        cv::circle(img, vfi->end, 1, CV_RGB(255, 0, 0), -1, CV_AA);
      }

    for (typename VF::const_iterator vfi = vf.begin(); vfi != vf.end(); ++vfi)
      {
        cv::circle(img, vfi->mid, 1, CV_RGB(255, 255, 0), -1, CV_AA);
      }

    cv::imshow(window_name, img);
    cv::waitKey(-1);
  }

  // 楕円特徴の描画
  template <class EF>
  void
  draw_EllipseFeatures(cv::Mat& img, const EF& ef)
  {
    const char window_name[] = "detected ellipses";

    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    
    for (typename EF::const_iterator efi = ef.begin(); efi != ef.end(); ++efi)
      {
        cv::RotatedRect rr = calc_ellipse_rr(efi->coef);
        if (rr.size.width < 1.0e-12 || rr.size.height < 1.0e-12)
          {
            continue;
          }

        const double ratio = rr.size.height / rr.size.width;
        if (rr.size.width > 5.0 && rr.size.height > 5.0
            && 1.0/20.0 < ratio && ratio < 20.0)
          {
            cv::ellipse(img, rr, CV_RGB(255, 0, 0), 1, CV_AA);
            cv::circle(img, rr.center, 1, CV_RGB(255, 255, 0), -1, CV_AA);
          }
      }

    cv::imshow(window_name, img);
    cv::waitKey(-1);
  }
}

#endif /* _EXTRACT_FEATURE_HPP */
