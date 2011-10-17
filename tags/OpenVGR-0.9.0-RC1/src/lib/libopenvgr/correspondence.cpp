/* -*- mode: C++; coding: utf-8 -*-
 correspondence.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#include <cstdio>
#include <cmath>

#include <vector>
#include <list>
#include <algorithm>

#include <cv.h>

#include "constants.hpp"
#include "correspondence.hpp"
#include "geometry.hpp"
#include "mathmisc.hpp"

using namespace ovgr;

#ifdef DEBUG_CORRES
static void
disp_Mat(const cv::Mat& m)
{
  for (int i = 0; i < m.rows; ++i)
    {
      const double* p = m.ptr<const double>(i);
      for (int j = 0; j < m.cols; ++j)
        {
          printf("% 10.3g ", p[j]);
        }
      printf("\n");
    }
  printf("\n");
}
#endif

inline static double
sign(const double val, const double eps = epsilon)
{
  if (fabs(val) < eps)
    {
      return 0.0;
    }

  return val < 0.0 ? -1.0 : 1.0;
}

struct VertexEvaluator
{
private:
  const double tolerance;

  const CameraParam& cp1;
  const CameraParam& cp2;

  cv::Mat H_inf;
  double ep[2][3];

protected:
  int calc_endpoint_relation(const double e[3], const VertexFeature& vf) const
  {
    double coef[3], p[3] = {vf.mid.x, vf.mid.y, 1.0};
    double rel;

    calc_line_joining_points(coef, e, p);
    rel = distance_from_line(coef, vf.start) * distance_from_line(coef, vf.end);
    
    if (fabs(rel) < epsilon)
      {
        return 0;
      }

    return rel < 0.0 ? -1 : 1;
  }

public:
  VertexEvaluator(const CameraParam& _cp1, const CameraParam& _cp2, const double _tolerance = 1.0)
    : tolerance(_tolerance),
      cp1(_cp1), cp2(_cp2),
      H_inf(calc_infinite_homography(_cp1, _cp2))
  {
    calc_epipole(ep[0], _cp1, _cp2);
    calc_epipole(ep[1], _cp2, _cp1);

    //fprintf(stderr, "epipole: (% f, % f), (% f, % f)\n", ep[0][0]/ep[0][2], ep[0][1]/ep[0][2], ep[1][0]/ep[1][2], ep[1][1]/ep[1][2]);
  }

  bool operator()(const VertexFeature& vf1, const VertexFeature& vf2) const
  {
    double homo_pos[2][3] = {{vf1.mid.x, vf1.mid.y, 1.0}, {vf2.mid.x, vf2.mid.y, 1.0}};
    double homo_pos_inf[2][3];
    cv::Mat pos[2] = {cv::Mat(3, 1, CV_64FC1, homo_pos[0]), cv::Mat(3, 1, CV_64FC1, homo_pos[1])};
    cv::Mat pos_inf[2];

    double coef[2][3];
    double line_dir[2][2];
    double distance[2] = {0.0, 0.0};

    // 他方の視点と頂点を通る直線上の無限遠点
    pos_inf[0] = H_inf.inv() * pos[1];
    pos_inf[1] = H_inf * pos[0];

    // エピポーラ線を求める
    for (int i = 0; i < 2; ++i)
      {
        for (int j = 0; j < 3; ++j)
          {
            homo_pos_inf[i][j] = pos_inf[i].at<double>(j, 0);
          }
        calc_line_joining_points(coef[i], ep[i], homo_pos_inf[i]);

        // エピポーラ線の方向
        line_dir[i][0] =  coef[i][1];
        line_dir[i][1] = -coef[i][0];
      }

    // 頂点がエピポーラ線上に乗っているか調べる
    distance[0] = fabs(distance_from_line(coef[0], vf1.mid));
    distance[1] = fabs(distance_from_line(coef[1], vf2.mid));
    if ((distance[0] > distance[1] ? distance[0] : distance[1]) >= tolerance)
      {
        return false;
      }

    // 頂点が他方のカメラの光学中心と無限遠点の間にあるか調べる
    for (int i = 0; i < 2; ++i)
      {
        double proj = (line_dir[i][0] * homo_pos[i][0] + line_dir[i][1] * homo_pos[i][1]) / homo_pos[i][2];
        double epi = (line_dir[i][0] * ep[i][0] + line_dir[i][1] * ep[i][1]) / ep[i][2];
        double proj_inf = (line_dir[i][0] * homo_pos_inf[i][0] + line_dir[i][1] * homo_pos_inf[i][1]) / homo_pos_inf[i][2];

        if (proj < epi && proj_inf < proj)
          {
            return false;
          }
      }

    // エピポーラ線に対する端点の位置を調べる
    int relative_pos[2][2] = {
      {sign(distance_from_line(coef[0], vf1.start)), sign(distance_from_line(coef[0], vf1.end))},
      {-sign(distance_from_line(coef[1], vf2.start)), -sign(distance_from_line(coef[1], vf2.end))}
    };
    // 符号だけが重要なので、考えやすいように並び替える
    if (!(relative_pos[0][0] >= 0 && relative_pos[0][1] >= 0)) /* 負値のみのとき符号を反転 */
      {
        for (int i = 0; i < 2; ++i)
          {
            for (int j = 0; j < 2; ++j)
              {
                relative_pos[i][j] = -relative_pos[i][j];
              }
          }
      }
    for (int i = 0; i < 2; ++i)
      {
        if (relative_pos[i][0] < relative_pos[i][1])
          {
            std::swap(relative_pos[i][0], relative_pos[i][1]);
          }
      }
    for (int i = 0; i < 2; ++i)
      {
        if (abs(relative_pos[0][i] - relative_pos[1][i]) >= 2)
          {
            return false;
          }
      }

    return true;
  }
};

static int
point_to_ellipse_tangent(const EllipseFeature& ef,
                         const double p[3],
                         Point2D t[2])
{
  const double a2[2] = {ef.axis[0] * ef.axis[0], ef.axis[1] * ef.axis[1]};
  double p2[3] = {p[0] - ef.center[0]*p[2], p[1] - ef.center[1]*p[2], p[2]};
  double q[3], l[3];

  // 正規化した楕円の座標系にpを変換
  q[0] =  cos(ef.theta) * p2[0] + sin(ef.theta) * p2[1];
  q[1] = -sin(ef.theta) * p2[0] + cos(ef.theta) * p2[1];
  q[2] = p2[2];

  // 点qで交わる2本の接線の接点を通る直線
  l[0] = q[0] / a2[0];
  l[1] = q[1] / a2[1];
  l[2] = -q[2];

  // 2次方程式を解く
  double x[2], y[2];
  if (fabs(l[0]) > fabs(l[1]))
    {
      solve_quad_eq(y, l[0]*l[0]*a2[0] + l[1]*l[1]*a2[1], 2.0 * l[1]*l[2] * a2[1], a2[1] * (l[2]*l[2] - l[0]*l[0]*a2[0]));

      x[0] = -(l[1]*y[0] + l[2]) / l[0];
      x[1] = -(l[1]*y[1] + l[2]) / l[0];
    }
  else
    {
      solve_quad_eq(x, l[0]*l[0]*a2[0] + l[1]*l[1]*a2[1], 2.0 * l[0]*l[2] * a2[0], a2[0] * (l[2]*l[2] - l[1]*l[1]*a2[1]));

      y[0] = -(l[0]*x[0] + l[2]) / l[1];
      y[1] = -(l[0]*x[1] + l[2]) / l[1];
    }

  // 画像座標系に戻す
  for (int i = 0; i < 2; ++i)
    {
      t[i].x = cos(ef.theta) * x[i] - sin(ef.theta) * y[i] + ef.center[0];
      t[i].y = sin(ef.theta) * x[i] + cos(ef.theta) * y[i] + ef.center[1];
    }

  return 0;
}

struct EllipseEvaluator
{
private:
  const double tolerance;

  const CameraParam& cp1;
  const CameraParam& cp2;

  cv::Mat F;
  double ep[2][3];

public:
  EllipseEvaluator(const CameraParam& _cp1, const CameraParam& _cp2, const double _tolerance = 1.0)
    : tolerance(_tolerance),
      cp1(_cp1), cp2(_cp2),
      F(calc_fundamental_matrix(_cp1, _cp2))
  {
    // カメラ1 のエピポールep[0]を計算 calc_epipole()
    calc_epipole(ep[0], _cp1, _cp2);
    // カメラ2 のエピポールep[1]を計算 calc_epipole()
    calc_epipole(ep[1], _cp2, _cp1);   
  }

  bool operator()(const EllipseFeature& ef1, const EllipseFeature& ef2) const
  {
    double	ep2[2][2];
    //double	t[2][2][2];
    Point2D	t[2][2];
    int	cam_t, cam_l, il, it;
    double	eline[3];
    double	dist[2][2][2];
    bool	is_caml_1[2] = {false, true};

    // カメラ1のエピポールから楕円に引いた接点x2 t[0][0,1][0,1]を計算
    //                                point_to_ellipse_tangent()
    ep2[0][0] = ep[0][0] / ep[0][2];
    ep2[0][1] = ep[0][1] / ep[0][2];
    point_to_ellipse_tangent(ef1, ep[0], t[0]);
    //point_to_ellipse_tangent(ef1.coef, ep2[0], t[0]);

    // カメラ2のエピポールから楕円に引いた接点x2 t[1][0,1][0,1]を計算
    //                                point_to_ellipse_tangent()
    ep2[1][0] = ep[1][0] / ep[1][2];
    ep2[1][1] = ep[1][1] / ep[0][2];
    point_to_ellipse_tangent(ef2, ep[1], t[1]);
    //point_to_ellipse_tangent(ef2.coef, ep2[1], t[1]);

    // カメラ１の接点を A B カメラ２の接点を a b とすると、
    // 対応は、(A-a, B-b) または (A-b, B-a)
    // 二つの接点がどう対応するか不明なので、8通りのエピポーラ線までの距離を計算し、
    // 判定する
    // (A-a, B-b) のとき A-l_a, a-l_A, B-lb, b-lB がすべてしきい値より小さい
    // (A-b, B-a) のとき A-l_b, b-l_A, B-la, a-lB がすべてしきい値より小さい
    for(cam_t = 0; cam_t < 2; cam_t++)
      {
        cam_l = 1 - cam_t;
        for(il = 0; il < 2; il++)
          {
            calc_epipolar_line(eline, F, t[cam_l][il], is_caml_1[cam_l]);
            for(it = 0; it < 2; it++)
              {
                dist[cam_t][il][it] = fabs(distance_from_line(eline, t[cam_t][it]));
              }
          }
      }

    if((dist[0][0][0] < tolerance && // A-l_a
        dist[1][0][0] < tolerance && // a-l_A
        dist[0][1][1] < tolerance && // B-l_b
        dist[1][1][1] < tolerance) ||// b-l_B
       (dist[0][0][1] < tolerance && // A-l_b
        dist[1][1][0] < tolerance && // b-l_A
        dist[0][1][0] < tolerance && // B-l_a
        dist[1][0][1] < tolerance))  // a-l_B
      {
        // 対応していればtrue、していなければfalseを返す
        return true;
      }

    return false;
  }
};

template <class T, class Function>
static feature_map_t
create_feature_map(const T& feature1, const CameraParam& cp1, const T& feature2, const CameraParam& cp2, Function eval_func)
{
  feature_map_t fmap(feature1.size());

#ifdef _OPENMP
#pragma omp parallel for shared(fmap)
#endif
  for (size_t i = 0; i < feature1.size(); ++i)
    {
      for (size_t j = 0; j < feature2.size(); ++j)
        {
          if (eval_func(feature1[i], feature2[j]) == true)
            {
#ifdef _OPENMP
#pragma omp critical
#endif
              fmap[i].push_back(j);
            }
        }
    }

  return fmap;
}

// 3次元の拘束を満たす2次元特徴の組を作る
CorrespondingPair
ovgr::make_corresponding_pairs(const Features2D& feature1, const CameraParam& cp1, const Features2D& feature2, const CameraParam& cp2, const CorrespondenceThresholds& thres)
{
  CorrespondingPair cp;

  //printf("feature: %d %d\n", feature1.vertex.size(), feature2.vertex.size());

  // 頂点特徴
  cp.vertex = create_feature_map(feature1.vertex, cp1, feature2.vertex, cp2, VertexEvaluator(cp1, cp2, thres.vertex_tolerance));

#if 0
  for (size_t i = 0; i < cp.vertex.size(); ++i)
    {
      if (cp.vertex[i].size() > 0)
        {
          printf("%4d:", i);
          for (feature_index_list_t::const_iterator j = cp.vertex[i].begin(); j != cp.vertex[i].end(); ++j)
            {
              printf(" %d", *j);
            }
          printf("\n");
        }
    }
#endif

#if 1
  // 楕円特徴
  cp.ellipse = create_feature_map(feature1.ellipse, cp1, feature2.ellipse, cp2, EllipseEvaluator(cp1, cp2, thres.ellipse_tolerance));

#if 0
  for (size_t i = 0; i < cp.ellipse.size(); ++i)
    {
      if (cp.ellipse[i].size() > 0)
        {
          printf("%4d:", i);
          for (feature_index_list_t::const_iterator j = cp.ellipse[i].begin(); j != cp.ellipse[i].end(); ++j)
            {
              printf(" %d", *j);
            }
          printf("\n");
        }
    }
#endif
#endif

  return cp;
}

static feature_list_t
unpack_feature_map(const feature_map_t& fm, const int num_camera = 2, const int offset = 0)
{
  feature_list_t fl;

  //printf("num_camera: %d, offset: %d\n", num_camera, offset);
  if (offset >= num_camera)
    {
      return fl;
    }

  for (size_t i = 0; i < fm.size(); ++i)
    {
      for (feature_index_list_t::const_iterator fili = fm[i].begin(); fili != fm[i].end(); ++fili)
        {
          feature_indexes_t fi(num_camera, -1);

          fi[offset] = i;
          fi[(offset+1) % num_camera] = *fili;

#if 0
          for (size_t j = 0; j < fi.size(); ++j)
            {
              printf("% d ", fi[j]);
            }
          printf("\n");
#endif
          fl.push_back(fi);
        }
    }

  return fl;
}

static bool
store_corresponding_features(feature_list_t *fl, const std::vector<const CorrespondingPair*>& corres_pair, const int num_camera, const feature_map_t CorrespondingPair::*feature, const int feature_no = -1, const int base_feature = -1, const int current_pos = -1, feature_indexes_t* fi = NULL)
{
  bool found = false;

  if (fi != NULL)
    {
      const feature_index_list_t& fil = (corres_pair[current_pos]->*feature)[feature_no];

      if (fil.size() < 1)
        {
          return false;
        }

      (*fi)[current_pos] = feature_no;

      if (num_camera - current_pos > 1)
        {          
          for (feature_index_list_t::const_iterator fili = fil.begin(); fili != fil.end(); ++fili)
            {
              found |= store_corresponding_features(fl, corres_pair, num_camera, feature, *fili, base_feature, current_pos + 1, fi);
            }
        }
      else
        {
          for (feature_index_list_t::const_iterator fili = fil.begin(); fili != fil.end(); ++fili)
            {
              if (*fili == base_feature)
                {
                  fl->push_back(*fi);
                  return true;
                }
            }
          return false;
        }
    }
  else
    {
      const feature_map_t& fm = corres_pair[0]->*feature;
      for (size_t i = 0; i < fm.size(); ++i)
        {
          if (fm[i].size() < 1)
            {
              continue;
            }

          feature_indexes_t fidx(num_camera, -1);
          fidx[0] = i;

          for (feature_index_list_t::const_iterator fili = fm[i].begin(); fili != fm[i].end(); ++fili)
            {
              found |= store_corresponding_features(fl, corres_pair, num_camera, feature, *fili, i, 1, &fidx);
            }
        }
    }

  return found;
}

/* corres_pair[i] はカメラ i とカメラ (i+1)%num_camera の対応とする */
static CorrespondingSet
extract_cyclic_correspondence(const std::vector<const CorrespondingPair*>& corres_pair, const int num_camera)
{
  CorrespondingSet cs;

  store_corresponding_features(&cs.vertex, corres_pair, num_camera, &CorrespondingPair::vertex);

  store_corresponding_features(&cs.ellipse, corres_pair, num_camera, &CorrespondingPair::ellipse);

  return cs;
}

// ペアリングの基準に適合する特徴に絞り込む
CorrespondingSet
ovgr::filter_corresponding_set(const std::vector<const CorrespondingPair*>& corres_pair, const CorrespondingCriteria criteria)
{
  const int num_camera = static_cast<int>((1.0 + sqrt(1.0 + 8.0*static_cast<double>(corres_pair.size()))) / 2.0 + 0.5); // 4眼以上の時に修正が必要
  CorrespondingSet cs;

  //printf("num_cameras: %d\n", num_camera);
  if (num_camera < 2)
    {
      return cs;
    }

  if (num_camera == 2 || criteria == CorresOr)
    {
      for (size_t i = 0; i < corres_pair.size(); ++i)
        {
          feature_list_t fl;

          fl = unpack_feature_map(corres_pair[i]->vertex, num_camera, i);
          cs.vertex.merge(fl);

          fl = unpack_feature_map(corres_pair[i]->ellipse, num_camera, i);
          cs.ellipse.merge(fl);
        }

      return cs;
    }

  switch (criteria)
    {
    case CorresOr:
      /* should not come here */
      break;

    case CorresAnd:
      cs = extract_cyclic_correspondence(corres_pair, num_camera);
      break;
    }

  return cs;
}
