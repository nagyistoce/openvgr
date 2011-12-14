/* -*- mode: C++; coding: utf-8 -*-
 correspondence.hpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#ifndef _CORRESPONDENCE_HPP
#define _CORRESPONDENCE_HPP

#include <vector>
#include <list>

#include "calib.h"
#include "extractFeature.hpp"

namespace ovgr
{
  typedef std::list<int> feature_index_list_t;
  typedef std::vector<feature_index_list_t> feature_map_t;

  struct CorrespondingPair
  {
    //CorrespondingPair(const Features2D& _from, const Features2D& _to) : from(_from), to(_to) {}

    // 2次元特徴への参照
    //const Features2D& from;
    //const Features2D& to;

    feature_map_t vertex;
    feature_map_t ellipse;
  };

  // 3次元復元候補
  typedef std::vector<int> feature_indexes_t;
  typedef std::list<feature_indexes_t> feature_list_t;

  struct CorrespondingSet
  {
    feature_list_t vertex;
    feature_list_t ellipse;
  };

  enum CorrespondingCriteria
    {
      CorresOr,
      CorresAnd
    };

  // 対応候補を決める閾値
  struct CorrespondenceThresholds
  {
    double vertex_tolerance;
    double ellipse_tolerance;

    CorrespondenceThresholds(const double vt = 1.0, const double et = 5.0)
      : vertex_tolerance(vt), ellipse_tolerance(et) {}
  };

  // 3次元の拘束を満たす2次元特徴の組を作る
  CorrespondingPair make_corresponding_pairs(const Features2D& feature1, const CameraParam& param1, const Features2D& feature2, const CameraParam& param2, const CorrespondenceThresholds& thres = CorrespondenceThresholds());

  // ペアリングの基準に適合する特徴に絞り込む
  CorrespondingSet filter_corresponding_set(const std::vector<const CorrespondingPair*>& corres_pair, const CorrespondingCriteria criteria = CorresOr);

}

#endif /* _CORRESPONDENCE_HPP */
