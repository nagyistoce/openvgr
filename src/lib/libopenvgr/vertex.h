/*
 vertex.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file vertex.h
 * @brief 3次元頂点特徴生成関連関数
 * @date \$Date::                            $
 */

#ifndef _VERTEX_H
#define _VERTEX_H

#include "extractFeature.hpp"
#include "correspondence.hpp"

//! 二次元双曲線データから三次元頂点データを生成
void HyperbolaToVertex(StereoPairing pairing,  // ステレオペアリング情報
		       CalibParam calib,       // キャリブレーションデータ
		       StereoData &stereo,     // ステレオ対応データ
		       Features2D_old *left,   // 左画像の２次元特徴データ
		       Features2D_old *right,  // 右画像の２次元特徴データ
		       unsigned char *edgeL,   // 左画像のエッジ画像
		       unsigned char *edgeR,   // 右画像のエッジ画像
		       Parameters parameters   // 全パラメータ
  );

//! 二次元頂点データから三次元頂点データを生成
void reconstruct_hyperbola_to_vertex3D(const std::vector<const ovgr::Features2D*> feature,
                                       const ovgr::CorrespondingSet& cs,
                                       const CameraParam* camParam[3],
                                       const unsigned char* edge[3],
                                       Features3D* scene,
                                       const Parameters& parameters);

#endif // _VERTEX_H
