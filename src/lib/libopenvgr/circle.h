/*
 circle.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file circle.h
 * @brief 3次元円特徴生成関連関数
 * @date \$Date::                            $
 */

#ifndef _CIRCLE_H
#define _CIRCLE_H

#include "extractFeature_old.h"
#include "stereo.h"

#include "extractFeature.hpp"
#include "correspondence.hpp"

// 二次元楕円のステレオ対応から三次元空間中の真円を推定・復元する
void EllipseToCircle(StereoPairing pairing,    // ステレオペア情報
		     CalibParam calib,         // キャリブレーションデータ
		     StereoData& stereo,       // ステレオ対応情報（復元真円情報を含む）
		     unsigned char* edgeL,     // 評価用エッジ画像（左）
		     unsigned char* edgeR,     // 評価用エッジ画像（右）
		     Parameters parameters);   // 全パラメータ

void
reconstruct_ellipse2D_to_circle3D(std::vector<const ovgr::Features2D*>& feature,
                                  const ovgr::CorrespondingSet& cs,
                                  const CameraParam* camParam[3],
                                  const unsigned char* edge[3],
                                  Features3D* scene,
                                  const Parameters& parameters);

//! 画像上の楕円の長軸・短軸に対応する3次元単位ベクトルの算出
void calc_3d_axes_of_circle(double major_axis[3],   //!< 長軸
                            double minor_axis[3],   //!< 短軸
                            const double normal[3], //!< 3次元円の法線
                            const CameraParam *cp); //!< カメラパラメータ

#endif // _CIRCLE_H
