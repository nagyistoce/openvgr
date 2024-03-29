/*
 stereo.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file stereo.h
 * @brief ステレオ処理関連関数
 * @date \$Date::                            $
 */

#ifndef _STEREO_H
#define _STEREO_H

#include "calib.h"
#include "extractFeature_old.h"
#include "match3Dfeature.h"

//! ステレオカメラキャリブレーションデータ
typedef struct StereoCalib
{
  int numOfCameras;             //!< ステレオセットのカメラ数
  int width;                    //!< 画像幅
  int height;                   //!< 画像高さ
  double baselineLR;            //!< LR間ベースライン長
  double baselineLV;            //!< LV間ベースライン長
  double baselineRV;            //!< RV間ベースライン長
} StereoCalib;

//! 三次元頂点特徴候補データ
typedef struct VertexCandidate
{
  int valid;                    //!< 有効・無効フラグ
  int n1;                       //!< for debug
  int n2;                       //!< for debug
  int n3;                       //!< for debug
  double angle;                 //!< 頂点を成す線分の角度
  double position[3];           //!< 頂点座標
  double endpoint1[3];          //!< 端点1座標
  double endpoint2[3];          //!< 端点2座標
  double vector1[3];            //!< 端点1向き単位ベクトル
  double vector2[3];            //!< 端点2向き単位ベクトル
  double len1;                  //!< 線分1の長さ
  double len2;                  //!< 線分2の長さ
} VertexCandidate;

//! 三次元円特徴候補データ
typedef struct CircleCandidate
{
  int valid;                    //!< 有効・無効フラグ
  double center[3];             //!< 中心座標
  double normal[3];             //!< 法線ベクトル（単位化済）
  double radius;                //!< 半径
} CircleCandidate;

//! ステレオ対応点から３次元座標を計算する
//! 戻り値：復元誤差＝２つの視線（エピポーラ線）間の距離
double calculateLR2XYZ(double position3D[3],    // ３次元座標
                       Data_2D posL,            // 左画像上の対応点座標
                       Data_2D posR,            // 右画像上の対応点座標
                       CameraParam* camParamL,  // 左画像のカメラパラメータ
                       CameraParam* camParamR); // 右画像のカメラパラメータ

double calculatePlane3D(double plane3D[4],             // ３次元平面
                        const double l11[3],           // 左画像上の対応線1
                        const double l12[3],           // 左画像上の対応線2
                        const double l21[3],           // 右画像上の対応線1
                        const double l22[3],           // 右画像上の対応線2
                        const CameraParam* camParamL,  // 左画像のカメラパラメータ
                        const CameraParam* camParamR); // 右画像のカメラパラメータ

//! ３次元点の２次元画像上への投影点座標を求める
void projectXYZ2LR(Data_2D* pos2D,             // ２次元画像上の投影点座標
                   double position[3],         // ３次元点座標
                   CameraParam* cameraParam);  // 投影する画像のカメラパラメータ

//! ステレオ処理結果を旧3次元特徴構造体へセットする
bool set_circle_to_OldFeature3D(const std::vector<CircleCandidate>& candidates,
                                Features3D* feature); // ３次元特徴データ

//! 頂点のステレオ処理結果を３次元特徴構造体へセットする
bool set_vertex_to_OldFeature3D(const std::vector<VertexCandidate>& candidates,
				Features3D* feature);

#endif // _STEREO_H
