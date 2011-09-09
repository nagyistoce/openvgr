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
#include "extractFeature.h"
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

//! 二次曲線ステレオ対応データ
typedef struct StereoConic
{
  ConicType type;               //!< 二次曲線のタイプ
  Feature2D* featureL;          //!< 左画像の２次元特徴
  Feature2D* featureR;          //!< 右画像の２次元特徴
  int valid;                    //!< 有効無効フラグ
  double error;                 //!< ステレオ対応誤差
  double center[3];             //!< 復元円中心、復元頂点の３次元座標
  union
  {
    VertexCandidate vertex;     //!< 頂点として処理した場合に保存する頂点情報
    CircleCandidate circle;     //!< 円として処理した場合に保存する円情報
  } work;
} StereoConic;

//! ステレオ対応データ
typedef struct StereoData
{
  int numOfconics;              //!< ２次曲線ステレオ対応個数
  StereoConic* conics;          //!< ２次曲線ステレオ対応データ
} StereoData;

//! 歪み補正点座標(X', Y')より視線ベクトルを計算する
void calculateSightVector(double* SightVector,         // 視線ベクトル
                          Data_2D icPos,               // 歪み補正点座標
                          CameraParam* cameraParam);   // カメラパラメータ

//! ステレオ対応点から３次元座標を計算する
//! 戻り値：復元誤差＝２つの視線（エピポーラ線）間の距離
double calculateLR2XYZ(double position3D[3],    // ３次元座標
                       Data_2D posL,            // 左画像上の対応点座標
                       Data_2D posR,            // 右画像上の対応点座標
                       CameraParam* camParamL,  // 左画像のカメラパラメータ
                       CameraParam* camParamR); // 右画像のカメラパラメータ

//! ３次元点の２次元画像上への投影点座標を求める
void projectXYZ2LR(Data_2D* pos2D,             // ２次元画像上の投影点座標
                   double position[3],         // ３次元点座標
                   CameraParam* cameraParam);  // 投影する画像のカメラパラメータ

//! ステレオ対応データのメモリ解放
void freeStereoData(StereoData* stereo);

//! ステレオ対応データの作成
//! 戻り値：ステレオ対応データ
StereoData StereoCorrespondence(StereoPairing pairing,   // ステレオペア情報
                                CalibParam calib,        // キャリブレーションデータ
                                Features2D* left,        // 左画像の２次元特徴
                                Features2D* right,       // 右画像の２次元特徴
                                Parameters parameters);  // 全パラメータ

//! ステレオ処理結果を３次元特徴構造体へセットする
bool setFeature3D(StereoData& stereo, // ステレオ対応データ
		  Features3D& feature); // ３次元特徴データ

//! ステレオ処理結果を３次元特徴構造体へセットする：３眼ＯＲ処理
bool setFeature3D_TBLOR(StereoData& stereoLR, // ＬＲペアのステレオ対応データ
                        StereoData& stereoLV, // ＬＶペアのステレオ対応データ
                        StereoData& stereoRV, // ＲＶペアのステレオ対応データ
                        Features3D& feature); // ３次元特徴データ

//! ステレオ処理結果を３次元特徴構造体へセットする：３眼ＡＮＤ処理
bool setFeature3D_TBLAND(StereoData& stereoLR, // ＬＲペアのステレオ対応データ
                         StereoData& stereoLV, // ＬＶペアのステレオ対応データ
                         Features3D& feature); // ３次元特徴データ

#endif // _STEREO_H
