/*
 calib.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file calib.h
 * @brief キャリブレーション関連の関数
 * @date \$Date::                            $
 */

#ifndef _CALIB_H
#define _CALIB_H

#include "common.h"

//! 歪みパラメータ
typedef struct DistortionParam
{
  double k1, k2;                //!< 半径方向の歪み係数
  double p1, p2;                //!< 円周方向の歪み係数
  double k3;                    //!< 半径方向の歪み係数
} DistortionParam;

//! カメラパラメータ
typedef struct CameraParam
{
  double Rotation[3][3];        //!< 回転行列
  double Translation[3];        //!< 移動ベクトル
  double Position[3];           //!< カメラ位置(-rR・T)
  DistortionParam Distortion;   //!< 歪みパラメータ
  double intrinsicMatrix[3][3]; //!< 内部パラメータ行列
} CameraParam;

//! キャリブレーションパラメータ
typedef struct CalibParam
{
  int numOfCameras;             //!< カメラ数
  int colsize;                  //!< 画像幅
  int rowsize;                  //!< 画像高さ
  CameraParam CameraL;          //!< 左カメラの実画像とワールド座標の関係の全パラメータ
  CameraParam CameraR;          //!< 右カメラの実画像とワールド座標の関係の全パラメータ
  CameraParam CameraV;          //!< 右カメラの実画像とワールド座標の関係の全パラメータ
} CalibParam;

// 歪みあり実画像位置→歪みなし理想画像位置(X', Y')
void undistortPosition(Data_2D* icPos,             // 歪みなし座標
                       Data_2D iPos,               // 歪みあり座標
                       CameraParam* cameraParam);  // カメラパラメータ

// 歪み補正後の理想画像位置→歪みのある実画像位置(X', Y')
void distortPosition(Data_2D* iPos2D,              // 歪みあり座標
                     Data_2D icPos2D,              // 歪みなし座標
                     CameraParam* cameraParam);    // カメラパラメータ

// 画像座標→正規化座標
void backprojectPoint(Data_2D* icPos,             // 正規化座標
                      Data_2D iPos,               // 画像座標
                      CameraParam* cameraParam);  // カメラパラメータ

// 正規化座標→画像座標
void projectPoint(Data_2D* iPos2D,                // 画像座標
                  Data_2D icPos2D,                // 正規化座標
                  CameraParam* cameraParam);      // カメラパラメータ

#endif // _CALIB_H
