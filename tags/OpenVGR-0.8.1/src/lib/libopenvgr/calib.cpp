/*
 calib.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file calib.cpp
 * @brief キャリブレーション関連の関数
 * @date \$Date::                            $
 */
#include <stdio.h>
#include <cxcore.h>
#include <cv.h>

#include "common.h"
#include "vectorutil.h"
#include "calib.h"

// 歪みあり実画像位置→カメラモデル系の理想画像位置(X', Y')
void
undistortPosition(Data_2D* icPos,              // 理想画像上の座標
                  Data_2D iPos,                // 歪みあり座標
                  CameraParam* cameraParam)    // カメラパラメータ
{
  CvMat src, dst, cameraMatrix, distCoeffs;
  double dpos[2] = {iPos.col, iPos.row}, pos[2];
  double k[5] = {
    cameraParam->Distortion.k1, cameraParam->Distortion.k2,
    cameraParam->Distortion.p1, cameraParam->Distortion.p2,
    cameraParam->Distortion.k3
  };

  src = cvMat(1, 1, CV_64FC2, dpos);
  dst = cvMat(1, 1, CV_64FC2,  pos);

  cameraMatrix = cvMat(3, 3, CV_64FC1, cameraParam->intrinsicMatrix);
  distCoeffs = cvMat(1, 5, CV_64FC1, k);

  cvUndistortPoints(&src, &dst, &cameraMatrix, &distCoeffs, NULL, NULL);

  icPos->col = pos[0];
  icPos->row = pos[1];

  return;
}

// 歪み補正後のカメラモデル系理想画像位置→画像系の歪みのある実画像位置(X', Y')
void
distortPosition(Data_2D* iPos2D,               // 歪みあり座標
                Data_2D icPos2D,               // 理想画像上の座標
                CameraParam* cameraParam)      // カメラパラメータ
{
  double x, y;
  double r2, r4, r6, a1, a2, a3, cdist;
  double k[5] = {
    cameraParam->Distortion.k1, cameraParam->Distortion.k2,
    cameraParam->Distortion.p1, cameraParam->Distortion.p2,
    cameraParam->Distortion.k3
  };
  double (*intr)[3] = cameraParam->intrinsicMatrix;

  r2 = icPos2D.col * icPos2D.col + icPos2D.row * icPos2D.row;
  r4 = r2 * r2;
  r6 = r4 * r2;
  a1 = 2.0 * icPos2D.col * icPos2D.row;
  a2 = r2 + 2.0 * icPos2D.col * icPos2D.col;
  a3 = r2 + 2.0 * icPos2D.row * icPos2D.row;

  cdist = 1.0 + k[0] * r2 + k[1] * r4 + k[4] * r6;
  x = icPos2D.col * cdist + k[2] * a1 + k[3] * a2;
  y = icPos2D.row * cdist + k[2] * a3 + k[3] * a1;

  iPos2D->col = intr[0][0] * x /* + intr[0][1] * y */ + intr[0][2];
  iPos2D->row =                  intr[1][1] * y + intr[1][2];

  return;
}

// 画像座標→正規化座標
void
backprojectPoint(Data_2D* icPos,              // 正規化座標
                 Data_2D iPos,                // 画像座標
                 CameraParam* cameraParam)    // カメラパラメータ
{
  double (*intr)[3] = cameraParam->intrinsicMatrix;

  icPos->row = (iPos.row - intr[1][2]) / intr[1][1];
  icPos->col = (iPos.col - intr[0][2] - intr[0][1] * icPos->row) / intr[0][0];
}

// 正規化座標→画像座標
void projectPoint(Data_2D* iPos2D,               // 画像座標
                  Data_2D icPos2D,               // 正規化座標
                  CameraParam* cameraParam)      // カメラパラメータ
{
  double (*intr)[3] = cameraParam->intrinsicMatrix;

  iPos2D->col = intr[0][0] * icPos2D.col + intr[0][1] * icPos2D.row + intr[0][2];
  iPos2D->row =                            intr[1][1] * icPos2D.row + intr[1][2];
}
