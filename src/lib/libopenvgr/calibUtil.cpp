/*
 calibUtil.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include "calibUtil.h"

#include "vectorutil.h"

// ワールド座標系におけるカメラ座標の計算
static void
calcCameraPos(double Tr[3], double Rr[3][3], double T[3])
{
  mulM33V3(Rr, T, Tr);
  mulV3S(-1.0, Tr, Tr);
  return;
}

//
// CameraImage 内のキャリブレーションデータを、
// CameraParam 構造体にセットする。
//
void
setCalibFromCameraImage(const Img::CameraImage& image, CameraParam& camera)
{
  int j, k;
  int length;

  // キャリブレーションデータをモデルデータに埋め込む。
  camera.intrinsicMatrix[0][0] = image.intrinsic.matrix_element[0];
  camera.intrinsicMatrix[0][1] = image.intrinsic.matrix_element[1];
  camera.intrinsicMatrix[0][2] = image.intrinsic.matrix_element[3];
  camera.intrinsicMatrix[1][0] = 0.0;
  camera.intrinsicMatrix[1][1] = image.intrinsic.matrix_element[2];
  camera.intrinsicMatrix[1][2] = image.intrinsic.matrix_element[4];
  camera.intrinsicMatrix[2][0] = 0.0;
  camera.intrinsicMatrix[2][1] = 0.0;
  camera.intrinsicMatrix[2][2] = 1.0;

  camera.fx = camera.intrinsicMatrix[0][0];     // 焦点
  camera.fy = camera.intrinsicMatrix[1][1];     // 焦点
  camera.cx = camera.intrinsicMatrix[0][2];     // カメラ中心
  camera.cy = camera.intrinsicMatrix[1][2];     // カメラ中心

  for (j = 0; j < 3; j++)
    {
      for (k = 0; k < 3; k++)
        {
          camera.Rotation[j][k] = image.extrinsic[j][k];
        }
      camera.Translation[j] = image.extrinsic[j][3];
    }

  // 回転行列の逆行列
  transposeM33(camera.Rotation, camera.rRotation);

  // 左カメラの位置 (-rR・T)
  calcCameraPos(camera.Position, camera.rRotation, camera.Translation);

  // 歪み補正パラメータのセット
  length = image.intrinsic.distortion_coefficient.length();
  if (length > 5)
    {
      length = 5;
    }

  double distortionMatrix[5];
  for (j = 0; j < 5; j++)
    {
      distortionMatrix[j] = 0.0;
    }

  for (j = 0; j < length; j++)
    {
      distortionMatrix[j] = image.intrinsic.distortion_coefficient[j];
    }

  camera.Distortion.k1 = distortionMatrix[0];
  camera.Distortion.k2 = distortionMatrix[1];
  camera.Distortion.k3 = distortionMatrix[4];
  camera.Distortion.p1 = distortionMatrix[2];
  camera.Distortion.p2 = distortionMatrix[3];
  return;
}
