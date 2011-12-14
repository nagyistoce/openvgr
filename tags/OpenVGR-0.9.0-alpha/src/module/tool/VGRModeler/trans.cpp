/*
 trans.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <math.h>
#include "quaternion.h"
#include "trans.h"

//! 任意軸周りの回転を表すクォータニオンを求める
void
quaternion_rotation(quaternion_t q, const double radian, double ax, double ay, double az)
{
  double norm = 0.0;
  double cosine = 0.0, sine = 0.0;

  quat_re(q) = quat_im(q, 0) = quat_im(q, 1) = quat_im(q, 2) = 0.0;

  norm = (ax * ax) + (ay * ay) + (az * az);

  if (norm <= 0)
    {
      return;
    }

  norm = 1.0 / sqrt(norm);

  ax *= norm;
  ay *= norm;
  az *= norm;

  cosine = cos(radian / 2.0);
  sine = sin(radian / 2.0);

  quat_re(q)    = cosine;
  quat_im(q, 0) = sine * ax;
  quat_im(q, 1) = sine * ay;
  quat_im(q, 2) = sine * az;
}

//! クォータニオンで回転
void
apply_3D_rotation(double* px, double* py, double* pz,
                  const double ax, const double ay, const double az,
                  const double xoff, const double yoff,
                  const double zoff, const double degree)
{
  double radianTheta = degree * M_PI / 180.0;
  quaternion_t q;
  double pos[3] = {*px - xoff, *py - yoff, *pz - zoff}, rpos[3];

  quaternion_rotation(q, -radianTheta, ax, ay, az);
  quat_rot(rpos, q, pos);

  *px = rpos[0] + xoff;
  *py = rpos[1] + yoff;
  *pz = rpos[2] + zoff;  
}

//! 点を3次元回転させる
void
translate_point(const RotationAngle* rotation,
                double xoff, double yoff, double zoff,
                double* returnX, double* returnY, double* returnZ)
{
  int rotationCount = 0;                // xyzのカウント
  double px = 0.0, py = 0.0, pz = 0.0;  // 回転前３次元位置
  double ax = 0.0, ay = 0.0, az = 0.0, degreePoint = 0.0; // 回転軸の方向と回転角度

  px = *returnX;
  py = *returnY;
  pz = *returnZ;

  for (rotationCount = 0; rotationCount < 3; rotationCount++)
    {                           // xyzの順で回転させる
      if (rotationCount == 0)
        {
          ax = -1;              // 回転軸の方向
          ay = 0;
          az = 0;
          degreePoint = rotation->rx;   // 回転角度
        }
      else if (rotationCount == 1)
        {
          ax = 0;
          ay = -1;
          az = 0;
          degreePoint = rotation->ry;
        }
      else
        {
          ax = 0;
          ay = 0;
          az = -1;
          degreePoint = rotation->rz;
        }
      xoff = 0;                 // 回転軸のある位置
      yoff = 0;
      zoff = 0;

      apply_3D_rotation(&px, &py, &pz, ax, ay, az, xoff, yoff, zoff, degreePoint);
    }

  *returnX = px;
  *returnY = py;
  *returnZ = pz;
  return;
}
