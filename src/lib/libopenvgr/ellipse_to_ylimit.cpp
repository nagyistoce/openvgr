/* -*- mode: C++; coding: utf-8
 ellipse_to_ylimit.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
// ellipse_to_ylimit()
// 配列 a に楕円の二次形式の係数を渡すと、楕円の y 方向の最大最小値を返す
// ylimit[0] が最小値、ylimit[1]が最大値

// 最初に数式の分母をチェックし、これが正であることを確認(楕円の条件)
// 楕円であれば最大最小が存在するのでDが負になることはないと思うけど一応チェックしている

/*
  y = (-2 a0 a4 + a1 a3 {plusminus} 2 sqrt(D)) / (4 a0 a2 - a1^2)

  D = (a0 a1^2 - 4 a0^2 a2) a5 + a0^2 a4^2 - a0 a1 a3 a4 + a0 a2 a3^2
*/

#include <math.h>

/* return value
   0: OK
   -1: error
 */

int
ellipse_to_ylimit(const double	a[6],
		  double	ylimit[2])
{
  double	bunbo;
  double	D;
  
  bunbo = 4.0 * a[0] * a[2] - a[1] * a[1];

  if(bunbo <= 0.0)
    {
      // not an ellipse
      return -1;
    }

  D = (a[0]*a[1]*a[1] - 4.0*a[0]*a[0]*a[2])*a[5]
    + a[0]*a[0]*a[4]*a[4] - a[0]*a[1]*a[3]*a[4] + a[0]*a[2]*a[3]*a[3];
  if(D < 0.0)
    {
      // must be plus
      return -1;
    }

  ylimit[0] = (-2.0*a[0]*a[4]+a[1]*a[3]-2.0*sqrt(D))/bunbo;
  ylimit[1] = (-2.0*a[0]*a[4]+a[1]*a[3]+2.0*sqrt(D))/bunbo;

  return 0;
}

