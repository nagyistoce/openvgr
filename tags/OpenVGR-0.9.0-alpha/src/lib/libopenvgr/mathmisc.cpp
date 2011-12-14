/* -*- mode: C++; coding: utf-8 -*-
 mathmisc.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#include <cmath>
#include "constants.hpp"
#include "mathmisc.hpp"

int
ovgr::solve_quad_eq(double x[2], const double a, const double b, const double c)
{
  const double D = b*b - 4.0 * a * c;

  if (D < -epsilon)
    {
      x[0] = x[1] = 0.0;
      return 0;
    }

  if (D < epsilon)
    {
      x[0] = x[1] = -b / 2.0 / a;
      return 1;
    }

#if 1
  if (b <= 0.0)
    {
      x[0] = (-b + sqrt(D)) / 2.0 / a;
      x[1] = c / x[0] / a;
    }
  else
    {
      x[1] = (-b - sqrt(D)) / 2.0 / a;
      x[0] = c / x[1] / a;
    }
#else
  x[0] = (-b + sqrt(D)) / 2.0 / a;
  x[1] = (-b - sqrt(D)) / 2.0 / a;
#endif

  return 2;
}

