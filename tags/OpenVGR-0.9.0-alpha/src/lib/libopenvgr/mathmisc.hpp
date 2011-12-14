/* -*- mode: C++; coding: utf-8 -*-
 mathmisc.hpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

namespace ovgr
{
  //! 2次方程式\f$a x^2 + b x + c = 0\f$を解く
  int solve_quad_eq(double x[2], const double a, const double b, const double c);
}
