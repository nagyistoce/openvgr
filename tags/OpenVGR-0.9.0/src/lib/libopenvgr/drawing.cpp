/*
 drawing.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "drawing.hpp"

using namespace ovgr;

/*
  Bresenham's line algorithm
  http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
*/
ovgr::PointsOnLine::PointsOnLine(const int x1, const int y1, const int x2, const int y2)
  : m_x(x1), m_y(y1), m_ex(x2), m_ey(y2)
{
  m_xinc = (x1 < x2) ? 1 : -1;
  m_yinc = (y1 < y2) ? 1 : -1;

  m_dx = abs(x2 - x1);
  m_dy = abs(y2 - y1);
  m_e = m_dx - m_dy;
}

int
ovgr::PointsOnLine::next()
{
  int e2 = 2 * m_e;

  if (e2 > -m_dy)
    {
      m_e -= m_dy;
      m_x += m_xinc;
    }

  if (e2 < m_dx)
    {
      m_e += m_dx;
      m_y += m_yinc;
    }

  return (m_x != m_ex) || (m_y != m_ey);
}
