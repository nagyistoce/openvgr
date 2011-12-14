/*
 drawing.hpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

namespace ovgr
{
  class PointsOnLine
  {
  public:
    PointsOnLine(const int x1, const int y1, const int x2, const int y2);
    int x() const { return m_x; }
    int y() const { return m_y; }
    int next();

  protected:
    int m_x, m_y;
    int m_ex, m_ey;

    int m_xinc, m_yinc;
    int m_e, m_dx, m_dy;
  };

  class PointsOnEllipse
  {
  };
}
