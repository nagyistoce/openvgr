/*
 local.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#ifndef _LOCAL_H
#define _LOCAL_H

#include <math.h>

inline static double s_dot3 (const double x[3], const double y[3]);
inline static void s_cross3 (double result[3], const double x[3], const double y[3]);
inline static double s_normalize (double x[3]);

/*
 * static function
 */

static double
s_dot3 (const double x[3], const double y[3])
{
  int i;
  double dot = 0.0;

  for (i = 0; i < 3; ++i)
    {
      dot += x[i] * y[i];
    }

  return dot;
}

static void
s_cross3 (double result[3], const double x[3], const double y[3])
{
  int i;

  for (i = 0; i < 3; ++i)
    {
      result[i] = x[(i+1) % 3] * y[(i+2) % 3] - x[(i+2) % 3] * y[(i+1) % 3];
    }
}

static double s_normalize (double x[3])
{
  double norm = sqrt (s_dot3 (x, x));

  if (norm > 0)
    {
      int i;

      for (i = 0; i < 3; ++i)
        {
          x[i] /= norm;
        }
    }

  return norm;
}

#endif /* _LOCAL_H */
