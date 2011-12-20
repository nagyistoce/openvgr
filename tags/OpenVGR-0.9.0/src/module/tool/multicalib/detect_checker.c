/*
 detect_checker.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <assert.h>

#include <cv.h>
#include <highgui.h>

#include "detect_checker.h"

checker_coord_t *
dc_checker_alloc (const int num_points)
{
  checker_coord_t *cc = NULL;
  int i;

  cc = (checker_coord_t *) malloc (sizeof (checker_coord_t));
  if (cc == NULL)
    {
      return NULL;
    }

  cc->checker_pos = (int (*)[2]) malloc (sizeof (int) * 2 * num_points);
  if (cc->checker_pos == NULL)
    {
      free (cc);
      return NULL;
    }

  cc->image_pos = (double (*)[2]) malloc (sizeof (double) * 2 * num_points);
  if (cc->image_pos == NULL)
    {
      free (cc->checker_pos);
      free (cc);
      return NULL;
    }

  for (i = 0; i < num_points; ++i)
    {
      int j;

      for (j = 0; j < 2; ++j)
        {
          cc->checker_pos[i][j] = -1;
          cc->image_pos[i][j]   = -1;
        }
    }

  cc->col = -1;
  cc->row = -1;
  cc->num_points = num_points;

  return cc;
}

void
dc_checker_free (checker_coord_t *cc)
{
  if (cc == NULL)
    {
      return;
    }

  cc->num_points = 0;
  cc->col = -1;
  cc->row = -1;

  free (cc->checker_pos); cc->checker_pos = NULL;
  free (cc->image_pos);   cc->image_pos = NULL;
}
