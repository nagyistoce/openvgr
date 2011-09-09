/*
 detect_checker.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#ifndef DETECT_CHECKER_H
#define DETECT_CHECKER_H

#include <cv.h>

#include "capture.h"

typedef struct tag_checker_coord
{
  int col;
  int row;

  int num_points;
  int (*checker_pos)[2];
  double (*image_pos)[2];
} checker_coord_t;

#ifdef __cplusplus
extern "C" {
#endif

checker_coord_t *dc_checker_alloc (const int num_points);
void dc_checker_free (checker_coord_t *cc);

#ifdef __cplusplus
}
#endif

#endif /* DETECT_CHECKER_H */
