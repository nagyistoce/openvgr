/*
 checker_data.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <satoshi.kawabata@aist.go.jp>

 $Date::                            $
*/

#ifndef CHECKER_DATA_H
#define CHECKER_DATA_H

struct FILE;

typedef struct tag_checker_point
{
  int num;  /*num of observed points */

  /* assigned coordinates on plane (0: x, 1: y) */
  double (*plane_coord)[2];

  /* corresponding point on an image (0: x, 1: y) */
  double (*image_coord)[2];

} checker_point_t;

typedef struct tag_checker_data
{
  double size; /* unit pattern size */
  int num_cameras;

  int num_observations;
  int capacity;
  checker_point_t **data;
} checker_data_t;

void checker_data_init (checker_data_t *cd, const double size, const int num_cameras);
void checker_data_clear (checker_data_t *cd);
void checker_data_final (checker_data_t *cd);

void checker_data_write (checker_data_t *cd, FILE *fp);
void checker_data_write_compat (checker_data_t *cd, FILE *fp);

checker_point_t *checker_data_new_observation (checker_data_t *cd);
int checker_point_resize (checker_point_t *cp, const int num_points);

#endif /* CHECKER_DATA_H */
