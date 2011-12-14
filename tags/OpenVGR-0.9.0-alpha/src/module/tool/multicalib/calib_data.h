/*
 calib_data.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#ifndef CALIB_DATA_H
#define CALIB_DATA_H

#include <stdio.h>

#define CDATA_SUCCESS  0
#define CDATA_ERROR   -1

struct tag_cdata_observed_points;

typedef struct tag_cdata
{
  double interval;
  double rotation_deg;

  int num_observations;
  int num_cameras;

  struct tag_cdata_observed_points *data; /* data(t, cam) = data[t*num_cameras + cam] */
  double *weights;
} cdata_t;

typedef double cdata_point_t[2];

typedef struct tag_cdata_observed_points
{
  int num_points;

  cdata_point_t *ref, *cam;
  double *weights;
} cdata_observed_points_t;


int cdata_create (cdata_t **cdata);
int cdata_init (cdata_t *cdata);
int cdata_alloc (cdata_t *cdata, const double interval, const double deg, const int nobserv, const int ncamera);
int cdata_free (cdata_t *cdata);
int cdata_final (cdata_t *cdata);
int cdata_destroy (cdata_t **cdata);

int cdata_read (cdata_t *cdata, FILE *fp);
int cdata_write (const cdata_t *cdata, FILE *fp);

int cdata_observed_points_init (cdata_observed_points_t *op);
int cdata_observed_points_alloc (cdata_observed_points_t *op, const int npoint);
int cdata_observed_points_free (cdata_observed_points_t *op);
int cdata_observed_points_final (cdata_observed_points_t *op);

#endif /* CALIB_DATA_H */
