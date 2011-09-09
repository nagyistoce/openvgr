/*
 camera_param.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <satoshi.kawabata@aist.go.jp>

 $Date::                            $
*/

#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include <stdio.h>

#include "quaternion.h"

#define CP_EPS 1.0e-15

/*
 *     [ alpha gamma c[0]]
 * A = [     0  beta c[2]]
 *     [     0     0    1]
 */
typedef struct tag_intrinsic_param
{
  double alpha, beta, gamma;
  double c[2];
} intrinsic_param_t;

typedef struct tag_distortion_param
{
  double elem[5];
  int dim; /* one of 0, 2, 4, and 5 */
} distortion_param_t;

typedef struct tag_extrinsic_param
{
  quaternion_t q;
  double t[3];
} extrinsic_param_t;

typedef struct tag_camera_param
{
  intrinsic_param_t intr;
  distortion_param_t dist;
  extrinsic_param_t ext;
} camera_param_t;

/* projection:
 * image = cp_mult_intrinsic <= cp_distort <= (division by z value) <= cp_mult_extrinsic
 *
 * back-projection:
 * 3-D ray = cp_mult_extrinsic_inv <= cp_undistort <= cp_mult_intrinsic_inv
 */

/* result = A * x */
void cp_mult_intrinsic (double result[3], const intrinsic_param_t *intr, const double x[3]);
/* result = A^(-1) * x */
void cp_mult_intrinsic_inv (double result[3], const intrinsic_param_t *intr, const double x[3]);

/* result = R(q)*x + t */
void cp_mult_extrinsic (double result[3], const extrinsic_param_t *ext, const double x[3]);
/* result = R(q)^(-1) (x - t) */
void cp_mult_extrinsic_inv (double result[3], const extrinsic_param_t *ext, const double x[3]);

/* result = distorted point of (nc[0], nc[1], 1.0) */
void cp_distort (double result[2], const distortion_param_t *dist, const double nc[2]);
/* result = undistorted point of (nc[0], nc[1], 1.0) */
void cp_undistort (double result[2], const distortion_param_t *dist, const double nc[2]);


/* normalized coordinate (divide each element by x[2]) */
void cp_normalize_point (double result[3], const double x[3]);

/* conversion to a matrix from of an intrinsic parameter */
/* A(i, j) = A[i + ldim*j] (column major) */
void cp_mat_intrinsic (double *A, const int ldim, const intrinsic_param_t *intr);
void cp_matinv_intrinsic (double *A, const int ldim, const intrinsic_param_t *intr);

/* result = [R(q1) t1] * [R(q2) t2] */
void cp_mult_extrinsics (extrinsic_param_t *result, const extrinsic_param_t *ext1, const extrinsic_param_t *ext2);

/* result = ext^(-1) */
void cp_inv_extrinsic (extrinsic_param_t *result, const extrinsic_param_t *ext);

/* copy function */
void cp_copy_intrinsic (intrinsic_param_t *dst, const intrinsic_param_t *src);
void cp_copy_extrinsic (extrinsic_param_t *dst, const extrinsic_param_t *src);
void cp_copy_distortion (distortion_param_t *dst, const distortion_param_t *src);
void cp_copy_camera_param (camera_param_t *dst, const camera_param_t *src);

/* output parameters */
void cp_print_intrinsic (FILE *fp, const intrinsic_param_t *intr);
void cp_print_extrinsic (FILE *fp, const extrinsic_param_t *ext, const int output_4th_row);
void cp_print_distortion (FILE *fp, const distortion_param_t *dist);

#endif /* CAMERA_PARAM_H */
