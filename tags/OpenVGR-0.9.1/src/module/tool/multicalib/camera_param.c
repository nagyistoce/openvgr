/*
 camera_param.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#include <stdio.h>

#include "camera_param.h"

#include "local.h"


/* for undistorted point calculation */
inline static double s_calc_Hr (double H[3], double r[2], const double d[5], const double ref[2], const double x[2]);


/* result = A * x */
void
cp_mult_intrinsic (double result[3], const intrinsic_param_t *intr, const double x[3], double (*dx)[3], double (*dp)[3])
{
  result[0] = intr->alpha * x[0] + intr->gamma * x[1] + intr->c[0] * x[2];
  result[1] =                      intr->beta  * x[1] + intr->c[1] * x[2];
  result[2] =                                                        x[2];

  if (dx != NULL)
    {
      cp_mat_intrinsic(dx[0], 3, intr);
    }

  /* dp[5][3] = alpha, gamma, beta, c[0], c[1] */
  if (dp != NULL)
    {
      int i, j, k;
      for (i = 0; i < 3; ++i)
        {
          for (j = 0; j <= i && j < 2; ++j)
            {
              for (k = 0; k < 3; ++k)
                {
                  dp[i*(i+1)/2 + j][k] = (j != k) ? 0.0 : x[i];
                }
            }
        }
    }
}

/* result = A^(-1) * x */
void
cp_mult_intrinsic_inv (double result[3], const intrinsic_param_t *intr, const double x[3])
{
  result[2] = x[2];
  result[1] = (x[1] - intr->c[1] * result[2]) / intr->beta;
  result[0] = (x[0] - intr->c[0] * result[2] - intr->gamma * result[1]) / intr->alpha;
}

/* result = R(q)*x + t */
void
cp_mult_extrinsic (double result[3], const extrinsic_param_t *ext, const double x[3], double (*dx)[3], double (*dp)[3])
{
  int i;

  quat_rot (result, ext->q, x);

  if (dx != NULL)
    {
      quat_R_from_q (dx[0], 3, ext->q);
    }

  if (dp != NULL)
    {
#ifdef USE_INFINITESIMAL_ROTATION
      /* dp[3][3] = w1, w2, w3 */
      dp[0][0] = 0.0;
      dp[0][1] = -result[2];
      dp[0][2] =  result[1];

      dp[1][0] =  result[2];
      dp[1][1] = 0.0;
      dp[1][2] = -result[0];

      dp[2][0] = -result[1];
      dp[2][1] =  result[0];
      dp[2][2] = 0.0;
#else
      /* dp[4][3] = q1, q2, q3, q0 */
      double dot, cross[3];
      int j;

      dot = s_dot3 (ext->q, x);
      s_cross3 (cross, ext->q, x);

      for (i = 0; i < 3; ++i)
        {
          for (j = 0; j < 3; ++j)
            {
              dp[i][j] = 2.0 * (quat_im (ext->q, j) * x[i]);
              if (i == j)
                {
                  dp[i][j] += 2.0 * dot;
                }
            }
        }
      dp[0][1] -= 2.0 * quat_re(ext->q) * x[2];
      dp[1][0] += 2.0 * quat_re(ext->q) * x[2];

      dp[0][2] += 2.0 * quat_re(ext->q) * x[1];
      dp[2][0] -= 2.0 * quat_re(ext->q) * x[1];

      dp[1][2] -= 2.0 * quat_re(ext->q) * x[0];
      dp[2][1] += 2.0 * quat_re(ext->q) * x[0];

      for (i = 0; i < 3; ++i)
        {
          dp[3][i] = 2.0 * (2.0 * quat_re(ext->q) * x[i] + cross[i]);
        }
#endif
    }

  for (i = 0; i < 3; ++i)
    {
      result[i] += ext->t[i];
    }
}

/* result = R(q)^(-1) (x - t) */
void
cp_mult_extrinsic_inv (double result[3], const extrinsic_param_t *ext, const double x[3])
{
  double temp[3];
  int i;

  for (i = 0; i < 3; ++i)
    {
      temp[i] = x[i] - ext->t[i];
    }

  quat_irot (result, ext->q, temp);
}

/* result = distorted point of (nc[0], nc[1], 1.0) */
void
cp_distort (double result[2], const distortion_param_t *dist, const double nc[2], double (*dx)[3], double (*dp)[2])
{
  const double r2 = nc[0]*nc[0] + nc[1]*nc[1];
  const int n = dist->dim < 5 ? dist->dim : 5;

  double d[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  int i;

  if (n < 1)
    {
      result[0] = nc[0];
      result[1] = nc[1];
      return;
    }

  for (i = 0; i < n; ++i)
    {
      d[i] = dist->elem[i];
    }

  result[0] = nc[0] * (((d[4]*r2 + d[1])*r2 + d[0])*r2 + 1.0) + 2.0*d[2]*nc[0]*nc[1] + d[3]*(r2 + 2.0*nc[0]*nc[0]);
  result[1] = nc[1] * (((d[4]*r2 + d[1])*r2 + d[0])*r2 + 1.0) + d[2]*(r2 + 2.0*nc[1]*nc[1]) + 2.0*d[3]*nc[0]*nc[1];

  if (dx != NULL)
    {
      const double ks = 1.0 + ((d[4] * r2 + d[1]) * r2 + d[0]) * r2;
      const double dks = d[0] + r2 * (2.0*d[1] + r2 * 3.0*d[4]);

      dx[0][0] = ks + 2.0*nc[0]*nc[0]*dks + 2.0*d[2]*nc[1] + 6.0*d[3]*nc[0];
      dx[0][1] = 2.0*nc[0]*nc[1]*dks + 2.0*d[2]*nc[1] + 2.0*d[3]*nc[0];
      dx[0][2] = 0.0;

      dx[1][0] = 2.0*nc[0]*nc[1]*dks + 2.0*d[2]*nc[0] + 2.0*d[3]*nc[1];
      dx[1][1] = ks + 2.0*nc[1]*nc[1]*dks + 6.0*d[2]*nc[1] + 2.0*d[3]*nc[0];
      dx[1][2] = 0.0;

      dx[2][0] = dx[2][1] = dx[2][2] = 0.0;
    }

  /* dp[5][3] = d[0] .. d[4] */
  if (dp != NULL)
    {
      dp[0][0] = nc[0] * r2;
      dp[0][1] = nc[1] * r2;

      dp[1][0] = nc[0] * r2 * r2;
      dp[1][1] = nc[1] * r2 * r2;

      dp[2][0] = 2.0 * nc[0] * nc[1];
      dp[2][1] = r2 + 2.0 * nc[1] * nc[1];

      dp[3][0] = r2 + 2.0 * nc[0] * nc[0];
      dp[3][1] = 2.0 * nc[0] * nc[1];

      dp[4][0] = nc[0] * r2 * r2 * r2;
      dp[4][1] = nc[1] * r2 * r2 * r2;
    }
}

/* result = undistorted point of (nc[0], nc[1], 1.0) */
void
cp_undistort (double result[2], const distortion_param_t *dist, const double nc[2])
{
  const int max_iter = 100;
  const int n = dist->dim < 5 ? dist->dim : 5;
  double d[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

  int iter, i;
  double uv[2], H[3], r[2], prev_err, err;

  double sigma = 1.0e-15;

  if (n < 1)
    {
      result[0] = nc[0];
      result[1] = nc[1];
      return;
    }

  for (i = 0; i < n; ++i)
    {
      d[i] = dist->elem[i];
    }

  uv[0] = nc[0];
  uv[1] = nc[1];
  err = s_calc_Hr (H, r, d, nc, uv);

  for (iter = 0; iter < max_iter; ++iter)
    {
      double new_uv[2], lo;

      prev_err = err;
      do
        {
          double det, m[2];
          double s, predict;

          s = 1.0 + sigma;

          det = s*s*H[0]*H[2] - H[1]*H[1];
          m[0] = ( s*H[2]*r[0] -   H[1]*r[1]) / det;
          m[1] = (  -H[1]*r[0] + s*H[0]*r[1]) / det;

          predict = r[0]*m[0] + r[1]*m[1] + s*m[0]*H[0]*m[0] + 2.0*m[0]*H[1]*m[1] + s*m[1]*H[2]*m[1];

          new_uv[0] = uv[0] - m[0];
          new_uv[1] = uv[1] - m[1];

          err = s_calc_Hr (H, r, d, nc, new_uv);
          lo = (prev_err - err) / predict;

          if (lo > 0.75)
            {
              sigma /= 2.0;
            }
          else if (lo < 0.25)
            {
              sigma *= 2.0;
            }
        } 
      while (lo < 0.25);
      
      uv[0] = new_uv[0];
      uv[1] = new_uv[1];

      if (err < CP_EPS)
        {
          break;
        }
    }
  if (iter == max_iter)
    {
      fprintf (stderr, "not converged (iter: %d, err:%e)\n", iter, err);
    }

  result[0] = uv[0];
  result[1] = uv[1];
}

/* computes hessian matrix, derivertive, and error */
double
s_calc_Hr (double H[3], double r[2], const double d[5], const double ref[2], const double x[2])
{
  const double r2 = x[0]*x[0] + x[1]*x[1];
  const double d123 = d[0] + (2.0*d[1] + 3.0*d[4]*r2) * r2;
  double dx[2], diff[2], f[2][2];
  
  /* distorted point of x */
  dx[0] = x[0] * (((d[4]*r2 + d[1])*r2 + d[0])*r2 + 1.0) + 2.0*d[2]*x[0]*x[1] + d[3]*(r2 + 2.0*x[0]*x[0]);
  dx[1] = x[1] * (((d[4]*r2 + d[1])*r2 + d[0])*r2 + 1.0) + d[2]*(r2 + 2.0*x[1]*x[1]) + 2.0*d[3]*x[0]*x[1];

  /* difference of the distorted point and reference one */
  diff[0] = dx[0] - ref[0];
  diff[1] = dx[1] - ref[1];

  /* derivative of the error function at x */
  f[0][0] = (((d[4]*r2 + d[1])*r2 + d[0])*r2 + 1.0) + 2.0*x[0]*x[0] * d123 + 2.0*d[2]*x[1] + 6.0*d[3]*x[0];
  f[0][1] = 2.0 * (x[0]*x[1] * d123 + d[2]*x[0] + d[3]*x[1]);

  f[1][0] = f[0][1];
  f[1][1] = (((d[4]*r2 + d[1])*r2 + d[0])*r2 + 1.0) + 2.0*x[1]*x[1] * d123 + 6.0*d[2]*x[1] + 2.0*d[3]*x[0];

  /* Hessian matrix */
  H[0] = f[0][0]*f[0][0] + f[0][1]*f[0][1];
  H[1] = f[0][0]*f[1][0] + f[0][1]*f[1][1];
  H[2] = f[1][0]*f[1][0] + f[1][1]*f[1][1];

  /* derivative value at x */
  r[0] = f[0][0] * diff[0] + f[0][1] * diff[1];
  r[1] = f[1][0] * diff[0] + f[1][1] * diff[1];

  return diff[0]*diff[0] + diff[1]*diff[1];
}

/* normalized coordinate (divide each element by x[2]) */
void
cp_normalize_point (double result[3], const double x[3], double (*dx)[3])
{
  int i;
  for (i = 0; i < 3; ++i)
    {
      result[i] = x[i] / x[2];
    }

  if (dx != NULL)
    {
      dx[0][0] = 1.0 / x[2];
      dx[0][1] = 0.0;
      dx[0][2] = 0.0;

      dx[1][0] = 0.0;
      dx[1][1] = 1.0 / x[2];
      dx[1][2] = 0.0;

      dx[2][0] = - x[0] / x[2] / x[2];
      dx[2][1] = - x[1] / x[2] / x[2];
      dx[2][2] = 0.0;
    }
}

/* conversion to a matrix from of an intrinsic parameter */
/* A(i, j) = A[i + ldim*j] (column major) */
void
cp_mat_intrinsic (double *A, const int ldim, const intrinsic_param_t *intr)
{
  int i, j;
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          A[j + ldim*i] = 0.0;
        }
    }

  A[0] = intr->alpha;

  A[0 + ldim] = intr->gamma;
  A[1 + ldim] = intr->beta;

  A[0 + ldim*2] = intr->c[0];
  A[1 + ldim*2] = intr->c[1];
  A[2 + ldim*2] = 1.0;
}

void
cp_matinv_intrinsic (double *A, const int ldim, const intrinsic_param_t *intr)
{
  int i, j;
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          A[j + ldim*i] = 0.0;
        }
    }

  A[0] = 1.0 / intr->alpha;

  A[0 + ldim] = -intr->gamma / intr->alpha / intr->beta;
  A[1 + ldim] = 1.0 / intr->beta;

  A[0 + ldim*2] = (-intr->beta*intr->c[0] + intr->gamma*intr->c[1]) / intr->alpha / intr->beta;
  A[1 + ldim*2] = -intr->c[1] / intr->beta;
  A[2 + ldim*2] = 1.0;
}

/* result = [R(q1) t1] * [R(q2) t2] */
void
cp_mult_extrinsics (extrinsic_param_t *result, const extrinsic_param_t *ext1, const extrinsic_param_t *ext2)
{
  double temp[3];
  int i;

  quat_mult (result->q, ext1->q, ext2->q);

  quat_rot (temp, ext1->q, ext2->t);
  for (i = 0; i < 3; ++i)
    {
      result->t[i] = temp[i] + ext1->t[i];
    }
}

/* result = ext^(-1) */
void
cp_inv_extrinsic (extrinsic_param_t *result, const extrinsic_param_t *ext)
{
  int i;

  quat_conj (result->q, ext->q);

  quat_irot (result->t, ext->q, ext->t);
  for (i = 0; i < 3; ++i)
    {
      result->t[i] = -result->t[i];
    }
}

/* copy function */
void
cp_copy_intrinsic (intrinsic_param_t *dst, const intrinsic_param_t *src)
{
  if (dst == NULL || src == NULL)
    {
      return;
    }

  dst->alpha = src->alpha;
  dst->beta  = src->beta;
  dst->gamma = src->gamma;
  dst->c[0]  = src->c[0];
  dst->c[1]  = src->c[1];
}

void
cp_copy_extrinsic (extrinsic_param_t *dst, const extrinsic_param_t *src)
{
  int i;

  if (dst == NULL || src == NULL)
    {
      return;
    }

  quat_copy (dst->q, src->q);
  for (i = 0; i < 3; ++i)
    {
      dst->t[i] = src->t[i];
    }
}

void
cp_copy_distortion (distortion_param_t *dst, const distortion_param_t *src)
{
  int i;

  if (dst == NULL || src == NULL)
    {
      return;
    }

  for (i = 0; i < 5; ++i)
    {
      dst->elem[i] = src->elem[i];
    }
  dst->dim = src->dim;
}

void
cp_copy_camera_param (camera_param_t *dst, const camera_param_t *src)
{
  if (dst == NULL || src == NULL)
    {
      return;
    }

  cp_copy_intrinsic (&dst->intr, &src->intr);
  cp_copy_distortion (&dst->dist, &src->dist);
  cp_copy_extrinsic (&dst->ext, &src->ext);
}

/* output parameters */
void
cp_print_intrinsic (FILE *fp, const intrinsic_param_t *intr)
{
  fprintf (fp, "% 22.16g ", intr->alpha);
  fprintf (fp, "% 22.16g ", intr->gamma);
  fprintf (fp, "% 22.16g\n", intr->c[0]);

  fprintf (fp, "% 22.16g ", 0.0);
  fprintf (fp, "% 22.16g ", intr->beta);
  fprintf (fp, "% 22.16g\n", intr->c[1]);
  
  fprintf (fp, "% 22.16g ", 0.0);
  fprintf (fp, "% 22.16g ", 0.0);
  fprintf (fp, "% 22.16g\n", 1.0);
}

void
cp_print_extrinsic (FILE *fp, const extrinsic_param_t *ext, const int output_4th_row)
{
  int i, j;
  double R[3*3];
  quat_R_from_q (R, 3, ext->q);

  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          fprintf (fp, "% 22.16g ", R[3*j + i]);
        }
      fprintf (fp, "% 22.16g\n", ext->t[i]);
    }

  if (output_4th_row)
    {
      for (i = 0; i < 3; ++i)
        {
          fprintf (fp, "% 22.16g ", 0.0);
        }
      fprintf (fp, "% 22.16g\n", 1.0);
    }
}

void
cp_print_distortion (FILE *fp, const distortion_param_t *dist)
{
  int i;

  for (i = 0; i < dist->dim; ++i)
    {
      fprintf (fp, "% 22.16g ", dist->elem[i]);
    }
  fprintf (fp, "\n");
}
