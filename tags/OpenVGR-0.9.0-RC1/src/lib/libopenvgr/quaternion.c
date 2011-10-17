/*
 quaternion.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#include <stdio.h>
#include <math.h>

#include "quaternion.h"
#include "local.h"

void
quat_fprintf (FILE *fp, const char *fmt, const char sep, const quaternion_t q)
{
  int i;

  fprintf (fp, "(");

  /* real part */
  fprintf (fp, fmt, quat_re (q));
  fprintf (fp, ";%c", sep);

  /* imaginary part */
  for (i = 0; i < 3; ++i)
    {
      fprintf (fp, fmt, quat_im (q, i));
      if (i < 2)
        {
          fprintf (fp, ",%c", sep);
        }
    }

  fprintf (fp, ")\n");
}

void
quat_copy (quaternion_t dst, const quaternion_t src)
{
  int i;
  for (i = 0; i < 4; ++i)
    {
      dst[i] = src[i];
    }
}

void
quat_mult (quaternion_t result, const quaternion_t q1, const quaternion_t q2)
{
  double dot, cross[3];
  int i;

  s_cross3 (cross, &quat_im (q1, 0), &quat_im (q2, 0));
  for (i = 0; i < 3; ++i)
    {
      quat_im (result, i) = quat_re (q1) * quat_im (q2, i) + quat_re (q2) * quat_im (q1, i) + cross[i];
    }

  dot = s_dot3 (&quat_im (q1, 0), &quat_im (q2, 0));
  quat_re (result) = quat_re (q1) * quat_re (q2) - dot;
}

void
quat_conj (quaternion_t result, const quaternion_t q)
{
  int i;

  for (i = 0; i < 3; ++i)
    {
      quat_im (result, i) = -quat_im (q, i);
    }
  quat_re (result) = quat_re (q);
}

/* compute the squared norm of q (i.e., |q|^2) */
double
quat_norm2 (const quaternion_t q)
{
  double norm = 0.0;
  int i;

  for (i = 0; i < 4; ++i)
    {
      norm += q[i] * q[i];
    }

  return norm;
}

/* scale entries s.t. |q| == 1 */
double
quat_normalize (quaternion_t q)
{
  double norm = sqrt (quat_norm2 (q));
  int i;

  for (i = 0; i < 3; ++i)
    {
      quat_im (q, i) /= norm;
    }
  quat_re (q) /= norm;

  return norm;
}

/* rotate x as q*x*conj(q) */
void
quat_rot (double result[3], const quaternion_t q, const double x[3])
{
  double dot, cross[3];
  int i;

  dot = s_dot3 (&quat_im (q, 0), x);
  s_cross3 (cross, &quat_im (q, 0), x);

  for (i = 0; i < 3; ++i)
    {
      result[i] = 2.0*dot*quat_im (q, i) + 2.0*quat_re (q) * (quat_re (q)*x[i] + cross[i]) - x[i];
    }
}

/* rotate x as conj(q)*x*q */
void
quat_irot (double result[3], const quaternion_t q, const double x[3])
{
  double dot, cross[3];
  int i;

  dot = s_dot3 (&quat_im (q, 0), x);
  s_cross3 (cross, &quat_im (q, 0), x);

  for (i = 0; i < 3; ++i)
    {
      result[i] = 2.0*dot*quat_im (q, i) + 2.0*quat_re (q) * (quat_re (q)*x[i] - cross[i]) - x[i];
    }
}

/* create quaternion that rotates a point along vector (x,y,z) by theta [rad] */
void
quat_make_from_rvec (quaternion_t result, const double theta, const double x, const double y, const double z)
{
  double norm = sqrt (x*x + y*y + z*z);
  double vec[3] = {x, y, z};
  int i;

  if (norm < QUAT_EPS)
    {
      quat_copy (result, quat_one);
      return;
    }

  for (i = 0; i < 3; ++i)
    {
      quat_im (result, i) = sin (theta/2.0) * vec[i]/norm;
    }
  quat_re (result) = cos (theta/2.0);

  //quat_normalize (result);
}

/* conversion between quaternion <-> rotation matrix (column major) */
void
quat_R_from_q (double *R, const int ldim, const quaternion_t q)
{
  /* 1st column */
  R[0] = 2.0 * (quat_im (q, 0)*quat_im (q, 0) + quat_re (q)*quat_re (q)) - 1.0;
  R[1] = 2.0 * (quat_im (q, 0)*quat_im (q, 1) + quat_im (q, 2)*quat_re (q));
  R[2] = 2.0 * (quat_im (q, 0)*quat_im (q, 2) - quat_im (q, 1)*quat_re (q));

  /* 2nd coulmn */
  R[ldim + 0] = 2.0 * (quat_im (q, 0)*quat_im (q, 1) - quat_im (q, 2)*quat_re (q));
  R[ldim + 1] = 2.0 * (quat_im (q, 1)*quat_im (q, 1) + quat_re (q)*quat_re (q)) - 1.0;
  R[ldim + 2] = 2.0 * (quat_im (q, 1)*quat_im (q, 2) + quat_im (q, 0)*quat_re (q));

  /* 3rd coulmn */
  R[2*ldim + 0] = 2.0 * (quat_im (q, 0)*quat_im (q, 2) + quat_im (q, 1)*quat_re (q));
  R[2*ldim + 1] = 2.0 * (quat_im (q, 1)*quat_im (q, 2) - quat_im (q, 0)*quat_re (q));
  R[2*ldim + 2] = 2.0 * (quat_im (q, 2)*quat_im (q, 2) + quat_re (q)*quat_re (q)) - 1.0;
}

void
quat_q_from_R (quaternion_t q, const double *R, const int ldim)
{
  double c2;

  c2 = ((R[0] + R[ldim + 1] + R[2*ldim + 2]) + 1.0) / 4.0;
  if (c2 < 0.0 || 1.0 < c2)
    {
      quat_copy (q, quat_one);
      return;
    }

  quat_re (q) = sqrt (c2);
  if (quat_re (q) > QUAT_EPS)
    {
      quat_im (q, 0) = (R[ldim + 2] - R[2*ldim + 1]) / 4.0 / quat_re (q);
      quat_im (q, 1) = (R[2*ldim + 0] - R[2]) / 4.0 / quat_re (q);
      quat_im (q, 2) = (R[1] - R[ldim]) / 4.0 / quat_re (q);
    }
  else
    {
      quat_im (q, 0) = sqrt((R[0] + 1.0)/2.0 - c2);
      quat_im (q, 1) = sqrt((R[ldim + 1] + 1.0)/2.0 - c2);
      quat_im (q, 2) = sqrt((R[2*ldim + 2] + 1.0)/2.0 - c2);
    }

  quat_normalize (q);
}
