/*
 quaternion.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <satoshi.kawabata@aist.go.jp>

 $Date::                            $
*/

#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdio.h>

typedef double quaternion_t[4];

#define QUAT_EPS 1.0e-15

#define QUAT_INIT_ZERO {0.0, 0.0, 0.0, 0.0}
#define QUAT_INIT_ONE  {0.0, 0.0, 0.0, 1.0}

const static quaternion_t quat_zero = QUAT_INIT_ZERO;
const static quaternion_t quat_one  = QUAT_INIT_ONE;

#define quat_re(q) (q)[3]
#define quat_im(q, i) (q)[(i)]

/* print the entries of q */
void quat_fprintf (FILE *fp, const char *fmt, const char sep, const quaternion_t q);
#define quat_printf(fmt, sep, q) quat_fprintf(stdout, (fmt), (sep), (q))
#define quat_fprint(fp, q) quat_fprintf ((fp), "% 10.3g", ' ', (q))
#define quat_print(q) quat_fprintf (stdout, "% 10.3g", ' ', (q))

void quat_copy (quaternion_t dst, const quaternion_t src);
void quat_mult (quaternion_t result, const quaternion_t q1, const quaternion_t q2);
void quat_conj (quaternion_t result, const quaternion_t q);

/* compute the squared norm of q (i.e., |q|^2) */
double quat_norm2 (const quaternion_t q);

/* scale entries s.t. |q| == 1 */
double quat_normalize (quaternion_t q);

/* rotate x as q*x*conj(q) */
void quat_rot (double result[3], const quaternion_t q, const double x[3]);
/* rotate x as conj(q)*x*q */
void quat_irot (double result[3], const quaternion_t q, const double x[3]);

/* create quaternion that rotates a point along vector (x,y,z) by theta [rad] */
void quat_make_from_rvec (quaternion_t result, const double theta, const double x, const double y, const double z);

/* conversion between quaternion <-> rotation matrix (column major) */
void quat_R_from_q (double *R, const int ldim, const quaternion_t q);
void quat_q_from_R (quaternion_t q, const double *R, const int ldim);

#endif /* QUATERNION_H */
