/* -*- mode: c; coding: utf-8 -*-
 optimizer.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <cv.h>

#include "optimizer.h"

const optimizer_context_t OPTI_CXT_INIT = {NULL, NULL, opti_update_default, -1, 1000};

#ifdef DEBUG_MULTICALIB
static void print_vec(CvMat *v)
{
  int i;
  for (i = 0; i < v->rows; ++i) {
    printf("% 10.3g ", v->data.db[i]);
  }
  printf("\n");
}

static void print_mat(CvMat *m)
{
  int i, j;
  for (i = 0; i < m->rows; ++i) {
    for (j = 0; j < m->cols; ++j) {
      printf("% 10.3g ", m->data.db[i * m->step + j]);
    }
    printf("\n");
  }
  printf("\n");
}
#endif /* DEBUG_MULTICALIB */

double optimize_lm(optimizer_context_t *cxt, CvMat *param)
{
  const int max_iter = cxt->max_iter;
  const int param_dim = param->rows;
  const int dparam_dim = cxt->num_diff_params < 0 ? param_dim : cxt->num_diff_params;

  int iter;
  double s = 1.0e-9;

  CvMat *rev = NULL, *param_rev = NULL, *jacobi = NULL, *jacobi_rev = NULL;
  CvMat *hesse = NULL, *hesse_mod = NULL;

  double error = -1.0;
  double norm2 = HUGE_VAL;

  /* allocate memories */
  rev = cvCreateMat (dparam_dim, 1, CV_64FC1);
  if (rev == NULL)
    {
      goto error_rev;
    }
  param_rev = cvCreateMat (param_dim, 1, CV_64FC1);
  if (param_rev == NULL)
    {
      goto error_param_rev;
    }
  jacobi = cvCreateMat (dparam_dim, 1, CV_64FC1);
  if (jacobi == NULL)
    {
      goto error_jacobi;
    }
  jacobi_rev = cvCreateMat (dparam_dim, 1, CV_64FC1);
  if (jacobi_rev == NULL)
    {
      goto error_jacobi_rev;
    }
  hesse = cvCreateMat (dparam_dim, dparam_dim, CV_64FC1);
  if (hesse == NULL)
    {
      goto error_hesse;
    }
  hesse_mod = cvCreateMat (dparam_dim, dparam_dim, CV_64FC1);
  if (hesse_mod == NULL)
    {
      goto error_hesse_mod;
    }

  /* initial value */
  error = cxt->func (cxt, param, jacobi, hesse);
  norm2 = cvDotProduct (jacobi, jacobi);

  /* Levenberg-Marquardt method */
  for (iter = 0; iter < max_iter; ++iter) {
    double prev_error = error;
    double prev_norm2 = norm2;
    double rho = 0.0;
    int i;

    /* compute an update value */
    do {
      double est;

      cvCopy (hesse, hesse_mod, NULL);
      //print_mat(hesse);
      for (i = 0; i < dparam_dim; ++i)
        {
          cvmSet (hesse_mod, i, i, cvmGet (hesse_mod, i, i) + s);
        }

      /* solve system for a candidate of update value */
      cvSolve (hesse_mod, jacobi, rev, CV_SVD);

      cxt->update (cxt, param, rev, param_rev);

      /* evaluate new parameter */
      error = cxt->func (cxt, param_rev, jacobi_rev, NULL);
      norm2 = cvDotProduct (jacobi_rev, jacobi_rev);

      /* evaluate goodness of quadratic approximation */
      est = cvDotProduct (rev, jacobi);
      rho = (est >= 0.0) ? (prev_error - error) / est : 0.0;
      if (rho > 0.5)
        {
          if (s > 1.0e-128) {
            s /= 2.0;
          }
          else {
            s = 0.0;
          }
        }
      else if (rho < 0.25) {
        if (s < 1.0e128) {
          s *= 2.0;
        }
        if (s < 1.0e-128) {
          s = 1.0e-128;
        }
      }
    } while (rho < 0.25 && (error > prev_error || norm2 > prev_norm2) && s < 1.0e128);

    /* update parameter */
    cvCopy (param_rev, param, NULL);
    //print_vec (param);
  
    error = cxt->func (cxt, param, jacobi, hesse);
    if (error >= prev_error) {
      break;
    }
  }
  if (iter >= max_iter) {
    fprintf (stderr, "%s: warning: not converged.\n", __FUNCTION__);
  }

  /* release memoties */
  cvReleaseMat (&hesse_mod);
 error_hesse_mod:
  cvReleaseMat (&hesse);
 error_hesse:
  cvReleaseMat (&jacobi_rev);
 error_jacobi_rev:
  cvReleaseMat (&jacobi);
 error_jacobi:
  cvReleaseMat (&param_rev);
 error_param_rev:
  cvReleaseMat (&rev);
 error_rev:

  return error;
}

void opti_update_default(optimizer_context_t *cxt, CvMat *param, CvMat *rev, CvMat *result)
{
  const int param_dim = param->rows;
  int i;

  for (i = 0; i < param_dim; ++i) {
    cvmSet (result, i, 0, cvmGet (param, i, 0) - cvmGet (rev, i, 0));
  }
}
