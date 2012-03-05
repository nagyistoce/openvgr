/* -*- mode: c; coding: utf-8 -*-
 optimizer.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <cv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* forward declaration */
typedef struct tag_optimizer_context optimizer_context_t;

/*
  val = func(param)

  (INPUT)  param : m-vector
  (OUTPUT) val   : n-vector
  (OUTPUT) jacobi: nxm matrix
  (OUTPUT) hesse : mxm matrix
  (INPUT)  data  : additional data for computing function value
*/
typedef double (*objective_func_t)(optimizer_context_t *cxt, CvMat *param, CvMat *jacobi, CvMat *hesse);

typedef void (*update_func_t)(optimizer_context_t *cxt, CvMat *param, CvMat *rev, CvMat *result);

struct tag_optimizer_context
{
  void *data;

  objective_func_t func;
  update_func_t update;

  int num_diff_params;

  int max_iter;
};

double optimize_lm(optimizer_context_t *cxt, CvMat *param);

void opti_update_default(optimizer_context_t *cxt, CvMat *param, CvMat *rev, CvMat *result);

extern const optimizer_context_t OPTI_CXT_INIT;

#ifdef __cplusplus
}
#endif

#endif /* OPTIMIZER_H */
