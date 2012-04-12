/*
 calib_proc.hpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#ifndef CALIB_PROC_HPP
#define CALIB_PROC_HPP

#include "calib_data.h"
#include "camera_param.h"

#define CALIB_FIX_SHEAR              (1)
#define CALIB_ZERO_DIST              (1 << 2)
#define CALIB_ZERO_TANGENT_DIST      (1 << 3)
#define CALIB_ZERO_HIGHER_ORDER_DIST (1 << 4)

#define CALIB_EPS 1.0e-12
#define CALIB_MAX_ITER 10000
#define CALIB_MIN_UPDATE 1.0e-6

typedef struct tag_calib_opt
{
  int optimize_flag;
  int no_relative_pose_estimation;
  int no_grid_regularization;

  int max_iter;
  double update_limit;
  int force_norm_1;

  int verbose_mode;
} calib_opt_t;

const static calib_opt_t CALIB_OPT_INIT = {CALIB_FIX_SHEAR, 0, 0, CALIB_MAX_ITER, CALIB_MIN_UPDATE, 0, 0};

double calibrate_cameras (camera_param_t *camera_params, const cdata_t *cdata, const calib_opt_t *opt);

#endif /* CALIB_PROC_HPP */
