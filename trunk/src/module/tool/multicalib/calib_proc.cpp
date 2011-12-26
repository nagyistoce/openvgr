/*
 calib_proc.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <cv.h>

#include "local.h"
#include "calib_proc.h"

/* for inner use */
#define CALIB_FIX_INTRINSIC          (1 <<  5)
#define CALIB_FIX_DISTORTION         (1 <<  6)
#define CALIB_FIX_CAMERA_ORIENTATION (1 <<  7)
#define CALIB_FIX_TRANSLATION        (1 <<  8)
#define CALIB_FIX_PLANE_ORIENTATION  (1 <<  9)
#define CALIB_FORCE_Q_NORM_1         (1 << 10)

#if defined (_OPENMP) && defined (__GNUC__)
#  pragma message "OpenMP(TM) enabled"
#endif

typedef struct tag_trans2D
{
  double theta;
  double t[2];
} trans2D_t;

static double individual_calibration (camera_param_t *camera_params, extrinsic_param_t *plane_poses, const cdata_t *cdata, const calib_opt_t *opt);
static void compute_initial_poses (camera_param_t *camera_params, const extrinsic_param_t *plane_poses, const cdata_t *cdata, const calib_opt_t *opt);
static void compute_displacement (trans2D_t *t2ds, const camera_param_t *camera_params, const cdata_t *cdata, const extrinsic_param_t *plane_poses, const calib_opt_t *opt);
static double optimize_camera_parameters (camera_param_t *camera_params, extrinsic_param_t *plane_poses, const trans2D_t *t2ds, const cdata_t *cdata, const calib_opt_t *opt, const int additional_flag);

static CvSize guess_image_size (CvMat *imagePoints);

static void convert_rodrigues2quat (const CvMat *rvec, quaternion_t q);
static void extract_plane_equation (double coeff[4], const extrinsic_param_t *ext);
static void compute_rotation (camera_param_t *camera_params, double (*coeffs)[4], const int nobserv, const int ncamera);
static void compute_translation (camera_param_t *camera_params, double (*coeffs)[4], const int nobserv, const int ncamera);

/* plane coordinate = homo * image coordinate */
static void construct_homography (CvMat *homo, const camera_param_t *camera_param, const extrinsic_param_t *plane);
static void construct_homography_inv (CvMat *homo, const camera_param_t *camera_param, const extrinsic_param_t *plane);

static void compute_undistorted_points (CvMat *points, const camera_param_t *camera_param, const cdata_observed_points_t *op);

static void estimate_nearest_point (double refpos[2], CvMat *Homo, CvMat *Homo_inv, const double imgpos[2], const double interval);

static void trans2D_apply (double y[2], const trans2D_t *t2d, const double x[2]);

static double calib_calc_error (const camera_param_t *camera_params, const extrinsic_param_t *plane_poses, const trans2D_t *t2ds, const cdata_t *cdata, CvMat *JtJ, CvMat *JtF, const double *lambdas, const int flag);
static void update_camera_params (camera_param_t *camera_params, extrinsic_param_t *plane_poses, double *lambdas, const CvMat *rev, const camera_param_t *camera_params_current, const int ncamera, const extrinsic_param_t *plane_poses_current, const int nobserv, const double *lambdas_current, const int flag);

#ifndef USE_INFINITESIMAL_ROTATION
static double mod_lambda (CvMat *JtJ, CvMat *JtF, const int pos, const int lpos, const double l, const quaternion_t q);
#endif

/****************************************************************/
/* higher level functions for multi-camera calibration */
/****************************************************************/
double
calibrate_cameras (camera_param_t *camera_params, const cdata_t *cdata, const calib_opt_t *opt)
{
  double error = 42;
  extrinsic_param_t *plane_poses = NULL;
  trans2D_t *t2ds = NULL;

  if (camera_params == NULL || cdata == NULL)
    {
      return -1.0;
    }

  //cdata_write (cdata, stdout);

  plane_poses = (extrinsic_param_t *) malloc (sizeof (extrinsic_param_t) * cdata->num_observations * cdata->num_cameras);
  if (plane_poses == NULL)
    {
      return -1.0;
    }

  t2ds = (trans2D_t *) malloc (sizeof (trans2D_t) * cdata->num_observations * cdata->num_cameras);
  if (t2ds == NULL)
    {
      free (plane_poses);
      return -1.0;
    }

  /* perform Zhang's calibration for each camera */
  error = individual_calibration (camera_params, plane_poses, cdata, opt);

  if (opt->verbose_mode && cdata->num_cameras > 1)
    {
      printf ("error (indivisual): %f\n", error);
    }

  if (cdata->num_cameras > 1)
    {
      /* compute initial poses of cameras */
      compute_initial_poses (camera_params, plane_poses, cdata, opt);
      compute_displacement (t2ds, camera_params, cdata, plane_poses, opt);
    }

  /* optimize all parameters */
  error = optimize_camera_parameters (camera_params, plane_poses, t2ds, cdata, opt, (opt->force_norm_1 ? CALIB_FORCE_Q_NORM_1 : 0));

  free (t2ds);
  free (plane_poses);

  return error;
}


static double
individual_calibration (camera_param_t *camera_params, extrinsic_param_t *plane_poses, const cdata_t *cdata, const calib_opt_t *opt)
{
  const int ncamera = cdata->num_cameras, nobserv = cdata->num_observations;
  double mean_err = 0.0;
  int Nsum = 0;
  int i, j, k;

#ifdef _OPENMP
#  pragma omp parallel for private (i, j, k) reduction (+: mean_err)
#endif
  for (i = 0; i < ncamera; ++i)
    {
      CvMat *objectPoints, *imagePoints, *pointCounts, *cameraMatrix, *distCoeffs, *rvecs, *tvecs;
      int N = 0, pos;

      CvMat *projectedPoints;
      double error;

      for (j = 0; j < nobserv; ++j)
        {
          N += cdata->data[ncamera*j + i].num_points;
        }

      /* allocate memories */
      objectPoints = cvCreateMat (3, N, CV_64FC1);
      imagePoints = cvCreateMat (2, N, CV_64FC1);
      pointCounts = cvCreateMat (nobserv, 1, CV_32SC1);
      cameraMatrix = cvCreateMat (3, 3, CV_64FC1); 
      distCoeffs = cvCreateMat (opt->optimize_flag & CALIB_ZERO_HIGHER_ORDER_DIST ? 4 : 5, 1, CV_64FC1);
      rvecs = cvCreateMat (nobserv, 3, CV_64FC1);
      tvecs = cvCreateMat (nobserv, 3, CV_64FC1);

      projectedPoints = cvCreateMat (2, N, CV_64FC1);

      /* initialize the cameraMatrix */
      cvSetIdentity (cameraMatrix, cvRealScalar (1.0));

      /* substitute reference coordinates and observed image coordinates */
      pos = 0;
      for (j = 0; j < nobserv; ++j)
        {
          const int npoint = cdata->data[ncamera*j + i].num_points;
          
          for (k = 0; k < npoint; ++k)
            {
              cvmSet (objectPoints, 0, pos, cdata->data[ncamera*j + i].ref[k][0]);
              cvmSet (objectPoints, 1, pos, cdata->data[ncamera*j + i].ref[k][1]);
              cvmSet (objectPoints, 2, pos, 0.0);

              cvmSet (imagePoints, 0, pos, cdata->data[ncamera*j + i].cam[k][0]);
              cvmSet (imagePoints, 1, pos, cdata->data[ncamera*j + i].cam[k][1]);

              ++pos;
            }
          pointCounts->data.i[j] = npoint;
        }

      /* calibrate camera */
      cvCalibrateCamera2 (objectPoints, imagePoints, pointCounts, guess_image_size(imagePoints), cameraMatrix, distCoeffs, rvecs, tvecs, (opt->optimize_flag & CALIB_ZERO_TANGENT_DIST ? CV_CALIB_ZERO_TANGENT_DIST : 0));

      /* compute error */
      pos = 0;
      for (j = 0; j < nobserv; ++j)
        {
          CvMat objectPoint, projectedPoint, rvec, tvec;

          cvGetCols (objectPoints, &objectPoint, pos, pos + pointCounts->data.i[j]);
          cvGetCols (projectedPoints, &projectedPoint, pos, pos + pointCounts->data.i[j]);
          cvGetRow (rvecs, &rvec, j);
          cvGetRow (tvecs, &tvec, j);

          cvProjectPoints2 (&objectPoint, &rvec, &tvec, cameraMatrix, distCoeffs, &projectedPoint, NULL, NULL, NULL, NULL, NULL, 0);

          pos += pointCounts->data.i[j];
        }
      error = 0.0;
      for (j = 0; j < N; ++j)
        {
          error += pow (cvmGet (projectedPoints, 0, j) - cvmGet (imagePoints, 0, j), 2) + pow (cvmGet (projectedPoints, 1, j) - cvmGet (imagePoints, 1, j), 2);
        }
      mean_err += error;
      Nsum += N;
      error = sqrt (error / (double)N);


#if 1
#ifdef _OPENMP
#  pragma omp critical
#endif
      if (opt->verbose_mode)
        {
          printf ("camera %d: err = %g\n", i, error);
          for (j = 0; j < cameraMatrix->rows; ++j)
            {
              for (k = 0; k < cameraMatrix->cols; ++k)
                {
                  printf ("% 11.5g ", cvmGet (cameraMatrix, j, k));
                }
              printf ("\n");
            }
          for (j = 0; j < distCoeffs->rows; ++j)
            {
              printf ("% 11.5g ", cvmGet (distCoeffs, j, 0));
            }
          printf ("\n");
        }
#endif

      /* copy the result to camera_params */
      /* intrinsic */
      camera_params[i].intr.alpha = cvmGet (cameraMatrix, 0, 0);
      camera_params[i].intr.beta  = cvmGet (cameraMatrix, 1, 1);
      camera_params[i].intr.gamma = cvmGet (cameraMatrix, 0, 1);
      camera_params[i].intr.c[0]  = cvmGet (cameraMatrix, 0, 2);
      camera_params[i].intr.c[1]  = cvmGet (cameraMatrix, 1, 2);
      
      /* distortion */
      for (j = 0; j < cvGetDimSize (distCoeffs, 0); ++j)
        {
          camera_params[i].dist.elem[j] = cvmGet (distCoeffs, j, 0);
        }
      if (!(opt->optimize_flag & CALIB_ZERO_DIST))
        {
          if (!(opt->optimize_flag & CALIB_ZERO_HIGHER_ORDER_DIST))
            {
              camera_params[i].dist.dim = 5;
            }
          else if (!(opt->optimize_flag & CALIB_ZERO_TANGENT_DIST))
            {
              camera_params[i].dist.dim = 4;
            }
          else
            {
              camera_params[i].dist.dim = 2;
            }
        }
      else
        {
          camera_params[i].dist.dim = 0;
        }

      /* extrinsic (R = I, t = 0 at this moment) */
      quat_copy (camera_params[i].ext.q, quat_one);
      for (j = 0; j < 3; ++j)
        {
          camera_params[i].ext.t[j] = 0.0;
        }

      /* convert rvec to quaternion (poses of observed reference plane) */
      for (j = 0; j < nobserv; ++j)
        {
          CvMat rvec;

          cvGetRow (rvecs, &rvec, j);
          convert_rodrigues2quat (&rvec, plane_poses[ncamera*j + i].q);

          for (k = 0; k < 3; ++k)
            {
              plane_poses[ncamera*j + i].t[k] = cvmGet (tvecs, j, k);
            }
        }

      /* free memories */
      cvReleaseMat (&projectedPoints);

      cvReleaseMat (&tvecs);
      cvReleaseMat (&rvecs);
      cvReleaseMat (&distCoeffs);
      cvReleaseMat (&cameraMatrix);
      cvReleaseMat (&pointCounts);
      cvReleaseMat (&imagePoints);
      cvReleaseMat (&objectPoints);
    }
  mean_err = sqrt (mean_err / (double) Nsum);
  
  return mean_err;
}

static void
compute_initial_poses (camera_param_t *camera_params, const extrinsic_param_t *plane_poses, const cdata_t *cdata, const calib_opt_t *opt)
{
  const int nobserv = cdata->num_observations, ncamera = cdata->num_cameras;
  double (*coeffs)[4] = NULL;
  int i, j;

  coeffs = (double (*)[4]) malloc (sizeof (double[4]) * nobserv * ncamera);
  if (coeffs == NULL)
    {
      return;
    }

  /* compute plane equations of each estimated pose of reference plane */
  for (i = 0; i < nobserv; ++i)
    {
      for (j = 0; j < ncamera; ++j)
        {
          extract_plane_equation (coeffs[ncamera*i + j], &plane_poses[ncamera*i + j]);
        }
    }

  /* estimate relative poses of cameras using the plane equations */
  compute_rotation (camera_params, coeffs, nobserv, ncamera);
  compute_translation (camera_params, coeffs, nobserv, ncamera);

  if (opt->verbose_mode)
    {
      for (i = 0; i < ncamera; ++i)
        {
          printf ("camera %d\n", i);
          printf ("q: "); quat_print (camera_params[i].ext.q);
          printf ("t: % f % f % f\n", camera_params[i].ext.t[0], camera_params[i].ext.t[1], camera_params[i].ext.t[2]);
          printf ("\n");
        }
    }

  free (coeffs);
}

static void
compute_displacement (trans2D_t *t2ds, const camera_param_t *camera_params, const cdata_t *cdata, const extrinsic_param_t *plane_poses, const calib_opt_t *opt)
{
  const int nobserv = cdata->num_observations, ncamera = cdata->num_cameras;
  int i, j, k;

  /* displacements of the first camera are always (0, 0), 0 */
  for (i = 0; i < nobserv; ++i)
    {
      t2ds[ncamera*i].t[0] = 0.0;
      t2ds[ncamera*i].t[1] = 0.0;
      t2ds[ncamera*i].theta = 0.0;
    }

  for (i = 0; i < nobserv; ++i)
    {
#ifdef _OPENMP
#  pragma omp parallel for private (j, k)
#endif
      for (j = 1; j < ncamera; ++j)
        {
          const int npoint = cdata->data[ncamera*i + j].num_points;
          CvMat *cam_points, *ref_points;
          CvMat Homo, Homo_inv;
          double h[3*3], h_inv[3*3], mean[2], ref_mean[2], c, s;

          int l;

          /* compute undistorted points of camera j at i-th observation */
          cam_points = cvCreateMat (1, npoint, CV_64FC2);
          compute_undistorted_points (cam_points, &camera_params[j], &cdata->data[ncamera*i + j]);

          /* compute homography for backprojecting image points onto reference plane */
          cvInitMatHeader (&Homo, 3, 3, CV_64FC1, h, CV_AUTOSTEP);
          construct_homography (&Homo, &camera_params[j], &plane_poses[ncamera*i]);
          cvInitMatHeader (&Homo_inv, 3, 3, CV_64FC1, h_inv, CV_AUTOSTEP);
          construct_homography_inv (&Homo_inv, &camera_params[j], &plane_poses[ncamera*i]);

          /* backproject each observed point to reference plane */
          ref_points = cvCreateMat (1, npoint, CV_64FC2);
          mean[0] = mean[1] = 0.0;
          ref_mean[0] = ref_mean[1] = 0.0;
          for (k = 0; k < npoint; ++k)
            {
              double *pos = (double *) CV_MAT_ELEM_PTR (*cam_points, 0, k);
              double *bp = (double *) CV_MAT_ELEM_PTR (*ref_points, 0, k);
              double *ref = cdata->data[ncamera*i + j].ref[k];

              estimate_nearest_point (bp, &Homo, &Homo_inv, pos, cdata->interval);

              for (l = 0; l < 2; ++l)
                {
                  mean[l] += bp[l];
                  ref_mean[l] += ref[l];
                }
            }

          for (k = 0; k < 2; ++k)
            {
              mean[k] /= (double) npoint;
              ref_mean[k] /= (double) npoint;
            }

          c = s = 0.0;
          for (k = 0; k < npoint; ++k)
            {
              double *bp = (double *) CV_MAT_ELEM_PTR (*ref_points, 0, k);
              double *ref = cdata->data[ncamera*i + j].ref[k];

              double v[2], ref_v[2];

              for (l = 0; l < 2; ++l)
                {
                  v[l] = bp[l] - mean[l];
                  ref_v[l] = ref[l] - ref_mean[l];
                }

              c += ref_v[0]*v[0] + ref_v[1]*v[1];
              s += ref_v[0]*v[1] - ref_v[1]*v[0];
            }

          /* round the difference to integral multiple of the interval of adjacent patterns */
          t2ds[ncamera*i + j].theta = cdata->rotation_deg * floor (atan2 (s, c) * 180.0 / M_PI / cdata->rotation_deg + 0.5) * M_PI / 180.0;
          for (k = 0; k < 2; ++k)
            {
              const double theta = t2ds[ncamera*i + j].theta;
              double rm[2];

              rm[0] = cos (theta) * ref_mean[0] - sin (theta) * ref_mean[1];
              rm[1] = sin (theta) * ref_mean[0] + cos (theta) * ref_mean[1];

              t2ds[ncamera*i + j].t[k] = floor ((mean[k] - rm[k]) / cdata->interval + 0.5) * cdata->interval;
            }

#ifdef _OPENMP
#  pragma omp critical
#endif
          if (opt->verbose_mode)
            {
              if (fabs(t2ds[ncamera*i + j].t[0]) >= 1 || fabs(t2ds[ncamera*i + j].t[1]) >= 1)
                {
                  fprintf (stderr, "plane %d, camera %d\n", i, j);
                  fprintf (stderr, "diff:   % f % f [unit], % f [deg]\n",
                           (mean[0] - ref_mean[0]) / cdata->interval,
                           (mean[1] - ref_mean[1]) / cdata->interval,
                           atan2(s, c) * 180.0/M_PI);

                  fprintf (stderr, "offset: % f % f [unit]. % f [deg]\n",
                           t2ds[ncamera*i + j].t[0] / cdata->interval,
                           t2ds[ncamera*i + j].t[1] / cdata->interval,
                           t2ds[ncamera*i + j].theta * 180.0/M_PI);
                  fprintf (stderr, "\n");
                }
            }

          cvReleaseMat (&ref_points);
          cvReleaseMat (&cam_points);
        }
    }
}

static double
optimize_camera_parameters (camera_param_t *camera_params, extrinsic_param_t *plane_poses, const trans2D_t *t2ds, const cdata_t *cdata, const calib_opt_t *opt, const int additional_flag)
{
  const int flag = opt->optimize_flag | additional_flag;

  const int nobserv = cdata->num_observations, ncamera = cdata->num_cameras;

  const int num_intr_param = 4 + ((flag & CALIB_FIX_SHEAR) ? 0 : 1);
  const int num_dist_param = (flag & CALIB_ZERO_DIST) ? 0 : (2 + ((flag & CALIB_ZERO_TANGENT_DIST) ? 0 : 2) + ((flag & CALIB_ZERO_HIGHER_ORDER_DIST) ? 0 : 1));
  const int num_ext_param = 4 + 3;

  const int num_cam_param = num_intr_param + num_dist_param;
#ifdef USE_INFINITESIMAL_ROTATION
  const int num_lambda = 0;
#else
  const int num_lambda = ncamera-1 + nobserv;
#endif
  const int num_params = num_cam_param * ncamera + num_ext_param * (ncamera-1 + nobserv) + num_lambda;

  double error = 42, prev_error, norm, prev_norm, scale = 1.0e-6;
  CvMat *JtJ, *JtF, *H, *rev, *JtJtemp, *JtFtemp;
  camera_param_t *new_camera_params = NULL;
  extrinsic_param_t *new_plane_poses = NULL;
  double *lambdas = NULL, *new_lambdas = NULL;

  int i, j, iter, npoint = 0;
  double min_thres;

  /* set zero to distortion coefficients if needed */
  for (i = 0; i < ncamera; ++i)
    {
      if (flag & CALIB_ZERO_DIST)
        {
          for (j = 0; j < 5; ++j)
            {
              camera_params[i].dist.elem[j] = 0.0;
            }
        }
      else
        {
          if (flag & CALIB_ZERO_TANGENT_DIST)
            {
              for (j = 0; j < 2; ++j)
                {
                  camera_params[i].dist.elem[2 + j] = 0.0;
                }
            }
          if (flag & CALIB_ZERO_HIGHER_ORDER_DIST)
            {
              camera_params[i].dist.elem[4] = 0.0;
            }
        }
    }

  /* allocate memories */
  JtJ = cvCreateMat (num_params, num_params, CV_64FC1);
  JtF = cvCreateMat (num_params, 1, CV_64FC1);

  H = cvCreateMat (num_params, num_params, CV_64FC1);
  rev = cvCreateMat (num_params, 1, CV_64FC1);

  JtJtemp = cvCreateMat (cvGetDimSize (JtJ, 0), cvGetDimSize (JtJ, 1), CV_64FC1);
  JtFtemp = cvCreateMat (cvGetDimSize (JtF, 0), cvGetDimSize (JtF, 1), CV_64FC1);
  
  new_camera_params = (camera_param_t *) malloc (sizeof (camera_param_t) * ncamera);
  if (new_camera_params == NULL)
    {
      goto error_new_cam;
    }
  for (i = 0; i < ncamera; ++i)
    {
      cp_copy_camera_param (&new_camera_params[i], &camera_params[i]);
    }

  new_plane_poses = (extrinsic_param_t *) malloc (sizeof (extrinsic_param_t) * ncamera * nobserv);
  if (new_plane_poses == NULL)
    {
      goto error_new_plane;
    }

#ifndef USE_INFINITESIMAL_ROTATION
  lambdas = (double *) malloc (sizeof (double) * num_lambda);
  if (lambdas == NULL)
    {
      goto error_lambdas;
    }
  for (i = 0; i < num_lambda; ++i)
    {
      lambdas[i] = 1.0;
    }

  new_lambdas = (double *) malloc (sizeof (double) * num_lambda);
  if (new_lambdas == NULL)
    {
      goto error_new_lambdas;
    }
#endif

  /* compute the initial value for optimization */
  error = calib_calc_error (camera_params, plane_poses, t2ds, cdata, JtJ, JtF, lambdas, flag);
  norm = cvNorm (JtF, NULL, CV_L2, NULL);

  /* count how many observed points we have */
  npoint = 0;
  for (i = 0; i < nobserv; ++i)
    {
      for (j = 0; j < ncamera; ++j)
        {
          npoint += cdata->data[ncamera*i + j].num_points;
        }
    }

  min_thres = pow (opt->update_limit * (double) npoint, 2);

  /* optimize by the Levenberg-Marquardt method */
  if (opt->verbose_mode)
    {
      fprintf (stderr, "optimizing...");
    }
  for (iter = 0; iter < opt->max_iter || opt->max_iter < 0; ++iter)
    {
      double rho, pred;

      prev_error = error;
      prev_norm = norm;

      /* compute new value */
      do
        {
          /* construct H */
          cvCopy (JtJ, H, NULL);
          for (i = 0; i < num_params; ++i)
            {
              CV_MAT_ELEM (*H, double, i, i) += scale * CV_MAT_ELEM (*H, double, i, i);
            }

          if (cvSolve (H, JtF, rev, CV_CHOLESKY) != 1)
            {
              cvSolve (H, JtF, rev, CV_SVD_SYM);
            }

          /* predict reduction of error */
          pred = 0.0;
          for (i = 0; i < num_params; ++i)
            {
              pred += 2.0 * cvmGet (JtF, i, 0) * cvmGet (rev, i, 0);
              for (j = 0; j < num_params; ++j)
                {
                  pred -= cvmGet (H, i, j) * cvmGet (rev, i, 0) * cvmGet (rev, j, 0);
                }
            }

          update_camera_params (new_camera_params, new_plane_poses, new_lambdas, rev, camera_params, ncamera, plane_poses, nobserv, lambdas, flag);
          error = calib_calc_error (new_camera_params, new_plane_poses, t2ds, cdata, JtJtemp, JtFtemp, new_lambdas, flag);
          norm = cvNorm (JtFtemp, NULL, CV_L2, NULL);

          rho = (prev_error - error) / pred;

          if ((rho < 0.25 && scale < 1.0e+128) || prev_error < error)
            {
              scale *= 2.0;
            }
        }
      while ((rho < 0.25 || (prev_error + CALIB_EPS < error && prev_norm + CALIB_EPS < norm)) && pred > CALIB_EPS);

      /* break if converged */
      if ((prev_norm <= norm + CALIB_EPS && prev_error <= error + CALIB_EPS) || (prev_error - error < min_thres) || norm < 1.0e-6)
        {
          //fprintf (stderr, "converged.\n");
          break;
        }

      if (0.75 < rho && scale > 1.0e-128)
        {
          scale /= 2.0;
        }

      /* copy new parameters for the next step */
      for (i = 0; i < ncamera; ++i)
        {
          cp_copy_camera_param (&camera_params[i], &new_camera_params[i]);
        }
      for (i = 0; i < nobserv; ++i)
        {
          cp_copy_extrinsic (&plane_poses[ncamera*i], &new_plane_poses[ncamera*i]);
        }
#ifndef USE_INFINITESIMAL_ROTATION
      for (i = 0; i < num_lambda; ++i)
        {
          lambdas[i] = new_lambdas[i];
        }
#endif

      /* compute derivertives and hessian matrix */
      error = calib_calc_error (camera_params, plane_poses, t2ds, cdata, JtJ, JtF, lambdas, flag);

      /* display progress */
      if (opt->verbose_mode && iter % 10 == 0 && iter > 0)
        {
          if (iter % 100 != 0)
            {
              fprintf (stderr, ".");
            }
          else
            {
              fprintf (stderr, "(%f)", sqrt (error / (double) npoint));
            }
        }
    }
  if (opt->verbose_mode)
    {
      if (iter < opt->max_iter)
        {
          fprintf (stderr, "done\n");
        }
      else
        {
          fprintf (stderr, "not converged. (iter = %d)\n", iter);
        }
    }

  /* normalize quaternions */
  for (i = 1; i < ncamera; ++i)
    {
      quat_normalize (camera_params[i].ext.q);
    }
  for (i = 0; i < nobserv; ++i)
    {
      quat_normalize (plane_poses[ncamera*i].q);
    }

  /* compute poses of reference planes from each camera view */
  for (i = 0; i < nobserv; ++i)
    {
      for (j = 1; j < ncamera; ++j)
        {
          cp_mult_extrinsics (&plane_poses[ncamera*i + j], &camera_params[j].ext, &plane_poses[ncamera*i]);
        }
    }

  /* compute the final error */
  error = calib_calc_error (camera_params, plane_poses, t2ds, cdata, NULL, NULL, NULL, 0);

#ifndef USE_INFINITESIMAL_ROTATION
  free (new_lambdas);
 error_new_lambdas:
  free (lambdas);
 error_lambdas:
#endif
  free (new_plane_poses);
 error_new_plane:
  free (new_camera_params);
 error_new_cam:
  cvReleaseMat (&JtFtemp);
  cvReleaseMat (&JtJtemp);
  cvReleaseMat (&rev);
  cvReleaseMat (&H);
  cvReleaseMat (&JtF);
  cvReleaseMat (&JtJ);

  return sqrt(error / (double) npoint);
}

static CvSize
guess_image_size (CvMat *imagePoints)
{
  int i, n;
  double x_mean = 0.0, d;
  const static int size[][2] = {
    {160, 120}, {320, 240}, {640, 480}, {800, 600}, {1024, 768}, {1280, 960}, {1600, 1200}
  };

  for (i = 0; i < cvGetDimSize (imagePoints, 1); ++i)
    {
      x_mean += cvmGet (imagePoints, 0, i);
    }
  x_mean /= (double) cvGetDimSize (imagePoints, 1);

  n = 0;
  d = fabs (2.0*x_mean - size[0][0]);
  for (i = 1; i < 7; ++i)
    {
      double dd = fabs (2.0*x_mean - size[i][0]);
      if (dd < d)
        {
          d = dd;
          n = i;
        }
    }

  return cvSize (size[n][0], size[n][1]);
}


/****************************************************************/
/* misc functions */
/****************************************************************/
static void
convert_rodrigues2quat (const CvMat *rvec, quaternion_t q)
{
  double theta = cvNorm (rvec, NULL, CV_L2, NULL);

  if (theta > CALIB_EPS)
    {
      int i;
      for (i = 0; i < 3; ++i)
        {
          quat_im (q, i) = sin (theta / 2.0) * cvmGet (rvec, 0, i) / theta;
        }
      quat_re (q) = cos (theta / 2.0);
    }
  else /* in case of quite small rotation */
    {
      quat_copy (q, quat_one);
    }
}

static void
extract_plane_equation (double coeff[4], const extrinsic_param_t *ext)
{
  double n[3];
  int i;

  /* normal vector (z-axis) */
  n[0] = 2.0 * (quat_im (ext->q, 0)*quat_im (ext->q, 2) + quat_im (ext->q, 1)*quat_re (ext->q));
  n[1] = 2.0 * (quat_im (ext->q, 1)*quat_im (ext->q, 2) - quat_im (ext->q, 0)*quat_re (ext->q));
  n[2] = 2.0 * (quat_im (ext->q, 2)*quat_im (ext->q, 2) + quat_re (ext->q)*quat_re (ext->q)) - 1.0;
  s_normalize (n);

  for (i = 0; i < 3; ++i)
    {
      coeff[i] = n[i];
    }
  coeff[3] = -s_dot3 (n, ext->t);
}

static void
compute_rotation (camera_param_t *camera_params, double (*coeffs)[4], const int nobserv, const int ncamera)
{
  int i;

#ifdef _OPENMP
#  pragma omp parallel for
#endif
  for (i = 1; i < ncamera; ++i)
    {
      double M[4*4], dot, cross[3];
      
      double sigma_elem[4], q[4*4];
      CvMat MM, sigma, MQ;
      int j, k, l;

      for (j = 0; j < 4; ++j)
        {
          for (k = 0; k < 4; ++k)
            {
              M[4*j + k] = 0.0;
            }
        }

      /* create data matrix */
      for (j = 0; j < nobserv; ++j)
        {
          dot = s_dot3 (coeffs[ncamera*j], coeffs[ncamera*j + i]);
          s_cross3 (cross, coeffs[ncamera*j], coeffs[ncamera*j + i]);
          
          for (k = 0; k < 3; ++k)
            {
              for (l = 0; l < 3; ++l)
                {
                  M[4*k + l] += coeffs[ncamera*j][k] * coeffs[ncamera*j + i][l] + coeffs[ncamera*j][l] * coeffs[ncamera*j + i][k];
                }
            }

          M[4*3 + 3] += dot;
          for (k = 0; k < 3; ++k)
            {
              M[4*k + k] -= dot;

              M[4*3 + k] += cross[k];
              M[4*k + 3] += cross[k];
            }
        }
      
      cvInitMatHeader (&MM, 4, 4, CV_64FC1, M, CV_AUTOSTEP);
      cvInitMatHeader (&sigma, 4, 1, CV_64FC1, sigma_elem, CV_AUTOSTEP);
      cvInitMatHeader (&MQ, 4, 4, CV_64FC1, q, CV_AUTOSTEP);

      /* compute eigenvectors */
      cvSVD (&MM, &sigma, &MQ, NULL, CV_SVD_U_T);

      /* copy the eigenvector corresponding to the largest eigenvalue to camera_params[i] */
      quat_copy (camera_params[i].ext.q, q);
    }
}

static void
compute_translation (camera_param_t *camera_params, double (*coeffs)[4], const int nobserv, const int ncamera)
{
  CvMat *AtA, *Atb, *ts;

  int i, j, k, l, num;

  /* allocate work memories */
  AtA = cvCreateMat (3*(ncamera-1), 3*(ncamera-1), CV_64FC1);
  Atb = cvCreateMat (3*(ncamera-1), 1, CV_64FC1);
  ts = cvCreateMat (3*(ncamera-1), 1, CV_64FC1);

  /* clear entries */
  cvSetZero (AtA);
  cvSetZero (Atb);

  /* construct data matrix (A^T)A, and vector (A^T)b */
  for (num = 0; num < nobserv; ++num)
    {
      const int base = ncamera * num;

      /* AtA */
      for (i = 0; i < ncamera-1; ++i)
        {
          for (j = 0; j < ncamera-1; ++j)
            {
              double scale = (i != j) ? -1.0 : (double) (ncamera-1);

              for (k = 0; k < 3; ++k)
                {
                  for (l = 0; l < 3; ++l)
                    {
                      CV_MAT_ELEM (*AtA, double, 3*i + k, 3*j + l) +=  scale * coeffs[base + i+1][k] * coeffs[base + j+1][l];
                    }
                }
            }
        }

      /* Atb */
      for (i = 0; i < ncamera-1; ++i)
        {
          for (j = 0; j < 3; ++j)
            {
              CV_MAT_ELEM (*Atb, double, 3*i + j, 0) += coeffs[base + i+1][j] * (coeffs[base][3] - coeffs[base + i+1][3]);
            }

          for (j = i+1; j < ncamera-1; ++j)
            {
              const double val = coeffs[base + j+1][3] - coeffs[base + i+1][3];

              for (k = 0; k < 3; ++k)
                {
                  CV_MAT_ELEM (*Atb, double, 3*i + k, 0) += coeffs[base + i+1][k] * val;
                  CV_MAT_ELEM (*Atb, double, 3*j + k, 0) -= coeffs[base + j+1][k] * val;
                }
            }
        }
    }

  /* solve the equation */
  cvSolve (AtA, Atb, ts, CV_SVD);

  /* substitute the result */
  for (i = 0; i < ncamera-1; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          camera_params[i+1].ext.t[j] = cvmGet (ts, 3*i + j, 0);
        }
    }

  /* free memories */
  cvReleaseMat (&ts);
  cvReleaseMat (&Atb);
  cvReleaseMat (&AtA);
}

/* plane coordinate = homo * image coordinate */
static void
construct_homography (CvMat *homo, const camera_param_t *camera_param, const extrinsic_param_t *plane)
{
  double A[3*3], rt[3*3];
  extrinsic_param_t cam_plane;
  int i, j, k;

  cp_mat_intrinsic (A, 3, &camera_param->intr);

  cp_mult_extrinsics (&cam_plane, &camera_param->ext, plane);
  quat_R_from_q (rt, 3, cam_plane.q);

  for (i = 0; i < 3; ++i)
    {
      rt[3*2 + i] = cam_plane.t[i];
    }

  /* homo = A * rt */
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          CV_MAT_ELEM (*homo, double, i, j) = 0.0;
          for (k = 0; k < 3; ++k)
            {
              CV_MAT_ELEM (*homo, double, i, j) += A[i + 3*k] * rt[k + 3*j];
            }
        }
    }
}

static void
construct_homography_inv (CvMat *homo, const camera_param_t *camera_param, const extrinsic_param_t *plane)
{
  double Ainv[3*3], temp[3*3], rt[3], rA[3*3];
  extrinsic_param_t cam_plane;
  int i, j, k;

  cp_matinv_intrinsic (Ainv, 3, &camera_param->intr);
  cp_mult_extrinsics (&cam_plane, &camera_param->ext, plane);

  /* temp = R^T */
  quat_R_from_q (temp, 3, cam_plane.q);

  /* rt[i] = dot (r_i, t) */
  for (i = 0; i < 3; ++i)
    {
      rt[i] = s_dot3 (&temp[3*i], cam_plane.t);
    }

  for (i = 0; i < 3; ++i)
    {
      temp[3*2 + i] /= rt[2];
    }

  /* rA = temp^T * Ainv */
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          rA[j + 3*i] = 0.0;
          for (k = 0; k < 3; ++k)
            {
              rA[j + 3*i] += temp[k + 3*j] * Ainv[k + 3*i];
            }
        }
    }

  /* homo = ([r1 r2 0]^T + diag (-r1^T t, -r2^T t, 1) * r3^T / (r3^T t)) * Ainv */
  for (i = 0; i < 2; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          CV_MAT_ELEM (*homo, double, i, j) = rA[i + 3*j] - rt[i] * rA[2 + 3*j];
        }
    }
  for (i = 0; i < 3; ++i)
    {
      CV_MAT_ELEM (*homo, double, 2, i) = rA[2 + 3*i];
    }
}

/* points: 2xN matrix */
static void
compute_undistorted_points (CvMat *points, const camera_param_t *camera_param, const cdata_observed_points_t *op)
{
  CvMat *distorted, cameraMatrix, dcoeff;
  double A[3*3], dcoeff_elem[5];
  int i;

  distorted = cvCreateMat (cvGetDimSize (points, 0), cvGetDimSize (points, 1), CV_64FC2);

  /* copy intrinsic parameters */
  cvInitMatHeader (&cameraMatrix, 3, 3, CV_64FC1, A, CV_AUTOSTEP);
  for (i = 0; i < 3*3; ++i)
    {
      A[i] = 0.0;
    }
  cvmSet (&cameraMatrix, 0, 0, camera_param->intr.alpha);
  cvmSet (&cameraMatrix, 0, 1, camera_param->intr.gamma);
  cvmSet (&cameraMatrix, 1, 1, camera_param->intr.beta);
  cvmSet (&cameraMatrix, 0, 2, camera_param->intr.c[0]);
  cvmSet (&cameraMatrix, 1, 2, camera_param->intr.c[1]);
  cvmSet (&cameraMatrix, 2, 2, 1.0);

  /* copy elements to the distorted */
  for (i = 0; i < op->num_points; ++i)
    {
      ((double *) CV_MAT_ELEM_PTR (*distorted, 0, i))[0] = op->cam[i][0];
      ((double *) CV_MAT_ELEM_PTR (*distorted, 0, i))[1] = op->cam[i][1];
    }

  /* set distortion coefficients */
  for (i = 0; i < (camera_param->dist.dim < 5 ? camera_param->dist.dim : 5); ++i)
    {
      dcoeff_elem[i] = camera_param->dist.elem[i];
    }
  while (i < 5)
    {
      dcoeff_elem[i] = 0.0;
      ++i;
    }
  cvInitMatHeader (&dcoeff, 1, (camera_param->dist.dim < 5 ? 4 : 5), CV_64FC1, dcoeff_elem, CV_AUTOSTEP);

  /* store undistorted points to points */
  cvUndistortPoints (distorted, points, &cameraMatrix, &dcoeff, NULL, NULL);

  /* compute undistorted point on the image plane and overwrite it */
  for (i = 0; i < op->num_points; ++i)
    {
      const intrinsic_param_t *intr = &camera_param->intr;
      double *ud, p[2];

      ud = (double *) CV_MAT_ELEM_PTR (*points, 0, i);
      p[0] = intr->alpha * ud[0] + intr->gamma * ud[1] + intr->c[0];
      p[1] = intr->beta * ud[1] + intr->c[1];

      ud[0] = p[0];
      ud[1] = p[1];
    }

  cvReleaseMat (&distorted);
}

static void
estimate_nearest_point (double refpos[2], CvMat *Homo, CvMat *Homo_inv, const double imgpos[2], const double interval)
{
  double bp[3], pos[2][2], ipos[4][3], distance;
  int i, j, k, min;

  for (i = 0; i < 3; ++i)
    {
      bp[i] = cvmGet (Homo_inv, i, 0) * imgpos[0] + cvmGet (Homo_inv, i, 1) * imgpos[1] + cvmGet (Homo_inv, i, 2);
    }
  for (i = 0; i < 3; ++i)
    {
      bp[i] /= bp[2];
    }

  for (i = 0; i < 2; ++i)
    {
      pos[i][0] = floor (bp[i]/interval) * interval;
      pos[i][1] = ceil (bp[i]/interval) * interval;
    }

  for (i = 0; i < 2; ++i)
    {
      for (j = 0; j < 2; ++j)
        {
          for (k = 0; k < 3; ++k)
            {
              ipos[2*i+j][k] = cvmGet (Homo, k, 0) * pos[0][j] + cvmGet (Homo, k, 1) * pos[1][i] + cvmGet (Homo, k, 2);
            }
          for (k = 0; k < 3; ++k)
            {
              ipos[2*i+j][k] /= ipos[2*i+j][2];
            }
        }
    }
  
  distance = sqrt (pow (ipos[0][0] - imgpos[0], 2) + pow (ipos[0][1] - imgpos[1], 2));

  min = 0;
  for (i = 1; i < 4; ++i)
    {
      double d = sqrt (pow (ipos[i][0] - imgpos[0], 2) + pow (ipos[i][1] - imgpos[1], 2));

      if (d < distance)
        {
          min = i;
          distance = d;
        }
    }

  refpos[0] = pos[0][min % 2];
  refpos[1] = pos[1][min / 2];
}

static void
trans2D_apply (double y[2], const trans2D_t *t2d, const double x[2])
{
  y[0] = cos (t2d->theta) * x[0] - sin (t2d->theta) * x[1] + t2d->t[0];
  y[1] = sin (t2d->theta) * x[0] + cos (t2d->theta) * x[1] + t2d->t[1];
}

static void
mult_mat33 (double m[3][3], double n[3][3])
{
  double temp[3];
  int i, j, k;

  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          temp[j] = 0.0;
          for (k = 0; k < 3; ++k)
            {
              temp[j] += m[k][i] * n[j][k];
            }
        }
      for (j = 0; j < 3; ++j)
        {
          m[j][i] = temp[j];
        }
    }
}

#ifdef DEBUG_MULTICALIB
static void
disp_mat (double (*m)[3], const int dim)
{
  int i, j;
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < dim; ++j)
        {
          printf("% 10.3g ", m[j][i]);
        }
      printf("\n");
    }
  printf("\n");
}
#endif

static double
calib_calc_error (const camera_param_t *camera_params, const extrinsic_param_t *plane_poses, const trans2D_t *t2ds, const cdata_t *cdata, CvMat *JtJ, CvMat *JtF, const double *lambdas, const int flag)
{
  const int nobserv = cdata->num_observations, ncamera = cdata->num_cameras;

  const int num_intr_param = 4 + ((flag & CALIB_FIX_SHEAR) ? 0 : 1);
  const int num_dist_param = (flag & CALIB_ZERO_DIST) ? 0 : (2 + ((flag & CALIB_ZERO_TANGENT_DIST) ? 0 : 2) + ((flag & CALIB_ZERO_HIGHER_ORDER_DIST) ? 0 : 1));
  const int num_ext_param = 4 + 3;

  const int num_cam_param = num_intr_param + num_dist_param;
#ifdef USE_INFINITESIMAL_ROTATION
  const int num_lambda = 0;
#else
  const int num_lambda = ncamera-1 + nobserv;
#endif
  const int num_params = num_cam_param * ncamera + num_ext_param * (ncamera-1 + nobserv) + num_lambda;

  const int intr_off = num_cam_param * ncamera;
  const int plane_off = num_cam_param * ncamera + num_ext_param * (ncamera-1);

  double error = 0.0;
  int amount_points = 0;
  double lambda_term = 0.0;

  CvMat *J = NULL;

  int i, j, k, l, m;

  if (JtJ != NULL || JtF != NULL)
    {
      J = cvCreateMat (2, num_params, CV_64FC1);
      cvSetZero (JtJ);
      cvSetZero (JtF);
    }

  for (i = 0; i < nobserv; ++i)
    {
      for (j = 0; j < ncamera; ++j)
        {
          const int npoint = cdata->data[ncamera*i + j].num_points;
          //const intrinsic_param_t  *intr = &camera_params[j].intr;
          //const distortion_param_t *dist = &camera_params[j].dist;
          const extrinsic_param_t  *ext  = &camera_params[j].ext;
          const extrinsic_param_t  *plane_pose = &plane_poses[ncamera*i];

          amount_points += npoint;

          for (k = 0; k < npoint; ++k)
            {
              const double *ref = cdata->data[ncamera*i + j].ref[k];
              const double *cam = cdata->data[ncamera*i + j].cam[k];

              double px[3], x[3], cx[3], nc[3], ncd[3], ix[3];
              double f[2];

              double dpx[3][3], dx[3][3], dcx[3][3], dnc[3][3], dncd[3][3];
              double dpp[4][3], dcp[4][3], dd[5][2], dA[5][3];

              /* transform points on a temporal coordinate system to on the global one */
              trans2D_apply (px, &t2ds[ncamera*i + j], ref);
              px[2] = 0.0;

              /* plane coordinate -> global coordinate */
              cp_mult_extrinsic (x, plane_pose, px, dpx, dpp);
              
              /* global coordinate -> camera coordinate */
              cp_mult_extrinsic (cx, ext, x, dx, dcp);

              /* camera coordinate -> normalized coordinate */
              cp_normalize_point (nc, cx, dcx);

              /* distortion */
              cp_distort (ncd, &camera_params[j].dist, nc, dnc, dd);
              ncd[2] = 1.0;

              /* camera coordinate -> image plane coordinate */
              cp_mult_intrinsic (ix, &camera_params[j].intr, ncd, dncd, dA);

              for (l = 0; l < 2; ++l)
                {
                  f[l] = ix[l] - cam[l];
                }

              error += f[0]*f[0] + f[1]*f[1];

              if (JtJ != NULL || JtF != NULL)
                {
                  CvMat Mf;
                  double dfdx[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

                  cvSetZero (J);
                  cvInitMatHeader (&Mf, 2, 1, CV_64FC1, f, CV_AUTOSTEP);
#if 1
                  /* intrinsic */
                  if (!(flag & CALIB_FIX_INTRINSIC))
                    {
                      CV_MAT_ELEM (*J, double, 0, num_cam_param*j    ) = dA[0][0]; /* alpha */
                      CV_MAT_ELEM (*J, double, 1, num_cam_param*j + 1) = dA[2][1]; /* beta  */
                      CV_MAT_ELEM (*J, double, 0, num_cam_param*j + 2) = dA[3][0]; /* c[0]  */
                      CV_MAT_ELEM (*J, double, 1, num_cam_param*j + 3) = dA[4][1]; /* c[1]  */
                      if (!(flag & CALIB_FIX_SHEAR))
                        {
                          CV_MAT_ELEM (*J, double, 0, num_cam_param*j + 4) = dA[1][0]; /* gamma */
                        }
                    }
#endif
                  mult_mat33 (dfdx, dncd);
#if 1
                  /* distortion */
                  if (!((flag & CALIB_ZERO_DIST) || (flag & CALIB_FIX_DISTORTION)))
                    {
                      for (l = 0; l < 2; ++l)
                        {
                          double val;

                          /* k1 */
                          val = dfdx[0][l] * dd[0][0] + dfdx[1][l] * dd[0][1];
                          CV_MAT_ELEM (*J, double, l, num_cam_param*j + num_intr_param) = val;

                          /* k2 */
                          val = dfdx[0][l] * dd[1][0] + dfdx[1][l] * dd[1][1];
                          CV_MAT_ELEM (*J, double, l, num_cam_param*j + num_intr_param + 1) = val;

                          if (!(flag & CALIB_ZERO_HIGHER_ORDER_DIST))
                            {
                              const int p_off = (flag & CALIB_ZERO_TANGENT_DIST) ? 2 : 4;
                              /* k3 */
                              val = dfdx[0][l] * dd[4][0] + dfdx[1][l] * dd[4][1];
                              CV_MAT_ELEM (*J, double, l, num_cam_param*j + num_intr_param + p_off) = val;
                            }

                          if (!(flag & CALIB_ZERO_TANGENT_DIST))
                            {
                              /* p1 */
                              val = dfdx[0][l] * dd[2][0] + dfdx[1][l] * dd[2][1];
                              CV_MAT_ELEM (*J, double, l, num_cam_param*j + num_intr_param + 2) = val;

                              /* p2 */
                              val = dfdx[0][l] * dd[3][0] + dfdx[1][l] * dd[3][1];
                              CV_MAT_ELEM (*J, double, l, num_cam_param*j + num_intr_param + 3) = val;
                            }
                        }
                    }
#endif
                  mult_mat33(dfdx, dnc);
                  mult_mat33(dfdx, dcx);

                  /* camera pose */
                  if (j > 0)
                    {
#if 1
                      /* q */
                      if (!(flag & CALIB_FIX_CAMERA_ORIENTATION))
                        {
# ifdef USE_INFINITESIMAL_ROTATION
                          for (l = 0; l < 3; ++l)
# else
                          for (l = 0; l < 4; ++l)
# endif
                            {
                              for (m = 0; m < 2; ++m)
                                {
                                  double val = dfdx[0][m] * dcp[l][0] + dfdx[1][m] * dcp[l][1] + dfdx[2][m] * dcp[l][2];
                                  CV_MAT_ELEM (*J, double, m, intr_off + num_ext_param*(j-1) + l) = val;
                                }
                            }
                        }
#endif
#if 1
                      /* t */
                      if (!(flag & CALIB_FIX_TRANSLATION))
                        {
                          for (l = 0; l < 3; ++l)
                            {
                              for (m = 0; m < 2; ++m)
                                {
                                  CV_MAT_ELEM (*J, double, m, intr_off + num_ext_param*(j-1) + 4 + l) = dfdx[l][m];
                                }
                            }
                        }
#endif
                    }

                  mult_mat33(dfdx, dx);

                  /* plane pose */
#if 1
                  /* q */
                  if (!(flag & CALIB_FIX_PLANE_ORIENTATION))
                    {
# ifdef USE_INFINITESIMAL_ROTATION
                      for (l = 0; l < 3; ++l)
# else
                      for (l = 0; l < 4; ++l)
# endif
                        {
                          for (m = 0; m < 2; ++m)
                            {
                              double val = dfdx[0][m] * dpp[l][0] + dfdx[1][m] * dpp[l][1] + dfdx[2][m] * dpp[l][2];
                              CV_MAT_ELEM (*J, double, m, plane_off + num_ext_param*i + l) = val;
                            }
                        }
                    }
#endif
#if 1
                  /* t */
                  if (!(flag & CALIB_FIX_TRANSLATION))
                    {
                      for (l = 0; l < 3; ++l)
                        {
                          for (m = 0; m < 2; ++m)
                            {
                              CV_MAT_ELEM (*J, double, m, plane_off + num_ext_param*i + 4 + l) = dfdx[l][m];
                            }
                        }
                    }
#endif

                  if (JtJ != NULL)
                    {
                      cvGEMM (J, J, 1.0, JtJ, 1.0, JtJ, CV_GEMM_A_T);
                    }
                  if (JtF != NULL)
                    {
                      cvGEMM (J, &Mf, 1.0, JtF, 1.0, JtF, CV_GEMM_A_T);
                    }
                }
            }
        }
    }

  lambda_term = 0.0;
#ifndef USE_INFINITESIMAL_ROTATION
  if (JtF != NULL)
    {
      const int l_off = num_cam_param * ncamera + num_ext_param * (ncamera-1 + nobserv);

      /* camera orientation */
      for (i = 1; i < ncamera; ++i)
        {
          const int q_off = num_cam_param*ncamera + num_ext_param*(i-1);

          lambda_term += mod_lambda (JtJ, JtF, q_off, l_off + (i-1), lambdas[i-1], camera_params[i].ext.q);
        }

      /* plane orientation */
      for (i = 0; i < nobserv; ++i)
        {
          const int q_off = num_cam_param*ncamera + num_ext_param*(ncamera-1) + num_ext_param*i;

          lambda_term += mod_lambda (JtJ, JtF, q_off, l_off + (ncamera-1) + i, lambdas[ncamera-1 + i], plane_poses[ncamera*i].q);
        }
    }
#endif

  if (J != NULL)
    {
      cvReleaseMat (&J);
    }

  return error - lambda_term;
}

static void
update_camera_params (camera_param_t *camera_params, extrinsic_param_t *plane_poses, double *lambdas,
                      const CvMat *rev,
                      const camera_param_t *camera_params_current, const int ncamera,
                      const extrinsic_param_t *plane_poses_current, const int nobserv,
                      const double *lambdas_current, const int flag)
{
  const int num_intr_param = 4 + ((flag & CALIB_FIX_SHEAR) ? 0 : 1);
  const int num_dist_param = (flag & CALIB_ZERO_DIST) ? 0 : (2 + ((flag & CALIB_ZERO_TANGENT_DIST) ? 0 : 2) + ((flag & CALIB_ZERO_HIGHER_ORDER_DIST) ? 0 : 1));
  const int num_ext_param = 4 + 3;

  const int num_cam_param = num_intr_param + num_dist_param;
#ifndef USE_INFINITESIMAL_ROTATION
  const int num_lambda = ncamera-1 + nobserv;
#endif

  int i, j;

  for (i = 0; i < ncamera; ++i)
    {
      camera_param_t *new_cam = &camera_params[i];
      const camera_param_t *old_cam = &camera_params_current[i];

      if (!(flag & CALIB_FIX_INTRINSIC))
        {
          new_cam->intr.alpha = old_cam->intr.alpha - cvmGet (rev, num_cam_param*i, 0);
          new_cam->intr.beta = old_cam->intr.beta - cvmGet (rev, num_cam_param*i + 1, 0);
          new_cam->intr.c[0] = old_cam->intr.c[0] - cvmGet (rev, num_cam_param*i + 2, 0);
          new_cam->intr.c[1] = old_cam->intr.c[1] - cvmGet (rev, num_cam_param*i + 3, 0);

          if (!(flag & CALIB_FIX_SHEAR))
            {
              new_cam->intr.gamma = old_cam->intr.gamma - cvmGet (rev, num_cam_param*i + 4, 0);
            }
        }
      else
        {
          new_cam->intr.alpha = old_cam->intr.alpha;
          new_cam->intr.beta = old_cam->intr.beta;
          new_cam->intr.gamma = old_cam->intr.gamma;
          new_cam->intr.c[0] = old_cam->intr.c[0];
          new_cam->intr.c[1] = old_cam->intr.c[1];
        }

      if (!(flag & CALIB_ZERO_DIST) && !(flag & CALIB_FIX_DISTORTION))
        {
          for (j = 0; j < 2; ++j)
            {
              new_cam->dist.elem[j] = old_cam->dist.elem[j] - cvmGet (rev, num_cam_param*i + num_intr_param + j, 0);
            }

          if (!(flag & CALIB_ZERO_TANGENT_DIST))
            {
              for (j = 0; j < 2; ++j)
                {
                  new_cam->dist.elem[2+j] = old_cam->dist.elem[2+j] - cvmGet (rev, num_cam_param*i + num_intr_param + 2 + j, 0);
                }
            }
          if (!(flag & CALIB_ZERO_HIGHER_ORDER_DIST))
            {
              const int dist_off = (flag & CALIB_ZERO_TANGENT_DIST) ? 2 : 4;
              new_cam->dist.elem[4] = old_cam->dist.elem[4] - cvmGet (rev, num_cam_param*i + num_intr_param + dist_off, 0);
            }
        }
      else
        {
          for (j = 0; j < 5; ++j)
            {
              new_cam->dist.elem[j] = old_cam->dist.elem[j];
            }
        }

      /* camera poses */
      if (i > 0)
        {
          if (!(flag & CALIB_FIX_CAMERA_ORIENTATION))
            {
#ifdef USE_INFINITESIMAL_ROTATION
              quaternion_t q;
              double norm2 = 0.0;
              for (j = 0; j < 3; ++j)
                {
                  quat_im (q, j) = - cvmGet(rev, num_cam_param*ncamera + num_ext_param * (i-1) + j, 0) / 2.0;
                  norm2 += quat_im (q, j) * quat_im (q, j);
                }
              quat_re (q) = sqrt(1.0 - norm2);

              quat_mult (new_cam->ext.q, q, old_cam->ext.q);
#else
              for (j = 0; j < 3; ++j)
                {
                  quat_im (new_cam->ext.q, j) = quat_im (old_cam->ext.q, j) - cvmGet (rev, num_cam_param*ncamera + num_ext_param * (i-1) + j, 0);
                }
              quat_re (new_cam->ext.q) = quat_re (old_cam->ext.q) - cvmGet (rev, num_cam_param*ncamera + num_ext_param * (i-1) + 3, 0);
#endif
              if (flag & CALIB_FORCE_Q_NORM_1)
                {
                  quat_normalize (new_cam->ext.q);
                }
            }
          else
            {
              for (j = 0; j < 3; ++j)
                {
                  quat_im (new_cam->ext.q, j) = quat_im (old_cam->ext.q, j);
                }
              quat_re (new_cam->ext.q) = quat_re (old_cam->ext.q);
            }

          if (!(flag & CALIB_FIX_TRANSLATION))
            {
              for (j = 0; j < 3; ++j)
                {
                  new_cam->ext.t[j] = old_cam->ext.t[j] - cvmGet (rev, num_cam_param*ncamera + num_ext_param * (i-1) + 4 + j, 0);
                }
            }
          else
            {
              for (j = 0; j < 3; ++j)
                {
                  new_cam->ext.t[j] = old_cam->ext.t[j];
                }
            }
        }
      else
        {
          quat_copy (new_cam->ext.q, quat_one);
          for (j = 0; j < 3; ++j)
            {
              new_cam->ext.t[j] = 0.0;
            }
        }                                                                                
    }

  /* plane poses */
  {
    const int plane_off = num_cam_param * ncamera + num_ext_param * (ncamera-1);

    for (i = 0; i < nobserv; ++i)
      {
        extrinsic_param_t *new_plane = &plane_poses[ncamera*i];
        const extrinsic_param_t *old_plane = &plane_poses_current[ncamera*i];

        if (!(flag & CALIB_FIX_PLANE_ORIENTATION))
          {
#ifdef USE_INFINITESIMAL_ROTATION
            quaternion_t q;
            double norm2 = 0.0;
            for (j = 0; j < 3; ++j)
              {
                quat_im (q, j) = - cvmGet(rev, plane_off + num_ext_param*i + j, 0) / 2.0;
                norm2 += quat_im (q, j) * quat_im (q, j);
              }
            quat_re (q) = sqrt(1.0 - norm2);

            quat_mult (new_plane->q, q, old_plane->q);
#else
            for (j = 0; j < 3; ++j)
              {
                quat_im (new_plane->q, j) = quat_im (old_plane->q, j) - cvmGet (rev, plane_off + num_ext_param*i + j, 0);
              }
            quat_re (new_plane->q) = quat_re (old_plane->q) - cvmGet (rev, plane_off + num_ext_param*i + 3, 0);
#endif

            if (flag & CALIB_FORCE_Q_NORM_1)
              {
                quat_normalize (new_plane->q);
              }
          }
        else
          {
            for (j = 0; j < 3; ++j)
              {
                quat_im (new_plane->q, j) = quat_im (old_plane->q, j);
              }
            quat_re (new_plane->q) = quat_re (old_plane->q);
          }

        if (!(flag & CALIB_FIX_TRANSLATION))
          {
            for (j = 0; j < 3; ++j)
              {
                new_plane->t[j] = old_plane->t[j] - cvmGet (rev, plane_off + num_ext_param*i + 4 + j, 0);
              }
          }
        else
          {
            for (j = 0; j < 3; ++j)
              {
                new_plane->t[j] = old_plane->t[j];
              }
          }
      }
  }

#ifndef USE_INFINITESIMAL_ROTATION
  /* lambdas */
  for (i = 0; i < num_lambda; ++i)
    {
      lambdas[i] = lambdas_current[i] - cvmGet (rev, num_cam_param * ncamera + num_ext_param * (ncamera-1 + nobserv) + i, 0);
    }
#endif
}

#ifndef USE_INFINITESIMAL_ROTATION
static double
mod_lambda (CvMat *JtJ, CvMat *JtF, const int pos, const int lpos, const double l, const quaternion_t q)
{
  int i, j;
  double qq, qq_1, lambda, JtJq[4], dldq[4];
  const double scale = 1.0;

  if (JtF == NULL)
    {
      return 0;
    }

  qq = quat_norm2 (q);
  qq_1 = qq - 1.0;

  lambda = cvmGet (JtF, pos + 3, 0) * quat_re (q);
  for (i = 0; i < 3; ++i)
    {
      lambda += cvmGet (JtF, pos + i, 0) * quat_im (q, i);
    }
  lambda /= qq;

  for (i = 0; i < 4; ++i)
    {
      JtJq[i] = CV_MAT_ELEM (*JtJ, double, pos + i, pos + 3) * quat_re (q);
      for (j = 0; j < 3; ++j)
        {
          JtJq[i] += CV_MAT_ELEM (*JtJ, double, pos + i, pos + j) * quat_im (q, j);
        }

      dldq[i] = JtJq[i] + CV_MAT_ELEM (*JtF, double, pos + i, 0);
    }

  if (JtJ != NULL)
    {
      for (i = 0; i < 4; ++i)
        {
          CV_MAT_ELEM (*JtJ, double, pos + i, pos + i) -= l * scale;

          CV_MAT_ELEM (*JtJ, double, pos + i, lpos) -= q[i] * scale;
          CV_MAT_ELEM (*JtJ, double, lpos, pos + i) -= q[i] * scale;
        }
    }

  if (JtF != NULL)
    {
      for (i = 0; i < 3; ++i)
        {
          CV_MAT_ELEM (*JtF, double, pos + i, 0) -= l * quat_im (q, i) * scale;
        }
      CV_MAT_ELEM (*JtF, double, pos + 3, 0) -= l * quat_re (q) * scale;

      CV_MAT_ELEM (*JtF, double, lpos, 0) = - qq_1 / 2.0 * scale;
    }

  return qq_1 * l;
}
#endif
