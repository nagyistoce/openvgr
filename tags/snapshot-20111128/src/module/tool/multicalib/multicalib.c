/*
 multicalib.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#include <unistd.h>
#include <stdio.h>
#include <time.h>

#include <cv.h>
#include <highgui.h>

#include "multicalib.h"
#include "calib_proc.h"
#include "calib_data.h"

static void parse_args (multicalib_opt_t *opt, const int argc, char **argv);
static void display_help (const char *name);
static void output_params (const camera_param_t *params, const int ncamera);
static void output_params_yaml (const camera_param_t *params, const int ncamera, const double error, const multicalib_opt_t *opt);

static char *alloc_strcpy (const char *str);

int
main (int argc, char **argv)
{
  multicalib_opt_t opt = MULTICALIB_OPT_INIT;
  camera_param_t *params = NULL;
  cdata_t cdata;

  FILE *fp = NULL;
  double err = 0.0;

  parse_args (&opt, argc, argv);

  cdata_init (&cdata);

  if (opt.ifile)
    {
      fp = fopen (opt.ifile, "r");
      if (fp == NULL)
        {
          fprintf (stderr, "could not open '%s'.\n", opt.ifile);
          return -1;
        }
    }
  else
    {
      fprintf (stderr, "input: stdin\n");
      fp = stdin;
    }

  cdata_read (&cdata, fp);
  fclose (fp); fp = NULL;

  if (cdata.num_cameras < 1)
    {
      return EXIT_FAILURE;
    }

  if (opt.interval > 0)
    {
      cdata.interval = opt.interval;
    }

  params = (camera_param_t *) malloc (sizeof (camera_param_t) * cdata.num_cameras);
  if (params == NULL)
    {
      fprintf (stderr, "could not allocate memory.\n");
      return -1;
    }

  err = calibrate_cameras (params, &cdata, &opt.calib_opt);
  fprintf (stderr, "error: %g\n", err);

  if (opt.ofile == NULL || opt.calib_opt.verbose_mode)
    {
      output_params (params, cdata.num_cameras);
    }

  if (opt.ofile)
    {
      output_params_yaml (params, cdata.num_cameras, err, &opt);
    }

  cdata_final (&cdata);
  free (params); params = NULL;

  if (opt.ifile != NULL)
    {
      free (opt.ifile);
    }

  if (opt.ofile != NULL)
    {
      free (opt.ofile);
    }

  return EXIT_SUCCESS;
}

static void
parse_args (multicalib_opt_t *opt, const int argc, char **argv)
{
  const char optstr[] = "d:hl:m:o:qsvw:";
  int c, num;

  while ((c = getopt (argc, argv, optstr)) >= 0)
    {
      switch (c)
        {
        case 'h':
          display_help (argv[0]);
          exit (EXIT_SUCCESS);
          break;

        case 'v':
          opt->calib_opt.verbose_mode = -1;
          break;

        case 'w':
          opt->interval = atof (optarg);
          break;

        case 'd':
          num = atoi (optarg);
          switch (num)
            {
            case 0:
              opt->calib_opt.optimize_flag |= CALIB_ZERO_DIST;
              break;

            case 2:
              opt->calib_opt.optimize_flag |= CALIB_ZERO_TANGENT_DIST | CALIB_ZERO_HIGHER_ORDER_DIST;
              break;

            case 4:
              opt->calib_opt.optimize_flag |= CALIB_ZERO_HIGHER_ORDER_DIST;
              break;

            case 5:
              opt->calib_opt.optimize_flag &= !(CALIB_ZERO_DIST | CALIB_ZERO_TANGENT_DIST | CALIB_ZERO_HIGHER_ORDER_DIST);
              break;

            default:
              fprintf (stderr, "invalid number of distortion coefficients (%d)\n", num);
              exit (EXIT_FAILURE);
              break;
            }
          break;

        case 'm':
          opt->calib_opt.max_iter = atoi (optarg);
          break;

        case 'l':
          opt->calib_opt.update_limit = atof (optarg);
          break;

        case 'q':
          opt->calib_opt.force_norm_1 = -1;
          break;

        case 's':
          opt->calib_opt.optimize_flag &= !CALIB_FIX_SHEAR;
          break;

        case 'o':
          opt->ofile = alloc_strcpy (optarg);
          break;

        default:
          exit (EXIT_FAILURE);
          break;
        }
    }

  if (argv[optind] != NULL && strlen (argv[optind]) > 0)
    {
      opt->ifile = alloc_strcpy (argv[optind]);
    }
}

static void
display_help (const char *name)
{
  printf ("usage: %s [-h] [-v] [-w <val>] [-d <val>] [-m <val>] [-l <val>] [-q] [-s] [-o <file>.yaml] <calib_data>\n", name);
  printf ("\n");

  printf ("\t-h: show this help.\n");
  printf ("\t-v: set verbose mode.\n");
  printf ("\n");
  
  printf ("\t-w <val>: use <val> for pattern interval.\n");
  printf ("\t-d <val>: set the number of distortion coefficients. {0,2,4,5} (default: 5)\n");
  printf ("\t-m <val>: set the maximum iteration number of optimization process. (default: %d)\n", CALIB_MAX_ITER);
  printf ("\t-l <val>: set minimum update value in pixel. (default: %g)\n", CALIB_MIN_UPDATE);
  printf ("\t-q: force |q| == 1 in each iteration.\n");
  printf ("\t-s: estimate shear coefficient of intrinsic parameter.\n");
  printf ("\t-o <file>: specifies a yaml filename to write the result.\n");

  printf ("\n");
}

static void
output_params (const camera_param_t *params, const int ncamera)
{
  int i;

  for (i = 0; i < ncamera; ++i)
    {
      printf ("# camera %d\n", i);

      printf ("# intrinsic\n");
      cp_print_intrinsic (stdout, &params[i].intr);
      printf ("\n");

      printf ("# distortion\n");
      cp_print_distortion (stdout, &params[i].dist);
      printf ("\n");

      printf ("# extrinsic\n");
      cp_print_extrinsic (stdout, &params[i].ext, 1);
      printf ("\n");
    }
}

static void
output_params_yaml (const camera_param_t *params, const int ncamera, const double error, const multicalib_opt_t *opt)
{
  CvFileStorage *fs = NULL;

  char str[64];
  time_t calib_time;
  int i;

  if (opt->ofile == NULL)
    {
      return;
    }

  fs = cvOpenFileStorage (opt->ofile, NULL, CV_STORAGE_WRITE);

  snprintf (str, sizeof (str), "error: %f", error);
  cvWriteComment (fs, str, 0);

  time(&calib_time);
  strftime (str, sizeof (str), "date: %c", localtime(&calib_time));
  cvWriteComment (fs, str, 0);

  cvWriteInt (fs, "num_cameras", ncamera);

  for (i = 0; i < ncamera; ++i)
    {
      const int num_coeff = (opt->calib_opt.optimize_flag & CALIB_ZERO_HIGHER_ORDER_DIST) ? 4 : 5;
      CvMat mat;
      double elem[4*4], R[3*3];
      
      int j;

      /* intrinsic */
      cvInitMatHeader (&mat, 3, 3, CV_64FC1, elem, CV_AUTOSTEP);
      cvmSet (&mat, 0, 0, params[i].intr.alpha);
      cvmSet (&mat, 0, 1, params[i].intr.gamma);
      cvmSet (&mat, 0, 2, params[i].intr.c[0]);

      cvmSet (&mat, 1, 0, 0.0);
      cvmSet (&mat, 1, 1, params[i].intr.beta);
      cvmSet (&mat, 1, 2, params[i].intr.c[1]);

      cvmSet (&mat, 2, 0, 0.0);
      cvmSet (&mat, 2, 1, 0.0);
      cvmSet (&mat, 2, 2, 1.0);

      snprintf (str, sizeof (str), "camera%d_intr", i);
      cvWrite (fs, str, &mat, cvAttrList (0, 0));

      /* distortion */
      cvInitMatHeader (&mat, num_coeff, 1, CV_64FC1, elem, CV_AUTOSTEP);

      for (j = 0; j < params[i].dist.dim; ++j)
        {
          cvmSet (&mat, j, 0, params[i].dist.elem[j]);
        }
      while (j < num_coeff)
        {
          cvmSet (&mat, j, 0, 0.0);
          ++j;
        }

      snprintf (str, sizeof (str), "camera%d_dist", i);
      cvWrite (fs, str, &mat, cvAttrList (0, 0));

      /* extrinsic */
      cvInitMatHeader (&mat, 4, 4, CV_64FC1, elem, CV_AUTOSTEP);

      quat_R_from_q (R, 3, params[i].ext.q);
      for (j = 0; j < 3; ++j)
        {
          int k;
          for (k = 0; k < 3; ++k)
            {
              cvmSet (&mat, j, k, R[3*k + j]);
            }
          cvmSet (&mat, j, 3, params[i].ext.t[j]);
        }
      for (j = 0; j < 4; ++j)
        {
          cvmSet (&mat, 3, j, j != 3 ? 0.0 : 1.0);
        }

      snprintf (str, sizeof (str), "camera%d_ext", i);
      cvWrite (fs, str, &mat, cvAttrList (0, 0));
    }

  cvReleaseFileStorage (&fs);
}

static char *
alloc_strcpy (const char *str)
{
  char *buf = NULL;
  int len = 0;

  len = strlen (str);
  if (str == NULL || len < 1)
    {
      return NULL;
    }

  buf = (char *) malloc (sizeof (char) * (len + 1));
  if (buf == NULL)
    {
      return NULL;
    }

  strncpy (buf, str, len);
  buf[len] = '\0';

  return buf;
}
