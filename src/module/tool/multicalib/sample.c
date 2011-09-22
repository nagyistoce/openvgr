/*
 sample.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>

#include <unistd.h>
#include <sys/time.h>

#include <cv.h>
#include <highgui.h>

#include <capture.h>
#include "checker_data.h"
#include "detect_checker.h"

typedef enum
{
  METHOD_OPENCV
} method_t;

typedef struct tag_opt
{
  int    pattern_col;  /* x */
  int    pattern_row;  /* y */
  double pattern_size; /* length of a side of one quadrangle */

  method_t method;
  int ieee1394b_mode;

  FILE *fp;
} opt_t;

static void show_help (const char *prog_name);
static int get_grid (const char *str, int *row, int *col);
static void parse_opt (int argc, char **argv, opt_t *opt);
static void destroy (const int err_code, opt_t *opt);
static int proc (capture_t *cap, capture_frame_t *frames, opt_t *opt);

static void s_convert_frame_to_iplimage (capture_frame_t *frame, IplImage *ipl);
static checker_coord_t *s_detect_checker (IplImage *image, opt_t *opt);
static void s_draw_checker (IplImage *image, checker_coord_t *cc, opt_t *opt);

int
main (int argc, char **argv)
{
  capture_t cap = CAPTURE_INIT_VAL;
  capture_frame_t *frames = NULL;
  int status;

  opt_t opt;

  /* parse options */
  parse_opt (argc, argv, &opt);

  /* setup a camera system */
  status = capture_init (&cap, 1);
  if (status != CAPTURE_SUCCESS)
    {
      fprintf (stderr, "error in capture_init()\n");
      goto on_capture_init_error;
    }

  cap.prefer_bmode = opt.ieee1394b_mode; /* use 1394b mode if the cameras support it */
  cap.drop_frames = 1; /* enable dropping frames */

  status = capture_setup (&cap, "ieee1394board.0");
  if (status != CAPTURE_SUCCESS)
    {
      fprintf (stderr, "error in capture_setup()\n");
      goto on_capture_setup_error;
    }

  status = capture_create_frames (&cap, &frames);
  if (status != CAPTURE_SUCCESS)
    {
      fprintf (stderr, "error in capture_create_frames()\n");
      goto on_capture_create_frames_error;
    }

  /* some image processing */
  if (cap.num_active > 0)
    {
      proc (&cap, frames, &opt);
    }


  /* clean up */
  capture_destroy_frames (&cap, &frames);
  capture_final (&cap);

  
  destroy (EXIT_SUCCESS, &opt); /* exit() will be called inside of this function */


  /* error handling */
 on_capture_create_frames_error:
  capture_final (&cap);
 on_capture_setup_error:
  ;
 on_capture_init_error:

  return EXIT_FAILURE;
}

static void
show_help (const char *prog_name)
{
  printf ("usage: %s [-o <filename>] [-s <size>] [-g <row>x<col>] [-l]\n", prog_name);

  printf ("\n");

  printf ("<filename> :\t");
  printf ("file name of output file\n");

  printf ("\n");

  printf ("<size>     :\t");
  printf ("pattern size [mm] (default: 20)\n");

  printf ("<row>x<col>:\t");
  printf ("nunber of points in each row/col (default: 8x8)\n");

  printf ("\n");

  printf ("-l         :\t");
  printf ("use IEEE 1394 legacy mode (default: IEEE 1394b mode)\n");

  printf ("\n");

  printf ("--- Keyboard operations ---\n");
  printf ("<mode change>\n");
  printf ("l\tlive mode; show captured images.\n");
  printf ("c\tchessboard detection mode; search a chessboard pattern using OpenCV.\n");
  printf ("q\tterminate this program.\n");

  printf ("\n");

  printf ("<in a detection mode>\n");
  printf ("[ret]\t\tadd current detection result to a queue.\n");
  printf ("W (shift-w)\twrite the queue to stdout or a specified file (-o <filename>).\n");

  printf ("\n");
}

static int
get_grid (const char *str, int *row, int *col)
{
  int i, n, r = -1, c = -1;
  char buf[10];
  
  buf[sizeof (buf) - 1] = '\0';

  /* read row element */
  i = 0;
  while (i < sizeof (buf) - 1 && isdigit (str[i]))
    {
      buf[i] = str[i];
      ++i;
    }
  buf[i] = '\0';

  if (i < 1)
    {
      return -1;
    }
  r = atoi (buf);
  
  /* check if the next charactor is 'x' */
  if (str[i] != 'x')
    {
      return -1;
    }
  n = i + 1;

  /* read col element */
  i = 0;
  while (i < sizeof (buf) - 1 && isdigit (str[n + i]))
    {
      buf[i] = str[n + i];
      ++i;
    }
  buf[i] = '\0';
  
  if (i < 1 || str[n + i] != '\0')
    {
      return -1;
    }
  c = atoi (buf);

  /* set the values */
  *row = r;
  *col = c;

  return 0;
}

static void
parse_opt (int argc, char **argv, opt_t *opt)
{
  const static char optstr[] = "s:g:lm:o:w:h";
  char ch;

  opt->pattern_col  =  8;
  opt->pattern_row  =  8;
  opt->pattern_size = 20;

  opt->method = METHOD_OPENCV;

  opt->ieee1394b_mode = 1;
  opt->fp = stdout;

  while ((ch = getopt (argc, argv, optstr)) != -1)
    {
      switch (ch)
        {
        case 'g':
          if (get_grid (optarg, &opt->pattern_row, &opt->pattern_col) != 0)
            {
              fprintf (stderr, "ERROR: invalid grid num (%s)\n", optarg);
              show_help (argv[0]);
              destroy (EXIT_FAILURE, opt);
            }
          break;

        case 'h':
          show_help (argv[0]);
          destroy (EXIT_SUCCESS, opt);
          break;

        case 'l':
          opt->ieee1394b_mode = 0;
          break;

        case 's':
          opt->pattern_size = atof (optarg);
          break;

        case 'w':
          fprintf (stderr, "WARNING: '-w' is obsoleted. Use '-o' instead.\n");
        case 'o':
          {
            FILE *fp = NULL;
            fp = fopen (optarg, "w");
            if (fp == NULL)
              {
                fprintf (stderr, "ERROR: could not open a file (%s)\n", optarg);
                destroy (EXIT_FAILURE, opt);
              }
            opt->fp = fp;
          }
          break;

        case '?':
        default:
          show_help (argv[0]);
          destroy (EXIT_FAILURE, opt);
          break;
        }
    }
}

static void
destroy (const int err_code, opt_t *opt)
{
  if (opt->fp != stdout)
    {
      fclose (opt->fp);
    }
  exit (err_code);
}

static int
proc (capture_t *cap, capture_frame_t *frames, opt_t *opt)
{
  enum { EXEC_MODE_LIVE, EXEC_MODE_CHECKER } exec_mode;

  checker_data_t checker_data;

  const int slen = 128;
  char (*winname)[slen];

  IplImage **images, **overlay;
  checker_coord_t **cc;

  int to_exit = 0;
  int ret_status = EXIT_SUCCESS;
  int i;

  /* for timing framerate */
  const int count_frames = 150;
  int num_frames = 0;
  struct timeval prev, current, elapsed;
  
  /* initialize checker_data */
  checker_data_init (&checker_data, opt->pattern_size, cap->num_active);

  fprintf (stderr, "num_active: %d\n", cap->num_active);
  for (i = 0; i < cap->num_active; ++i)
    {
      fprintf (stderr, "frame[%d]: %dx%d, format = %d\n", i, frames[i].width, frames[i].height, frames[i].format);
    }

  /* allocate memory */
  /* for window title  */
  winname = (char (*)[slen]) malloc (sizeof (char[slen]) * cap->num_active);
  if (winname == NULL)
    {
      ret_status = EXIT_FAILURE;
      goto winname_err;
    }

  /* for captured image */
  images = (IplImage **) malloc (sizeof (IplImage *) * cap->num_active);
  if (images == NULL)
    {
      ret_status = EXIT_FAILURE;
      goto images_err;
    }

  /* for chessboard pattern detection */
  cc = (checker_coord_t **) malloc (sizeof (checker_coord_t *) * cap->num_active);
  if (cc == NULL)
    {
      ret_status = EXIT_FAILURE;
      goto cc_err;
    }
  for (i = 0; i < cap->num_active; ++i)
    {
      cc[i] = NULL;
    }

  for (i = 0; i < cap->num_active; ++i)
    {
      int channel = 0;

      if (frames[i].format == CAPTURE_FRAME_FORMAT_GRAY)
        {
          channel = 1;
        }
      else if (frames[i].format == CAPTURE_FRAME_FORMAT_RGB)
        {
          channel = 3;
        }

      images[i] = cvCreateImage (cvSize (frames[i].width, frames[i].height), IPL_DEPTH_8U, channel);

      if (images[i] == NULL)
        {
          while (--i >= 0)
            {
              cvReleaseImage (&images[i]);
            }
          ret_status = EXIT_FAILURE;
          goto iplimage_err;
        }
    }

  /* for display of detected result */
  overlay = (IplImage **) malloc (sizeof (IplImage *) * cap->num_active);
  if (overlay == NULL)
    {
      ret_status = EXIT_FAILURE;
      goto overlay_err;
    }

  for (i = 0; i < cap->num_active; ++i)
    {
      overlay[i] = cvCreateImage (cvSize (frames[i].width, frames[i].height), IPL_DEPTH_8U, 3);
      if (overlay[i] == NULL)
        {
          while (--i >= 0)
            {
              cvReleaseImage (&overlay[i]);
            }
          ret_status = EXIT_FAILURE;
          goto overlay_image_err;
        }
    }

  /* create window */
  for (i = 0; i < cap->num_active; ++i)
    {
      snprintf (winname[i], slen, "Camera %d", i);

      cvNamedWindow (winname[i], CV_WINDOW_AUTOSIZE);
    }
  

  /* main processing */
  exec_mode = EXEC_MODE_LIVE;
  num_frames = 0;
  gettimeofday (&prev, NULL);
  while (!to_exit)
    {
      char ch;

      capture_multi (cap, frames);

      /* fps calculation */
      ++num_frames;
      if (num_frames >= count_frames)
        {
          gettimeofday (&current, NULL);
          timersub (&current, &prev, &elapsed);

          fprintf (stderr, "%.2f fps\n", (double) count_frames / ((double) elapsed.tv_sec + (double) elapsed.tv_usec/1000000.0));

          num_frames = 0;
          prev = current;
        }

      for (i = 0; i < cap->num_active; ++i)
        {
          s_convert_frame_to_iplimage (&frames[i], images[i]);
        }

      switch (exec_mode)
        {
        case EXEC_MODE_LIVE:
          for (i = 0; i < cap->num_active; ++i)
            {
              cvShowImage (winname[i], images[i]);
            }
          break;

        case EXEC_MODE_CHECKER:
          for (i = 0; i < cap->num_active; ++i)
            {
              cc[i] = s_detect_checker (images[i], opt);
        
              if (cc[i] != NULL)
                {
                  if (frames[i].format == CAPTURE_FRAME_FORMAT_GRAY)
                    {
                      cvCvtColor (images[i], overlay[i], CV_GRAY2BGR);
                    }
                  else
                    {
                      cvCopy (images[i], overlay[i], NULL);
                    }

                  s_draw_checker (overlay[i], cc[i], opt);

                  cvShowImage (winname[i], overlay[i]);
                }
              else
                {
                  cvShowImage (winname[i], images[i]);
                }
            }
          break;
      
        default:
          fprintf (stderr, "unknown exec_mode.(%d)\n", exec_mode);
        }

      ch = cvWaitKey (2);

      switch (ch)
        {
        case 'q':
        case 0x1b:
          to_exit = -1;
          break;

        case 'c':
          exec_mode = EXEC_MODE_CHECKER;
          printf ("mode: Chessboard pattern detection");
          switch (opt->method)
            {
            case METHOD_OPENCV:
              printf ("(%dx%d)\n", opt->pattern_col, opt->pattern_row);
              break;

            default:
              printf ("\n");
            }
          break;

        case 'l':
          exec_mode = EXEC_MODE_LIVE;
          printf ("mode: live\n");
          break;

        case 'p':
          if (exec_mode == EXEC_MODE_LIVE)
            {
              struct timeval captured_time;
              char timestamp[32];

              /* create a timestamp */
              if (gettimeofday (&captured_time, NULL) == 0)
                {
                  struct tm lt;
                  int len;

                  localtime_r (&captured_time.tv_sec, &lt);
                  strftime (timestamp, sizeof (timestamp), "%Y%m%d-%H%M%S", &lt);

                  /* append usec value */
                  len = strlen (timestamp);
                  snprintf (&timestamp[len], sizeof (timestamp) - len, ".%06ld", captured_time.tv_usec);
                }
              else
                {
                  strncpy (timestamp, "unknown", sizeof (timestamp) - 1);
                  timestamp[sizeof (timestamp) - 1] = '\0';
                }

              for (i = 0; i < cap->num_active; ++i)
                {
                  char filename[256];
                  snprintf (filename, sizeof (filename), "cap_%s_%02u.png", timestamp, i);

                  cvSaveImage (filename, images[i], 0);
                }

              fprintf (stderr, "cap_%s_*.png saved.\n", timestamp);
            }
          break;

        case 0x0a:
          if (exec_mode == EXEC_MODE_CHECKER)
            {
              int num_detected = 0;
              checker_point_t *planes = NULL;

              /* check if the pattern was detected in all cameras */
              for (i = 0; i < cap->num_active; ++i)
                {
                  if (cc[i] != NULL)
                    {
                      ++num_detected;
                    }
                }
              if (num_detected < cap->num_active)
                {
                  break;
                }

              /* prepare a new space */
              planes = checker_data_new_observation (&checker_data);
              if (planes == NULL)
                {
                  break;
                }

              for (i = 0; i < cap->num_active; ++i)
                {
                  int j;

                  if (checker_point_resize (&planes[i], cc[i]->num_points) != 0)
                    {
                      continue;
                    }

                  /* copy coordinates */
                  for (j = 0; j < cc[i]->num_points; ++j)
                    {
                      int k;
                      for (k = 0; k < 2; ++k)
                        {
                          planes[i].plane_coord[j][k] = cc[i]->checker_pos[j][k] * checker_data.size;
                          planes[i].image_coord[j][k] = cc[i]->image_pos[j][k];
                        }
                    }
                }
              fprintf (stderr, "added the detected points. (%d)\n", checker_data.num_observations);
            }
          break;

        case 'W':
          checker_data_write (&checker_data, opt->fp);
          if (opt->fp != stdout)
            {
              fprintf (stderr, "output to a file.\n");
            }
          break;
        }

      /* free memory */
      if (exec_mode == EXEC_MODE_CHECKER)
        {
          for (i = 0; i < cap->num_active; ++i)
            {
              dc_checker_free (cc[i]);
              cc[i] = NULL;
            }
        }
    }

  /* destroy windows */
  cvDestroyAllWindows ();

  /* free memory */
  for (i = 0; i < cap->num_active; ++i)
    {
      cvReleaseImage (&overlay[i]);
    }
 overlay_image_err:
  free (overlay);
 overlay_err:
  for (i = 0; i < cap->num_active; ++i)
    {
      cvReleaseImage (&images[i]);
    }
 iplimage_err:
  free (cc);
 cc_err:
  free (images);
 images_err:
  free (winname);
 winname_err:

  checker_data_final (&checker_data);

  fprintf (stderr, "quit.\n");
  return EXIT_SUCCESS;
}

static void
s_convert_frame_to_iplimage (capture_frame_t *frame, IplImage *ipl)
{
  IplImage temp;

  switch (frame->format)
    {
    case CAPTURE_FRAME_FORMAT_GRAY:
      memcpy (ipl->imageData, frame->raw_data, frame->width * frame->height);
      break;

    case CAPTURE_FRAME_FORMAT_RGB:
      cvInitImageHeader (&temp, cvSize (frame->width, frame->height), IPL_DEPTH_8U, 3, 0, 4);
      temp.imageData = frame->raw_data;
      cvCvtColor (&temp, ipl, CV_RGB2BGR);
      break;

    default:
      ;
    }
}

static checker_coord_t *
s_detect_checker(IplImage *image, opt_t *opt)
{
  IplImage *gray_img = NULL;
  checker_coord_t *cc = NULL;
  int i, j, n;

  /* convert to gray image if the input is color */
  if (image->nChannels > 1)
    {
      gray_img = cvCreateImage (cvGetSize (image), image->depth, 1);
      if (gray_img == NULL)
        {
          return NULL;
        }

      cvCvtColor (image, gray_img, CV_RGB2GRAY);
    }
  else
    {
      gray_img = image;
    }

  /* chessboard pattern detection */
  switch (opt->method)
    {
    case METHOD_OPENCV:
      {
        CvPoint2D32f *corners = NULL;
        int count = 0;

        /* allocate memory */
        corners = (CvPoint2D32f *) cvAlloc (sizeof (CvPoint2D32f) * opt->pattern_col * opt->pattern_row);
        if (corners == NULL)
          {
            return NULL;
          }

        /* detect corners */
        if (cvFindChessboardCorners (gray_img, cvSize (opt->pattern_col, opt->pattern_row), corners, &count,
                                    CV_CALIB_CB_ADAPTIVE_THRESH
                                    | CV_CALIB_CB_NORMALIZE_IMAGE
                                    | CV_CALIB_CB_FILTER_QUADS
#if ((CV_MAJOR_VERSION) == 2) && ((CV_MINOR_VERSION) > 0)
                                    | CV_CALIB_CB_FAST_CHECK
#endif
                                    ) != 0)
          {

            cvFindCornerSubPix (gray_img, corners, count, cvSize (5, 5), cvSize (-1, -1), cvTermCriteria (CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 15, 0.01));

            /* substitute the result to checker_coord_t structure */
            cc = dc_checker_alloc (count);
            if (cc == NULL)
              {
                return NULL;
              }

            cc->col = opt->pattern_col;
            cc->row = opt->pattern_row;

            for (i = 0, n = 0; i < cc->row && n < count; ++i)
              {
                for (j = 0; j < cc->col && n < count; ++j)
                  {
                    cc->checker_pos[i*cc->col + j][0] = (double) j;
                    cc->checker_pos[i*cc->col + j][1] = (double) i;

                    cc->image_pos[i*cc->col + j][0] = corners[n].x;
                    cc->image_pos[i*cc->col + j][1] = corners[n].y;

                    ++n;
                  }
              }
          }

        /* free the allocated memory */
        cvFree (&corners);
        break;
      }
    
    default:
      break;
    }

  /* free allocated memory if needed */
  if (image->nChannels > 1)
    {
      cvReleaseImage (&gray_img);
    }

  return cc;
}

static void
s_draw_checker (IplImage *image, checker_coord_t *cc, opt_t *opt)
{
  switch (opt->method)
    {
    case METHOD_OPENCV:
      {
        CvPoint2D32f *corners = NULL;
        int i;

        corners = (CvPoint2D32f *) cvAlloc (sizeof (CvPoint2D32f) * cc->num_points);
        if (corners == NULL)
          {
            return;
          }

        for (i = 0; i < cc->num_points; ++i)
          {
            corners[i].x = cc->image_pos[i][0];
            corners[i].y = cc->image_pos[i][1];
          }

        cvDrawChessboardCorners (image, cvSize (opt->pattern_col, opt->pattern_row), corners, cc->num_points, 1);

        cvFree (&corners);

        break;
      }

    default:
      break;
    }
}
