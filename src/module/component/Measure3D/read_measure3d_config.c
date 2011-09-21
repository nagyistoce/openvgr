/*
 read_measure3d_config.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*
  int disparity_type	int

  CvStereoBMState:
    // pre-filtering (normalization of input images)
    int preFilterType; // =CV_STEREO_BM_NORMALIZED_RESPONSE now
    int preFilterSize; // averaging window size: ~5x5..21x21
    int preFilterCap; // the output of pre-filtering is clipped by [-preFilterCap,preFilterCap]

    // correspondence using Sum of Absolute Difference (SAD)
    int SADWindowSize; // ~5x5..21x21
    int minDisparity;  // minimum disparity (can be negative)
    int numberOfDisparities; // maximum disparity - minimum disparity (> 0)

    // post-filtering
    int textureThreshold;  // the disparity is only computed for pixels
                           // with textured enough neighborhood
    int uniquenessRatio;   // accept the computed disparity d* only if
                           // SAD(d) >= SAD(d*)*(1 + uniquenessRatio/100.)
                           // for any d != d*+/-1 within the search range.
    int speckleWindowSize; // disparity variation window
    int speckleRange; // acceptable range of variation in window

    int trySmallerWindows; // if 1, the results may be more accurate,
                           // at the expense of slower processing 

 */

#include <stdio.h>

#include <cv.h>
#include "Img.hh"
#include "measure3d_scene.h"

// the number of variables in CvStereoBMState
#define N_CVSTEREO_ARGS	(11)

#define	STRLEN	(256)

static int
skip_space(char *str, int k)
{
  while (str[k] == ' ' || str[k] == '\t')
    {
      k++;
    }
  if (str[k] == '\0') return -1;

  return k;
}

static int
skip_nonspace(char *str, int k)
{
  while (str[k] != ' ' && str[k] != '\t' && str[k] != '\0')
    {
      k++;
    }
  return k;
}

static int
get_token(char *str, char **token)
{
  int len, k0, k1, k2, k3;

  len = strlen(str);
  k0 = skip_space(str, 0);
  if (k0 < 0)
    {
      return 0;
    }
  k1 = skip_nonspace(str, k0);
  k2 = skip_space(str, k1);
  if (k2 < 0)
    {
      return 1;
    }
  k3 = skip_nonspace(str, k2);

  token[0] = &str[k0];
  token[1] = &str[k2];
  str[k1] = '\0';
  str[k3] = '\0';

  return 2;
}

int
read_measure3d_config(char *fname, CvStereoBMState *state)
{
  const char *name[N_CVSTEREO_ARGS] =
    {
      "preFilterType", 
      "preFilterSize", 
      "preFilterCap", 
      "SADWindowSize", 
      "minDisparity",  
      "numberOfDisparities", 
      "textureThreshold",  
      "uniquenessRatio",   
      "speckleWindowSize", 
      "speckleRange", 
      "trySmallerWindows"
    };
  int	flag[N_CVSTEREO_ARGS];
  FILE	*fp;
  int	i, nline;
  char	str[STRLEN], *token[2];
  int	ntoken;
  int	val;
  int	check;

  fp = fopen(fname, "r");
  if (fp == NULL)
    {
      return ERROR_OPEN_CONFIG_FILE;
    }

  for (i = 0; i < N_CVSTEREO_ARGS; i++)
    {
      flag[i] = 0;
    }

  nline = 0;
  while (fgets(str, STRLEN, fp))
    {
      ntoken = get_token(str, token);
      if (ntoken < 2)
        {
          fprintf (stderr,
                   "Err: read_measure3d_config()"
                   " ntoken is %d != 2 at line %d\n",
                   ntoken, nline);
          fclose(fp);
          return ERROR_PARSE_CONFIG_FILE;
        }
      if (sscanf(token[1], "%d", &val) != 1)
        {
          fprintf(stderr,
                  "Err: read_measure3d_config()"
                  " failed to scanf value(%s) at line %d\n",
                  token[1], nline);
          fclose(fp);
          return ERROR_PARSE_CONFIG_FILE;
        }
      check = 1;
      for (i = 0; check && i < N_CVSTEREO_ARGS; i++)
        {
          if (strcmp(token[0], name[i]) == 0)
            {
              check = 0;
              flag[i] = 1;
              (&(state->preFilterType))[i] = val;
            }
        }
      if (check ==1)
        {
          fprintf(stderr,
                  "Err: read_measure3d_config()"
                  " name mismatch(%s) at line %d\n",
                  token[0], nline);
          fclose(fp);
          return ERROR_PARSE_CONFIG_FILE;
        }
      nline++;
    }

  for (i = 0; i < N_CVSTEREO_ARGS; i++)
    {
      if (flag[i] == 0)
        {
          fprintf(stderr,
                  "Err: read_measure3d_config()"
                  " some value(s) are missing\n");
          fclose(fp);
          return ERROR_PARSE_CONFIG_FILE;
        }
    }
  
  fclose(fp);

  /* set NULL to all temporary pointers in CvStereoBMState struct*/
  state->preFilteredImg0 = NULL;
  state->preFilteredImg1 = NULL;
  state->slidingSumBuf = NULL;
#if (!defined(CV_VERSION))
  || (((CV_MAJOR_VERSION) == 2) && ((CV_MINOR_VERSION) == 0))
  state->dbmin = NULL;
  state->dbmax = NULL;
#endif

  return 0;
}
