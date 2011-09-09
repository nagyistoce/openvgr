/*
 calib_data.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <satoshi.kawabata@aist.go.jp>

 $Date::                            $
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "calib_data.h"

static const char magic[] = "MLTCLB";

int
cdata_create (cdata_t **cdata)
{
  if (cdata == NULL)
    {
      return CDATA_ERROR;
    }

  *cdata = (cdata_t *) malloc (sizeof (cdata_t));
  if (*cdata == NULL)
    {
      return CDATA_ERROR;
    }

  cdata_init (*cdata);

  return CDATA_SUCCESS;
}

int
cdata_init (cdata_t *cdata)
{
  if (cdata == NULL)
    {
      return CDATA_ERROR;
    }

  cdata->interval = 0;
  cdata->rotation_deg = 0;

  cdata->num_observations = 0;
  cdata->num_cameras = 0;

  cdata->data = NULL;
  cdata->weights = NULL;

  return CDATA_SUCCESS;
}

int
cdata_alloc (cdata_t *cdata, const double interval, const double deg,
            const int nobserv, const int ncamera)
{
  int i;

  if (cdata == NULL || interval <= 0.0 || deg < 0 || nobserv < 1 || ncamera < 1)
    {
      return CDATA_ERROR;
    }

  cdata->data = (cdata_observed_points_t *) malloc (sizeof (cdata_observed_points_t) * nobserv * ncamera);
  if (cdata->data == NULL)
    {
      return CDATA_ERROR;
    }

  cdata->interval = interval;
  cdata->rotation_deg = deg;
  cdata->num_observations = nobserv;
  cdata->num_cameras = ncamera;

  for (i = 0; i < nobserv * ncamera; ++i)
    {
      cdata_observed_points_init (&cdata->data[i]);
    }

  return CDATA_SUCCESS;
}

int
cdata_free (cdata_t *cdata)
{
  int i;

  if (cdata == NULL)
    {
      return CDATA_ERROR;
    }

  for (i = 0; i < cdata->num_observations * cdata->num_cameras; ++i)
    {
      cdata_observed_points_final (&cdata->data[i]);
    }

  cdata->num_observations = 0;
  cdata->num_cameras = 0;

  free (cdata->data);
  cdata->data = NULL;

  return CDATA_SUCCESS;
}

int
cdata_final (cdata_t *cdata)
{
  if (cdata == NULL)
    {
      return CDATA_SUCCESS;
    }

  cdata_free (cdata);

  cdata->interval = 0;
  cdata->rotation_deg = 0;

  return CDATA_SUCCESS;
}

int
cdata_destroy (cdata_t **cdata)
{
  if (cdata == NULL || *cdata == NULL)
    {
      return CDATA_SUCCESS;
    }

  cdata_final (*cdata);
  free (*cdata);
  *cdata = NULL;

  return CDATA_SUCCESS;
}

int
cdata_read (cdata_t *cdata, FILE *fp)
{
  int version, nobserv, ncamera;
  double interval, deg;
  char buf[256];
  int i, j, k;

  if (cdata == NULL || fp == NULL)
    {
      return CDATA_ERROR;
    }

  if (cdata_init (cdata) != CDATA_SUCCESS)
    {
      return CDATA_ERROR;
    }

  /* check magic code */
  if (fscanf (fp, "%255s", buf) != 1)
    {
      return CDATA_ERROR;
    }
  if (strncmp (magic, buf, sizeof (magic)) != 0)
    {
      fprintf (stderr, "not valid data.\n");
      return CDATA_ERROR;
    }

  /* version */
  if (fscanf (fp, "%d", &version) != 1)
    {
      return CDATA_ERROR;
    }
  if (version != 1)
    {
      return CDATA_ERROR;
    }

  /* interval, rotation_deg, num_observations, num_cameras */
  if (fscanf (fp, "%lf %lf %d %d", &interval, &deg, &nobserv, &ncamera) != 4)
    {
      fprintf (stderr, "not valid data.\n");
      return CDATA_ERROR;
    }

  if (cdata_alloc (cdata, interval, deg, nobserv, ncamera) != CDATA_SUCCESS)
    {
      return CDATA_ERROR;
    }

  /* skip to the next line */
  while (fgetc (fp) != '\n');

  /* read each point data */
  for (i = 0; i < nobserv; ++i)
    {
      for (j = 0; j < ncamera; ++j)
        {
          int npoint;

          /* read num of points */
          if (fgets (buf, sizeof (buf), fp) == NULL)
            {
              fprintf (stderr, "not valid data.\n");
              cdata_free (cdata);
              return CDATA_ERROR;
            }

          if (sscanf (buf, "%d", &npoint) != 1)
            {
              fprintf (stderr, "not valid data.\n");
              cdata_free (cdata);
              return CDATA_ERROR;
            }

          /* allocate memory to store points */
          if (cdata_observed_points_alloc (&cdata->data[i*ncamera + j], npoint) != CDATA_SUCCESS)
            {
              fprintf (stderr, "could not allocate memory.\n");
              cdata_free (cdata);
              return CDATA_ERROR;
            }

          /* read point pairs */
          for (k = 0; k < npoint; ++k)
            {
              cdata_observed_points_t *const op = &cdata->data[i*ncamera + j];

              if (fgets (buf, sizeof (buf), fp) == NULL)
                {
                  fprintf (stderr, "not valid data.\n");
                  cdata_free (cdata);
                  return CDATA_ERROR;
                }

              if (sscanf (buf, "%lf %lf %lf %lf", &op->ref[k][0], &op->ref[k][1], &op->cam[k][0], &op->cam[k][1]) != 4)
                {
                  fprintf (stderr, "not valid data.\n");
                  cdata_free (cdata);
                  return CDATA_ERROR;
                }
            }
        }
    }
  
  return CDATA_SUCCESS;
}

int
cdata_write (const cdata_t *cdata, FILE *fp)
{
  const int version = 1;
  int i;

  if (cdata == NULL || fp == NULL)
    {
      return CDATA_ERROR;
    }

  fprintf (fp, "%s %d\n", magic, version);
  fprintf (fp, "%g %g\n", cdata->interval, cdata->rotation_deg);
  fprintf (fp, "%d %d\n", cdata->num_observations, cdata->num_cameras);

  for (i = 0; i < cdata->num_observations * cdata->num_cameras; ++i)
    {
      cdata_observed_points_t *const op = &cdata->data[i];
      int j;

      fprintf (fp, "%d\n", op->num_points);
      for (j = 0; j < op->num_points; ++j)
        {
          fprintf (fp, "% .20e % .20e % .20e % .20e\n", op->ref[j][0], op->ref[j][1], op->cam[j][0], op->cam[j][1]);
        }
    }

  return CDATA_SUCCESS;
}



int
cdata_observed_points_init (cdata_observed_points_t *op)
{
  if (op == NULL)
    {
      return CDATA_ERROR;
    }

  op->num_points = 0;
  op->ref = NULL;
  op->cam = NULL;
  op->weights = NULL;

  return CDATA_SUCCESS;
}

int
cdata_observed_points_alloc (cdata_observed_points_t *op, const int npoint)
{
  cdata_point_t *new_ref = NULL, *new_cam = NULL;

  if (op == NULL)
    {
      return CDATA_ERROR;
    }

  new_ref = (cdata_point_t *) realloc (op->ref, sizeof (cdata_point_t) * npoint);
  if (new_ref == NULL)
    {
      return CDATA_ERROR;
    }

  new_cam = (cdata_point_t *) realloc (op->cam, sizeof (cdata_point_t) * npoint);
  if (new_cam == NULL)
    {
      free (new_ref);
      return CDATA_ERROR;
    }

  op->ref = new_ref;
  op->cam = new_cam;

  op->num_points = npoint;

  return CDATA_SUCCESS;
}

int
cdata_observed_points_free (cdata_observed_points_t *op)
{
  if (op == NULL)
    {
      return CDATA_ERROR;
    }

  op->num_points = 0;

  free (op->ref); op->ref = NULL;
  free (op->cam); op->cam = NULL;

  return CDATA_SUCCESS;
}

int
cdata_observed_points_final (cdata_observed_points_t *op)
{
  return cdata_observed_points_free (op);
}
