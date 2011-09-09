/*
 checker_data.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#include <stdio.h>
#include <stdlib.h>

#include "checker_data.h"

static int s_checker_data_add_observe (checker_data_t *cd);

static void s_checker_point_final (checker_point_t *cp);

void
checker_data_init (checker_data_t *cd, const double size, const int num_cameras)
{
  static const int capacity_initial = 16;

  if (cd == NULL)
    {
      return;
    }

  cd->size             = size;
  cd->num_cameras      = num_cameras;
  cd->num_observations = 0;
  cd->capacity         = 0;
  cd->data             = NULL;

  cd->data = (checker_point_t **) malloc (sizeof (checker_point_t *) * capacity_initial);
  if (cd->data != NULL)
    {
      int i;

      for (i = 0; i < capacity_initial; ++i)
        {
          cd->data[i] = NULL;
        }
      cd->capacity = capacity_initial;
    }
}

void
checker_data_clear (checker_data_t *cd)
{
  int i, n;

  if (cd == NULL)
    {
      return;
    }

  n = cd->num_observations;
  cd->num_observations = 0;
  for (i = 0; i < n; ++i)
    {
      int j;

      if (cd->data[i] == NULL)
        {
          continue;
        }

      for (j = 0; j < cd->num_cameras; ++j)
        {
          s_checker_point_final (&cd->data[i][j]);
        }
      free (cd->data[i]);
      cd->data[i] = NULL;
    }
}

void
checker_data_final (checker_data_t *cd)
{
  if (cd == NULL)
    {
      return;
    }

  checker_data_clear (cd);
  cd->capacity = 0;
  free (cd->data);

  cd->num_cameras = 0;
  cd->size        = 0;
}

void
checker_data_write (checker_data_t *cd, FILE *fp)
{
  int i, j, k;

  fprintf (fp, "MLTCLB 1\n");

  fprintf (fp, "%g 90\n", cd->size);
  fprintf (fp, "%d %d\n", cd->num_observations, cd->num_cameras);

  for (i = 0; i < cd->num_observations; ++i)
    {
      if (cd->data[i] == NULL)
        {
          continue;
        }

      for (j = 0; j < cd->num_cameras; ++j)
        {
          checker_point_t *cp = &cd->data[i][j];
          fprintf (fp, "%d\n", cp->num);
          for (k = 0; k < cp->num; ++k)
            {
              fprintf (fp, "% .20e % .20e % .20e % .20e\n", cp->plane_coord[k][0], cp->plane_coord[k][1], cp->image_coord[k][0], cp->image_coord[k][1]);
            }
        }
    }
}

checker_point_t *
checker_data_new_observation (checker_data_t *cd)
{
  if (s_checker_data_add_observe (cd) != 0)
    {
      return NULL;
    }
  return cd->data[cd->num_observations - 1];
}

int
checker_point_resize (checker_point_t *cp, const int num_points)
{
  double (*p)[2] = NULL;

  if (num_points < cp->num)
    {
      cp->num = num_points;
      return 0;
    }

  p = (double (*)[2]) realloc (cp->plane_coord, sizeof (double) * num_points * 2);
  if (p == NULL)
    {
      return -1;
    }
  cp->plane_coord = p;

  p = (double (*)[2]) realloc (cp->image_coord, sizeof (double) * num_points * 2);
  if (p == NULL)
    {
      return -1;
    }
  cp->image_coord = p;

  cp->num = num_points;
  return 0;
}

/*
    static function
*/
static int
s_checker_data_add_observe (checker_data_t *cd)
{
  static const int step = 16;
  int i, n;

  if (cd->num_observations + 1 >= cd->capacity)
    {
      checker_point_t **data_new = NULL;

      data_new = (checker_point_t **) realloc (cd->data, sizeof (checker_point_t *) * (cd->capacity + step));
      if (data_new == NULL)
        {
          return -1;
        }

      cd->data = data_new;
      for (i = cd->capacity; i < cd->capacity + step; ++i)
        {
          cd->data[i] = NULL;
        }
      cd->capacity += step;
    }

  n = cd->num_observations;
  cd->data[n] = (checker_point_t *) (malloc (sizeof (checker_point_t) * cd->num_cameras));
  for (i = 0; i < cd->num_cameras; ++i)
    {
      cd->data[n][i].num = 0;
      cd->data[n][i].plane_coord = NULL;
      cd->data[n][i].image_coord = NULL;
    }

  ++cd->num_observations;

  return 0;
}

static void
s_checker_point_final (checker_point_t *cp)
{
  if (cp == NULL)
    {
      return;
    }

  cp->num = 0;
  if (cp->plane_coord != NULL)
    {
      free (cp->plane_coord);
    }

  if (cp->image_coord != NULL)
    {
      free (cp->image_coord);
    }
}
