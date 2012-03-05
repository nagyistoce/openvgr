/*
 reset_bus.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#include <stdio.h>
#include <stdlib.h>

#include <getopt.h>

#ifdef __cplusplus
#  define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <dc1394/dc1394.h>
#include <dc1394/linux/control.h>

#define MAX_NUM_ID (16)

typedef struct tag_opts
{
  int show_list;
  int bus_id[MAX_NUM_ID];
  int num_id;
} opts_t;
const opts_t OPTS_INIT = { 0, {0}, 0 };

static void show_help ();
static void parse_opts (opts_t *opts, int argc, char **argv);
static void reset_bus (opts_t *opts, dc1394_t *dc1394_cxt, dc1394camera_list_t *cam_list);
static void list_cameras (dc1394_t *dc1394_cxt, dc1394camera_list_t *cam_list);

int
main (int argc, char **argv)
{
  opts_t opts = OPTS_INIT;

  dc1394_t *dc1394_cxt = NULL;
  dc1394camera_list_t *cam_list = NULL;
  dc1394error_t err;

  if (argc < 2)
    {
      show_help ();
      return EXIT_SUCCESS;
    }

  parse_opts (&opts, argc, argv);
  dc1394_cxt = dc1394_new ();
  if (dc1394_cxt == NULL)
    {
      printf ("error: dc1394_new() failed.\n");
      return EXIT_FAILURE;
    }

  err = dc1394_camera_enumerate (dc1394_cxt, &cam_list);
  if (err != DC1394_SUCCESS || cam_list == NULL)
    {
      printf ("error: dc1394_camera_enumerate() failed.\n");
      dc1394_free (dc1394_cxt);
      return EXIT_FAILURE;
    }

  /* reset specified buses */
  if (opts.show_list == 0)
    {
      reset_bus (&opts, dc1394_cxt, cam_list);
    }
  else  /* show the list of detected cameras */
    {
      list_cameras (dc1394_cxt, cam_list);
    }

  dc1394_camera_free_list (cam_list);
  dc1394_free (dc1394_cxt);
  return EXIT_SUCCESS;
}

static void
show_help ()
{
  printf ("usage: reset_bus [-[hl] | [<bus_id> [<bus_id>...]]\n");

  printf ("\n");

  printf ("-h: show this help.\n");
  printf ("-l: show a list of available bus id(s).\n");
}

static void
parse_opts (opts_t *opts, int argc, char **argv)
{
  static char optstr[] = "hl";
  static struct option longopts[] = {
    {"help", no_argument, NULL, 'h'},
    {"list", no_argument, NULL, 'l'},
    {"", 0, NULL, 0}
  };

  int optchar, i, n;

  while ((optchar = getopt_long (argc, argv, optstr, longopts, NULL)) >= 0)
    {
      switch (optchar)
        {
        case 'h':
          show_help ();
          exit (EXIT_SUCCESS);
          break;

        case 'l':
          opts->show_list = -1;
          break;

        case '?':
        default:
          printf ("error: parse_opts\n");
        exit (EXIT_FAILURE);
        }
    }

  n = 0;
  for (i = optind; i < argc && n < MAX_NUM_ID; ++i)
    {
      int j, new_id, found;

      new_id = atoi (argv[i]);

      /* check if the same id is already specified */
      found = 0;
      for (j = 0; j < n; ++j)
        {
          if (opts->bus_id[j] == new_id)
            {
              found = -1;
              break;
            }
        }

      /* register the new_id to a list */
      if (found == 0)
        {
          opts->bus_id[n] = new_id;
          ++n;
        }
    }
  opts->num_id = n;
}

static void
reset_bus (opts_t *opts, dc1394_t *dc1394_cxt, dc1394camera_list_t *cam_list)
{
  int is_found[MAX_NUM_ID] = { 0 };
  dc1394camera_id_t target[MAX_NUM_ID];
  dc1394error_t err;

  int i;

  /* search target cameras connected to the buses for resetting */
  for (i = 0; i < cam_list->num; ++i)
    {
      dc1394camera_t *camera = NULL;

      camera = dc1394_camera_new_unit (dc1394_cxt, cam_list->ids[i].guid, cam_list->ids[i].unit);
      if (camera != NULL)
        {
          uint32_t port = -1;
          err = dc1394_camera_get_linux_port (camera, &port);
          if (err == DC1394_SUCCESS)
            {
              int j;

              for (j = 0; j < opts->num_id; ++j)
                {
                  if ((is_found[j] == 0) && (opts->bus_id[j] == port))
                    {
                      target[j] = cam_list->ids[i];
                      is_found[j] = -1;
                      break;
                    }
                }
            }

          dc1394_camera_free (camera);
          camera = NULL;
        }
    }

  /* reset buses in the specified order */
  for (i = 0; i < opts->num_id; ++i)
    {
      if (is_found[i] != 0)
        {
          dc1394camera_t *camera = NULL;

          printf ("resetting bus %d...", opts->bus_id[i]);

          camera = dc1394_camera_new_unit (dc1394_cxt, target[i].guid, target[i].unit);
          if (camera != NULL)
            {
              err = dc1394_reset_bus (camera);
              dc1394_camera_free (camera);
              camera = NULL;
            }
          else
            {
              err = DC1394_FAILURE;
            }

          if (err == DC1394_SUCCESS)
            {
              printf ("OK\n");
            }
          else
            {
              printf ("NG\n");
            }
        }
      else
        {
          printf ("warning: bus %d is not found.\n", opts->bus_id[i]);
        }
    }
}

static void
list_cameras (dc1394_t *dc1394_cxt, dc1394camera_list_t *cam_list)
{
  dc1394error_t err;
  int i;

  printf ("bus_id\tguid              \tunit\tmodel (vendor)\n");
  for (i = 0; i < cam_list->num; ++i)
    {
      dc1394camera_t *camera = NULL;

      camera = dc1394_camera_new_unit (dc1394_cxt, cam_list->ids[i].guid, cam_list->ids[i].unit);
      if (camera != NULL)
        {
          uint32_t port = -1;
          err = dc1394_camera_get_linux_port (camera, &port);
          if (err == DC1394_SUCCESS)
            {
              printf ("%6d", port);
            }
          else
            {
              printf ("   n/a");
            }
          printf ("\t0x%016" PRIx64 "\t%4" PRIu16 "\t%s (%s)\n",
                  camera->guid, camera->unit, camera->model, camera->vendor);
          dc1394_camera_free (camera);
          camera = NULL;
        }
    }
}
