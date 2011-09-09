/*
 genconf.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <satoshi.kawabata@aist.go.jp>

 $Date::                            $
*/

#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus

#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <dc1394/dc1394.h>

static dc1394video_mode_t
s_get_video_mode (dc1394camera_t *camera)
{
  dc1394video_mode_t mode = DC1394_VIDEO_MODE_640x480_MONO8;

  uint32_t bits;
  dc1394video_modes_t modes;
  dc1394error_t err;

  int i;

  err = dc1394_video_get_supported_modes (camera, &modes);
  if (err == DC1394_SUCCESS)
    {
      bits = 0;
      i = modes.num - 1;
      do
        {
          dc1394color_coding_t color_coding;

          if (modes.modes[i] >= DC1394_VIDEO_MODE_FORMAT7_MIN)
            {
              continue;
            }

          err = dc1394_get_color_coding_from_video_mode (camera, modes.modes[i], &color_coding);
          DC1394_WRN (err, "get_color_coding_from_video_mode");

          if (err != DC1394_SUCCESS)
            {
              continue;
            }

          err = dc1394_get_color_coding_data_depth (color_coding, &bits);
          DC1394_WRN (err, "get_color_coding_data_depth");
        }
      while (bits != 8 && --i >= 0);

      if (bits == 8)
        {
          mode = modes.modes[i];
        }
    }

  return mode;
}

static dc1394framerate_t
s_get_maximum_framerate (dc1394camera_t *camera, dc1394video_mode_t mode)
{
  dc1394framerates_t framerates;
  dc1394error_t err;

  err = dc1394_video_get_supported_framerates (camera, mode, &framerates);
  if (err != DC1394_SUCCESS)
    {
      return DC1394_FRAMERATE_30;
    }

  return framerates.framerates[framerates.num - 1];
}

static const char *
s_get_color_mode_str (dc1394camera_t *camera, dc1394video_mode_t mode)
{
  dc1394error_t err;
  dc1394color_coding_t coding;

  const static char *mode_str[DC1394_COLOR_CODING_NUM + 1] = {
    "Y(mono)",
    "YUV(4:1:1)",
    "YUV(4:2:2)",
    "YUV(4:4:4)",
    "RGB",
    "Y(mono16)",
    "(unsupported)",            /* RGB16   */
    "(unsupported)",            /* MONO16S */
    "(unsupported)",            /* RGB16S  */
    "(unsupported)",            /* RAW8    */
    "(unsupported)",            /* RAW16   */
    "(unknown)"
  };

  err = dc1394_get_color_coding_from_video_mode (camera, mode, &coding);
  if (err != DC1394_SUCCESS)
    {
      return mode_str[DC1394_COLOR_CODING_NUM];
    }

  return mode_str[coding - DC1394_COLOR_CODING_MIN];
}

int
main (int argc, char **argv)
{
  dc1394_t *dc1394_cxt = NULL;
  dc1394camera_list_t *cam_list = NULL;
  dc1394error_t err;
  int i;

  dc1394_cxt = dc1394_new ();
  if (dc1394_cxt == NULL)
    {
      return EXIT_FAILURE;
    }

  err = dc1394_camera_enumerate (dc1394_cxt, &cam_list);
  if (err != DC1394_SUCCESS || cam_list == NULL)
    {
      dc1394_free (dc1394_cxt);
      return EXIT_FAILURE;
    }

  fprintf (stderr, "%d camera(s) found.\n", cam_list->num);

  printf ("0 %d\n", cam_list->num);
  for (i = 0; i < cam_list->num; ++i)
    {
      dc1394camera_t *camera = NULL;
      dc1394video_mode_t mode;
      dc1394framerate_t framerate;

      uint32_t width = 0, height = 0;
      float fps = 0.0;

      camera = dc1394_camera_new (dc1394_cxt, cam_list->ids[i].guid);
      if (camera == NULL)
        {
          fprintf (stderr, "camera 0x%016" PRIx64 " isn't available.\n", cam_list->ids[i].guid);
          break;
        }

      /* GUID */
      printf ("0x%016" PRIx64, cam_list->ids[i].guid);

      /* image size */
      mode = s_get_video_mode (camera);
      err = dc1394_get_image_size_from_video_mode (camera, mode, &width, &height);
      DC1394_WRN (err, "get_image_size_from_video_mode");

      /* framerate */
      framerate = s_get_maximum_framerate (camera, mode);
      err = dc1394_framerate_as_float (framerate, &fps);
      DC1394_WRN (err, "framerate_as_float");

      /* configulations */
      printf (" %dx%d-%s %gfps BRIGHTNESS 0 AUTO_EXPOSURE 0 GAMMA 1024 SHUTTER 650 GAIN 0\n", width, height, s_get_color_mode_str (camera, mode), fps);

      dc1394_camera_free (camera); camera = NULL;
    }

  dc1394_camera_free_list (cam_list); cam_list = NULL;
  dc1394_free (dc1394_cxt);

  return EXIT_SUCCESS;
}
