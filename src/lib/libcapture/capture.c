/*
 capture.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <time.h>

#ifdef __cplusplus
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <assert.h>

#include <dc1394/dc1394.h>
#include <dc1394/linux/control.h>

#include "capture.h"

typedef struct tag_camera_setting
{
  u_int64_t uid;

  dc1394video_mode_t mode;
  dc1394framerate_t framerate;

  /* features */
  int brightness;
  int exposure;
  int sharpness;
  int white_balance[2]; /* 0:ub, 1:vr */
  int hue;
  int saturation;
  int gamma;
  int shutter;
  int gain;
  int iris;
  int focus;
  int temperature;

  int zoom;
  int pan;
  int tilt;
} camera_setting_t;

typedef struct tag_conf_param
{
  int delay;
  int num_cameras;

  camera_setting_t *settings;
} conf_param_t;

#define CONF_PARAM_INIT_VAL {0, 0, NULL};

typedef struct tag_conf_token
{
  const char *token;
  const int key;
} conf_token_t;

static const conf_token_t conf_mode[] = {
  {"160x120-YUV(4:4:4)",   DC1394_VIDEO_MODE_160x120_YUV444},
  {"320x240-YUV(4:2:2)",   DC1394_VIDEO_MODE_320x240_YUV422},
  {"640x480-YUV(4:1:1)",   DC1394_VIDEO_MODE_640x480_YUV411},
  {"640x480-YUV(4:2:2)",   DC1394_VIDEO_MODE_640x480_YUV422},
  {"640x480-RGB",          DC1394_VIDEO_MODE_640x480_RGB8},
  {"640x480-Y(mono)",      DC1394_VIDEO_MODE_640x480_MONO8},
  {"640x480-Y(mono16)",    DC1394_VIDEO_MODE_640x480_MONO16},

  {"800x600-YUV(4:2:2)",   DC1394_VIDEO_MODE_800x600_YUV422},
  {"800x600-RGB",          DC1394_VIDEO_MODE_800x600_RGB8},
  {"800x600-Y(mono)",      DC1394_VIDEO_MODE_800x600_MONO8},
  {"1024x768-YUV(4:2:2)",  DC1394_VIDEO_MODE_1024x768_YUV422},
  {"1024x768-RGB",         DC1394_VIDEO_MODE_1024x768_RGB8},
  {"1024x768-Y(mono)",     DC1394_VIDEO_MODE_1024x768_MONO8},
  {"800x600-Y(mono16)",    DC1394_VIDEO_MODE_800x600_MONO16},
  {"1024x768-Y(mono16)",   DC1394_VIDEO_MODE_1024x768_MONO16},

  {"1280x960-YUV(4:2:2)",  DC1394_VIDEO_MODE_1280x960_YUV422 },
  {"1280x960-RGB",         DC1394_VIDEO_MODE_1280x960_RGB8},
  {"1280x960-Y(mono)",     DC1394_VIDEO_MODE_1280x960_MONO8},
  {"1600x1200-YUV(4:2:2)", DC1394_VIDEO_MODE_1600x1200_YUV422},
  {"1600x1200-RGB",        DC1394_VIDEO_MODE_1600x1200_RGB8},
  {"1600x1200-Y(mono)",    DC1394_VIDEO_MODE_1600x1200_MONO8},
  {"1280x960-Y(mono16)",   DC1394_VIDEO_MODE_1280x960_MONO16},
  {"1600x1200-Y(mono16)",  DC1394_VIDEO_MODE_1600x1200_MONO16},
  {"", 0}
};


static const conf_token_t conf_framerate[] = {
  {"1.875fps", DC1394_FRAMERATE_1_875},
  {"3.75fps",  DC1394_FRAMERATE_3_75},
  {"7.5fps",   DC1394_FRAMERATE_7_5},
  {"15fps",    DC1394_FRAMERATE_15},
  {"30fps",    DC1394_FRAMERATE_30},
  {"60fps",    DC1394_FRAMERATE_60},
  {"120fps",   DC1394_FRAMERATE_120},
  {"240fps",   DC1394_FRAMERATE_240},
  {"", 0},
};


static const conf_token_t conf_feature[] = {
  {"BRIGHTNESS",    DC1394_FEATURE_BRIGHTNESS},
  {"AUTO_EXPOSURE", DC1394_FEATURE_EXPOSURE},
  {"SHARPNESS",     DC1394_FEATURE_SHARPNESS},
  {"WHITE_BALANCE", DC1394_FEATURE_WHITE_BALANCE},
  {"HUE",           DC1394_FEATURE_HUE},
  {"SATURATION",    DC1394_FEATURE_SATURATION},
  {"GAMMA",         DC1394_FEATURE_GAMMA},
  {"SHUTTER",       DC1394_FEATURE_SHUTTER},
  {"GAIN",          DC1394_FEATURE_GAIN},
  {"IRIS",          DC1394_FEATURE_IRIS},
  {"FOCUS",         DC1394_FEATURE_FOCUS},
  {"TEMPERATURE",   DC1394_FEATURE_TEMPERATURE},
  {"ZOOM",          DC1394_FEATURE_ZOOM},
  {"PAN",           DC1394_FEATURE_PAN},
  {"TILT",          DC1394_FEATURE_TILT},
  {"", 0},
};


static const size_t s_num_ports_max = 16;

static inline int s_byte_per_packet(const capture_frame_format_t format);
static int s_read_conf_file(const char *conf_file, conf_param_t *data);
static int s_clear_conf_param(conf_param_t *data);

static int s_get_capture_size(unsigned int mode, int *width, int *height);
static int s_get_capture_format(unsigned int mode, capture_frame_format_t *format);
// static int s_get_capture_framerate(unsigned int framerate, double *fps);

static int s_camera_setting_init(camera_setting_t *s);
#ifdef DEBUG_LIBCAPTURE
static void s_camera_setting_disp(camera_setting_t *s);
#endif
static int s_camera_setting_set_feature(camera_setting_t *s, unsigned int feature, const int val);
static int s_set_parameters(capture_t *cap, const int index, camera_setting_t *s);
static int s_activate_camera(capture_t *cap, const int index);
static void s_flush_buffer(capture_t *cap, const int index);

static void s_copy_frame(dc1394video_frame_t *vframe, capture_frame_t *frame);
static dc1394video_mode_t s_get_appropriate_mode(dc1394camera_t *cam, dc1394video_mode_t mode);
static dc1394framerate_t s_get_appropriate_framerate(dc1394camera_t *cam, dc1394video_mode_t mode, dc1394framerate_t framerate);

//capture_t *capture_new();  /* memory allocation   */

int capture_init(capture_t *cap, const int verbose_mode)
{
  dc1394error_t err;

  /* initialize cap */
  cap->num_cameras = 0;
  cap->dc1394_cxt  = NULL;
  cap->camera_list = NULL;
  cap->num_active  = 0;
  cap->cameras     = NULL;

  cap->dc1394_cxt = dc1394_new();
  if (cap->dc1394_cxt == NULL) {
    return CAPTURE_ERROR;
  }

  err = dc1394_camera_enumerate(cap->dc1394_cxt, &cap->camera_list);
  DC1394_WRN(err, "Failed to enumerate cameras");
  if (err != DC1394_SUCCESS) {
    return CAPTURE_ERROR;
  }

  if (verbose_mode) {
    fprintf(stderr, "%d camers(s) found.\n", cap->camera_list->num);
  }

  cap->num_cameras = cap->camera_list->num;

  cap->num_buffers = 4;
  cap->prefer_one_shot = 0;
  cap->prefer_bmode = 0;
  cap->verbose = verbose_mode;

  return CAPTURE_SUCCESS;
}

int capture_final(capture_t *cap)
{
  int i;

  for (i = 0; i < cap->num_active; ++i) {
    dc1394_video_set_transmission(cap->cameras[i], DC1394_OFF);
    dc1394_capture_stop(cap->cameras[i]);
    dc1394_camera_free(cap->cameras[i]);
  }

  cap->num_active = 0;
  free(cap->cameras);  cap->cameras = NULL;

  if (cap->camera_list != NULL) {
    dc1394_camera_free_list(cap->camera_list);
    cap->camera_list = NULL;
  }
  cap->num_cameras = 0;

  if (cap->dc1394_cxt != NULL) {
    dc1394_free(cap->dc1394_cxt);
    cap->dc1394_cxt = NULL;
  }

  return CAPTURE_SUCCESS;
}

//int capture_delete(capture_t **cap); /* memory deallocation */

int capture_setup(capture_t *cap, const char *conf_file)
{
  conf_param_t param = CONF_PARAM_INIT_VAL;
  int i, num_found;

  /* read configulation from a specified file */
  if (s_read_conf_file(conf_file, &param) < 1) {
    return CAPTURE_ERROR;
  }


  /* allocate memory */
  cap->cameras = (dc1394camera_t **)malloc(sizeof(dc1394camera_t *) * param.num_cameras);
  if (cap->cameras == NULL) {
    return CAPTURE_ERROR;
  }

  cap->num_active = 0;
  num_found = 0;
  for (i = 0; i < param.num_cameras; ++i) {
    dc1394error_t err;

    cap->cameras[num_found] = dc1394_camera_new(cap->dc1394_cxt, param.settings[i].uid);

    if (cap->cameras[num_found] == NULL) {
      if (cap->verbose) {
        fprintf(stderr, "camera 0x%0"PRIx64" not found.\n", param.settings[i].uid);
      }
      continue;
    }

    if (cap->verbose) {
      fprintf(stderr, "GUID[%d]: 0x%0"PRIx64"\n", i, cap->cameras[num_found]->guid);
    }

#if 0
    {
      uint32_t port = 42;
      dc1394_camera_get_linux_port(cap->cameras[num_found], &port);
      printf("port: %d\n", port);
    }
#endif

    /* set parameters */
    if (cap->verbose) {
      fprintf(stderr, "set parameter.\n");
    }
    s_set_parameters(cap, num_found, &param.settings[i]);

    /* set up a capture construct */
    if (cap->verbose) {
      fprintf(stderr, "set up a capture construct.\n");
    }

    err = dc1394_capture_setup(cap->cameras[num_found], cap->num_buffers, DC1394_CAPTURE_FLAGS_DEFAULT);
    DC1394_WRN(err, "Setup capture failed.");
    if (err != DC1394_SUCCESS) {
      continue;
    }
    
    /* set ready to capture */
    if (cap->verbose) {
      fprintf(stderr, "activate camera.\n");
    }
    s_activate_camera(cap, num_found);

    ++num_found;
  }
  cap->num_active = num_found;

  s_clear_conf_param(&param);

  return CAPTURE_SUCCESS;
}

int capture_single(capture_t *cap, const int index, capture_frame_t *frame)
{
  dc1394camera_t *cam = cap->cameras[index];
  dc1394error_t err;
  dc1394video_frame_t *vframe = NULL;
  uint32_t frames_behind = 0;

  if (cap->prefer_one_shot && cam->one_shot_capable == DC1394_TRUE) {
    err = dc1394_video_set_one_shot(cam, DC1394_ON);
    DC1394_WRN(err, "could not set one shot mode.");
    if (err != DC1394_SUCCESS) {
      return CAPTURE_ERROR;
    }
  }

  err = dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_WAIT, &vframe);
  DC1394_WRN(err, "could not dequeue frame.");
  if (err != DC1394_SUCCESS) {
    return CAPTURE_ERROR;
  }
  frames_behind = vframe->frames_behind;

  /* copy the image to frame->raw_data */
  s_copy_frame(vframe, frame);

  err = dc1394_capture_enqueue(cam, vframe);
  DC1394_WRN(err, "could not enqueue frame.");
  if (err != DC1394_SUCCESS) {
    return CAPTURE_ERROR;
  }

  /* drop behind frames if drop_frame is enable */
  if (cap->drop_frames) {
    while (frames_behind-- > 0) {
      err = dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_WAIT, &vframe);
      DC1394_WRN(err, "could not dequeue frame.");
      if (err != DC1394_SUCCESS) {
        return CAPTURE_ERROR;
      }
      
      err = dc1394_capture_enqueue(cam, vframe);
      DC1394_WRN(err, "could not enqueue frame.");
      if (err != DC1394_SUCCESS) {
        return CAPTURE_ERROR;
      }
    }
  }

  return CAPTURE_SUCCESS;
}

int capture_multi(capture_t *cap, capture_frame_t *frames)
{
  int i;
  dc1394error_t err;
  dc1394video_frame_t **vframes = NULL;
  uint32_t frames_behind = 0;

  vframes = (dc1394video_frame_t **)malloc(sizeof(dc1394video_frame_t *) * cap->num_active);
  if (vframes == NULL) {
    return CAPTURE_ERROR;
  }

  for (i = 0; i < cap->num_active; ++i) {
    if (cap->prefer_one_shot && cap->cameras[i]->one_shot_capable) {
      err = dc1394_video_set_one_shot(cap->cameras[i], DC1394_ON);
      DC1394_WRN(err, "could not set one shot mode.");
      if (err != DC1394_SUCCESS) {
        return CAPTURE_ERROR;
      }
    }
  }

  for (i = 0; i < cap->num_active; ++i) {
    err = dc1394_capture_dequeue(cap->cameras[i], DC1394_CAPTURE_POLICY_WAIT, &vframes[i]);
    DC1394_WRN(err, "could not dequeue frame.");
    if (err != DC1394_SUCCESS) {
      free(vframes);
      return CAPTURE_ERROR;
    }
  }

  /* find the minimum number of behind frames */
  frames_behind = vframes[0]->frames_behind;
  for (i = 1; i < cap->num_active; ++i) {
    if (frames_behind > vframes[i]->frames_behind) {
      frames_behind = vframes[i]->frames_behind;
    }
  }

  /* copy the image to frame->raw_data */
  for (i = 0; i < cap->num_active; ++i) {
    s_copy_frame(vframes[i], &frames[i]);
  }

  for (i = 0; i < cap->num_active; ++i) {
    err = dc1394_capture_enqueue(cap->cameras[i], vframes[i]);
    DC1394_WRN(err, "could not enqueue frame.");
    if (err != DC1394_SUCCESS) {
      free(vframes);
      return CAPTURE_ERROR;
    }
  }

  /* drop behind frames if drop_frame is enable */
  if (cap->drop_frames != 0) {
    while (frames_behind-- > 0) {
      for (i = 0; i < cap->num_active; ++i) {
        err = dc1394_capture_dequeue(cap->cameras[i], DC1394_CAPTURE_POLICY_WAIT, &vframes[i]);
        DC1394_WRN(err, "could not dequeue frame.");
        if (err != DC1394_SUCCESS) {
          free(vframes);
          return CAPTURE_ERROR;
        }
      }
      
      for (i = 0; i < cap->num_active; ++i) {
        err = dc1394_capture_enqueue(cap->cameras[i], vframes[i]);
        DC1394_WRN(err, "could not enqueue frame.");    
        if (err != DC1394_SUCCESS) {
          free(vframes);
          return CAPTURE_ERROR;
        }
      }
    }
  }

  free(vframes);

  return CAPTURE_SUCCESS;
}

int capture_create_frames(capture_t *cap, capture_frame_t **frames)
{
  *frames = (capture_frame_t *)malloc(sizeof(capture_frame_t) * cap->num_active);
  if (*frames == NULL) {
    return CAPTURE_ERROR;
  }

  fprintf(stderr, "init frames...");
  if (capture_frames_init(cap, *frames) != cap->num_active) {
    free(*frames);
    *frames = NULL;
    return CAPTURE_ERROR;
  }
  fprintf(stderr, "done.\n");

  return CAPTURE_SUCCESS;
}

void capture_destroy_frames(capture_t *cap, capture_frame_t **frames)
{
  if (*frames == NULL) {
    return;
  }

  capture_frames_final(cap, *frames);
  free(*frames);
  *frames = NULL;
}

int capture_frames_init(capture_t *cap, capture_frame_t *frames)
{
  int i;

  for (i = 0; i < cap->num_active; ++i) {
    int width, height;
    capture_frame_format_t format;

    unsigned int mode;

    frames[i].width = 0;
    frames[i].height = 0;
    frames[i].format = CAPTURE_FRAME_FORMAT_UNKNOWN;
    frames[i].timestamp = 0;
    frames[i].raw_data = NULL;

    //fprintf(stderr, "%d-th frame initializing...", i);

    /* query parameters */
    if (dc1394_video_get_mode(cap->cameras[i], (dc1394video_mode_t *)&mode) != DC1394_SUCCESS) {
      //fprintf(stderr, "dc1394_get_video_mode failed.\n");
      goto on_error;
    }

    if (s_get_capture_size(mode, &width, &height) != CAPTURE_SUCCESS) {
      //fprintf(stderr, "dc1394_get_capture_size failed.\n");
      goto on_error;
    }

    if (s_get_capture_format(mode, &format) != CAPTURE_SUCCESS) {
      //fprintf(stderr, "dc1394_get_capture_format failed.\n");
      goto on_error;
    }

    /* allocate memory */
    switch (format) {
    case CAPTURE_FRAME_FORMAT_GRAY:
      frames[i].raw_data = malloc(sizeof(unsigned char) * width * height);
      break;

    case CAPTURE_FRAME_FORMAT_RGB:
      frames[i].raw_data = malloc(sizeof(unsigned char) * width * height * 3);
      break;

    default:
      ;
    }
    if (frames[i].raw_data == NULL) {
      //fprintf(stderr, "raw_data is NULL.\n");
      goto on_error;
    }

    frames[i].width  = width;
    frames[i].height = height;
    frames[i].format = format;

    //fprintf(stderr, "done.\n");
  }

  //fprintf(stderr, "i = %d\n", i);
  return i;

 on_error:
  while (--i >= 0) {
    free(frames[i].raw_data);
  }
  return -1;
}

void capture_frames_final(capture_t *cap, capture_frame_t *frames)
{
  int i;

  for (i = 0; i < cap->num_active; ++i) {
    frames[i].width  = 0;
    frames[i].height = 0;
    frames[i].format = CAPTURE_FRAME_FORMAT_UNKNOWN;
    frames[i].timestamp = 0;

    free(frames[i].raw_data);
    frames[i].raw_data = NULL;
  }
}

capture_frame_t *capture_frame_new(const int width, const int height, const capture_frame_format_t format)
{
  capture_frame_t *cf = NULL;

  /* allocate memory */
  cf = (capture_frame_t *)malloc(sizeof(capture_frame_t));
  if (cf == NULL) {
    return NULL;
  }

  cf->raw_data = NULL;
  switch (format) {
  case CAPTURE_FRAME_FORMAT_GRAY:
    cf->raw_data = malloc(sizeof(unsigned char) * width * height);
    break;

  case CAPTURE_FRAME_FORMAT_RGB:
    cf->raw_data = malloc(sizeof(unsigned char) * width * height * 3);
    break;

  default:
    ;
  }
  if (cf->raw_data == NULL) {
    free(cf);
    return NULL;
  }

  cf->width  = width;
  cf->height = height;
  cf->format = format;

  return cf;
}

void capture_frame_delete(capture_frame_t **cf)
{
  free((*cf)->raw_data);
  free(*cf);
  *cf = NULL;
}


/*
  static functions
*/

static inline int s_byte_per_packet(const capture_frame_format_t format)
{
  switch (format) {
  case CAPTURE_FRAME_FORMAT_GRAY:
    return 1;
    break;

  case CAPTURE_FRAME_FORMAT_RGB:
    return 3;
    break;

  case CAPTURE_FRAME_FORMAT_UNKNOWN:
  default:
    break;
  }

  return 0;
}

static char s_ret_n(FILE *fp)
{
  char ch = getc(fp);
  
  if (ch != '\n') {
    ungetc(ch, fp);
  }

  return '\n';
}

static char s_get_str(FILE *fp, char *buf, const int buf_size)
{
  char ch;
  
  /* skip leading spaces */
  do {
    ch = getc(fp);
  } while (isblank(ch));

  /* copy consecutive chars to buf */
  int i = 0;
  while (i < buf_size-1 && isgraph(ch)) {
    buf[i] = ch;
    ++i;

    ch = getc(fp);
  }
  buf[i] = '\0';

  /* return '\n' if it reaches the end of line */
  if (ch == '\r') {
    return s_ret_n(fp);
  }

  return ch;
}

static int s_check_token(const char *buf, const int buf_size, const conf_token_t *token_list)
{
  int i = 0;

  while (token_list[i].token[0] != '\0') {
    if (strncmp(token_list[i].token, buf, buf_size) == 0) {
      //fprintf(stderr, "token: %s\n", token_list[i].token);
      
      break;
    }
    ++i;
  }

  if (token_list[i].token[0] == '\0') {
    return -1; /* not found */
  }

  return i;
}

static int s_read_conf_file(const char *conf_file, conf_param_t *data)
{
  int num_confs;
  FILE *fp;
  char ch, buf[1024];

  assert(data->settings == NULL);
  data->settings = NULL;

  fp = fopen(conf_file, "r");
  if (fp == NULL) {
    goto on_error;
  }

  /* delay */
  if ((ch = s_get_str(fp, buf, sizeof(buf))) != EOF && isdigit(buf[0])) {
    data->delay = atoi(buf);
  }
  else {
    goto on_error;
  }

  /* # cameras */
  if ((ch = s_get_str(fp, buf, sizeof(buf))) != EOF && isdigit(buf[0])) {
    data->num_cameras = atoi(buf);
    //fprintf(stderr, "# cameras: %d\n", data->num_cameras);
  }
  else {
    goto on_error;
  }


  /* allocate memory for storing settings of cameras */
  data->settings = (camera_setting_t *)malloc(sizeof(camera_setting_t) * data->num_cameras);
  if (data->settings == NULL) {
    goto on_error;
  }

  /* read each setting */
  for (num_confs = 0; num_confs < data->num_cameras; ++num_confs) {
    /* initialize parameter */
    s_camera_setting_init(&data->settings[num_confs]);


    /* GUID */
    ch = s_get_str(fp, buf, sizeof(buf));
    if (strncmp(buf, "0x", 2) == 0) {
      sscanf(buf, "%"SCNx64, &data->settings[num_confs].uid);
      //fprintf(stderr, "# guid: 0x%0"PRIx64"\n", data->settings[i].uid);
    }
    else {
      break;
    }


    /* configulations */
    do {
      ch = s_get_str(fp, buf, sizeof(buf));

      if (buf[0] != '\0') {
        int found = 0;

#if 1
        /* mode */
        if (found == 0) {
          int index;
          index = s_check_token(buf, sizeof(buf), conf_mode);
          if (index >= 0) {
            found = -1;

            data->settings[num_confs].mode = (dc1394video_mode_t)conf_mode[index].key;
          }
        }
#endif

#if 1
        /* frame rate */
        if (found == 0) {
          int index;
          index = s_check_token(buf, sizeof(buf), conf_framerate);
          if (index >= 0) {
            found = -1;

            data->settings[num_confs].framerate = (dc1394framerate_t)conf_framerate[index].key;
          }
        }
#endif

#if 1
        /* feature */
        if (found == 0) {
          int index;
          index = s_check_token(buf, sizeof(buf), conf_feature);
          if (index >= 0) {
              if ((ch = s_get_str(fp, buf, sizeof(buf))) == EOF ||
                  !(isdigit(buf[0]) || buf[0] == '-')) {
                goto on_error;
              }
              //fprintf(stderr, "val: %s\n", buf);

              if (conf_feature[index].key != DC1394_FEATURE_WHITE_BALANCE) {
                found = -1;

                s_camera_setting_set_feature(&data->settings[num_confs], conf_feature[index].key, atoi(buf));
              }
              else {
                int ub = atoi(buf);

                if ((ch = s_get_str(fp, buf, sizeof(buf))) == EOF ||
                    !(isdigit(buf[0]) || buf[0] == '-')) {
                  goto on_error;
                }
                //fprintf(stderr, "val: %s\n", buf);

                found = -1;

                data->settings[num_confs].white_balance[0] = ub;
                data->settings[num_confs].white_balance[1] = atoi(buf);                
              }
          }
        }
#endif
        
        /* too long string.. */
        if (isgraph(ch)) {
          do {
            ch = getc(fp);
          } while (isgraph(ch) && ch != EOF);
        }
      }
      else {
        break;
      }
    } while (ch != '\n' && ch != EOF);

#ifdef DEBUG_LIBCAPTURE
    s_camera_setting_disp(&data->settings[num_confs]);
    printf("\n");
#endif
  }

  if (data->num_cameras != num_confs) {
    goto on_error;
  }

  return num_confs;


 on_error:
  free(data->settings);

  data->delay = 0;
  data->num_cameras = 0;
  data->settings = NULL;
    
  return -1;
}

static int s_clear_conf_param(conf_param_t *data)
{
  data->delay = 0;
  data->num_cameras = 0;

  free(data->settings);
  data->settings = NULL;

  return CAPTURE_SUCCESS;
}


static int s_get_capture_size(unsigned int mode, int *width, int *height)
{
  switch (mode) {
  case DC1394_VIDEO_MODE_160x120_YUV444:
    *width  = 160;
    *height = 120;
    break;

  case DC1394_VIDEO_MODE_320x240_YUV422:
    *width  = 320;
    *height = 240;
    break;

  case DC1394_VIDEO_MODE_640x480_YUV411:
  case DC1394_VIDEO_MODE_640x480_YUV422:
  case DC1394_VIDEO_MODE_640x480_RGB8:
  case DC1394_VIDEO_MODE_640x480_MONO8:
  case DC1394_VIDEO_MODE_640x480_MONO16:
    *width  = 640;
    *height = 480;
    break;

  case DC1394_VIDEO_MODE_800x600_YUV422:
  case DC1394_VIDEO_MODE_800x600_RGB8:
  case DC1394_VIDEO_MODE_800x600_MONO8:
  case DC1394_VIDEO_MODE_800x600_MONO16:
    *width  = 800;
    *height = 600;
    break;

  case DC1394_VIDEO_MODE_1024x768_YUV422:
  case DC1394_VIDEO_MODE_1024x768_RGB8:
  case DC1394_VIDEO_MODE_1024x768_MONO8:
  case DC1394_VIDEO_MODE_1024x768_MONO16:
    *width  = 1024;
    *height =  768;
    break;

  case DC1394_VIDEO_MODE_1280x960_YUV422:
  case DC1394_VIDEO_MODE_1280x960_RGB8:
  case DC1394_VIDEO_MODE_1280x960_MONO8:
  case DC1394_VIDEO_MODE_1280x960_MONO16:
  case DC1394_VIDEO_MODE_1600x1200_MONO16:
    *width  = 1280;
    *height =  960;
    break;

  case DC1394_VIDEO_MODE_1600x1200_YUV422:
  case DC1394_VIDEO_MODE_1600x1200_RGB8:
  case DC1394_VIDEO_MODE_1600x1200_MONO8:
    *width  = 1600;
    *height = 1200;
    break;

  default:
    *width  = -1;
    *height = -1;
    return CAPTURE_ERROR;
  }

  return CAPTURE_SUCCESS;
}

static int s_get_capture_format(unsigned int mode, capture_frame_format_t *format)
{
  switch (mode) {
  case DC1394_VIDEO_MODE_160x120_YUV444:
    *format = CAPTURE_FRAME_FORMAT_RGB;
    break;

  case DC1394_VIDEO_MODE_320x240_YUV422:
  case DC1394_VIDEO_MODE_640x480_YUV422:
  case DC1394_VIDEO_MODE_800x600_YUV422:
  case DC1394_VIDEO_MODE_1024x768_YUV422:
  case DC1394_VIDEO_MODE_1280x960_YUV422:
  case DC1394_VIDEO_MODE_1600x1200_YUV422:
    *format = CAPTURE_FRAME_FORMAT_RGB;
    break;

  case DC1394_VIDEO_MODE_640x480_YUV411:
    *format = CAPTURE_FRAME_FORMAT_RGB;
    break;

  case DC1394_VIDEO_MODE_640x480_RGB8:
  case DC1394_VIDEO_MODE_800x600_RGB8:
  case DC1394_VIDEO_MODE_1024x768_RGB8:
  case DC1394_VIDEO_MODE_1280x960_RGB8:
  case DC1394_VIDEO_MODE_1600x1200_RGB8:
    *format = CAPTURE_FRAME_FORMAT_RGB;
    break;

  case DC1394_VIDEO_MODE_640x480_MONO8:
  case DC1394_VIDEO_MODE_800x600_MONO8:
  case DC1394_VIDEO_MODE_1024x768_MONO8:
  case DC1394_VIDEO_MODE_1280x960_MONO8:
  case DC1394_VIDEO_MODE_1600x1200_MONO8:
    *format = CAPTURE_FRAME_FORMAT_GRAY;
    break;

  case DC1394_VIDEO_MODE_640x480_MONO16:
  case DC1394_VIDEO_MODE_800x600_MONO16:
  case DC1394_VIDEO_MODE_1024x768_MONO16:
  case DC1394_VIDEO_MODE_1280x960_MONO16:
  case DC1394_VIDEO_MODE_1600x1200_MONO16:
    *format = CAPTURE_FRAME_FORMAT_GRAY;
    break;

  default:
    *format = CAPTURE_FRAME_FORMAT_UNKNOWN;
    return CAPTURE_ERROR;
  }

  return CAPTURE_SUCCESS;
}

#if 0
static int s_get_capture_framerate(unsigned int framerate, double *fps)
{
  switch (framerate) {
  case DC1394_FRAMERATE_1_875:
    *fps = 1.875;
    break;

  case DC1394_FRAMERATE_3_75:
    *fps = 3.75;
    break;

  case DC1394_FRAMERATE_7_5:
    *fps = 7.5;
    break;

  case DC1394_FRAMERATE_15:
    *fps = 15.0;
    break;

  case DC1394_FRAMERATE_30:
    *fps = 30.0;
    break;

  case DC1394_FRAMERATE_60:
    *fps = 60.0;
    break;

  case DC1394_FRAMERATE_120:
    *fps = 120.0;
    break;

  case DC1394_FRAMERATE_240:
    *fps = 240.0;
    break;

  default:
    *fps = 0;
    return CAPTURE_ERROR;
  }

  return CAPTURE_SUCCESS;
}
#endif

static int s_camera_setting_init(camera_setting_t *s)
{
  s->uid = 0;

  s->mode      = (dc1394video_mode_t)-1;
  s->framerate = (dc1394framerate_t)-1;

  /* features */
  s->brightness       = -1;
  s->exposure         = -1;
  s->sharpness        = -1;
  s->white_balance[0] = -1;
  s->white_balance[1] = -1;
  s->hue              = -1;
  s->saturation       = -1;
  s->gamma            = -1;
  s->shutter          = -1;
  s->gain             = -1;
  s->iris             = -1;
  s->focus            = -1;
  s->temperature      = -1;

  s->zoom = -1;
  s->pan  = -1;
  s->tilt = -1;

  return CAPTURE_SUCCESS;
}

#ifdef DEBUG_LIBCAPTURE
static void s_camera_setting_disp(camera_setting_t *s)
{
  printf("GUID         : 0x%0"PRIx64"\n", s->uid);

  printf("\n");

  printf("mode         : %d\n", s->mode);
  printf("frame rate   : %d\n", s->framerate);

  printf("\n");

  /* features */
  printf("brightness   : %d\n", s->brightness);
  printf("exprosure    : %d\n", s->exposure);
  printf("sharpness    : %d\n", s->sharpness);
  printf("white balance: %d %d\n", s->white_balance[0], s->white_balance[1]);
  printf("hue          : %d\n", s->hue);
  printf("saturation   : %d\n", s->saturation);
  printf("gamma        : %d\n", s->gamma);
  printf("shutter      : %d\n", s->shutter);
  printf("gain         : %d\n", s->gain);
  printf("iris         : %d\n", s->iris);
  printf("focus        : %d\n", s->focus);
  printf("temperature  : %d\n", s->temperature);

  printf("zoom         : %d\n", s->zoom);
  printf("pan          : %d\n", s->pan);
  printf("tilt         : %d\n", s->tilt);

  printf("\n");
}
#endif /* DEBUG_LIBCAPTURE */

static int s_camera_setting_set_feature(camera_setting_t *s, unsigned int feature, const int val)
{
  switch (feature) {
  case DC1394_FEATURE_BRIGHTNESS:
    s->brightness = val;
    break;

  case DC1394_FEATURE_EXPOSURE:
    s->exposure = val;
    break;

  case DC1394_FEATURE_SHARPNESS:
    s->sharpness = val;
    break;

  case DC1394_FEATURE_HUE:
    s->hue = val;
    break;

  case DC1394_FEATURE_SATURATION:
    s->saturation = val;
    break;

  case DC1394_FEATURE_GAMMA:
    s->gamma = val;
    break;

  case DC1394_FEATURE_SHUTTER:
    s->shutter = val;
    break;

  case DC1394_FEATURE_GAIN:
    s->gain = val;
    break;

  case DC1394_FEATURE_IRIS:
    s->iris = val;
    break;

  case DC1394_FEATURE_FOCUS:
    s->focus = val;
    break;

  case DC1394_FEATURE_TEMPERATURE:
    s->temperature = val;
    break;

  case DC1394_FEATURE_TRIGGER:    
  case DC1394_FEATURE_TRIGGER_DELAY:  
  case DC1394_FEATURE_WHITE_SHADING:
  case DC1394_FEATURE_FRAME_RATE:
    return CAPTURE_ERROR;

    /* 16 reserved features */

  case DC1394_FEATURE_ZOOM:
    s->zoom = val;
    break;

  case DC1394_FEATURE_PAN:
    s->pan = val;
    break;

  case DC1394_FEATURE_TILT:
    s->tilt = val;
    break;

  case DC1394_FEATURE_OPTICAL_FILTER:
    /* 12 reserved features */
  case DC1394_FEATURE_CAPTURE_SIZE:
  case DC1394_FEATURE_CAPTURE_QUALITY:
    /* 14 reserved features */
    return CAPTURE_ERROR;

  case DC1394_FEATURE_WHITE_BALANCE: /* require two parameters */
  default:
    return CAPTURE_ERROR;
  }

  return CAPTURE_SUCCESS;
}

#define SET_FEATURE(feature, val) \
  do { \
    if ((val) >= 0) { \
      dc1394_feature_set_mode(cam, (feature), DC1394_FEATURE_MODE_MANUAL); \
      if (dc1394_feature_set_value(cam, (feature), (val)) != DC1394_SUCCESS) { \
        ret = CAPTURE_ERROR; \
      } \
    } \
    else { \
      if (dc1394_feature_set_mode(cam, (feature), DC1394_FEATURE_MODE_AUTO) != DC1394_SUCCESS) { \
        ret = CAPTURE_ERROR; \
      } \
    } \
  } while (0)

static int s_set_parameters(capture_t *cap, const int index, camera_setting_t *s)
{
  int ret = CAPTURE_SUCCESS;

  dc1394camera_t *cam = cap->cameras[index];
  dc1394error_t err;

  /* set operation mode to 1394b, if possible */
  if (cam->bmode_capable != 0 && cap->prefer_bmode) {
    err = dc1394_video_set_operation_mode(cam, DC1394_OPERATION_MODE_1394B);
    DC1394_WRN(err, "could not set 1394b mode.");

    if (err == DC1394_SUCCESS) {
      err = dc1394_video_set_iso_speed(cam, DC1394_ISO_SPEED_800);
      DC1394_WRN(err, "could not set iso mode to S800.");

      if (err != DC1394_SUCCESS) {
        err = dc1394_video_set_iso_speed(cam, DC1394_ISO_SPEED_400);
        DC1394_WRN(err, "could not set iso mode to S400.");
      }
    }
  }
  else {
    err = dc1394_video_set_operation_mode(cam, DC1394_OPERATION_MODE_LEGACY);
    DC1394_WRN(err, "could not set 1394-legacy mode.");

    err = dc1394_video_set_iso_speed(cam, DC1394_ISO_SPEED_400);
    DC1394_WRN(err, "could not set iso mode to S400.");
  }

  /* mode */
  s->mode = s_get_appropriate_mode(cam, ((s->mode >= 0) ? s->mode : DC1394_VIDEO_MODE_640x480_MONO8));
  if (dc1394_video_set_mode(cam, s->mode) != DC1394_SUCCESS) {
    ret = CAPTURE_ERROR;
  }
  dc1394_video_get_mode(cam, &s->mode);

  /* frame rate */
  s->framerate = s_get_appropriate_framerate(cam, s->mode, ((s->framerate >= 0) ? s->framerate : DC1394_FRAMERATE_30));
  if (dc1394_video_set_framerate(cam, s->framerate) != DC1394_SUCCESS) {
    ret = CAPTURE_ERROR;
  }

  /*
    features
  */
  SET_FEATURE(DC1394_FEATURE_BRIGHTNESS,    s->brightness);
  SET_FEATURE(DC1394_FEATURE_EXPOSURE,      s->exposure);
  SET_FEATURE(DC1394_FEATURE_SHARPNESS,     s->sharpness);
  SET_FEATURE(DC1394_FEATURE_HUE,           s->hue);
  SET_FEATURE(DC1394_FEATURE_SATURATION,    s->saturation);
  SET_FEATURE(DC1394_FEATURE_GAMMA,         s->gamma);
  SET_FEATURE(DC1394_FEATURE_SHUTTER,       s->shutter);
  SET_FEATURE(DC1394_FEATURE_GAIN,          s->gain);
  SET_FEATURE(DC1394_FEATURE_IRIS,          s->iris);
  SET_FEATURE(DC1394_FEATURE_FOCUS,         s->focus);
  //SET_FEATURE(DC1394_FEATURE_TEMPERATURE,   s->temperature); is not supported by dc1394_feature_set_value()
  
  SET_FEATURE(DC1394_FEATURE_ZOOM, s->zoom);
  SET_FEATURE(DC1394_FEATURE_PAN,  s->pan);
  SET_FEATURE(DC1394_FEATURE_TILT, s->tilt);

  /* white blance */
  if (s->white_balance[0] >= 0 && s->white_balance[1] >= 0) {
    dc1394_feature_set_mode(cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_MANUAL);
    if (dc1394_feature_whitebalance_set_value(cam, s->white_balance[0], s->white_balance[1]) != DC1394_SUCCESS) {
      ret = CAPTURE_ERROR;
    }
  }
  else {
    if (dc1394_feature_set_mode(cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_AUTO) != DC1394_SUCCESS) {
      ret = CAPTURE_ERROR;
    }
  }

  return ret;
}

static int s_activate_camera(capture_t *cap, const int index)
{
  s_flush_buffer(cap, index);

  if ((!cap->prefer_one_shot) || cap->cameras[index]->one_shot_capable != DC1394_TRUE) {
    dc1394error_t err;

    /* use continuous transmission mode */
    err = dc1394_video_set_transmission(cap->cameras[index], DC1394_ON);
    DC1394_WRN(err, "could not start iso transmission");
    if (err != DC1394_SUCCESS) {
      return CAPTURE_ERROR;
    }
  }

  return CAPTURE_SUCCESS;
}

static void s_flush_buffer(capture_t *cap, const int index)
{
  dc1394camera_t *cam = cap->cameras[index];

  dc1394switch_t pwr = DC1394_OFF;
  dc1394video_frame_t *frame = NULL;

  dc1394_video_get_transmission(cam, &pwr);
  if (pwr == DC1394_ON) {
    dc1394_video_set_transmission(cam, DC1394_OFF);
  }

  while (dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_POLL, &frame), frame != NULL) {
    dc1394error_t err;

    err = dc1394_capture_enqueue(cam, frame);
    DC1394_ERR(err, "could not enqueue");
  }
}

static void s_copy_frame(dc1394video_frame_t *vframe, capture_frame_t *frame)
{
  dc1394error_t err;

  switch (frame->format) {
  case CAPTURE_FRAME_FORMAT_GRAY:
    err = dc1394_convert_to_MONO8(vframe->image, (uint8_t *)frame->raw_data, vframe->size[0], vframe->size[1], vframe->yuv_byte_order, vframe->color_coding, vframe->data_depth);
    DC1394_ERR(err, "conversion failed");
    break;

  case CAPTURE_FRAME_FORMAT_RGB:
    err = dc1394_convert_to_RGB8(vframe->image, (uint8_t *)frame->raw_data, vframe->size[0], vframe->size[1], vframe->yuv_byte_order, vframe->color_coding, vframe->data_depth);
    DC1394_ERR(err, "conversion failed");

  default:
    ;
  }

  frame->timestamp = vframe->timestamp;
}

static dc1394video_mode_t s_get_appropriate_mode(dc1394camera_t *cam, dc1394video_mode_t mode)
{
  dc1394video_modes_t supported_modes;
  dc1394video_mode_t selected_mode = mode;

  uint32_t width, height;
  dc1394color_coding_t coding;
  int found = 0;
  unsigned int num_pixels = 0;

  dc1394error_t err;
  unsigned int i;

  err = dc1394_video_get_supported_modes(cam, &supported_modes);
  DC1394_WRN(err, "failed to get supported modes.");
  if (err != DC1394_SUCCESS) {
    return mode;
  }

  for (i = 0; i < supported_modes.num; ++i) {
    /* return given mode if the camera supports it */
    if (supported_modes.modes[i] == mode) {
      return mode;
    }
  }

  /*
    reaches here when no mode is exactly matched with one of supported modes
  */

  err = dc1394_get_image_size_from_video_mode(cam, mode, &width, &height);
  DC1394_WRN(err, "failed to get image size.");
  if (err != DC1394_SUCCESS) {
    return mode;
  }
  num_pixels = width * height;

  err = dc1394_get_color_coding_from_video_mode(cam, mode, &coding);
  DC1394_WRN(err, "failed to get color coding.");
  if (err != DC1394_SUCCESS) {
    return mode;
  }

  found = 0;
  /* search the mode of size does not exceed the given mode and has the same color coding */
  for (i = 0; i < supported_modes.num; ++i) {
    dc1394color_coding_t c;
    if (dc1394_get_color_coding_from_video_mode(cam, supported_modes.modes[i], &c) != DC1394_SUCCESS) {
      continue;
    }

    if (c == coding) {
      uint32_t w, h;

      if (dc1394_get_image_size_from_video_mode(cam, supported_modes.modes[i], &w, &h) != DC1394_SUCCESS) {
        continue;
      }

      if (w*h <= width*height && num_pixels < w*h) {
        num_pixels = w*h;
        selected_mode = supported_modes.modes[i];
        found = 1;
      }
    }
  }

  if (found == 0) {
    return mode;
  }

  return selected_mode;
}

static dc1394framerate_t s_get_appropriate_framerate(dc1394camera_t *cam, dc1394video_mode_t mode, dc1394framerate_t framerate)
{
  dc1394framerates_t framerates;
  dc1394framerate_t selected_framerate;
  dc1394error_t err;
  unsigned int i;

  err = dc1394_video_get_supported_framerates(cam, mode, &framerates);
  DC1394_WRN(err, "failed to get framerates");
  if (err != DC1394_SUCCESS) {
    return framerate;
  }

  selected_framerate = (dc1394framerate_t)0;
  for (i = 0; i < framerates.num; ++i) {
    if (selected_framerate < framerates.framerates[i] && framerates.framerates[i] <= framerate) {
      selected_framerate = framerates.framerates[i];
    }
  }

  if (selected_framerate == 0) {
    return framerate;
  }

  return selected_framerate;
}
