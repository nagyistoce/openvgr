/*
 capture.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#ifndef _CAPTURE_H
#define _CAPTURE_H

#include <dc1394/dc1394.h>

#define CAPTURE_SUCCESS ( 0)
#define CAPTURE_ERROR   (-1)

typedef struct tag_capture
{
  int num_cameras;
  dc1394_t *dc1394_cxt;
  dc1394camera_list_t *camera_list;

  int num_active;
  dc1394camera_t **cameras;

  int num_buffers;

  int prefer_one_shot;
  int prefer_bmode;

  int verbose;
} capture_t;

const static capture_t CAPTURE_INIT_VAL = {0, NULL, NULL, 0, NULL, 4, 0, 0, 1};


typedef enum tag_capture_frame_format
{
  CAPTURE_FRAME_FORMAT_UNKNOWN,
  CAPTURE_FRAME_FORMAT_GRAY,
  CAPTURE_FRAME_FORMAT_RGB
} capture_frame_format_t;

typedef struct tag_capture_frame
{
  int width;
  int height;

  capture_frame_format_t format;

  uint64_t timestamp;
  void *raw_data;
} capture_frame_t;


#ifdef __cplusplus
extern "C" {
#endif

//capture_t *capture_new();  /* memory allocation   */
int capture_init(capture_t *cap, const int verbose_mode);    /* initialization      */
int capture_final(capture_t *cap);   /* finalization        */
//int capture_delete(capture_t **cap); /* memory deallocation */

int capture_setup(capture_t *cap, const char *conf_file);
int capture_single(capture_t *cap, const int index, capture_frame_t *frame);
int capture_multi(capture_t *cap, capture_frame_t *frames);

int capture_create_frames(capture_t *cap, capture_frame_t **frames);
void capture_destroy_frames(capture_t *cap, capture_frame_t **frames);

int capture_frames_init(capture_t *cap, capture_frame_t *frames);
void capture_frames_final(capture_t *cap, capture_frame_t *frames);

capture_frame_t *capture_frame_new(const int width, const int height, const capture_frame_format_t format);
void capture_frame_delete(capture_frame_t **cf);

#ifdef __cplusplus
}
#endif

#endif /* _CAPTURE_H */
