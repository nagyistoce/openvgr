/*
 measure3d_scene.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#ifndef MEASURE3D_SCENE_H
#define MEASURE3D_SCENE_H

#include "Img.hh"
#include "Vision.hh"

#define NIMAGE  (2)
#define NDCOEF  (5)

#define ERROR_SIZE_MISMATCH     (-1)
#define ERROR_IMAGE_FORMAT      (-2)
#define ERROR_CREATE_IMAGE      (-3)
#define ERROR_CREATE_MAT        (-4)
#define ERROR_IMAGE_SEQ_LEN     (-5)
#define ERROR_FORMAT_MISMATCH   (-6)
#define ERROR_OPEN_CONFIG_FILE  (-7)
#define ERROR_PARSE_CONFIG_FILE (-8)

#define NCHANNELS_GRAY  (1)
#define NCHANNELS_COLOR (3)
#define NCHANNELS_3d (3)

typedef struct
{
  CvMat *cmat[NIMAGE]; // (3x3)x2
  CvMat *dcoef[NIMAGE]; // (NDCOEFx1)x2
  CvMat *R;//3x3
  CvMat *Ri[NIMAGE];//3x3 x 2
  CvMat *T;//3x1
  CvMat *Pi[NIMAGE];//3x4 x 2
  CvMat *Q;//4x4

  // following values are set at onActivate()
  CvStereoBMState       state;
} Work3D;  

int measure3d_scene(struct Img::TimedMultiCameraImage *in_data,
                    struct Stereo3D *outdata,
                    Work3D *work);

int read_measure3d_config(char *fname,
                          CvStereoBMState *state);

#endif /* MEASURE3D_SCENE_H */
