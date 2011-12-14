/*
 measure3d_scene.c

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <stdio.h>
#include <cv.h>
#include "Img.hh"
#include "Vision.hh"

#include "measure3d_scene.h"
#include <highgui.h>

static int
addressImage(IplImage *img,
             int      row,
             int      col)
{
  return img->widthStep * row + col;
}

// release local arrays
static void
release_arrays(IplImage **src,
               IplImage **dst,
               IplImage **incolor,
               IplImage **dstcolor,
               IplImage **disp_s,
               IplImage **disp_f,
               IplImage **_3dImage,
               CvMat    **matx,
               CvMat    **maty)
{
  int iimg;
  
  for (iimg = 0; iimg < NIMAGE; iimg++)
    {
      cvReleaseImage(&src[iimg]);
      cvReleaseImage(&dst[iimg]);
      cvReleaseImage(&incolor[iimg]);
      cvReleaseMat(&matx[iimg]);
      cvReleaseMat(&maty[iimg]);
    }

  cvReleaseImage(dstcolor);
  cvReleaseImage(disp_s);
  cvReleaseImage(disp_f);
  cvReleaseImage(_3dImage);

  return;
}

// set structure CvSize with width and height
static CvSize
cvsize(int width, int height)
{
  CvSize cvsize;
  cvsize.width = width;
  cvsize.height = height;
  return cvsize;
}

static void
mul_m33_v3(double M[3][3], double v[3], double Mv[3])
{
  int j;

  for (j = 0; j < 3; j++)
    {
      Mv[j] = M[j][0]*v[0]+M[j][1]*v[1]+M[j][2]*v[2];
    }

  return;
}

// calculate 3d from images by BM
// the number of img_in must be previously checked .

// sequence:
// 1 create IplImage structure
//  1-1 if the input images are color, they are converted to gray images, 
//      because the FindStereoCorrespondenceBM() function cannot
//      treat color images.
// 2 cvStereoRectify() <= cmat[12],dcoef[12] => R_[12],P_[12],Q_[12]
// 3 (2*)cvInitUndistortRectifyMap() <= cmat[i],dcoef[i],R[i]
//               => create ma11[i], map2[i] which is new 2d coordinates
// 4 (2*)cvRemap <= src[i],map1[i],map2[i]=> dst[i] (rectified image)
// 5 cvFindStereoCorrespondenceBM() <= dst[1],dst[2],param =>disp_s
// 5-1 copy disp_s to disp_f multiplied by 1/16
// 6 cvReprojectImageTo3D() <=disparity,Q1[1]=>3dimage

int
measure3d_scene(struct Img::TimedMultiCameraImage *in_data,
                struct Stereo3D *outdata,
                Work3D *work)
{
  struct Img::CameraImage *img_in;
  IplImage *src[NIMAGE];
  IplImage *dst[NIMAGE];
  IplImage *incolor[NIMAGE];
  IplImage *dstcolor;
  IplImage *disp_s;
  IplImage *disp_f;
  IplImage *_3dImage;
  CvSize size;
  int i, j, k;
  int iimg;
  int row, col;
  CvMat *mapx[NIMAGE], *mapy[NIMAGE];
  int count;
  CvScalar tmpv;

  /*fprintf (stderr, "MEMO:measure3d(): check\n");*/
  // check image sequence length
  if (in_data->data.image_seq.length () < NIMAGE)
    {
      return ERROR_IMAGE_SEQ_LEN; /*-5*/
    }

  outdata->mimg = *in_data;
  img_in = &(outdata->mimg.data.image_seq[0]);
  // check image size
  if (img_in[0].image.width != img_in[1].image.width ||
      img_in[0].image.height != img_in[1].image.height)
    {
      return ERROR_SIZE_MISMATCH ;/*-1*/
    }
  size = cvsize(img_in[0].image.width, img_in[0].image.height);

  // create image, mat
  src[0] = src[1] = dst[0] = dst[1] = NULL;
  mapx[0] = mapx[1] = mapy[0] = mapy[1] = NULL;
  disp_s = NULL;
  disp_f = NULL;
  _3dImage = NULL;
  incolor[0] = incolor[1] = dstcolor = NULL;

  // check image format
  if (img_in[0].image.format != img_in[1].image.format)
    {
      return ERROR_FORMAT_MISMATCH ;/*-6*/
    }

  /*fprintf(stderr, "MEMO:measure3d(): create images\n");*/
  if (img_in[0].image.format == Img::CF_RGB)
    {
      incolor[0] = cvCreateImage(size, IPL_DEPTH_8U, NCHANNELS_COLOR /*3*/);
      if (incolor[0] == NULL)
        {
          release_arrays(src, dst, incolor, &dstcolor,
                         &disp_s, &disp_f, &_3dImage, mapx, mapy);
          return ERROR_CREATE_IMAGE/*-3*/;
        }
      incolor[1] = cvCreateImage(size, IPL_DEPTH_8U, NCHANNELS_COLOR /*3*/);
      if (incolor[1] == NULL)
        {
          release_arrays(src, dst, incolor, &dstcolor,
                         &disp_s, &disp_f, &_3dImage, mapx, mapy);
          return ERROR_CREATE_IMAGE/*-3*/;
        }
      dstcolor = cvCreateImage(size, IPL_DEPTH_8U, NCHANNELS_COLOR /*3*/);
      if (dstcolor == NULL)
        {
          release_arrays(src, dst, incolor, &dstcolor,
                         &disp_s, &disp_f, &_3dImage, mapx, mapy);
          return ERROR_CREATE_IMAGE/*-3*/;
        }
    }
  
  /*fprintf(stderr, "MEMO:measure3d(): loop start\n");*/
  //cmat[0] = cmat[1] = NULL;
  //dcoef[0] = dcoef[1] = NULL;
  for (iimg = 0; iimg < NIMAGE; iimg++)
    {
      /*fprintf(stderr, "MEMO:measure3d(): loop [%d]\n", iimg);
	fprintf(stderr, "MEMO:measure3d(): set src,dst\n");*/
      // IplImage
      switch (img_in[iimg].image.format)
        {
        case Img::CF_GRAY:
          src[iimg] = cvCreateImage(size, IPL_DEPTH_8U, NCHANNELS_GRAY /*1*/);
          if (src[iimg] == NULL)
            {
              release_arrays(src, dst, incolor, &dstcolor,
                             &disp_s, &disp_f, &_3dImage, mapx, mapy);
              return ERROR_CREATE_IMAGE/*-3*/;
            }   
          dst[iimg] = cvCreateImage(size, IPL_DEPTH_8U, NCHANNELS_GRAY /*1*/);
          if (dst[iimg] == NULL)
            {
              release_arrays(src, dst, incolor, &dstcolor,
                             &disp_s, &disp_f, &_3dImage, mapx, mapy);
              return ERROR_CREATE_IMAGE/*-3*/;
            }
          for (row = 0; row < size.height; row++)
            {
              for (col = 0; col < size.width; col++)
                {
                  ((unsigned char *)(src[iimg]->imageData))[addressImage(src[iimg], row, col)]
                    = img_in[iimg].image.raw_data[row * size.width + col];
                }
            }
          break;
        case Img::CF_RGB:
          src[iimg] = cvCreateImage (size, IPL_DEPTH_8U, NCHANNELS_GRAY /*1*/);
          if (src[iimg] == NULL)
            {
              release_arrays (src, dst, incolor, &dstcolor,
                              &disp_s, &disp_f, &_3dImage, mapx, mapy);
              return ERROR_CREATE_IMAGE/*-3*/;
            }
          dst[iimg] = cvCreateImage (size, IPL_DEPTH_8U, NCHANNELS_GRAY /*1*/);
          if (dst[iimg] == NULL)
            {
              release_arrays (src, dst, incolor, &dstcolor,
                              &disp_s, &disp_f, &_3dImage, mapx, mapy);
              return ERROR_CREATE_IMAGE/*-3*/;
            }
          for (row = 0; row < size.height; row++)
            {
              for (col = 0; col < size.width; col++)
                {
                  for (k = 0; k < 3; k++)
                    {
                      ((unsigned char *)(incolor[iimg]->imageData))
                        [addressImage (incolor[iimg], row, col*3)+k]
                        = img_in[iimg].image.raw_data[(row * size.width + col)*3+k];
                    }
                }
            }
          cvCvtColor(incolor[iimg],src[iimg],CV_RGB2GRAY);

          break;
        default: /* CF_UNKNOWN */
          release_arrays(src, dst, incolor, &dstcolor,
                         &disp_s, &disp_f, &_3dImage, mapx, mapy);
          return ERROR_IMAGE_FORMAT;/*-2*/
          break;
        }

      /*fprintf(stderr, "MEMO:measure3d(): set cmat\n");*/
      // cmat 
      //cmat[i] = cvCreateMat(3, 3, CV_32SC1);

      // (fx  0 cx)     (a0 a1 a3)
      // ( 0 fy cy)  =  ( 0 a2 a4)
      // ( 0  0  1)     ( 0  0  1)
      cvmSet(work->cmat[iimg], 0, 0, img_in[iimg].intrinsic.matrix_element[0]);
      cvmSet(work->cmat[iimg], 0, 1, 0.0);
      cvmSet(work->cmat[iimg], 0, 2, img_in[iimg].intrinsic.matrix_element[3]);

      cvmSet(work->cmat[iimg], 1, 0, 0.0);
      cvmSet(work->cmat[iimg], 1, 1, img_in[iimg].intrinsic.matrix_element[2]);
      cvmSet(work->cmat[iimg], 1, 2, img_in[iimg].intrinsic.matrix_element[4]);

      cvmSet(work->cmat[iimg], 2, 0, 0.0);
      cvmSet(work->cmat[iimg], 2, 1, 0.0);
      cvmSet(work->cmat[iimg], 2, 2, 1.0);

      //dcoef
      //dcoef[iimg] = cvCreateMat(NDCOEF, 1, CV_32SC1);
      int n = img_in[iimg].intrinsic.distortion_coefficient.length();
      if (NDCOEF < n)
        {
          n = NDCOEF;
        }
      for (k = 0; k < n; k++)
        {
          cvmSet(work->dcoef[iimg], k, 0, img_in[iimg].intrinsic.distortion_coefficient[k]);
        }
      while (k++ < NDCOEF)
        {
          cvmSet(work->dcoef[iimg], k, 0, 0.0);
        }
    }

  /*fprintf(stderr, "MEMO:measure3d(): set mapx, mapy\n");*/
  for (iimg = 0; iimg < NIMAGE; iimg++)
    {
      mapx[iimg]
        = cvCreateMat(img_in[0].image.height, img_in[0].image.width, CV_32FC1);
      if (mapx[iimg] == NULL)
        {
          release_arrays(src, dst, incolor, &dstcolor,
                         &disp_s, &disp_f, &_3dImage, mapx, mapy);
          return ERROR_CREATE_MAT/*-4*/;
        }
      mapy[iimg]
        = cvCreateMat(img_in[0].image.height, img_in[0].image.width, CV_32FC1);
      if (mapy[iimg] == NULL)
        {
          release_arrays(src, dst, incolor, &dstcolor,
                         &disp_s, &disp_f, &_3dImage, mapx, mapy);
          return ERROR_CREATE_MAT/*-4*/;
        }
    }

  /*fprintf(stderr, "MEMO:measure3d(): set disp_s and disp_f\n");*/

  /* short */
  disp_s = cvCreateImage(size, IPL_DEPTH_16S, NCHANNELS_GRAY /*1*/);
  if (disp_s == NULL)
    {
      release_arrays(src, dst, incolor, &dstcolor,
                     &disp_s, &disp_f, &_3dImage, mapx, mapy);
      return ERROR_CREATE_IMAGE/*-3*/;
    }

  /* float */
  disp_f = cvCreateImage(size, IPL_DEPTH_32F, NCHANNELS_GRAY /*1*/);
  if (disp_f == NULL)
    {
      release_arrays(src, dst, incolor, &dstcolor,
                     &disp_s, &disp_f, &_3dImage, mapx, mapy);
      return ERROR_CREATE_IMAGE/*-3*/;
    }

  /*fprintf(stderr, "MEMO:measure3d(): set _3dImage\n");*/
  _3dImage = cvCreateImage(size, IPL_DEPTH_32F, NCHANNELS_3d /*3*/);
  if (_3dImage == NULL)
    {
      release_arrays(src, dst, incolor, &dstcolor,
                     &disp_s, &disp_f, &_3dImage, mapx, mapy);
      return ERROR_CREATE_IMAGE/*-3*/;
    }

  // 2 cvStereoRectify() <= cmat[12],dcoef[12] => R_[12],P_[12],Q_[12]

  // R (3x3)
  // R1 = R R0
  // R = R1 R0^(T)
  for (row = 0; row < 3; row++)
    {
      for (col = 0; col < 3; col++)
        {
          work->Ri[0]->data.db[row*3+col] = img_in[0].extrinsic[row][col];
          work->Ri[1]->data.db[row*3+col] = img_in[1].extrinsic[row][col];
        }
    }

  /* R2 = R * R1 */
  cvGEMM((const CvArr *)work->Ri[1], (const CvArr *)work->Ri[0],
         1.0, NULL, 0.0, (CvArr *)work->R, CV_GEMM_B_T);

  // T (3x1)
  /* T2 = R *T1 + T
     T = T2 - R*T1
  */
  for (row = 0; row < 3; row++)
    {
      work->T->data.db[row]
        = img_in[1].extrinsic[row][3]
        -(cvGet2D((const CvArr*)work->R,row,0).val[0]*img_in[0].extrinsic[0][3]+
          cvGet2D((const CvArr*)work->R,row,1).val[0]*img_in[0].extrinsic[1][3]+
          cvGet2D((const CvArr*)work->R,row,2).val[0]*img_in[0].extrinsic[2][3]);
    }
  

  
  /*fprintf(stderr, "MEMO:measure3d(): cvStereoRectify()\n");*/
  cvStereoRectify(work->cmat[0], work->cmat[1],
                  work->dcoef[0], work->dcoef[1],
                  size,
                  work->R, work->T,
                  work->Ri[0], work->Ri[1], work->Pi[0], work->Pi[1], work->Q,
                  /* CV_CALIB_ZERO_DISPARITY */
                  0);


  for (iimg = 0; iimg < NIMAGE; iimg++)
    {
      // 3 (2*)cvInitUndistortRectifyMap() <= cmat[iimg],dcoef[iimg],R[iimg]
      //         => create ma11[iimg], map2[iimg] which is new 2d coordinates
    
      /*fprintf(stderr,
	"MEMO:measure3d(): cvInitUndistortRectifyMap()[%d]\n",iimg);*/
      cvInitUndistortRectifyMap(work->cmat[iimg],
                                work->dcoef[iimg],
                                work->Ri[iimg],
                                work->Pi[iimg],
                                mapx[iimg],
                                mapy[iimg]);

      // 4 (2*)cvRemap <= src[iimg],map1[iimg],map2[iimg]=> dst[iimg] (rectified image)
      /*fprintf(stderr, "MEMO:measure3d(): cvRemap()[%d]\n",iimg);*/
      cvRemap(src[iimg], dst[iimg], mapx[iimg],mapy[iimg]);
    }

  
  // 5 cvFindStereoCorrespondenceBM () <= dst[1],dst[2],param =>disparity
  /*fprintf(stderr, "MEMO:measure3d(): cvFindStereoCorrespondenceBM ()\n");*/
  cvFindStereoCorrespondenceBM((const CvArr *)dst[0], (const CvArr *)dst[1], 
                               (CvArr*)disp_s,
                               (CvStereoBMState*)&work->state);

  // 5-1 copy disp_s to disp_f multiplied by 1/16
  for (row = 0; row < disp_s->height; row++)
    {
      for (col = 0; col < disp_s->width; col++)
        {
          tmpv.val[0] = -cvGet2D((const CvArr*)disp_s, row, col).val[0] / 16.0;
          cvSet2D((CvArr*)disp_f, row, col, tmpv);
        }
    }

  // 6 cvReprojectImageTo3D() <=disparity,Q=>3dimage
  /*fprintf(stderr, "MEMO:measure3d(): cvReprojectImageTo3D()\n");*/
  cvReprojectImageTo3D((const CvArr*) disp_f, (CvArr*) _3dImage,
                       (const CvMat*) work->Q, 0);

  // 6.1 convert data to vision coordinate
  // p1 = R p0 + t
  // p2 = Ri[0] p1
  // p1 = Ri[0]^(-1) p2
  // p0 = R^(-1)(p1 - t)
  {
    double p0[3], p1[3], p2[3], Rit[3][3], Rt[3][3], tvec[3];
    short tmps;

    for (j = 0; j < 3; j++)
      {
        for (i = 0; i < 3; i++)
          {
            Rit[j][i] = cvGet2D((const CvArr*)work->Ri[0], i, j).val[0];
          }
      }
    for (j = 0; j < 3; j++)
      {
        for (i = 0; i < 3; i++)
          {
            Rt[j][i] = img_in[0].extrinsic[i][j];
          }
      }
    for (j = 0; j < 3; j++)
      {
        tvec[j] = img_in[0].extrinsic[j][3];
      }
      
    for (row = 0; row < disp_s->height; row++)
      {
        for (col = 0; col < disp_s->width; col++)
          {
            tmps = cvGet2D((const CvArr*) disp_s, row, col).val[0];
            if (tmps / 16 > work->state.minDisparity-1)
              {
                for (j = 0; j < 3; j++)
                  {
                    p2[j] = cvGet2D((const CvArr*) _3dImage, row, col).val[j];
                  }

                mul_m33_v3(Rit, p2, p1);

                //cvMul(Rit, p2, p1);
                for (j = 0; j < 3; j++)
                  {
                    p1[j] -= tvec[j];
                  }
                mul_m33_v3(Rt, p1, p0);
                //cvMul(Rt, p1, p0);
                for (j = 0; j < 3; j++)
                  {
                    tmpv.val[j] = p0[j];
                  }
                cvSet2D(_3dImage, row, col, tmpv);
              }
          }
      }
  }

  // 7 copy 3dimage to data
  // Stereo3D{
  //   TimedPointCloud obj;{
  //     Time tm;
  //     PointCloud data;{
  //       long id;
  //       Mat44 T;
  //       sequence Vec3 point;
  //       sequence Vec3 color;
  //     }
  //   }
  //   TimedMultiCameraImage mimg
  // }

  /*fprintf(stderr, "MEMO:measure3d(): copy 3ddata\n");*/
  outdata->obj.data.id = in_data->data.camera_set_id;
  outdata->obj.tm = in_data->tm;

  /* set I matrix */
  for (i = 0; i < 4; i++)
    {
      for (j = 0; j < 4; j++)
        {
          if (i == j)
            {
              outdata->obj.data.T[i][j] = 1.0;
            }
          else
            {
              outdata->obj.data.T[i][j] = 0.0;
            }
        }
    }

  {
    short tmps;
    count = 0;
    for (row = 0; row < disp_s->height; row++)
      {
        for (col = 0; col < disp_s->width; col++)
          {
            tmps = cvGet2D((const CvArr*) disp_s, row, col).val[0];
            if (tmps / 16 > work->state.minDisparity-1)
              {
                count++;
              }
          }
      }

    outdata->obj.data.point.length (count);
    outdata->obj.data.color.length (count);

    if (img_in[0].image.format == Img::CF_GRAY)
      {
        count = 0;
        for (row = 0; row < disp_s->height; row++)
          {
            for (col = 0; col < disp_s->width; col++)
              {
                tmps = cvGet2D((const CvArr*) disp_s, row, col).val[0];
                if (tmps / 16 > work->state.minDisparity-1)
                  {
                    for (k = 0; k < 3; k++)
                      {
                        outdata->obj.data.point[count][k]
                          = cvGet2D((const CvArr*) _3dImage, row, col).val[k];
                        outdata->obj.data.color[count][k]
                          = cvGet2D((const CvArr*) dst[0], row, col).val[0];
                      }
                    count++;
                  }
              }
          }
      }
    else
      {
        cvRemap((const CvArr *) incolor[0], (CvArr *) dstcolor,
                (const CvArr *) mapx[0], (const CvArr *) mapy[0],
                CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll (0));
    
        count = 0;
        for (row = 0; row < disp_s->height; row++)
          {
            for (col = 0; col < disp_s->width; col++)
              {
                tmps = cvGet2D((const CvArr *) disp_s, row, col).val[0];
                if (tmps / 16 > work->state.minDisparity-1)
                  {
                    for (k = 0; k < 3; k++)
                      {
                        outdata->obj.data.point[count][k]
                          = cvGet2D((const CvArr *) _3dImage, row, col).val[k];
                        outdata->obj.data.color[count][k]
                          = cvGet2D((const CvArr *) dstcolor, row, col).val[k];
                      }
                    count++;
                  }
              }
          }
      }
  }
  
  release_arrays(src, dst, incolor, &dstcolor,
                 &disp_s, &disp_f, &_3dImage, mapx, mapy);
  
  
  return 0;
}
