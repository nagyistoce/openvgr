/*
 imageUtil.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file imageUtil.cpp
 * @brief 画像入出力関数
 * @date \$Date::                            $
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <cv.h>

#include "recogImage.h"
#include "imageUtil.h"
#include "visionErrorCode.h"

//
//! CameraIntrinsicParameter 構造体を OpenCV の CvMat 構造体形式に変換する。
//
int
convertCameraIntrinsicParameterToCvMat(const Img::CameraIntrinsicParameter& intrinsic,
                                       CvMat** intrinsic_matrix,
                                       CvMat** distortion_coeffs)
{
  *intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
  if (*intrinsic_matrix == (CvMat*) NULL)
    {
      return VISION_MALLOC_ERROR;
    }
  *distortion_coeffs = cvCreateMat(1, 5, CV_32FC1);
  if (*distortion_coeffs == (CvMat*) NULL)
    {
      cvReleaseMat(intrinsic_matrix);
      *intrinsic_matrix = (CvMat*)NULL;
      return VISION_MALLOC_ERROR;
    }

  (*intrinsic_matrix)->data.fl[0] = intrinsic.matrix_element[0];
  (*intrinsic_matrix)->data.fl[1] = intrinsic.matrix_element[1];
  (*intrinsic_matrix)->data.fl[2] = intrinsic.matrix_element[3];

  (*intrinsic_matrix)->data.fl[3] = 0.0;
  (*intrinsic_matrix)->data.fl[4] = intrinsic.matrix_element[2];
  (*intrinsic_matrix)->data.fl[5] = intrinsic.matrix_element[4];

  (*intrinsic_matrix)->data.fl[6] = 0.0;
  (*intrinsic_matrix)->data.fl[7] = 0.0;
  (*intrinsic_matrix)->data.fl[8] = 1.0;

  int i;

  for (i = 0; i < 5; i++)
    {
      (*distortion_coeffs)->data.fl[i] = 0.0;
    }

  int ndist = intrinsic.distortion_coefficient.length();

  if (ndist > 5)
    {
      ndist = 5;
    }

  for (i = 0; i < ndist; i++)
    {
      (*distortion_coeffs)->data.fl[i] = intrinsic.distortion_coefficient[i];
    }

  return 0;
}

//
//! CameraIntrinsicParameter 構造体から、OpenCV の歪み補正マップを作成する。
//
int
createUndistortionMap(int width, int height,
                      const Img::CameraIntrinsicParameter& intrinsic,
                      IplImage** mapx, IplImage** mapy)
{
  CvMat *intrinsic_matrix = NULL;
  CvMat *distortion_coeffs = NULL;

  int ret = convertCameraIntrinsicParameterToCvMat(intrinsic, &intrinsic_matrix, &distortion_coeffs);
  if (ret != 0)
    {
      return ret;
    }

  *mapx = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
  *mapy = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);

  if (*mapx && *mapy)
    {
      cvInitUndistortMap(intrinsic_matrix, distortion_coeffs, *mapx, *mapy);
      ret = 0;
    }
  else
    {
      if (*mapx)
        {
          cvReleaseImage(mapx);
        }
      if (*mapy)
        {
          cvReleaseImage(mapy);
        }
      ret =  VISION_MALLOC_ERROR;
    }

  cvReleaseMat(&intrinsic_matrix);
  cvReleaseMat(&distortion_coeffs);

  return 0;
}

//
//! TimedMultiCameraImage 構造体のそれぞれの画像の歪み補正マップを作成する。
//
int
createUndistortionMapFromTimedMultiCameraImage(const Img::TimedMultiCameraImage& frame,
                                               IplImage*** mapx,
                                               IplImage*** mapy)
{
  int imageNum = frame.data.image_seq.length();
  if (imageNum < 1)
    {
      // 画像がない場合、終了。
      return VISION_PARAM_ERROR;
    }
  if (imageNum > 3)
    {
      // 画像が 3 枚以上含まれる場合は 3 枚まで
      imageNum = 3;
    }

  int imageWidth = frame.data.image_seq[0].image.width;
  int imageHeight = frame.data.image_seq[0].image.height;

  *mapx = (IplImage**) malloc(imageNum * sizeof(IplImage*));
  if (*mapx == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  *mapy = (IplImage**) malloc(imageNum * sizeof(IplImage*));
  if (*mapy == NULL)
    {
      free(*mapx);
      *mapx = NULL;
      return VISION_MALLOC_ERROR;
    }

  int ret;
  int i, j;
  for (i = 0; i < imageNum; i++)
    {
      ret = createUndistortionMap(imageWidth, imageHeight,
                                  frame.data.image_seq[i].intrinsic,
                                  *mapx + i, *mapy + i);
      if (ret != 0)
        {
          for (j = 0; j < i; j++)
            {
              cvReleaseImage(*mapx + i);
              cvReleaseImage(*mapy + i);
            }
          free(*mapx);
          free(*mapy);
          return ret;
        }
    }

  return 0;
}

//
//! TimedMultiCameraIamge 画像データを、歪みを補正して OpenCV の IplImage 
//! 構造体形式に変換する。
//! ImageData::raw_data は、TopLeft、RGB 順、行単位での padding なし。
//! 画像 1 枚ごとに個別の IplImage 構造体を作成する。
//! 画像は channel 数 3 で確保する。
//
IplImage**
convertTimedMultiCameraImageToUndistortIplImage(const Img::TimedMultiCameraImage& frame,
                                                IplImage*** mapx,
                                                IplImage*** mapy)
{
  int imageNum = frame.data.image_seq.length();
  if (imageNum < 1)
    {
      // 画像がない場合、終了。
      return NULL;
    }
  if (imageNum > 3)
    {
      // 画像が 3 枚以上含まれる場合は 3 枚まで
      imageNum = 3;
    }

  int imageWidth = frame.data.image_seq[0].image.width;
  int imageHeight = frame.data.image_seq[0].image.height;
  int channelNum = 3;
  if (frame.data.image_seq[0].image.format == Img::CF_GRAY)
    {
      channelNum = 1;
    }
  int imageSize = imageWidth * imageHeight;

  IplImage** resultImage = (IplImage**) malloc(imageNum * sizeof(IplImage*));
  if (resultImage == NULL)
    {
      return NULL;
    }

  IplImage* distImage;

  // 歪み補正マップを作成する。
  int ret =
    createUndistortionMapFromTimedMultiCameraImage(frame, mapx, mapy);
  if (ret != 0)
    {
      free(resultImage);
      return NULL;
    }

  int i, j;
  int count;

  imageSize *= 3;

  CvSize size;
  size.width = imageWidth;
  size.height = imageHeight;
  for (i = 0; i < imageNum; i++)
    {
      distImage = cvCreateImage(size, IPL_DEPTH_8U, 3);
      if (distImage == NULL)
        {
          free(resultImage);
          return NULL;
        }

      *(resultImage + i) = cvCreateImage(size, IPL_DEPTH_8U, 3);
      if (*(resultImage + i) == NULL)
        {
          // エラー処理
          for (j = 0; j < i; j++)
            {
              cvReleaseImage(resultImage + j);
            }
          free(resultImage);

          return NULL;
        }

      // 画像 Data をコピーする。
      if (channelNum == 3)
        {
          // カラー画像
          for (j = 0; j < imageSize; j += 3)
            {
              distImage->imageData[j]
                = (unsigned char) frame.data.image_seq[i].image.raw_data[j + 2];
              distImage->imageData[j + 1]
                = (unsigned char) frame.data.image_seq[i].image.raw_data[j + 1];
              distImage->imageData[j + 2]
                = (unsigned char) frame.data.image_seq[i].image.raw_data[j];
            }
        }
      else if (channelNum == 1)
        {
          // グレー画像
          count = 0;
          for (j = 0; j < imageSize; j += 3)
            {
              distImage->imageData[j]
                = (unsigned char) frame.data.image_seq[i].image.raw_data[count];
              distImage->imageData[j + 1]
                = (unsigned char) frame.data.image_seq[i].image.raw_data[count];
              distImage->imageData[j + 2]
                = (unsigned char) frame.data.image_seq[i].image.raw_data[count];
              count++;
            }
        }

      // 歪み補正を行う。
      cvRemap(distImage, *(resultImage + i), *(*mapx + i), *(*mapy + i));
#ifdef _DEBUG
      char filename[256];
      sprintf(filename, "/tmp/undistImage%d.ppm", i);
      cvSaveImage(filename, *(resultImage + i));
#endif
      cvReleaseImage(&distImage);
    }

  return resultImage;
}

//
//! convertTimedMultiCameraImageToUndistortIplImage によって
//! 確保されたメモリを開放する
//
void
freeUndistortIplImage(IplImage** resultImage,
                      IplImage** undistortMapX,
                      IplImage** undistortMapY,
                      int imageNum)
{
  if( !resultImage || !undistortMapX || !undistortMapY )
    {
      return;
    }

  if( imageNum < 0 )
    {
      return;
    }

  if( imageNum > 3 )
    {
      imageNum = 3;
    }

  for(int i = 0; i < imageNum; i++)
    {
      cvReleaseImage(resultImage + i);
      cvReleaseImage(undistortMapX + i);
      cvReleaseImage(undistortMapY + i);
    }
  free(resultImage);
  free(undistortMapX);
  free(undistortMapY);
}

//
//! TimedMultiCameraIamge 画像データを OpenCV の IplImage 構造体形式に変換する。
//! ImageData::raw_data は、TopLeft、RGB 順、行単位での padding なし。
//! 画像 1 枚ごとに個別の IplImage 構造体を作成する。
//! 画像は channel 数 3 で確保する。
//
IplImage**
convertTimedMultiCameraImageToIplImage(const Img::TimedMultiCameraImage& frame)
{
  int imageNum = frame.data.image_seq.length();
  if (imageNum < 1)
    {
      // 画像がない場合、終了。
      return NULL;
    }
  if (imageNum > 3)
    {
      // 画像が 3 枚以上含まれる場合は 3 枚まで
      imageNum = 3;
    }

  int imageWidth = frame.data.image_seq[0].image.width;
  int imageHeight = frame.data.image_seq[0].image.height;
  int channelNum = 3;
  if (frame.data.image_seq[0].image.format == Img::CF_GRAY)
    {
      channelNum = 1;
    }
  int imageSize = imageWidth * imageHeight;

  IplImage** resultImage = (IplImage**) malloc(imageNum * sizeof(IplImage *));
  if (resultImage == NULL)
    {
      return NULL;
    }

  int i, j;
  int count;

  imageSize *= 3;

  CvSize size;
  size.width = imageWidth;
  size.height = imageHeight;
  for (i = 0; i < imageNum; i++)
    {
      *(resultImage + i) = cvCreateImage(size, IPL_DEPTH_8U, 3);
      if (*(resultImage + i) == NULL)
        {
          // エラー処理
          for (j = 0; j < i; j++)
            {
              cvReleaseImage(resultImage + j);
            }
          free(resultImage);
          return NULL;
        }

      // 画像 Data をコピーする。
      if (channelNum == 3)
        {
          // カラー画像
          for (j = 0; j < imageSize; j += 3)
            {
              (*(resultImage + i))->imageData[j] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[j + 2];
              (*(resultImage + i))->imageData[j + 1] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[j + 1];
              (*(resultImage + i))->imageData[j + 2] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[j];
            }
        }
      else if (channelNum == 1)
        {
          // グレー画像
          count = 0;
          for (j = 0; j < imageSize; j += 3)
            {
              (*(resultImage + i))->imageData[j] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[count];
              (*(resultImage + i))->imageData[j + 1] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[count];
              (*(resultImage + i))->imageData[j + 2] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[count];
              count++;
            }
        }
    }

  return resultImage;
}

//
//! TimedMultiCameraImage 画像データを、RecogImage 構造体形式に変換する。
//! カラー画像はグレー画像に変換する。
//
RecogImage**
convertTimedMultiCameraImageToRecogImage(const Img::TimedMultiCameraImage& frame)
{
  int imageNum = frame.data.image_seq.length();
  if (imageNum < 1)
    {
      // 画像がない場合、終了。
      return NULL;
    }

  int imageWidth = frame.data.image_seq[0].image.width;
  int imageHeight = frame.data.image_seq[0].image.height;
  int channelNum = 3;
  if (frame.data.image_seq[0].image.format == Img::CF_GRAY)
    {
      channelNum = 1;
    }
  int imageSize = imageWidth * imageHeight * channelNum;

  RecogImage** resultImage = (RecogImage**) malloc(imageNum * sizeof(RecogImage *));
  if (resultImage == NULL)
    {
      return NULL;
    }

  RecogImage* colorImage = NULL;
  if (channelNum == 3)
    {
      colorImage = constructImage(imageWidth, imageHeight, 3);
      if (colorImage == NULL)
        {
          return NULL;
        }
    }

  int i, j;

  for (i = 0; i < imageNum; i++)
    {
      *(resultImage + i) = constructImage(imageWidth, imageHeight, 1);
      if (*(resultImage + i) == NULL)
        {
          // エラー処理
          for (j = 0; j < i; j++)
            {
              destructImage(*(resultImage + j));
            }
          free(resultImage);
          destructImage(colorImage);
          return NULL;
        }

      // 画像 Data をコピーする。
      if (channelNum == 3)
        {
          // カラー画像
          for (j = 0; j < imageSize; j += 3)
            {
              colorImage->pixel[j] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[j + 2];
              colorImage->pixel[j + 1] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[j + 1];
              colorImage->pixel[j + 2] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[j];
            }

          // グレー画像に変換する。
          rgb2grayImage(*(resultImage + i), colorImage);

        }
      else if (channelNum == 1)
        {
          // グレー画像
          for (j = 0; j < imageSize; j++)
            {
              (*(resultImage + i))->pixel[j] =
                (unsigned char) frame.data.image_seq[i].image.raw_data[j];
            }
        }
    }

  destructImage(colorImage);

  return resultImage;
}

//
//! convertTimedMultiCameraImageToRecogImage によって
//! 確保されたメモリを開放する
//
void
freeConvertedRecogImage(RecogImage** recogImage, int imageNum)
{
  if (recogImage)
    {
      for (int i = 0; i < imageNum; i++)
        {
          destructImage(*(recogImage + i));
        }
      free(recogImage);
    }
}
