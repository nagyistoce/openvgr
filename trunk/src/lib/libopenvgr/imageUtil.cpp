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

#include "recogImage.h"
#include "imageUtil.h"
#include "calibUtil.h"
#include "visionErrorCode.h"


//! TimedMultiCameraIamge 画像データを、歪みを補正して OpenCV の IplImage 
//! 構造体形式に変換する。
//! ImageData::raw_data は、TopLeft、RGB 順、行単位での padding なし。
//! 画像 1 枚ごとに個別の IplImage 構造体を作成する。
//! 画像は channel 数 3 で確保する。
//
IplImage**
convertTimedMultiCameraImageToUndistortIplImage(const Img::TimedMultiCameraImage& frame,
                                                CalibParam& calib)
{
  int imageNum = frame.data.image_seq.length();
  IplImage** resultImage = convertTimedMultiCameraImageToIplImage(frame);

  if (resultImage == NULL)
    {
      return NULL;
    }

  // キャリブレーションデータを変換する。
  calib.numOfCameras = imageNum;
  calib.colsize = frame.data.image_seq[0].image.width;
  calib.rowsize = frame.data.image_seq[1].image.height;
  setCalibFromCameraImage(frame.data.image_seq[0], calib.CameraL);
  setCalibFromCameraImage(frame.data.image_seq[1], calib.CameraR);
  if (imageNum > 2)
    {
      setCalibFromCameraImage(frame.data.image_seq[2], calib.CameraV);
    }

  CameraParam* cp[3];
  cp[0] = &calib.CameraL;
  cp[1] = &calib.CameraR;
  cp[2] = &calib.CameraV;

  for (int i = 0; i < imageNum; i++)
    {
      // 歪み補正を行う。
      undistortImage(*(resultImage + i), *(resultImage + i), cp[i]);
#ifdef _DEBUG
      char filename[256];
      sprintf(filename, "/tmp/undistImage%d.ppm", i);
      cvSaveImage(filename, *(resultImage + i));
#endif
    }

  return resultImage;
}

//
//! convertTimedMultiCameraImageToUndistortIplImage によって
//! 確保されたメモリを開放する
//
void
freeUndistortIplImage(IplImage** resultImage, int imageNum)
{
  if( !resultImage )
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
    }
  free(resultImage);
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
//! color_mode == false のときカラー画像をグレー画像に変換する。
//
RecogImage**
convertTimedMultiCameraImageToRecogImage(const Img::TimedMultiCameraImage& frame, const bool color_mode)
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
  fprintf(stderr, "Channel (%s): %d\n", __FUNCTION__, channelNum);

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

  int i, j, k;

  // メモリの確保
  for (i = 0; i < imageNum; ++i)
    {
      if (color_mode == false)
        {
          resultImage[i] = constructImage(imageWidth, imageHeight, 1);
        }
      else
        {
          resultImage[i] = constructImage(imageWidth, imageHeight, 3);
        }

      if (resultImage[i] == NULL)
        {
          // エラー処理
          while (--i >= 0)
            {
              destructImage(resultImage[i]);
            }
          free(resultImage);
          destructImage(colorImage);
          return NULL;
        }
    }

  // 画像データをコピーする。
  if (color_mode == false)
    {
      for (i = 0; i < imageNum; i++)
        {
          if (channelNum == 3)
            {
              // カラー画像
              for (j = 0; j < imageSize; j++)
                {
                  colorImage->pixel[j] = (unsigned char) frame.data.image_seq[i].image.raw_data[j];
                }
              // グレー画像に変換する。
              rgb2grayImage(resultImage[i], colorImage);
            }
          else if (channelNum == 1)
            {
              // グレー画像
              for (j = 0; j < imageSize; j++)
                {
                  resultImage[i]->pixel[j] = (unsigned char) frame.data.image_seq[i].image.raw_data[j];
                }
            }
        }
    }
  else
    {
      for (i = 0; i < imageNum; i++)
        {
          if (channelNum == 3)
            {
              // カラー画像
              for (j = 0; j < imageSize; j++)
                {
                  resultImage[i]->pixel[j] = (unsigned char) frame.data.image_seq[i].image.raw_data[j];
                }
            }
          else if (channelNum == 1)
            {
              // グレー画像
              for (j = 0; j < imageSize; j++)
                {
                  for (k = 0; k < 3; k++)
                    {
                      resultImage[i]->pixel[3*j + k] = (unsigned char) frame.data.image_seq[i].image.raw_data[j];
                    }
                }
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
