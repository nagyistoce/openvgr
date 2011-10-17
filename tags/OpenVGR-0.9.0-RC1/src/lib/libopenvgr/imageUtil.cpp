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
