/*
 recogImage.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file recogImage.cpp
 * @brief 画像入出力関数
 * @date \$Date::                            $
 */
#include <stdlib.h>
#include <string.h>

#include "recogImage.h"

//! 画像メモリの確保と初期化
RecogImage*
constructImage(const int colsize, const int rowsize, const int bytePerPixel)
{
  RecogImage* image;

  image = (RecogImage*) malloc(sizeof(RecogImage));
  if (image == NULL)
    {
      return NULL;
    }
  image->colsize = colsize;
  image->rowsize = rowsize;
  image->bytePerPixel = bytePerPixel;
  image->pixel = (unsigned char*) malloc(sizeof(*(image->pixel)) * colsize * rowsize * bytePerPixel);
  if (image->pixel == NULL)
    {
      free(image);
      return NULL;
    }
  memset(image->pixel, 0, sizeof(*(image->pixel)) * colsize * rowsize * bytePerPixel);

  return image;
}

//! 画像メモリの解放
void
destructImage(RecogImage* image)
{
  free(image->pixel);
  free(image);
  return;
}

//! RGB画像からGrey画像への変換
void
rgb2grayImage(RecogImage* target, RecogImage* source)
{
  if (target->bytePerPixel == source->bytePerPixel)
    {
      return;
    }
  if (target->bytePerPixel == 1)
    {                           // RGB->GRAY
      const int size = (target->colsize) * (target->rowsize);
      unsigned char* tar = target->pixel;
      unsigned char* src = source->pixel;
      int c;
      int r, g, b;
      for (c = 0; c < size; c++)
        {
          r = *(src++);
          g = *(src++);
          b = *(src++);
#if 0
#if 0
          //Grey 画像は0.299 * Red + 0.587 * Green + 0.114 * Blue で計算している。
          *(tar++) = 0.299 * r + 0.587 * g + 0.114 * b;
#else
          // 256 倍して int のままで計算する。
          *(tar++) = (76 * r + 150 * g + 29 * b) / 256;
#endif
#else
#if 1
          *(tar++) = (r + g + b) / 3;
#else
          // green 成分のみ使用
          *(tar++) = g;
#endif
#endif
        }
    }
  else
    {                           // GRAY->RGB
      const int size = (target->colsize) * (target->rowsize);
      unsigned char* tar = target->pixel;
      unsigned char* src = source->pixel;
      int c;
      for (c = 0; c < size; c++)
        {
          int c = *(src++);
          *(tar++) = c;
          *(tar++) = c;
          *(tar++) = c;
        }
    }
  return;
}
