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

#include <cv.h>
#include <highgui.h>

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
  if (image)
    {
      free(image->pixel);
      free(image);
    }
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

void
undistortImage(const RecogImage* src, RecogImage* dst, CameraParam* cp)
{
  double Anew_elem[3][3], dist[5];
  cv::Mat A(3, 3, CV_64FC1, cp->intrinsicMatrix), Anew(3, 3, CV_64FC1, Anew_elem), distCoeffs(5, 1, CV_64FC1, dist);
  cv::Mat dimg, img, result;

  int i;

  assert(src->rowsize == dst->rowsize);
  assert(src->colsize == dst->colsize);

  /* 歪み有り画像データをセット */
  if (src->bytePerPixel == 1)
    {
      dimg = cv::Mat(src->rowsize, src->colsize, CV_8UC1, src->pixel);
    }
  else
    {
      dimg = cv::Mat(src->rowsize, src->colsize, CV_8UC3, src->pixel);
    }

  /* 歪みパラメータをセット */
  dist[0] = cp->Distortion.k1;
  dist[1] = cp->Distortion.k2;
  dist[2] = cp->Distortion.p1;
  dist[3] = cp->Distortion.p2;
  dist[4] = cp->Distortion.k3;

  /* 歪み補正後の内部パラメータを計算 */
#if 0
  Anew = cv::getOptimalNewCameraMatrix(A, distCoeffs, cv::Size(src->rowsize, src->colsize), 0.0);
#else
  A.copyTo(Anew);
#endif

  /* 歪み補正画像を作成 */
  cv::undistort(dimg, img, A, distCoeffs, Anew);

#if 0
  cv::namedWindow("original image", CV_WINDOW_AUTOSIZE);
  cv::imshow("original image", dimg);

  cv::namedWindow("corrected image", CV_WINDOW_AUTOSIZE);
  cv::imshow("corrected image", img);

  cv::waitKey(-1);
#endif

  /* 出力先のカラーフォーマットに合わせる */
  if (src->bytePerPixel == dst->bytePerPixel)
    {
      result = img;
    }
  else
    {
      if (src->bytePerPixel == 1) /* 白黒をカラーに */
        {
          cv::cvtColor(img, result, CV_GRAY2RGB);
        }
      else /* カラーを白黒に */
        {
          cv::cvtColor(img, result, CV_RGB2GRAY);
        }
    }

  /* 補正済み画像をコピー */
  for (i = 0; i < dst->rowsize; ++i)
    {
      const int byte_per_row = dst->colsize * dst->bytePerPixel;
      uchar* uptr = result.ptr<uchar>(i);

      memcpy(dst->pixel + byte_per_row * i, uptr, sizeof(uchar) * byte_per_row);
    }

  /* 新しいカメラパラメータを設定 */
  Anew.copyTo(A);

  cp->Distortion.k1 = 0.0;
  cp->Distortion.k2 = 0.0;
  cp->Distortion.p1 = 0.0;
  cp->Distortion.p2 = 0.0;
  cp->Distortion.k3 = 0.0;
}
