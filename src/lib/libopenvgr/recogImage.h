/* -*- coding: utf-8 -*-
 recogImage.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file recogImage.h
 * @brief 画像入出力関数
 * @date \$Date::                            $
 */

#ifndef _RECOGIMAGE_H
#define _RECOGIMAGE_H

#include <cv.h>
#include <highgui.h>

#include "common.h"
#include "calib.h"

typedef struct _recogImage
{
  int colsize;
  int rowsize;
  int bytePerPixel;
  unsigned char *pixel;
} RecogImage;

// 画像メモリの生成
RecogImage *constructImage (const int colsize, const int rowsize,
                            const int bytePerPixel);

// 画像メモリの解放
void destructImage (RecogImage * image);

// RGB画像からGray画像への変換
void rgb2grayImage (RecogImage * target, RecogImage * source);

// 歪み補正画像の作成
void undistortImage(const RecogImage* src, RecogImage* dst, CameraParam* cp);

// 歪み補正画像の作成(IplImage用)
void undistortImage(const IplImage* src, IplImage* dst, CameraParam* cp);

// 画像メモリのファイル出力　デバッグ用
void writeRecogImage(const char* filename, const RecogImage* img);
#endif // _RECOGIMAGE_H
