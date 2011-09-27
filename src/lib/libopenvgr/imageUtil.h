/*
 imageUtil.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file imageUtil.h
 * @brief 画像入出力関連
 * @date \$Date::                            $
 */

#ifndef _IMAGEUTIL_H
#define _IMAGEUTIL_H

#include "recogImage.h"
#include "match3Dfeature.h"
#include "Img.hh"

//! TimedMultiCameraImage 画像データを、歪み補正をして OpenCV の IplImage
//! 構造体形式に変換する。
//! 画像 1 枚ごとに個別の IplImage 構造体を作成する。
//! 画像は、channel 数 3 で確保する。
extern IplImage** convertTimedMultiCameraImageToUndistortIplImage(const Img::TimedMultiCameraImage& frame,
                                                                  CalibParam& calib);

//! convertTimedMultiCameraImageToUndistortIplImage によって
//! 確保されたメモリを開放する
extern void freeUndistortIplImage(IplImage** resultImage, int imageNum);

//! TimedMultiCameraImage 画像データを、OpenCV の IplImage 構造体形式に変換する。
//! 画像 1 枚ごとに個別の IplImage 構造体を作成する。
//! 画像は、channel 数 3 で確保する。
extern IplImage** convertTimedMultiCameraImageToIplImage(const Img::TimedMultiCameraImage& frame);

//! TimedMultiCameraImage 画像データを、RecogImage 構造体形式に変換する。
//! 画像は 1 枚ごとに個別の RecogImage 構造体を作成する。
extern RecogImage** convertTimedMultiCameraImageToRecogImage(const Img::TimedMultiCameraImage& frame, const bool color_mode = false);

//! convertTimedMultiCameraImageToRecogImage によって
//! 確保されたメモリを開放する
extern void freeConvertedRecogImage(RecogImage** recogImage, int imageNum);

#endif // _IMAGEUTIL_H
