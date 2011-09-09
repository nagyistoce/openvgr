/*
 calibUtil.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
/*!
 * @file calibUtil.h
 * @brief キャリブレーションデータの変換関連
 */

#ifndef _CALIBUTIL_H
#define _CALIBUTIL_H

#include "match3Dfeature.h"
#include "Img.hh"

//! CameraImage 内のキャリブレーションデータを、
//! Calib 構造体にセットする。
extern void setCalibFromCameraImage(const Img::CameraImage& image, CameraParam& camera);

#endif // _CALIBUTIL_H
