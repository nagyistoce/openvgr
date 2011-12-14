/*
 RecognitionKernel.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
/*!
 * @file RecognitionKernel.h
 * @brief 3次元物体認識の中核処理
 */

#ifndef RECOGNITIONKERNEL_H
#define RECOGNITIONKERNEL_H

// 3 次元物体認識の実行
Match3Dresults
RecognitionKernel(RecogImage** image,
                  CalibParam&  calib,
                  Features3D&  model,
                  Parameters&  param);

#endif // RECOGNITIONKERNEL_H
