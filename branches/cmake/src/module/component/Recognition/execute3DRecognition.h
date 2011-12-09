/*
 execute3DRecognition.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
/*!
 * @file execute3DRecognition.h
 * @brief 3次元物体認識の実行
 */

#ifndef EXECUTE3DRECOGNITION_H
#define EXECUTE3DRECOGNITION_H

#include "VisionSVC_impl.h"

// 3 次元物体認識の実行
extern int execute3DRecognition(Img::TimedMultiCameraImage& frame,
                                int modelID, char* modelFilePath,
                                Parameters& param,
                                TimedRecognitionResult& result);

#endif // EXECUTE3DRECOGNITION_H
