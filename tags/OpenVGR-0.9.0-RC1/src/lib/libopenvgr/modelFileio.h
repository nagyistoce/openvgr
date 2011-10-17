/*
 modelFileio.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
/*!
 * @file modelFileio.h
 * @brief モデルをファイルから読み込む。
 */
#ifndef _MODELFILEIO_H
#define _MODELFILEIO_H

#include "match3Dfeature.h"

// モデルをファイルから読み込む。
extern int loadModelFile(char* path, Features3D& model);

#endif // _MODELFILEIO_H
