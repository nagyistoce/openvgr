/*
 ModelInformation.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#ifndef MODELINFORMATION_H
#define MODELINFORMATION_H

#include <stdio.h>
#include "Model.h"

#ifdef __cplusplus
extern "C" {
#endif

//! 入力ファイルのコマンドを出力
  void writeCommandInfo(FILE* fp, const ModelParameter* commandParameter,
                        const RotationAngle* rotation, const Vector* moveBuff);

//! 重心情報を出力
void writeGravityInfo(FILE * fp, const Vector * gravity);

//! 円筒の情報を出力
void writeCylinderlInfo(FILE * fp, const ModelParameter * circleParameter,
                        const LinePoint* centerPoint,   // 円の中心座標
                        const Vector* normalVector);    // 法線ベクトル
  
//! 直方体の頂点情報を出力
void writeVertex(FILE* fp, const SquareParameter* faceParameter, const int nSquare);

#ifdef __cplusplus
}
#endif

#endif // MODELINFORMATION_H
