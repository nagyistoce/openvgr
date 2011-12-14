/*
 VRMLWriter.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#ifndef VRMLWRITER_H
#define VRMLWRITER_H

#include <stdio.h>
#include "Model.h"

#ifdef __cplusplus
extern "C" {
#endif

//! VRMLのヘッダ出力
void writeVRML_Header(FILE* fp);

//! VRMLの入れ子の一番外側
void writeVRML_Intro(FILE* fp);

//! ワイヤーフレームモデルの出力、点の定義まで
void writeWireModelStart(FILE* fp);

//! 点の定義
void pointDefine(FILE* fp, const LinePoint* defPoint);

//! 点の定義を終了し、繋ぐ部分へ進める
void pointDefineEnd(FILE* fp);

//! 立体出力終了
//! 色をつけていない場合 = 0, つけている場合 = 1
void writeWireModelEnd(FILE* fp, const int colorFlag);

//! VRMLの文字描画
void writeVRML_Text(FILE* fp, const char outText[128],
                    const double coordX, const double coordY, const double coordZ);

//! 座標軸をVRMLファイルに挿入する関数
void writeAxisModel(FILE* fp, double axisLength);

//! 法線描画用
void writeNormalModel(FILE* fp, const LinePoint* center,
                      const Vector* normalVector, const double length,
                      const int nFace);

#ifdef __cplusplus
}
#endif

#endif
