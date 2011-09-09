/* -*- coding: utf-8 -*-
 VRMLWriter.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include "VRMLWriter.h"

//! VRMLのヘッダ出力
void
writeVRML_Header(FILE* fp)
{
  fprintf(fp, "#VRML V2.0 utf8\n");
}

//! VRMLの入れ子の一番外側
void
writeVRML_Intro(FILE* fp)
{
  fprintf(fp, "Transform {\n\tchildren [\n");
}

//! ワイヤーフレームモデルの出力、点の定義まで
void
writeWireModelStart(FILE* fp)
{
  writeVRML_Intro(fp);
  fprintf(fp, "Shape {\n\tgeometry IndexedLineSet {\n\t\tcoord Coordinate {\n");
  fprintf(fp, "\t\t\tpoint [\n");
}

//! 点の定義
void
pointDefine(FILE* fp, const LinePoint* defPoint)
{
  fprintf(fp, "\t\t\t\t%.3lf %.3lf %.3lf,\n",
          defPoint->x, defPoint->y, defPoint->z);
}

//! 点の定義を終了し、繋ぐ部分へ進める
void
pointDefineEnd(FILE* fp)
{
  fprintf(fp, "\t\t\t]\n\t\t}\n\t\tcoordIndex [\n");
}

//! 立体出力終了
//! 色をつけていない場合 = 0, つけている場合 = 1
void
writeWireModelEnd(FILE* fp, const int colorFlag)
{
  if (colorFlag == USE_COLOR)
    {
      fprintf(fp, "\t\t}\n\t}\n}\n\t]\n}\n\n");
    }
  else
    {
      fprintf(fp, "\t\t]\n\t}\n}\n\t]\n}\n\n");
    }
}

//! VRMLの文字描画
void
writeVRML_Text(FILE* fp, const char outText[128],
	       const double coordX, const double coordY, const double coordZ)
{
  writeVRML_Intro(fp);
  fprintf(fp, "Billboard {\n\tchildren [\nShape {\n\tgeometry Text {\n");
  fprintf(fp, "\t\tstring [ \"%s\" ]\n\t}\n}\n\t]\n\taxisOfRotation 0 0 0\n", outText);
  fprintf(fp, "}\n\t]\n\ttranslation %.3lf %.3lf %.3lf\n}\n\n", coordX, coordY, coordZ);
}

//! 座標軸をVRMLファイルに挿入する関数
void
writeAxisModel(FILE* fp, double axisLength)
{
  axisLength *= 1.5;            // 軸の長さは立体の一番長い部分のｎ倍

  // 原点に軸
  fprintf(fp, "#座標軸\n");
  writeWireModelStart(fp);
  fprintf(fp, "\t\t\t\t0 0 0,\n\t\t\t\t%.3lf 0 0,\n", axisLength);
  fprintf(fp, "\t\t\t\t0 0 0,\n\t\t\t\t0 %.3lf 0,\n", axisLength);
  fprintf(fp, "\t\t\t\t0 0 0,\n\t\t\t\t0 0 %.3lf\n", axisLength);
  pointDefineEnd(fp);
  fprintf(fp, "\t\t\t0 1 -1,\n\t\t\t2 3 -1,\n\t\t\t4 5 -1\n\t\t]\n");  // 線を結ぶ
  fprintf(fp, "\t\tcolor Color {\n\t\t\tcolor [\n\t\t\t\t1 0 0,\n\t\t\t\t1 0 0,\n\t\t\t\t0 1 0,\n");
  fprintf(fp, "\t\t\t\t0 1 0,\n\t\t\t\t0 0 1,\n\t\t\t\t0 0 1,\n\t\t\t]\n");
  writeWireModelEnd(fp, USE_COLOR);

  axisLength = (axisLength * 1.1);      // xyzの文字は軸の少し先に書く
  // xyz軸の文字
  fprintf(fp, "#座標軸の文字(xyz)\n");
  writeVRML_Text(fp, "x", axisLength, 0, 0);
  writeVRML_Text(fp, "y", 0, axisLength, 0);
  writeVRML_Text(fp, "z", 0, 0, axisLength);
}

//! 法線描画用
void
writeNormalModel(FILE* fp, const LinePoint* center,
                 const Vector* normalVector, const double length,
                 const int nFace)
{
  int faceNumber;
  LinePoint normalVectorEnd[2];

  for (faceNumber = 0; faceNumber < nFace; faceNumber++)
    {
      normalVectorEnd[faceNumber].x
	= (normalVector[faceNumber].x * length) + center[faceNumber].x;

      normalVectorEnd[faceNumber].y
	= (normalVector[faceNumber].y * length) + center[faceNumber].y;

      normalVectorEnd[faceNumber].z
	= (normalVector[faceNumber].z * length) + center[faceNumber].z;
    }

  fprintf(fp, "#法線\n");
  writeWireModelStart(fp);
  // 法線の点を定義
  for (faceNumber = 0; faceNumber < nFace; faceNumber++)
    {
      pointDefine(fp, &center[faceNumber]);
      pointDefine(fp, &normalVectorEnd[faceNumber]);
    }
  pointDefineEnd(fp);

  // 法線を結ぶ
  for (faceNumber = 0; faceNumber < nFace; faceNumber++)
    {
      fprintf(fp, "\t\t\t%d %d -1,\n", (faceNumber * 2), (faceNumber * 2) + 1);
    }
  fprintf(fp, "\t\t]\n");

  // 色をつける
  fprintf(fp, "\t\tcolor Color {\n\t\t\tcolor [\n");
  for (faceNumber = 0; faceNumber < nFace; faceNumber++)
    {
      fprintf(fp, "\t\t\t\t1 %d %d,\n\t\t\t\t1 %d %d,\n",
              faceNumber, 1 - faceNumber, faceNumber, 1 - faceNumber);
    }
  fprintf(fp, "\t\t\t]\n");
  writeWireModelEnd(fp, USE_COLOR);
}
