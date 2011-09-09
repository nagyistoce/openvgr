/*
 ModelInformation.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <stdio.h>
#include <math.h>

#include "VGRModeler.h"
#include "ModelInformation.h"

//! 入力ファイルのコマンドを出力
void
writeCommandInfo(FILE* fp, const ModelParameter* commandParameter,
                 const RotationAngle* rotation, const Vector* moveBuff)
{
  fprintf(fp, "#モデル設定入力コマンド\n");
  if (commandParameter->label[0] == 'C')
    {
      fprintf(fp, "#%s %.3lf,%.3lf\n", 
              commandParameter->label, commandParameter->r, commandParameter->height);
    }
  else if (commandParameter->label[0] == 'B')
    {
      fprintf(fp, "#%s %.3lf,%.3lf,%.3lf\n", 
              commandParameter->label, commandParameter->x, commandParameter->y, commandParameter->z);
    }
  else
    {
      /* nothing to do */
    }

  if ((fabs(rotation->rx) > EPS) ||
      (fabs(rotation->ry) > EPS) ||
      (fabs(rotation->rz) > EPS))
    {
      fprintf(fp, "#R %.3lf,%.3lf,%.3lf\n", rotation->rx, rotation->ry, rotation->rz);
    }
  if ((fabs(moveBuff->x) > EPS) ||
      (fabs(moveBuff->y) > EPS) ||
      (fabs(moveBuff->z) > EPS))
    {
      fprintf(fp, "#T %.3lf,%.3lf,%.3lf\n", moveBuff->x, moveBuff->y, moveBuff->z);
    }
  fprintf(fp, "\n");
  return;
}

//! 重心情報を出力
void
writeGravityInfo(FILE* fp, const Vector* gravity)
{
  fprintf(fp, "#RTVCML GRAVITY %.3lf %.3lf %.3lf\n", gravity->x, gravity->y, gravity->z);
  return;
}

//! 円筒の情報を出力
void
writeCylinderlInfo(FILE* fp, const ModelParameter* circleParameter,
		   const LinePoint* centerPoint,      // 円の中心座標
		   const Vector* normalVector)        // 法線ベクトル
{
  fprintf(fp, "#RTVCML CIRCLE %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
          circleParameter->r, centerPoint->x, centerPoint->y, centerPoint->z,
          normalVector->x, normalVector->y, normalVector->z);
  return;
}

//! 直方体の頂点情報を出力
void
writeVertex(FILE* fp, const SquareParameter* faceParameter, const int nSquare)
{
  const LinePoint* vertex;
  int squareCount = 0, vertexCount = 0;

  for (squareCount = 0; squareCount < nSquare; squareCount++)
    {                           // 6面回す
      vertex = faceParameter[squareCount].squarePoint;
      for (vertexCount = 0; vertexCount < 2; vertexCount++)
        {                       // 各面の2頂点分回す
          fprintf(fp, "#RTVCML VERTEX %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
                  vertex[vertexCount + 2].x, vertex[vertexCount + 2].y, vertex[vertexCount + 2].z,
                  vertex[vertexCount + 1].x, vertex[vertexCount + 1].y, vertex[vertexCount + 1].z,
                  vertex[vertexCount].x, vertex[vertexCount].y, vertex[vertexCount].z);
        }
      fprintf(fp, "#RTVCML VERTEX %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
              vertex[0].x, vertex[0].y, vertex[0].z,
              vertex[3].x, vertex[3].y, vertex[3].z,
              vertex[2].x, vertex[2].y, vertex[2].z);
      fprintf(fp, "#RTVCML VERTEX %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
              vertex[1].x, vertex[1].y, vertex[1].z,
              vertex[0].x, vertex[0].y, vertex[0].z,
              vertex[3].x, vertex[3].y, vertex[3].z);
    }
  fprintf(fp, "\n");
  return;
}
