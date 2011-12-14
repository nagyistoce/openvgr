/*
 Model.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#ifndef MODEL_H
#define MODEL_H

//! 関数writeWireModelEndに使用する列挙体
//! ワイヤーフレームに色を付けている場合はUSE_COLORを選択
enum FrameColor
{
  NOT_USE_COLOR,
  USE_COLOR
};

//! 点列格納用構造体
typedef struct
{
  double x;
  double y;
  double z;
} LinePoint;

//! 入力されたモデルのパラメーター
typedef struct
{
  double x;
  double y;
  double z;
  double r;                     // 半径
  double height;
  char label[8];                // モデル種別
} ModelParameter;

//! 直方体の1面
typedef struct
{
  LinePoint squarePoint[4];
} SquareParameter;

typedef struct
{
  LinePoint pointNumber[360];   // 円筒のひとつの円の点列
} InsideCylinder;

//! モデルの回転角
typedef struct
{
  double rx;                    // 回転度数x
  double ry;                    // 回転度数y
  double rz;                    // 回転度数z
} RotationAngle;

//! ベクトル用
typedef struct
{
  double x;
  double y;
  double z;
} Vector;

typedef struct
{
  char* inFileName;
  char* outFileName;
} OptionParameter;

#endif // MODEL_H
