/*
 VGRModeler.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <stdio.h>
#include <unistd.h>

#include <math.h>

#include "VGRModeler.h"

//! getopt用設定
int
parseOption(int argc, char** argv, OptionParameter& startOption)
{
  int ch;

  while ((ch = getopt(argc, argv, "i:o:")) != -1)
    {
      switch (ch)
        {
        case 'i':              // 入力先頭ファイル名
          startOption.inFileName = optarg;
          break;
        case 'o':              // 出力ファイル名
          startOption.outFileName = optarg;
          break;
        default:
          printf("usage : VGRModeler -i [input filename] -o [output filename]\n");
          return -1;
        }
    }
  return 0;
}

//! 法線ベクトル計算用関数
int
calcNormal(const LinePoint* point1,
           const LinePoint* point2,
           const LinePoint* point3,
           Vector* normalVector)
{
  double vector1[3];
  double vector2[3];
  double cross[3];
  double length = 0;
  int index = 0;

  // ベクトル1 = p3 - p2
  vector1[0] = point3->x - point2->x;
  vector1[1] = point3->y - point2->y;
  vector1[2] = point3->z - point2->z;

  // ベクトル2 = p1 - p2
  vector2[0] = point1->x - point2->x;
  vector2[1] = point1->y - point2->y;
  vector2[2] = point1->z - point2->z;

  // 外積(v2 * v1)を計算
  for (index = 0; index < 3; index++)
    {
      cross[index] = (vector2[(index + 1) % 3] * vector1[(index + 2) % 3])
        - (vector2[(index + 2) % 3] * vector1[(index + 1) % 3]);
    }

  // 外積(v2 * v1)の長さを求める
  length = sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);

  // lengthが0の時は法線ベクトルは求められない
  if (fabs(length) < EPS)
    {
      return -1;
    }

  // 外積を長さで割って法線ベクトルnを求める
  normalVector->x = cross[0] / length;
  normalVector->y = cross[1] / length;
  normalVector->z = cross[2] / length;

  return 0;
}


//! 対面している平行な面を作成
void
parallelSquare(const LinePoint* originalSquare, LinePoint* copySquare,
               const ModelParameter* boxParameter,
               int axis) // 0 = x, 1 = y, 2 = z
{
  double delta[3] = { 0 };
  int squareCount = 0;
  int reverseCount = 3;

  switch (axis)
    {
    case 0:
      delta[0] = boxParameter->x;
      break;

    case 1:
      delta[1] = boxParameter->y;
      break;

    case 2:
      delta[2] = boxParameter->z;
      break;

    default:
      /* nothing to do */
      ;
    }

  for (squareCount = 0; squareCount < 4; squareCount++)
    {
      copySquare[reverseCount].x = originalSquare[squareCount].x + delta[0];
      copySquare[reverseCount].y = originalSquare[squareCount].y + delta[1];
      copySquare[reverseCount].z = originalSquare[squareCount].z + delta[2];
      reverseCount--;
    }
  return;
}

//! 6面体の各面の4座標をセットする関数
void
setBox(const ModelParameter* boxParameter, SquareParameter* faceParameter)
{
  double halfLength[3];

  halfLength[0] = boxParameter->x / 2;
  halfLength[1] = boxParameter->y / 2;
  halfLength[2] = boxParameter->z / 2;

  // z一定の面の4頂点の座標決定
  faceParameter[0].squarePoint[0].x = -halfLength[0];   // 左上の点
  faceParameter[0].squarePoint[0].y = halfLength[1];
  faceParameter[0].squarePoint[0].z = -halfLength[2];

  faceParameter[0].squarePoint[1].x = -halfLength[0];   // 左下の点
  faceParameter[0].squarePoint[1].y = -halfLength[1];
  faceParameter[0].squarePoint[1].z = -halfLength[2];

  faceParameter[0].squarePoint[2].x = halfLength[0];    // 右下の点
  faceParameter[0].squarePoint[2].y = -halfLength[1];
  faceParameter[0].squarePoint[2].z = -halfLength[2];

  faceParameter[0].squarePoint[3].x = halfLength[0];    // 右上の点
  faceParameter[0].squarePoint[3].y = halfLength[1];
  faceParameter[0].squarePoint[3].z = -halfLength[2];

  // z座標だけ増やした面の4頂点の座標決定
  parallelSquare(faceParameter[0].squarePoint, faceParameter[1].squarePoint,
                 boxParameter, 2);

  if (boxParameter->z > 0)
    {                           // 高さ0以上の場合
      // y一定の面の4頂点の座標決定
      faceParameter[2].squarePoint[0].x = halfLength[0];        // 右上の点
      faceParameter[2].squarePoint[0].y = -halfLength[1];
      faceParameter[2].squarePoint[0].z = halfLength[2];

      faceParameter[2].squarePoint[1].x = halfLength[0];        // 右下の点
      faceParameter[2].squarePoint[1].y = -halfLength[1];
      faceParameter[2].squarePoint[1].z = -halfLength[2];

      faceParameter[2].squarePoint[2].x = -halfLength[0];       // 左下の点
      faceParameter[2].squarePoint[2].y = -halfLength[1];
      faceParameter[2].squarePoint[2].z = -halfLength[2];

      faceParameter[2].squarePoint[3].x = -halfLength[0];       // 左上の点
      faceParameter[2].squarePoint[3].y = -halfLength[1];
      faceParameter[2].squarePoint[3].z = halfLength[2];

      // y座標だけ増やした面の4頂点座標決定
      parallelSquare(faceParameter[2].squarePoint, faceParameter[3].squarePoint,
                     boxParameter, 1);

      // x一定の面の4頂点の座標決定
      faceParameter[4].squarePoint[0].x = -halfLength[0];       // 左上の点
      faceParameter[4].squarePoint[0].y = halfLength[1];
      faceParameter[4].squarePoint[0].z = halfLength[2];

      faceParameter[4].squarePoint[1].x = -halfLength[0];       // 左下の点
      faceParameter[4].squarePoint[1].y = -halfLength[1];
      faceParameter[4].squarePoint[1].z = halfLength[2];

      faceParameter[4].squarePoint[2].x = -halfLength[0];       // 右下の点
      faceParameter[4].squarePoint[2].y = -halfLength[1];
      faceParameter[4].squarePoint[2].z = -halfLength[2];

      faceParameter[4].squarePoint[3].x = -halfLength[0];       // 右上の点
      faceParameter[4].squarePoint[3].y = halfLength[1];
      faceParameter[4].squarePoint[3].z = -halfLength[2];

      // x座標だけ増やした面の4頂点の座標決定
      parallelSquare(faceParameter[4].squarePoint, faceParameter[5].squarePoint,
                     boxParameter, 0);
    }
  return;
}

//! 点の移動
void
pointTranslation(LinePoint* movePoint, const Vector* moveBuff, const int maxTranslatePoint)
{
  int translateCount;

  for (translateCount = 0; translateCount < maxTranslatePoint; translateCount++)
    {
      movePoint[translateCount].x += moveBuff->x;
      movePoint[translateCount].y += moveBuff->y;
      movePoint[translateCount].z += moveBuff->z;
    }
}

//! 円の点列初期値作成
void
createCircle(LinePoint* circlePoints,
             const ModelParameter* circleParameter,
             const double radian,
              int flag)  // flag : 0 = 上の円, 1 = 下の円
{
  circlePoints->x = circleParameter->r * cos(radian);
  circlePoints->y = circleParameter->r * sin(radian);
  if (flag == 0)
    {
      circlePoints->z = circleParameter->height / 2;
    }
  else if (flag == 1)
    {
      circlePoints->z = -circleParameter->height / 2;
    }
  else
    {
      /* nothing to do */
    }
  return;
}

//! 円筒出力用関数
void
outCylinder(FILE* fp,
            const ModelParameter* cylinderParameter,
            const RotationAngle* rotation,
            const Vector* moveBuff,
            int step,
            int nCircle)        // 座標軸の長さ, 円の粗さ,中の円の数
{
  double radian = 0.0;          // ラジアン計算用
  int pointNumberCount = 0;     // 上の円の線を結ぶ点列のカウント
  int pointNumberCountNext = 0; // 底の円の線を結ぶ点列のカウント
  int arcPointCount = 0;        // 円を構成する点列のカウント
  int arcNumber = 0;            // 円の数カウント
  int startCount = 0;           // 円を線で結ぶ際のカウント初期値記憶用
  int insideCount = 0;          // 中の円を結ぶ際のindex
  int crossCheck = 0;           // 法線ベクトルを求められるかチェックフラグ
  int faceNumber = 0;           // 上下の面のカウント
  int nFace = 0;                // 面の数（1個か2個か）
  int angleCount = 0;           // 法線ベクトル算出用座標のカウント
  int jointNumber = 0;          // 何番の点まで結ぶか
  int maxPoint = 0;             // 最大頂点数
  LinePoint arcPoint[2][360];   // 上下の円の頂点[最大360度]
  InsideCylinder arcPointInside[36];    // 中の円

  // 法線ベクトル算出用座標
  LinePoint normalPoint[2][2];  // [上下][90度地点 or 180度地点]
  // 上下の円の法線ベクトル
  Vector normalVector[2] = { {0}, {0} };

  // 上下の円の中心座標
  LinePoint center[2];

  jointNumber = (360 / step) - 1;
  maxPoint = 360 / step;

  // 高さが0だった場合は1個の円だけを出力する。
  if (fabs(cylinderParameter->height) < EPS)
    {
      nFace = 1;
    }
  else
    {
      nFace = 2;
    }

  // 上下の円の座標初期値作成
  for (arcPointCount = 0; arcPointCount < maxPoint; arcPointCount++)
    {
      radian = step * arcPointCount * M_PI / 180.0;
      for (faceNumber = 0; faceNumber < nFace; faceNumber++)
        {
          createCircle(&arcPoint[faceNumber][arcPointCount],
                       cylinderParameter, radian, faceNumber);
        }
    }

  // 上下の円を回転させる
  for (arcPointCount = 0; arcPointCount < maxPoint; arcPointCount++)
    {                           // 頂点数分
      for (faceNumber = 0; faceNumber < nFace; faceNumber++)
        {
          translate_point(rotation,
                          cylinderParameter->x,
                          cylinderParameter->y,
                          cylinderParameter->z,
                          &arcPoint[faceNumber][arcPointCount].x,
                          &arcPoint[faceNumber][arcPointCount].y,
                          &arcPoint[faceNumber][arcPointCount].z);
        }
    }

  if (nFace == 2)
    {
      // 中の円の座標初期値作成
      for (arcNumber = 0; arcNumber < nCircle; arcNumber++)
        {
          for (arcPointCount = 0; arcPointCount < maxPoint; arcPointCount++)
            {
              radian = step * arcPointCount * M_PI / 180.0;
              createCircle(&arcPointInside[arcNumber].pointNumber[arcPointCount],
                           cylinderParameter,radian, 3);
              // y座標は別に入力
              arcPointInside[arcNumber].pointNumber[arcPointCount].z =
                (cylinderParameter->height / 2)
                - ((cylinderParameter->height / (1 + nCircle)) * (1 + arcNumber));
            }
        }
      // 中の円を回転させる
      for (arcNumber = 0; arcNumber < nCircle; arcNumber++)
        {                       // 円の数だけ
          for (arcPointCount = 0; arcPointCount < maxPoint; arcPointCount++)
            {                   // 頂点数分
              translate_point(rotation,
                              cylinderParameter->x,
                              cylinderParameter->y,
                              cylinderParameter->z,
                              &arcPointInside[arcNumber].pointNumber[arcPointCount].x,
                              &arcPointInside[arcNumber].pointNumber[arcPointCount].y,
                              &arcPointInside[arcNumber].pointNumber[arcPointCount].z);
            }
        }

      // 中の円を移動させる
      for (arcNumber = 0; arcNumber < nCircle; arcNumber++)
        {
          pointTranslation(arcPointInside[arcNumber].pointNumber,
                           moveBuff, maxPoint);
        }
    }

  // 法線計算用に、上下の円の90度地点と180度地点を作成
  for (angleCount = 0; angleCount < 2; angleCount++)
    {
      radian = (angleCount + 1) * 90 * M_PI / 180.0;
      for (faceNumber = 0; faceNumber < nFace; faceNumber++)
        {
          createCircle(&normalPoint[faceNumber][angleCount],
                       cylinderParameter, radian, faceNumber);
        }
    }

  // 法線計算用座標を回転
  for (angleCount = 0; angleCount < 2; angleCount++)
    {
      for (faceNumber = 0; faceNumber < nFace; faceNumber++)
        {
          translate_point(rotation,
                          cylinderParameter->x,
                          cylinderParameter->y,
                          cylinderParameter->z,
                          &normalPoint[faceNumber][angleCount].x,
                          &normalPoint[faceNumber][angleCount].y,
                          &normalPoint[faceNumber][angleCount].z);
        }
    }

  for (faceNumber = 0; faceNumber < nFace; faceNumber++)
    {
      // 法線計算用座標を移動させる
      pointTranslation(normalPoint[faceNumber], moveBuff, 2);
      // 上下の円を移動させる
      pointTranslation(arcPoint[faceNumber], moveBuff, maxPoint);
    }

  // 中心座標計算
  for (faceNumber = 0; faceNumber < nFace; faceNumber++)
    {
      center[faceNumber].x = (arcPoint[faceNumber][0].x + normalPoint[faceNumber][1].x) / 2;
      center[faceNumber].y = (arcPoint[faceNumber][0].y + normalPoint[faceNumber][1].y) / 2;
      center[faceNumber].z = (arcPoint[faceNumber][0].z + normalPoint[faceNumber][1].z) / 2;
    }

  // 法線ベクトル計算。円の頂点数に影響されないよう、0度点と専用に作成した90度及び180度の3点から算出
  for (faceNumber = 0; faceNumber < nFace; faceNumber++)
    {
      crossCheck += calcNormal(&arcPoint[faceNumber][0],
                               &normalPoint[faceNumber][1 - faceNumber],
                               &normalPoint[faceNumber][0 + faceNumber],
                               &normalVector[faceNumber]);
    }
  if (crossCheck < 0)
    {
      printf("ERROR : Cannot calc normal vector.\n");
    }

  // 重心情報
  writeGravityInfo(fp, moveBuff);

  // 半径、円の中心座標、法線ベクトル情報
  for (faceNumber = nFace - 1; faceNumber >= 0; faceNumber--)
    {
      writeCylinderlInfo(fp, cylinderParameter,
                         &center[faceNumber], &normalVector[faceNumber]);
    }
  fprintf(fp, "\n");

  // 入力ファイルのデータを出力
  writeCommandInfo(fp, cylinderParameter, rotation, moveBuff);

  // 法線を書き込む
  writeNormalModel(fp, center, normalVector, cylinderParameter->r, nFace);

  // 立体のVRMLモデルを書き込む
  fprintf(fp, "#立体モデル（円筒）\n");
  writeWireModelStart(fp);

  // 点の定義
  for (faceNumber = 0; faceNumber < nFace; faceNumber++)
    {                           // 上下の円
      for (arcPointCount = 0; arcPointCount < maxPoint; arcPointCount++)
        {
          pointDefine(fp, &arcPoint[faceNumber][arcPointCount]);
        }
      fprintf(fp, "\n");
    }

  if (nFace == 2)
    {
      for (arcNumber = 0; arcNumber < nCircle; arcNumber++)
        {                       // 中の円
          for (arcPointCount = 0; arcPointCount < maxPoint; arcPointCount++)
            {
              pointDefine(fp, &arcPointInside[arcNumber].pointNumber[arcPointCount]);
            }
          fprintf(fp, "\n");
        }
    }

  // 点の定義終了
  pointDefineEnd(fp);

  // 上の円を結ぶ
  startCount = 0;
  for (pointNumberCount = 0; pointNumberCount < jointNumber; pointNumberCount++)
    {
      fprintf(fp, "\t\t\t%d %d -1,\n",
              pointNumberCount, (pointNumberCount + 1));
    }
  fprintf(fp, "\t\t\t%d %d -1,\n",
          pointNumberCount, startCount);

  if (nFace == 2)
    {
      // 下の円を結ぶ
      startCount = pointNumberCount + 1;
      for (pointNumberCountNext = (pointNumberCount + 1);
           pointNumberCountNext < ((jointNumber * 2) + 1);
           pointNumberCountNext++)
        {
          fprintf(fp, "\t\t\t%d %d -1,\n",
                  pointNumberCountNext, (pointNumberCountNext + 1));
        }
      fprintf(fp, "\t\t\t%d %d -1,\n", pointNumberCountNext, startCount);

      // 中の円を結ぶ
      insideCount = pointNumberCountNext + 1;   // 続きから結ぶ
      for (arcNumber = 0; arcNumber < nCircle; arcNumber++)
        {
          startCount = insideCount;
          for (arcPointCount = 0; arcPointCount < jointNumber; arcPointCount++)
            {
              fprintf(fp, "\t\t\t%d %d -1,\n", insideCount, (insideCount + 1));
              insideCount++;
            }
          fprintf(fp, "\t\t\t%d %d -1,\n", insideCount, startCount);
          insideCount++;
        }
    }

  // 円筒出力終了
  writeWireModelEnd(fp, NOT_USE_COLOR);
  return;
}

//! 直方体出力用関数
void
outBox(FILE* fp, const ModelParameter* boxParameter,
       const RotationAngle* rotation, const Vector* moveBuff)
{
  SquareParameter faceParameter[6];     // 6面の各4座標
  int squareCount;                      // 面のカウント
  int connectPointNumber;               // 線を繋ぐ番号
  int vertexCount;                      // 頂点のカウント
  int nSquare;

  if (boxParameter->z > 0)
    {
      nSquare = 6;
    }
  else
    {
      nSquare = 1;
    }

  setBox(boxParameter, faceParameter); // xyzから6面体の頂点(6*4個)初期値を入れる

  for (squareCount = 0; squareCount < nSquare; squareCount++)
    {                           // n面を回す
      for (vertexCount = 0; vertexCount < 4; vertexCount++)
        {                       // 各面の4頂点を回して点を回転させる
          translate_point(rotation, 0, 0, 0,
                          &faceParameter[squareCount].squarePoint[vertexCount].x,
                          &faceParameter[squareCount].squarePoint[vertexCount].y,
                          &faceParameter[squareCount].squarePoint[vertexCount].z);
        }
    }

  // 点の移動
  for (squareCount = 0; squareCount < nSquare; squareCount++)
    {                           // n面
      pointTranslation(faceParameter[squareCount].squarePoint, moveBuff, 4);
    }

  // 重心情報を出力
  writeGravityInfo(fp, moveBuff);

  // 頂点情報を出力
  writeVertex(fp, faceParameter, nSquare);

  // 入力ファイルのデータを出力
  writeCommandInfo(fp, boxParameter, rotation, moveBuff);

  // 立体のVRMLモデルを書き込む
  fprintf(fp, "#立体モデル（直方体）\n");
  writeWireModelStart(fp);

  // 点の定義
  for (squareCount = 0; squareCount < nSquare; squareCount++)
    {                           // n面分
      for (vertexCount = 0; vertexCount < 4; vertexCount++)
        {                       // 4頂点
          pointDefine(fp, &faceParameter[squareCount].squarePoint[vertexCount]);
        }
    }

  // 点の定義終了
  pointDefineEnd(fp);

  // 定義した点を結ぶ
  for (squareCount = 0; squareCount < nSquare; squareCount++)
    {                           // 6面分
      for (vertexCount = 0; vertexCount < 3; vertexCount++)
        {                       // 各面の3頂点目までを回して点を結ぶ
          connectPointNumber = (squareCount * 4) + vertexCount;
          fprintf(fp, "\t\t\t%d %d -1,\n",
                  connectPointNumber, connectPointNumber + 1);
        }
      fprintf(fp, "\t\t\t%d %d -1,\n",
              connectPointNumber + 1, connectPointNumber - 2); // 4頂点目を結ぶ
    }

  // 直方体出力終了
  writeWireModelEnd(fp, NOT_USE_COLOR);
  return;
}

//! モデル指定のチェック
int
commandCheck(const int cylinderFlag,
             const int boxFlag,
             const ModelParameter* cylinderParameter,
             const ModelParameter* boxParameter)
{
  int checkFlag = 0;
  int errCount = 0;

  if ((cylinderFlag == 1) && (boxFlag == 1))
    {
      printf("COMMAND ERROR : ");
      printf("You described both of the cylinder and box.\n");
      return -1;
    }
  else if ((cylinderFlag == 0) && (boxFlag == 0))
    {
      printf("COMMAND ERROR : ");
      printf("You described neither the cylinder nor box.\n");
      return -1;
    }
  else if ((cylinderFlag == 1) && (boxFlag == 0))
    {
      if (cylinderParameter->r <= 0)
        {
          printf("COMMAND ERROR : ");
          printf("\"r\" parameter must be greater than zero.\n");
          errCount++;
          checkFlag = -1;
        }
      if (cylinderParameter->height < 0)
        {
          printf("COMMAND ERROR : ");
          printf
            ("\"height\" parameter must be greater than or equal zero.\n");
          errCount++;
          checkFlag = -1;
        }
      if (errCount == 0)
        {
          checkFlag = 0;
        }
    }
  else if ((cylinderFlag == 0) && (boxFlag == 1))
    {
      if ((boxParameter->x <= 0) || (boxParameter->y <= 0))
        {
          printf("COMMAND ERROR : ");
          printf("\"Width\" and \"Depth\" parameter"
                 " must be greater than zero.\n");
          errCount++;
          checkFlag = -1;
        }
      if (boxParameter->z < 0)
        {
          printf("COMMAND ERROR : ");
          printf("\"height\" parameter"
                 " must be greater than or equal zero.\n");
          errCount++;
          checkFlag = -1;
        }
      if (errCount == 0)
        {
          checkFlag = 1;
        }
    }
  else
    {
      /* nothing to do */
    }

  return checkFlag;
}

//! モデル作成関数
static int
createModel(FILE* fp, const int cylinderFlag, const int boxFlag,
            const ModelParameter* cylinderParameter,
            const ModelParameter* boxParameter,
            const RotationAngle* rotation, Vector* moveBuff)
{
  int commandFlag;

  // 設定ファイルの記述チェック
  commandFlag = commandCheck(cylinderFlag, boxFlag, cylinderParameter, boxParameter);
  if (commandFlag == -1)
    {
      printf("Please check the command file.\n");
    }
  else if (commandFlag == 0)
    {
      // fp, 円柱設定, 移動, 回転, 座標の線の長さ, 円の粗さ, 中の円の数
      outCylinder(fp, cylinderParameter, rotation, moveBuff, 10, 6);
      printf("The cylinder created.\n");
    }
  else if (commandFlag == 1)
    {
      outBox(fp, boxParameter, rotation, moveBuff);
      printf("The box created.\n");
    }
  else
    {
      /* nothing to do */
    }

  return commandFlag;
}

//! メイン関数
int
main(int argc, char* argv[])
{
  FILE* in_fp;
  FILE* out_fp;
  char* inFileName = NULL;
  char* output = NULL;
  char buff[256];
  char name[16];                // コマンド名
  int buffSize = sizeof(buff);
  double axisLength = 0;        // 座標軸の長さ
  OptionParameter startOption = { 0 };
  int errCheck = 0;

  // 円筒の入出力用
  ModelParameter cylinderParameter = { 0 };
  int cylinderFlag = 0;

  // 直方体の入出力用
  ModelParameter boxParameter;
  int boxFlag = 0;

  // 回転と移動用
  RotationAngle rotation = { 0 };
  RotationAngle rotationTemp = { 0 };
  Vector moveBuff = { 0 };
  Vector moveBuffTemp = { 0 };

  // コマンドラインオプション解析
  if (parseOption(argc, argv, startOption))
    {
      fprintf(stderr, "Invalid option used.\n");
      return -1;
    }

  if (startOption.inFileName)
    {
      inFileName = startOption.inFileName;
    }
  else
    {
      fprintf(stderr, "Input filename required.\n");
      return -1;
    }

  if (startOption.outFileName)
    {
      output = startOption.outFileName;
    }
  else
    {
      fprintf(stderr, "Output filename required.\n");
      return -1;
    }

  // 設定ファイル読み込み
  if ((in_fp = fopen(inFileName, "r")) == NULL)
    {
      printf("Cannot open input file.\n");
      return 0;
    }
  if ((out_fp = fopen(output, "w")) == NULL)
    {
      fclose(in_fp);

      printf("Cannot open output file.\n");
      return 0;
    }

  // VRMLのヘッダ情報を書き込む
  writeVRML_Header(out_fp);

  while ((fgets(buff, (buffSize - 1), in_fp)) && (errCheck != -1))
    {
      if ((buff[0] == '#') || (buff[0] == '\n') || 
          (buff[0] == '\r') || (buff[0] == '\0'))
        {
          continue;
        }
      else if (buff[0] == 'C')
        {                       // 円柱
          if ((cylinderFlag == 1) || (boxFlag == 1))
            {
              errCheck = createModel(out_fp, cylinderFlag, boxFlag,
                                     &cylinderParameter, &boxParameter,
                                     &rotation, &moveBuff);
              memset(&cylinderParameter, 0, sizeof(ModelParameter));
              memset(&rotation, 0, sizeof(RotationAngle));
              memset(&moveBuff, 0, sizeof(Vector));
              cylinderParameter.height = -1;
              boxFlag = 0;
            }
          if (sscanf(buff, "%s %lf,%lf", 
                     cylinderParameter.label,
                     &cylinderParameter.r,
                     &cylinderParameter.height) < 3)
            {
              printf("The \"C\" member doesn't suffice.\n");
              errCheck = -1;
            }
          if (axisLength < cylinderParameter.r)
            {
              axisLength = cylinderParameter.r;
            }
          if (axisLength < cylinderParameter.height)
            {
              axisLength = cylinderParameter.height;
            }
          cylinderFlag = 1;
        }
      else if (buff[0] == 'B')
        {                       // 直方体
          if ((cylinderFlag == 1) || (boxFlag == 1))
            {
              errCheck = createModel(out_fp, cylinderFlag, boxFlag,
                                     &cylinderParameter, &boxParameter,
                                     &rotation, &moveBuff);
              memset(&boxParameter, 0, sizeof(ModelParameter));
              memset(&rotation, 0, sizeof(RotationAngle));
              memset(&moveBuff, 0, sizeof(Vector));
              boxParameter.z = -1;
              cylinderFlag = 0;
            }
          if (sscanf(buff, "%s %lf,%lf,%lf",
                     boxParameter.label,
                     &boxParameter.x,
                     &boxParameter.y,
                     &boxParameter.z) < 4)
            {
              printf("The \"B\" member doesn't suffice.\n");
              errCheck = -1;
            }
          if (axisLength < boxParameter.x)
            {
              axisLength = boxParameter.x;
            }
          if (axisLength < boxParameter.y)
            {
              axisLength = boxParameter.y;
            }
          if (axisLength < boxParameter.z)
            {
              axisLength = boxParameter.z;
            }
          boxFlag = 1;
        }
      else if (buff[0] == 'R')
        {                       // 回転コマンド
          if (sscanf(buff, "%s %lf,%lf,%lf", name, &rotationTemp.rx, &rotationTemp.ry, &rotationTemp.rz) < 4)
            {
              printf("The \"R\" member doesn't suffice.\n");
              errCheck = -1;
            }
          rotation.rx += rotationTemp.rx;
          rotation.ry += rotationTemp.ry;
          rotation.rz += rotationTemp.rz;
          memset(&rotationTemp, 0, sizeof(RotationAngle));
        }
      else if (buff[0] == 'T')
        {                       // 平行移動コマンド
          if (sscanf (buff, "%s %lf,%lf,%lf", name, &moveBuffTemp.x, &moveBuffTemp.y, &moveBuffTemp.z) < 4)
            {
              printf("The \"T\" member doesn't suffice.\n");
              errCheck = -1;
            }
          moveBuff.x += moveBuffTemp.x;
          moveBuff.y += moveBuffTemp.y;
          moveBuff.z += moveBuffTemp.z;
          memset(&moveBuffTemp, 0, sizeof(Vector));
        }
      else
        {
          errCheck = -1;
          printf("Invalid command exist.\n");
        }
    }
  fclose(in_fp);

  if (errCheck != -1)
    {
      errCheck = createModel(out_fp, cylinderFlag, boxFlag, &cylinderParameter,
                             &boxParameter, &rotation, &moveBuff);
      // 座標軸を書き込む
      writeAxisModel(out_fp, axisLength);
    }
  fclose(out_fp);

  if (errCheck == -1)
    {
      remove(output);
      printf("\"%s\" was not created.\n", output);
    }

  return 0;
}
