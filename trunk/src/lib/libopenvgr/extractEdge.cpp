/*
 extractEdge.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file extractEdge.cpp
 * @brief エッジ抽出関連関数
 * @date \$Date::                            $
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <cv.h>
#include <highgui.h>
#include <cxtypes.h>
#include <cxcore.h>

#include "parameters.h"
#include "extractEdge.h"

// このプログラム内での８方位の方向の定義
// column の＋方向は右、row の＋方向は下
// 0: 左下 col- row+ 方向
// 1: 下   col0 row+ 方向
// 2: 右下 col+ row+ 方向
// 3: 右   col+ row0 方向
// 4: 左上 col+ row- 方向
// 5: 上   col0 row- 方向
// 6: 左上 col- row- 方向
// 7: 左   col- row0 方向

// 移動方向を与えると、その差を与える配列
// 剰余系を使用しやすくするために、２周分(0,1,,,15)を定義する

static const int dv[16][2] = {
  {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0},
  {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}
};


// 与えられた画像が形式上正常かどうか確認する
template <typename T>
static int
imageSanity(T* image, const int colsize, const int rowsize)
{
  if (image == NULL)
    {
      return -1;                // 異常終了
    }
  if (colsize <= 0)
    {
      return -1;                // 異常終了
    }
  if (rowsize <= 0)
    {
      return -1;                // 異常終了
    }
  return 0;                     // 正常
}

// 画像のまわりの一定数の幅について、画像の内側からコピーする
template <typename T>
static int
roundFill(T* image, const int colsize, const int rowsize, const int width = 1)
{
  if (imageSanity(image, colsize, rowsize) < 0)
    {
      return -1;                // 異常終了
    }
  if ((width < 1) || (width * 2 + 1 >= colsize) || (width * 2 + 1 >= rowsize))
    {
      return -1;                // 異常終了
    }

  int col, row;
  // 上辺
  for (col = width; col < colsize - width; col++)
    {
      const T val = image[width * colsize + col];
      for (row = 0; row < width; row++)
        {
          image[row * colsize + col] = val;
        }
    }

  // 下辺
  for (col = width; col < colsize - width; col++)
    {
      const T val = image[(rowsize - width - 1) * colsize + col];
      for (row = rowsize - width; row < rowsize; row++)
        {
          image[row * colsize + col] = val;
        }
    }

  // 左辺
  for (row = 0; row < rowsize; row++)
    {
      const T val = image[row * colsize + width];
      for (col = 0; col < width; col++)
        {
          image[row * colsize + col] = val;
        }
    }

  // 右辺
  for (row = 0; row < rowsize; row++)
    {
      const T val = image[row * colsize + (colsize - width - 1)];
      for (col = colsize - width; col < colsize; col++)
        {
          image[row * colsize + col] = val;
        }
    }

  return 0;
}

#define Gray(col, row) (gray[(row)*colsize+(col)])

// 垂直方向の画像微分 (Sobel5x5)
// col がプラス方向の画素と比較して、明るくなる場合は＋の値をとる
static int
diffHorizontal5(int* edge, unsigned char* gray, const int colsize,
		const int rowsize)
{
  int row, col, n1;
  for (row = 2; row < rowsize - 2; row++)
    {
      n1 = row * colsize;
      for (col = 2; col < colsize - 2; col++)
        {
          edge[n1 + col] =
            (Gray(col + 2, row - 2) + Gray(col + 1, row - 2) * 2
             - Gray(col - 1, row - 2) * 2 - Gray(col - 2, row - 2))
            + (Gray(col + 2, row - 1) + Gray(col + 1, row - 1) * 2
               - Gray(col - 1, row - 1) * 2 - Gray(col - 2, row - 1)) * 4
            + (Gray(col + 2, row) + Gray(col + 1, row) * 2
               - Gray(col - 1, row) * 2 - Gray(col - 2, row)) * 6
            + (Gray(col + 2, row + 1) + Gray(col + 1, row + 1) * 2
               - Gray(col - 1, row + 1) * 2 - Gray(col - 2, row + 1)) * 4
            + (Gray(col + 2, row + 2) + Gray(col + 1, row + 2) * 2
               - Gray(col - 1, row + 2) * 2 - Gray(col - 2, row + 2));
        }
    }
  roundFill(edge, colsize, rowsize, 2);
  return 0;
}

// 水平方向の画像微分 (Sobel5x5)
// row がプラス方向の画素と比較して、明るくなる場合は＋の値をとる
static int
diffVertical5(int* edge, unsigned char* gray, const int colsize,
	      const int rowsize)
{
  int row, col, n1;
  for (row = 2; row < rowsize - 2; row++)
    {
      n1 = row * colsize;
      for (col = 2; col < colsize - 2; col++)
        {
          edge[n1 + col] =
            (Gray(col - 2, row + 2) + Gray(col - 2, row + 1) * 2
             - Gray(col - 2, row - 1) * 2 - Gray(col - 2, row - 2))
            + (Gray(col - 1, row + 2) + Gray(col - 1, row + 1) * 2
               - Gray(col - 1, row - 1) * 2 - Gray(col - 1, row - 2)) * 4
            + (Gray(col, row + 2) + Gray(col, row + 1) * 2
               -  Gray(col, row - 1) * 2 - Gray(col, row - 2)) * 6
            + (Gray(col + 1, row + 2) + Gray(col + 1, row + 1) * 2
               - Gray(col + 1, row - 1) * 2 - Gray(col + 1, row - 2)) * 4
            + (Gray(col + 2, row + 2) + Gray(col + 2, row + 1) * 2
               - Gray(col + 2, row - 1) * 2 - Gray(col + 2, row - 2));
        }
    }
  roundFill(edge, colsize, rowsize, 2);
  return 0;
}

// 垂直方向の画像微分 (Sobel3x3)
// col がプラス方向の画素と比較して、明るくなる場合は＋の値をとる
static int
diffHorizontal3(int* edge, unsigned char* gray, const int colsize,
		const int rowsize)
{
  int row, col, n1;
  for (row = 1; row < rowsize - 1; row++)
    {
      n1 = row * colsize;
      for (col = 1; col < colsize - 1; col++)
        {
          edge[n1 + col] =
            gray[(row - 1) * colsize + (col + 1)] - gray[(row - 1) * colsize + (col - 1)]
            + (gray[(row) * colsize + (col + 1)] - gray[(row) * colsize + (col - 1)]) * 2
            + gray[(row + 1) * colsize + (col + 1)] - gray[(row + 1) * colsize + (col - 1)];
        }
    }
  roundFill(edge, colsize, rowsize, 1);
  return 0;
}

// 水平方向の画像微分 (Sobel3x3)
// row がプラス方向の画素と比較して、明るくなる場合は＋の値をとる
static int
diffVertical3(int* edge, unsigned char* gray, const int colsize,
	      const int rowsize)
{
  int row, col, n1;
  for (row = 1; row < rowsize - 1; row++)
    {
      n1 = row * colsize;
      for (col = 1; col < colsize - 1; col++)
        {
          edge[n1 + col] =
            gray[(row + 1) * colsize + (col - 1)] - gray[(row - 1) * colsize + (col - 1)]
            + (gray[(row + 1) * colsize + (col)] - gray[(row - 1) * colsize + (col)]) * 2
            + gray[(row + 1) * colsize + (col + 1)] - gray[(row - 1) * colsize + (col + 1)];
        }
    }
  roundFill(edge, colsize, rowsize, 1);
  return 0;
}

#undef Gray                     // 一時的にのみ使用

// 輝度勾配で暗い方向から明るい方向に向いた方位を求める
// エッジの方向は直交する方向になるので注意
static int
edgeDirection(const double horizontal, const double vertical)
{
  const double T22 = tan(22.5 * M_PI / 180.0); // ８方位に分割した時の縦横比
  const double T67 = tan(67.5 * M_PI / 180.0); // ８方位に分割した時の縦横比

  if (horizontal > 0)
    {
      const double h1 = horizontal * T22;
      const double h2 = horizontal * T67;
      if (vertical > h2)
        {
          return 1;
        }
      if (vertical > h1)
        {
          return 2;
        }
      if (vertical < -h2)
        {
          return 5;
        }
      if (vertical < -h1)
        {
          return 4;
        }
      return 3;
    }
  else
    {
      const double h1 = -horizontal * T22;
      const double h2 = -horizontal * T67;
      if (vertical > h2)
        {
          return 1;
        }
      if (vertical > h1)
        {
          return 0;
        }
      if (vertical < -h2)
        {
          return 5;
        }
      if (vertical < -h1)
        {
          return 6;
        }
      return 7;                 // (0.0, 0.0) の場合は本来は未定だが、７を返すので注意
    }
}

// エッジの強度と方向を計算する
// eStrength2 は強度の２乗を返す
static int
detectEdge(unsigned int* eStrength2, unsigned char* eDirection,
	   unsigned char* gray, Parameters parameters)
{
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int imgsize = parameters.imgsize;
  int col, row, n1, n2;

  int* diffVertical = NULL;
  int* diffHorizontal = NULL;

  if ((diffVertical = (int*) calloc(imgsize, sizeof(int))) == NULL)
    {
      goto failed_exit;
    }
  if ((diffHorizontal = (int*) calloc(imgsize, sizeof(int))) == NULL)
    {
      goto failed_exit;
    }

  switch (parameters.feature2D.edgeDetectFunction)
    {
    case 0:                    // Sobel 3x3
      diffHorizontal3(diffHorizontal, gray, colsize, rowsize);
      diffVertical3(diffVertical, gray, colsize, rowsize);
      break;

    case 1:                    // Sobel 5x5
      diffHorizontal5(diffHorizontal, gray, colsize, rowsize);
      diffVertical5(diffVertical, gray, colsize, rowsize);
      break;

    default:
      goto failed_exit;
    }

  for (row = 1; row < rowsize - 1; row++)
    {
      n1 = row * colsize;
      for (col = 1; col < colsize - 1; col++)
        {
          n2 = n1 + col;
          eStrength2[n2] = (unsigned int) (pow((double) diffVertical[n2], 2) +
					   pow((double) diffHorizontal[n2], 2));
          eDirection[n2] = edgeDirection(diffHorizontal[n2], diffVertical[n2]);
        }
    }

  free(diffVertical);
  free(diffHorizontal);
  return 0;

failed_exit:
  free(diffVertical);
  free(diffHorizontal);
  return -1;
}

// 方向の比較
// 隣接する方向までを同一方向とみなす
static int
cmpDirection(const int d1, const int d2)
{
  const int diff = abs(d1 - d2);
  if ((diff <= 1) || (diff >= 7))
    {
      return 1;
    }
  return 0;
}

// 非極大細線化
// 各点について、そのエッジ強度を隣接する点と比較してみて、極大になっている場合のみを残す
// これによって、エッジの細線化が行われる
static int
thinEdge(unsigned char* thin, unsigned char* gray, Parameters parameters)
{
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int imgsize = parameters.imgsize;
  int col, row, n1, n2, n3;

  const unsigned int minThreshold2 = (unsigned int) pow(parameters.feature2D.edgeStrength, 2);

  unsigned int* eStrength2 = NULL;
  unsigned char* eDirection = NULL;

  // 各点のエッジ強度の二乗
  if ((eStrength2 = (unsigned int*) calloc(imgsize, sizeof(unsigned int))) == NULL)
    {
      goto failed_exit;
    }

  // 各点のエッジの方向
  if ((eDirection = (unsigned char*) calloc(imgsize, sizeof(unsigned char))) == NULL)
    {
      goto failed_exit;
    }

  if (detectEdge(eStrength2, eDirection, gray, parameters))
    {
      goto failed_exit;
    }

  for (row = 1; row < rowsize - 1; row++)
    {
      n1 = row * colsize;
      for (col = 1; col < colsize - 1; col++)
        {
          n2 = n1 + col;
          const unsigned int center = eStrength2[n2];
          const int direction = eDirection[n2];
          if (center < minThreshold2)
            {
              thin[n2] = EEnotEdge;     // エッジ点がない
            }
          else
            {
              thin[n2] = EEcandidate;   // エッジ点の候補
              switch (eDirection[n2] % 4)
                {
                case 2:
                  {    // 右上に伸びるエッジ (斜め col+, row+ 方向に変化が大きい
                    n3 = (row - 1) * colsize + (col - 1);
                    if (cmpDirection(eDirection[n3], direction))
                      {         // 同じ方向のエッジなら比較する
                        if (eStrength2[n3] > center)
                          {
                            thin[n2] = EEerasedThin;    // 近傍極大ではない
                          }
                      }
                    n3 = (row + 1) * colsize + (col + 1);
                    if (cmpDirection(eDirection[n3], direction))
                      {         // 同じ方向のエッジなら比較する
                        if (eStrength2[n3] >= center)
                          {
                            thin[n2] = EEerasedThin;    // 近傍極大ではない
                          }
                      }
                    break;
                  }
                case 1:
                  {             // 横に伸びるエッジ (縦 row 方向に変化が大きい)
                    n3 = (row - 1) * colsize + (col);
                    if (cmpDirection(eDirection[n3], direction))
                      {         // 同じ方向のエッジなら比較する
                        if (eStrength2[n3] > center)
                          {
                            thin[n2] = EEerasedThin;    // 近傍極大ではない
                          }
                      }
                    n3 = (row + 1) * colsize + (col);
                    if (cmpDirection(eDirection[n3], direction))
                      {         // 同じ方向のエッジなら比較する
                        if (eStrength2[n3] >= center)
                          {
                            thin[n2] = EEerasedThin;    // 近傍極大ではない
                          }
                      }
                    break;
                  }
                case 0:
                  {    // 右下に伸びるエッジ  (斜め col+, row- 方向に変化が大きい
                    n3 = (row - 1) * colsize + (col + 1);
                    if (cmpDirection(eDirection[n3], direction))
                      {         // 同じ方向のエッジなら比較する
                        if (eStrength2[n3] > center)
                          {
                            thin[n2] = EEerasedThin;    // 近傍極大ではない
                          }
                      }
                    n3 = (row + 1) * colsize + (col - 1);
                    if (cmpDirection(eDirection[n3], direction))
                      {         // 同じ方向のエッジなら比較する
                        if (eStrength2[n3] >= center)
                          {
                            thin[n2] = EEerasedThin;    // 近傍極大ではない
                          }
                      }
                    break;
                  }
                case 3:
                  {             // 縦にのびるエッジ (横 col 方向に変化が大きい）
                    n3 = (row) * colsize + (col - 1);
                    if (cmpDirection(eDirection[n3], direction))
                      {         // 同じ方向のエッジなら比較する
                        if (eStrength2[n3] > center)
                          {
                            thin[n2] = EEerasedThin;    // 近傍極大ではない
                          }
                      }
                    n3 = (row) * colsize + (col + 1);
                    if (cmpDirection(eDirection[n3], direction))
                      {         // 同じ方向のエッジなら比較する
                        if (eStrength2[n3] >= center)
                          {
                            thin[n2] = EEerasedThin;    // 近傍極大ではない
                          }
                      }
                    break;
                  }
                default:
                  /* nothing to do */;
                }
            }
        }
    }

  // 続く処理のために画像の端はエッジでないとマークする
  // 上辺
  for (col = 1; col < colsize - 1; col++)
    {
      thin[col] = EEnotEdge;
    }
  // 下辺
  for (col = 1; col < colsize - 1; col++)
    {
      thin[(rowsize - 1) * colsize + col] = EEnotEdge;
    }
  // 左辺
  for (row = 0; row < rowsize; row++)
    {
      thin[row * colsize] = EEnotEdge;
    }
  // 右辺
  for (row = 0; row < rowsize; row++)
    {
      thin[row * colsize + colsize - 1] = EEnotEdge;
    }
  free(eStrength2);
  free(eDirection);
  return 0;

failed_exit:
  free(eStrength2);
  free(eDirection);
  return -1;
}


#define Edge(col, row) (edge[(row)*colsize+(col)])

// 8連結境界線追跡を行ってその外周長等を求める
static int
trackTest(unsigned char* edge, const int colsize, const int icol, const int irow)
{
  int d;
  int n = 0;

  // 初期位置の点のまわりを探してみる(4連結)
  for (d = 0; d < 8; d++)
    {
      if (Edge(icol + dv[d][0], irow + dv[d][1]) >= 2)
        {
          break;
        }
    }
  if (d == 8)
    {                           // まわりに点がなかった
      return 1;                 // 孤立点なので以降の処理は行わない
    }

  // 探索を開始する点
  int col = icol;
  int row = irow;

  // 最初に移動する方向を記録
  const int idir = d;

  do
    { // 同じ点に同じ向きから戻ってくるまで境界線を追跡する
      // 次に移動する方向を覚えておく
      const int prevdir = d;

      // 指示された方向に (row, col) を移動する
      col += dv[d][0];
      row += dv[d][1];

      // 次に移動する方向を、探す。初期値を設定
      const int sd = (prevdir >= 2) ? prevdir - 2 : prevdir + 6;
      for (d = sd; d < sd + 7; d++)
        {
          if (Edge(col + dv[d][0], row + dv[d][1]) >= 2)
            {
              break;
            }
        }
      if (d >= 8)
        {
          d -= 8;               // ８の剰余とする
        }
      n++;
    }
  while ((row != irow) || (col != icol) || (d != idir));
  return n;
}

static const int markTable[8][8] = {
  {1, 1, 1, 1, 1, 0, 0, 0},
  {1, 1, 1, 1, 1, 1, 0, 0},
  {1, 1, 1, 1, 1, 1, 1, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {1, 0, 0, 0, 0, 0, 0, 0},
  {1, 1, 0, 0, 0, 0, 0, 0},
  {1, 1, 1, 1, 0, 0, 0, 0},
  {1, 1, 1, 1, 1, 0, 0, 0}
};

// 8連結境界線追跡し、追跡済のマークをつける
static void
trackSet(unsigned char* edge,
	 const int colsize, const int icol, const int irow, unsigned char mark)
{
  int d;

  // 初期位置の点のまわりを探してみる(4連結)
  for (d = 0; d < 8; d++)
    {
      if (Edge(icol + dv[d][0], irow + dv[d][1]) >= 2)
        {
          break;
        }
    }
  if (d == 8)
    {                           // まわりに点がなかった
      Edge(icol, irow) = mark;  // マークのみ実行
      return;                   // 孤立点なので以降の処理は行わない
    }

  // 探索を開始する点
  int col = icol;
  int row = irow;

  // 最初に移動する方向を記録
  const int idir = d;

  do
    { // 同じ点に同じ向きから戻ってくるまで境界線を追跡する

      // 次に移動する方向を覚えておく
      const int prevdir = d;

      // 指示された方向に (row, col) を移動する
      col += dv[d][0];
      row += dv[d][1];

      // 次に移動する方向を、探す。初期値を設定
      const int sd = (prevdir >= 2) ? prevdir - 2 : prevdir + 6;
      for (d = sd; d < sd + 7; d++)
        {
          if (Edge(col + dv[d][0], row + dv[d][1]) >= 2)
            {
              break;
            }
        }

      if (d >= 8)
        {
          d -= 8;               // ８の剰余とする
        }

      // dの方向によってマークするかどうかが異なるので注意
      if (markTable[prevdir][d] == 1)
        {
          Edge(col, row) = mark;       // mark
        }
    }
  while ((row != irow) || (col != icol) || (d != idir));
}

// 外周の点数が指定した値より小さなフラグメントをマークする
static void
markSmallFragment(unsigned char* edge, Parameters parameters)
{
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int col, row;

  for (row = 1; row < rowsize - 1; row++)
    {
      int left = EEnotEdge;     // エッジ点が見つかっていない状態
      for (col = 1; col < colsize - 1; col++)
        {
          // 開始点を探す
          if (left == EEnotEdge)
            {                   // 左隣は境界線外となっている
              if (Edge(col, row) >= EEsearchedSmall)
                {               // ここから既知の境界線内となる
                  left = Edge(col, row);
                }
              else if (Edge(col, row) == EEnotSearched)
                {               // 未知の境界線がみつかった
                  const int leftupper = Edge(col - 1, row - 1);
                  if (leftupper < EEnotSearched)
                    {           // これは外周境界
                      if (trackTest(edge, colsize, col, row) > parameters.feature2D.minFragment)
                        {
                          trackSet(edge, colsize, col, row, EEsearchedLarge);
                          left = EEsearchedLarge;
                        }
                      else
                        {
                          trackSet(edge, colsize, col, row, EEsearchedSmall);
                          left = EEsearchedSmall;
                        }
                    }
                  else
                    {           // これは内周境界
                      trackSet(edge, colsize, col, row, leftupper);
                      left = leftupper;
                    }
                }
            }
          else
            {                   // 左隣は境界線内である
              if (Edge(col, row) < EEnotSearched)
                {               // ここは境界線の右端
                  left = EEnotEdge;
                }
              else
                {               // 境界線内のマークがついていない
                  Edge(col, row) = left;       // 境界線内のマークをつける
                }
            }
        }
    }
  return;
}

// まわりの点の接続数を調べる。また１点のみの場合はその方向を返す
static int
connectionAround(int* direction, unsigned char* edge,
                 const int colsize, const int rowsize,
                 const int col, const int row)
{
  *direction = -1;

  int n = 0;
  int d;
  for (d = 0; d < 8; d++)
    {
      if (col + dv[d][0] < 0 || col + dv[d][0] > colsize - 1
          || row + dv[d][1] < 0 || row + dv[d][1] > rowsize - 1)
        {
          continue;
        }
      if (Edge(col + dv[d][0], row + dv[d][1]) >= EEsearchedLarge)
        {
          n++;
          *direction = d;
        }
    }
  return n;
}

// まわりの点の接続をグループ単位で調べる。
static int
connectionGroup(int* group, unsigned char* edge,
		const int colsize, const int rowsize,
		const int col, const int row)
{
  int n = 0;
  int g = 0;
  int d;
  for (d = 0; d < 8; d++)
    {
      if (col + dv[d][0] < 0 || col + dv[d][0] > colsize - 1
          || row + dv[d][1] < 0 || row + dv[d][1] > rowsize - 1)
        {
          continue;
        }
      if (Edge(col + dv[d][0], row + dv[d][1]) >= EEsearchedLarge)
        {
          n++;
          if (Edge(col + dv[d + 1][0], row + dv[d + 1][1]) < EEsearchedLarge)
            {                   // 次の点が違ったら
              g++;
            }
        }
    }
  *group = g;
  return n;
}

// 指定された方法に延長した場合の連結数を数えて、２以上であれば延長する
static int
extendTrial(const int direction, unsigned char* edge,
	    const int colsize, const int rowsize,
	    const int col, const int row)
{
  const int ncol = col + dv[direction][0];
  const int nrow = row + dv[direction][1];

  int ndirection;
  if (connectionAround(&ndirection, edge, colsize, rowsize, ncol, nrow) >= 2)
    {
      Edge(ncol, nrow) = EEextended;
      return 1;                 // 延長できた
    }
  else
    {
      return 0;                 // 延長できない
    }
}

// 端点を探して延長できないか検討する
static void
extendEdge(unsigned char* edge, Parameters parameters)
{
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int col, row;

  for (row = 1; row < rowsize - 1; row++)
    {
      for (col = 1; col < colsize - 1; col++)
        {
          if (Edge(col, row) == EEsearchedLarge)
            {
              int dir;
              if (connectionAround(&dir, edge, colsize, rowsize, col, row) == 1)
                {               // 端点がみつかった
                  // 反対の３方向にむかって延長できないか検討する
                  if (extendTrial((dir + 4) % 8, edge, colsize, rowsize, col, row) == 1)
                    {
                      continue; // 延長できた
                    }
                  if (extendTrial((dir + 3) % 8, edge, colsize, rowsize, col, row) == 1)
                    {
                      continue; // 延長できた
                    }
                  if (extendTrial((dir + 5) % 8, edge, colsize, rowsize, col, row) == 1)
                    {
                      continue; // 延長できた
                    }
                }
            }
        }
    }
  return;
}

// 一点のみ飛び出しているような部分を消去する
static void
deleteLid(unsigned char* edge, Parameters parameters)
{
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int col, row;

  for (row = 1; row < rowsize - 1; row++)
    {
      for (col = 1; col < colsize - 1; col++)
        {
          if (Edge(col, row) >= EEsearchedLarge)
            {                   // 大きな境界線以上が対象
              int group;
              if (connectionGroup(&group, edge, colsize, rowsize, col, row) >= 2)
                {               // 該当する点がみつかった
                  if (group == 1)
                    {           // 接続している「かたまり」はひとつだけ
                      Edge(col, row) = EEerasedThin;   // 細線化で消去されたのと同じ扱いとする
                    }
                }
            }
        }
    }
  return;
}

// エッジ点を検出するプログラム
int
extractEdge(unsigned char* edge,       // エッジ画像
            unsigned char* gray,       // 原画像
            const int threshold,       // エッジ閾値
            Parameters parameters)     // 全パラメータ
{
  int i;
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int imgsize = parameters.imgsize;

  // 非極大細線化
  if (thinEdge(edge, gray, parameters))
    {
      return -1; // メモリ確保失敗
    }

  // 小断片の除去
  markSmallFragment(edge, parameters);
  // 端点の延長
  extendEdge(edge, parameters);
  // 鍋蓋形状の除去
  deleteLid(edge, parameters);

  // エッジの閾値処理
  for (i = 0; i < imgsize; i++)
    {
      edge[i] = (edge[i] >= threshold) ? 1 : 0;
    }

  // 画像の端をゼロリセット
  // 上辺
  for (i = 1; i < colsize - 1; i++)
    {
      edge[i] = 0;
    }
  // 下辺
  for (i = 1; i < colsize - 1; i++)
    {
      edge[(rowsize - 1) * colsize + i] = 0;
    }
  // 左辺
  for (i = 0; i < rowsize; i++)
    {
      edge[i * colsize] = 0;
    }
  // 右辺
  for (i = 0; i < rowsize; i++)
    {
      edge[i * colsize + colsize - 1] = 0;
    }
  return 0;
}

// エッジ点を検出するプログラム
void
extractEdge_new(unsigned char* edge,       // エッジ画像
                unsigned char* gray,       // 原画像
                const int threshold,       // エッジ閾値
                Parameters parameters)     // 全パラメータ
{
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;

  int i;

  cv::Mat gray_image(rowsize, colsize, CV_8UC1, gray);
  cv::Mat edge_image(rowsize, colsize, CV_8UC1, edge);

#if 0
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);

  cv::imshow("image", gray_image);
  cv::waitKey(-1);
#endif

  cv::Canny(gray_image, edge_image, 60, 30);
#if 0
  cv::imshow("image", edge_image);
  cv::waitKey(-1);  
#endif

  for (i = 0; i < rowsize; ++i)
    {
      int j;
      for (j = 0; j < colsize; ++j)
        {
          if (edge[i*colsize + j] != 0)
            {
              edge[i*colsize + j] = 1;
            }
        }
    }

  // 画像の端をゼロリセット
  // 上辺
  for (i = 1; i < colsize - 1; i++)
    {
      edge[i] = 0;
    }
  // 下辺
  for (i = 1; i < colsize - 1; i++)
    {
      edge[(rowsize - 1) * colsize + i] = 0;
    }
  // 左辺
  for (i = 0; i < rowsize; i++)
    {
      edge[i * colsize] = 0;
    }
  // 右辺
  for (i = 0; i < rowsize; i++)
    {
      edge[i * colsize + colsize - 1] = 0;
    }
  return;
}
