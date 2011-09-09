/*
 score2d.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file score2d.cpp
 * @brief 2次元評価関連関数
 * @date \$Date::                            $
 */
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <math.h>

#include "modelpoints.h"
#include "match3Dfeature.h"
#include "score2d.h"

/* ８方向の座標値生成テーブル */
#define COL_TABLE {1,  1,  0, -1, -1, -1, 0, 1}
#define ROW_TABLE {0, -1, -1, -1,  0,  1, 1, 1}

#define BOTTOMOFFSET      1
#define EDGE_SEARCH_MAX  36
#define EDGE_SEARCH_SIZE  EDGE_SEARCH_MAX
#define EDGE_SEARCH_2SIZE (((EDGE_SEARCH_SIZE)*2)+1+BOTTOMOFFSET)

// ピクセル値の取得
static int
getpixel(int col, int row, int p_camera, Features3D* model)
{
  int colsize = model->calib->colsize;
  int rowsize = model->calib->rowsize;
  if (row < 0 || row >= rowsize || col < 0 || col >= colsize)
    {
      return 0;
    }

  return model->image[p_camera][colsize * row + col];
}

// 認識結果ベクトルの比較
static int
comparePropertyVector(const double* vec1, const double* vec2)
{
  int i;
  for (i = 0; i < 7; i++)
    {
      if (fabs(vec2[i] - vec1[i]) < VISION_EPS)
        {
          continue;
        }
      else if (vec2[i] > vec1[i])
        {
          return 1;
        }
      else
        {
          return -1;
        }
    }
  return 0;
}

// 認識結果評価値の比較
// 戻り値：比較結果
int
compareResultScore(const void* c1,     // 評価値１
                   const void* c2)     // 評価値２
{
  const MatchResult* r1 = (const MatchResult*) c1;
  const MatchResult* r2 = (const MatchResult*) c2;

  if (r2->score == r1->score)
    {
      // 認識結果ベクトル値でもソートする
      return comparePropertyVector(r1->vec, r2->vec);
    }
  else if (r2->score > r1->score)
    {
      return 1;
    }
  else
    {
      return -1;
    }
}

// エッジ極大の取得
static int
getEdgePeakInfo(Trace* data,        // 検出結果
                int colbuff[],      // 座標
                int rowbuff[],      // 座標
                int sumbuff[],      // エッジ強度
                int p_search,       // 探索範囲
                int p_edge)         // エッジ強度しきい値
{
  int i;
  
  for (i = 3; i < p_search * 2 - 2; i++)
    {
      // エッジ強度極大かどうか調べる
      if ((((sumbuff[i - 1] <= sumbuff[i]) && (sumbuff[i] > sumbuff[i + 1]))
           || ((sumbuff[i - 1] < sumbuff[i]) && (sumbuff[i] >= sumbuff[i + 1])))
          && (sumbuff[i] >= p_edge))
        {
          // 結果をセットする
          // エッジの点位置を記録する
          data->peakcr[0] = colbuff[i];
          data->peakcr[1] = rowbuff[i];
          
          // エッジの探索位置と強度を記録する
          data->search = abs(i - p_search);
          data->edge = sumbuff[i];
          
          return 0;
        }
    }
  return -1;
}

// エッジ強度の取得
static int
getEdgeStrength(uchar imgbuff[],    // 画素値
                int sumbuff[],      // エッジ強度値
                int p_search)       // 探索範囲
{
  int edgbuff[EDGE_SEARCH_2SIZE] = { 0 };
  int i;
  
  // エッジ値を求める
  for (i = 1; i < p_search * 2; i++)
    {
      edgbuff[i] = abs (imgbuff[i - 1] - imgbuff[i + 1]);
    }
  
  // エッジ強度としてエッジ値の和を求める
  for (i = 2; i < p_search * 2 - 1; i++)
    {
      sumbuff[i] = edgbuff[i - 1] + edgbuff[i] + edgbuff[i + 1];
    }
  
  return 0;
}

// 直交する画素の取得
static int
getOrthogonalPixels(uchar imgbuff[],    // 画素値
                    int colbuff[],      // 座標値
                    int rowbuff[],      // 座標値
                    int col,            // 現在座標
                    int row,            // 現在座標
                    int azimuth,        // 探索方向
                    int p_search,       // 探索範囲
                    int p_camera,       // カメラ番号
                    Features3D* finfo)  // 3次元特徴
{
  static int Coltbl[8] = COL_TABLE;
  static int Rowtbl[8] = ROW_TABLE;
  
  int pcol = col;
  int prow = row;
  int dcol = Coltbl[azimuth];
  int drow = Rowtbl[azimuth];
  int i;

  for (i = 1; i < p_search * 2; i++)
    {
      int ii = i - p_search;
      // azimuth はモデル直線の進行方向を示す
      // モデル直線に直交する方向（進行方向左側）に移動した画素座標を求める
      pcol = col + dcol * ii;
      prow = row + drow * ii;
      if (isValidPixelPosition (pcol, prow, finfo))
        {
          // imgbuff には画素値を、colbuf, rowbuf には col, row 値を格納する
          imgbuff[p_search + ii] = getpixel (pcol, prow, p_camera, finfo);
          colbuff[p_search + ii] = pcol;
          rowbuff[p_search + ii] = prow;
        }
      else
        {
          // ピクセル位置が対応範囲外
          return -1;
        }
    }
  
  return 0;
}


// data->colrow[2] から data->direction で決まる方向に
// 画像データをトレースし、
// 最初に見つかったエッジピークを peakcr に格納する
// 戻り値：エラーコード
int
tracePoint(Features3D* finfo, // ３次元特徴情報
           Trace* data,       // トレース結果
           int p_search,      // 探索領域サイズ
           int p_edge,        // エッジ強度の閾値
           int p_camera)      // カメラ（画像）番号 (0|1|2)
{
  uchar imgbuff[EDGE_SEARCH_2SIZE] = { 0 };
  int colbuff[EDGE_SEARCH_2SIZE] = { 0 };
  int rowbuff[EDGE_SEARCH_2SIZE] = { 0 };
  int sumbuff[EDGE_SEARCH_2SIZE] = { 0 };

  //  p_search - 3 まで負方向の配列アクセスがあるので
  //  BOTTOMOFFSET = 1 で p_search >= 2 の配列負方向の
  //  深さを -1 まで保証する
  uchar *imgbp = imgbuff + BOTTOMOFFSET;
  int *colbp = colbuff + BOTTOMOFFSET;
  int *rowbp = rowbuff + BOTTOMOFFSET;
  int pcol, prow, pdir;
  int ret;
  
  // 探索範囲が正しいか確認
  if (p_search < 2 || p_search > EDGE_SEARCH_SIZE)
    {
      return -2;
    }
  
  pdir = data->direction;
  pcol = (int)floor((data->colrow)[0] + 0.5);
  prow = (int)floor((data->colrow)[1] + 0.5);
  
  // モデル点の方向が正しいか確認
  if (data->direction < 0 || data->direction >= 8)
    {
      return -2;
    }
  
  // モデル点の位置が正しいか確認
  if (isValidPixelPosition(pcol, prow, finfo) == 0)
    {
      return -2;
    }
  
  // モデル点(pcol, prow)から pdir 方向に直交する画素列の取り出し imgbuffへ格納する
  if ( (ret = getOrthogonalPixels(imgbp, colbp, rowbp, pcol, prow,
                                  pdir, p_search, p_camera, finfo) ) != 0)
    {
      return ret;
    }
  
  // 各点のエッジ強度を求め sumbuffへ格納する
  if ( (ret = getEdgeStrength(imgbp, sumbuff, p_search) ) != 0)
    {
      return ret;
    }
  
  // エッジ強度極大を求めて結果を格納する
  if ( (ret = getEdgePeakInfo(data, colbp, rowbp, sumbuff, p_search, p_edge) ) != 0)
    {
      return ret;
    }
  
  return 0;
}

// 結果の２次元評価値算出
#ifdef USE_DISTANCETRANSFORM
void
getResultScore(MatchResult* results,   // 認識結果情報
               int numOfResults,       // 認識結果数
               Features3D* model,      // モデルの３次元特徴情報
               StereoPairing& pairing, // ステレオ処理ペア情報
               const std::vector<cv::Mat>& dstImages,
               double weight)          // 評価値の重みづけ
{
  int i, j, num;
  int status;

  // 認識結果ベクトル（位置＋回転ベクトル）の値でソートする
  qsort(results, numOfResults, sizeof(MatchResult), compareResultScore);

  num = 0;
  // 完全に同じ位置姿勢の結果には評価不要の印をつける
  for (i = 0; i < numOfResults; i++)
    {
      // 既に不要となった結果はスキップ
      if (results[i].score == -1)
        {
          continue;
        }

      for (j = i + 1; j < numOfResults; j++)
        {
          // 既に不要となった結果はスキップ
          if (results[j].score == -1)
            {
              continue;
            }
          // 認識結果が全く同じ場合は評価値に -1 を入れて評価不要とする
          status = comparePropertyVector(results[i].vec, results[j].vec);
          if (status == 0)
            {
              results[j].score = -1;
            }
        }

      ++num;
    }

  // 評価する結果を先頭に集める
  qsort(results, numOfResults, sizeof(MatchResult), compareResultScore);

  // 評価
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (i = 0; i < num; i++)
    {
      // 距離変換画像を用いた評価値計算
      results[i].score = 
        calcEvaluationValue2DMultiCameras(model, pairing, results[i].mat, dstImages) * weight;
    }

  // 評価値でソート
  qsort(results, num, sizeof(MatchResult), compareResultScore);

  return;
}
#else
void
getResultScore(MatchResult* results,   // 認識結果情報
               int numOfResults,       // 認識結果数
               Features3D* model,      // モデルの３次元特徴情報
               StereoPairing& pairing, // ステレオ処理ペア情報
               double weight)          // 評価値の重みづけ
{
  int i, j, num;
  int status;

  // 認識結果ベクトル（位置＋回転ベクトル）の値でソートする
  qsort(results, numOfResults, sizeof(MatchResult), compareResultScore);

  // 完全に同じ位置姿勢の結果には評価不要の印をつける
  for (i = 0; i < numOfResults; i++)
    {
      // 既に不要となった結果はスキップ
      if (results[i].score == -1)
        {
          continue;
        }

      for (j = i + 1; j < numOfResults; j++)
        {
          // 既に不要となった結果はスキップ
          if (results[j].score == -1)
            {
              continue;
            }
          // 認識結果が全く同じ場合は評価値に -1 を入れて評価不要とする
          status = comparePropertyVector(results[i].vec, results[j].vec);
          if (status == 0)
            {
              results[j].score = -1;
            }
        }
    }

  // 評価する結果を先頭に集める
  qsort(results, numOfResults, sizeof(MatchResult), compareResultScore);

  // 評価
  num = 0;
  for (i = 0; i < numOfResults; i++)
    {
      if (results[i].score == -1)
        {
          break;
        }
      // モデルを画像に投影することによって認識結果の評価値計算をする
      results[i].score = traceModelPointsMultiCameras(model, pairing, results[i].mat) * weight;
      ++num;
    }

  // 評価値でソート
  qsort(results, num, sizeof(MatchResult), compareResultScore);
  return;
}
#endif
