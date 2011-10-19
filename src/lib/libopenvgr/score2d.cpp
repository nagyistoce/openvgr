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
      return 0;
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

// 結果の２次元評価値算出
void
getResultScore(MatchResult* results,   // 認識結果情報
               int numOfResults,       // 認識結果数
               Features3D* model,      // モデルの３次元特徴情報
               StereoPairing& pairing, // ステレオ処理ペア情報
               const std::vector<cv::Mat>& dstImages,
               double weight)          // 評価値の重みづけ
{
  int i, j, num;

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
      cv::Mat plot = cv::Mat::zeros(cv::Size(model->calib->colsize, model->calib->rowsize), CV_8UC1);
      // 距離変換画像を用いた評価値計算
      results[i].score = 
        calcEvaluationValue2DMultiCameras(model, pairing, &results[i], dstImages, &plot) * weight;
    }

  // 評価値でソート
  qsort(results, num, sizeof(MatchResult), compareResultScore);

  return;
}
