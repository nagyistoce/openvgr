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

// 認識結果評価値の比較
// 戻り値：比較結果
int
compareResultScore(const void* c1,     // 評価値１
                   const void* c2)     // 評価値２
{
  const MatchResult* r1 = (const MatchResult*) c1;
  const MatchResult* r2 = (const MatchResult*) c2;

  return (int)(r2->score - r1->score);
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
  int i;

  // 評価
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (i = 0; i < numOfResults; i++)
    {
      plot_t plot;

      if (results[i].score < 0.0)
        {
          continue;
        }

      // 距離変換画像を用いた評価値計算
      results[i].score =
        calcEvaluationValue2DMultiCameras(model, pairing, &results[i], &plot, dstImages) * weight;
    }

  // 評価値でソート
  qsort(results, numOfResults, sizeof(MatchResult), compareResultScore);

  return;
}
