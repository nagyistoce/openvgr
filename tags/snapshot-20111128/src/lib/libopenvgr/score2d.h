/*
 score2d.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file score2d.h
 * @brief 2次元評価関連関数
 * @date \$Date::                            $
 */

#ifndef _SCORE2D_H
#define _SCORE2D_H

// 認識結果評価値の比較
// 戻り値：比較結果
int compareResultScore(const void* c1,   // 評価値１
                       const void* c2);  // 評価値２

// 結果の２次元評価値算出
void getResultScore(MatchResult* results,      // 認識結果情報
                    int numOfResults,          // 認識結果数
                    Features3D* model,         // モデルの３次元特徴情報
                    StereoPairing& pairing,    // ステレオ処理ペア情報
                    const std::vector<cv::Mat>& dstImages,
                    double weight);            // 評価値の重みづけ

#endif // _SCORE2D_H
