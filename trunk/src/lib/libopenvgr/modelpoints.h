/*
 modelpoints.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file modelpoints.h
 * @brief モデル評価点生成関連関数
 * @date \$Date::                            $
 */

#ifndef _MODELPOINTS_H
#define _MODELPOINTS_H

#include "match3Dfeature.h"

//! モデル評価点の描画（認識結果確認表示用）
void drawModelPoints(Features3D* model,        // モデルの３次元特徴情報
                     double matrix[4][4],      // 認識結果の位置姿勢変換行列
                     char* filename,           // 結果表示の出力ファイル
                     int p_camera,             // 結果表示する画像のカメラ番号
                     unsigned char* img,       // 結果表示用の画像データ
                     int lineThickness);       // 描画する線の太さ

//! モデルの評価点の生成
//! 戻り値：総評価点数
int makeModelPoints(Features3D* model,         // モデルの３次元特徴データ
                    double pdist);             // 評価点間隔

//! 合同変換行列を位置ベクトルと回転ベクトルを合わせた7次元ベクトルに変換する
void getPropertyVector(double mat[4][4],       // 合同変換行列
                       double vec[7]);         // ７次元ベクトル

//! 使用した全画像を用いた２次元評価値計算
//! 戻り値：２次元評価値
double traceModelPointsMultiCameras(Features3D* model,      // モデルの３次元特徴情報
                                    StereoPairing& pairing, // ステレオペア情報
                                    double matrix[4][4]);   // 認識結果の位置姿勢変換行列

//! 使用した全画像を用いた２次元評価値計算。距離変換画像の利用
//! 戻り値：２次元評価値
double calcEvaluationValue2DMultiCameras(Features3D* model,      // モデルの３次元特徴情報
                                         StereoPairing& pairing, // ステレオペア情報
                                         double matrix[4][4],    // 認識結果の位置姿勢変換行列
                                         const std::vector<cv::Mat>& dstImages); // 距離変換画像

// 座標値が正しい範囲内か確認
inline int
isValidPixelPosition(int col, int row, Features3D* finfo)
{
  int wlimit = finfo->calib->colsize - 2;
  int hlimit = finfo->calib->rowsize - 2;

  if (col >= 2 && col < wlimit && row >= 2 && row < hlimit)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

#endif // _MODELPOINTS_H
