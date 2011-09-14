/*
 vectorutil.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file vectorutil.h
 * @brief ベクトル処理、行列処理　ユーティリティ関数
 * @date \$Date::                            $
 */

#ifndef _VECTORUTIL_H
#define _VECTORUTIL_H

#include <cxcore.h>
#include "common.h"
#include "quaternion.h"

typedef double V2[2];           //!< 2 dimensional vector
typedef double V3[3];           //!< 3 dimensional vector
typedef double M33[3][3];       //!< 3x3 matrix

int isZero(double value);      // ゼロチェック

// ２次元ベクトル計算
void copyV2(V2 in, V2 out);                    // コピー
void mulV2S(V2 in, const double s, V2 out);    // スカラー積
void normalizeV2(V2 in, V2 out);               // 規格化
double getNormV2(V2 in);                       // ノルム
double getDistanceV2(V2 in1, V2 in2);          // ２点間の距離

// ベクトルのなす角（ラジアン）
double getAngle2D(double vec1[2], double vec2[2]);

// ３次元ベクトル計算
void copyV3(V3 in, V3 out);                     // コピー
void addV3(V3 in1, V3 in2, V3 out);             // 加算
void subV3(V3 in1, V3 in2, V3 out);             // 減算
void mulV3S(const double s, V3 in, V3 out);     // スカラー積
void normalizeV3(V3 in, V3 out);                // 規格化
double getInnerProductV3(V3 in1, V3 in2);       // 内積
double getNormV3(V3 in);                        // ノルム
void getCrossProductV3(V3 in1, V3 in2, V3 out); // クロス積
double getDistanceV3(V3 in1, V3 in2);           // ２点間の距離

// ３ｘ３行列計算
void subM33(M33 in1, M33 in2, M33 out);        // 減算
void transposeM33(M33 in, M33 out);            // 転置
void mulM33(M33 in1, M33 in2, M33 out);        // 積
void mulM33V3(M33 in1, V3 in2, V3 out);        // ベクトル積
int inverseM33(M33 in, M33 out);               // 逆行列計算

// 終点から始点への方向ベクトル
void getDirectionVector(double tail[3], double head[3], double data[3], CvMat* vec);
// 法線に対する直交ベクトル
int getOrthogonalDir(double axis[3], double normal[3], double dir[3]);
// 法線に対する直交ベクトル
int getOrthogonalDir(CvMat* axis, CvMat* normal, CvMat* dir);

// 任意軸周りの回転を表すクォータニオンを求める
void quaternion_rotation(quaternion_t q, const double radian, double axis[3]);

#endif // _VECTORUTIL_H
