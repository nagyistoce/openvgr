/*
 extractFeature_old.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file extractFeature_old.h
 * @brief 2次元特徴抽出関連関数
 * @date \$Date::                            $
 */

#ifndef _EXTRACTFEATURE_OLD_H
#define _EXTRACTFEATURE_OLD_H

#include "conic.h"
#include "match3Dfeature.h"

#define ALLOC_STEP	(1024)

#define SKIP_LEN	(10)
#define MAX_SLIDE_LEN	(10)

// 楕円arclist
typedef struct _ellipse_arc_
{
  struct Features2D_old	*f2Ds;
  int	ntrack;
  int	start;
  int	goal;
} EllipseArc;

typedef struct _ellipse_arc_list_
{
  int	n;
  EllipseArc	*arc;
} EllipseArcList;
  

//! 各２次元特徴
typedef struct Feature2D_old
{
  ConicType type;               //!< 二次曲線の分類
  double coef[6];               //!< 二次曲線の係数。a x^2 + bxy + c y^2 + d x + e y + f = 0,
                                //!< a = coef[0], ... , f = coef[5] に対応
  double center[2];             //!< 楕円中心または、双曲線の漸近線交点
  double startPoint[2];         //!< 曲線上の始点
  double endPoint[2];           //!< 曲線上の終点
  int start;                    //!< 始点番号
  int end;                      //!< 終点番号
  int all;                      //!< 輪郭全体の点数（nPoint）
  double startSPoint[2];        //!< 点列の始点の位置
  double middleSPoint[2];       //!< 点列の中間の位置
  double endSPoint[2];          //!< 点列の終点の位置
  double ev[2][2];              //!< 楕円の回転行列
  double axis[2];               //!< 楕円の長半径、短半径
  double direction[2];          //!< 直線の方向ベクトル
  int nPoints;                  //!< 特徴抽出に使われた点数
  int nTrack;                   //!< 輪郭番号
  double error;                 //!< 当てはめ誤差
  double lineLength;            //!< 直線の長さ
  double lineLengthSQ;          //!< 直線の長さの自乗
  double lineLength1;           //!< 双曲線の線分1の長さ
  double lineLength2;           //!< 双曲線の線分2の長さ
  double lineAngle;             //!< 双曲線の2線分のなす角度

  EllipseArcList	arclist; // マージされた楕円特徴の点列集合
} Feature2D_old;

//! 輪郭情報
typedef struct Track
{
  int nPoint;                   //!< 点数
  int* Point;                   //!< 点列
  double offset[2];             //!< 楕円係数計算時のオフセット
} Track;

//! ２次元特徴情報
typedef struct Features2D_old
{
  int nAlloc;                   //!< メモリ確保量
  int nFeature;                 //!< ２次元特徴数
  Feature2D_old* feature;           //!< ２次元特徴情報
  int nTrack;                   //!< 輪郭数
  Track* track;                 //!< 輪郭情報
} Features2D_old;

//! 楕円重複除去用グループ情報
typedef struct EllipseGroup
{
  int* groupNums;               //!< グループ要素番号
  double groupCenter[2];        //!< (work)グループの中心座標
  int nCurrNum;                 //!< (work) グループ要素数
} EllipseGroup;

//! ２次元特徴情報のメモリ解放
void destructFeatures(Features2D_old* features);

//! ２次元特徴データのメモリ拡張
Features2D_old*
expandFeatures(Features2D_old* features);

//! ステレオ画像の一枚から二次元特徴の抽出
Features2D_old* ImageToFeature2D_old(unsigned char* src, unsigned char* edge,
                                     Parameters parameters, 
                                     const int id, // データを識別するためのインデックス
                                     Features3D model);

#endif // _EXTRACTFEATURE_OLD_H
