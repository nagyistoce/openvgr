/*
 match3Dfeature.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file match3Dfeature.h
 * @brief 3次元特徴による認識関連関数
 * @date \$Date::                            $
 */

#ifndef _MATCH3DFEATURE_H
#define _MATCH3DFEATURE_H

#include <cv.h>
#include "parameters.h"
#include "calib.h"

//! 表裏を表す定数
enum m3df_side
{
  M3DF_FRONT = 0,
  M3DF_BACK  = 1
};

//! 3次元特徴のラベル
enum m3df_feature_label
{
  M3DF_LABEL_NONE = 0,
  M3DF_LABEL_NOEVAL = 1
};

//! 認識結果評価用サンプリング点列情報
typedef struct Trace
{
  int label;                    //!< ラベル：可視情報(VISIBLE/INVISIBLE)
  double weight;                // 未使用
  double xyz[4];                //!< 3次元位置情報(mm)
  double colrow[2];             //!< 画像投影位置情報（画素）
  int direction;                //!< 対応エッジ探索方向
  int search;                   //!< 対応エッジの距離（画素）
  int edge;                     //!< 対応エッジの強度
  double peakcr[2];             //!< 対応エッジ点の位置（画素）
} Trace;

//! 3次元位置情報
typedef struct P3D
{
  double xyz[3];                //!< 3次元座標
} P3D;

//! 2次元位置情報
typedef struct P2D
{
  double colrow[2];             //!< 2次元座標
} P2D;

//! ワイヤフレームモデル
typedef struct tag_Wireframe
{
  int num_vertices;             //!< 頂点数
  double (*vertex)[3];          //!< 頂点座標の配列

  struct Segment                //!< 線分
  {
    int vertex_id[2];

    Segment()
    {
      vertex_id[0] = vertex_id[1] = -1;
    }

    Segment(const int id1, const int id2)
    {
      vertex_id[0] = id1;
      vertex_id[1] = id2;
    }
  };
  std::vector<Segment> segment; //!< 線分情報の配列

  struct Face                   //!< 面
  {
    std::vector<int> segment_id;//!< 面を構成する線分
    double normal[3];           //!< 面の法線ベクトル

    Face() : segment_id()
    {
      normal[0] = normal[1] = normal[2] = 0.0;
    }
  };
  std::vector<Face> face;       //!< 面情報の配列
} Wireframe;

//! ３次元頂点情報
typedef struct Vertex
{
  int label;                    //!< ラベル
  int n;                        //!< 通し番号
  int side;                     //!< 表裏情報
  double endpoint1[3];          //!< 辺の端点(mm)
  double endpoint2[3];          //!< 辺の端点(mm)
  double direction1[3];         //!< 辺の方向
  double direction2[3];         //!< 辺の方向
  double tPose[4][4];           //!< 認識用姿勢行列
  double angle;                 //!< 頂点角度(ラジアン）
  int numOfTracePoints;         //!< 認識評価用のサンプリング点列数
  Trace* tracepoints;           //!< 認識評価用のサンプリング点列情報
  P3D* transformed;             //!< 認識時の位置・姿勢変換後の3次元点列(mm)
  P2D* projected;               //!< 2次元評価時の画像投影2次元点列(画素
} Vertex;

//! ３次元円情報
typedef struct Circle
{
  int label;                    //!< ラベル
  int n;                        //!< 通し番号
  int side;                     //!< 表裏情報
  double radius;                //!< 半径
  double normal[3];             //!< 法線
  double tPose[4][4];           //!< 認識用姿勢行列
  int numOfTracePoints;         //!< 認識評価用のサンプリング点列数
  Trace* tracepoints;           //!< 認識評価用のサンプリング点列情報
  P3D* transformed;             //!< 認識時の位置・姿勢変換後の3次元点列
  P2D* projected;               //!< 2次元評価時の画像投影2次元点列
} Circle;

//! ３次元特徴情報
typedef struct Features3D
{
  CalibParam* calib;            //!< キャリブレーションデータポインタ
  int numOfVertices;            //!< ３次元頂点特徴数
  Vertex* Vertices;             //!< ３次元頂点特徴
  Wireframe wireframe;          //!< ワイヤフレームデータ
  int numOfCircles;             //!< ３次元円特徴数
  Circle* Circles;              //!< ３次元円特徴
  uchar* edge[3];               //!< エッジ画像ポインタ
  int pointCounts;              //!< ２次元評価のための全評価点数
  double traceCounts;           //!< ２次元評価に用いた評価点数
} Features3D;

//! 各認識結果情報
typedef struct MatchResult
{
  int n;                        //!< 通し番号
  int type;                     //!< 特徴タイプ 0:頂点、1:単円、2:２円
  int scene[2];                 //!< シーン特徴番号
  int model[2];                 //!< モデル特徴番号
  double score;                 //!< ２次元評価値
  double mat[4][4];             //!< 変換行列
  double vec[7];                //!< 変換行列の７次元のベクトル（位置＋回転）表現
  int npoint;                   //!< 総投影点数
  int cpoint;                   //!< 総対応点数
} MatchResult;

//! 全認識結果
typedef struct Match3Dresults
{
  int error;                    //!< 認識エラーフラグ
  int numOfResults;             //!< 認識結果数
  MatchResult* Results;         //!< 各認識結果
} Match3Dresults;

//! 認識結果データのメモリ解除
void freeMatch3Dresults(Match3Dresults* holder);

//! 不要になった認識結果データの開放
void shrinkMatch3Dresults(Match3Dresults* Match);

//! ３次元特徴データのメモリ解放
void freeFeatures3D(Features3D* feature);

//! 認識：シーン特徴とモデル特徴の照合
//! 戻り値：認識結果
Match3Dresults matchFeatures3D(Features3D& scene,    // シーンの３次元特徴情報
                               Features3D& model,    // モデルの３次元特徴情報
                               Parameters& parameters); // 全パラメータ

//! ２円を使った照合
//! 戻り値：認識結果
extern Match3Dresults matchPairedCircles(Features3D& scene,      // シーンの３次元特徴情報
                                         Features3D& model,      // モデルの３次元特徴情報
                                         const std::vector<cv::Mat>& dstImages,
                                         double tolerance,       // 照合の許容値
                                         StereoPairing& pairing);  // ステレオペアの情報

#endif // _MATCH3DFEATURE_H
