/*
 rtvcm.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file rtvcm.h
 * @brief モデル入出力関連関数
 * @date \$Date::                            $
 */

#ifndef _RTVCM_H
#define _RTVCM_H

//! モデル内の頂点データ
typedef struct RTVCM_Vertex
{
  int n;                        //!< 通し番号
  double position[3];           //!< ３次元位置
  double endpoint1[3];          //!< ３次元端点１
  double endpoint2[3];          //!< ３次元端点２
  double angle;                 //!< ２直線のなす角度
  int nbox;                     //!< 属する立方体の通し番号
  void* reserved;               //!< 拡張
} RTVCM_Vertex;

//! モデル内の円データ
typedef struct RTVCM_Circle
{
  int n;                        //!< 通し番号
  double radius;                //!< 半径
  double center[3];             //!< 中心の３次元位置
  double normal[3];             //!< ３次元法線方向
  int ncyliner;                 //!< 属する円筒の通し番号
  void* reserved;               //!< 拡張
} RTVCM_Circle;

//! モデル内の立方体データ
typedef struct RTVCM_Box
{
  int n;                        //!< 通し番号
  double x, y, z;               //!< 幅、奥行き、高さ
  double Rotate[3][3];          //!< 基準位置からの回転
  double Trans[3];              //!< 基準位置からの移動
  int nVertex[24];              //!< 頂点通し番号列
  void* reserved;               //!< 拡張
} RTVCM_Box;

//! モデル内の円筒データ
typedef struct RTVCM_Cylinder
{
  int n;                        //!< 通し番号
  double radius;                //!< 半径
  double height;                //!< 高さ
  double Rotate[3][3];          //!< 基準位置からの回転
  double Trans[3];              //!< 基準位置からの移動
  int* nCircle[2];              //!< 構成円の通し番号配列
  void* reserved;               //!< 拡張
} RTVCM_Cylinder;

typedef int RTVCM_Label;

//! モデルデータ構造体
typedef struct RTVertexCircleModel
{
  RTVCM_Label label;            //!< 属性
  int n;                        //!< 通し番号
  double gravity[3];            //!< 重心

  double width;
  //!< 属性が立方体の場合は幅(x),（円柱の場合は外接矩形の幅(x)に使っても良い）
  double height;
  //!< 属性が立方体の場合は奥行き (y),円柱の場合は高さ(y)
  double depth;
  //!< 属性が立方体の場合は高さ(z),（円柱の場合は外接矩形の奥行き(z)に使っても良い）

  double radius;                //!< 属性が円柱の場合の半径
  int nvertex;                  //!< 頂点数
  RTVCM_Vertex* vertex;         //!< 頂点列
  int ncircle;                  //!< 円数
  RTVCM_Circle* circle;         //!< 円列
  int nbox;                     //!< 直方体数
  RTVCM_Box* box;               //!< 直方体列（表示用）
  int ncylinder;                //!< 円筒数
  RTVCM_Cylinder* cylinder;     //!< 円筒列（表示用）
  void* reserved;               //!< 拡張
} RTVCM;

//! モデルデータのメモリ解放
void freeRTVCM(RTVCM& rtvcm);

//! モデルデータの読み込み
//! 戻り値：エラーコード
int readRTVCModel(char* filename,      // モデルデータファイル名
		  RTVCM& rtvcm);       // モデルデータ

//! ３次元頂点データの裏データ作成
void reverseVertex(Vertex src,         // ３次元頂点データ
                   Vertex& dst);       // ３次元頂点データ

//! ３次元円データの裏データ作成
void reverseCircle(Circle src,         // ３次元円データ
                   Circle& dst);       // ３次元円データ

//! モデルデータから３次元特徴データへの変換
int convertRTVCMtoFeatures3D(RTVCM rtvcm,           // モデルデータ
                             Features3D& feature);  // ３次元特徴データ

#endif // _RTVCM_H
