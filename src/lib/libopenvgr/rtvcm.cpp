/*
 rtvcm.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file rtvcm.cpp
 * @brief モデル入出力関連関数
 * @date \$Date::                            $
 */
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include "match3Dfeature.h"
#include "vectorutil.h"
#include "rtvcm.h"
#include "visionErrorCode.h"
#include "circle.h"

using namespace std;

// 文字列が数値なら true を返す
static bool
isNumber(string word)
{
  string::size_type dp;
  string target = word;

  // 引数文字列にスペースが含まれないことが前提

  // 符号を削除しておく
  dp = target.find("-");
  if (dp != string::npos)
    {
      target.erase(dp, 1);
    }
  // 符号を削除しておく
  dp = target.find("+");
  if (dp != string::npos)
    {
      target.erase(dp, 1);
    }
  // 小数点を削除しておく
  dp = target.find(".");
  if (dp != string::npos)
    {
      target.erase(dp, 1);
    }
  // 指数記号を削除しておく
  dp = target.find("e");
  if (dp != string::npos)
    {
      target.erase(dp, 2);
    }
  // 指数記号を削除しておく
  dp = target.find("E");
  if (dp != string::npos)
    {
      target.erase(dp, 2);
    }
  // 指数符号を削除しておく
  dp = target.find("-");
  if (dp != string::npos)
    {
      target.erase(dp, 1);
    }
  // 指数符号を削除しておく
  dp = target.find("+");
  if (dp != string::npos)
    {
      target.erase(dp, 1);
    }
  // 数字以外の文字があったら false を返す
  const char *str = target.c_str();
  size_t i;
  for (i = 0; i < target.length(); i++)
    {
      if (!isdigit(str[i]))
        {
          return false;
        }
    }
  return true;
}

// 一行の文字列データから最初の数値データを取り出す
static bool
getNumber(string& line, string& word)
{
  size_t i;

  // 行頭スペース文字の削除
  for (i = 0; i < line.length() && isspace(line.at(i)); i++);
  line.erase(0, i);

  // スペース文字の検索
  string::size_type dp = line.find(" ");
  if (dp == string::npos)
    {
      // スペースが無いときは行全部を単語とする
      word = line.substr(0, line.length());
    }
  else
    {
      // スペースの前までを取り出す
      word = line.substr(0, dp);
      // 見つけたスペースまでを行データから消去
      line.erase(0, dp + 1);
    }
  // 取り出した単語が数値であるか確認する
  // 数値でないときは false を返す
  return isNumber(word);
}

// ３次元データの取得
static bool
get3Ddata(string& line, double data[3])
{
  string word;
  int i;

  for (i = 0; i < 3; i++)
    {
      // 文字列の数値部分を取り出す
      if (!getNumber(line, word))
        {
          return false;
        }
      // 数値データに変換する
      data[i] = atof(word.c_str());
    }
  return true;
}

// 入力行から重心情報を読み取る
static bool
readRTVCMgravity(string line, RTVCM& rtvcm)
{
  // 重心座標の取得
  return get3Ddata(line, rtvcm.gravity);
}

// 入力行からモデル頂点情報を読み取る
static bool
readRTVCMvertex(string line, RTVCM_Vertex& vertex)
{
  // 端点１座標の取得
  if (!get3Ddata(line, vertex.endpoint1))
    {
      return false;
    }
  // 頂点座標の取得
  if (!get3Ddata(line, vertex.position))
    {
      return false;
    }
  // 端点２座標の取得
  if (!get3Ddata(line, vertex.endpoint2))
    {
      return false;
    }
  return true;
}

// 入力行からモデル円情報を読み取る
static bool
readRTVCMcircle(string line, RTVCM_Circle& circle)
{
  string word;
  // 半径の取得
  if (!getNumber(line, word))
    {
      return false;
    }
  circle.radius = atof(word.c_str());
  // 中心座標の取得
  if (!get3Ddata(line, circle.center))
    {
      return false;
    }
  // 法線ベクトルの取得
  if (!get3Ddata(line, circle.normal))
    {
      return false;
    }
  return true;
}

// 入力行からキーワードを検索する
static bool
findKeyword(string keyword, string& line)
{
  // キーワードが見つかったら削除して true を返す
  string::size_type dp = line.find(keyword);
  if (dp != string::npos)
    {
      line.erase(0, dp + keyword.length());
      return true;
    }
  return false;
}

// モデルデータのメモリ解放
void
freeRTVCM(RTVCM& rtvcm)
{
  free(rtvcm.vertex);
  free(rtvcm.circle);
  rtvcm.vertex = NULL;
  rtvcm.circle = NULL;
  rtvcm.nvertex = 0;
  rtvcm.ncircle = 0;
  return;
}

// モデルデータの読み込み
// 戻り値：エラーコード
int
readRTVCModel(char* filename,  // モデルデータファイル名
              RTVCM& rtvcm)    // モデルデータ
{
  ifstream ifs;
  string line;
  string::size_type dp;
  RTVCM_Vertex vertex = { 0 };
  RTVCM_Circle circle = { 0 };
  int p, q;
  int crPos;
  int errcode = 0;

  ifs.open(filename);

  if (ifs)
    {
      // ファイル中にある VERTEX と CIRCLE の行数を調べる
      rtvcm.nvertex = 0;
      rtvcm.ncircle = 0;
      try
      {
        while (getline(ifs, line))
          {
            dp = line.find("VERTEX");
            if (dp != string::npos)
              {
                ++rtvcm.nvertex;
              }
            dp = line.find("CIRCLE");
            if (dp != string::npos)
              {
                ++rtvcm.ncircle;
              }
          }
      }
      catch (...)
      {
        cerr << "ファイル読み込みエラー ！ " << filename << endl;
        return VISION_FILE_FORMAT_ERROR;
      }

      if (rtvcm.nvertex == 0 && rtvcm.ncircle == 0)
        {
          cerr << "有効データなし ！ " << filename << endl;
          return VISION_FILE_FORMAT_ERROR;
        }

      if (rtvcm.nvertex)
        {
          rtvcm.vertex = (RTVCM_Vertex*) calloc(rtvcm.nvertex, sizeof(RTVCM_Vertex));
          if (rtvcm.vertex == NULL)
            {
              errcode = VISION_MALLOC_ERROR;
              goto errorEnding;
            }
        }

      if (rtvcm.ncircle)
        {
          rtvcm.circle = (RTVCM_Circle*) calloc(rtvcm.ncircle, sizeof(RTVCM_Circle));
          if (rtvcm.circle == NULL)
            {
              errcode = VISION_MALLOC_ERROR;
              goto errorEnding;
            }
        }
      // ファイル読み込みを先頭に戻す
      ifs.clear();
      ifs.seekg(0, ios::beg);

      p = q = 0;

      try
      {
        while ((!ifs.eof()) && (getline(ifs, line)))
          {
            if (!findKeyword("RTVCML ", line))
              {
                continue;
              }

            if ((crPos = line.find("\r")) != -1)
              {
                line.erase(crPos, 1);
              }

            if (findKeyword("GRAVITY ", line))
              {
                if (!readRTVCMgravity(line, rtvcm))
                  {
                    errcode = VISION_FILE_FORMAT_ERROR;
                    goto errorEnding;
                  }
              }
            else if (findKeyword("VERTEX ", line))
              {
                if (!readRTVCMvertex(line, vertex))
                  {
                    errcode = VISION_FILE_FORMAT_ERROR;
                    goto errorEnding;
                  }
                vertex.n = p;
                rtvcm.vertex[p] = vertex;
                p++;
              }
            else if (findKeyword("CIRCLE ", line))
              {
                if (!readRTVCMcircle(line, circle))
                  {
                    errcode = VISION_FILE_FORMAT_ERROR;
                    goto errorEnding;
                  }
                circle.n = q;
                rtvcm.circle[q] = circle;
                q++;
              }
          }
      }
      catch (...)
      {
        cerr << "ファイル読み込みエラー ！ " << filename << endl;
        errcode = VISION_FILE_FORMAT_ERROR;
        goto errorEnding;
      }

    }
  else
    {
      cerr << "ファイルオープンエラー ！ " << filename << endl;
      return VISION_FILE_OPEN_ERROR;
    }

  return 0;

errorEnding:
  if (rtvcm.vertex != NULL)
    {
      free(rtvcm.vertex);
    }
  if (rtvcm.circle != NULL)
    {
      free(rtvcm.circle);
    }
  freeRTVCM(rtvcm);
  return errcode;
}

// モデルの頂点データを３次元頂点データへ変換
static void
convertVertex(RTVCM_Vertex src, Vertex& dst)
{
  double endpoint[2][3];
  double length[2];
  double vangle;
  int i;

  // 端点の頂点に対する相対座標
  for (i = 0; i < 3; ++i)
    {
      endpoint[0][i] = src.endpoint1[i] - src.position[i];
      endpoint[1][i] = src.endpoint2[i] - src.position[i];
    }

  length[0] = getNormV3(endpoint[0]);
  length[1] = getNormV3(endpoint[1]);

  normalizeV3(endpoint[0], endpoint[0]);
  normalizeV3(endpoint[1], endpoint[1]);

  // 頂点を構成する線分の成す角を求める
  vangle = acos(getInnerProductV3(endpoint[0], endpoint[1]));
  dst.angle = vangle * 180.0 / M_PI;

  // 以下の３つのベクトルを用いて姿勢を表す行列をつくる
  // 頂点の法線を求める
  getCrossProductV3(endpoint[0], endpoint[1], dst.tPose[2]);
  normalizeV3(dst.tPose[2], dst.tPose[2]);

  // 頂点を構成する線分が成す角の２等分線（単位方向ベクトルの中線）を求める
  addV3(endpoint[0], endpoint[1], dst.tPose[1]);
  normalizeV3(dst.tPose[1], dst.tPose[1]);

  // 頂点の法線と中線の両方に直交する軸の方向を求める
  getCrossProductV3(dst.tPose[1], dst.tPose[2], dst.tPose[0]);

  // 端点の座標を代入する
  dst.endpoint1[0] = length[0] * sin(vangle / 2.0);
  dst.endpoint1[1] = length[0] * cos(vangle / 2.0);
  dst.endpoint1[2] = 0.0;

  dst.endpoint2[0] = length[1] * -sin(vangle / 2.0);
  dst.endpoint2[1] = length[1] * cos(vangle / 2.0);
  dst.endpoint2[2] = 0.0;

  // 頂点を構成する線分の単位方向ベクトルを求める
  normalizeV3(dst.endpoint1, dst.direction1);
  normalizeV3(dst.endpoint2, dst.direction2);

  // 平行移動成分
  for (i = 0; i < 3; ++i)
    {
      dst.tPose[3][i] = src.position[i];
    }

  for (i = 0; i < 4; ++i)
    {
      dst.tPose[i][3] = (i < 3) ? 0.0 : 1.0;
    }

  // 特徴の通し番号を設定
  dst.n = src.n;
  dst.side = M3DF_FRONT;
  return;
}

// ３次元頂点データの裏データ作成
void
reverseVertex(Vertex src,      // ３次元頂点データ
              Vertex& dst)     // ３次元頂点データ
{
  int i;

  // 元の情報をコピーする
  dst.n = src.n;
  dst.angle = src.angle;
  // 反対向きの情報をコピーする
  for (i = 0; i < 3; ++i)
    {
      dst.endpoint1[i] = ((i != 1) ? -1.0 : 1.0) * src.endpoint1[i];
      dst.endpoint2[i] = ((i != 1) ? -1.0 : 1.0) * src.endpoint2[i];
      dst.direction1[i] = ((i != 1) ? -1.0 : 1.0) * src.direction1[i];
      dst.direction2[i] = ((i != 1) ? -1.0 : 1.0) * src.direction2[i];
    }
  // 反対向きの姿勢行列をつくる
  copyV3(src.tPose[1], dst.tPose[1]);
  copyV3(src.tPose[3], dst.tPose[3]);
  mulV3S(-1.0, src.tPose[2], dst.tPose[2]);
  mulV3S(-1.0, src.tPose[0], dst.tPose[0]);
  for (i = 0; i < 4; ++i)
    {
      dst.tPose[i][3] = (i < 3) ? 0.0 : 1.0;
    }
  dst.side = M3DF_BACK;
  return;
}

// モデルの円データを３次元円データへ変換
static void
convertCircle(RTVCM_Circle src, Circle& dst)
{
  int i;

  // 元の情報をコピーする
  dst.radius = src.radius;
  copyV3(src.normal, dst.tPose[2]);

  // 法線と直交するベクトルを求める
  calc_3d_axes_of_circle(dst.tPose[0], dst.tPose[1], dst.tPose[2], NULL);

  // 円の法線
  for (i = 0; i < 3; ++i)
    {
      dst.normal[i] = (i != 2) ? 0.0 : 1.0; // 法線はz軸
    }

  // 平行移動成分
  copyV3(src.center, dst.tPose[3]);
  
  for (i = 0; i < 4; ++i)
    {
      dst.tPose[i][3] = (i < 3) ? 0.0 : 1.0;
    }

  // 特徴の通し番号を設定
  dst.n = src.n;
  dst.side = M3DF_FRONT;
  return;
}

// ３次元円データの裏データ作成
void
reverseCircle(Circle src,      // ３次元円データ
              Circle& dst)     // ３次元円データ
{
  int i;

  // 元の情報をコピーする
  dst.n = src.n;
  dst.radius = src.radius;
  copyV3(src.normal, dst.normal);
  copyV3(src.tPose[1], dst.tPose[1]);
  copyV3(src.tPose[3], dst.tPose[3]);
  // 反対向きの情報をコピーする
  mulV3S(-1.0, src.tPose[0], dst.tPose[0]);
  mulV3S(-1.0, src.tPose[2], dst.tPose[2]);
  for (i = 0; i < 4; ++i)
    {
      dst.tPose[i][3] = (i < 3) ? 0.0 : 1.0;
    }
  dst.side = M3DF_BACK;
  return;
}

// モデルデータから３次元特徴データへの変換
int
convertRTVCMtoFeatures3D(RTVCM rtvcm,          // モデルデータ
                         Features3D& feature)  // ３次元特徴データ
{
  int numOfVertices, numOfCircles, i;

  numOfVertices = rtvcm.nvertex;
  if (numOfVertices)
    {
      feature.Vertices = (Vertex*) calloc(numOfVertices, sizeof(Vertex));
      if (feature.Vertices == NULL)
        {
          return VISION_MALLOC_ERROR;
        }
    }

  numOfCircles = rtvcm.ncircle;
  if (numOfCircles)
    {
      feature.Circles = (Circle*) calloc(numOfCircles, sizeof(Circle));
      if (feature.Circles == NULL)
        {
          free(feature.Vertices);
          feature.Vertices = NULL;
          return VISION_MALLOC_ERROR;
        }
    }

  feature.numOfVertices = numOfVertices;
  feature.numOfCircles = numOfCircles;

  for (i = 0; i < rtvcm.nvertex; i++)
    {
      convertVertex(rtvcm.vertex[i], feature.Vertices[i]);
    }

  for (i = 0; i < rtvcm.ncircle; i++)
    {
      convertCircle(rtvcm.circle[i], feature.Circles[i]);
    }

  return 0;
}
