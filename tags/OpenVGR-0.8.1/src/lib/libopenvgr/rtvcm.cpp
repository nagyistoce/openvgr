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
  double vangle;
  CvMat vec1, vec2;
  CvMat normal, bisector, perpendicular;
  int i;

  // 元の情報をコピーする
  copyV3(src.position, dst.position);
  copyV3(src.endpoint1, dst.endpoint1);
  copyV3(src.endpoint2, dst.endpoint2);
  // 頂点を構成する線分の単位方向ベクトルを求める
  getDirectionVector(dst.position, dst.endpoint1, dst.direction1, &vec1);
  getDirectionVector(dst.position, dst.endpoint2, dst.direction2, &vec2);
  // 頂点を構成する線分の成す角を求める
  vangle = cvDotProduct(&vec1, &vec2);
  vangle = (acos(vangle) / M_PI) * 180.0;
  dst.angle = vangle;
  // 以下の３つのベクトルを用いて姿勢を表す行列をつくる
  // 頂点の法線を求める
  normal = cvMat(3, 1, CV_64FC1, dst.orientation[2]);
  cvCrossProduct(&vec1, &vec2, &normal);
  // 頂点を構成する線分が成す角の２等分線（単位方向ベクトルの中線）を求める
  bisector = cvMat(3, 1, CV_64FC1, dst.orientation[1]);
  cvAdd(&vec1, &vec2, &bisector);
  cvScale(&bisector, &bisector, 0.5);
  cvNormalize(&bisector, &bisector);
  // 頂点の法線と中線の両方に直交する軸の方向を求める
  perpendicular = cvMat(3, 1, CV_64FC1, dst.orientation[0]);
  cvCrossProduct(&bisector, &normal, &perpendicular);
  // 同次行列にする
  for (i = 0; i < 3; ++i)
    {
      dst.orientation[3][i] = 0.0;
    }
  dst.orientation[3][3] = 1.0;
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
  copyV3(src.position, dst.position);
  // 反対向きに情報をコピーする
  copyV3(src.endpoint2, dst.endpoint1);
  copyV3(src.endpoint1, dst.endpoint2);
  copyV3(src.direction2, dst.direction1);
  copyV3(src.direction1, dst.direction2);
  // 反対向きの姿勢行列をつくる
  copyV3(src.orientation[1], dst.orientation[1]);
  //copyV3( src.orientation[3], dst.orientation[3] );
  mulV3S(-1.0, src.orientation[2], dst.orientation[2]);
  mulV3S(-1.0, src.orientation[0], dst.orientation[0]);
  for (i = 0; i < 3; ++i)
    {
      dst.orientation[3][i] = 0.0;
    }
  dst.orientation[3][3] = 1.0;
  dst.side = M3DF_BACK;
  return;
}

// モデルの円データを３次元円データへ変換
static void
convertCircle(RTVCM_Circle src, Circle& dst)
{
  CvMat axis, normal, dir1, dir2;
  double adata[3];
  int i, sts;

  // 元の情報をコピーする
  dst.radius = src.radius;
  copyV3(src.center, dst.center);
  copyV3(src.normal, dst.normal);
  copyV3(src.normal, dst.orientation[0]);

  axis = cvMat(3, 1, CV_64FC1, adata);
  normal = cvMat(3, 1, CV_64FC1, dst.orientation[0]);
  dir1 = cvMat(3, 1, CV_64FC1, dst.orientation[1]);

  for (i = 0; i < 3; i++)
    {
      cvSetZero(&axis);
      // x, y, z 軸の単位方向ベクトルを順に試す
      cvmSet(&axis, i, 0, 1.0);
      // axis と normal に直交するベクトルを dir1 に返す
      sts = getOrthogonalDir(&axis, &normal, &dir1);
      // うまくいったらループから出る
      if (sts == 0)
        {
          break;
        }
    }

  if (sts == -1)
    {
      cvSetZero(&axis);
      cvSetZero(&normal);
      cvSetZero(&dir1);
      return;
    }

  dir2 = cvMat(3, 1, CV_64FC1, dst.orientation[2]);
  // normal と dir1 に直交するベクトルを dir2 に返す
  cvCrossProduct(&normal, &dir1, &dir2);
  cvNormalize(&dir2, &dir2);

  // 同次行列にする
  for (i = 0; i < 3; ++i)
    {
      dst.orientation[3][i] = 0.0;
    }
  dst.orientation[3][3] = 1.0;

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
  copyV3(src.center, dst.center);
  copyV3(src.orientation[2], dst.orientation[2]);
  // 反対向きの情報をコピーする
  mulV3S(-1.0, src.orientation[1], dst.orientation[1]);
  mulV3S(-1.0, src.normal, dst.normal);
  copyV3(dst.normal, dst.orientation[0]);
  // 同次行列にする
  for (i = 0; i < 3; ++i)
    {
      dst.orientation[3][i] = 0.0;
    }
  dst.orientation[3][3] = 1.0;
  dst.side = M3DF_BACK;
  return;
}

// モデルデータから３次元特徴データへの変換
int
convertRTVCMtoFeatures3D(RTVCM rtvcm,          // モデルデータ
                         Features3D& feature)  // ３次元特徴データ
{
  int numOfVertices, numOfCircles, i, j;

  // 表裏の特徴をつくるため元の２倍の領域を確保する

  numOfVertices = rtvcm.nvertex * 2;
  if (numOfVertices)
    {
      feature.Vertices = (Vertex*) calloc(numOfVertices, sizeof(Vertex));
      if (feature.Vertices == NULL)
        {
          return VISION_MALLOC_ERROR;
        }
    }

  numOfCircles = rtvcm.ncircle * 2;
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
      j = i * 2;
      // 表の特徴を作成
      convertVertex(rtvcm.vertex[i], feature.Vertices[j]);
      // 裏の特徴を作成
      reverseVertex(feature.Vertices[j], feature.Vertices[j + 1]);
    }

  for (i = 0; i < rtvcm.ncircle; i++)
    {
      j = i * 2;
      // 表の特徴を作成
      convertCircle(rtvcm.circle[i], feature.Circles[j]);
      // 裏の特徴を作成
      reverseCircle(feature.Circles[j], feature.Circles[j + 1]);
    }

  return 0;
}
