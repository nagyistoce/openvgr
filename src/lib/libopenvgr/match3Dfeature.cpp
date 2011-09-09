/*
 match3Dfeature.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file match3Dfeature.cpp
 * @brief 3次元特徴による認識関連関数
 * @date \$Date::                            $
 */
#include <math.h>
#include <cxtypes.h>
#include <cxcore.h>
#include <cv.h>
#include <highgui.h>

#include "parameters.h"
#include "calib.h"
#include "vectorutil.h"
#include "modelpoints.h"
#include "score2d.h"
#include "match3Dfeature.h"

// ３次元円情報のメモリ解放
static void
freeCirclePoints(Circle* cir)
{
  free(cir->tracepoints);
  free(cir->transformed);
  free(cir->projected);
  cir->tracepoints = NULL;
  cir->transformed = NULL;
  cir->projected = NULL;
  cir->numOfTracePoints = 0;
  return;
}

// ３次元頂点情報のメモリ解放
static void
freeVertexPoints(Vertex* ver)
{
  if (ver->tracepoints != NULL)
    {
      free(ver->tracepoints);
    }
  if (ver->transformed != NULL)
    {
      free(ver->transformed);
    }
  if (ver->projected != NULL)
    {
      free(ver->projected);
    }
  ver->tracepoints = NULL;
  ver->transformed = NULL;
  ver->projected = NULL;
  ver->numOfTracePoints = 0;
  return;
}

// ３次元特徴データのメモリ解放
void
freeFeatures3D(Features3D* feature)
{
  int i;

  for (i = 0; i < feature->numOfVertices; i++)
    {
      freeVertexPoints(&feature->Vertices[i]);
    }

  free(feature->Vertices);
  feature->Vertices = NULL;
  feature->numOfVertices = 0;

  for (i = 0; i < feature->numOfCircles; i++)
    {
      freeCirclePoints(&feature->Circles[i]);
    }

  free(feature->Circles);
  feature->Circles = NULL;
  feature->numOfCircles = 0;
  return;
}

// 認識結果データのメモリ解除
void
freeMatch3Dresults(Match3Dresults* holder)
{
  free(holder->Results);
  holder->Results = NULL;
  holder->numOfResults = 0;
  return;
}

// 原点から position に移動させる 4x4 行列を求める
static void
getMatrixMovingFromOrigin(double position[3], CvMat* T)
{
  cvSetIdentity(T);
  cvmSet(T, 0, 3, position[0]);
  cvmSet(T, 1, 3, position[1]);
  cvmSet(T, 2, 3, position[2]);
  return;
}

// positionを原点に移動させる 4x4 行列を求める
static void
getMatrixMovingToOrigin(double position[3], CvMat* T)
{
  cvSetIdentity(T);
  cvmSet(T, 0, 3, -position[0]);
  cvmSet(T, 1, 3, -position[1]);
  cvmSet(T, 2, 3, -position[2]);
  return;
}

// smat : 特徴の元姿勢行列 , dmat : 特徴の現姿勢行列 , src : 元位置, dst : 現位置, matrix : 変換行列
static void
getRTmatrix(double smat[4][4], double dmat[4][4], double src[3],
            double dst[3], double mat[4][4])
{
  CvMat Sr;                     // 元姿勢の逆行列
  CvMat Dr;                     // 現姿勢の逆行列
  CvMat W;                      // 作業用行列
  CvMat R;                      // 回転行列
  CvMat RT;                     // 4x4 回転平行移動行列
  CvMat s2o;                    // srcを原点へ移動させる平行移動行列
  CvMat o2d;                    // 原点からdstへ移動させる平行移動行列
  double Rdata[4][4];           // 回転行列データ配列
  double Wdata[4][4];           // 作業用行列データ配列
  double t1dat[4][4];           // 平行移動行列データ配列
  double t2dat[4][4];           // 平行移動行列データ配列

  // smat, dmat は本来の意味の転置行列（逆行列）になっている
  // 従って元姿勢行列を S, 現姿勢行列を D,
  // 元姿勢から現姿勢への回転を R とすると
  // D = R*S だから R = D*inv(S)
  // D -> inv(Dr), inv(S) -> Sr より 
  // R = inv(Dr)*Sr

  Sr = cvMat(4, 4, CV_64FC1, smat);
  Dr = cvMat(4, 4, CV_64FC1, dmat);
  W = cvMat(4, 4, CV_64FC1, Wdata);

  // dmatの逆行列を求める 回転行列なので転置でよい
  cvTranspose(&Dr, &W);

  // 元姿勢を現姿勢に変換する回転行列を求める
  R = cvMat(4, 4, CV_64FC1, Rdata);
  cvMatMul(&W, &Sr, &R);

  // src を原点に移動させる 4x4 行列を求める
  s2o = cvMat(4, 4, CV_64FC1, t1dat);
  getMatrixMovingToOrigin(src, &s2o);

  // 原点から dst に移動させる 4x4 行列を求める
  o2d = cvMat(4, 4, CV_64FC1, t2dat);
  getMatrixMovingFromOrigin(dst, &o2d);

  // W = R * s2o  元位置を原点に移動して回転
  cvMatMul(&R, &s2o, &W);

  // RT = o2d * R * s2o  さらに現位置へ移動
  RT = cvMat(4, 4, CV_64FC1, mat);
  cvMatMul(&o2d, &W, &RT);
  return;
}

// 頂点照合：なす角の差が閾値以内なら、位置姿勢の回転行列を求める
static int
matchVertex(Vertex* scene, Vertex* model, double diffrate, double mat[4][4])
{
  double sangle, mangle, adif;

  // 頂点を構成する線分の成す角を求める
  sangle = scene->angle;
  mangle = model->angle;

  // 成す角の異なる度合いを調べる
  adif = 100.0 * fabs(sangle - mangle) / mangle;
  // 閾値より大きく異なるときは照合不能
  if (adif > diffrate)
    {
      return -1;
    }

  getRTmatrix(model->orientation, scene->orientation, 
              model->position, scene->position, mat);

  return 0;
}

// 円照合：半径差が閾値以内なら、位置姿勢の回転行列を求める
static int
matchCircle(Circle* scene, Circle* model, double diffrate, double mat[4][4])
{
  double rdif;

  // 半径の異なる度合いを調べる
  rdif = 100.0 * fabs(model->radius - scene->radius) / model->radius;
  // 閾値より大きく異なるときは照合不能
  if (rdif > diffrate)
    {
      return -1;
    }

  getRTmatrix(model->orientation, scene->orientation,
              model->center, scene->center, mat);

  return 0;
}


// 頂点による照合
static Match3Dresults
matchVertices(Features3D scene, Features3D model,
              double tolerance, StereoPairing pairing)
{
  Match3Dresults Match = { 0 };
  MatchResult* memory = NULL;
  int i, j, status;
  size_t k, numOfVerResults;
  double score_weight = 1.0;    // 2円を差別化するために導入
  double maxnum = (double)scene.numOfVertices * (double)model.numOfVertices;

  if (maxnum > (double)INT_MAX)
    {
      numOfVerResults = INT_MAX/sizeof(MatchResult);
    }
  else
    {
      numOfVerResults = (size_t)maxnum;
    }

  if ((memory = (MatchResult*) calloc(numOfVerResults, sizeof(MatchResult))) == NULL)
    {
      Match.error = VISION_MALLOC_ERROR;
      return Match;
    }

  Match.Results = memory;
  k = 0;
  for (j = 0; j < model.numOfVertices; j++)
    {
      for (i = 0; i < scene.numOfVertices; i++)
        {
          status = matchVertex(&scene.Vertices[i], &model.Vertices[j], tolerance, Match.Results[k].mat);
          if (status == 0)
            {
              Match.Results[k].n = k;
              Match.Results[k].type = 0;
              Match.Results[k].score = 0.0;
              Match.Results[k].scene[0] = scene.Vertices[i].n;
              Match.Results[k].scene[1] = scene.Vertices[i].side;
              Match.Results[k].model[0] = model.Vertices[j].n;
              Match.Results[k].model[1] = model.Vertices[j].side;
              // 認識結果の行列を７次元のベクトル（位置＋回転）としてもあらわす
              getPropertyVector(Match.Results[k].mat, Match.Results[k].vec);
              if (++k >= numOfVerResults)
              {
                goto breakout;
              }
            }
        }
    }

breakout:
  Match.numOfResults = k;

  if (Match.numOfResults)
    {
      // 認識結果の評価値を作成する．完全重複した結果は評価しない．
      getResultScore(Match.Results, k, &model, pairing, score_weight);
    }
  else
    {
      freeMatch3Dresults(&Match);
    }

  return Match;
}

// 単円による照合
static Match3Dresults
matchCircles(Features3D scene, Features3D model,
	     double tolerance, StereoPairing pairing)
{
  Match3Dresults Match = { 0 };
  MatchResult* memory = NULL;
  int i, j, status;
  size_t k, numOfCirResults;
  double score_weight = 1.0;    // 2円を差別化するために導入
  double maxnum = (double)scene.numOfCircles * (double)model.numOfCircles;

  if (maxnum > (double)INT_MAX)
    {
      numOfCirResults = INT_MAX/sizeof(MatchResult);
    }
  else
    {
      numOfCirResults = (size_t)maxnum;
    }

  memory = (MatchResult*) calloc(numOfCirResults, sizeof(MatchResult));
  if (memory != NULL)
    {
      Match.Results = memory;
      k = 0;
      for (j = 0; j < model.numOfCircles; j++)
        {
          for (i = 0; i < scene.numOfCircles; i++)
            {
              status = matchCircle(&scene.Circles[i], &model.Circles[j],
                                   tolerance, Match.Results[k].mat);
              if (status == 0)
                {
                  Match.Results[k].n = k;
                  Match.Results[k].type = 1;
                  Match.Results[k].score = 0.0;
                  Match.Results[k].scene[0] = scene.Circles[i].n;
                  Match.Results[k].scene[1] = scene.Circles[i].side;
                  Match.Results[k].model[0] = model.Circles[j].n;
                  Match.Results[k].model[1] = model.Circles[j].side;
                  // 認識結果の行列を７次元のベクトル（位置＋回転）としてもあらわす
                  getPropertyVector(Match.Results[k].mat, Match.Results[k].vec);
                  if (++k >= numOfCirResults)
                    {
                      goto breakout;
                    }
                }
            }
        }
    breakout:
      Match.numOfResults = k;
      // 認識結果の評価値を作成する．完全重複した結果は評価しない．
      getResultScore(memory, k, &model, pairing, score_weight);
    }
  else
    {
      Match.error = VISION_MALLOC_ERROR;
    }

  if(Match.numOfResults == 0)
    {
      freeMatch3Dresults(&Match);
    }

  return Match;
}

// 認識結果のマージ
static int
mergeMatch3Dresults(Match3Dresults* base, Match3Dresults* append)
{
  MatchResult* memory;
  int numOfResults, copySize;

  if (base == NULL || append == NULL)
    {
      return VISION_PARAM_ERROR;
    }

  if (base->error || append->error)
    {
      return VISION_PARAM_ERROR;
    }

  if (append->numOfResults == 0)
    {
      return 0;
    }

  if (base->numOfResults == 0)
    {
      base->Results = append->Results;
      base->numOfResults = append->numOfResults;
      append->Results = NULL;
      append->numOfResults = 0;
      return 0;
    }

  numOfResults = base->numOfResults + append->numOfResults;

  memory = (MatchResult*) calloc(numOfResults, sizeof(MatchResult));
  if (memory)
    {
      copySize = base->numOfResults * sizeof(MatchResult);
      memcpy(memory, base->Results, copySize);
      copySize = append->numOfResults * sizeof(MatchResult);
      memcpy(&memory[base->numOfResults], append->Results, copySize);
      qsort(memory, numOfResults, sizeof(MatchResult), compareResultScore);
      freeMatch3Dresults(base);
      base->Results = memory;
      base->numOfResults = numOfResults;
      freeMatch3Dresults(append);
      return 0;
    }
  else
    {
      base->error = VISION_MALLOC_ERROR;
    }

  return VISION_MALLOC_ERROR;
}


// 認識：シーン特徴とモデル特徴の照合
// 戻り値：認識結果
Match3Dresults
matchFeatures3d(Features3D& scene,             // シーンの３次元特徴情報
                Features3D& model,             // モデルの３次元特徴情報
                unsigned char* edgeL,          // １番目の画像のエッジ画像
                unsigned char* edgeR,          // ２番目の画像のエッジ画像 
                unsigned char* edgeV,          // ３番目の画像のエッジ画像 
                Parameters& parameters)        // 全パラメータ
{
  Match3Dresults Match = { 0 };
  Match3Dresults cir1Match = { 0 };
  Match3Dresults cir2Match = { 0 };
  double tolerance1 = parameters.match.tolerance1;
  double tolerance2 = parameters.match.tolerance2;
  StereoPairing pairing = parameters.pairing;
  int error;

  // 認識結果評価用にモデルサンプル点を生成する
  model.pointCounts = makeModelPoints(&model, parameters.match.pdist);
  if (model.pointCounts < 0)
    {
      Match.error = model.pointCounts; // エラーコード
      return Match;
    }

  if (scene.numOfVertices > 0 && model.numOfVertices > 0)
    {
      Match = matchVertices(scene, model, tolerance1, pairing);
    }

  if (Match.error == 0 && scene.numOfCircles > 0 && model.numOfCircles > 2)
    {
      cir2Match = matchPairedCircles(scene, model, tolerance2, pairing);
      error = mergeMatch3Dresults(&Match, &cir2Match);
      if (error)
        {
          freeMatch3Dresults(&cir2Match);
          Match.error = error;
        }
    }

  if (cir2Match.numOfResults > 0)
    {
      return Match;
    }

  if (Match.error == 0 && scene.numOfCircles > 0 && model.numOfCircles > 0)
    {
      cir1Match = matchCircles(scene, model, tolerance2, pairing);
      error = mergeMatch3Dresults(&Match, &cir1Match);
      if (error)
        {
          freeMatch3Dresults(&cir1Match);
          Match.error = error;
        }
    }

  return Match;
}
