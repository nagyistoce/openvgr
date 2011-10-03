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


//! 不要になった認識結果データの開放
void
shrinkMatch3Dresults(Match3Dresults* Match)
{
  MatchResult* memory = NULL;
  int num;

  for (num = 0; num < Match->numOfResults; ++num)
    {
      if (Match->Results[num].score < 0.0)
        {
          break;
        }
      Match->Results[num].n = num;
    }
  if (num == 0)
    {
      return;
    }
  if ((memory = (MatchResult*) realloc(Match->Results, num * sizeof(MatchResult))) == NULL)
    {
      return;
    }

  //fprintf(stderr, "shrank %d -> %d\n", Match->numOfResults, num);

  Match->Results = memory;
  Match->numOfResults = num;
}

// smat : 特徴の元姿勢行列 , dmat : 特徴の現姿勢行列 , matrix : 変換行列
static void
getRTmatrix(double smat[4][4], double dmat[4][4], double mat[4][4])
{
  int i, j, k;

  // mat = dmat^T (smat^T)^-1
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          mat[i][j] = 0.0;
          for (k = 0; k < 3; ++k)
            {
              mat[i][j] += dmat[k][i] * smat[k][j];
            }
        }
    }

  for (i = 0; i < 3; ++i)
    {
      mat[i][3] = dmat[3][i];
      for (j = 0; j < 3; ++j)
        {
          mat[i][3] -= mat[i][j] * smat[3][j];
        }
    }

  for (i = 0; i < 4; ++i)
    {
      mat[3][i] = (i < 3) ? 0.0 : 1.0;
    }

  return;
}

// 頂点照合：なす角の差が閾値以内なら、位置姿勢の回転行列を求める
static int
matchVertex(Vertex* scene, Vertex* model, double diffrate, double mat[4][4])
{
  double sangle, mangle, adif;

  // 評価不要のラベルがついているか調べる
  if (model->label & M3DF_LABEL_NOEVAL)
    {
      return -1;
    }

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

  getRTmatrix(model->tPose, scene->tPose, mat);

  return 0;
}

// 円照合：半径差が閾値以内なら、位置姿勢の回転行列を求める
static int
matchCircle(Circle* scene, Circle* model, double diffrate, double mat[4][4])
{
  double rdif;

  // 評価不要のラベルがついているか調べる
  if (model->label & M3DF_LABEL_NOEVAL)
    {
      return -1;
    }

  // 半径の異なる度合いを調べる
  rdif = 100.0 * fabs(model->radius - scene->radius) / model->radius;
  // 閾値より大きく異なるときは照合不能
  if (rdif > diffrate)
    {
      return -1;
    }

  getRTmatrix(model->tPose, scene->tPose, mat);

  return 0;
}

// 頂点による照合
static Match3Dresults
matchVertices(Features3D scene, Features3D model,
              const std::vector<cv::Mat>& dstImages,
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
      getResultScore(Match.Results, k, &model, pairing, dstImages, score_weight);

      shrinkMatch3Dresults(&Match);
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
             const std::vector<cv::Mat>& dstImages,
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
      getResultScore(memory, k, &model, pairing, dstImages, score_weight);

      shrinkMatch3Dresults(&Match);
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

// 距離変換画像の生成
static void
createDistanceTranceformImages(const Features3D& model, 
                               std::vector<cv::Mat>* dstImages)
{
  int numCameras;
  int i, j;

  cv::Mat src_img = cv::Mat::zeros(cv::Size(model.calib->colsize, model.calib->rowsize), CV_8UC1);

  numCameras = 3;
  for(i = 0; i < numCameras; i++)
    {
      cv::Mat dstImage = 
        cv::Mat::zeros(cv::Size(model.calib->colsize, model.calib->rowsize), CV_32FC1);

      if (model.edge[i] != NULL)
        {
          // エッジを0としてセットする
          for (j = 0; j < model.calib->rowsize * model.calib->colsize; j++)
            {
              if (model.edge[i][j] == 0)
                {
                  src_img.data[j] = 1;
                }
              else
                {
                  src_img.data[j] = 0;
                }
            }
          cv::distanceTransform(src_img, dstImage, CV_DIST_L1, 3);
        }

      dstImages->push_back(dstImage);
#if 0
      cv::Mat src_img_norm = 
        cv::Mat::zeros(cv::Size(model.calib->colsize, model.calib->rowsize), CV_8UC1);
      cv::Mat dst_img_norm = 
        cv::Mat::zeros(cv::Size(model.calib->colsize, model.calib->rowsize), CV_8UC1);
      cv::normalize((*dstImages)[i], dst_img_norm, 0.0, 1.0, CV_MINMAX);
      cv::normalize(src_img, src_img_norm, 0.0, 255.0, CV_MINMAX);
      cv::namedWindow("Source", CV_WINDOW_AUTOSIZE);
      cv::namedWindow("Distance Image", CV_WINDOW_AUTOSIZE);
      cv::imshow("Source", src_img_norm);
      cv::imshow("Distance Image", dst_img_norm);
      cv::waitKey(0);
#endif
    }

  return;
}

// 認識：シーン特徴とモデル特徴の照合
// 戻り値：認識結果
Match3Dresults
matchFeatures3D(Features3D& scene,             // シーンの３次元特徴情報
                Features3D& model,             // モデルの３次元特徴情報
                Parameters& parameters)        // 全パラメータ
{
  Match3Dresults Match = { 0 };
  Match3Dresults cir1Match = { 0 };
  Match3Dresults cir2Match = { 0 };
  double tolerance1 = parameters.match.tolerance1;
  double tolerance2 = parameters.match.tolerance2;
  StereoPairing pairing = parameters.pairing;
  std::vector<cv::Mat> dstImages; // 距離変換画像
  int error;

  // 認識結果評価用にモデルサンプル点を生成する
  model.pointCounts = makeModelPoints(&model, parameters.match.pdist);
  if (model.pointCounts < 0)
    {
      Match.error = model.pointCounts; // エラーコード
      return Match;
    }

  // 距離変換画像の生成
  createDistanceTranceformImages(model, &dstImages);

  if (scene.numOfVertices > 0 && model.numOfVertices > 0)
    {
      Match = matchVertices(scene, model, dstImages, tolerance1, pairing);
    }

  if (Match.error == 0 && scene.numOfCircles > 0 && model.numOfCircles > 1)
    {
      cir2Match = matchPairedCircles(scene, model, dstImages, tolerance2, pairing);
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
      cir1Match = matchCircles(scene, model, dstImages, tolerance2, pairing);
      error = mergeMatch3Dresults(&Match, &cir1Match);
      if (error)
        {
          freeMatch3Dresults(&cir1Match);
          Match.error = error;
        }
    }

  return Match;
}
