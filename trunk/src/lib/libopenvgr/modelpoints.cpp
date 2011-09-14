/*
 modelpoints.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file modelpoints.cpp
 * @brief モデル評価点生成関連関数
 * @date \$Date::                            $
 */
#include <math.h>
#include <cv.h>
#include <highgui.h>

#include "common.h"
#include "quaternion.h"
#include "circle.h"
#include "stereo.h"
#include "match3Dfeature.h"
#include "score2d.h"
#include "vectorutil.h"
#include "modelpoints.h"
#include "drawing.hpp"
#include "mathmisc.hpp"

//#define PROJECT_DEBUG

using namespace ovgr;

typedef enum
{
  AZIMUTH_S = 0,                // 南
  AZIMUTH_SE = 1,               // 南東
  AZIMUTH_E = 2,                // 東
  AZIMUTH_NE = 3,               // 北東
  AZIMUTH_N = 4,                // 北
  AZIMUTH_NW = 5,               // 北西
  AZIMUTH_W = 6,                // 西
  AZIMUTH_SW = 7,               // 南西
  AZIMUTH_NONE = -1
} Azimuth;

static void
setOneAtEnd(double v[4])
{
  v[3] = 1.0;
}

// 円の評価点を生成する
static int
makeCirclePoints(Circle* cir, double pdist)
{
  Trace* points;
  P3D* transformed;
  P2D* projected;
  double len;
  int np, i, j;
  double axis[2][3];


  // 点間隔パラメータが不適当な時は終了
  if (pdist <= 0)
    {
      return VISION_PARAM_ERROR;
    }

  len = 2.0 * M_PI * cir->radius;
  np = (int) (len / pdist) + 1;
  // 点数が少なすぎるときは終了
  if (np < 3)
    {
      return VISION_PARAM_ERROR;
    }

  points = (Trace*) calloc(np, sizeof(Trace));
  if (points == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  transformed = (P3D*) calloc(np, sizeof(P3D));
  if (transformed == NULL)
    {
      free(points);
      return VISION_MALLOC_ERROR;
    }

  projected = (P2D*) calloc(np, sizeof(P2D));
  if (projected == NULL)
    {
      free(points);
      free(transformed);
      return VISION_MALLOC_ERROR;
    }

  cir->numOfTracePoints = np;
  cir->tracepoints = points;
  cir->transformed = transformed;
  cir->projected = projected;

  calc_3d_axes_of_circle(axis[0], axis[1], cir->normal, NULL);
  for (i = 0; i < np; ++i)
    {
      double theta = (double)i / (double)(np - 1) * M_PI * 2.0;
      for (j = 0; j < 3; ++j)
        {
          points[i].xyz[j] = cir->radius * (axis[0][j] * cos(theta) + axis[1][j] * sin(theta));
        }
      points[i].xyz[3] = 1.0;
    }

  return np;
}

// 頂点を構成する２線分の評価点を生成する
static int
makeVertexPoints(Vertex* ver, double pdist)
{
  Trace* points;
  P3D* transformed;
  P2D* projected;
  double len1, len2, len;
  double ep1[3], ep2[3], dir[3];
  unsigned int np, np1, np2, i;

  // 点間隔パラメータが不適当な時は終了
  if (pdist <= 0)
    {
      return VISION_PARAM_ERROR;
    }

  mulV3S(-1.0, ver->endpoint1, ep1);
  copyV3(ver->endpoint2, ep2);
  len1 = getNormV3(ep1);
  len2 = getNormV3(ep2);
  // 分割数をもとめる
  np1 = (unsigned int) (len1 / pdist);
  len = (double) np1 * pdist;
  // 逆算でもとの長さに満たないときは分割数をひとつ増やす
  if (len < len1)
    {
      ++np1;
    }
  // 点の数は分割数+1
  ++np1;
  // np2 も np1 と同様にもとめる
  np2 = (unsigned int) (len2 / pdist);
  len = (double) np2 * pdist;
  if (len < len2)
    {
      ++np2;
    }
  ++np2;
  np = np1 + np2;

  // 点数が少なすぎるときは終了
  if (np < 3)
    {
      return VISION_PARAM_ERROR;
    }

  points = (Trace*) calloc(np, sizeof(Trace));
  if (points == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  transformed = (P3D*) calloc(np, sizeof(P3D));
  if (transformed == NULL)
    {
      free(points);
      return VISION_MALLOC_ERROR;
    }

  projected = (P2D*) calloc(np, sizeof(P2D));
  if (projected == NULL)
    {
      free(points);
      free(transformed);
      return VISION_MALLOC_ERROR;
    }

  ver->numOfTracePoints = np;
  ver->tracepoints = points;
  ver->transformed = transformed;
  ver->projected = projected;

  // 点間隔毎の位置ベクトル増分値を求める
  normalizeV3(ep1, dir);
  dir[0] *= pdist;
  dir[1] *= pdist;
  dir[2] *= pdist;
  // endpoint1 から一定間隔の点列を作る
  copyV3(ver->endpoint1, points[0].xyz);
  setOneAtEnd(points[0].xyz);
  for (i = 1; i < np1 - 1; i++)
    {
      addV3(points[i - 1].xyz, dir, points[i].xyz);
      // 同次行列乗算のため末尾 1 のベクトルとする
      setOneAtEnd(points[i].xyz);
    }
  // 点列の最後が頂点になるようにする
  for (i = 0; i < 3; ++i)
    {
      points[np1 - 1].xyz[i] = 0.0;
    }
  setOneAtEnd(points[np1 - 1].xyz);

  // 点間隔毎の位置ベクトル増分値を求める
  normalizeV3(ep2, dir);
  dir[0] *= pdist;
  dir[1] *= pdist;
  dir[2] *= pdist;
  // position から一定間隔の点列を作る
  for (i = 0; i < 3; ++i)
    {
      points[np1].xyz[i] = 0.0;
    }
  setOneAtEnd(points[np1].xyz);
  for (i = np1 + 1; i < np - 1; i++)
    {
      addV3(points[i - 1].xyz, dir, points[i].xyz);
      // 同次行列乗算のため末尾 1 のベクトルとする
      setOneAtEnd(points[i].xyz);
    }
  // 点列の最後が endpoint2 になるようにする
  copyV3(ver->endpoint2, points[np - 1].xyz);
  setOneAtEnd(points[np - 1].xyz);

  return np;
}

// Prev(x,y) -> Curr(x,y) の方向（８方向）
static Azimuth
getDirection(double* Curr, double* Prev)
{
  double dx, dy;
  double theta;
  int index = -1;

  dx = Curr[0] - Prev[0];
  dy = Curr[1] - Prev[1];

  if (isZero(dx*dx + dy*dy))
    {
      return AZIMUTH_NONE;
    }

  theta = atan2(dy, dx) * 8.0 / 2.0 / M_PI;
  index = (int)fmod(theta + 8.5, 8.0); // E:0 S:2 W:4 N:6
  
  return (Azimuth)((8 - index + 2) % 8);
}

// 視線ベクトルの計算
static void
calcPointDirection(double sight[3], const CameraParam* cp, const double point[3])
{
  int i, j;

  for (i = 0; i < 3; ++i)
    {
      sight[i] = cp->Translation[i];
      for (j = 0; j < 3; ++j)
        {
          sight[i] += cp->Rotation[i][j] * point[j];
        }
    }
  normalizeV3(sight, sight);

  return;
}

inline static bool
is_visible(const CameraParam *cp, double matrix[4][4], double pos[3], double normal[3])
{
  double vw[3], nw[3], v[3], n[3], dot, norm2;
  int i, j;

  for (i = 0; i < 3; ++i)
    {
      vw[i] = matrix[i][3];
      nw[i] = 0.0;

      for (j = 0; j < 3; ++j)
        {
          vw[i] += matrix[i][j] * pos[j];
          nw[i] += matrix[i][j] * normal[j];
        }
    }

  for (i = 0; i < 3; ++i)
    {
      v[i] = cp->Translation[i];
      n[i] = 0.0;

      for (j = 0; j < 3; ++j)
        {
          v[i] += cp->Rotation[i][j] * vw[j];
          n[i] += cp->Rotation[i][j] * nw[j];
        }
    }

  dot = 0.0;
  norm2 = 0.0;
  for (i = 0; i < 3; ++i)
    {
      dot += v[i] * n[i];
      norm2 = v[i] * v[i];
    }

  return dot / sqrt(norm2) < -VISION_EPS;
}

// 頂点の可視判定
static inline int
isVisibleVertex(Features3D* model, double matrix[4][4], Vertex* vertex, int p_camera)
{
  CameraParam* cameraParam;

  switch (p_camera)
    {
    case 0:
      cameraParam = &model->calib->CameraL;
      break;
    case 1:
      cameraParam = &model->calib->CameraR;
      break;
    case 2:
      cameraParam = &model->calib->CameraV;
      break;
    default:
      cameraParam = &model->calib->CameraL;
      break;
    }

  return is_visible(cameraParam, matrix, vertex->orientation[3], vertex->orientation[2]);
}

// 3次元点の2次元画像上への投影
static void
projectXYZ2LRwithTrans(Features3D* model, double matrix[4][4], int p_camera,
                       double xyz[3], cv::Point* colrow)
{
  cv::Mat RTmat = cv::Mat(4, 4, CV_64FC1, matrix);
  double psrc[4], pdst[4];
  cv::Mat src = cv::Mat(4, 1, CV_64FC1, psrc);
  cv::Mat dst = cv::Mat(4, 1, CV_64FC1, pdst);
  CameraParam* cameraParam;
  Data_2D iPos;

  switch (p_camera)
    {
    case 0:
      cameraParam = &model->calib->CameraL;
      break;
    case 1:
      cameraParam = &model->calib->CameraR;
      break;
    case 2:
      cameraParam = &model->calib->CameraV;
      break;
    default:
      cameraParam = &model->calib->CameraL;
      break;
    }

  psrc[0] = xyz[0];
  psrc[1] = xyz[1];
  psrc[2] = xyz[2];
  psrc[3] = 1.0;
  dst = RTmat * src;

  projectXYZ2LR(&iPos, pdst, cameraParam);
  colrow->x = floor(iPos.col + 0.5);
  colrow->y = floor(iPos.row + 0.5);

  return;
}

static void
mult_orientation(double result[3], double orientation[4][4], const double pos[3])
{
  int i, j;

  for (i = 0; i < 3; ++i)
    {
      result[i] = orientation[3][i];
      for (j = 0; j < 3; ++j)
        {
          result[i] += orientation[j][i] * pos[j];
        }
    }
}

// 頂点の２線分の評価点の画像投影
static void
projectVertexPoint(Features3D* model, double matrix[4][4], Vertex& vertex, int p_camera)
{
  int num, i;
  double pos3d[3];
  cv::Point pos2d[3];
  CameraParam* cameraParam;

  switch (p_camera)
    {
    case 0:
      cameraParam = &model->calib->CameraL;
      break;
    case 1:
      cameraParam = &model->calib->CameraR;
      break;
    case 2:
      cameraParam = &model->calib->CameraV;
      break;
    default:
      cameraParam = &model->calib->CameraL;
      break;
    }

  num = vertex.numOfTracePoints;

  mult_orientation(pos3d, vertex.orientation, vertex.endpoint1);
  projectXYZ2LRwithTrans(model, matrix, p_camera, pos3d, &pos2d[0]);

  projectXYZ2LRwithTrans(model, matrix, p_camera, vertex.orientation[3], &pos2d[1]);

  mult_orientation(pos3d, vertex.orientation, vertex.endpoint2);
  projectXYZ2LRwithTrans(model, matrix, p_camera, pos3d, &pos2d[2]);

  for (i = 0; i < num/2; ++i)
    {
      double alpha = (double)i / (double)(num / 2);

      vertex.tracepoints[i].colrow[0] = (1.0 - alpha) * pos2d[0].x + alpha * pos2d[1].x;
      vertex.tracepoints[i].colrow[1] = (1.0 - alpha) * pos2d[0].y + alpha * pos2d[1].y;
    }
  for (i = num/2; i < num; ++i)
    {
      double alpha = (double)(i - num/2) / (double)(num - 1 - num/2);

      vertex.tracepoints[i].colrow[0] = (1.0 - alpha) * pos2d[1].x + alpha * pos2d[2].x;
      vertex.tracepoints[i].colrow[1] = (1.0 - alpha) * pos2d[1].y + alpha * pos2d[2].y;
    }

  return;
}

// 裏向きの円の観測可能な稜線の範囲算出
static int
calc_observable_angle(double angle[2], double matrix[4][4], const Circle& circle, const CameraParam* cp)
{
  int i, j, k;

  cv::Mat M(4, 4, CV_64FC1), T(4, 4, CV_64FC1, matrix), model(4, 4, CV_64FC1, const_cast<double (*)[4]>(circle.orientation));

  M = T * model.t(); // モデルの行列は転置されている..

  // 外部パラメータの算出
  double ext[3][4];
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          ext[i][j] = 0.0;
          for (k = 0; k < 3; ++k)
            {
              ext[i][j] += cp->Rotation[i][k] * M.at<double>(k, j);
            }
        }

      ext[i][3] = cp->Translation[i];
      for (j = 0; j < 3; ++j)
        {
          ext[i][3] += cp->Rotation[i][j] * M.at<double>(j, 3);
        }
    }

#if 0
  for (i = 0; i < 4; ++i)
    {
      for (j = 0; j < 4; ++j)
        {
          printf("% 10.3g ", T.at<double>(i, j));
        }
      printf("\n");
    }
  printf("\n");
#endif

  angle[0] = angle[1] = 0.0;

  // 視線ベクトルと円の法線が直交する角度を計算
  double a = 0.0, b = 0.0, c[2] = {0.0, 0.0};
  int num = 0;
  for (i = 0; i < 3; ++i)
    {
      a += ext[i][0] * ext[i][3];
      b += ext[i][1] * ext[i][3];
    }

  num = solve_quad_eq(c, a*a + b*b, 2.0*a * circle.radius, circle.radius * circle.radius - b*b);

  if (num < 1)
    {
      return num;
    }

  // sinの符号から角度を一意に決める
  double s[2] = {0.0, 0.0};
  for (i = 0; i < num; ++i)
    {
      s[i] = -(circle.radius + a*c[i]) / b;
      angle[i] = (s[i] >= 0.0) ? acos(c[i]) : -acos(c[i]);
    }

  // angle[0]からangle[1]の間が観測される範囲
  if (circle.radius - a*s[0] + b*c[0] > 0.0)
    {
      double temp = angle[0];
      angle[0] = angle[1];
      angle[1] = temp;
    }

  // angle[0] <= angle[1]となるようにする
  if ((angle[1] < 0) && (angle[0] > angle[1]))
    {
      angle[1] += 2.0 * M_PI;
    }

  return num;
}

// 円の評価点の画像投影
static void
projectCirclePoint(Features3D* model, double matrix[4][4], Circle& circle, int p_camera)
{
  CvMat RTmat, src, dst;
  double hvec[4], pdata[4], dot, len, vec[4], center[4];
  int i, numOfpoints;
  CameraParam* cameraParam;
  Data_2D iPos;

  switch (p_camera)
    {
    case 0:
      cameraParam = &model->calib->CameraL;
      break;
    case 1:
      cameraParam = &model->calib->CameraR;
      break;
    case 2:
      cameraParam = &model->calib->CameraV;
      break;
    default:
      cameraParam = &model->calib->CameraL;
      break;
    }

  numOfpoints = circle.numOfTracePoints;

  RTmat = cvMat(4, 4, CV_64FC1, matrix);
  for (i = 0; i < numOfpoints; i++)
    {
      mult_orientation(hvec, circle.orientation, circle.tracepoints[i].xyz);
      hvec[3] = 1.0;
      src = cvMat(4, 1, CV_64FC1, hvec);
      dst = cvMat(4, 1, CV_64FC1, pdata);
      cvMatMul(&RTmat, &src, &dst);
      copyV3(pdata, circle.transformed[i].xyz);
    }
  for (i = 0; i < numOfpoints; i++)
    {
      projectXYZ2LR(&iPos, circle.transformed[i].xyz, cameraParam);
      circle.tracepoints[i].colrow[0] = iPos.col;
      circle.tracepoints[i].colrow[1] = iPos.row;
    }

  // 法線による可視判定
  // カメラ視線ベクトル
  copyV3(circle.orientation[3], hvec);
  setOneAtEnd(hvec);
  src = cvMat(4, 1, CV_64FC1, hvec);
  dst = cvMat(4, 1, CV_64FC1, pdata);
  cvMatMul(&RTmat, &src, &dst);
  copyV3(pdata, center);
  calcPointDirection(vec, cameraParam, pdata);

  // 法線
  copyV3(circle.orientation[2], hvec);
  hvec[3] = 0.0;
  src = cvMat(4, 1, CV_64FC1, hvec);
  dst = cvMat(4, 1, CV_64FC1, pdata);
  cvMatMul(&RTmat, &src, &dst);
  dot = getInnerProductV3(pdata, vec);
  if (dot < 0.0) // VISION_EPSを使う？
    {
      for (i = 0; i < numOfpoints; i++)
        {
          circle.tracepoints[i].label = VISIBLE;
        }
      return;
    }

  for (i = 0; i < numOfpoints; i++)
    {
      // 中心から各点へのベクトル
      pdata[0] = circle.transformed[i].xyz[0] - center[0];
      pdata[1] = circle.transformed[i].xyz[1] - center[1];
      pdata[2] = circle.transformed[i].xyz[2] - center[2];
      pdata[3] = 0.0;
      dot = getInnerProductV3(pdata, vec);
      len = getNormV3(pdata);
      if (dot / len < 0.0) // VISION_EPSを使う？
        {
          circle.tracepoints[i].label = VISIBLE;
        }
      else
        {
          circle.tracepoints[i].label = INVISIBLE;
        }
    }
  return;
}

// 各モデルの評価点を画像に投影して近傍を探索し、２次元評価値を算出する
static double
traceModelPoints(Features3D* model, int p_camera, double matrix[4][4])
{
  Vertex vertex;
  Circle circle;
  Trace* trace;
  double* curr;
  double* prev;
  double mpoint;
  double tpoint;
  double score;
  double dx, dy, dlen;
  double thr_pdist2 = model->trace_pdist * model->trace_pdist;
  int i, j, jj;
  int sts;
  int search = model->trace_search;
  int edge = model->trace_edge;

  tpoint = 0.0;
  mpoint = 0.0;
  score = 0.0;

  for (i = 0; i < model->numOfVertices; i++)
    {
      vertex = model->Vertices[i];
      // 裏側は表側と同じ点列なのでスキップ
      if (vertex.side == M3DF_BACK)
        {
          continue;
        }
      if (vertex.numOfTracePoints == 0)
        {
          continue;
        }
      if (isVisibleVertex(model, matrix, &vertex, p_camera) != VISIBLE)
        {
          continue;
        }

      projectVertexPoint(model, matrix, vertex, p_camera);

      trace = &vertex.tracepoints[0];
      trace->direction = AZIMUTH_NONE;
      jj = 0;
      for (j = 1; j < vertex.numOfTracePoints; j++)
        {
          trace = &vertex.tracepoints[j];
          curr = vertex.tracepoints[j].colrow;
          prev = vertex.tracepoints[jj].colrow;
          dx = curr[0] - prev[0];
          dy = curr[1] - prev[1];
          dlen = dx * dx + dy * dy;
          if (dlen < thr_pdist2)
            {
              continue;
            }
          mpoint += 1.0;
          trace->direction = getDirection(curr, prev);
          sts = tracePoint(model, trace, search, edge, p_camera);
          if (sts == 0)
            {
              tpoint += 1.0;
              score += 1.0 / (double) (trace->search + 1);
            }
          jj = j;
        }
    }

  for (i = 0; i < model->numOfCircles; i++)
    {
      circle = model->Circles[i];
      // 裏側は表側と同じ点列なのでスキップ
      if (circle.side == M3DF_BACK)
        {
          continue;
        }
      if (circle.numOfTracePoints == 0)
        {
          continue;
        }

      projectCirclePoint(model, matrix, circle, p_camera);
      trace = &circle.tracepoints[0];
      trace->direction = AZIMUTH_NONE;
      jj = 0;
      for (j = 1; j < circle.numOfTracePoints; j++)
        {
          if (circle.tracepoints[j].label == INVISIBLE)
            {
              jj = j;
              continue;
            }
          trace = &circle.tracepoints[j];
          curr = circle.tracepoints[j].colrow;
          prev = circle.tracepoints[jj].colrow;
          dx = curr[0] - prev[0];
          dy = curr[1] - prev[1];
          dlen = dx * dx + dy * dy;
          if (dlen < thr_pdist2)
            {
              continue;
            }
          mpoint += 1.0;
          trace->direction = getDirection(curr, prev);
          sts = tracePoint(model, trace, search, edge, p_camera);
          if (sts == 0)
            {
              tpoint += 1.0;
              score += 1.0 / (double) (trace->search + 1);
            }
          jj = j;
        }
    }

  model->traceCounts = tpoint;
  model->pointCounts = (int) mpoint;

  return score;
}

inline static void
calc_xyz_of_circle(double xyz[3], const Circle& circle, const double angle)
{
  int i;

  for (i = 0; i < 3; ++i)
    {
      xyz[i] = circle.radius * (circle.orientation[0][i] * cos(angle)
                                + circle.orientation[1][i] * sin(angle))
        + circle.orientation[3][i];
    }
}

// モデル評価点の描画（認識結果確認表示用）
void
drawModelPoints(Features3D* model,     // モデルの３次元特徴情報
                double matrix[4][4],   // 認識結果の位置姿勢変換行列
                char* filename,        // 結果表示の出力ファイル
                int p_camera,          // 結果表示する画像のカメラ番号
                unsigned char* img,    // 結果表示用の画像データ
                int lineThickness)     // 描画する線の太さ
{
  Vertex vertex;
  int i, j;
  cv::Mat cimg;
  cv::Scalar color;
  cv::Point p1, p2, p3;

  color = cv::Scalar(0, 255, 0);
  cimg = cv::Mat(model->calib->rowsize, model->calib->colsize, CV_8UC3, img);

  for (i = 0; i < model->numOfVertices; i++)
    {
      double pos3d[3];

      vertex = model->Vertices[i];
      if (vertex.side == M3DF_BACK)
        {
          continue;
        }
      if (isVisibleVertex(model, matrix, &vertex, p_camera) != VISIBLE)
        {
          continue;
        }

      // 頂点および端点の投影
      projectXYZ2LRwithTrans(model, matrix, p_camera, vertex.orientation[3], &p1);

      mult_orientation(pos3d, vertex.orientation, vertex.endpoint1);
      projectXYZ2LRwithTrans(model, matrix, p_camera, pos3d, &p2);

      mult_orientation(pos3d, vertex.orientation, vertex.endpoint2);
      projectXYZ2LRwithTrans(model, matrix, p_camera, pos3d, &p3);

      cv::line(cimg, p1, p2, color, lineThickness, CV_AA);
      cv::line(cimg, p1, p3, color, lineThickness, CV_AA);
    }

  {
    CameraParam *cp = NULL;

    switch (p_camera)
      {
      case 0:
        cp = &model->calib->CameraL;
        break;
      case 1:
        cp = &model->calib->CameraR;
        break;
      case 2:
        cp = &model->calib->CameraV;
        break;
      default:
        cp = &model->calib->CameraL;
        break;
      }

    cv::Point pos[2][2];
    Circle *circle[2];
    double angle[2][2];
    int nangle[2] = {0, 0};
    int n = 0;
    for (i = 0; i < model->numOfCircles; i++)
      {
        double xyz[4];

        circle[n] = &model->Circles[i];

        if (circle[n]->side == M3DF_BACK)
          {
            continue;
          }

        // 遮蔽輪郭線と楕円の交点を求める
        nangle[n] = calc_observable_angle(angle[n], matrix, *circle[n], cp);
        for (j = 0; j < 2; ++j)
          {
            calc_xyz_of_circle(xyz, *circle[n], angle[n][j]);
            xyz[3] = 1.0;
            
            projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &pos[n][j]);
          }

        // 円一つの場合
        if (model->numOfCircles == 1
            && is_visible(cp, matrix, circle[n]->orientation[3], circle[n]->orientation[2]))
          {
            const int num = circle[n]->numOfTracePoints;
            cv::Point curve[2];
            
            calc_xyz_of_circle(xyz, *circle[n], 0.0);
            xyz[3] = 1.0;
            projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[0]);

            for (j = 1; j <= num; ++j)
              {
                double t = 2.0 * M_PI * (double)j / (double)num;

                calc_xyz_of_circle(xyz, *circle[n], t);
                projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[j%2]);

                cv::line(cimg, curve[(j+1)%2], curve[j%2], color, lineThickness, CV_AA);
              }
          }
          
        // 円筒の描画
        if (n == 1)
          {
            for (j = 0; j < 2; ++j)
              {
                const int num = circle[j]->numOfTracePoints;
                cv::Point curve[2];
                int k;
                    
                if (is_visible(cp, matrix, circle[j]->orientation[3], circle[j]->orientation[2]))
                  {
                    // 円が見える場合
                    calc_xyz_of_circle(xyz, *circle[j], 0.0);
                    xyz[3] = 1.0;
                    projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[0]);

                    for (k = 1; k <= num; ++k)
                      {
                        double t = 2.0 * M_PI * (double)k / (double)num;

                        calc_xyz_of_circle(xyz, *circle[j], t);
                        projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[k%2]);

                        cv::line(cimg, curve[(k+1)%2], curve[k%2], color, lineThickness, CV_AA);
                      }
                  }
                else if (nangle[j] == 2)
                  {
                    // 裏向きの場合は手前側の円弧のみ描画する
                    const double delta = angle[j][1] - angle[j][0];
                    const int pnum = (int)(ceil((double)num * (delta / 2.0 / M_PI)));

                    calc_xyz_of_circle(xyz, *circle[j], angle[j][0]);
                    xyz[3] = 1.0;
                    projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[0]);
                    
                    for (k = 1; k <= pnum; ++k)
                      {
                        double t = 2.0 * M_PI * (double)k / (double)num + angle[j][0];

                        calc_xyz_of_circle(xyz, *circle[j], t);
                        projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[k%2]);

                        cv::line(cimg, curve[(k+1)%2], curve[k%2], color, lineThickness, CV_AA);
                      }
                  }
              }

            if (nangle[0] == 2 && nangle[1] == 2)
              {
                // 遮蔽輪郭線(円筒の側面)を描画
                cv::line(cimg, pos[0][0], pos[1][1], color, lineThickness, CV_AA);
                cv::line(cimg, pos[0][1], pos[1][0], color, lineThickness, CV_AA);
              }
          }

        n = (n + 1) % 2;
      }
  }

  // 元のデータに書き戻す。
  if (filename)
    {
      cv::imwrite(filename, cimg);
    }

  return;
}

// モデルの評価点の生成
// 戻り値：総評価点数
int
makeModelPoints(Features3D* model,    // モデルの３次元特徴データ
                double pdist)         // 評価点間隔
{
  int i, count, totalcount = 0;

  for (i = 0; i < model->numOfVertices; i++)
    {
      // 裏側は表側と同じ点列なのでスキップ
      if (model->Vertices[i].side == M3DF_BACK)
        {
          continue;
        }

      model->Vertices[i].numOfTracePoints = ceil((getNormV3(model->Vertices[i].endpoint1) + getNormV3(model->Vertices[i].endpoint2)) / pdist);
      count = model->Vertices[i].numOfTracePoints;

      totalcount += count;
    }

  for (i = 0; i < model->numOfCircles; i++)
    {
      // 裏側は表側と同じ点列なのでスキップ
      if (model->Circles[i].side == M3DF_BACK)
        {
          continue;
        }

      model->Circles[i].numOfTracePoints = (int) (model->Circles[i].radius * 2.0 * M_PI / pdist) + 1;
      count = model->Circles[i].numOfTracePoints;

      totalcount += count;
    }

  return totalcount;
}

// 合同変換行列を位置ベクトルと回転ベクトルを合わせた7次元ベクトルに変換する
void
getPropertyVector(double mat[4][4],            // 合同変換行列
                  double vec[7])               // ７次元ベクトル
{
  int i;

  for (i = 0; i < 3; i++)
    {
      // 位置ベクトルをコピー
      vec[i] = mat[i][3];
    }

  quat_q_from_R(&vec[3], mat[0], 4);

  return;
}

// 使用した全画像を用いた２次元評価値計算
// 戻り値：２次元評価値
double
traceModelPointsMultiCameras(Features3D* model,        // モデルの３次元特徴情報
                             StereoPairing& pairing,   // ステレオペア情報
                             double matrix[4][4])      // 認識結果の位置姿勢変換行列
{
  double score = 0.0;

  switch (pairing)
    {
    case DBL_LR:
      score = traceModelPoints(model, 0, matrix);
      score += traceModelPoints(model, 1, matrix);
      break;

    case DBL_LV:
      score = traceModelPoints(model, 0, matrix);
      score += traceModelPoints(model, 2, matrix);
      break;

    case DBL_RV:
      score = traceModelPoints(model, 1, matrix);
      score += traceModelPoints(model, 2, matrix);
      break;

    case TBL_OR:
      score = traceModelPoints(model, 0, matrix);
      score += traceModelPoints(model, 1, matrix);
      score += traceModelPoints(model, 2, matrix);
      break;

    case TBL_AND:
      score = traceModelPoints(model, 0, matrix);
      break;
    }

  return score;
}

static double
calcEvaluationValue2D_on_line(Features3D* model, const cv::Point& p1, const cv::Point& p2, const cv::Mat& dstImage)
{
  ovgr::PointsOnLine points(p1.x, p1.y, p2.x, p2.y);
  double score = 0.0;

  do
    {
      int pcol, prow;

      pcol = points.x();
      prow = points.y();
      if (isValidPixelPosition(pcol, prow, model))
        {
          float dist_value = dstImage.at<float>(prow, pcol);
          score += 1.0 / (double) (dist_value + 1.0);
        }
    }
  while (points.next());

  return score;
}

// 各モデルの評価点を画像に投影して距離変換画像を参照し、２次元評価値を算出する
static double
calcEvaluationValue2D(Features3D* model, int p_camera, double matrix[4][4],
                      const cv::Mat& dstImage)
{
  Vertex vertex;
  double score;
  int i, j;

#ifdef PROJECT_DEBUG
  cv::Mat dstImage_norm = 
    cv::Mat::zeros(cv::Size(model->calib->colsize, model->calib->rowsize), CV_8UC1);
  cv::normalize(dstImage, dstImage_norm, 0, 1, CV_MINMAX);
  cv::Mat dstImage_color = 
    cv::Mat::zeros(cv::Size(model->calib->colsize, model->calib->rowsize), CV_8UC3);
  cv::cvtColor(dstImage_norm, dstImage_color, CV_GRAY2RGB);

  cv::Scalar color = CV_RGB(0, 255, 0);
  double lineThickness = 1.0;
#endif

  score = 0.0;

#ifdef _OPENMP
#pragma omp parallel for private (vertex, j), reduction (+: score)
#endif
  for (i = 0; i < model->numOfVertices; i++)
    {
      vertex = model->Vertices[i];

      // 裏側は表側と同じ点列なのでスキップ
      if (vertex.side == M3DF_BACK)
        {
          continue;
        }
      if (isVisibleVertex(model, matrix, &vertex, p_camera) != VISIBLE)
        {
          continue;
        }

      double pos3d[3];
      cv::Point pos2d[3];

      mult_orientation(pos3d, vertex.orientation, vertex.endpoint1);
      projectXYZ2LRwithTrans(model, matrix, p_camera, pos3d, &pos2d[0]);

      projectXYZ2LRwithTrans(model, matrix, p_camera, vertex.orientation[3], &pos2d[1]);

      mult_orientation(pos3d, vertex.orientation, vertex.endpoint2);
      projectXYZ2LRwithTrans(model, matrix, p_camera, pos3d, &pos2d[2]);

      score += (calcEvaluationValue2D_on_line(model, pos2d[0], pos2d[1], dstImage)
                + calcEvaluationValue2D_on_line(model, pos2d[1], pos2d[2], dstImage));
# ifdef PROJECT_DEBUG
      cv::line(dstImage_color, pos2d[0], pos2d[1], color, lineThickness, CV_AA);
      cv::line(dstImage_color, pos2d[1], pos2d[2], color, lineThickness, CV_AA);
# endif
    }

  {
    CameraParam *cp = NULL;

    switch (p_camera)
      {
      case 0:
        cp = &model->calib->CameraL;
        break;
      case 1:
        cp = &model->calib->CameraR;
        break;
      case 2:
        cp = &model->calib->CameraV;
        break;
      default:
        cp = &model->calib->CameraL;
        break;
      }

    cv::Point pos[2][2];
    Circle *circle[2];
    double angle[2][2];
    int nangle[2] = {0, 0};
    int n = 0;
    for (i = 0; i < model->numOfCircles; i++)
      {
        double xyz[4];

        circle[n] = &model->Circles[i];

        if (circle[n]->side == M3DF_BACK)
          {
            continue;
          }

        // 遮蔽輪郭線と楕円の交点を求める
        nangle[n] = calc_observable_angle(angle[n], matrix, *circle[n], cp);
        for (j = 0; j < 2; ++j)
          {
            calc_xyz_of_circle(xyz, *circle[n], angle[n][j]);
            xyz[3] = 1.0;
            
            projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &pos[n][j]);
          }

        // 円一つの場合
        if (model->numOfCircles == 1
            && is_visible(cp, matrix, circle[n]->orientation[3], circle[n]->orientation[2]))
          {
            const int num = circle[n]->numOfTracePoints;
            cv::Point curve[2];
            
            calc_xyz_of_circle(xyz, *circle[n], 0.0);
            xyz[3] = 1.0;
            projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[0]);

            for (j = 1; j <= num; ++j)
              {
                double t = 2.0 * M_PI * (double)j / (double)num;

                calc_xyz_of_circle(xyz, *circle[n], t);
                projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[j%2]);
# ifdef PROJECT_DEBUG
                cv::line(dstImage_color, curve[(j+1)%2], curve[j%2], color, lineThickness, CV_AA);
# endif
                score += calcEvaluationValue2D_on_line(model, curve[(j+1)%2], curve[j%2], dstImage);
              }
          }
          
        // 円筒の評価
        if (n == 1)
          {
            for (j = 0; j < 2; ++j)
              {
                const int num = circle[j]->numOfTracePoints;
                cv::Point curve[2];
                int k;
                    
                if (is_visible(cp, matrix, circle[j]->orientation[3], circle[j]->orientation[2]))
                  {
                    // 円が見える場合
                    calc_xyz_of_circle(xyz, *circle[j], 0.0);
                    xyz[3] = 1.0;
                    projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[0]);

                    for (k = 1; k <= num; ++k)
                      {
                        double t = 2.0 * M_PI * (double)k / (double)num;

                        calc_xyz_of_circle(xyz, *circle[j], t);
                        projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[k%2]);

# ifdef PROJECT_DEBUG
                        cv::line(dstImage_color, curve[(k+1)%2], curve[k%2], color, lineThickness, CV_AA);
# endif
                        score += calcEvaluationValue2D_on_line(model, curve[(k+1)%2], curve[k%2], dstImage);
                      }
                  }
                else if (nangle[j] == 2)
                  {
                    // 裏向きの場合は手前側の円弧のみ評価する
                    const double delta = angle[j][1] - angle[j][0];
                    const int pnum = (int)(ceil((double)num * (delta / 2.0 / M_PI)));

                    calc_xyz_of_circle(xyz, *circle[j], angle[j][0]);
                    xyz[3] = 1.0;
                    projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[0]);
                    
                    for (k = 1; k <= pnum; ++k)
                      {
                        double t = 2.0 * M_PI * (double)k / (double)num + angle[j][0];

                        calc_xyz_of_circle(xyz, *circle[j], t);
                        projectXYZ2LRwithTrans(model, matrix, p_camera, xyz, &curve[k%2]);

# ifdef PROJECT_DEBUG
                        cv::line(dstImage_color, curve[(k+1)%2], curve[k%2], color, lineThickness, CV_AA);
# endif
                        score += calcEvaluationValue2D_on_line(model, curve[(k+1)%2], curve[k%2], dstImage);
                      }
                  }
              }

            if (nangle[0] == 2 && nangle[1] == 2)
              {
# ifdef PROJECT_DEBUG
                cv::line(dstImage_color, pos[0][0], pos[1][1], color, lineThickness, CV_AA);
                cv::line(dstImage_color, pos[0][1], pos[1][0], color, lineThickness, CV_AA);
# endif
                // 遮蔽輪郭線(円筒の側面)を評価
                score += calcEvaluationValue2D_on_line(model, pos[0][0], pos[1][1], dstImage);
                score += calcEvaluationValue2D_on_line(model, pos[0][1], pos[1][0], dstImage);
              }
          }

        n = (n + 1) % 2;
      }
  }

#ifdef PROJECT_DEBUG
  cv::namedWindow ("Projection result", CV_WINDOW_AUTOSIZE);
  cv::imshow ("Projection result", dstImage_color);
  cv::imwrite("debug.png", dstImage_color);
  cv::waitKey (0);
#endif

  return score;
}

// 使用した全画像を用いた２次元評価値計算。距離変換画像の利用
// 戻り値：２次元評価値
double calcEvaluationValue2DMultiCameras(Features3D* model,      // モデルの３次元特徴情報
                                         StereoPairing& pairing, // ステレオペア情報
                                         double matrix[4][4],    // 認識結果の位置姿勢変換行列
                                         const std::vector<cv::Mat>& dstImages)  // 距離変換画像
{
  double score = 0.0;

  switch (pairing)
    {
    case DBL_LR:
      score = calcEvaluationValue2D(model, 0, matrix, dstImages[0]);
      score += calcEvaluationValue2D(model, 1, matrix, dstImages[1]);
      break;

    case DBL_LV:
      score = calcEvaluationValue2D(model, 0, matrix, dstImages[0]);
      score += calcEvaluationValue2D(model, 2, matrix, dstImages[2]);
      break;

    case DBL_RV:
      score = calcEvaluationValue2D(model, 1, matrix, dstImages[1]);
      score += calcEvaluationValue2D(model, 2, matrix, dstImages[2]);
      break;

    case TBL_OR:
    case TBL_AND:
      score = calcEvaluationValue2D(model, 0, matrix, dstImages[0]);
      score += calcEvaluationValue2D(model, 1, matrix, dstImages[1]);
      score += calcEvaluationValue2D(model, 2, matrix, dstImages[2]);
      break;
    }

  return score;
}
