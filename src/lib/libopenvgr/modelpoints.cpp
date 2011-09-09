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
#include "stereo.h"
#include "match3Dfeature.h"
#include "score2d.h"
#include "vectorutil.h"
#include "modelpoints.h"

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

// 円の始点座標計算
int
getPointOnCircle(double normal[3], double radius, double point[3])
{
  CvMat avec, nvec, dvec;
  double data[3], dir[3];
  int i, sts;

  avec = cvMat(3, 1, CV_64FC1, data);
  nvec = cvMat(3, 1, CV_64FC1, normal);
  dvec = cvMat(3, 1, CV_64FC1, dir);
  for (i = 0; i < 3; i++)
    {
      // 基準軸を x, y, z と変えながら試す
      cvSetZero(&avec);
      cvmSet(&avec, i, 0, 1.0);
      // 基準軸と法線に直交する単位方向ベクトルをもとめる
      sts = getOrthogonalDir(&avec, &nvec, &dvec);
      // 方向ベクトルが計算できたらループ終了
      if (sts == 0)
        {
          break;
        }
    }
  // 方向ベクトルが計算できなかったらエラー終了
  if (sts)
    {
      return -1;
    }
  // 原点を中心とする円上の点をもとめる
  mulV3S(radius, dir, point);
  return 0;
}

// 円の評価点を生成する
static int
makeCirclePoints(Circle* cir, double pdist)
{
  Trace* points;
  P3D* transformed;
  P2D* projected;
  double genP[3];
  double len;
  double theta;
  quaternion_t q;
  int np, i, sts;

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

  // 円の始点座標計算
  sts = getPointOnCircle(cir->normal, cir->radius, genP);
  // 始点計算ができないときは終了
  if (sts)
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

  // 点間の回転角度を求める
  theta = 2.0 * M_PI / (double) (np - 1);
  // 始点を格納
  addV3(genP, cir->center, points[0].xyz);
  setOneAtEnd(points[0].xyz);
  // 回転を表す単位クォータニオンの計算
  quaternion_rotation(q, -theta, cir->normal);
  for (i = 1; i < np; i++)
    {
      // 円中心からの法線を回転軸にして円の点列座標を計算する
      quat_rot(genP, q, genP);
      addV3(genP, cir->center, points[i].xyz);
      // 同次行列乗算のため末尾 1 のベクトルとする
      setOneAtEnd(points[i].xyz);
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

  subV3(ver->position, ver->endpoint1, ep1);
  subV3(ver->endpoint2, ver->position, ep2);
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
  copyV3(ver->position, points[np1 - 1].xyz);
  setOneAtEnd(points[np1 - 1].xyz);

  // 点間隔毎の位置ベクトル増分値を求める
  normalizeV3(ep2, dir);
  dir[0] *= pdist;
  dir[1] *= pdist;
  dir[2] *= pdist;
  // position から一定間隔の点列を作る
  copyV3(ver->position, points[np1].xyz);
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

  dx = Curr[0] - Prev[0];
  dy = Curr[1] - Prev[1];

  if (isZero(dx))
    {
      if (isZero(dy))
        {
          return AZIMUTH_NONE;
        }
      else
        {
          if (dy > 0.0)
            {
              return AZIMUTH_S;
            }
          else
            {
              return AZIMUTH_E;
            }
        }
    }
  theta = tan(dy/dx)/M_PI * 180.0;
  
  if (fabs(theta) <= 22.5 )
    {
      return AZIMUTH_E;
    }
  if (theta > 0.0)
    {
      if (theta < 67.5)
        {
          return AZIMUTH_SE;
        }
      else
        {
          if (theta < 112.5)
            {
              return AZIMUTH_S;
            }
          else
            {
              if (theta < 157.5)
                {
                  return AZIMUTH_SW;
                }
              else
                {
                  return AZIMUTH_W;
                }
            }
        }
    }
  else // theta < 0.0
    {
      if (theta > -67.5)
        {
          return AZIMUTH_NE;
        }
      else
        {
          if (theta > -112.5 )
            {
              return AZIMUTH_N;
            }
          else
            {
              if (theta > -157.5)
                {
                  return AZIMUTH_NW;
                }
              else
                {
                  return AZIMUTH_W;
                }
            }
        }
    }
}

// 回転行列を回転ベクトルに変換
static void
getRotationVector(double rotmat[3][3], double rotvec[3])
{
  CvMat RotMat, RotVec;
  RotMat = cvMat(3, 3, CV_64FC1, rotmat);
  RotVec = cvMat(1, 3, CV_64FC1, rotvec);
  cvRodrigues2(&RotMat, &RotVec);
  return;
}

// 視線ベクトルの計算
static void
calcPointDirection(double sight[3], const CameraParam* cp, const double point[3])
{
  int i;
  double norm = 0.0;

  for (i = 0; i < 3; ++i)
    {
      sight[i] = point[i] - cp->Position[i];
      norm += sight[i] * sight[i];
    }

  norm = sqrt(norm);
  for (i = 0; i < 3; ++i)
    {
      sight[i] /= norm;
    }
  return;
}

// 頂点の可視判定
static void
setVisibleVertex(Features3D* model, double matrix[4][4], Vertex* vertex, int p_camera)
{
  cv::Mat RTmat = cv::Mat(4, 4, CV_64FC1, matrix);
  double vec[4], psrc[4], pdst[4];
  cv::Mat src = cv::Mat(4, 1, CV_64FC1, psrc);
  cv::Mat dst = cv::Mat(4, 1, CV_64FC1, pdst);
  double dot, len;
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

  // 法線による可視判定
  // 視線ベクトル
  psrc[0] = vertex->position[0];
  psrc[1] = vertex->position[1];
  psrc[2] = vertex->position[2];
  psrc[3] = 1.0;
  dst = RTmat * src;
  calcPointDirection(vec, cameraParam, pdst);

  psrc[0] = vertex->orientation[2][0];
  psrc[1] = vertex->orientation[2][1];
  psrc[2] = vertex->orientation[2][2];
  psrc[3] = 0.0;
  dst = RTmat * src;

  dot = getInnerProductV3(pdst, vec);
  len = getNormV3(pdst);

  if (dot / len > 0.0) // VISION_EPSを使う？
    {
      vertex->label = VISIBLE;
    }
  else
    {
      vertex->label = INVISIBLE;
    }

  return;
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
  colrow->x = iPos.col;
  colrow->y = iPos.row;

  return;
}

// 頂点の２線分の評価点の画像投影
static void
projectVertexPoint(Features3D* model, double matrix[4][4], Vertex& vertex, int p_camera)
{
  CvMat RTmat, src, dst;
  double pdata[4];
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

  numOfpoints = vertex.numOfTracePoints;

  RTmat = cvMat(4, 4, CV_64FC1, matrix);

  for (i = 0; i < numOfpoints; i++)
    {
      src = cvMat(4, 1, CV_64FC1, vertex.tracepoints[i].xyz);
      dst = cvMat(4, 1, CV_64FC1, pdata);
      cvMatMul(&RTmat, &src, &dst);
      copyV3(pdata, vertex.transformed[i].xyz);
    }
  for (i = 0; i < numOfpoints; i++)
    {
      projectXYZ2LR(&iPos, vertex.transformed[i].xyz, cameraParam);
      vertex.tracepoints[i].colrow[0] = iPos.col;
      vertex.tracepoints[i].colrow[1] = iPos.row;
    }

  return;
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
      copyV3(circle.tracepoints[i].xyz, hvec);
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
  copyV3(circle.center, hvec);
  setOneAtEnd(hvec);
  src = cvMat(4, 1, CV_64FC1, hvec);
  dst = cvMat(4, 1, CV_64FC1, pdata);
  cvMatMul(&RTmat, &src, &dst);
  copyV3(pdata, center);
  calcPointDirection(vec, cameraParam, pdata);

  // 法線
  copyV3(circle.normal, hvec);
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

      setVisibleVertex(model, matrix, &vertex, p_camera);
      if (vertex.label == INVISIBLE)
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

// 観測可能な点の数の計算
static int
count_visible_points(const Circle& circle)
{
  int c = 0;
  for (int i = 0; i < circle.numOfTracePoints; ++i)
    {
      if (circle.tracepoints[i].label == VISIBLE)
        {
          c++;
        }
    }

  return c;
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
  Circle circle;
  double* curr;
  double* prev;
  int i, j, ii, jj;
  cv::Mat cimg;
  cv::Scalar color;
  cv::Point p1, p2, p3;
  std::vector<cv::Point> CR[2];

  color = cv::Scalar(0, 255, 0);
  cimg = cv::Mat(model->calib->rowsize, model->calib->colsize, CV_8UC3, img);

  for (i = 0; i < model->numOfVertices; i++)
    {
      vertex = model->Vertices[i];
      if (vertex.side == M3DF_BACK)
        {
          continue;
        }
      if (vertex.numOfTracePoints == 0)
        {
          continue;
        }

      setVisibleVertex(model, matrix, &vertex, p_camera);
      if (vertex.label == INVISIBLE)
        {
          continue;
        }
      // 頂点および端点の投影
      projectXYZ2LRwithTrans(model, matrix, p_camera, vertex.position, &p1);
      projectXYZ2LRwithTrans(model, matrix, p_camera, vertex.endpoint1, &p2);
      projectXYZ2LRwithTrans(model, matrix, p_camera, vertex.endpoint2, &p3);
      cv::line(cimg, p1, p2, color, lineThickness, CV_AA);
      cv::line(cimg, p1, p3, color, lineThickness, CV_AA);
    }

  for (i = 0; i < model->numOfCircles; i++)
    {
      circle = model->Circles[i];
      if (circle.side == M3DF_BACK)
        {
          continue;
        }
      if (circle.numOfTracePoints == 0)
        {
          continue;
        }
      projectCirclePoint(model, matrix, circle, p_camera);
      ii = (int) (i / 2) % 2;
      for (j = 0; j < circle.numOfTracePoints; j++)
        {
          jj = j - 1;
          if (j == 0)
            {
              jj = circle.numOfTracePoints - 1;
            }
          curr = circle.tracepoints[j].colrow;
          prev = circle.tracepoints[jj].colrow;
          p1.x = (int) floor(prev[0] + 0.5);
          p1.y = (int) floor(prev[1] + 0.5);
          p2.x = (int) floor(curr[0] + 0.5);
          p2.y = (int) floor(curr[1] + 0.5);
          if (CR[ii].size() > static_cast<unsigned int>(j))
            {
              CR[ii].at(j) = p1;
            }
          else
            {
              CR[ii].push_back(p1);
            }
        }

      // 円一つの場合
      if (model->numOfCircles == 2)
        {
          int nPoints = circle.numOfTracePoints;
          for (j = 0; j < nPoints; j++)
            {
              jj = j - 1;
              if (j == 0)
                {
                  jj = nPoints - 1;
                }
              cv::line(cimg, CR[0][j], CR[0][jj], color, lineThickness, CV_AA);
            }
        }

      if (ii % 2 == 1)
        {
          // 凸包を計算することにより、遮蔽輪郭部分を描画する
          std::vector<cv::Point> points;
          std::vector<int> hull;
          cv::Point pt;
          int vpoints1 = 0, vpoints2 = 0;

          // 観測可能な点の数の計算
          vpoints1 = count_visible_points(circle);
          vpoints2 = count_visible_points(model->Circles[i - 2]);

          int circle_index = 1;
          int nPoints = circle.numOfTracePoints;
          if (vpoints1 < vpoints2)
            {
              circle_index = 0;
              nPoints = model->Circles[i - 2].numOfTracePoints;
            }
          for (j = 0; j < nPoints; j++)
            {
              jj = j - 1;
              if (j == 0)
                {
                  jj = nPoints - 1;
                }
              cv::line(cimg, CR[circle_index][j], CR[circle_index][jj], color, lineThickness, CV_AA);
            }

          for (j = 0; j < circle.numOfTracePoints; j++)
            {
              points.push_back(CR[1][j]);
            }
          circle = model->Circles[i - 2];
          for (j = 0; j < circle.numOfTracePoints; j++)
            {
              points.push_back(CR[0][j]);
            }
          
          cv::convexHull(cv::Mat(points), hull, CV_CLOCKWISE);
          int hullcount = (int) hull.size();
          cv::Point pt0 = points[hull[hullcount - 1]];
          for (j = 0; j < hullcount; j++)
            {
              cv::Point pt = points[hull[j]];
              cv::line(cimg, pt0, pt, color, lineThickness, CV_AA);
              pt0 = pt;
            }
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
      count = makeVertexPoints(&model->Vertices[i], pdist);
      if (count < 0)
        {
          return count; // エラーコード
        }
      totalcount += count;
    }

  for (i = 0; i < model->numOfCircles; i++)
    {
      // 裏側は表側と同じ点列なのでスキップ
      if (model->Circles[i].side == M3DF_BACK)
        {
          continue;
        }
      count = makeCirclePoints(&model->Circles[i], pdist);
      if (count < 0)
        {
          return count; // エラーコード
        }
      totalcount += count;
    }

  return totalcount;
}

// 合同変換行列を位置ベクトルと回転ベクトルを合わせた7次元ベクトルに変換する
void
getPropertyVector(double mat[4][4],            // 合同変換行列
                  double vec[7])               // ７次元ベクトル
{
  double Rotation[3][3], len;
  int i, j;

  for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 3; j++)
        {
          // ３ｘ３回転行列をコピー
          Rotation[i][j] = mat[i][j];
        }
      // 位置ベクトルをコピー
      vec[i] = mat[i][3];
    }
  // 回転行列を回転ベクトルに変換
  getRotationVector(Rotation, &vec[3]);
  // 回転角度（ベクトルの大きさ: OpenCV の仕様による）をもとめる
  len = getNormV3(&vec[3]);
  if (isZero(len))
    {
      vec[6] = 0;
    }
  else
    {
      vec[6] = len;          // 回転角度
    }

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
