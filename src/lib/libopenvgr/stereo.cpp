/*
 stereo.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file stereo.cpp
 * @brief ステレオ処理関連関数
 * @date \$Date::                            $
 */
#include "common.h"
#include "stereo.h"
#include "vectorutil.h"
#include "rtvcm.h"
#include "debugutil.h"

#include <cv.h>

// シーンの頂点特徴情報を認識用の３次元頂点特徴情報に変換する
static void
convertVertexS(VertexCandidate src, Vertex& dst, int num)
{
  double vangle, position[3] = {0.0, 0.0, 0.0};
  CvMat vec1, vec2;
  CvMat normal, bisector, perpendicular;
  int i;

  for (i = 0; i < 3; ++i)
    {
      dst.endpoint1[i] = src.endpoint1[i] - src.position[i];
      dst.endpoint2[i] = src.endpoint2[i] - src.position[i];
    }
  // 頂点を構成する線分の単位方向ベクトルを求める
  getDirectionVector(position, dst.endpoint1, dst.direction1, &vec1);
  getDirectionVector(position, dst.endpoint2, dst.direction2, &vec2);
  // 頂点を構成する線分の成す角を求める
  vangle = cvDotProduct(&vec1, &vec2);
  vangle = (acos(vangle) / M_PI) * 180.0;
  dst.angle = vangle;
  // 以下の３つのベクトルを用いて姿勢を表す行列をつくる
  // 頂点の法線を求める
  normal = cvMat(3, 1, CV_64FC1, dst.tPose[2]);
  cvCrossProduct(&vec1, &vec2, &normal);
  // 頂点を構成する線分が成す角の２等分線（単位方向ベクトルの中線）を求める
  bisector = cvMat(3, 1, CV_64FC1, dst.tPose[1]);
  cvAdd(&vec1, &vec2, &bisector);
  cvScale(&bisector, &bisector, 0.5);
  cvNormalize(&bisector, &bisector);
  // 頂点の法線と中線の両方に直交する軸の方向を求める
  perpendicular = cvMat(3, 1, CV_64FC1, dst.tPose[0]);
  cvCrossProduct(&bisector, &normal, &perpendicular);
  // 平行移動成分
  copyV3(src.position, dst.tPose[3]);
  dst.tPose[3][3] = 1.0;
  // 特徴の通し番号を設定
  dst.n = num;
  dst.side = M3DF_FRONT;

  return;
}

// シーンの円特徴情報を認識用の３次元円特徴情報に変換する
static void
convertCircleS(CircleCandidate src, Circle& dst, int num)
{
  CvMat axis, normal, dir1, dir2;
  double adata[3];
  int i, sts;

  // 元の情報をコピーする
  dst.radius = src.radius;
  copyV3(src.normal, dst.tPose[2]);

  axis = cvMat(3, 1, CV_64FC1, adata);
  normal = cvMat(3, 1, CV_64FC1, dst.tPose[2]);
  dir1 = cvMat(3, 1, CV_64FC1, dst.tPose[0]);

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

  dir2 = cvMat(3, 1, CV_64FC1, dst.tPose[1]);
  // normal と dir1 に直交するベクトルを dir2 に返す
  cvCrossProduct(&normal, &dir1, &dir2);
  cvNormalize(&dir2, &dir2);

  // 円の法線
  for (i = 0; i < 3; ++i)
    {
      dst.normal[i] = (i != 0) ? 0.0 : 1.0; // 法線はx軸
    }

  // 平行移動成分
  copyV3(src.center, dst.tPose[3]);
  dst.tPose[3][3] = 1.0;

  // 特徴の通し番号を設定
  dst.n = num;
  dst.side = M3DF_FRONT;

  return;
}

//ステレオ対応点から３次元座標を計算するの2
//戻り値：再投影誤差の平均
double
calculateLR2XYZ(double position3D[3],          // 出力３次元座標
                Data_2D posL,                  // 左画像上の対応点座標
                Data_2D posR,                  // 右画像上の対応点座標
                CameraParam* camParamL,        // 左画像のカメラパラメータ
                CameraParam* camParamR)        // 右画像のカメラパラメータ
{
  double (*R[2])[3] = {camParamL->Rotation, camParamR->Rotation};
  double *t[2] = {camParamL->Translation, camParamR->Translation};

  Data_2D pos[2], rpos[2];
  double error = 0.0;

  double A[4][3], b[4];
  cv::Mat mA(4, 3, CV_64FC1, A), mb(4, 1, CV_64FC1, b), x(3, 1, CV_64FC1, position3D);

  int i, j;

  /* 正規化座標系に変換 */
  backprojectPoint(&pos[0], posL, camParamL);
  backprojectPoint(&pos[1], posR, camParamR);

  /* 復元計算用の行列を作成 */
  for (i = 0; i < 2; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          A[2*i  ][j] = R[i][0][j] - R[i][2][j] * pos[i].col;
          A[2*i+1][j] = R[i][1][j] - R[i][2][j] * pos[i].row;
        }

      b[2*i  ] = t[i][2] * pos[i].col - t[i][0];
      b[2*i+1] = t[i][2] * pos[i].row - t[i][1];
    }

  /* 方程式を解く */
  cv::solve(mA, mb, x, cv::DECOMP_QR);

  /* 再投影点を計算 */
  projectXYZ2LR(&rpos[0], position3D, camParamL);
  projectXYZ2LR(&rpos[1], position3D, camParamR);

  error = (sqrt(pow(rpos[0].col - posL.col, 2) + pow(rpos[0].row - posL.row, 2))
           + sqrt(pow(rpos[1].col - posR.col, 2) + pow(rpos[1].row - posR.row, 2))) / 2.0;

  //printf("error: %g\n", error);

  return error;
}

// 対応する2組の線から3次元平面を求める
double calculatePlane3D(double plane3D[4],             // ３次元平面
                        const double l11[3],           // 左画像上の対応線1
                        const double l12[3],           // 左画像上の対応線2
                        const double l21[3],           // 右画像上の対応線1
                        const double l22[3],           // 右画像上の対応線2
                        const CameraParam* camParamL,  // 左画像のカメラパラメータ
                        const CameraParam* camParamR)  // 右画像のカメラパラメータ
{
  cv::Mat P[2] = {cv::Mat(3, 4, CV_64FC1), cv::Mat(3, 4, CV_64FC1)};
  const CameraParam* cp[2] = {camParamL, camParamR};

  // 射影行列を計算する
  for (int i = 0; i < 2; ++i)
    {
      for (int j = 0; j < 3; ++j)
        {
          for (int k = 0; k < 3; ++k)
            {
              P[i].at<double>(j, k) = 0.0;
              for (int l = 0; l < 3; ++l)
                {
                  P[i].at<double>(j, k) += cp[i]->intrinsicMatrix[j][l] * cp[i]->Rotation[l][k];
                }
            }

          P[i].at<double>(j, 3) = 0.0;
          for (int k = 0; k < 3; ++k)
            {
              P[i].at<double>(j, 3) += cp[i]->intrinsicMatrix[j][k] * cp[i]->Translation[k];
            }
        }
    }

  cv::Mat M(4, 4, CV_64FC1);
  cv::Mat ml[2][2] = {{cv::Mat(1, 3, CV_64FC1, const_cast<double*>(l11)),
                       cv::Mat(1, 3, CV_64FC1, const_cast<double*>(l12))},
                      {cv::Mat(1, 3, CV_64FC1, const_cast<double*>(l21)),
                       cv::Mat(1, 3, CV_64FC1, const_cast<double*>(l22))}};

  // 対応線から3次元直線を計算
  for (int i = 0; i < 2; ++i)
    {
      cv::Mat L(2, 4, CV_64FC1);

      for (int j = 0; j < 2; ++j)
        {
          L.row(j) = ml[j][i] * P[j];
        }

      cv::SVD svd(L, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

      for (int j = 0; j < 2; ++j)
        {
          for (int k = 0; k < 4; ++k)
            {
              M.at<double>(2*i + j, k) = svd.vt.at<double>(2 + j, k);
            }
        }
    }

  // 2本の3次元直線を含む平面を算出
  cv::SVD svd(M, cv::SVD::MODIFY_A);

#if 0
  printf("singular value: ");
  for (int i = 0; i < 4; ++i)
    {
      printf("% 10.3g ", svd.w.at<double>(i, 0));
    }
  printf("\n");

  for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
        {
          printf("% 10.3g ", svd.vt.at<double>(i, j));
        }
      printf("\n");
    }
  printf("\n");
#endif

  // 計算結果を代入
  double norm = 0.0;
  for (int i = 0; i < 3; ++i)
    {
      norm += svd.vt.at<double>(3, i) * svd.vt.at<double>(3, i);
    }
  norm = sqrt(norm);

  if (norm >= VISION_EPS)
    {
      if (svd.vt.at<double>(3, 3) > 0.0)
        {
          norm = -norm;
        }

      for (int i = 0; i < 4; ++i)
        {
          plane3D[i] = svd.vt.at<double>(3, i) / norm;
        }
    }
  else
    {
      plane3D[0] = plane3D[1] = plane3D[2] = 0.0;
      plane3D[3] = 1.0;
    }

  //printf("plane: % 10.3g % 10.3g % 10.3g % 10.3g\n", plane3D[0], plane3D[1], plane3D[2], plane3D[3]);

  return svd.w.at<double>(3, 0);
}

// ３次元点の２次元画像上への投影点座標を求める
void
projectXYZ2LR(Data_2D* pos2D,                  // ２次元画像上の投影点座標
              double position[3],              // ３次元点座標
              CameraParam* cameraParam)        // 投影する画像のカメラパラメータ
{
  double xyz[3];
  Data_2D icPos2D;

  mulM33V3(cameraParam->Rotation, position, xyz);
  addV3(xyz, cameraParam->Translation, xyz);
  icPos2D.col = xyz[0] / xyz[2];
  icPos2D.row = xyz[1] / xyz[2];
  projectPoint(pos2D, icPos2D, cameraParam);

  return;
}

// ステレオ処理結果を３次元特徴構造体へセットする
bool
set_circle_to_OldFeature3D(const std::vector<CircleCandidate>& candidates,
                           Features3D* feature)
{
  size_t numOfcircles = 0;
  size_t i, j;

  numOfcircles = candidates.size();

  if (numOfcircles == 0)
    {
      return true;
    }

  // 表裏の特徴をつくるため元の２倍の領域を確保する
  feature->numOfCircles = numOfcircles * 2;

  feature->Circles = (Circle*) calloc(feature->numOfCircles, sizeof(Circle));
  if (feature->Circles == NULL)
    {
      free(feature->Vertices);
      feature->Vertices = NULL;
      return false;
    }


  j = 0;
  for (i = 0; i < candidates.size(); i++)
    {
      // 有効な円特徴をコピーする
      // 表の特徴を作成
      convertCircleS(candidates[i], feature->Circles[j], (int) (j / 2));
      // 裏の特徴を作成
      reverseCircle(feature->Circles[j], feature->Circles[j + 1]);
      j += 2;
    }

  return true;
}

// 頂点のステレオ処理結果を３次元特徴構造体へセットする
bool
set_vertex_to_OldFeature3D(const std::vector<VertexCandidate>& candidates,
                           Features3D* feature)
{
  size_t numOfvertices = 0;
  size_t i, j;

  numOfvertices = candidates.size();

  if (numOfvertices == 0)
    {
      return true;
    }

  // 表裏の特徴をつくるため元の２倍の領域を確保する
  feature->numOfVertices = numOfvertices * 2;

  feature->Vertices = (Vertex*) calloc(feature->numOfVertices, sizeof(Vertex));
  if (feature->Vertices == NULL)
    {
      return false;
    }


  j = 0;
  for (i = 0; i < candidates.size(); i++)
    {
      // 有効な円特徴をコピーする
      // 表の特徴を作成
      convertVertexS(candidates[i], feature->Vertices[j], (int) (j / 2));
      // 裏の特徴を作成
      reverseVertex(feature->Vertices[j], feature->Vertices[j + 1]);
      j += 2;
    }

  return true;
}
