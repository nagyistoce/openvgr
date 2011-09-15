/*
 vertex.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file vertex.cpp
 * @brief 3次元頂点特徴生成関連関数
 * @date \$Date::                            $
 */
#include <iostream>
#include "stereo.h"
#include "vectorutil.h"
#include "debugutil.h"
#include "vertex.h"
#include "rtvcm.h"

#include "local.h"
#include "geometry.hpp"

// 画像内かどうかの判定
inline int
isValidPixelPosition(double col, double row, const Parameters& parameters)
{
  if (col < 0 || col > parameters.colsize || row < 0 || row > parameters.rowsize)
    {
      return 0;
    }
  else
    {
      return 1;
    }
}

static int
reconstruct_vertex3D(::Vertex vertex[2],
                     const ovgr::VertexFeature& vf1, const ovgr::VertexFeature& vf2,
                     const cv::Mat& F, const bool inv,
                     const CameraParam& cp1, const CameraParam& cp2)
{
  int num = 0;

  const ovgr::Point2D* pos[2][2] = {{&vf1.start, &vf1.end}, {&vf2.start, &vf2.end}};
  const ovgr::Point2D* mid[2] = {&vf1.mid, &vf2.mid};

  Data_2D pos_temp[2] = {{vf1.mid.x, vf1.mid.y},
                         {vf2.mid.x, vf2.mid.y}};
  double vpos[3];
  double error = 0.0;

  // 頂点の3次元位置を復元
  error = calculateLR2XYZ(vpos, pos_temp[0], pos_temp[1],
                  const_cast<CameraParam*>(&cp1), const_cast<CameraParam*>(&cp2));
  //fprintf(stderr, "error: % 10.3g\n", error);

  for (int pair = 0; pair < 2; ++pair)
    {
      Data_2D corres_pos[2][2];
      bool is_valid = true;

      // 対応点を求める
      for (int i = 0; i < 2; ++i)
        {
          const int pi = (pair + i) % 2;

          double epi_line[3];
          double cross_pos[2][3];
          
          // 第1カメラ画像上の点に対応する第2カメラ画像上の点
          calc_epipolar_line(epi_line, F, *pos[0][i], inv);
          s_cross3(cross_pos[1], epi_line, vf2.line_coef[pi]);
          s_normalize(cross_pos[1]);

          // 第2カメラ画像上の点に対応する第1カメラ画像上の点
          calc_epipolar_line(epi_line, F, *pos[1][pi], !inv);
          s_cross3(cross_pos[0], epi_line, vf1.line_coef[i]);
          s_normalize(cross_pos[0]);

          // 観測点と上で計算した点のうち、頂点に近い方(＝短い線分)を選ぶ
          bool use_projected[2] = {false, false};
          for (int j = 0; j < 2; ++j)
            {
              const int p = (j == 0) ? i : pi;

              const double dot = (pos[j][p]->x - mid[j]->x) * (cross_pos[j][0] / cross_pos[j][2] - mid[j]->x) + (pos[j][p]->y - mid[j]->y) * (cross_pos[j][1] / cross_pos[j][2] - mid[j]->y);
              const double norm2 = pow(pos[j][p]->x - mid[j]->x, 2) + pow(pos[j][p]->y - mid[j]->y, 2);
              const double ratio = dot / norm2;

              if (ratio < VISION_EPS || fabs(cross_pos[j][2]) < VISION_EPS)
                {
                  is_valid = false;
                  break;
                }

              if (ratio <= 1.0)
                {
                  corres_pos[j][i].col = cross_pos[j][0] / cross_pos[j][2];
                  corres_pos[j][i].row = cross_pos[j][1] / cross_pos[j][2];
                  use_projected[j] = true;
                }
              else
                {
                  corres_pos[j][i].col = pos[j][p]->x;
                  corres_pos[j][i].row = pos[j][p]->y;
                  use_projected[j] = false;
                }
            }
          if (use_projected[0] == use_projected[1])
            {
              is_valid = false;
            }

          if (!is_valid)
            {
              break;
            }
        }
      if (!is_valid)
        {
          continue;
        }

#if 0
      printf("% f % f % f % f\n", corres_pos[0][0].col, corres_pos[0][0].row,
             corres_pos[1][0].col, corres_pos[1][0].row);
      printf("% f % f % f % f\n", vf1.mid.x, vf1.mid.y, vf2.mid.x, vf2.mid.y);
      printf("% f % f % f % f\n", corres_pos[0][1].col, corres_pos[0][1].row,
             corres_pos[1][1].col, corres_pos[1][1].row);
      printf("\n");
#endif

      // 3次元端点位置
      error = calculateLR2XYZ(vertex[num].endpoint1, corres_pos[0][0], corres_pos[1][0],
                      const_cast<CameraParam*>(&cp1), const_cast<CameraParam*>(&cp2));
      //fprintf(stderr, "error: % 10.3g ", error);

      error = calculateLR2XYZ(vertex[num].endpoint2, corres_pos[0][1], corres_pos[1][1],
                      const_cast<CameraParam*>(&cp1), const_cast<CameraParam*>(&cp2));
      //fprintf(stderr, "% 10.3g\n", error);

      for (int i = 0; i < 3; ++i)
        {
          vertex[num].endpoint1[i] -= vpos[i]; // 頂点が原点
          vertex[num].endpoint2[i] -= vpos[i];
        }
#if 0
      printf("% f % f % f\n", vertex[num].endpoint1[0], vertex[num].endpoint1[1], vertex[num].endpoint1[2]);
      printf("% f % f % f\n", vertex[num].endpoint2[0], vertex[num].endpoint2[1], vertex[num].endpoint2[2]);
      printf("\n");
#endif

      double vangle;
      CvMat vec1, vec2;
      CvMat normal, bisector, perpendicular;

      // 頂点を構成する線分の単位方向ベクトルを求める
      double position[3] = {0.0, 0.0, 0.0};
      getDirectionVector(position, vertex[num].endpoint1, vertex[num].direction1, &vec1);
      getDirectionVector(position, vertex[num].endpoint2, vertex[num].direction2, &vec2);

      // 頂点を構成する線分の成す角を求める
      vangle = cvDotProduct(&vec1, &vec2);
      vangle = (acos(vangle) / M_PI) * 180.0;
      vertex[num].angle = vangle;

      // 以下の３つのベクトルを用いて姿勢を表す行列をつくる
      // 頂点の法線を求める
      normal = cvMat(3, 1, CV_64FC1, vertex[num].tPose[2]);
      cvCrossProduct(&vec1, &vec2, &normal);
      // 頂点を構成する線分が成す角の２等分線（単位方向ベクトルの中線）を求める
      bisector = cvMat(3, 1, CV_64FC1, vertex[num].tPose[1]);
      cvAdd(&vec1, &vec2, &bisector);
      cvNormalize(&bisector, &bisector);
      // 頂点の法線と中線の両方に直交する軸の方向を求める
      perpendicular = cvMat(3, 1, CV_64FC1, vertex[num].tPose[0]);
      cvCrossProduct(&bisector, &normal, &perpendicular);

      // 3次元頂点位置
      for (int i = 0; i < 3; ++i)
        {
          vertex[num].tPose[3][i] = vpos[i];
        }
      vertex[num].tPose[3][3] = 1.0;

      vertex[num].n = -1;
      vertex[num].side = M3DF_FRONT;

      vertex[num].label = M3DF_LABEL_NONE;
      vertex[num].numOfTracePoints = 0;
      vertex[num].tracepoints = NULL;
      vertex[num].transformed = NULL;
      vertex[num].projected = NULL;

      ++num;
    }

  //fprintf(stderr, "# num: %d\n", num);
  return num;
}

void 
reconstruct_hyperbola_to_vertex3D(const std::vector<const ovgr::Features2D*> feature,
                                  const ovgr::CorrespondingSet& cs,
                                  const CameraParam* camParam[3],
                                  const unsigned char* edge[3],
                                  Features3D* scene,
                                  const Parameters& parameters)
{
  // 使用するパラメータの設定
  const double ethr = parameters.stereo.ethr;
  const double depn = parameters.stereo.depn >= 0.0 ? parameters.stereo.depn : 0.0;
  const double depf = parameters.stereo.depf >= 0.0 ? parameters.stereo.depf : -1.0;
  const double amin = parameters.stereo.amin;
  const double amax = parameters.stereo.amax;
  //const double lmin = parameters.stereo.lmin;
  //const double lmax = parameters.stereo.lmax;

  // カメラの各組み合わせに対するF行列を計算
  std::vector<cv::Mat> F;
  if (feature.size() < 3)
    {
      F.resize(1);
      F[0] = ovgr::calc_fundamental_matrix(*camParam[0], *camParam[1]);
    }
  else
    {
      F.resize(3);

      for (int i = 0; i < 2; ++i)
        {
          for (int j = i+1; j < 3; ++j)
            {
              F[i + (j-1)*j/2] = ovgr::calc_fundamental_matrix(*camParam[i], *camParam[j]);
            }
        }
    }

  // 対応候補（頂点の組）について繰り返し
  std::vector< ::Vertex> scene_temp; // 頂点復元結果格納用
  for (ovgr::feature_list_t::const_iterator it = cs.vertex.begin(); it != cs.vertex.end(); ++it)
    {
      std::vector<Feature2D_old*> old_f(it->size());
      std::vector<const ovgr::VertexFeature*> new_f(it->size());
      std::vector<int> c_index(it->size(), -1); // カメラのインデックス
      size_t n = 0;

      for (size_t c = 0; c < it->size(); ++c)
        {
          // -1 はカメラ c における対応候補が見つからなかったことを示す
          if ((*it)[c] != -1)
            {
              c_index[n] = c;
              new_f[n] = &(feature[c]->vertex[(*it)[c]]);
              n++;
            }
        }

      // 頂点情報
      Data_2D pos[2] = {{new_f[0]->mid.x, new_f[0]->mid.y},
                        {new_f[1]->mid.x, new_f[1]->mid.y}};
      double pos3D[3] = {0.0, 0.0, 0.0};
      double error = calculateLR2XYZ(pos3D, pos[0], pos[1],
                                     const_cast<CameraParam*>(camParam[c_index[0]]), 
                                     const_cast<CameraParam*>(camParam[c_index[1]]));
      if (error > ethr)
        {
          continue;
        }

      // 一つ目のカメラと復元した頂点の距離を求める
      double dep = camParam[c_index[0]]->Translation[2];
      for (int i = 0; i < 3; ++i)
        {
          dep += camParam[c_index[0]]->Rotation[2][i] * pos3D[i];
        }
      //fprintf(stderr, "dep: %f [%f, %f]\n", dep, depn, depf);

      // dep が[depn, depf]の範囲外であれば不採用
      if (dep < depn || (depf > 0.0 && depf < dep))
        {
          continue;
        }

      // F行列を求める
#if 0
      bool inv = false;
      cv::Mat Fmat = ovgr::calc_fundamental_matrix(*camParam[c_index[0]], *camParam[c_index[1]]);

#else
      bool inv = c_index[0] > c_index[1];
      cv::Mat Fmat;
      if (!inv)
        {
          Fmat = F[c_index[0] + (c_index[1]-1)*c_index[1]/2];
        }
      else
        {
          Fmat = F[c_index[1] + (c_index[0]-1)*c_index[0]/2];
        }
#endif

      // 対応する2次元頂点の組から3次元の頂点特徴を生成
      ::Vertex vertex[2];
      int num_vertex = 0;
      memset(vertex, 0, sizeof(::Vertex) * 2);
      num_vertex = reconstruct_vertex3D(vertex, *new_f[0], *new_f[1], Fmat, inv,
                                        *camParam[c_index[0]], *camParam[c_index[1]]);
      for (int i = 0; i < num_vertex; ++i)
        {
          if (vertex[i].angle < amin || amax < vertex[i].angle)
            {
              continue;
            }

          scene_temp.push_back(vertex[i]);
        }
    } /* for (it) */

  // 旧3次元特徴へ代入
  // 表裏の特徴をつくるため元の2倍の領域を確保する
  scene->numOfVertices = scene_temp.size() * 2;

  scene->Vertices = (Vertex*) calloc(scene->numOfVertices, sizeof(Vertex));
  if (scene->Vertices == NULL)
    {
      return;
    }

  for (size_t i = 0; i < scene_temp.size(); ++i)
    {
      scene_temp[i].n = i; // 通し番号

      // 表の特徴をコピー
      memcpy(&scene->Vertices[2*i], &scene_temp[i], sizeof(Vertex));

      // 裏の特徴を作成
      reverseVertex(scene->Vertices[2*i], scene->Vertices[2*i + 1]);
    }

#if 0
  for (int i = 0; i < scene->numOfVertices; ++i)
    {
      Data_2D pos;

      projectXYZ2LR(&pos, scene->Vertices[i].endpoint1, const_cast<CameraParam*>(camParam[0]));
      printf("% f % f\n", pos.col, pos.row);

      projectXYZ2LR(&pos, scene->Vertices[i].endpoint2, const_cast<CameraParam*>(camParam[0]));
      printf("% f % f\n", pos.col, pos.row);

      printf("\n");
    }
#endif
}
