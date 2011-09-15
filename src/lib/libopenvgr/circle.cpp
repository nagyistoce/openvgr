/*
 circle.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file circle.cpp
 * @brief 3次元円特徴生成関連関数
 * @date \$Date::                            $
 */
#include "stereo.h"
#include "vectorutil.h"
#include "debugutil.h"
#include "modelpoints.h"

#include "correspondence.hpp"
#include "extractFeature.hpp"

// 二次元楕円の形状から三次元空間中の真円の半径と法線の推定値を計算する
// Reference:
// Kenichi Katanani and Wu Liu, "3D Interpretation of Conics and Orthogonality", 
// CVGIP: Image Understanding, Vol.58, No.3, pp.286-301, 1993
static double
CircleNormal(CameraParam* cameraParam, Feature2D_old* feature, double center[3],
             double normal[2][3])
{
  cv::Mat Q(3, 3, CV_64FC1), eigenvalues(3, 1, CV_64FC1), eigenvectors(3, 3, CV_64FC1);
  cv::Mat n1(1, 3, CV_64FC1), n2(1, 3, CV_64FC1);
  double e[3];
  double det;
  double lambda1, lambda2;
  double newcoef[6];
  const double t_x = -cameraParam->intrinsicMatrix[0][2], t_y = -cameraParam->intrinsicMatrix[1][2];
  const double f = cameraParam->intrinsicMatrix[0][0];
  double val, d, cc[3], radius;
  int i, j;

  // 原点を光学中心に移動
  newcoef[0] = feature->coef[0];
  newcoef[1] = feature->coef[1];
  newcoef[2] = feature->coef[2];
  newcoef[3] = -2.0 * feature->coef[0] * t_x - feature->coef[1] * t_y + feature->coef[3];
  newcoef[4] = -feature->coef[1] * t_x - 2.0 * feature->coef[2] * t_y + feature->coef[4];
  newcoef[5] = feature->coef[0] * t_x * t_x + feature->coef[1] * t_x * t_y +
    feature->coef[2] * t_y * t_y - feature->coef[3] * t_x - feature->coef[4] * t_y +
    feature->coef[5];

  Q.at<double>(0, 0) = newcoef[0];
  Q.at<double>(0, 1) = newcoef[1] / 2;
  Q.at<double>(1, 0) = Q.at<double>(0, 1);
  Q.at<double>(1, 1) = newcoef[2];
  Q.at<double>(2, 0) = newcoef[3] / 2 / f;
  Q.at<double>(0, 2) = Q.at<double>(2, 0);
  Q.at<double>(1, 2) = newcoef[4] / 2 / f;
  Q.at<double>(2, 1) = Q.at<double>(1, 2);
  Q.at<double>(2, 2) = newcoef[5] / f / f;

  // 行列式が -1 になるように正規化
  det = cv::determinant(Q);
  val = -cbrt(det);
  Q /= val;

  cv::eigen(Q, eigenvalues, eigenvectors);
  e[0] = eigenvalues.at<double>(0, 0);
  e[1] = eigenvalues.at<double>(1, 0);
  e[2] = eigenvalues.at<double>(2, 0);

  // 法線ベクトルの算出
  lambda1 = sqrt((e[0] - e[1]) / (e[0] - e[2]));
  lambda2 = sqrt((e[1] - e[2]) / (e[0] - e[2]));
  n1 = lambda1 * eigenvectors.row(0) + lambda2 * eigenvectors.row(2);
  n2 = lambda1 * eigenvectors.row(0) - lambda2 * eigenvectors.row(2);

  // 単位ベクトルへの正規化
  n1 = n1 / sqrt(n1.dot(n1));
  n2 = n2 / sqrt(n2.dot(n2));

  // 法線をワールド系に変換
  for (i = 0; i < 3; i++)
    {
      normal[0][i] = normal[1][i] = 0;
      for (j = 0; j < 3; j++)
        {
          normal[0][i] += cameraParam->Rotation[j][i] * n1.at<double>(0, j);
          normal[1][i] += cameraParam->Rotation[j][i] * n2.at<double>(0, j);
        }
    }

  // 支持平面までの距離
  d = 0.0;
  for (i = 0; i < 3; i++)
    {
      cc[i] = center[i] - cameraParam->Position[i];
      d += normal[0][i] * cc[i];
    }
  d = fabs(d);

  // 半径
  radius = d / sqrt(e[1] * e[1] * e[1]);

  return radius;
}

//! 画像上の楕円の長軸・短軸に対応する3次元単位ベクトルの算出
void
calc_3d_axes_of_circle(double major_axis[3],   //!< 長軸
                       double minor_axis[3],   //!< 短軸
                       const double normal[3], //!< 3次元円の法線
                       const CameraParam *cp)  //!< カメラパラメータ
{
  double norm = 0.0;

  // 長軸の方向をカメラの奥行き方向と法線方向の外積で決める
  if (cp != NULL)
    {
      getCrossProductV3(const_cast<double*>(cp->Rotation[2]), const_cast<double*>(normal), major_axis);
      norm = getNormV3(major_axis);
      if (norm >= VISION_EPS)
        {
          mulV3S(1.0/norm, major_axis, major_axis);
          getCrossProductV3(const_cast<double*>(normal), major_axis, minor_axis);
        }
      else
        {
          copyV3(const_cast<double*>(cp->Rotation[0]), major_axis);
          copyV3(const_cast<double*>(cp->Rotation[1]), minor_axis);
        }
    }
  else // cp == NULL の時はカメラの姿勢＝単位行列とする
    {
      major_axis[0] = -normal[1];
      major_axis[1] =  normal[0];
      major_axis[2] = 0.0;

      norm = getNormV3(major_axis);
      if (norm >= VISION_EPS)
        {
          mulV3S(1.0/norm, major_axis, major_axis);
          getCrossProductV3(const_cast<double*>(normal), major_axis, minor_axis);
        }
      else
        {
          major_axis[0] = 1.0;
          major_axis[1] = 0.0;
          major_axis[2] = 0.0;

          minor_axis[0] = 0.0;
          minor_axis[1] = normal[2] > 0.0 ? 1.0 : -1.0;
          minor_axis[2] = 0.0;
        }
    }
}

// 真円を２次元エッジ画像に投影して評価する
static int
evalCircleOnEdge(unsigned char* edge, CameraParam* cameraParam,
		 double radius, double normal[3], double center[3],
		 Parameters parameters)
{
  double in_xyz3d[3];           // 円上の点（開始は３次元円周上の一点）
  int iCountOnEdge = 0;
  int rowsize = parameters.rowsize;
  int colsize = parameters.colsize;
  Data_2D iPos;
  int ndiv = 360; // 分割数
  int d;
  int col, row;

  double axis[2][3];

  calc_3d_axes_of_circle(axis[0], axis[1], normal, NULL);
  for (d = 0; d < ndiv; ++d)
    {
      int i;
      double theta = (double)d / (double)(ndiv - 1) * M_PI * 2.0;
      for (i = 0; i < 3; ++i)
        {
          in_xyz3d[i] = radius * (axis[0][i] * cos(theta) + axis[1][i] * sin(theta)) + center[i];
        }

      projectXYZ2LR(&iPos, in_xyz3d, cameraParam);

      col = (int) floor(iPos.col + 0.5);
      row = (int) floor(iPos.row + 0.5);
      if (col >= 0 && col < colsize && row >= 0 && row < rowsize)
        {
          if (edge[row * colsize + col] > 1)
            {                       // 楕円に使用されたエッジは２
              iCountOnEdge++;
            }
        }
    }

  return iCountOnEdge;
}

// 二次元楕円のステレオ対応から三次元空間中の真円を推定・復元する
void
reconstruct_ellipse2D_to_circle3D(std::vector<const ovgr::Features2D*>& feature,
                                  const ovgr::CorrespondingSet& cs,
                                  const CameraParam* camParam[3],
                                  const unsigned char* edge[3],
                                  Features3D* scene,
                                  const Parameters& parameters)  // 全パラメータ
{
  double ethr, depn, depf, rthr, nthr;
  double nL[2][3], rL;
  double nR[2][3], rR;
  double wcenter[3] = { 0 };
  double wnormal[3] = { 0 };
  double radius;
  double ip[4], max;
  double diff;
  int maxL, maxR;
  int iCountOnEdge, iMaxCountOnEdge, iMaxN;

  int count = 0;

  ethr = parameters.stereo.ethr;
  depn = parameters.stereo.depn >= 0.0 ? parameters.stereo.depn : 0.0;
  depf = parameters.stereo.depf >= 0.0 ? parameters.stereo.depf : -1.0;
  rthr = parameters.stereo.rdif;
  nthr = parameters.stereo.ndif;

  std::vector<Features2D_old*> old_Features2D(feature.size());
  std::vector<CircleCandidate> ccandidates; // 3次元円復元結果格納用
  std::vector<size_t> nv(feature.size()); // 頂点数

  // 旧特徴への変換、頂点数の計算
  for (size_t i = 0; i < feature.size(); ++i)
    {
      old_Features2D[i] = ovgr::create_old_features_from_new_one(*feature[i]);      
      nv[i] = feature[i]->vertex.size();
    }

  for (ovgr::feature_list_t::const_iterator it = cs.ellipse.begin(); it != cs.ellipse.end(); ++it)
    {
      std::vector<Feature2D_old*> old_f(it->size());
      std::vector<const ovgr::EllipseFeature*> new_f(it->size());
      std::vector<int> c_index(it->size()); // カメラのインデックス
      size_t n = 0;
      
      for (size_t c = 0; c < it->size(); ++c)
        {
          // データが存在していたら
          if ((*it)[c] != -1)
            {
              c_index[n] = c;
              new_f[n] = &(feature[c]->ellipse[(*it)[c]]);
              // 旧特徴は、頂点、円と並んでいる
              old_f[n] = &(old_Features2D[c]->feature[nv[c] + (*it)[c]]);
              n++;
            }
        }

      CircleCandidate ccandidate = {0}; // 円候補
      // 中心の復元
      Data_2D posL, posR;
#if 0
      posL.col = old_f[0]->center[0];
      posL.row = old_f[0]->center[1];
      posR.col = old_f[1]->center[0];
      posR.row = old_f[1]->center[1];
#else
      posL.col = new_f[0]->center[0];
      posL.row = new_f[0]->center[1];
      posR.col = new_f[1]->center[0];
      posR.row = new_f[1]->center[1];
#endif
      double error = calculateLR2XYZ(wcenter, posL, posR, 
                                     const_cast<CameraParam*>(camParam[c_index[0]]), 
                                     const_cast<CameraParam*>(camParam[c_index[1]]));
      // ここでの判定は将来的には必要ないはず
      // テスト段階では前候補がここにくるので判定が必要
      if(error > ethr)
        {
          continue;
        }

      // 一つ目のカメラと復元した頂点の距離を求める
      double dep = camParam[c_index[0]]->Translation[2];
      for (int i = 0; i < 3; ++i)
        {
          dep += camParam[c_index[0]]->Rotation[2][i] * wcenter[i];
        }
      //fprintf(stderr, "dep: %f [%f, %f]\n", dep, depn, depf);

      // dep が[depn, depf]の範囲外であれば不採用
      if (dep < depn || (depf > 0.0 && depf < dep))
        {
          continue;
        }

      // 円の半径と法線を求める．法線は２つずつ得られる
      rL = CircleNormal(const_cast<CameraParam*>(camParam[c_index[0]]), old_f[0], wcenter, nL);
      rR = CircleNormal(const_cast<CameraParam*>(camParam[c_index[1]]), old_f[1], wcenter, nR);

      // 半径差が大きいときは誤対応
      if (fabs(rL - rR) > rthr)
        {
          continue;
        }

      // 左右半径の平均を仮の半径とする
      radius = (rL + rR) / 2.0;

      // 法線ベクトルの向きを比較するために左右２個ずつのベクトルの
      // 組み合わせで内積をとり，最大になるものを見つける
      max = -DBL_MAX;
      maxL = maxR = 0;
      for (int j = 0, n = 0; j < 2; j++)
        {
          for (int k = 0; k < 2; k++, n++)
            {
              ip[n] = getInnerProductV3(nL[j], nR[k]);
              if (ip[n] > max)
                {
                  max = ip[n];
                  maxL = j;
                  maxR = k;
                }
            }
        }

      // 法線の向きが異なるときは誤対応
      if (max < 0)
        {
          continue;
        }

      // １に近いものは同じ方向とする
      diff = acos(max) * 180 / M_PI;
      if (diff > nthr)
        {
          continue;
        }

      // ここまでくれば正しい対応とみなす
      // 左右法線の平均をとる
      for (n = 0; n < 3; n++)
        {
          wnormal[n] = (nL[maxL][n] + nR[maxR][n]) / 2.0;
        }

      // 単位ベクトル化
      normalizeV3(wnormal, wnormal);
      
      // 3つの法線（Ｌ，Ｒ，平均）からもっとも良さそうな法線を選択する
      // 3次元円を2次元エッジ画像に投影して、円上のエッジ点をカウントし、もっとも多いのを選択
      iMaxN = 0;
      normalizeV3(nL[maxL], nL[maxL]);
      normalizeV3(nR[maxR], nR[maxR]);
      iMaxCountOnEdge =
        evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[0]]), 
                         const_cast<CameraParam*>(camParam[c_index[0]]), 
                         radius, wnormal, wcenter, parameters)
        + evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[1]]), 
                           const_cast<CameraParam*>(camParam[c_index[1]]), 
                           radius, wnormal, wcenter, parameters);
      iCountOnEdge =
        evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[0]]), 
                         const_cast<CameraParam*>(camParam[c_index[0]]), 
                         rL, nL[maxL], wcenter, parameters)
        + evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[1]]), 
                           const_cast<CameraParam*>(camParam[c_index[1]]), 
                           rL, nL[maxL], wcenter, parameters);
      if (iCountOnEdge > iMaxCountOnEdge)
        {
          iMaxN = 1;
          iMaxCountOnEdge = iCountOnEdge;
        }
      iCountOnEdge =
        evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[0]]), 
                         const_cast<CameraParam*>(camParam[c_index[0]]), 
                         rR, nR[maxR], wcenter, parameters)
        + evalCircleOnEdge(const_cast<unsigned char*>(edge[c_index[1]]), 
                           const_cast<CameraParam*>(camParam[c_index[1]]), 
                           rR, nR[maxR], wcenter, parameters);
      if (iCountOnEdge > iMaxCountOnEdge)
        {
          iMaxN = 2;
          iMaxCountOnEdge = iCountOnEdge;
        }
      // 半径・法線・中心を保存する
      // ワールド系の出力の場合はこのまま．カメラ系が必要なら変換すること
      copyV3(wcenter, ccandidate.center);
      switch (iMaxN)
        {
        case 1:
          ccandidate.radius = rL;
          copyV3(nL[maxL], ccandidate.normal);
          break;

        case 2:
          ccandidate.radius = rR;
          copyV3(nR[maxR], ccandidate.normal);
          break;

        case 0:
          /* fall through */
        default:
          ccandidate.radius = radius;
          copyV3(wnormal, ccandidate.normal);
          break;
        }
      count++;

      // 復元結果として追加
      ccandidates.push_back(ccandidate);
    }

  // 旧3次元特徴に代入
  set_circle_to_OldFeature3D(ccandidates, scene);

  if ( parameters.dbgimag )
    {
      for (size_t i = 0; i < feature.size(); ++i)
        {
          // 円の３次元復元結果画像の表示・保存
          drawCircleCandidate(edge[i], ccandidates, i, parameters, camParam[i]);
        }
    }

  // メモリの開放
  for (size_t i = 0; i < feature.size(); ++i)
    {
      destructFeatures(old_Features2D[i]);
    }

  return;
}
