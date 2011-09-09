/*
 pairedcircle.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file pairedcircle.cpp
 * @brief 2円照合関連関数
 * @date \$Date::                            $
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include "match3Dfeature.h"
#include "score2d.h"
#include "vectorutil.h"
#include "modelpoints.h"

// 組み合わせテーブル
typedef struct
{
  int mc1;
  int mc2;
  double cdist;
} PairTable;

// ２円の中心間距離の比較
static int
compareCenterDistance(const void* cc1, const void* cc2)
{
  const PairTable* c1 = (const PairTable*) cc1;
  const PairTable* c2 = (const PairTable*) cc2;
  if (fabs(c2->cdist - c1->cdist) < VISION_EPS)
    {
      return 0;
    }
  else if (c2->cdist > c1->cdist)
    {
      return 1;
    }
  else
    {
      return -1;
    }
}

// 直交ベクトルを求める
static int
getOrthogonalAxis(CvMat* axis, CvMat* snormal, CvMat* dnormal,
                  CvMat* sdir, CvMat* ddir)
{
  double snorm, dnorm;
  // axis と snormal に直交するベクトルを sdir に返す
  cvCrossProduct(axis, snormal, sdir);
  snorm = cvNorm(sdir);
  // axis と dnormal に直交するベクトルを ddir に返す
  cvCrossProduct(axis, dnormal, ddir);
  dnorm = cvNorm(ddir);
  // 外積ベクトルの大きさが零なら無効
  if (isZero(snorm) || isZero(dnorm))
    {
      return -1;
    }
  cvNormalize(sdir, sdir);
  cvNormalize(ddir, ddir);
  return 0;
}

// 2 円の姿勢データを作る
static int
makeOrientationDataForCirclePair(Circle* c1, Circle* c2,
                                 double orientation[12][3])
{
  CvMat axis;
  CvMat dir11, dir12;
  CvMat dir21, dir22;
  CvMat normal1;
  CvMat normal2;
  double dir[3] = { 0.0 };
  double mat1[4][4] = { {0.0} };
  double mat2[4][4] = { {0.0} };
  double dot;
  double r1, r2, cv1, cv2, nv1, nv2;
  double pv1, pv2, qv1, qv2;
  int i, sts;

  for (i = 0; i < 3; i++)
    {
      mat1[0][i] = c1->normal[i];
      mat2[0][i] = c2->normal[i];
      dir[i] = c1->center[i] - c2->center[i];
    }

  axis = cvMat(3, 1, CV_64FC1, dir);
  normal1 = cvMat(3, 1, CV_64FC1, mat1[0]);
  dir11 = cvMat(3, 1, CV_64FC1, mat1[1]);
  dir12 = cvMat(3, 1, CV_64FC1, mat1[2]);
  normal2 = cvMat(3, 1, CV_64FC1, mat2[0]);
  dir21 = cvMat(3, 1, CV_64FC1, mat2[1]);
  dir22 = cvMat(3, 1, CV_64FC1, mat2[2]);

  dot = cvDotProduct(&normal1, &normal2);
  // ２円の法線が同じ側にあるようにする
  if (dot < 0.0)
    {
      cvScale(&normal2, &normal2, -1.0);
    }

  sts = getOrthogonalAxis(&axis, &normal1, &normal2, &dir11, &dir21);

  if (sts)
    {
      for (i = 0; i < 3; i++)
        {
          cvSetZero(&axis);
          // x, y, z 軸の単位方向ベクトルを順に試す
          cvmSet(&axis, i, 0, 1.0);
          // axis と normal1, normal2 に直交するベクトルを dir11, dir21 にそれぞれ返す
          sts = getOrthogonalAxis(&axis, &normal1, &normal2, &dir11, &dir21);
          // うまくいったらループから出る
          if (sts == 0)
            {
              break;
            }
        }
    }

  if (sts)
    {
      // 法線の直交ベクトルが計算できなかったのでエラー
      return -1;
    }

  // normal1 と dir11 に直交するベクトルを dir12 に返す
  cvCrossProduct(&normal1, &dir11, &dir12);
  cvNormalize(&dir12, &dir12);
  // normal2 と dir21 に直交するベクトルを dir22 に返す
  cvCrossProduct(&normal2, &dir21, &dir22);
  cvNormalize(&dir22, &dir22);

  r1 = c1->radius;
  r2 = c2->radius;

  for (i = 0; i < 3; i++)
    {
      cv1 = c1->center[i];
      nv1 = mat1[0][i];
      pv1 = mat1[1][i];
      qv1 = mat1[2][i];
      cv2 = c2->center[i];
      nv2 = mat2[0][i];
      pv2 = mat2[1][i];
      qv2 = mat2[2][i];
      orientation[0][i] = cv1 + r1 * nv1;
      orientation[1][i] = cv1 - r1 * nv1;
      orientation[2][i] = cv1 + r1 * pv1;
      orientation[3][i] = cv1 - r1 * pv1;
      orientation[4][i] = cv1 + r1 * qv1;
      orientation[5][i] = cv1 - r1 * qv1;
      orientation[6][i] = cv2 + r2 * nv2;
      orientation[7][i] = cv2 - r2 * nv2;
      orientation[8][i] = cv2 + r2 * pv2;
      orientation[9][i] = cv2 - r2 * pv2;
      orientation[10][i] = cv2 + r2 * qv2;
      orientation[11][i] = cv2 - r2 * qv2;
    }

  return 0;
}

// 回転を求める
static double
findRotation(double rotation[3][3], CvMat* before_rotated,
             CvMat* after_rotated, const double* weight, const int nSample)
{
  double matN[4][4];
  double eigen_value[4];
  double eigen_vector[4][4];
  double weight_sum = 0.0;
  double maxE;
  CvMat eigenvalue, eigenvector;
  CvMat MatN;

  quaternion_t q;
  double R[3*3];

  int i, j, k;

  MatN = cvMat(4, 4, CV_64FC1, matN);
  cvSetZero(&MatN);

  for (i = 0; i < nSample; i++)
    {
      double w = weight ? weight[i] : 1.0;
      double s[3][3];

      weight_sum += w;

      for (j = 0; j < 3; ++j)
        {
          for (k = 0; k < 3; ++k)
            {
              s[j][k] = cvmGet(before_rotated, i, j) * cvmGet(after_rotated, i, k);
            }
        }

      matN[0][0] += w * (s[0][0] + s[1][1] + s[2][2]);
      matN[0][1] += w * (s[1][2] - s[2][1]);
      matN[0][2] += w * (s[2][0] - s[0][2]);
      matN[0][3] += w * (s[0][1] - s[1][0]);

      //matN[1][0] = matN[0][1];
      matN[1][1] += w * (s[0][0] - s[1][1] - s[2][2]);
      matN[1][2] += w * (s[0][1] + s[1][0]);
      matN[1][3] += w * (s[0][2] + s[2][0]);

      //matN[2][0] = matN[0][2];
      //matN[2][1] = matN[1][2];
      matN[2][2] += w * (-s[0][0] + s[1][1] - s[2][2]);
      matN[2][3] += w * (s[1][2] + s[2][1]);

      //matN[3][0] = matN[0][3];
      //matN[3][1] = matN[1][3];
      //matN[3][2] = matN[2][3];
      matN[3][3] += w * (-s[0][0] - s[1][1] + s[2][2]);
    }

  // 対称な要素をコピー
  for (i = 1; i < 4; ++i)
    {
      for (j = 0; j < i; ++j)
        {
          matN[i][j] = matN[j][i];
        }
    }

  // 最小固有値に対応する固有ベクトル算出
  eigenvalue = cvMat(4, 1, CV_64FC1, eigen_value);
  eigenvector = cvMat(4, 4, CV_64FC1, eigen_vector);
  cvEigenVV(&MatN, &eigenvector, &eigenvalue);
  maxE = eigen_value[0];

  // 要素を代入
  quat_re(q) = eigen_vector[0][0];
  for (i = 0; i < 3; ++i)
    {
      quat_im(q, i) = eigen_vector[0][i+1];
    }

  // 回転行列に変換して結果を代入
  quat_R_from_q(R, 3, q);
  for (i = 0; i < 3; ++i)
    {
      for (j = 0; j < 3; ++j)
        {
          rotation[i][j] = R[i + 3*j];
        }
    }

  return maxE / weight_sum;
}

// 重心を求める
static void
getDataCenter(CvMat* center, CvMat* before_centered,
              const double* weight, const int nSample)
{
  double sum[3] = { 0.0, 0.0, 0.0 };
  double wdt[3];
  double tWeight = 0.0;
  CvMat data;
  CvMat wdata;
  CvMat Sum;
  int i;

  if (nSample <= 0)
    {
      return;                   // do nothing
    }

  Sum = cvMat(1, 3, CV_64FC1, sum);

  if (weight == NULL)
    {                           // 重みなし
      for (i = 0; i < nSample; i++)
        {
          cvGetRow(before_centered, &data, i);
          cvAdd(&Sum, &data, &Sum);
          tWeight += 1.0;
        }
    }
  else
    {                           // 重みあり
      wdata = cvMat(1, 3, CV_64FC1, wdt);
      for (i = 0; i < nSample; i++)
        {
          cvGetRow(before_centered, &data, i);
          cvScale(&data, &wdata, weight[i]);
          cvAdd(&Sum, &wdata, &Sum);
          tWeight += weight[i];
        }
    }

  if (tWeight != 0.0)
    {
      cvScale(&Sum, center, 1.0 / tWeight);
    }
  return;
}

// 重心を求めて移動する
static void
moveCenter(CvMat* centered, CvMat* center, CvMat* before_centered,
           const double* weight, const int nSample)
{
  CvMat src, dst;
  int i;

  if (nSample <= 0)
    {
      return;                   // do nothing
    }

  getDataCenter(center, before_centered, weight, nSample);

  for (i = 0; i < nSample; i++)
    {
      cvGetRow(before_centered, &src, i);
      cvGetRow(centered, &dst, i);
      cvSub(&src, center, &dst);
    }
  return;
}

// 3次元空間中の点について変換前の点と変換後の点の対応付けから回転と移動を求める
static void
findTransformationMatrix(double rotation[3][3],        // 回転行列
                         double translation[3],        // 移動ベクトル
                         CvMat* before_moved,          // 変換前の点座標配列
                         CvMat* after_moved,           // 変換後の点座標配列
                         const double* weight,         // 各データの重み配列
                         int nSample)                  // サンプル数
{
  CvMat* before_rotated;
  CvMat* after_rotated;
  CvMat before_gc;
  CvMat after_gc;
  CvMat after_rot;
  double bgc[3];
  double agc[3];
  double arot[3];
  CvMat Rmat;
  CvMat Tvec;

  // 移動後の回転を求めるための領域の確保
  before_rotated = cvCreateMat(nSample, 3, CV_64FC1);
  after_rotated = cvCreateMat(nSample, 3, CV_64FC1);

  // 回転前と回転後の重心をそれぞれ求めて移動する
  before_gc = cvMat(1, 3, CV_64FC1, bgc);
  moveCenter(before_rotated, &before_gc, before_moved, weight, nSample);
  after_gc = cvMat(1, 3, CV_64FC1, agc);
  moveCenter(after_rotated, &after_gc, after_moved, weight, nSample);

  // 重心が一致する標本間での回転を求める
  findRotation(rotation, before_rotated, after_rotated, weight, nSample);

  // 重心の差は回転前の重心を回転させて, 回転後の重心から引いたものになる
  Rmat = cvMat(3, 3, CV_64FC1, rotation);
  before_gc = cvMat(3, 1, CV_64FC1, bgc);
  after_rot = cvMat(3, 1, CV_64FC1, arot);
  cvMatMul(&Rmat, &before_gc, &after_rot);
  after_gc = cvMat(3, 1, CV_64FC1, agc);
  Tvec = cvMat(3, 1, CV_64FC1, translation);
  cvSub(&after_gc, &after_rot, &Tvec);

  cvReleaseMat(&before_rotated);
  cvReleaseMat(&after_rotated);
  return;
}

// mc1, mc2 を dc1, dc2 に移動させる座標変換行列を求める
static void
makeMatchTransformationForPairedCircle(Circle *dc1, Circle *dc2,
                                       Circle *mc1, Circle *mc2,
                                       double mat[4][4])
{
  double Mpoint[12][3], Dpoint[12][3];
  static CvMat before, after;
  int sts, i, j;

  // 単位行列を設定
  for (i = 0; i < 4; i++)
    {
      for (j = 0; j < 4; j++)
        {
          mat[i][j] = 0.0;
          if (i == j)
            {
              mat[i][j] = 1.0;
            }
        }
    }

  // モデルとシーンについて 2 つの円の姿勢を示す 12 個のベクトルデータを作る
  sts = makeOrientationDataForCirclePair(dc1, dc2, Dpoint);
  if (sts)
    {
      return;
    }
  sts = makeOrientationDataForCirclePair(mc1, mc2, Mpoint);
  if (sts)
    {
      return;
    }

  before = cvMat(12, 3, CV_64FC1, Mpoint);
  after = cvMat(12, 3, CV_64FC1, Dpoint);

  // before 点列 から after 点列への変換行列を求める
  double rotation[3][3], translation[3];
  findTransformationMatrix(rotation, translation, &before, &after, NULL, 12);

  for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 3; j++)
        {
          mat[i][j] = rotation[i][j];
        }
      mat[i][3] = translation[i];
    }
  mat[3][3] = 1;
  return;
}

// 二つの円の半径が同じかどうかを判別する
static int
isSameRadius(Circle* dcir, Circle* mcir, double tolerance)
{
  double mr, dr;
  double rdif;
  double thr;

  mr = mcir->radius;
  dr = dcir->radius;
  rdif = fabs(mr - dr);
  thr = tolerance * mr;
  return (rdif < thr);
}

// モデル円とシーン円の対応表のメモリ確保と初期化
static int**
createCorrespondenceTable(int rowlen, int collen)
{
  int i;
  int** cortbl = NULL;
  size_t memsize;

  memsize = rowlen * sizeof(int*);
  cortbl = (int**) calloc(1, memsize);
  if (cortbl == NULL)
    {
      return NULL;
    }
  for (i = 0; i < rowlen; i++)
    {
      memsize = collen * sizeof(int);
      cortbl[i] = (int*) malloc(memsize);
      if (cortbl[i] == NULL)
        {
          break;
        }
      memset(cortbl[i], -1, memsize);
    }
  return cortbl;
}

// モデル円とシーン円の対応表のメモリ解放
static void
freeCorrespondenceTable(int** cortbl, int rowlen)
{
  if (cortbl)
    {
      int i;
      for (i = 0; i < rowlen; i++)
        {
          free(cortbl[i]);
        }
      free(cortbl);
    }
  return;
}

// モデル円とシーン円の対応表作成
static int**
makeCorrespondenceTable(Circle* dcir, Circle* mcir,
                        int dnum, int mnum, double tolerance, int* nscene)
{
  int i, j, k;
  int* pwork;
  int kmax = 0;
  int** cortbl = NULL;
  size_t tblsize;

  // モデル円とシーン円の対応表領域確保
  cortbl = createCorrespondenceTable(mnum, dnum);
  if (cortbl == NULL)
    {
      return NULL;
    }

  for (i = 0; i < mnum; i++)
    {
      k = 0;
      // モデル円とシーン円で半径の近いものを見つけて対応表を作成
      if (mcir[i].side == M3DF_FRONT)
        {
          for (j = 0; j < dnum; j++)
            {
              if (dcir[j].side == M3DF_BACK)
                {
                  continue;
                }
              if (isSameRadius(&dcir[j], &mcir[i], tolerance))
                {
                  cortbl[i][k] = j;
                  k++;
                }
            }
        }
      if (k == 0)
        {
          // 対応シーンデータがないとき領域開放
          free(cortbl[i]);
          cortbl[i] = NULL;
        }
      else
        {
          // -1 を終端にして余分な領域を開放する
          tblsize = (k + 1) * sizeof(int);
          pwork = cortbl[i];
          pwork = (int*) realloc(pwork, tblsize);
          if (pwork != NULL)
            {
              cortbl[i] = pwork;
            }
          else
            {
              freeCorrespondenceTable(cortbl, mnum);
              return NULL;
            }
          if (kmax < k)
            {
              kmax = k;
            }
        }
    }
  // 対応シーン円数の最大値を返す
  *nscene = kmax;

  return cortbl;
}

static int*
createCorrespondenceTable(int** cortbl, int nm, int* ncor)
{
  int* mctbl = NULL;
  int i, nc = 0;
  size_t memsize;

  memsize = nm * sizeof(int);
  mctbl = (int*) malloc(memsize);
  if (mctbl != NULL)
    {
      memset(mctbl, -1, memsize);
      // 対応するシーン円を持つモデル円番号表の作成
      for (nc = 0, i = 0; i < nm; i++)
        {
          if (cortbl[i] != NULL)
            {
              mctbl[nc] = i;
              nc++;
            }
        }
    }
  *ncor = nc;
  return mctbl;
}

// ２円の組み合わせテーブル作成
static PairTable*
makePairTable(Circle* mcir, int** cortbl, int tblen, int* nmodel)
{
  PairTable* pairtbl = NULL;
  int* mctbl;
  int nm, i, j, k;
  int mc1, mc2;
  size_t memsize;
  double dist;

  // シーン円との対応があるモデル円番号表の作成
  mctbl = createCorrespondenceTable(cortbl, tblen, &nm);
  if (mctbl == NULL)
    {
      return NULL;
    }

  // 対応候補表領域の確保
  memsize = nm * (nm - 1) * sizeof(PairTable) / 2;
  pairtbl = (PairTable*) malloc(memsize);
  if (pairtbl == NULL)
    {
      free(mctbl);
      return NULL;
    }
  memset(pairtbl, -1, memsize);
  // シーン円と対応があるモデル円どうしの組合せを取り、中心間距離を求める
  k = 0;
  for (i = 0; i < nm; i++)
    {
      for (j = i + 1; j < nm; j++)
        {
          mc1 = mctbl[i];
          mc2 = mctbl[j];
          pairtbl[k].mc1 = mc1;
          pairtbl[k].mc2 = mc2;

          {
            // 中心間距離の計算
            dist = getDistanceV3(mcir[mc1].center, mcir[mc2].center);
          }
          pairtbl[k].cdist = dist;
          ++k;
        }
    }
  // 中心間距離の大きい順に並べ替え
  qsort(pairtbl, k, sizeof(PairTable), compareCenterDistance);
  *nmodel = k;

  free(mctbl);
  return pairtbl;
}

// 組み合わせテーブルのメモリ解放
static void
freePairTable(PairTable* pairtbl, int nmodel)
{
  if (pairtbl)
    {
      free(pairtbl);
    }
  return;
}

// 姿勢行列で変換したmcirとdcirの法線の違いをチェック
static int
testNormal(Circle* scir, Circle* mcir, double trans[4][4], double diff)
{
  CvMat Rmat;
  CvMat snormal;
  CvMat mnormal;
  CvMat rnormal;
  double rmat[3][3];
  double rn[3];
  double dot;
  int i, j;

  for (j = 0; j < 3; j++)
    {
      for (i = 0; i < 3; i++)
        {
          rmat[j][i] = trans[j][i];
        }
    }

  Rmat = cvMat(3, 3, CV_64FC1, rmat);

  snormal = cvMat(3, 1, CV_64FC1, scir->normal);
  mnormal = cvMat(3, 1, CV_64FC1, mcir->normal);
  rnormal = cvMat(3, 1, CV_64FC1, rn);

  // モデルの法線ベクトルに変換行列の回転部分をかける
  cvMatMul(&Rmat, &mnormal, &rnormal);

  // モデルとシーンの法線がなす角について cosθ を求める
  dot = cvDotProduct(&rnormal, &snormal);
  if (fabs(dot) < diff)
    {
      return -1;                // 不合格
    }
  else
    {
      return 0;                 // 合格とみなす
    }
}

// モデルからシーンへの変換行列の算出
static int
calcTransformationMatrix(Circle* sc1, Circle* sc2,
                         Circle* mc1, Circle* mc2,
                         double limit, double mat[4][4])
{
  int sts;

  // モデル mc1, mc2 をシーン sc1, sc2 に変換する行列の計算
  makeMatchTransformationForPairedCircle(sc1, sc2, mc1, mc2, mat);
  // 照合結果について法線ベクトルチェック. OKのときは sts == 0
  sts = testNormal(sc1, mc1, mat, limit);
  if (sts == 0)
    {
      sts = testNormal(sc2, mc2, mat, limit);
    }
  return sts;
}

// ２円を使った照合
// 戻り値：認識結果
Match3Dresults
matchPairedCircles(Features3D& scene,          // シーンの３次元特徴情報
                   Features3D& model,          // モデルの３次元特徴情報
                   double tolerance,           // 照合の許容値
                   StereoPairing& pairing)     // ステレオペアの情報
{
  Match3Dresults Match = { 0 };
  MatchResult* memory = NULL;
  Circle* scenes = NULL;
  Circle* models = NULL;
  PairTable* pairtbl = NULL;
  int** cortbl = NULL;
  int i, j, k;
  int nmc, nsc, nm, ns;
  int* sc1;
  int* sc2;
  int m1, m2, s1, s2;
  int sts;
  size_t p;
  size_t maxResultNum;
  size_t maxAllocNum;
  size_t addsize;
  size_t extended;
  double mcdist, scdist, ddif, mddif;
  double normthr;
  double score_weight = 2.0;    // 単円と2円を差別化するために導入

  nmc = model.numOfCircles;
  nsc = scene.numOfCircles;

  // 円がないときは終了
  if (nmc <= 1 || nsc <= 1)
    {
      return Match;
    }

  scenes = scene.Circles;
  models = model.Circles;

  tolerance /= 100.0;

  // モデル円とシーン円で半径の近いものを見つけて対応表を作成
  cortbl = makeCorrespondenceTable(scenes, models, nsc, nmc, tolerance, &ns);
  if (cortbl == NULL)
    {
      Match.error = VISION_MALLOC_ERROR;
      goto ending;
    }

  // シーン円と対応があるモデル円どうしの組合せを取り、中心間距離順に並べる
  pairtbl = makePairTable(models, cortbl, nmc, &nm);
  if (pairtbl == NULL || nm == 0)
    {
      Match.error = VISION_MALLOC_ERROR;
      goto ending;
    }

  // 結果配列数の指定可能な最大値
  maxAllocNum  = INT_MAX / sizeof(MatchResult);
  // 結果配列数初期値
  maxResultNum = 4096;
  // 結果配列数の拡張時増分値
  addsize = 1024;

  memory = (MatchResult*) calloc(maxResultNum, sizeof(MatchResult));
  if (memory == NULL)
    {
      Match.error = VISION_MALLOC_ERROR;
      goto ending;
    }

  Match.Results = memory;

  // 認識結果の法線角度許容差閾値
  normthr = cos(90.0 * tolerance * (M_PI / 180.0));

  p = 0;
  for (i = 0; i < nm; i++)
    {
      // モデル組合せの円番号を取得する
      m1 = pairtbl[i].mc1;
      m2 = pairtbl[i].mc2;
      mcdist = pairtbl[i].cdist;
      // 許容する距離差
      mddif = mcdist * tolerance;
      // モデル円に対応するシーン円番号配列を取り出す
      sc1 = cortbl[m1];
      sc2 = cortbl[m2];
      // シーン円番号の組合せを生成する
      for (j = 0; sc1[j] != -1; j++)
        {
          for (k = 0; sc2[k] != -1; k++)
            {
              // 同じシーン円についてはスキップ
              if (sc1[j] == sc2[k])
                {
                  continue;
                }

              // シーン円番号を得る
              s1 = sc1[j];
              s2 = sc2[k];
              // シーン円の中心間距離を求める
              scdist = getDistanceV3(scenes[s1].center, scenes[s2].center);
              // 中心間距離がモデルとシーンで異なるものは除外
              ddif = fabs (mcdist - scdist);
              if (ddif >= mddif)
                {
                  continue;
                }

              // モデルとシーンの対応が正しいと思えるので位置姿勢を求める
              sts = calcTransformationMatrix(&scenes[s1], &scenes[s2],
                                             &models[m1], &models[m2], normthr,
                                             Match.Results[p].mat);
              if (sts != 0)
                {
                  continue;
                }
              Match.Results[p].n = p;
              Match.Results[p].type = 2;
              Match.Results[p].score = 0.0;
              Match.Results[p].scene[0] = scenes[s1].n;
              Match.Results[p].scene[1] = scenes[s2].n;
              Match.Results[p].model[0] = models[m1].n;
              Match.Results[p].model[1] = models[m2].n;
              // 認識結果の行列を７次元のベクトル（位置＋回転）としてもあらわす
              getPropertyVector(Match.Results[p].mat, Match.Results[p].vec);
              // 結果配列数が不足したら拡張する
              if (++p >= maxResultNum)
                {
                  extended = maxResultNum + addsize;
                  // 指定可能な最大値を越えるときは拡張中止
                  if (extended > maxAllocNum)
                    {
                      goto outOfLoop;
                    }
                  maxResultNum = extended;
                  extended *= sizeof(MatchResult);
                  memory = (MatchResult*) realloc(Match.Results, extended);
                  // 拡張できなかったときはそこまでの結果を出す
                  if (memory != NULL)
                    {
                      Match.Results = memory;
                    }
                  else
                    {
                      goto outOfLoop;
                    }
                }
            }
        }
    }

outOfLoop:

  if (p)
    {
      Match.numOfResults = p;
      memory = (MatchResult*) realloc(Match.Results, sizeof(MatchResult) * p);
      if (memory)
        {
          Match.Results = memory;
        }
      // 認識結果の評価値を作成する．完全重複した結果は評価しない．
      getResultScore(Match.Results, p, &model, pairing, score_weight);
    }
  else
    {
      freeMatch3Dresults(&Match);
    }

ending:
  freeCorrespondenceTable(cortbl, nmc);
  freePairTable(pairtbl, nm);
  return Match;
}
