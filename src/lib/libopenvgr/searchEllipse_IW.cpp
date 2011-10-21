/* -*- coding: utf-8 -*-
 searchEllipse_IW.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

// 2011.08.14 max_ellipse を max_f2D に変更

/* 2011/08/25 loop において、以下のような場合に対応する
   [0,19] [1,20] (0,20) が楕円条件を満たしているとき
   従来は、以下のように処理をしていた
   [0,19]から(-1,19)に進もうとして失敗、[1,20]に
   [1,20]から(0,20)に進む
   (0,20)から[0,19]に進む
   この結果、turnは-1
   このようになるのは、[0,19]において隣接した(0,20)に進もうとしないから

   しかし、決め打ちで[0,19]から(0,20)のみに進もうとすると、やはり問題がある
   [0,19]NG
   (0,20)OK
   (0,21)OK
   [1,20]OK
   (1,21)OK
   だと、
   [1,20]->(1,21)->(0,21)->(0,20)->(1,21)->...
   として無限ループになる

   つまり、minLen の場合に、s-=1 の場合と g+=1の場合の二つ隣接した場合があるので、
   それらをどちらも試さないといけない

   最初はs-=1 の場合、次は g+=1 の場合
   check_track_start() の編集
 */
/*
  extractFeature.cpp に組み込むため、extract_ellipses()と同様の引数にする
  extract_ellipses_inchworm();
  searchEllipse_inchworm
 */
/*
  2011/09/01
  全周でminLen付近で楕円がフィットすると skip_body() でtrackの開始点がみつからない
  これを解決するために、skip_body()を改造し、len方向に探索する
  領域に穴があったときのために、trackの開始点を保存し、戻ってきたときに　turn < 0 ならばさらに
  skip_body()をよぶ
 */

#include <stdio.h>

#include "extractFeature_old.h"
#include "vectorutil.h"
#include "ellipseIW.h"

#include "paramEllipseIW.h"

// 未定状態の max_curve_len の値
#define MAX_CURVE_LEN_UNDEFINED (-1)

#define TRACKING_OFF (0)
#define TRACKING_ON  (1)

// 四近傍方向の数
#define NDIR4 (4)

#define DIR_START_DEC (1)
#define DIR_START_INC (3)
#define DIR_GOAL_DEC  (2)
#define DIR_GOAL_INC  (0)

#define TURN_LEFT     (1)
#define TURN_FORWARD  (0)
#define TURN_RIGHT    (3)
#define TURN_BACKWARD (2)

#define DS_SHIFT  (1)
#define DG_SHIFT  (1)

static  int ds[NDIR4] = {0, -1, 0, 1};
static  int dg[NDIR4] = {1, 0, -1, 0};

#define CHECK_NEXT_FAIL    (0)
#define CHECK_NEXT_SUCCESS (1)

#define CHECK_TRACK_FAIL    (0)
#define CHECK_TRACK_SUCCESS (1)

#define POINT_TO_ELLIPSE_FAIL    (0)
#define POINT_TO_ELLIPSE_SUCCESS (1)

#define LOOP_CONTINUE    (0)
#define LOOP_EXIT_WHOLE  (1)
#define LOOP_EXIT_NORMAL (2)

int
mod_nPoint(int  n,
           int  nPoint)
{
  while (n < 0)
    {
      n += nPoint;
    }

  return n % nPoint;
}

static double
get_angle_sa(double tmp[NDIM3][NDIM3],
             int  r,
             double vdst[NDIM3])
{
  CvMat vVec0, vVec1,vDst;
  double  vec0[NDIM3], vec1[NDIM3], len;
  int k;
  int rp1;

  // tmp[r] の正規化=>vec0
  for (k = 0; k < NDIM3; k++)
    {
      vec0[k] = tmp[r][k];
    }
  vVec0 = cvMat(NDIM3,1,CV_64FC1, vec0);
  len = sqrt(cvDotProduct(&vVec0, &vVec0));
  if (len == 0.0)
    {
      return 0.0;
    }
  for (k = 0; k < NDIM3; k++)
    {
      vec0[k] /= len;
    }

  // tmp[rp1] の正規化=>vec1
  rp1 = (r+1)%NDIM3;
  for (k = 0; k < NDIM3; k++)
    {
      vec1[k] = tmp[rp1][k];
    }
  vVec1 = cvMat(NDIM3,1,CV_64FC1, vec1);
  len = sqrt(cvDotProduct(&vVec1, &vVec1));
  if (len == 0.0)
    {
      return 0.0;
    }
  for (k = 0; k < NDIM3; k++)
    {
      vec1[k] /= len;
    }

  // vec0 と vec1の外積を計算 長さは sin(t)
  vDst = cvMat(NDIM3,1,CV_64FC1, vdst);
  cvCrossProduct(&vVec0, &vVec1, &vDst);

  // 外積ベクトルの長さを計算
  len = sqrt(cvDotProduct(&vDst, &vDst));
  for (k = 0; k < NDIM3; k++)
    {
      vdst[k] /= len;
    }

  return len;
}

// eigen3x3() 内で get_angle_sa() の戻り値を判定するときチェック
#define CHECK_SA_OK (0)
#define CHECK_SA_NG (1)

// eigen3x3() 対象行列に限定しない固有値、固有ベクトル計算関数
// m: 3x3 行列（入力)
// evec 固有ベクトル（出力）
// eval 固有値（出力）
// thresh A-lambda*I行列のランク判定閾値
//       二つのベクトルのsinの絶対値の最大値がこの値より小さければエラー

static int
eigen3x3(const double m[NDIM3][NDIM3],
         double evec[NDIM3][NDIM3],
         double eval[NDIM3],
         double thresh_sa)
{
  double  coef[NCOEFCUBIC], tmp[NDIM3][NDIM3];
  CvMat vCoef, vE;
  int nRoot, ie, r, c;
  int max_r;
  double  max_sa, tmp_sa;
  double  max_vec[NDIM3], vec[NDIM3];
  int check[NDIM3];
  int count_check;
  int i, j;

  coef[0] = -1;
  coef[1] = m[0][0] + m[1][1] + m[2][2];
  coef[2] = (- m[0][0] * m[1][1]
             - m[1][1] * m[2][2]
             - m[2][2] * m[0][0]
             + m[1][2] * m[2][1]
             + m[0][2] * m[2][0]
             + m[0][1] * m[1][0]);
  coef[3] = (m[0][0] * m[1][1] * m[2][2]
             + m[0][1] * m[1][2] * m[2][0]
             + m[0][2] * m[1][0] * m[2][1]
             - m[0][0] * m[1][2] * m[2][1]
             - m[0][1] * m[1][0] * m[2][2]
             - m[0][2] * m[1][1] * m[2][0]);

  vCoef = cvMat (NCOEFCUBIC, 1, CV_64FC1, coef);
  vE = cvMat (NDIM3, 1, CV_64FC1, eval);

  nRoot = cvSolveCubic (&vCoef, &vE);

  count_check = 0;
  for (ie = 0; ie < nRoot; ie++)
    {
      for (r = 0; r < NDIM3; r++)
        {
          for (c = 0; c < NDIM3; c++)
            {
              tmp[r][c] = m[r][c];
            }
        }
      for (r = 0; r < NDIM3; r++)
        {
          tmp[r][r] -= eval[ie];
        }

      max_r = -1;
      max_sa = -2.0;
      max_vec[0] = max_vec[1] = max_vec[2] = 0.0;
      for (r = 0; r < NDIM3; r++)
        {
          tmp_sa = get_angle_sa(tmp, r, vec);
          if (max_r == -1 || tmp_sa > max_sa)
            {
              max_r = r;
              max_sa = tmp_sa;
              for (c = 0; c < NDIM3; c++)
                {
                  max_vec[c] = vec[c];
                }
            }
        }

      if (max_sa > thresh_sa)
        {
          check[ie] = CHECK_SA_OK;
          count_check++;
        }
      else
        {
          check[ie] = CHECK_SA_NG;
        }
      for (c = 0; c < NDIM3; c++)
        {
          evec[ie][c] = max_vec[c];
        }
    }

  if (count_check == 0) return 0;

  // 不適な結果を排除
  if (count_check < nRoot)
    {
      for (i = j = 0; i < nRoot; i++)
        {
          if (j < i && check[i] == CHECK_SA_OK)
            {
              eval[j] = eval[i];
              for (c = 0; c < NDIM3; c++)
                {
                  evec[j][c] = evec[i][c];
                }
              j++;
            }
        }

      nRoot = j;
    }

  return nRoot;
}

#define ADD_NEW_ELLIPSE_OK  (1)
#define ADD_NEW_ELLIPSE_NG  (0)

static int
add_new_ellipse(Features2D_old* f2D,
                int start,
                int goal,
                Feature2D_old* max_f2D,
                const ParamEllipseIW* paramE,
                int nPoint,
                int iTrack,
                const int* point)
{
  int i;

  /* dummy for test*/
  /*
  printf("id:%d\n", f2D->nFeature);
  printf("  coef   = %g %g %g %g %g %g\n",
   max_f2D->coef[0], max_f2D->coef[1], max_f2D->coef[2],
   max_f2D->coef[3], max_f2D->coef[4], max_f2D->coef[5]);
  printf("  center = %g %g\n",
   max_f2D->center[0], max_f2D->center[1]);
  printf("  rad    = %g %g\n",
   max_f2D->axis[0], max_f2D->axis[1]);
  printf("  axis   = (%g %g) (%g %g)\n",
   max_f2D->ev[0][0], max_f2D->ev[0][1],
   max_f2D->ev[1][0], max_f2D->ev[1][1]);
  printf("  err= %g\n", max_f2D->error);
  */
  // copy max_f2D to feature

  if (f2D->nFeature >= f2D->nAlloc)
    {                           // 記憶域を確保する
      if (expandFeatures(f2D) == NULL)
        {
          return ADD_NEW_ELLIPSE_NG;
        }
    }

  memcpy(&f2D->feature[f2D->nFeature], max_f2D, sizeof(Feature2D_old));

  f2D->feature[f2D->nFeature].type = ConicType_Ellipse;
  f2D->feature[f2D->nFeature].all = nPoint;
  f2D->feature[f2D->nFeature].nTrack = iTrack;
  for (i = 0; i < NDIM2; i++)
    {
      f2D->feature[f2D->nFeature].startPoint[i]
	= point[mod_nPoint(max_f2D->start, nPoint)*2+i];
    }
  for (i = 0; i < NDIM2; i++)
    {
      f2D->feature[f2D->nFeature].endPoint[i]
	= point[mod_nPoint(max_f2D->end, nPoint)*2+i];
    }
  for (i = 0; i < NDIM2; i++)
    {
      f2D->feature[f2D->nFeature].startSPoint[i]
	= point[mod_nPoint(max_f2D->start, nPoint)*2+i];
    }
  //double middleSPoint[2];       //!< 点列の中間の位置
  for (i = 0; i < NDIM2; i++)
    {
      f2D->feature[f2D->nFeature].endSPoint[i]
	= point[mod_nPoint(max_f2D->end, nPoint)*2+i];
    }

  ++(f2D->nFeature);

  return ADD_NEW_ELLIPSE_OK;
}

static void
advance_next_curve(int  ds,
                   int  dg,
                   int* start,
                   int* goal,
                   int* len,
                   int* max_curve_len,
                   Ellipse* ellipse,
                   Feature2D_old* max_f2D,
                   const ParamEllipseIW* paramE
                   )
{
  int i, j;

  *start += ds;
  *goal += dg;
  *len = *goal - *start + 1;

  // *len > MinLength のとき
  if (max_curve_len)
    {
      if (*max_curve_len == MAX_CURVE_LEN_UNDEFINED
          || *max_curve_len < *len)
        {
          *max_curve_len = *len;

          for (i = 0; i < NDIM_CONIC_FULL; i++)
            {
              max_f2D->coef[i] = ellipse->coef[i];
            }
          for (i = 0; i < NDIM2; i++)
            {
              max_f2D->center[i] = ellipse->center[i];
            }
          //for (i = 0; i < NDIM2; i++)
          //{
          //max_f2D->startPoint[i] = point[(*start)*2+i];
          //}
          //for (i = 0; i < NDIM2; i++)
          //{
          //max_f2D->endPoint[i] = point[(*goal)*2+i];
          //}
          max_f2D->start = *start;
          max_f2D->end = *goal;
          //  int all;                      //!< 輪郭全体の点数（nPoint）
          //for (i = 0; i < NDIM2; i++)
          //{
          //max_f2D->startSPoint[i] = point[(*start)*2+i];
          //}
          //double middleSPoint[2];       //!< 点列の中間の位置
          //for (i = 0; i < NDIM2; i++)
          //{
          //max_f2D->endSPoint[i] = point[(*goal)*2+i];
          //}
          for (j = 0; j < NAXIS; j++)
            {
              for (i = 0; i < NDIM2; i++)
                {
                  max_f2D->ev[j][i] = ellipse->axis[j][i];    //!< 楕円の回転行列
                }
            }
          for (j = 0; j < NAXIS; j++)
            {
              max_f2D->axis[j] = ellipse->rad[j];             //!< 楕円の長半径、短半径
            }
          //int nPoints;                  //!< 特徴抽出に使われた点数
          //int nTrack;                   //!< 輪郭番号
          switch (paramE->Condition)
            {
            case ELLIPSE_CONDITION_MEAN:
              max_f2D->error = ellipse->meanError;
              break;
            case ELLIPSE_CONDITION_MAX:
              max_f2D->error = ellipse->maxError;
              break;
            }
          //double lineLength;            //!< 直線の長さ
          //double lineLength1;           //!< 双曲線の線分1の長さ
          //double lineLength2;           //!< 双曲線の線分2の長さ
          //double lineAngle;             //!< 双曲線の2線分のなす角度
        }
    }
  return;
}

void
addArcSum(SumSet* sum,
          const int* pointX,
          const double* offsetD)
{
  double  x, y;

  x = (double)pointX[0] - offsetD[0];
  y = (double)pointX[1] - offsetD[1];

  sum->x4 += x*x*x*x;
  sum->x3y += x*x*x*y;
  sum->x2y2 += x*x*y*y;
  sum->xy3 += x*y*y*y;
  sum->y4 += y*y*y*y;
  sum->x3 += x*x*x;
  sum->x2y += x*x*y;
  sum->xy2 += x*y*y;
  sum->y3 += y*y*y;
  sum->x2 += x*x;
  sum->xy += x*y;
  sum->y2 += y*y;
  sum->x += x;
  sum->y += y;
  sum->n++;

  return;
}

static void
subArcSum(SumSet* sum,
          const int* pointX,
          const double* offsetD)
{
  double  x, y;

  x = (double)pointX[0] - offsetD[0];
  y = (double)pointX[1] - offsetD[1];

  sum->x4 -= x*x*x*x;
  sum->x3y -= x*x*x*y;
  sum->x2y2 -= x*x*y*y;
  sum->xy3 -= x*y*y*y;
  sum->y4 -= y*y*y*y;
  sum->x3 -= x*x*x;
  sum->x2y -= x*x*y;
  sum->xy2 -= x*y*y;
  sum->y3 -= y*y*y;
  sum->x2 -= x*x;
  sum->xy -= x*y;
  sum->y2 -= y*y;
  sum->x -= x;
  sum->y -= y;
  sum->n--;

  return;
}


static void
modify_sum(const int* point,
           int  nPoint,
           OffsetProp* offsetProp,
           SumSet* sum,
           int  start,
           int  goal,
           int  ds0,
           int  dg0)
{
  start = start % nPoint;
  goal = goal % nPoint;
  switch(ds0)
    {
    case -1:
      addArcSum(sum, &((int *)point)[((start+nPoint-1)%nPoint)*2], offsetProp->d);
      break;
    case 1:
      subArcSum(sum, &((int *)point)[((start+nPoint)%nPoint)*2], offsetProp->d);
      break;
    }

  switch(dg0)
    {
    case -1:
      subArcSum(sum, &((int *)point)[((goal+nPoint)%nPoint)*2], offsetProp->d);

      break;
    case 1:
      addArcSum(sum, &((int *)point)[((goal+nPoint+1)%nPoint)*2], offsetProp->d);
      break;
    }

  return;
}

// ２次曲線の係数から、その性質を調べる
void
avec_to_ellipse(int k_min_error,
                Ellipse* ellipse
                )
{
  double D;
  double F;
  Ellipse *pe;
  double  *pa;
  double a[2][2];
  double e[2];
  double  p;
  int idim;

  pa = ellipse->a[k_min_error];
  pe = ellipse;

  // 4 a[0] a[2] - a[1]^2 = 1 , a[0]>0, a[2] > 0 にする
  p=sqrt(4*pa[0]*pa[2] - pa[1]*pa[1]);

  if(pa[0] < 0.0) p = -p;
  for(idim = 0; idim < NDIM_CONIC_FULL; idim++){
    pa[idim] /= p;
  }

  // 中心を求める 放物線は適用外なので注意！
  D = 4.0 * pa[0] * pa[2] - pa[1] * pa[1];
  pe->center[0] = -(2.0 * pa[2] * pa[3] - pa[1] * pa[4]) / D;
  pe->center[1] = -(2.0 * pa[0] * pa[4] - pa[1] * pa[3]) / D;

  // 中心を移動した場合の F は
  F = (pa[0] * pe->center[0] * pe->center[0]
       + pa[1] * pe->center[0] * pe->center[1]
       + pa[2] * pe->center[1] * pe->center[1]
       + pa[3] * pe->center[0]
       + pa[4] * pe->center[1]
       + pa[5]);
  // これにより a(x+xc)^2 + b(x+xc)(y+yc) + c(y+yc)^2 + F = 0 の形になる

  // 次にbを消すために回転を求める
  a[0][0] = pa[0];
  a[0][1] = a[1][0] = pa[1] / 2.0;
  a[1][1] = pa[2];

  eigenM22(e, pe->axis, a, VISION_EPS);

  // ev が回転行列 長軸を ev[0] と考えて回転
  double c = pe->axis[0][0];
  double s = pe->axis[0][1];
  double A = pa[0] * c * c + pa[1] * c * s + pa[2] * s * s;
  double C = pa[0] * s * s - pa[1] * c * s + pa[2] * c * c;

  // F による規格化
  if (F != 0.0)
    {
      A /= -(F);
      C /= -(F);
      // これで A x^2 + C y^2 = 1 の標準形
      //if ((A > 0.0) && (C > 0.0))
      {                       // 楕円
        pe->rad[0] = sqrt(1.0 / A);
        pe->rad[1] = sqrt(1.0 / C);
      }
    }
  else
    {
      pe->rad[0] = 0.0;
      pe->rad[1] = 0.0;
    }


  for (idim = 0; idim < NDIM_CONIC_FULL; idim++)
    {
      pe->coef[idim] = pa[idim];
    }

  return;
 }

// ２次曲線の係数と、特定の点からのおおよその距離を求める
// 簡易計算法なので厳密ではない
double
distanceAConic(const double coef[6],
               const int* point)
{
  double D = (coef[0] * (double)point[0] * (double)point[0] +
              coef[1] * (double)point[0] * (double)point[1] +
              coef[2] * (double)point[1] * (double)point[1] +
              coef[3] * (double)point[0] +
              coef[4] * (double)point[1] +
              coef[5]);
  const double dx = (2.0 * coef[0] * (double)point[0]
		     + coef[1] * (double)point[1] + coef[3]);
  const double dy = (2.0 * coef[2] * (double)point[1]
		     + coef[1] * (double)point[0] + coef[4]);
  const double inc = sqrt(dx * dx + dy * dy);

  if (inc < VISION_EPS)
    {
      return D / VISION_EPS;
    }
  return D / inc;
}

// calculate distance from points to ellipse
static void
eval_ellipse(int  start,
             int  goal,
             int  nPoint,
             const int* point,
             const double* coef,
             double* meanError,
             double* maxError)
{
  double  sum, e;
  int i;
  int nloop, i0[2], i1[2], np, loop;
  int modstart, modgoal;

  modstart = (start + nPoint) % nPoint;
  modgoal = (goal + nPoint) % nPoint;

  if (start - modstart == goal - modgoal)
    {
      nloop = 1;
      i0[0] = (start+nPoint)%nPoint;
      i1[0] = (goal+nPoint)%nPoint;
    }
  else
    {
      nloop = 2;
      i0[0] = (start + nPoint) % nPoint;
      i1[0] = nPoint-1;
      i0[1] = 0;
      i1[1] = goal % nPoint;
    }
  sum = 0.0;
  *maxError = 0.0;
  np = 0;
  for (loop = 0; loop < nloop; loop++)
    {
      for (i = i0[loop]; i <= i1[loop]; i++)
        {
          e = fabs(distanceAConic((double *)coef, &point[i*2]));
          sum += e;
          np++;
          if (e > *maxError)
            {
              *maxError = e;
            }
        }
    }

  *meanError = sum/(double)np;

  return;
}

static void
sum_to_P_static(const SumSet* sum,
                Ellipse* ellipse,
                OffsetProp* offsetProp)
{
  /*P00*/
  ellipse->P00[0][0] = sum->x4;
  ellipse->P00[0][1] = sum->x3y;
  ellipse->P00[0][2] = sum->x2y2;
  ellipse->P00[1][0] = sum->x3y;
  ellipse->P00[1][1] = sum->x2y2;
  ellipse->P00[1][2] = sum->xy3;
  ellipse->P00[2][0] = sum->x2y2;
  ellipse->P00[2][1] = sum->xy3;
  ellipse->P00[2][2] = sum->y4;

  /*P01*/
  ellipse->P01[0][0] = sum->x3;
  ellipse->P01[0][1] = sum->x2y;
  ellipse->P01[0][2] = sum->x2;
  ellipse->P01[1][0] = sum->x2y;
  ellipse->P01[1][1] = sum->xy2;
  ellipse->P01[1][2] = sum->xy;
  ellipse->P01[2][0] = sum->xy2;
  ellipse->P01[2][1] = sum->y3;
  ellipse->P01[2][2] = sum->y2;

  /*P10*/
  ellipse->P10[0][0] = sum->x3;
  ellipse->P10[0][1] = sum->x2y;
  ellipse->P10[0][2] = sum->xy2;
  ellipse->P10[1][0] = sum->x2y;
  ellipse->P10[1][1] = sum->xy2;
  ellipse->P10[1][2] = sum->y3;
  ellipse->P10[2][0] = sum->x2;
  ellipse->P10[2][1] = sum->xy;
  ellipse->P10[2][2] = sum->y2;

  /*P11*/
  ellipse->P11[0][0] = sum->x2;
  ellipse->P11[0][1] = sum->xy;
  ellipse->P11[0][2] = sum->x;
  ellipse->P11[1][0] = sum->xy;
  ellipse->P11[1][1] = sum->y2;
  ellipse->P11[1][2] = sum->y;
  ellipse->P11[2][0] = sum->x;
  ellipse->P11[2][1] = sum->y;
  ellipse->P11[2][2] = sum->n;

  ellipse->offset[0] = offsetProp->d[0];
  ellipse->offset[1] = offsetProp->d[1];

  return;
}

void
sum_to_P_dynamic(const SumSet* sum,
                 Ellipse* ellipse,
		 OffsetProp* offsetProp)
{
  double  dn;

  double  sx4, sy4, sx3y, sxy3, sx2y2;
  double  sx3, sxy2, sx2y, sy3;
  double  sx2, sxy, sy2;
  double  sx, sy;

  dn = sum->n;

  sx4 = (sum->x4 - 4 *sum->x * sum->x3 / dn
         + 6 *sum->x *sum->x * sum->x2 / (dn * dn)
         - 3 *sum->x *sum->x *sum->x *sum->x / (dn * dn * dn));
  sy4 = (sum->y4 - 4 *sum->y * sum->y3 / dn
         + 6 *sum->y *sum->y *sum->y2 / (dn * dn)
         - 3 *sum->y *sum->y *sum->y *sum->y / (dn * dn * dn));
  sx3y = (sum->x3y - 3 *sum->x * sum->x2y / dn
          + 3 *sum->x *sum->x * sum->xy / (dn * dn)
          -sum->y * sum->x3 / dn
          + 3 *sum->x *sum->y * sum->x2 / (dn * dn)
          - 3 *sum->x *sum->x *sum->x *sum->y / (dn * dn * dn));
  sxy3 = (sum->xy3 - 3 *sum->y * sum->xy2 / dn
          + 3 *sum->y *sum->y * sum->xy / (dn * dn)
          -sum->x * sum->y3 / dn
          + 3 *sum->y *sum->x *sum->y2 / (dn * dn)
          - 3 *sum->y *sum->y *sum->y *sum->x / (dn * dn * dn));
  sx2y2 = (sum->x2y2 - 2 *sum->x * sum->xy2 / dn
           - 2 *sum->y * sum->x2y / dn +sum->x *sum->x *sum->y2 / (dn * dn)
           +sum->y *sum->y * sum->x2 / (dn * dn)
           + 4 *sum->x *sum->y * sum->xy / (dn * dn)
           - 3 *sum->x *sum->x *sum->y *sum->y / (dn * dn * dn));

  sx3 = (sum->x3 - 3 *sum->x * sum->x2 / dn
         + 2 *sum->x *sum->x *sum->x / (dn * dn));
  sy3 = (sum->y3 - 3 *sum->y *sum->y2 / dn
         + 2 *sum->y *sum->y *sum->y / (dn * dn));
  sx2y = (sum->x2y - 2 *sum->x * sum->xy / dn
          -sum->y * sum->x2 / dn + 2 * sum->x * sum->x * sum->y / (dn * dn));
  sxy2 = (sum->xy2 - 2 *sum->y * sum->xy / dn
          -sum->x * sum->y2 / dn + 2 * sum->y * sum->y * sum->x / (dn * dn));

  sx2 = (sum->x2 -sum->x *sum->x / dn);
  sy2 = (sum->y2 -sum->y *sum->y / dn);
  sxy = (sum->xy -sum->x *sum->y / dn);

  sx = sy = 0;

  /*P00*/
  ellipse->P00[0][0] = sx4;
  ellipse->P00[0][1] = sx3y;
  ellipse->P00[0][2] = sx2y2;
  ellipse->P00[1][0] = sx3y;
  ellipse->P00[1][1] = sx2y2;
  ellipse->P00[1][2] = sxy3;
  ellipse->P00[2][0] = sx2y2;
  ellipse->P00[2][1] = sxy3;
  ellipse->P00[2][2] = sy4;

  /*P01*/
  ellipse->P01[0][0] = sx3;
  ellipse->P01[0][1] = sx2y;
  ellipse->P01[0][2] = sx2;
  ellipse->P01[1][0] = sx2y;
  ellipse->P01[1][1] = sxy2;
  ellipse->P01[1][2] = sxy;
  ellipse->P01[2][0] = sxy2;
  ellipse->P01[2][1] = sy3;
  ellipse->P01[2][2] = sy2;

  /*P10*/
  ellipse->P10[0][0] = sx3;
  ellipse->P10[0][1] = sx2y;
  ellipse->P10[0][2] = sxy2;
  ellipse->P10[1][0] = sx2y;
  ellipse->P10[1][1] = sxy2;
  ellipse->P10[1][2] = sy3;
  ellipse->P10[2][0] = sx2;
  ellipse->P10[2][1] = sxy;
  ellipse->P10[2][2] = sy2;

  /*P11*/
  ellipse->P11[0][0] = sx2;
  ellipse->P11[0][1] = sxy;
  ellipse->P11[0][2] = sx;
  ellipse->P11[1][0] = sxy;
  ellipse->P11[1][1] = sy2;
  ellipse->P11[1][2] = sy;
  ellipse->P11[2][0] = sx;
  ellipse->P11[2][1] = sy;
  ellipse->P11[2][2] = dn;

  if (offsetProp)
    {
      ellipse->offset[0] = sum->x / sum->n + offsetProp->d[0];
      ellipse->offset[1] = sum->y / sum->n + offsetProp->d[1];
    }
  else
    {
      ellipse->offset[0] = sum->x / sum->n;
      ellipse->offset[1] = sum->y / sum->n;
    }

  return;
}

void
P_to_avec_and_fix(Ellipse* ellipse)
{
  int iroot, idim;

  double  C1inv[NDIM_CONIC_HALF][NDIM_CONIC_HALF] = {
    {0.0, 0.0, 0.5},
    {0.0, -1.0, 0.0},
    {0.5, 0.0, 0.0}
  };
  double P11inv[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  // 逆行列の計算
  CvMat vP11 = cvMat(NDIM_CONIC_HALF, NDIM_CONIC_HALF, CV_64FC1, ellipse->P11);
  CvMat vP11inv = cvMat(NDIM_CONIC_HALF, NDIM_CONIC_HALF, CV_64FC1, P11inv);

  double ret = cvInvert(&vP11, &vP11inv);
  if (ret == 0)
    {
      //fprintf(stderr, "Error invert fail.Abort\n");
      ellipse->neval = 0;
      return;
    }

  double TMP1[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  mulM33(P11inv, ellipse->P10, TMP1);
  double TMP2[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  mulM33(ellipse->P01, TMP1, TMP2);
  double TMP3[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  subM33(ellipse->P00, TMP2, TMP3);

  double TMP4[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  mulM33(C1inv, TMP3, TMP4);
  //debug_print_3x3("TMP4 = C1inv(P00-P01*P11^(-1)*P10)", TMP4);

  // 非対称行列の固有値の計算

  double  ev[NDIM_CONIC_HALF][NDIM_CONIC_HALF];

  const int nEigen = eigen3x3(TMP4, ev, ellipse->eval, VISION_EPS);

  for (iroot = 0; iroot < nEigen; iroot++)
    {
      for (idim = 0; idim < NDIM_CONIC_HALF; idim++)
        {
          ellipse->a[iroot][idim] = ev[iroot][idim];
        }
      mulM33V3(TMP1, ellipse->a[iroot], &(ellipse->a[iroot][NDIM_CONIC_HALF]));
      mulV3S(-1.0, &(ellipse->a[iroot][NDIM_CONIC_HALF]),
             &(ellipse->a[iroot][NDIM_CONIC_HALF]));

    }

  // a0 x^2 + a1 xy + a2 y^2
  // +(a3-2a0 xoff - a1 yoff) x +(a4-a1 xoff-2a2 yoff) y
  // + (a0 xoff^2+a1 xoff yoff + a2 yoff^2 - a3 xoff -a4 yoff +a5) = 0

  // offset のシフト分をキャンセルする
  for (iroot = 0; iroot < nEigen; iroot++)
    {
      ellipse->a[iroot][5] +=
        (ellipse->a[iroot][0]*ellipse->offset[0]*ellipse->offset[0]
         +ellipse->a[iroot][1]*ellipse->offset[0]*ellipse->offset[1]
         +ellipse->a[iroot][2]*ellipse->offset[1]*ellipse->offset[1]
         -ellipse->a[iroot][3]*ellipse->offset[0]
         -ellipse->a[iroot][4]*ellipse->offset[1]);
      ellipse->a[iroot][3] -=
        (2.0*ellipse->a[iroot][0]*ellipse->offset[0]
         +ellipse->a[iroot][1]*ellipse->offset[1]);
      ellipse->a[iroot][4] -=
        (ellipse->a[iroot][1] * ellipse->offset[0]
         + 2.0 * ellipse->a[iroot][2] * ellipse->offset[1]);
    }

  ellipse->neval = nEigen;

  return;
}

// 固有値の絶対値の比を比較し、最大値が閾値より大きければ 0 を返す
#define CHECK_EIGEN_VALUE_RATIO_NG	(0)
#define CHECK_EIGEN_VALUE_RATIO_OK	(1)
static int
check_eigen_value_ratio(Ellipse* ellipse,
			double	th_ratio)
{
  double	maxd, mind, tmpd;
  int	i;

  maxd = mind = fabs(ellipse->eval[0]);

  for (i = 1; i < NDIM_CONIC_HALF; i++)
    {
      tmpd = fabs(ellipse->eval[i]);
      if (tmpd > maxd)
	{
	  maxd = tmpd;
	}
      else if (tmpd < mind)
	{
	  mind = tmpd;
	}
    }

  if (mind == 0.0)
    {
      return CHECK_EIGEN_VALUE_RATIO_NG;
    }

  if (maxd / mind > th_ratio)
    {
      return CHECK_EIGEN_VALUE_RATIO_NG;
    }

  return CHECK_EIGEN_VALUE_RATIO_OK;
}

static int
point_to_ellipse(int  start1,
                 int  goal1,
                 int  nPoint,
                 const int* point,
                 SumSet* sum,
                 Ellipse* ellipse,
                 const ParamEllipseIW* paramE, // one
                 OffsetProp* offsetProp)
{
  double  tmp_meanError, tmp_maxError;
  double  min_meanerror, min_maxerror;
  int k_min_error, k;
  int idim, iaxis;

  if (offsetProp->mode == ELLIPSE_OFFSET_STATIC)
    {
      /* ELLIPSE_OFFSET_STATIC:*/
      // set PXX matrix and ellipse->offset
      sum_to_P_static(sum, ellipse, offsetProp);
    }
  else
    {
      /* ELLIPSE_OFFSET_DYNAMIC:*/
      // set PXX matrix and ellipse->offset
      sum_to_P_dynamic(sum, ellipse, offsetProp);
    }

  // calc from P to a
  // set ellipse->neval
  // fix avec with ellips->offset
  P_to_avec_and_fix(ellipse);

  //if (ellipse->neval == 0)
  if (ellipse->neval < NDIM_CONIC_HALF)
    {
      for (idim = 0; idim < NDIM_CONIC_FULL; idim++)
        {
          ellipse->coef[idim] = 0.0; // not found
        }
      return POINT_TO_ELLIPSE_FAIL;
    }

  // 固有値の比が大きいときには排除する
  if (check_eigen_value_ratio(ellipse, 1000.0) == CHECK_EIGEN_VALUE_RATIO_NG)
    {
      for (idim = 0; idim < NDIM_CONIC_FULL; idim++)
        {
          ellipse->coef[idim] = 0.0; // not found
        }
      return POINT_TO_ELLIPSE_FAIL;
    }

  // 計算された係数の評価
  min_meanerror = -1.0;
  min_maxerror = -1.0;
  k_min_error = -1;
  for (k = 0; k < ellipse->neval; k++)
    {
      if (4.0*ellipse->a[k][0]*ellipse->a[k][2]-ellipse->a[k][1]*ellipse->a[k][1] > 0.0)
        {
          eval_ellipse(start1, goal1, nPoint, point, ellipse->a[k],
                       &tmp_meanError, &tmp_maxError);
          switch (paramE->Condition)
            {
            case ELLIPSE_CONDITION_MEAN:
              if (k_min_error == -1 || tmp_meanError < min_meanerror)
                {
                  min_meanerror = tmp_meanError;
                  min_maxerror = tmp_maxError;
                  k_min_error = k;
                }
              break;
            case ELLIPSE_CONDITION_MAX :
              if (k_min_error == -1 || tmp_maxError < min_maxerror)
                {
                  min_meanerror = tmp_meanError;
                  min_maxerror = tmp_maxError;
                  k_min_error = k;
                }
              break;
            }
        }
    }

  if (k_min_error != -1)
    {
      avec_to_ellipse(k_min_error, ellipse);
      ellipse->meanError = min_meanerror;
      ellipse->maxError = min_maxerror;
    }
  else
    {
      /* clean some of ellipse vars */
      for (k = 0; k < NDIM_CONIC_FULL; k++)
        {
          ellipse->coef[k] = 0.0;
        }
      ellipse->center[0] = ellipse->center[1] = 0.0;
      for (iaxis = 0; iaxis < NAXIS; iaxis++)
        {
          for (idim = 0; idim < NDIM2; idim++)
            {
              ellipse->axis[iaxis][idim] = 0.0;
            }
        }
      ellipse->rad[0] = ellipse->rad[1] = 0.0;

      return POINT_TO_ELLIPSE_FAIL;
    }

  return POINT_TO_ELLIPSE_SUCCESS;
}

/* moved to ellipseIW.h
#define CHECK_ELLIPSE_NG  (0)
#define CHECK_ELLIPSE_OK  (1)
*/
int
check_ellipse_cond(Ellipse* ellipse,
                   const ParamEllipseIW* paramE)
{
  if (ellipse->rad[0] < paramE->MinShortRadPrev
      || ellipse->rad[1] < paramE->MinShortRadPrev)
    {
      return CHECK_ELLIPSE_NG;
    }


  switch (paramE->Condition)
    {
    case ELLIPSE_CONDITION_MEAN:
      if (ellipse->meanError > paramE->ThMeanError)
        {
          return CHECK_ELLIPSE_NG;
        }
      break;
    case ELLIPSE_CONDITION_MAX:
      if (ellipse->maxError > paramE->ThMaxError)
        {
          return CHECK_ELLIPSE_NG;
        }
      break;
    }
  return CHECK_ELLIPSE_OK;
}

/*
  minlength のときだけ呼ばれる
  if (current)
    {
      if (start-=1)
        {
          OK
        }
    if (goal+=1)
      {
        OK
      }
    }
  NG
  current の点列を調べ、
  start -1 の場合を調べ
  さらに goal が +1 のときを調べ、どちらも
  大丈夫ならすすめる
 */
static int
check_track_start(const int* point,
                  int nPoint,
                  OffsetProp* offsetProp,
                  int start0,
                  int goal0,
                  int* dir,
                  Ellipse* ellipse,
                  SumSet* sum,
                  const ParamEllipseIW* paramE,
                  int gapcount
                  )
{
  // 現在の点列のチェック
  if (point_to_ellipse(start0, goal0, nPoint, point, sum, ellipse,
                       paramE, offsetProp) == POINT_TO_ELLIPSE_FAIL)
    {
      return CHECK_TRACK_FAIL;
    }

  if (check_ellipse_cond(ellipse, paramE) == CHECK_ELLIPSE_NG)
    {
      return CHECK_TRACK_FAIL;
    }

  /*
    隣接する山があるときのために、直前に帰ってきた方向に戻らないようにする
   */
  if (gapcount > 0)
    {
      // (start0-1,goal) の点列のsumを用意
      modify_sum(point, nPoint, offsetProp, sum, start0, goal0, -1, 0);

      if (point_to_ellipse(start0-1, goal0, nPoint, point, sum, ellipse,
                           paramE, offsetProp) == POINT_TO_ELLIPSE_SUCCESS)
        {
          if (check_ellipse_cond(ellipse, paramE) == CHECK_ELLIPSE_OK)
            {
              *dir = DIR_START_DEC;
              return CHECK_TRACK_SUCCESS;
            }
        }

      // 失敗のときには sum を (start0-1,goal)->(start0, goal+1)にする
      modify_sum(point, nPoint, offsetProp, sum, start0-1, goal0, 1, 1);
    }
  else
    {
      // (start0,goal+1) の点列のsumを用意
      modify_sum(point, nPoint, offsetProp, sum, start0, goal0, 0, 1);
    }

  if (point_to_ellipse(start0, goal0+1, nPoint, point, sum, ellipse,
                       paramE, offsetProp) == POINT_TO_ELLIPSE_SUCCESS)
    {
      if (check_ellipse_cond(ellipse, paramE) == CHECK_ELLIPSE_OK)
        {
          *dir = DIR_GOAL_INC;
          return CHECK_TRACK_SUCCESS;
        }
    }

  // 失敗のときには　sum を元に戻す
  modify_sum(point, nPoint, offsetProp, sum, start0, goal0+1, 0, -1);

  return CHECK_TRACK_FAIL;
}

/*
  前回の進行方向dirと現在のstart, goal および回転方向 turn
  から新しいnewstart, newgoal を作成し、sumを更新
  そのsumをもとに楕円フィットをして誤差を評価し、進めるべきかどうかを確認する
  また、start が負にならないかどうかもチェック
 */
static int
check_next_curve(const int* point,
                 int  nPoint,
                 OffsetProp* offsetProp,
                 int  start0,
                 int  goal0,
                 int  dir,
                 int  turn,
                 Ellipse* ellipse,
                 SumSet* sum,
                 const ParamEllipseIW* paramE)
{
  int vec_s, vec_g;
  int start1, goal1;
  SumSet	tmpsum;

  tmpsum = *sum;

  dir = (dir + turn) % NDIR4;

  vec_s = ds[dir];
  vec_g = dg[dir];

  start1 = start0 + vec_s;
  goal1 = goal0 + vec_g;

  modify_sum(point, nPoint, offsetProp, sum, start0, goal0, vec_s, vec_g);

  if (point_to_ellipse(start1, goal1, nPoint, point, sum, ellipse,
                       paramE, offsetProp) == POINT_TO_ELLIPSE_SUCCESS)
    {
      if (check_ellipse_cond(ellipse, paramE) == CHECK_ELLIPSE_OK)
        {
          return CHECK_NEXT_SUCCESS;
        }
    }

  // 失敗のときには　sum を元に戻す
  //modify_sum(point, nPoint, offsetProp, sum, start1, goal1, -vec_s, -vec_g);
  // 誤差を減らすためにコピーする
  *sum = tmpsum;

  return CHECK_NEXT_FAIL;
}

// 追跡した境界が穴だったとき、len=min の状態で
// 次の point_to_ellipse()が失敗するところまでスキップ
/*
#define SKIP_BODY_WHOLE (1)
#define SKIP_BODY_NORMAL  (0)

static int
skip_body(const int* point,
          int nPoint,
          OffsetProp* offsetProp,
          int* start,
          int* goal,
          int* len,
          int start1,
          Ellipse* ellipse,
          SumSet* sum,
          const ParamEllipseIW* paramE)
{
  int loop;

  do
    {
      loop = 0;
      modify_sum(point, nPoint, offsetProp, sum, *start, *goal, DS_SHIFT, DG_SHIFT);
      if (point_to_ellipse((*start)+1, (*goal)+1,
                           nPoint, point, sum, ellipse, paramE, offsetProp)
          == POINT_TO_ELLIPSE_SUCCESS)
        {
          if (check_ellipse_cond(ellipse, paramE) == CHECK_ELLIPSE_OK)
            {
              loop = 1;
            }
        }
      advance_next_curve(DS_SHIFT, DG_SHIFT, start, goal, len, NULL, NULL, NULL, NULL);
    }
  while(loop == 1);

  if ((start1+1) % nPoint == (*start) % nPoint)
    {
      return SKIP_BODY_WHOLE;
    }
  return SKIP_BODY_NORMAL;
}
*/

#define DS_SKIP (0)
#define DG_SKIP (1)

#define SKIP_LOOP_STOP  (0)
#define SKIP_LOOP_FULL  (1)
#define SKIP_LOOP_CONTINUE  (2)

static int
skip_body(const int* point,
          int nPoint,
          OffsetProp* offsetProp,
          int* start,
          int* goal,
          int* len,
          int* max_curve_len,
          Feature2D_old* max_f2D,
          //int start1,
          Ellipse* ellipse,
          SumSet* sum,
          const ParamEllipseIW* paramE)
{
  int loop;

  do
    {
      loop = SKIP_LOOP_STOP;

      modify_sum(point, nPoint, offsetProp, sum, *start, *goal, DS_SKIP, DG_SKIP);
      if (point_to_ellipse((*start)+DS_SKIP, (*goal)+DG_SKIP,
                           nPoint, point, sum, ellipse, paramE, offsetProp)
          == POINT_TO_ELLIPSE_SUCCESS)
        {
          if (check_ellipse_cond(ellipse, paramE) == CHECK_ELLIPSE_OK)
            {
              advance_next_curve(DS_SKIP, DG_SKIP, start, goal, len,
                                 max_curve_len, ellipse, max_f2D, paramE);
              if (*len == nPoint)
                {
                  loop = SKIP_LOOP_FULL;
                }
              else
                {
                  loop = SKIP_LOOP_CONTINUE;
                }
            }
        }
    }
  while (loop == SKIP_LOOP_CONTINUE);

  if (loop == SKIP_LOOP_STOP)
    {
      modify_sum(point, nPoint, offsetProp, sum,
                 (*start)+DS_SKIP, (*goal)+DG_SKIP, -DS_SKIP, -DG_SKIP);
    }

  return loop;
}

int
searchEllipseIW(Features2D_old* f2D,
                int iTrack,
                const ParamEllipseIW* paramE)
{
  // この点列について開始した時点での特徴点の数
  //int first_feature = f2D->nFeature;

  int nPoint = f2D->track[iTrack].nPoint;  // 輪郭線の点数
  int* point = f2D->track[iTrack].Point;   // 輪郭線の点列

  SumSet sum;             // 当てはめのための係数総和行列
  SumSet  sum0;  // start0 におけるバックアップ
  SumSet  tmpsum;
  SumSet  sum1;
  static SumSet null_sum;

  int ip;
  int start, goal, len, dir;
  int max_curve_len;
  int tracking;
  static Ellipse  null_ellipse;
  Ellipse ellipse;
  //Ellipse max_ellipse;
  int flag_start_zero;
  int loop_cond;
  static Feature2D_old  null_f2D;
  Feature2D_old max_f2D;
  //int i;
  int turn;
  int start0;
  int goal0, goal1;
  OffsetProp  offsetProp;
  int gapcount;
  int dir0;
  int len0, len1;

  turn = 0;
  start0 = 0;
  goal0 = 0;
  goal1 = 0;
  dir0 = 0;
  len0 = 0;
  len1 = 0;

  offsetProp.mode = paramE->OffsetMode;
  offsetProp.d[0] = f2D->track[iTrack].offset[0];// 楕円係数計算時のオフセット
  offsetProp.d[1] = f2D->track[iTrack].offset[1];

  // もしtrackの長さが最小点列より小ならなにもしない
  if (nPoint < paramE->MinLength)
    {
      return SEARCH_FEATURES2_OK;
    }

  // 構造体の初期化
  sum = null_sum;
  sum0 = null_sum;
  ellipse = null_ellipse;
  //max_ellipse = null_ellipse;
  max_f2D = null_f2D;

  // paramE->MinLength 個の配列を初期化
  for (ip = 0; ip < paramE->MinLength; ip++)
    {
      addArcSum(&sum, &point[ip * 2], offsetProp.d);
    }

  tracking = TRACKING_OFF;
  gapcount = 1;
  start = 0;
  goal = paramE->MinLength-1;
  len = paramE->MinLength;
  dir = DIR_START_INC; // 3 len = paramE->MinLength のときはこの値にする
  max_curve_len = MAX_CURVE_LEN_UNDEFINED; // -1
  //max_ellipse = null_ellipse;
  flag_start_zero = 0;

  loop_cond = LOOP_CONTINUE;

  // main loop
  // 進行方向左手をチェックし、進めれば進む
  // 進行方向をチェックし、進めれば進む
  // 進行方向右手をチェックし、進めれば進む
  // 逆方向に進む
  //

  do
    {
      // check left d+1
      // check forward d
      // check right d+3
      // backward d+2

      if (tracking == TRACKING_OFF) // check minimum length
        {
    // 開始条件のチェック
          if (check_track_start(point, nPoint, &offsetProp, start, goal, &dir,
                                &ellipse, &sum, paramE, gapcount)
              == CHECK_TRACK_SUCCESS)
            {
              // advance GOAL_INC
              turn = 0; // if turn < 0, it goes backward
              //dir = DIR_START_DEC;
              start0 = start;
              goal0 = goal1 = goal;
              len0 = len1 = paramE->MinLength;
              dir0 = dir;
              advance_next_curve(ds[dir], dg[dir], &start, &goal, &len,
                                 &max_curve_len,
                                 &ellipse, &max_f2D, paramE);
              tracking = TRACKING_ON;

              sum0 = sum;
              modify_sum(point, nPoint, &offsetProp, &sum0,
                         start, goal,
                         ds[(dir+2)%4], dg[(dir+2)%4]);
            }
          else
            { // current=OK next=NG
              // advance start++ goal++
              modify_sum(point, nPoint, &offsetProp, &sum, start, goal,
			 DS_SHIFT, DG_SHIFT);
              advance_next_curve(DS_SHIFT, DG_SHIFT, &start, &goal, &len,
				 NULL, NULL, NULL, NULL);
              gapcount++;

              if (start >= nPoint)
                {
                  loop_cond = LOOP_EXIT_NORMAL;
                }
            }
        }
      else
        {
          // advance one of four points
          // check left, fwd, right
          if (check_next_curve(point, nPoint, &offsetProp, start, goal, dir,
                               TURN_LEFT, &ellipse, &sum, paramE)
	      == CHECK_NEXT_SUCCESS)
            // check left
            {
              // advance left
              turn--;
              dir = (dir + TURN_LEFT) % NDIR4;
              advance_next_curve(ds[dir], dg[dir], &start, &goal, &len,
				 &max_curve_len,
                                 &ellipse, &max_f2D, paramE);
            }
          else if (check_next_curve(point, nPoint, &offsetProp, start, goal, dir,
                                    TURN_FORWARD, &ellipse, &sum, paramE)
		   == CHECK_NEXT_SUCCESS)
            // check forward
            {
              // advance forward
              dir = (dir + TURN_FORWARD) % NDIR4;
              advance_next_curve(ds[dir], dg[dir], &start, &goal, &len,
				 &max_curve_len,
                                 &ellipse, &max_f2D, paramE);
            }
          else if (check_next_curve(point, nPoint, &offsetProp, start, goal, dir,
                                    TURN_RIGHT, &ellipse, &sum, paramE)
		   == CHECK_NEXT_SUCCESS)
            // check right
            {
              // advance right
              turn++;
              dir = (dir + TURN_RIGHT) % NDIR4;
              advance_next_curve(ds[dir], dg[dir], &start, &goal, &len,
				 &max_curve_len,
                                 &ellipse, &max_f2D, paramE);
            }
          else
            {
              // advance backward
              turn+=2;
              dir = (dir + TURN_BACKWARD) % NDIR4;
              modify_sum(point, nPoint, &offsetProp, &sum, start, goal,
                         ds[dir], dg[dir]);
              advance_next_curve(ds[dir], dg[dir], &start, &goal, &len,
                                 &max_curve_len,
                                 &ellipse, &max_f2D, paramE);
            }

          // goal1, len1 のupdate
          if (mod_nPoint(start, nPoint) == mod_nPoint(start0, nPoint)
              && len > len1)
            {
              len1 = len;
              goal1 = start0 + len1 - 1;
              sum1 = sum;
            }

          // evaluate status
          if (mod_nPoint(goal, nPoint) == mod_nPoint(start + nPoint - 1, nPoint))
            // 一周点列判定
            {
              if (add_new_ellipse(f2D, start, goal, &max_f2D, paramE,
				  nPoint, iTrack, point)
                   == ADD_NEW_ELLIPSE_NG)
                {
                  return SEARCH_FEATURES2_NG;
                }
              loop_cond = LOOP_EXIT_WHOLE;
            }
          else if (len == paramE->MinLength)
            // 最短点列状態
            {
              tracking = TRACKING_OFF;
              max_curve_len = MAX_CURVE_LEN_UNDEFINED;

              if (turn < 0) // 穴
                {
                  /*
                    sum = sum0;
                    start1 = start;
                    start = start0;
                    goal = start0 + paramE->MinLength-1;
                    if(skip_body(point, nPoint, &offsetProp,
                    &start, &goal, &len, start1,
                    &ellipse, &sum, paramE) == SKIP_BODY_WHOLE)
                    {
                    if(add_new_ellipse(f2D, 0, nPoint-1, &max_f2D, paramE,
                    nPoint, iTrack, point) == -1)
                    {
                    return SEARCH_FEATURES2_NG;
                    }
                    loop_cond = LOOP_EXIT_WHOLE;
                    }
                  */
                  sum = sum0;
                  start = start0;
                  goal = goal1;
                  len = len1;
                  if (skip_body(point, nPoint, &offsetProp,
                                &start, &goal, &len,
                                &max_curve_len, &max_f2D,
                                &ellipse, &sum, paramE) == SKIP_LOOP_FULL)
                    {
                      // 全周の楕円が見つかった。それを登録して終了
                      if (add_new_ellipse(f2D, start, goal, &max_f2D, paramE,
                                          nPoint, iTrack, point)
			  == ADD_NEW_ELLIPSE_NG)
                        {
                          return SEARCH_FEATURES2_NG;
                        }
                      loop_cond = LOOP_EXIT_WHOLE;
                    }
                  else if (len == paramE->MinLength)
                    {
                      // skip した外周候補が MinLength
                      // (start,goal+1)はNGなので、次の点にシフト
                      modify_sum(point, nPoint, &offsetProp, &sum, start, goal,
                                 DS_SHIFT, DG_SHIFT);
                      advance_next_curve(DS_SHIFT, DG_SHIFT,
                                         &start, &goal, &len,
                                         NULL, NULL, NULL, NULL);
                      tracking = TRACKING_OFF;
                      gapcount=1;

                      if (start >= nPoint)
                        {
                          loop_cond = LOOP_EXIT_NORMAL;
                        }
                    }
                  else
                    {
                      // 新しいループを開始
                      tracking = TRACKING_ON;
                      start0 = start;
		      len0 = len1 = len;
                      goal0 = goal1 = start0 + len0 - 1;
                      sum0 = tmpsum = sum;
                      turn = 0;
                      // (start-1,goal)の点列をチェック
                      // もし楕円条件を満たしていれば
                      // dir = DIR_START_INC
                      // そうでなければ
                      // dir = DIR_GOAL_INC
                      dir0 = dir = DIR_GOAL_INC;
                      modify_sum(point, nPoint, &offsetProp, &tmpsum,
                                 start, goal,
                                 ds[DIR_START_DEC],
                                 dg[DIR_START_DEC]);
                      if (point_to_ellipse(start-1, goal, nPoint, point,
                                           &tmpsum, &ellipse,
                                           paramE, &offsetProp)
			  == POINT_TO_ELLIPSE_SUCCESS)
                        {
                          if (check_ellipse_cond(&ellipse, paramE)
			      == CHECK_ELLIPSE_OK)
                            {
                              dir0 = dir = DIR_START_INC;
                            }
                        }
                    }
                }
              else
                {
                  // add new feature
                  if (len0 == paramE->MinLength)
                    {
                      if (add_new_ellipse(f2D, start, goal, &max_f2D, paramE,
                                          nPoint, iTrack, point)
			  == ADD_NEW_ELLIPSE_NG)
                        {
                          return SEARCH_FEATURES2_NG;
                        }
                    }

                  if (start >= nPoint)
                    {
                      loop_cond = LOOP_EXIT_NORMAL;
                    }

                  gapcount = 0;
                  if (dir == DIR_GOAL_DEC)
                    {
                      // advance start++ goal++
                      modify_sum(point, nPoint, &offsetProp, &sum, start, goal,
                                 DS_SHIFT, DG_SHIFT);
                      advance_next_curve(DS_SHIFT, DG_SHIFT, &start, &goal, &len,
                                         NULL, NULL, NULL, NULL);
                    }
                }
            }
          else if (mod_nPoint(start, nPoint) == mod_nPoint(start0, nPoint)
                   && mod_nPoint(goal, nPoint) == mod_nPoint(goal0, nPoint)
                   && dir == dir0)
            {
              // 開始点に戻った場合
              if (turn < 0)
                {
                  // skip_body 後の探索が穴であった
                  // もう一度 skip_body()を行う
                  sum = sum1;
                  goal = goal1;
                  len = (goal - start + 1 + nPoint) % nPoint;
                  if (skip_body(point, nPoint, &offsetProp,
                                &start, &goal, &len,
                                &max_curve_len, &max_f2D,
                                &ellipse, &sum, paramE) == SKIP_LOOP_FULL)
                    {
                      // 全周の楕円が見つかった。それを登録して終了
                      if (add_new_ellipse(f2D, start, goal, &max_f2D, paramE,
                                          nPoint, iTrack, point)
			  == ADD_NEW_ELLIPSE_NG)
                        {
                          return SEARCH_FEATURES2_NG;
                        }
                      loop_cond = LOOP_EXIT_WHOLE;
                    }
                  else
                    {
                      // 新しいループを開始
                      tracking = TRACKING_ON;
                      start0 = start;
		      len0 = len1 = len;
                      goal0 = goal1 = start0 + len0 - 1;
                      sum0 = tmpsum = sum;
                      turn = 0;
                      // (start-1,goal)の点列をチェック
                      // もし楕円条件を満たしていれば
                      // dir = DIR_START_INC
                      // そうでなければ
                      // dir = DIR_GOAL_INC
                      dir0 = dir = DIR_GOAL_INC;
                      modify_sum(point, nPoint, &offsetProp, &tmpsum,
                                 start, goal,
                                 ds[DIR_START_DEC],
                                 dg[DIR_START_DEC]);
                      if (point_to_ellipse(start-1, goal, nPoint, point,
                                           &tmpsum, &ellipse,
                                           paramE, &offsetProp)
                          == POINT_TO_ELLIPSE_SUCCESS)
                        {
                          if (check_ellipse_cond(&ellipse, paramE)
			      == CHECK_ELLIPSE_OK)
                            {
                              dir0 = dir = DIR_START_INC;
                            }
                        }
                    }
                }
              else
                {
                  // minLenより大きい外周が存在する
                  // max_f2D を登録し、ループを抜ける
                  // add new feature
                  if (add_new_ellipse(f2D, start, goal, &max_f2D, paramE,
                                      nPoint, iTrack, point)
		      == ADD_NEW_ELLIPSE_NG)
                    {
                      return SEARCH_FEATURES2_NG;
                    }
                  loop_cond = LOOP_EXIT_NORMAL;
                }
            }
        }
    }
  while(loop_cond == LOOP_CONTINUE);

  return SEARCH_FEATURES2_OK;
}
