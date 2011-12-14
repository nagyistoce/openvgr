/* -*- coding: utf-8 -*-
 ellipseIW.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/

#ifndef _ELLIPSE_IW_H_
#define _ELLIPSE_IW_H_

#include "extractFeature_old.h"
#include "paramEllipseIW.h"

// 三次元配列のサイズ
#define NDIM3 (3)
// 二次元配列のサイズ
#define NDIM2 (2)
// 楕円の軸数
#define NAXIS (2)

// 三次方程式の係数の数
#define NCOEFCUBIC  (4)

// 楕円二次形式係数の数
#define NDIM_CONIC_HALF (3)
#define NDIM_CONIC_FULL (6)

typedef struct _Ellipse_ {
  // geometrical features
  double  center[NDIM2];
  double  rad[NDIM2];
  double  axis[NAXIS][NDIM2];
  // Result
  double  coef[NDIM_CONIC_FULL];
  // work
  // PXX matrix
  double  P00[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  double  P01[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  double  P10[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  double  P11[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  // quadric coefs (candidates)
  double  a[NDIM_CONIC_HALF][NDIM_CONIC_FULL];
  // M matrix
  double  M[NDIM_CONIC_HALF][NDIM_CONIC_HALF];
  // cubic coefs for eigen values
  double  cubic[NCOEFCUBIC];
  // number of eigen values
  int neval;
  // eigen values
  double  eval[NDIM_CONIC_HALF];
  // offset used
  double  offset[NDIM2]; // 出力
  // errors
  double  meanError;
  double  maxError;
} Ellipse;

// 点列の積和
typedef struct _SumSet_ {
  double  x4, x3y, x2y2, xy3, y4;
  double  x3, x2y, xy2, y3;
  double  x2, xy, y2;
  double  x, y;
  double  n;
} SumSet;

typedef struct _offset_prop_ {
  int mode; // -> see paramEllipse.h
  double  d[2]; // -> f2D->track[iTrack].offset or 0
} OffsetProp;

//// eigen3x3() 対象行列に限定しない固有値、固有ベクトル計算関数
//// m: 3x3 行列（入力)
//// evec 固有ベクトル（出力）
//// eval 固有値（出力）
//// thresh A-lambda*I行列のランク判定閾値
////       二つのベクトルのsinの絶対値の最大値がこの値より小さければエラー
//int eigen3x3(const double m[NDIM3][NDIM3],
//     double evec[NDIM3][NDIM3],
//     double eval[NDIM3],
//     double thresh_sa);

void	addArcSum(SumSet* sum,
		  const int* pointX,
		  const double* offsetD);

#define SEARCH_FEATURES2_OK (1)
#define SEARCH_FEATURES2_NG (0)
int searchEllipseIW(Features2D_old* f2D,
                    int iTrack,
                    const ParamEllipseIW* paramE);

void	sum_to_P_dynamic(const SumSet* sum,
			 Ellipse* ellipse);

void	P_to_avec_and_fix(Ellipse* ellipse);

void	avec_to_ellipse(int k_min_error,
			Ellipse* ellipse 
			);

#define CHECK_ELLIPSE_NG  (0)
#define CHECK_ELLIPSE_OK  (1)
int	check_ellipse_cond(Ellipse* ellipse,
			   const ParamEllipseIW* paramE);

int	mod_nPoint(int  n,
		   int  nPoint);

double	distanceAConic(const double coef[6],
		       const int* point);

#define MERGE_ELLIPSE_OK (0)
#define MERGE_ELLIPSE_NG (1)

int merge_ellipse(Features2D_old	*f2D,
		  const ParamEllipseIW* paramE);

#endif
