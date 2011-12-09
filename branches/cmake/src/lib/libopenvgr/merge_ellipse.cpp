/* -*- coding: utf-8 -*-
 merge_ellipse.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
// Features2D_old 構造体に格納された ConicType_Ellipse という特徴同士の
// マージ可能性を調べ、マージ可能な場合にはテストし、条件を満たせば
// マージ後の楕円をリストに追加する

// マージの判定は、ある円弧の内側に他の円弧の中心が存在するかを調べ、
// 円弧の部分集合のすべてで相互に中心内在条件を満たしていれば楕円の候補として
// 楕円をフィットし、その楕円からの距離が誤差の条件以内であれば楕円として登録する

// ある楕円 A の中心が 楕円 B の内側にあるための条件は、
// A の中心を Ac
// B の端点を Bt0, Bt1 、 B の中心を Bc とすると
// Bc と Bt0 Bt1 を結ぶ半直線より Ac が円弧 B 側にある場合には 楕円の内部にあること
// それ以外の場合には　Bt0 Bt1 における接線の内側にあること
// を条件とする

#include <stdio.h>

#include "extractFeature_old.h"
#include "ellipseIW.h"

// 戻り値
#define INITIAL_ARRAYS_OK (0)
#define INITIAL_ARRAYS_NG (1)

#define SEARCH_ANOTHER_ARC_OK (0)
#define SEARCH_ANOTHER_ARC_NG (1)

#define REF_SELF	(-1)
#define REF_OK		(0)
#define REF_NG		(1)

#define NOELLIPSE	(0)
#define ELLIPSE_TERMINAL_PART	(1)
#define ELLIPSE_TERMINAL_WHOLE	(2)

#define SIGN_UNDEF	(0)
#define SIGN_INC	(1)
#define SIGN_DEC	(2)
#define SIGN_ZERO	(3)

#define NTERMINAL	(2)

#define ANOTHER_EXIST_NG	(0)
#define ANOTHER_EXIST_OK	(1)
#define ANOTHER_EXIST_ERROR	(2)

#define ALL_REF_OK	(1)
#define ALL_REF_NG	(0)

#define TRY_ELLIPSE_TERMINATE	(0) // err is large, stop
#define TRY_ELLIPSE_CONTINUE	(1) // err is middle, continue 
#define TRY_ELLIPSE_REGISTER	(2) // err is small, register and continue

typedef struct _EllipseTerminal_ {
  int	flag;  // ELLIPSE_TERMINAL_{WHOLE|PART},NOELLIPSE
  double	center[NDIM2];
  double	ev[NAXIS][NDIM2];
  double	axis[NAXIS];
  double	coef[NDIM_CONIC_FULL];
  double	min_theta, max_theta;
  int		min_i, max_i;
  double	tangent[NTERMINAL][NDIM2];
  double	p[NTERMINAL][NDIM2];
  double	normal[NTERMINAL][NDIM2];
} EllipseTerminal;

typedef struct _MergeEllipseArrays_ {
  int	**tab;
  EllipseTerminal	*eterm; // nFeature
  int	*flist;
  SumSet	*sum;
  int	nFeature0;
} MergeEllipseArrays;

static void
debug_print_tab(int	nFeature,
		int	**tab)
{
  int	i, j;

  for(i = 0; i < nFeature; i++){
    for(j = 0; j < nFeature; j++){
      printf("%d ", tab[i][j]);
    }
    printf("\n");
  }

  return;
}

static void
debug_center_list(char	*imgname,
		  int	i_img,
		  int	i_feature,
		  MergeEllipseArrays	*me,
		  Features2D_old	*f2D)
{
  char	fname0[256];
  char	fname1[256];
  char	fname2[256];
  char	fname3[256];
  char	fname4[256];
  char	fname5[256];
  FILE	*fpa, *fpb;
  Feature2D_old	*pf;
  Track	*pt;
  int	ipoint;
  int	j_feature;
  int	count_ok, count_ng;

  sprintf(fname0, "%s_%d_edge.d", imgname, i_img);
  sprintf(fname1, "%s_%d_%d_point.d", imgname, i_img, i_feature);
  sprintf(fname2, "%s_%d_%d_centers_inside.d", imgname, i_img, i_feature);
  sprintf(fname3, "%s_%d_%d_centers_outside.d", imgname, i_img, i_feature);
  sprintf(fname4, "%s_%d_%d.plot", imgname, i_img, i_feature);
  sprintf(fname5, "%s_%d_%d_tangent.d", imgname, i_img, i_feature);

  fpa = fopen(fname1, "w");
  pf = &f2D->feature[i_feature];
  pt = &f2D->track[pf->nTrack];
  for(ipoint = pf->start; ipoint <= pf->end; ipoint++){
    fprintf(fpa, "%d %d\n",
	    pt->Point[2*((ipoint+pt->nPoint)%pt->nPoint)],
	    pt->Point[2*((ipoint+pt->nPoint)%pt->nPoint)+1]);
  }
  fclose(fpa);

  fpa = fopen(fname2, "w");
  fpb = fopen(fname3, "w");
  count_ok = count_ng = 0;
  for(j_feature = 0; j_feature < f2D->nFeature; j_feature++){
    if(me->tab[i_feature][j_feature] == REF_OK){
      fprintf(fpa, "%g %g\n",
	      me->eterm[j_feature].center[0],
	      me->eterm[j_feature].center[1]);
      count_ok++;
    }else if(me->tab[i_feature][j_feature] == REF_NG){
      fprintf(fpb, "%g %g\n",
	      me->eterm[j_feature].center[0],
	      me->eterm[j_feature].center[1]);
      count_ng++;
    }
  }
  fclose(fpa);
  fclose(fpb);

  fpa = fopen(fname5, "w");
  fprintf(fpa, "%g %g\n",
	  me->eterm[i_feature].p[0][0],
	  me->eterm[i_feature].p[0][1]);
  fprintf(fpa, "%g %g\n\n",
	  me->eterm[i_feature].p[0][0] +
	  me->eterm[i_feature].tangent[0][0] * 100.0,
	  me->eterm[i_feature].p[0][1]+
	  me->eterm[i_feature].tangent[0][1] * 100.0
	  );
  fprintf(fpa, "%g %g\n",
	  me->eterm[i_feature].p[1][0],
	  me->eterm[i_feature].p[1][1]);
  fprintf(fpa, "%g %g\n\n",
	  me->eterm[i_feature].p[1][0] -
	  me->eterm[i_feature].tangent[1][0] * 100.0,
	  me->eterm[i_feature].p[1][1] -
	  me->eterm[i_feature].tangent[1][1] * 100.0
	  );
  fclose(fpa);

  fpa = fopen(fname4, "w");
  fprintf(fpa, "set parametric\n");
  if(count_ok){
    if(count_ng){
      fprintf(fpa,
	      "plot \"%s\" w dot, \"%s\",\"%s\" t \"outside\", \"%s\" t \"inside\",\"%s\" w line notitle, ",
	      fname0, fname1, fname3, fname2, fname5);
    }else{
      fprintf(fpa,
	      "plot \"%s\" w dot, \"%s\", \"%s\" t \"inside\",\"%s\" w line notitle, ",
	      fname0, fname1, fname2, fname5);
    }
  }else{
    if(count_ng){
      fprintf(fpa,
	      "plot \"%s\" w dot, \"%s\", \"%s\" t \"outside\",\"%s\" w line notitle, ",
	      fname0, fname1, fname3, fname5);
    }else{
      fprintf(fpa,
	      "plot \"%s\" w dot, \"%s\",\"%s\" w line notitle, ",
	      fname0, fname1, fname5);
    }
  }
  fprintf(fpa,
	  "%g*cos(t)*(%g)+%g*sin(t)*(%g)+(%g),%g*cos(t)*(%g)+%g*sin(t)*(%g)+(%g) w line notitle\n",
	  me->eterm[i_feature].axis[0],
	  me->eterm[i_feature].ev[0][0],
	  me->eterm[i_feature].axis[1],
	  me->eterm[i_feature].ev[1][0],
	  me->eterm[i_feature].center[0],
	  me->eterm[i_feature].axis[0],
	  me->eterm[i_feature].ev[0][1],
	  me->eterm[i_feature].axis[1],
	  me->eterm[i_feature].ev[1][1],
	  me->eterm[i_feature].center[1]);
  fclose(fpa);

  return;
}

static void
debug_track_data(char	*imgname,
		 int	i_img,
		 Features2D_old	*f2D)
{
  char	fname[256];
  FILE	*fp;
  int	iTrack, ipoint;
  sprintf(fname, "%s_%d_edge.d", imgname, i_img);
  fp = fopen(fname, "w");
  for(iTrack = 0; iTrack < f2D->nTrack; iTrack++){
    for(ipoint = 0; ipoint < f2D->track[iTrack].nPoint; ipoint++){
      fprintf(fp, "%d %d\n",
	      f2D->track[iTrack].Point[2*ipoint],
	      f2D->track[iTrack].Point[2*ipoint+1]);
    }
  }
  fclose(fp);
  return;
}

// MergeEllipseArrays の配列メモリ確保

static int
initial_arrays(Features2D_old	*f2D,
	       MergeEllipseArrays	*me,
	       Features2D_old	*f_e)
{
  int	iFeature, jFeature;

  // f_e のセット
  f_e->nFeature = 0;
  for(iFeature = 0; iFeature < f2D->nFeature; iFeature++)
    {
      if(f2D->feature[iFeature].type == ConicType_Ellipse)
	{
	  f_e->nFeature++;
	}
    }

  f_e->nAlloc = f_e->nFeature;
  f_e->nTrack = f2D->nTrack;
  f_e->track = f2D->track;
  f_e->feature = (Feature2D_old *)calloc(f_e->nFeature,
					 sizeof(Feature2D_old));

  if(f_e->feature == NULL)
    {
      return INITIAL_ARRAYS_NG;
    }

  for(iFeature = jFeature = 0; iFeature < f2D->nFeature; iFeature++)
    {
      if(f2D->feature[iFeature].type == ConicType_Ellipse)
	{
	  f_e->feature[jFeature] = f2D->feature[iFeature];
	  jFeature++;
	}
    }

  me->tab = (int **)calloc(f_e->nFeature, sizeof(int *));
  if(me->tab == NULL){
    return INITIAL_ARRAYS_NG;
  }

  me->tab[0] = (int *)calloc(f_e->nFeature*f_e->nFeature, sizeof(int));
  if(me->tab[0] == NULL){
    return INITIAL_ARRAYS_NG;
  }

  for(iFeature = 1; iFeature < f_e->nFeature; iFeature++){
    me->tab[iFeature] = me->tab[iFeature-1] + f_e->nFeature;
  }

  me->eterm = (EllipseTerminal*)calloc(f_e->nFeature,
				       sizeof(EllipseTerminal));
  if(me->eterm == NULL)
    {
      return INITIAL_ARRAYS_NG;
    }

  me->flist = (int*)calloc(f_e->nFeature, sizeof(int));
  if(me->flist == NULL)
    {
      return INITIAL_ARRAYS_NG;
    }

  me->sum = (SumSet*)calloc(f_e->nFeature, sizeof(SumSet));
  if(me->sum == NULL)
    {
      return INITIAL_ARRAYS_NG;
    }

  return INITIAL_ARRAYS_OK;
}

// MergeEllipseArrays の配列メモリ開放

static void
free_arrays(MergeEllipseArrays	*me,
	    Features2D_old	*f_e)
{
  if(f_e->feature)
    {
      free(f_e->feature);
      f_e->feature = NULL;
    }
  if(me->tab){
    if(me->tab[0]){
      free(me->tab[0]);
    }
    free(me->tab);
  }
  if(me->eterm) free(me->eterm);
  if(me->flist) free(me->flist);
  if(me->sum) free(me->sum);

  return;
}

// 楕円上の点は
// (axis[0]*cos(t)*ev[0][0]+axis[1]*sin(t)*ev[1][0]+center[0],
//  axis[0]*cos(t)*ev[0][1]+axis[1]*sin(t)*ev[1][1]+center[1])
// なので、
// cos(t) = (ev[1][1](x-center[0])-ev[1][0](y-center[1]))/det/axis[0]
// sin(t) = (-ev[0][1](x-center[0])+ev[0][0](y-center[1]))/det/axis[1]

// 点が楕円に近いと想定し、楕円における角度パラメータを返す
static double
calc_angle_d(EllipseTerminal	*ei,
	     double	*pointd)
{
  double	x,y, theta, det;

  x = pointd[0]-ei->center[0];
  y = pointd[1]-ei->center[1];

  det = ei->ev[0][0]*ei->ev[1][1] - ei->ev[0][1]*ei->ev[1][0];

  theta = atan2((-ei->ev[0][1]*x+ei->ev[0][0]*y)/det/ei->axis[1],
		(ei->ev[1][1]*x-ei->ev[1][0]*y)/det/ei->axis[0]);

  return theta;
}

// 点が楕円に近いと想定し、楕円における角度パラメータを返す
// 点列の角度計算、連続性を調べるためにmodify_theta を使う
// 点列は整数画素値
static double
calc_angle_i(EllipseTerminal	*ei,
	     int	*point,
	     double	modify_theta)
{
  double	pointd[2], theta;

  pointd[0] = (double)point[0];
  pointd[1] = (double)point[1];

  theta = calc_angle_d(ei, pointd) + modify_theta;

  return theta;
}

// 角度パラメータから楕円の接線ベクトルを計算する
static void
calc_tangent_vector(EllipseTerminal	*ei,
		    double	theta,
		    double	*vec)
{
  double	len1;

  vec[0] = (ei->axis[0]*(-sin(theta))*ei->ev[0][0]+
	    ei->axis[1]*cos(theta)*ei->ev[1][0]);
  vec[1] = (ei->axis[0]*(-sin(theta))*ei->ev[0][1]+
	    ei->axis[1]*cos(theta)*ei->ev[1][1]);

  len1 = sqrt(vec[0]*vec[0]+vec[1]*vec[1]);

  vec[0] /= len1;
  vec[1] /= len1;

  return;
}

// 角度パラメータから楕円上の点を計算する
static void
calc_terminal_point(EllipseTerminal	*ei,
		    double	theta,
		    double	*p)
{
  p[0] = (ei->axis[0]*cos(theta)*ei->ev[0][0]+
	  ei->axis[1]*sin(theta)*ei->ev[1][0]+
	  ei->center[0]);
  p[1] = (ei->axis[0]*cos(theta)*ei->ev[0][1]+
	  ei->axis[1]*sin(theta)*ei->ev[1][1]+
	  ei->center[1]);

  return;
}


// 楕円上の点は
// (axis[0]*cos(t)*ev[0][0]+axis[1]*sin(t)*ev[1][0]+center[0],
//  axis[0]*cos(t)*ev[0][1]+axis[1]*sin(t)*ev[1][1]+center[1])
//  tangent : dP/dt .. t増加方向の接線
// 外側の法線は dP/dt を時計回りに(col,row画像中では半時計回りに)９０度まわしたもの

// 楕円端点の幾何情報をセット
static void
set_maxmin_part(EllipseTerminal	*ei)
{
  int	i;

  ei->flag = ELLIPSE_TERMINAL_PART;

  // dP/dt
  // axis[0]*(-sin(t))*ev[0][0]+axis[1]*cos(t)*ev[1][0],
  // axis[0]*(-sin(t))*ev[0][1]+axis[1]+cos(t)*ev[1][1]

  calc_tangent_vector(ei, ei->max_theta, ei->tangent[0]);
  calc_terminal_point(ei, ei->max_theta, ei->p[0]);

  calc_tangent_vector(ei, ei->min_theta, ei->tangent[1]);
  calc_terminal_point(ei, ei->min_theta, ei->p[1]);

  for(i = 0; i < NTERMINAL; i++){
    ei->normal[i][0] = ei->tangent[i][1];
    ei->normal[i][1] = -ei->tangent[i][0];
  }

  return;
}

// ei->min_i および ei->max_i を使って
// ei->min_theta および　ei->max_theta を書き換え
static void
shorten_maxmin(int	*point,
	       int	nPoint,
	       int	start,
	       int	goal,
	       EllipseTerminal	*ei,
	       const ParamEllipseIW* paramE)
{
  int	d[2];
  int	count, k, i1;
  double	theta[2], tmp;

  d[0] = - paramE->ShortenEllipseMerging;
  d[1] = paramE->ShortenEllipseMerging;

  // ei->min_theta の修正
  count = 0;
  for (k = 0; k < 2; k++)
    {
      i1 = ei->min_i +d[k];
      if ( i1 >= start && i1 <= goal)
	{
	  theta[count] = calc_angle_i(ei, &point[mod_nPoint(i1, nPoint)*2],
				      0.0);
	  while (theta[count] > ei->min_theta + M_PI * 2.0)
	    {
	      theta[count] -= M_PI * 2.0;
	    }
	  while (theta[count] < ei->min_theta)
	    {
	      theta[count] += M_PI * 2.0;
	    }
	  count++;
	}
    }

  if (count)
    {
      tmp = 0.0;
      for(k = 0; k < count; k++){
	tmp += theta[k];
      }
      ei->min_theta = tmp /= (double)count;
    }

  // ei->max_theta の修正
  count = 0;
  for (k = 0; k < 2; k++)
    {
      i1 = ei->max_i +d[k];
      if ( i1 >= start && i1 <= goal)
	{
	  theta[count] = calc_angle_i(ei, &point[mod_nPoint(i1, nPoint)*2],
				      0.0);
	  while (theta[count] < ei->min_theta + M_PI * 2.0)
	    {
	      theta[count] += M_PI * 2.0;
	    }
	  while (theta[count] > ei->min_theta)
	    {
	      theta[count] -= M_PI * 2.0;
	    }
	  count++;
	}
    }

  if (count)
    {
      tmp = 0.0;
      for(k = 0; k < count; k++){
	tmp += theta[k];
      }
      ei->max_theta = tmp /= (double)count;
    }

  return;
}

// 楕円の端点接線の計算
static void
set_eterminal(Features2D_old	*f_e,
	      int	iFeature,
	      EllipseTerminal	*ei,
	      const ParamEllipseIW* paramE)
{
  int	i, j;
  Feature2D_old	*e0;
  int	*point;
  int	nPoint;
  int	start, goal;
  int	prev_sign;
  //double	prev_locmin, prev_locmax;
  double	diff;
  //int	sign;
  double	theta, prev_theta;
  double	modify_theta;

  e0 = &(f_e->feature[iFeature]);

  if(e0->type != ConicType_Ellipse){
    ei->flag = NOELLIPSE;
    return;
  }

  // copy geometric param
  for(i = 0; i < NDIM2; i++){
    ei->center[i] = e0->center[i];
  }
  for(i = 0; i < NAXIS; i++){
    for(j = 0; j < NDIM2; j++){
      ei->ev[i][j] = e0->ev[i][j];
    }
  }
  // 向きをそろえる
  if(ei->ev[0][1]*ei->ev[1][0]-ei->ev[0][0]*ei->ev[1][1] < 0.0){
    ei->ev[1][0] = -ei->ev[1][0];
    ei->ev[1][1] = -ei->ev[1][1];
  }

  for(i = 0; i < NAXIS; i++){
    ei->axis[i] = e0->axis[i];
  }
  for(i = 0; i < NDIM_CONIC_FULL; i++){
    ei->coef[i] = e0->coef[i];
  }

  // 端点をさがす
  // 単に start や end ではなく、楕円中心からみたときの角度範囲の端
  // 曲線の表裏が連続してひとつの楕円を構成している場合もある
  // そういった場合、 end == start + nPoint-1 であっても全周とは限らない

  // (x_i - x_c)(y_{i+1}-y_c) - (y_i - y_c)(x_{i+1}-x_c) の符号で
  // 角度が増加か減少かを判定するプラスなら増加

  // 極大、極小を検出し、そのときの角度を atan2 で計算して保持する。
  // 楕円上の点は
  // (axis[0]*cos(t)*ev[0][0]+axis[1]*sin(t)*ev[1][0]+center[0],
  //  axis[0]*cos(t)*ev[0][1]+axis[1]*sin(t)*ev[1][1]+center[1])
  // なので、
  // cos(t) = (ev[1][1](x-center[0])-ev[1][0](y-center[1]))/axis[0]
  // sin(t) = (ev[0][1](x-center[0])+ev[0][0](y-center[1]))/axis[1]
  //
  // そして
  // すでに計算された 最大、最小角度と比較して更新する
  // 点列のサイズが nPoint と同一のときのみ、最後の点と最初の点の間の判定を加える

  // 角度が±Piをまたがるかどうかをチェックしておき、角度の比較をする場合に修正する

  point = f_e->track[e0->nTrack].Point;
  nPoint = f_e->track[e0->nTrack].nPoint;
  start = e0->start;
  goal = e0->end;
  if(goal < start) goal += nPoint;
  prev_sign = SIGN_UNDEF;

  // 点列がtrackの全点であれば、endの点とstart の点を結ぶ線分を調べる
  if(mod_nPoint(goal+1, nPoint) == mod_nPoint(start, nPoint)){
    goal++;
  }

#if 1
  modify_theta = 0.0;
  ei->min_theta = ei->max_theta
    = prev_theta
    = calc_angle_i(ei, &point[mod_nPoint(start, nPoint)*2], modify_theta);
  ei->min_i = ei->max_i = start;

  for (i = start+1; i <= goal; i++)
    {
      // 幾何学的に増減を調べる
      diff = -((double)(point[mod_nPoint(i-1, nPoint)*2] - e0->center[0])*
	       (double)(point[mod_nPoint(i, nPoint)*2+1] - e0->center[1]) -
	       (double)(point[mod_nPoint(i-1, nPoint)*2+1] - e0->center[1])*
	       (double)(point[mod_nPoint(i, nPoint)*2] - e0->center[0]));

      theta = calc_angle_i(ei, &point[mod_nPoint(i, nPoint)*2],
			   modify_theta);

      // modify_theta の修正
      if (diff > 0.0  && theta < prev_theta)
	{
	  theta += M_PI * 2.0;
	  modify_theta += M_PI * 2.0;
	}
      else if (diff < 0.0 && theta > prev_theta)
	{
	  theta -= M_PI * 2.0;
	  modify_theta -= M_PI * 2.0;
	}

      // 最大最小値の更新
      if (theta > ei->max_theta)
	{
	  ei->max_theta = theta;
	  ei->max_i = i;
	}
      else if (theta < ei->min_theta)
	{
	  ei->min_theta = theta;
	  ei->min_i = i;
	}

      // 全円周のチェック
      if(ei->max_theta > ei->min_theta + 2.0*M_PI)
	{
	  // 楕円全周のデータあり
	  ei->flag = ELLIPSE_TERMINAL_WHOLE;
	  return;
	}

      prev_theta = theta;
    }
#else
  // 以下の部分は、全点で角度を計算しない方法。
  // 高速化は期待できるが多少複雑になるのでまだ未完成
  // 問題点は、第二象限と第三象限の境界をまたがる部分の判定が難しいこと
  // atan2() の動作がマニュアルを読んでも不明確なため
  modify_theta = 0.0;

  ei->min_theta = ei->max_theta
    = prev_locmin = prev_locmax
    = calc_angle_i(ei, &point[mod_nPoint(start, nPoint)*2], modify_theta);

  for (i = start; i <= goal-1; i++)
    {
      diff = -((double)(point[mod_nPoint(i, nPoint)*2] - e0->center[0])*
	       (double)(point[mod_nPoint(i+1, nPoint)*2+1] - e0->center[1]) -
	       (double)(point[mod_nPoint(i, nPoint)*2+1] - e0->center[1])*
	       (double)(point[mod_nPoint(i+1, nPoint)*2] - e0->center[0]));

      sign = SIGN_ZERO;
      if(diff > 0)
	{
	  sign = SIGN_INC;
	}
      else if(diff < 0)
	{
	  sign = SIGN_DEC;
	}

      switch(sign)
	{
	case SIGN_INC:
	  if(prev_sign == SIGN_DEC)
	    {
	      // 極小
	      theta = calc_angle_i(ei, &point[mod_nPoint(i, nPoint)*2],
				   modify_theta);
	      /*if(theta > prev_locmax)
		{
		theta -= 2.0*M_PI;
		}*/
	      if(theta < ei->min_theta)
		{
		  ei->min_theta = theta;
		}
	      prev_locmin = theta;
	    }
	  break;
	case SIGN_DEC:
	  if(prev_sign == SIGN_INC)
	    {
	      // 極大
	      theta = calc_angle_i(ei, &point[mod_nPoint(i, nPoint)*2],
				   modify_theta);
	      /*if(theta < prev_locmin)
		{
		theta += 2.0*M_PI;
		}*/
	      if(theta > ei->max_theta)
		{
		  ei->max_theta = theta;
		}
	      prev_locmax = theta;
	    }
	  break;
	case SIGN_ZERO:
	  break;
	}

      if(ei->max_theta > ei->min_theta + 2.0*M_PI)
	{
	  // 楕円全周のデータあり
	  ei->flag = ELLIPSE_TERMINAL_WHOLE;
	  return;
	}

      if(sign != SIGN_ZERO)
	{
	  prev_sign = sign;
	}
      check_domain(ei, point[mod_nPoint(i, nPoint)*2],
		   point[mod_nPoint(i+1, nPoint)*2], &modify_theta);
    }

  theta = calc_angle_i(ei, &point[mod_nPoint(goal, nPoint)*2],
		       modify_theta);
  if(prev_sign == SIGN_INC)
    {
      /*if(theta < prev_locmin)
	{
	theta += 2.0*M_PI;
	}*/
      if(theta > ei->max_theta)
	{
	  ei->max_theta = theta;
	}
    }
  else
    {
      /*if(theta > prev_locmax)
	{
	theta -= 2.0*M_PI;
	}*/
      if(theta < ei->min_theta)
	{
	  ei->min_theta = theta;
	}
    }

#endif

  if (paramE->ShortenEllipseMerging > 0)
    {
      shorten_maxmin(point, nPoint, start, goal, ei, paramE);
    }
  // 部分データのセット
  set_maxmin_part(ei);

  return;
}

// 楕円に切れ目がない場合に楕円の内外判定をする
static int
check_center_zone_whole(EllipseTerminal	*ei,
			Feature2D_old	*fj)
{
  double	val;

  val = (ei->coef[0]*fj->center[0]*fj->center[0]+
	 ei->coef[1]*fj->center[0]*fj->center[1]+
	 ei->coef[2]*fj->center[1]*fj->center[1]+
	 ei->coef[3]*fj->center[0]+
	 ei->coef[4]*fj->center[1]+
	 ei->coef[5]);
  if(val > 0.0)
    {
      // 楕円の外側
      return REF_NG;
    }

  return REF_OK;
}

// 楕円の内外判定をする
static int
check_center_zone(EllipseTerminal	*ei,
		  Feature2D_old	*fj)
{
  double	theta;
  int	n, i;
  double	tmpvec[2];
  double	a0, a1;

  // 全周
  if(ei->flag == ELLIPSE_TERMINAL_WHOLE)
    {
      return check_center_zone_whole(ei, fj);
    }

  // 全周以外
  theta = calc_angle_d(ei, fj->center);

  while(theta < ei->min_theta){
    theta += 2.0*M_PI;
  }

  while(theta >= ei->min_theta + 2.0 * M_PI){
    theta -= 2.0*M_PI;
  }

  if(theta < ei->max_theta)
    {
      //円弧側
      return check_center_zone_whole(ei, fj);
    }


  //開放側
  for(n = 0; n < NTERMINAL; n++){
    for(i = 0; i < NDIM2; i++){
      tmpvec[i] = fj->center[i] - ei->p[n][i];
    }
    a0 = ei->normal[n][0]*tmpvec[0]+ei->normal[n][1]*tmpvec[1];
    for(i = 0; i < NDIM2; i++){
      tmpvec[i] = ei->center[i] - ei->p[n][i];
    }
    a1 = ei->normal[n][0]*tmpvec[0]+ei->normal[n][1]*tmpvec[1];

    if(a0 * a1 < 0.0) return REF_NG;
  }

  return REF_OK;
}

// SumSet 構造体の加算
static void
sum_plus_sum(SumSet	*total,
	     SumSet	*one)
{
  total->x4 += one->x4;
  total->x3y += one->x3y;
  total->x2y2 += one->x2y2;
  total->xy3 += one->xy3;
  total->y4 += one->y4;
  total->x3 += one->x3;
  total->x2y += one->x2y;
  total->xy2 += one->xy2;
  total->y3 += one->y3;
  total->x2 += one->x2;
  total->xy += one->xy;
  total->y2 += one->y2;
  total->x += one->x;
  total->y += one->y;
  total->n += one->n;

  return;
}

// me->flist[] に登録された複数の feature について楕円との距離誤差を計算する
static void
eval_ellipse_multi(int	step,
		   MergeEllipseArrays	*me,
		   Features2D_old	*f2D,
		   double	coef[NDIM_CONIC_FULL],
		   double	*meanError,
		   double	*maxError)
{
  double	sum;
  int	np;
  int	ifeature;
  Feature2D_old	*f;
  Track	*t;
  int	st, en;
  int	nloop, i0[2], i1[2];
  int	iloop, i;
  double	e;
  int	*point;

  *maxError = *meanError = -1.0;
  sum = 0.0;
  np = 0;
  for (ifeature = 0; ifeature < step; ifeature++)
    {
      f = &f2D->feature[me->flist[ifeature]];
      t = &f2D->track[f->nTrack];
      st = mod_nPoint(f->start, t->nPoint);
      en = mod_nPoint(f->end, t->nPoint);
      point = t->Point;
      if ( st < en )
	{
	  nloop = 1;
	  i0[0] = st;
	  i1[0] = en;
	}
      else
	{
	  nloop = 2;
	  i0[0] = st;
	  i1[0] = t->nPoint-1;
	  i0[1] = 0;
	  i1[1] = en;
	}

      for (iloop = 0; iloop < nloop; iloop++)
	{
	  for(i = i0[iloop]; i <= i1[iloop]; i++){
	    e = fabs(distanceAConic((double *)coef, &point[i*2]));
	    sum += e;
	    np++;
	    if (e > *maxError)
	      {
		*maxError = e;
	      }
	  }
	}
    }

  *meanError = sum/(double)np;

  return;
}

// 複数の円弧に対して楕円をフィットし、そこからの距離によって判定する
static int
try_ellipse_merge(int	step,
		  MergeEllipseArrays	*me,
		  Features2D_old	*f2D,
		  Ellipse	*ellipse,
		  const ParamEllipseIW* paramE)
{
  SumSet	tmpsum;
  int	ifeature, idim;
  double	min_meanerror;
  double	min_maxerror;
  double	tmp_meanError;
  double	tmp_maxError;
  int	k_min_error, k;
  int	iaxis;

  memset(&tmpsum, 0, sizeof(SumSet));

  for (ifeature = 0; ifeature < step; ifeature++)
    {
      sum_plus_sum(&tmpsum, &me->sum[me->flist[ifeature]]);
    }

  sum_to_P_dynamic(&tmpsum, ellipse, NULL); // using no offset

  P_to_avec_and_fix(ellipse, paramE);

  if (ellipse->neval == 0)
    {
      for (idim = 0; idim < NDIM_CONIC_FULL; idim++)
        {
          ellipse->coef[idim] = 0.0; // not found
        }
      return TRY_ELLIPSE_TERMINATE;
    }

  // 計算された係数の評価
  min_meanerror = -1.0;
  min_maxerror = -1.0;
  k_min_error = -1;
  for (k = 0; k < ellipse->neval; k++)
    {
      if (4.0 * ellipse->a[k][0] * ellipse->a[k][2]
	  - ellipse->a[k][1] * ellipse->a[k][1] > 0.0)
        {
          eval_ellipse_multi(step, me, f2D,
			     /*start1, goal1, nPoint, point,*/
			     ellipse->a[k],
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

      return TRY_ELLIPSE_TERMINATE;
    }

  if (check_ellipse_cond(ellipse, paramE) == CHECK_ELLIPSE_NG)
    {
      switch (paramE->Condition)
	{
	case ELLIPSE_CONDITION_MEAN:
	  if(ellipse->meanError > paramE->ThMeanErrorMerging)
	    {
	      return TRY_ELLIPSE_TERMINATE;
	    }
	  break;
	case ELLIPSE_CONDITION_MAX:
	  if (ellipse->maxError > paramE->ThMaxErrorMerging) 
	    {
	      return TRY_ELLIPSE_TERMINATE;
	    }
	  break;
	}

      return TRY_ELLIPSE_CONTINUE;
    }

  return TRY_ELLIPSE_REGISTER;
}

#define ADD_NEW_MULTI_ELLIPSE_OK  (1)
#define ADD_NEW_MULTI_ELLIPSE_NG  (0)

// 複数の円弧を一つの feature として登録する
static int
add_new_multi_ellipse(Features2D_old* f2D,
		      //int start,
		      //int goal,
		      Ellipse	*ellipse,
		      Features2D_old* f_e,
		      int	*flist,
		      int	step,
		      const ParamEllipseIW* paramE
		      //int nPoint,
		      //int iTrack,
		      //const int* point
		      )
{
  int i, j;
  Feature2D_old	*newf, *tmpf;

  /* dummy for test*/
  // copy max_f2D to feature

  if (f2D->nFeature >= f2D->nAlloc)
    {                           // 記憶域を確保する
      if (expandFeatures(f2D) == NULL)
        {
          return ADD_NEW_MULTI_ELLIPSE_NG;
        }
    }

  //memcpy(&f2D->feature[f2D->nFeature], new_f, sizeof(Feature2D_old));

  newf = &f2D->feature[f2D->nFeature];
  newf->type = ConicType_Ellipse;
  for (i = 0; i < NDIM_CONIC_FULL; i++)
    {
      newf->coef[i] = ellipse->coef[i];
    }
  for (i = 0; i < NDIM2; i++)
    {
      newf->center[i] = ellipse->center[i];
    }
  for (j = 0; j < NAXIS; j++)
    {
      for (i = 0; i < NDIM2; i++)
	{
	  newf->ev[j][i] = ellipse->axis[j][i];
	}
    }
  for (i = 0; i < NDIM2; i++)
    {
      newf->axis[i] = ellipse->rad[i];
    }
  if(paramE->Condition == ELLIPSE_CONDITION_MEAN)
    {
      newf->error = ellipse->meanError;
    }
  else
    {
      newf->error = ellipse->maxError;
    }

  /* dummy */
  for (i = 0; i < NDIM2; i++)
    {
      newf->startPoint[i] = 0.0;
      newf->endPoint[i] = 0.0;
      newf->startSPoint[i] = 0.0;
      newf->middleSPoint[i] = 0.0;
      newf->endSPoint[i] = 0.0;
      newf->direction[i] = 0.0;
    }
  // sum of len info
  newf->start = 0;
  newf->all = 0;
  for(i = 0; i < step; i++){
    tmpf = &f_e->feature[flist[i]];
    newf->all += tmpf->end - tmpf->start + 1;
  }
  newf->end = newf->all-1;

  newf->nPoints = newf->nTrack = -1;
  newf->lineLength = newf->lineLength1 = newf->lineLength2 = 0.0;
  newf->lineAngle = 0.0;

  ++(f2D->nFeature);

  return ADD_NEW_MULTI_ELLIPSE_OK;
}


// 再帰的に円弧をマージする
static int
search_another_arc(int	step,
		   int	i0,
		   MergeEllipseArrays	*me,
		   Features2D_old	*f_e,
		   Features2D_old	*f2D,
		   const ParamEllipseIW *paramE
		   )
{
  //int	another_exist;
  int	i, j;
  int	cond;
  Ellipse	ellipse;
  int	next_ret;
  int	result_try;

  //another_exist = ANOTHER_EXIST_NG;
  for(i = i0; i < me->nFeature0; i++)
    {
      // test arc i with other arcs in list
      cond = ALL_REF_OK;
      for(j = 0; j < step && cond == ALL_REF_OK; j++)
	{
	  if(me->tab[i][me->flist[j]] != REF_OK ||
	     me->tab[me->flist[j]][i] != REF_OK)
	    {
	      cond = ALL_REF_NG;
	    }
	}

      if(cond == ALL_REF_OK)
	{
	  me->flist[step] = i;

	  result_try = try_ellipse_merge(step+1, me, f_e, &ellipse, paramE);
	    /*if(try_ellipse_merge(step+1, me, f2D, &ellipse, paramE)
	      == TRY_ELLIPSE_OK)*/

	  if (result_try == TRY_ELLIPSE_REGISTER)
	    {
	      if(add_new_multi_ellipse(f2D, &ellipse,
				       f_e, me->flist, step+1,
				       paramE)
		 == ADD_NEW_MULTI_ELLIPSE_NG)
		{
		  return ANOTHER_EXIST_ERROR;
		}
	    }

	  //another_exist = ANOTHER_EXIST_OK;
	  if (result_try != TRY_ELLIPSE_TERMINATE)
	    {
	      next_ret = search_another_arc(step+1, i+1, me, f_e, f2D, paramE);
	      if (next_ret == ANOTHER_EXIST_ERROR)
		{
		  return ANOTHER_EXIST_ERROR;
		}
	    }

	  me->flist[step] = -1;
	}
    }
  return ANOTHER_EXIST_OK;
}

// 各 feature のための SetSum を計算する
static void
set_sum(int	iFeature,
	MergeEllipseArrays	*me,
	Features2D_old	*f_e)
{
  Feature2D_old	*tmpf;
  Track	*tmpt;
  int	i;
  const double	offset_zero[NDIM2] = {0.0,0.0};

  tmpf = &f_e->feature[iFeature];
  tmpt = &f_e->track[tmpf->nTrack];

  if(tmpf->start < tmpf->end){
    for(i = tmpf->start; i <= tmpf->end; i++){
      addArcSum(&me->sum[iFeature],
		(const int *)&tmpt->Point[mod_nPoint(i, tmpt->nPoint)*2],
		offset_zero);
    }
  }else{
    for(i = tmpf->start; i < tmpt->nPoint; i++){
      addArcSum(&me->sum[iFeature],
		(const int *)&tmpt->Point[mod_nPoint(i, tmpt->nPoint)*2],
		offset_zero);
    }
    for(i = 0; i <= tmpf->end; i++){
      addArcSum(&me->sum[iFeature],
		(const int *)&tmpt->Point[mod_nPoint(i, tmpt->nPoint)*2],
		offset_zero);
    }
  }

  return;
}

// 楕円のマージ
int
merge_ellipse(Features2D_old	*f2D,
	      const ParamEllipseIW* paramE)
{
  Features2D_old	f_e = { 0 }; // 楕円のみのローカル構造体
  MergeEllipseArrays	me = { 0 };
  int	iFeature;
  int	jFeature;
  //int	found_pair;

  // ローカル配列構造体の malloc
  if(initial_arrays(f2D, &me, &f_e) == INITIAL_ARRAYS_NG)
    {
      free_arrays(&me, &f_e);

      return MERGE_ELLIPSE_NG;
    }

  /*
  // 参照tableの作成
  tab = create_ref_table(f2D);
  if(tab == NULL){
    return MERGE_ELLIPSE_ERR;
  }

  // 端点情報行列
  eterm = (EllipseTerminal *)calloc(f2D->nFeature, sizeof(EllipseTerminal));
  if(eterm == NULL){
    free_arrays(tab);
    return MERGE_ELLIPSE_ERR;
  }
  */

  // 各楕円の端点接線の計算（全周の楕円かどうかのフラグも立てる）
  for(iFeature = 0; iFeature < f_e.nFeature; iFeature++){
    set_eterminal(&f_e, iFeature, &me.eterm[iFeature], paramE);
  }

  // 参照table の作成 楕円の弧の範囲に他の楕円の中心があるか
  for(iFeature = 0; iFeature < f_e.nFeature; iFeature++){
    for(jFeature = 0; jFeature < f_e.nFeature; jFeature++){
      if(jFeature == iFeature){
	me.tab[iFeature][jFeature] = REF_SELF;
      }else{
	me.tab[iFeature][jFeature]
	  = check_center_zone(&me.eterm[iFeature],
			      &f_e.feature[jFeature]);
      }
    }
  }

  //debug_print_tab(f2D->nFeature, me.tab);

  // Sum の計算
  for(iFeature = 0; iFeature < f_e.nFeature; iFeature++){
    me.flist[iFeature] = 0;
  }
  for(iFeature = 0; iFeature < f_e.nFeature; iFeature++)
    {
      for(jFeature = iFeature+1; jFeature < f_e.nFeature; jFeature++)
	{
	  if(me.tab[iFeature][jFeature] == REF_OK &&
	     me.tab[jFeature][iFeature] == REF_OK)
	    {
	      if(me.flist[jFeature] == 0)
		{
		  set_sum(iFeature, &me, &f_e);
		  me.flist[iFeature] = 1;
		}
	      if(me.flist[jFeature] == 0)
		{
		  set_sum(jFeature, &me, &f_e);
		  me.flist[jFeature] = 1;
		}
	    }
	}
    }

  // 組み合わせ
  me.nFeature0 = f_e.nFeature;
  for(iFeature = 0; iFeature < me.nFeature0; iFeature++){
    me.flist[iFeature] = -1;
  }
  for(iFeature = 0; iFeature < me.nFeature0; iFeature++){
    me.flist[0] = iFeature;
    if(search_another_arc(1, iFeature+1, &me, &f_e, f2D, paramE) ==
       ANOTHER_EXIST_ERROR){
      free_arrays(&me, &f_e);
      return MERGE_ELLIPSE_NG;
    };
  }

  free_arrays(&me, &f_e);

  return MERGE_ELLIPSE_OK;
}
