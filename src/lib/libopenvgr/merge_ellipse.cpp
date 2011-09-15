// Features2D_old 構造体に格納された ConicType_Ellipse という特徴同士の
// マージ可能性を調べ、マージ可能な場合にはテストし、条件を満たせば
// マージ後の楕円をリストに追加する
#include <stdio.h>

#include "extractFeature_old.h"
#include "ellipseIW.h"

// 戻り値
#define MERGE_ELLIPSE_OK (0)
#define MERGE_ELLIPSE_NG (1)

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

#define ALL_REF_OK	(1)
#define ALL_REF_NG	(0)

#define TRY_ELLIPSE_OK	(1)
#define TRY_ELLIPSE_NG	(0)

typedef struct _EllipseTerminal_ {
  int	flag;  // ELLIPSE_TERMINAL_{WHOLE|PART},NOELLIPSE
  double	center[NDIM2];
  double	ev[NAXIS][NDIM2];
  double	axis[NAXIS];
  double	coef[NDIM_CONIC_FULL];
  double	min_theta, max_theta;
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
  for(j_feature = 0; j_feature < f2D->nFeature; j_feature++){
    if(me->tab[i_feature][j_feature] == REF_OK){
      fprintf(fpa, "%g %g\n",
	      me->eterm[j_feature].center[0],
	      me->eterm[j_feature].center[1]);
    }else if(me->tab[i_feature][j_feature] == REF_NG){
      fprintf(fpb, "%g %g\n",
	      me->eterm[j_feature].center[0],
	      me->eterm[j_feature].center[1]);
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
  fprintf(fpa,
	  "plot \"%s\" w dot, \"%s\", \"%s\" t \"inside\",\"%s\" t \"outside\",\"%s\" w line notitle, ",
	  fname0, fname1, fname2, fname3, fname5);
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


static int
mod_nPoint(int  n,
           int  nPoint)
{
  while (n < 0)
    {
      n += nPoint;
    }

  return n % nPoint;
}

static int
initial_arrays(Features2D_old	*f2D,
	       MergeEllipseArrays	*me)
{
  int	iFeature;

  me->tab = (int **)calloc(f2D->nFeature, sizeof(int *));
  if(me->tab == NULL){
    return INITIAL_ARRAYS_NG;
  }

  me->tab[0] = (int *)calloc(f2D->nFeature*f2D->nFeature, sizeof(int));
  if(me->tab[0] == NULL){
    return INITIAL_ARRAYS_NG;
  }

  for(iFeature = 1; iFeature < f2D->nFeature; iFeature++){
    me->tab[iFeature] = me->tab[iFeature-1] + f2D->nFeature;
  }

  me->eterm = (EllipseTerminal*)calloc(f2D->nFeature,
				       sizeof(EllipseTerminal));
  if(me->eterm == NULL)
    {
      return INITIAL_ARRAYS_NG;
    }

  me->flist = (int*)calloc(f2D->nFeature, sizeof(int));
  if(me->flist == NULL)
    {
      return INITIAL_ARRAYS_NG;
    }

  me->sum = (SumSet*)calloc(f2D->nFeature, sizeof(SumSet));
  if(me->sum == NULL)
    {
      return INITIAL_ARRAYS_NG;
    }

  return INITIAL_ARRAYS_OK;
}

static void
free_arrays(MergeEllipseArrays	*me)
{
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
// cos(t) = (ev[1][1](x-center[0])-ev[1][0](y-center[1]))/axis[0]
// sin(t) = (ev[0][1](x-center[0])+ev[0][0](y-center[1]))/axis[1]

static double
calc_angle_i(EllipseTerminal	*ei,
	    int	*point)
{
  double	x,y, theta;

  x = (double)point[0]-ei->center[0];
  y = (double)point[1]-ei->center[1];

  theta = atan2((ei->ev[0][1]*x+ei->ev[0][0]*y)/ei->axis[1],
		(ei->ev[1][1]*x-ei->ev[1][0]*y)/ei->axis[0]);

  return theta;
}

static double
calc_angle_d(EllipseTerminal	*ei,
	    double	*pointd)
{
  double	x,y, theta;

  x = pointd[0]-ei->center[0];
  y = pointd[1]-ei->center[1];

  theta = atan2((ei->ev[0][1]*x+ei->ev[0][0]*y)/ei->axis[1],
		(ei->ev[1][1]*x-ei->ev[1][0]*y)/ei->axis[0]);

  return theta;
}

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

static void
set_eterminal(Features2D_old	*f2D,
	      int	iFeature,
	      EllipseTerminal	*ei)
{
  int	i, j;
  Feature2D_old	*e0;
  int	*point;
  int	nPoint;
  int	start, goal;
  int	prev_sign;
  double	prev_locmin, prev_locmax;
  int	diff;
  int	sign;
  double	theta;

  e0 = &(f2D->feature[iFeature]);

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

  point = f2D->track[e0->nTrack].Point;
  nPoint = f2D->track[e0->nTrack].nPoint;
  start = e0->start;
  goal = e0->end;
  if(goal < start) goal += nPoint;
  prev_sign = SIGN_UNDEF;

  // 点列がtrackの全点であれば、endの点とstart の点を結ぶ線分を調べる
  if(mod_nPoint(goal+1, nPoint) == mod_nPoint(start, nPoint)){
    goal++;
  }

  ei->min_theta = ei->max_theta
    = prev_locmin = prev_locmax 
    = calc_angle_i(ei, &point[mod_nPoint(start, nPoint)*2]);

  for (i = start; i <= goal-1; i++)
    {
      diff = ((point[mod_nPoint(i, nPoint)*2] - e0->center[0])*
	      (point[mod_nPoint(i+1, nPoint)*2+1] - e0->center[1]) -
	      (point[mod_nPoint(i, nPoint)*2+1] - e0->center[1])*
	      (point[mod_nPoint(i+1, nPoint)*2] - e0->center[0]));

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
	      theta = calc_angle_i(ei, &point[mod_nPoint(i, nPoint)*2]);
	      if(theta > prev_locmax)
		{
		  theta -= 2.0*M_PI;
		}
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
	      theta = calc_angle_i(ei, &point[mod_nPoint(i, nPoint)*2]);
	      if(theta < prev_locmin)
		{
		  theta += 2.0*M_PI;
		}
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
    }

  theta = calc_angle_i(ei, &point[mod_nPoint(goal, nPoint)*2]);
  if(prev_sign == SIGN_INC)
    {
      if(theta < prev_locmin)
	{
	  theta += 2.0*M_PI;
	}
      if(theta > ei->max_theta)
	{
	  ei->max_theta = theta;
	}
    }
  else
    {
      if(theta > prev_locmax)
	{
	  theta -= 2.0*M_PI;
	}
      if(theta < ei->min_theta)
	{
	  ei->min_theta = theta;
	}
    }

  // 部分データのセット
  set_maxmin_part(ei);

  return;
}

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

// 再帰的に円弧をマージする
static int
search_another_arc(int	step,
		   int	i0,
		   MergeEllipseArrays	*me,
		   Features2D_old	*f2D)
{
  int	another_exist;
  int	i, j;
  int	cond;

  another_exist = ANOTHER_EXIST_NG;
  for(i = i0; i < me->nFeature0; i++)
    {
      // test arc i with other arcs in list
      cond = ALL_REF_OK;
      for(j = 0; j < me->nFeature0 && cond == ALL_REF_OK; j++)
	{
	  if(me->flist[j])
	    {
	      if(me->tab[i][j] != REF_OK || me->tab[j][i] != REF_OK)
		{
		  cond = ALL_REF_NG;
		}
	    }
	}

      /*
	確認待ち
      if(cond == ALL_REF_OK)
	{
	  if(try_ellipse_merge() == TRY_ELLIPSE_OK)
	    {
	      another_exist = ANOTHER_EXIST_OK;
	      me->flist[i] = 1;
	      if(search_another_arc(step+1, i+1, me, f2D) == ANOTHER_EXIST_NG)
		{
		  add_this_set();
		}
	      me->flist[i] = 0;
	    }
	}
      */
    }
  return another_exist;
}

int
merge_ellipse(Features2D_old	*f2D)
{
  MergeEllipseArrays	me = { 0 };
  int	iFeature;
  int	jFeature;
  int	found_pair;

  // ローカル配列構造体の malloc
  if(initial_arrays(f2D, &me) == INITIAL_ARRAYS_NG)
    {
      free_arrays(&me);

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
  for(iFeature = 0; iFeature < f2D->nFeature; iFeature++){
    set_eterminal(f2D, iFeature, &me.eterm[iFeature]);
  }

  // 参照table の作成 楕円の弧の範囲に他の楕円の中心があるか
  for(iFeature = 0; iFeature < f2D->nFeature; iFeature++){
    for(jFeature = 0; jFeature < f2D->nFeature; jFeature++){
      if(jFeature == iFeature){
	me.tab[iFeature][jFeature] = REF_SELF;
      }else{
	me.tab[iFeature][jFeature]
	  = check_center_zone(&me.eterm[iFeature],
			      &f2D->feature[jFeature]);
      }
    }
  }

  // Sum の計算
  /*
    table の確認待ち
  for(iFeature = 0; iFeature < f2D->nFeature; iFeature++){
    me.flist[iFeature] = 0;
  }
  for(iFeature = 0; iFeature < f2D->nFeature; iFeature++)
    {
      if(me.flist[iFeature] == 0)
	{
	  found_pair = 0;
	  for(jFeature = iFeature+1;
	      jFeature < f2D->nFeature && found_pair == 0; jFeature++)
	    {
	      if(me.tab[iFeature][jFeature] == REF_OK &&
		 me.tab[jFeature][iFeature] == REF_OK)
		{
		  set_sum(iFeature, &me, f2D);
		  me.flist[iFeature] = 1;
		  if(me.flist[jFeature] == 0)
		    {
		      set_sum(jFeature, &me, f2D);
		      me.flist[jFeature] = 1;
		    }
		  found_pair = 1;
		}
	    }
	}
    }
  */

  // 組み合わせリストの作成
  me.nFeature0 = f2D->nFeature;
  for(iFeature = 0; iFeature < me.nFeature0; iFeature++){
    me.flist[iFeature] = 1;
    search_another_arc(1, iFeature+1, &me, f2D);
    me.flist[iFeature] = 0;
  }

  free_arrays(&me);

  return MERGE_ELLIPSE_OK;
}
