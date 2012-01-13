/*
 extractFeature_old.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file extractFeature.cpp
 * @brief 2次元特徴抽出関連関数
 * @date \$Date::                            $
 */
#include <algorithm>

#include "extractEdge.h"
#include "extractFeature_old.h"
#include "vectorutil.h"
#include "debugutil.h"

#include "ellipseIW.h"

// Feature_List 楕円特徴を点数でソートするための構造体
typedef struct _feature_list_
{
  Feature2D_old	*pf2D;
  int	np;
} Feature_List;


// ２次元特徴データのメモリ確保と初期化
static Features2D_old*
constructFeatures()
{
  Features2D_old* features = (Features2D_old*) malloc(sizeof(Features2D_old));
  if (features == NULL)
    {
      return NULL;              // malloc failed
    }

  features->nAlloc = 0;
  features->nFeature = 0;
  features->nTrack = 0;
  features->feature = NULL;
  features->track = NULL;

  return features;
}

// ２次元特徴データのメモリ解放
void
destructFeatures(Features2D_old* features)
{
  int i;

  if (features != NULL)
    {
      if (features->feature != NULL)
        {
	  for (i = 0; i < features->nFeature; i++)
	    {
	      if(features->feature[i].arclist.arc)
		{
		  free(features->feature[i].arclist.arc);
		  features->feature[i].arclist.arc = NULL;
		}
	    }
          free(features->feature);
        }
      if (features->track != NULL)
        {
          for (i = 0; i < features->nTrack; i++)
            {
              if (features->track[i].Point != NULL && features->track[i].nPoint > 0)
                {
                  free(features->track[i].Point);
                }
            }
          free(features->track);
        }
      free(features);
    }
  return;
}

static void
clearFeatures2Dpointer(Features2D_old** features)
{
  if (features)
    {
      destructFeatures(*features);
      *features = NULL;
    }
}

// ２次元特徴データのメモリ拡張
Features2D_old*
expandFeatures(Features2D_old* features)
{
  //const int allocStep = 1024 * 5;       // 1024;

  features->nAlloc += ALLOC_STEP;

  if (features->feature == NULL)
    {                           // 新たに確保する
      if ((features->feature = (Feature2D_old*) malloc(sizeof(Feature2D_old) * (features->nAlloc))) == NULL)
        {
          return NULL;          // malloc failed
        }
      else
        {
          features->feature->nPoints = 0;
          return features;
        }
    }
  else
    {
      Feature2D_old* save = features->feature;      // 失敗時のために覚えておく
      features->feature = (Feature2D_old*) realloc(save, sizeof(Feature2D_old) * (features->nAlloc));
      if (features->feature == NULL)
        {
          free(save);
          return NULL;          // malloc failed
        }
      else
        {
          return features;
        }
    }
}

// ２次元特徴の追加
static int
addFeature(Features2D_old* features, Feature2D_old* feature)
{
  if (features == NULL)
    {
      return -1;
    }

  if (feature == NULL)
    {
      return -1;
    }

  if (features->nFeature >= features->nAlloc)
    {                           // 記憶域を確保する
      if (expandFeatures(features) == NULL)
        {
          return -1;
        }
    }

  memcpy(&features->feature[features->nFeature], feature, sizeof(Feature2D_old));

  if (feature->arclist.n > 0)
    {
      int	iarc;

      features->feature[features->nFeature].arclist.arc
	= (EllipseArc *)calloc(feature->arclist.n, sizeof(EllipseArc));
      if (features->feature[features->nFeature].arclist.arc == NULL)
	{
	  return -1;
	}
      for (iarc = 0; iarc < feature->arclist.n; iarc++)
	{
	  features->feature[features->nFeature].arclist.arc[iarc]
	    = feature->arclist.arc[iarc];
	}
    }

  ++(features->nFeature);
  return 1;
}


// arclist の構築
// メモリ確保に失敗したときに -1 を返す　正常は０
static int
setup_ellipse_arclist(Features2D_old	*f2Ds)
{
  int	iFeature;
  Feature2D_old *f2D;

  f2D = f2Ds->feature;
  for(iFeature = 0; iFeature < f2Ds->nFeature; iFeature++, f2D++)
    {
      f2D->arclist.n = 0;
      f2D->arclist.arc = NULL;
    }
      
  f2D = f2Ds->feature;
  for(iFeature = 0; iFeature < f2Ds->nFeature; iFeature++, f2D++)
    {
      if(f2D->type == ConicType_Ellipse)
	{
	  f2D->arclist.n = 1;
	  f2D->arclist.arc = (EllipseArc *)calloc(1, sizeof(EllipseArc));
	  if(f2D->arclist.arc == NULL)
	    {
	      return -1;
	    }
	  f2D->arclist.arc[0].f2Ds = f2Ds;
	  f2D->arclist.arc[0].ntrack = f2D->nTrack;
	  f2D->arclist.arc[0].start = f2D->start;
	  f2D->arclist.arc[0].goal = f2D->end;
	}
    }

  return 0;
}

// 双曲線の中心とデータ点の最短距離
static double
minDistanceCenterOfHyperbola(Features2D_old* features, Feature2D_old* feature)
{
  double minDistance = DBL_MAX;
  double distance;
  double dx, dy;
  int i;
  int *pt;
  int istart, iend, ii;

  pt = features->track[feature->nTrack].Point;
  istart = feature->start;
  iend = feature->end;
  if (istart > iend)
    {
      iend += feature->all;
    }
  for (i = istart; i <= iend; i++)
    {
      ii = (i % feature->all) * 2;
      dx = feature->center[0] - pt[ii];
      dy = feature->center[1] - pt[ii + 1];
      distance = sqrt(dx * dx + dy * dy);
      if (distance < minDistance)
        {
          minDistance = distance;
        }
    }
  return minDistance;
}

// 楕円周囲長計算
static double
lengthEllipse(Feature2D_old* elp)
{
  double a = elp->axis[0], b = elp->axis[1];
  double c = 3.0 * pow((a - b) / (a + b), 2);

  return M_PI * (a + b) * (1.0 + c / (10.0 + sqrt(4.0 - c)));
}

// オーバーラップ長計算
static int
calcOverlapLength(const int s1, const int e1, const int s2, const int e2)
{
  int e1e2, e1s2, s1e2, s1s2;
  int flag, len;

  e1e2 = (e1 > e2) ? 1 : 0;
  e1s2 = (e1 > s2) ? 1 : 0;
  s1e2 = (s1 > e2) ? 1 : 0;
  s1s2 = (s1 > s2) ? 1 : 0;
  flag = (e1e2 << 3) + (e1s2 << 2) + (s1e2 << 1) + s1s2;
  switch (flag)
    {
    case 15:                   // 1111
      len = 0;
      break;
    case 13:                   // 1101
      len = e2 - s1;
      break;
    case 12:                   // 1100
      len = e2 - s2;
      break;
    case 5:                    // 101
      len = e1 - s1;
      break;
    case 4:                    // 100
      len = e1 - s2;
      break;
    case 0:
      len = 0;
      break;
    default:
      len = -1;                 // エラー処理
      break;
    }
  return len;
}

// オーバーラップ評価：直前の同じ特徴とのみ誤差比較
static int
overlapFeatures(int* overlap, Features2D_old* features,
                const int sFeature, Feature2D_old* feature, Parameters parameters)
{
  double theta;
  double vec1[2], vec2[2];
  double len = 0.0;
  Feature2D_old* feature2;
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;

  ParametersFeature2D paramF2D = parameters.feature2D;
  double thr_radian = paramF2D.min_radian_hyperbola;    // 双曲線のなす角度閾値（0,180度に近いものを除去する）
  double thr_min_length_hyperbola_vector = paramF2D.min_length_hyperbola_vector;        // 双曲線での中心から端点までの距離の閾値
  double thr_min_length_ellipse_axis = paramF2D.min_length_ellipse_axis;        // 楕円の軸長の閾値
  double thr_min_filling_ellipse = paramF2D.min_filling_ellipse;        // 楕円の充填率の閾値
  double thr_max_flatness_ellipse = paramF2D.max_flatness_ellipse;      // 楕円の偏平率（長軸/短軸)

  // 双曲線の中心がデータから離れている時は、エラー
  // ※注意：角が丸い時に誤差が大きくなり検出できなくなる
  if (feature->type == ConicType_Hyperbola)
    {
      // 中心が画像外の時は、エラー
      if (feature->center[0] < 0.0 || feature->center[0] > (double) (colsize - 1)
          || feature->center[1] < 0.0 || feature->center[1] > (double) (rowsize - 1))
        {
          return 0;
        }
      // 中心がデータから離れている時は、エラー
      if (minDistanceCenterOfHyperbola(features, feature) > paramF2D.min_length_hyperbola_data)
        {
          return 0;
        }
    }

  // 楕円の場合
  if (feature->type == ConicType_Ellipse)
    {
      // 中心が画像外の時は、エラー
      if (feature->center[0] < 0.0 || feature->center[0] > (double) (colsize - 1)
          || feature->center[1] < 0.0 || feature->center[1] > (double) (rowsize - 1))
        {
          return 0;
        }
      // 当てはめ充填率が閾値以下ならエラー
      len = lengthEllipse(feature);
      if (feature->nPoints / len < thr_min_filling_ellipse)
        {
          return 0;
        }
      // 短軸の長さが閾値以下ならエラー
      if (feature->axis[0] > feature->axis[1])
        {
          if (feature->axis[1] < thr_min_length_ellipse_axis)
            {
              return 0;
            }
        }
      else
        {
          if (feature->axis[0] < thr_min_length_ellipse_axis)
            {
              return 0;
            }
        }
      // 偏平率が閾値以上ならエラー
      if (feature->axis[0] > feature->axis[1])
        {
          if (feature->axis[0] / feature->axis[1] > thr_max_flatness_ellipse)
            {
              return 0;
            }
        }
      else
        {
          if (feature->axis[1] / feature->axis[0] > thr_max_flatness_ellipse)
            {
              return 0;
            }
        }
    }

  // 双曲線の場合に、中心から始点、終点へのベクトルの角度が１８０度に近い時は、エラー
  // 中心から端点までの距離が閾値以下の場合はエラー
  if (feature->type == ConicType_Hyperbola)
    {
      if (feature->lineLength1 < thr_min_length_hyperbola_vector)
        {
          return 0;
        }
      if (feature->lineLength2 < thr_min_length_hyperbola_vector)
        {
          return 0;
        }
      if (fabs(feature->lineAngle) < thr_radian
          || fabs(M_PI - feature->lineAngle) < thr_radian)
        {
          return 0;
        }
    }

  // 初めての場合はここでＯＫとする
  int nFeature = features->nFeature - 1;
  if (nFeature < 0)
    {
      *overlap = -1;
      return 1;
    }
  if (sFeature == nFeature + 1)
    {
      *overlap = -1;
      return 1;
    }

  // 一つ前と比較
  feature2 = &(features->feature[nFeature]);

  // フィッティング結果が違う、もしくは輪郭線が違う場合は、オーバーラップなしと判断
  if (feature2->type != feature->type || feature2->nTrack != feature->nTrack)
    {
      *overlap = -1;
      return 1;
    }

  // 前の特徴と今回の特徴が双曲線の場合に、
  // 前回と今回の中心から始点のベクトルのなす角度、および中心から終点のベクトルの成す角度が閾値以上なら
  // オーバーラップなし（別特徴）と判断する
  if ((feature2->type == ConicType_Hyperbola) && (feature->type == ConicType_Hyperbola))
    {
      vec1[0] = feature->startSPoint[0] - feature->center[0];
      vec1[1] = feature->startSPoint[1] - feature->center[1];
      vec2[0] = feature2->startSPoint[0] - feature2->center[0];
      vec2[1] = feature2->startSPoint[1] - feature2->center[1];
      theta = getAngle2D(vec1, vec2);
      if (theta > thr_radian)
        {
          *overlap = -1;
          return 1;
        }
      vec1[0] = feature->endSPoint[0] - feature->center[0];
      vec1[1] = feature->endSPoint[1] - feature->center[1];
      vec2[0] = feature2->endSPoint[0] - feature2->center[0];
      vec2[1] = feature2->endSPoint[1] - feature2->center[1];
      theta = getAngle2D(vec1, vec2);
      if (theta > thr_radian)
        {
          *overlap = -1;
          return 1;
        }
    }

  // それ以外の場合は、オーバーラップ比率が閾値以上あれば、誤差が小さい方を選択
  int s1 = feature->start;
  int e1 = feature->end;
  int s2 = feature2->start;
  int e2 = feature2->end;
  int all = feature->all;

  if (e1 < s1)
    {
      e1 += all;
    }
  if (e2 < s2)
    {
      e2 += all;
    }
  if (e2 < s1)
    {
      if (e2 + all > s1 && s2 + all < e1)
        {
          s2 += all;
          e2 += all;
        }
    }
  if (e1 < s2)
    {
      if (e1 + all > s2 && s1 + all < e2)
        {
          s1 += all;
          e1 += all;
        }
    }

  len = calcOverlapLength(s1, e1, s2, e2);
  if (len == 0)
    {
      *overlap = -1;
      return 1;
    }
  if (len < 0)
    {
      return 0;                 // エラー処理
    }
  // 直線の場合は長さ優先
  if (feature2->type == ConicType_Line && feature->type == ConicType_Line)
    {
      if (len / (double) feature2->nPoints > paramF2D.overlapRatioLine)
        {
          if (feature2->lineLengthSQ > feature->lineLengthSQ)
            {                   // 長さ優先
              return 0;
            }
          else
            {
              *overlap = nFeature;
              return 1;
            }
        }
      else
        {
          *overlap = -1;
          return 1;
        }
    }
  // 双曲線の場合は焦点精度優先
  if ((feature2->type == ConicType_Hyperbola) && (feature->type == ConicType_Hyperbola))
    {
      if (len / (double) feature->nPoints > paramF2D.overlapRatioLine)
        {
          if (minDistanceCenterOfHyperbola(features, feature2)
              < minDistanceCenterOfHyperbola(features, feature))
            {                   // 焦点精度優先
              return 0;
            }
          else
            {
              *overlap = nFeature;
              return 1;
            }
        }
      else
        {
          *overlap = -1;
          return 1;
        }
    }
  // それ以外＝楕円の場合は精度優先
  if ((feature2->type == ConicType_Ellipse) &&
      (feature->type == ConicType_Ellipse))
    {
      if (len / (double) feature->nPoints > paramF2D.overlapRatioCircle)
        {
          if (feature2->error < feature->error)
            {                   // 精度優先
              return 0;
            }
          else
            {
              *overlap = nFeature;
              return 1;
            }
        }
      else
        {
          *overlap = -1;
          return 1;
        }
    }
  return 0;
}


// 特徴点を一時的に記録する
static void
memorizeFeature(Feature2D_old* feature, double error,
                ConicType type, const int start, const int end,
                double coef[6], int *point, const int nPoint,
                const int nTrack)
{
  int middle;
  double dx, dy;
  double vec1[2], vec2[2];

  memset(feature, 0x0, sizeof(Feature2D_old));

  feature->error = error;
  feature->type = type;
  feature->start = start;
  feature->end = end;
  feature->all = nPoint;
  feature->nTrack = nTrack;
  memcpy(feature->coef, coef, sizeof(double) * 6);
  feature->startSPoint[0] = point[start * 2];
  feature->startSPoint[1] = point[start * 2 + 1];
  feature->endSPoint[0] = point[end * 2];
  feature->endSPoint[1] = point[end * 2 + 1];

  // 楕円、双曲線のcenterと楕円の情報をセット
  if ((type == ConicType_Ellipse) || (type == ConicType_Hyperbola))
    {
      getConicProperty(coef, &type, feature->center, feature->ev,
                       &feature->axis[0], &feature->axis[1]);
    }

  // 中間点を求める
  if (start < end)
    {
      middle = (start + end) / 2;
    }
  else
    {
      middle = (start + end + nPoint) / 2;
      if (middle >= nPoint)
        {
          middle -= nPoint;
        }
    }
  feature->middleSPoint[0] = point[middle * 2];
  feature->middleSPoint[1] = point[middle * 2 + 1];

  if (start < end)
    {
      feature->nPoints = end - start + 1;
    }
  else
    {
      feature->nPoints = nPoint - start + 1 + end;
    }

  // 直線の長さと方向ベクトル
  if (type == ConicType_Line)
    {
      dx = feature->endSPoint[0] - feature->startSPoint[0];
      dy = feature->endSPoint[1] - feature->startSPoint[1];
      feature->lineLengthSQ = dx * dx + dy * dy;
      feature->direction[0] = dx;
      feature->direction[1] = dy;
    }
  // 双曲線の線分長
  if (type == ConicType_Hyperbola)
    {
      vec1[0] = feature->startSPoint[0] - feature->center[0];
      vec1[1] = feature->startSPoint[1] - feature->center[1];
      vec2[0] = feature->endSPoint[0] - feature->center[0];
      vec2[1] = feature->endSPoint[1] - feature->center[1];

      feature->lineAngle = getAngle2D(vec1, vec2);
      feature->lineLength1 = sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1]);
      feature->lineLength2 = sqrt(vec2[0] * vec2[0] + vec2[1] * vec2[1]);
    }

  return;
}

// 特徴点を比較して必要なら追加する
static int
compareAndAddFeature(Features2D_old* features, Feature2D_old* feature,
                     const int sFeature, const int nPoint,
                     Parameters parameters)
{
  int overlap;
  if (overlapFeatures(&overlap, features, sFeature, feature, parameters))
    {
      if (overlap == -1)
        {
          if (addFeature(features, feature) < 0)
            {
              return -1;
            }
        }
      else
        {
          features->feature[overlap] = *feature;
        }
      return 1;
    }
  else
    {
      return 0;
    }
}

// 線分の交点
// 戻り値：0 線分が平行、1 交点あり
static int
intersectLineSegment(const double coef1[3], const double coef2[3], double out[2])
{
  double p[3];

  getCrossProductV3(const_cast<double*>(coef1), const_cast<double*>(coef2), p);

  // 平行な場合
  if (fabs(p[2]) <= getNormV3(p) * VISION_EPS)
    {
      return 0;
    }

  out[0] = p[0] / p[2];
  out[1] = p[1] / p[2];

  return 1;
}

// 内積の計算
#define INPROD2D(vec0, vec1)	(vec0[0]*vec1[0]+vec0[1]*vec1[1])

static void
norm2tangent(const double	*norm,
	     double	*tangent)
{
  tangent[0] = norm[1];
  tangent[1] = -norm[0];
  return;
}


static void
calc_h_t(const double	*coef_l0, //line 0
	 const double	*point_l1, // point
	 const double	*zero, // point t=0
	 double	*h, // height
	 double	*t, // t
	 double	*q) // foot if not NULL
{
  double	tangent[2];
  int	k;
  double	diff[2];

  *h = -INPROD2D(coef_l0, point_l1) - coef_l0[2];

  norm2tangent(coef_l0, tangent);

  for(k = 0; k < 2; k++)
    {
      diff[k] = point_l1[k] - zero[k];
    }

  *t = INPROD2D(diff, tangent);

  if(q)
    {
      for(k = 0; k < 2; k++)
	q[k] = point_l1[k] + *h * coef_l0[k];
    }

  return;
}

// 二つの直線の交点と二点でvertex featureを登録する
// 交点と端点の距離が短ければ登録しない
static int
set_vertex_feature(Features2D_old	*VertexF,
		   double	**coef, //[2][3]
		   double	*point0,//[2]
		   double	*point1,//[2]
		   const Parameters	*param)
{
  double	cross[2];
  double	th_minlengthSQ;
  const ParametersFeature2D	*paramF2D = &param->feature2D;
  Feature2D_old	feature;
  double	*point[2];
  int	ip, k;
  double	diff[2][2];
  double	lenSQ[2];

  point[0] = point0;
  point[1] = point1;

  // coef から交点の計算
  cross[0] = ((coef[1][1])*(-coef[0][2])+(-coef[0][1])*(-coef[1][2]))
    /(coef[0][0]*coef[1][1]-coef[0][1]*coef[1][0]);
  cross[1] = ((-coef[1][0])*(-coef[0][2])+(coef[0][0])*(-coef[1][2]))
    /(coef[0][0]*coef[1][1]-coef[0][1]*coef[1][0]);

  if(cross[0] < 0.0 || cross[0] >= (double)param->colsize ||
     cross[1] < 0.0 || cross[1] >= (double)param->rowsize)
    {
      // 交点が画像内にない
      return 0;
    }

  // 交点とpointの距離を調べ閾値より短ければ何もしない
  th_minlengthSQ = paramF2D->min_length_line*paramF2D->min_length_line;
  for(ip = 0; ip < 2; ip++)
    {
      for(k = 0; k < 2; k++)
	{
	  diff[ip][k] = point[ip][k]- cross[k];
	}
      lenSQ[ip] = INPROD2D(diff[ip], diff[ip]);
      if(lenSQ[ip] < th_minlengthSQ)
	{
	  // 交点と端点 ip が近すぎる
	  return 0;
	}
    }

  // 登録作業
  feature.type = ConicType_Hyperbola;
  // lineAngle は、extractFeature.cppで再計算される
  for(k = 0; k < 2; k++)
    {
      feature.center[k] = cross[k];
      feature.startSPoint[k] = point0[k];
      feature.endSPoint[k] = point1[k];
    }
  feature.lineLength1 = sqrt(lenSQ[0]);
  feature.lineLength2 = sqrt(lenSQ[1]);

  // 線分１からみた線分２の角度が１８０度を越えないようにする。
  // 越えていたら入れ替え
  if(diff[0][0]*diff[1][1] - diff[0][1]*diff[1][0] < 0.0)
    {
      for(k = 0; k < 2; k++)
	{
	  std::swap(feature.startSPoint[k], feature.endSPoint[k]);
	}
      std::swap(feature.lineLength1, feature.lineLength2);
    }
  feature.error = 0.0;
  feature.nTrack = -1;

  feature.arclist.n = 0;
  feature.arclist.arc = NULL;

  // 特徴を保存
  if (addFeature(VertexF, &feature) < 0)
    {
      return -1;
    }

  return 0;
}

// 二本の直線から頂点を形成するかをテストする
// メモリエラー時には-1を返す そのほかは0
static int
test_vertex(Features2D_old	*VertexF,
	    double	**coef, //[NLINE(2)][3]
	    double	*point[2][2], //[NLINE(2)][NPOINT(2)][2]
	    const double	*tmax,    // [NLINE(2)]
	    const Parameters	*param) 
//const ParametersFeature2D	*paramF2D) 
{
  int	i0, i1;
  double	dx, dy, len2;
  double	thr_dist, thr_dist2;
  int	il, ip;
  double	h[2][2], t;
  int	iv;
  int	ip0, ip1;
  const ParametersFeature2D	*paramF2D = &param->feature2D;

  thr_dist = paramF2D->max_distance_end_points;
  thr_dist2 = thr_dist * thr_dist;

  // 1 端点x端点のチェック
  for(i0 = 0; i0 < 2; i0++) // endpoint id for line 0
    {
      for(i1 = 0; i1 < 2; i1++) // endpoint id for line 1
	{
	  dx = point[0][i0][0] - point[1][i1][0];
	  dy = point[0][i0][1] - point[1][i1][1];
	  len2 = dx * dx + dy * dy;

	  if(len2 < thr_dist2)
	    {
	      if(set_vertex_feature(VertexF,
				    coef, point[0][1-i0], point[1][1-i1],
				    param) == -1)
		{
		  return -1;
		}
	      return 0; // only one vertex
	    }
	}
    }

  // 2 check endpoint x line
  for(il = 0; il < 2; il++) // line id
    {
      for(ip = 0; ip < 2; ip++) // point id for another line
	{
	  calc_h_t(coef[il], point[1-il][ip], point[il][0],
		   &h[1-il][ip], &t, NULL);
	  if(fabs(h[1-il][ip]) < thr_dist && 0.0 <= t && t <= tmax[il])
	    // 垂線の足の長さが短く、垂線の足が線分il上にある
	    {
	      for(iv = 0; iv < 2; iv++)
		{
		  if(set_vertex_feature(VertexF, coef,
					point[il][0], point[1-il][ip],
					param) == -1)
		    {
		      return -1;
		    }
		}
	      return 0;
	    }
	}
    }

  // 3 非常に稀であるが、交差する場合
  if(h[0][0]*h[0][1] < 0.0 && h[1][0]*h[1][1] < 0.0)
    {
      for(ip0 = 0; ip0 < 2; ip0++)
	{
	  for(ip1 = 0; ip1 < 2; ip1++)
	    {
	      if(set_vertex_feature(VertexF, coef,
				    point[0][ip0], point[1][ip1],
				    param) == -1)
		{
		  return -1;
		}
	    }
	}
    }

  return 0;
}

// ２直線から頂点を形成
static int
line_to_vertex(Features2D_old	*LineF,
	       Features2D_old	*VertexF,
	       const Parameters	*param)
{
  double	cos_max;
  Feature2D_old	*pfi, *pfj;
  int	i, j;
  double	*point[2][2]; //[NLINE][NPTS]([DIM])
  double	*coef[2]; //[NLINE](0-2) coef[][0]*x+coef[][1]*y+coef[][2]=0
  double	tmax[2];
  double	cos_ij;
  const ParametersFeature2D	*paramF2D = &param->feature2D;

  cos_max = cos(paramF2D->min_radian_hyperbola);

  // check linelength
  pfi = LineF->feature;
  for(i = 0; i < LineF->nFeature; i++, pfi++)
    {
      if(pfi->type == ConicType_Line)
	{
	  if(pfi->lineLength < paramF2D->min_length_line)
	    {
	      pfi->type = ConicType_Unknown;
	    }
	}
    }

  // main loop
  pfi = LineF->feature;
  for(i = 0; i < LineF->nFeature-1; i++, pfi++)
    {
      if(pfi->type == ConicType_Line){
	point[0][0] = pfi->startSPoint;//開始点
	point[0][1] = pfi->endSPoint;  //終端点
	coef[0] = &pfi->coef[3];      //係数　c[][0]*x+c[][1]*y+c[][2] = 0
	tmax[0] = pfi->lineLength; // t の最大値 = lineLength
	
	pfj = pfi+1;
	for(j = i+1; j < LineF->nFeature; j++, pfj++)
	  {
	    if(pfj->type == ConicType_Line){
	      point[1][0] = pfj->startSPoint;//開始点
	      point[1][1] = pfj->endSPoint;  //終端点
	      coef[1] = &pfj->coef[3];      //係数　c[][0]*x+c[][1]*y+c[][2] = 0
	      tmax[1] = pfj->lineLength; // t の最大値 = lineLength

	      // 角度チェック
	      // 法線の内積の絶対値
	      cos_ij = fabs(INPROD2D(coef[0], coef[1]));
	      if(cos_ij < cos_max)
		{
		  // 法線の角度が十分に大きい
		  if(test_vertex(VertexF, coef, point, tmax, param) == -1)
		    {
		      return -1;
		    }
		}
	    } 
	  }
      }
    }
  
  return 0;
}

// 2直線の組み合わせで端点の距離が閾値以内でなす角度が閾値以上の場合に頂点を生成する
static int
Line2Vertex(Features2D_old* lineFeatures, Features2D_old* features, Parameters parameters)
{
  const int nLineFeature = lineFeatures->nFeature;      // 直線特徴の数
  Feature2D_old feature;                                // 最後にあてはまった特徴を記録しておく
  Feature2D_old* feature1;
  Feature2D_old* feature2;                              // 比較する直線特徴
  double startp1[2], endp1[2], startp2[2], endp2[2];
  double dist[4];                                       // 端点ペア間距離
  int nMinPair;                                         // 最小端点間距離のペア番号
  double dx, dy, dx2, dy2;                              // 距離計算の変数
  double lineAngle;                                     // 2直線のなす角度
  double vec1[2], vec2[2];                              // 角度を計算するベクトル

  ParametersFeature2D paramF2D = parameters.feature2D;
  double thr_minD = paramF2D.max_distance_end_points;   // 端点間距離の閾値
  double thr_radian = paramF2D.min_radian_hyperbola;    // 2直線のなす角度閾値（１８０度に近いものを除去する）
  double thr_lineLength = paramF2D.min_length_line;     // 直線の長さの閾値

  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;

  int i, j;

  for (i = 0; i < nLineFeature; i++)
    {
      feature1 = &lineFeatures->feature[i];
      if (feature1->type != ConicType_Line)
        {                       // 念のため直線の特徴であることを確認する
          continue;
        }
      // 無効な線分
      if (feature1->error < 0.0)
        {
          continue;
        }
      // 直線の長さによる閾値処理
      if (feature1->lineLength < thr_lineLength)
        {
          continue;
        }
      // 各端点のセット
      startp1[0] = feature1->startSPoint[0];
      startp1[1] = feature1->startSPoint[1];
      endp1[0] = feature1->endSPoint[0];
      endp1[1] = feature1->endSPoint[1];
      for (j = i + 1; j < nLineFeature; j++)
        {
          feature2 = &lineFeatures->feature[j];
          if (feature2->type != ConicType_Line)
            {                   // 念のため直線の特徴であることを確認する
              continue;
            }
          if (feature2->error < 0.0)
            {
              continue;
            }
          if (feature2->lineLength < thr_lineLength)
            {
              continue;
            }
          // 各端点のセット
          startp2[0] = feature2->startSPoint[0];
          startp2[1] = feature2->startSPoint[1];
          endp2[0] = feature2->endSPoint[0];
          endp2[1] = feature2->endSPoint[1];
          // 各端点間距離の計算
          // S1-S2
          dx = startp1[0] - startp2[0];
          dy = startp1[1] - startp2[1];
          dist[0] = sqrt(dx * dx + dy * dy);
          // S1-E2
          dx = startp1[0] - endp2[0];
          dy = startp1[1] - endp2[1];
          dist[1] = sqrt(dx * dx + dy * dy);
          // E1-S2
          dx = endp1[0] - startp2[0];
          dy = endp1[1] - startp2[1];
          dist[2] = sqrt(dx * dx + dy * dy);
          // E1-E2
          dx = endp1[0] - endp2[0];
          dy = endp1[1] - endp2[1];
          dist[3] = sqrt(dx * dx + dy * dy);
          // 端点間距離の最小の組み合わせ
          if (dist[1] < dist[0])
            {
              nMinPair = 1;
            }
          else
            {
              nMinPair = 0;
            }
          if (dist[2] < dist[nMinPair])
            {
              nMinPair = 2;
            }
          if (dist[3] < dist[nMinPair])
            {
              nMinPair = 3;
            }
          // 最小端点間距離の閾値処理
          if (dist[nMinPair] > thr_minD)
            {
              continue;
            }
          // ２直線のベクトルの成す角度の計算
#if 0
          vec1[0] = endp1[0] - startp1[0];
          vec1[1] = endp1[1] - startp1[1];
          vec2[0] = endp2[0] - startp2[0];
          vec2[1] = endp2[1] - startp2[1];
#else
          vec1[0] = feature1->coef[3];
          vec1[1] = feature1->coef[4];
          vec2[0] = feature2->coef[3];
          vec2[1] = feature2->coef[4];
#endif
          lineAngle = getAngle2D(vec1, vec2);
          // 2直線のベクトルの成す角度が0、180度に近い時組み合わせはスキップする
          if (fabs(lineAngle) < thr_radian || fabs(M_PI - lineAngle) < thr_radian)
            {
              continue;
            }

          // 2直線の交点を求める
          if(intersectLineSegment(&feature1->coef[3], &feature2->coef[3], feature.center) == 0)
            {
              continue;
            }

          // 交点が画像内かどうかの判定
          if (feature.center[0] < 0 || feature.center[0] >= colsize
              || feature.center[1] < 0 || feature.center[1] >= rowsize)
            {
              continue;
            }
          // 特徴タイプを双曲線にセットする
          feature.type = ConicType_Hyperbola;
          // 2直線の成す角度をセットする
          feature.lineAngle = lineAngle;
          // 端点をセットする
          switch (nMinPair)
            {
            case 0:            // SS: E1->S1S2->E2
              feature.startSPoint[0] = endp1[0];
              feature.startSPoint[1] = endp1[1];
              feature.endSPoint[0] = endp2[0];
              feature.endSPoint[1] = endp2[1];
              break;
            case 1:            // SE: E1->S1E2->S2
              feature.startSPoint[0] = endp1[0];
              feature.startSPoint[1] = endp1[1];
              feature.endSPoint[0] = startp2[0];
              feature.endSPoint[1] = startp2[1];
              break;
            case 2:            // ES: S1->E1S2->E2
              feature.startSPoint[0] = startp1[0];
              feature.startSPoint[1] = startp1[1];
              feature.endSPoint[0] = endp2[0];
              feature.endSPoint[1] = endp2[1];
              break;
            case 3:            // EE: S1->E1E2->S2
              feature.startSPoint[0] = startp1[0];
              feature.startSPoint[1] = startp1[1];
              feature.endSPoint[0] = startp2[0];
              feature.endSPoint[1] = startp2[1];
              break;
            default:
              break;
            }
          // 2直線の長さをセットする
          dx = feature.startSPoint[0] - feature.center[0];
          dy = feature.startSPoint[1] - feature.center[1];
          dx2 = feature.endSPoint[0] - feature.center[0];
          dy2 = feature.endSPoint[1] - feature.center[1];
          if (dx * dy2 - dy * dx2 >= 0.0)
            {
              feature.lineLength1 = sqrt(dx * dx + dy * dy);
              feature.lineLength2 = sqrt(dx2 * dx2 + dy2 * dy2);
            }
          else  // 線分1と2の外積が正になるように入れ替える
            {
              std::swap(feature.startSPoint[0], feature.endSPoint[0]);
              std::swap(feature.startSPoint[1], feature.endSPoint[1]);
              feature.lineLength1 = sqrt(dx2 * dx2 + dy2 * dy2);
              feature.lineLength2 = sqrt(dx * dx + dy * dy);
            }
          // その他の情報のセット
          feature.error = 0.0;
          feature.nTrack = -1;

	  feature.arclist.n = 0;
	  feature.arclist.arc = NULL;

          // 特徴を保存
          if (addFeature(features, &feature) < 0)
            {
              return -1;
            }
        }
    }
  return 0;
}

// 直線を検出する
static int
searchLineFeatures(unsigned char* work, Features2D_old* features,
                   int nTrack, Parameters parameters)
{
  const int sFeature = features->nFeature;      // この点列について開始した時点での特徴点の数
  int nPoint = features->track[nTrack].nPoint;  // 輪郭線の点数
  int* point = features->track[nTrack].Point;   // 輪郭線の点列
  double* offset = features->track[nTrack].offset; // 楕円形数計算時のオフセット

  Feature2D_old feature;        // 最後にあてはまった特徴を記録しておく
  double sum[5][5];             // 当てはめのための係数総和行列

  double error;
  ConicType type = ConicType_Unknown;
  double coef[6];

  int skip_len = SKIP_LEN;            // 当てはめに必要な最低限の長さ
  int max_slide_len = MAX_SLIDE_LEN;       // スライドする最大長さ
  int len;

  if (nPoint < skip_len)
    {                           // 小さな部品については探索しない
      return 0;
    }

  clearConicSum(sum);          // 行列の初期化

  // 尺取虫のような方式で特徴点を探す
  int start = 0;                // 最初の開始点は０番より
  int end;

  // 最初の数点を総和に加える
  for (end = 0; end < skip_len; end++)
    {
      addConicSum(sum, &point[end * 2], offset);
    }

  while (1)
    {                           // 全体のループで start が始点に戻ると終了する
      if (start >= nPoint)
        {
          return 0;
        }
      if (end >= nPoint)
        {
          end -= nPoint;
        }
      if (end == start)
        {                       // １周全体であてはめてしまった
          end = start - 1;
          if (end < 0)
            {
              end += nPoint;
            }
          // 直線に当てはまるか
          type = fitConicAny(coef, &error, sum, point, nPoint,
                             start, end, parameters, 1, offset);
          if (type == ConicType_Line)
            {
              memorizeFeature(&feature, error, type, start, end,
                              coef, point, nPoint, nTrack);
              if (compareAndAddFeature(features, &feature, sFeature,
                                       nPoint, parameters) < 0)
                {
                  return -1; // 特徴メモリ確保失敗
                }
            }
          return 0;               // ここで終了
        }
      // 終点データを加算
      addConicSum(sum, &point[end * 2], offset);
      // 直線に当てはまるか
      type = fitConicAny(coef, &error, sum, point, nPoint,
                         start, end, parameters, 1, offset);
      // 直線の場合は、特徴データをセットして、登録チェック
      if (type == ConicType_Line)
        {
          memorizeFeature(&feature, error, type,
                          start, end, coef, point, nPoint, nTrack);
          int addsts = compareAndAddFeature(features, &feature, sFeature,
                                            nPoint, parameters);
          if (addsts > 0)
            {
              // 登録ＯＫなら、延長
              ++end;
              continue;
            }
          else if (addsts < 0)
            {
              return -1; // 特徴メモリ確保失敗
            }
        }
      // 直線でない、もしくは登録ＮＧなので、長さが範囲内ならスライド、範囲外なら始点短縮
      subConicSum(sum, &point[start * 2], offset);
      ++start;
      len = end - start;
      if (len < 0)
        {
          len += nPoint;
        }
      if (len >= nPoint)
        {
          return 0;
        }
      if (len < max_slide_len)
        {
          ++end;                // スライドするため、終点を延長
        }
      else
        {
          // 短縮するため、終点はそのままなので、一度削除する
          subConicSum(sum, &point[end * 2], offset);
        }
    }
  return 0;
}

#if 0
// 特徴点を探す
static int
searchFeatures(unsigned char* work, Features2D_old* features,
               int nTrack, Parameters parameters)
{
  // この点列について開始した時点での特徴点の数
  const int sFeature = features->nFeature;

  double maxErrorofConicFit = parameters.feature2D.maxErrorofConicFit;

  int nPoint = features->track[nTrack].nPoint;  // 輪郭線の点数
  int* point = features->track[nTrack].Point;   // 輪郭線の点列
  double* offset = features->track[nTrack].offset; // 楕円係数計算時のオフセット

  Feature2D_old feature;        // 最後にあてはまった特徴を記録しておく
  double sum[5][5];             // 当てはめのための係数総和行列

  double error;
  double prev_error = DBL_MAX;
  ConicType type = ConicType_Unknown;
  double coef[6];

  int skip_len = 10;            // 当てはめに必要な最低限の長さ
  int max_slide_len = 20;       // スライドする最大長さ
  int len;

  clearConicSum(sum);           // 行列の初期化

  if (nPoint < skip_len)
    {                           // 小さな部品については探索しない
      return 0;
    }

  // 尺取虫のような方式で特徴点を探す
  int start = 0;                // 最初の開始点は０番より
  int end;

  // 最初の数点を総和に加える
  for (end = 0; end < skip_len; end++)
    {
      addConicSum(sum, &point[end * 2], offset);
    }

  while (1)
    {                           // 全体のループで start が始点に戻ると終了する
      // 終点側を伸ばすループ
      feature.end = -1;
      prev_error = DBL_MAX;
      while (1)
        {                       // 当てはまるまで伸ばす
          if (start >= nPoint)
            {
              return 0;
            }
          if (end >= nPoint)
            {
              end -= nPoint;
            }
          if (end == start)
            {                   // １周全体であてはめてしまった
              return 0;         // ここで終了
            }
          // 終点データを加算
          addConicSum(sum, &point[end * 2], offset);
          // どのようなものにあてはまるか？
          type = fitConicAny(coef, &error, sum, point, nPoint,
                             start, end, parameters, 0, offset);

          // 該当なしか放物線なら、長さが範囲内ならスライド、範囲外なら始点短縮
          if (type == ConicType_Unknown || type == ConicType_Parabola)
            {
              subConicSum(sum, &point[start * 2], offset);
              ++start;
              len = end - start;
              if (len < 0)
                {
                  len += nPoint;
                }
              if (len < max_slide_len)
                {
                  ++end;
                }
              else
                {
                  // スライドしない場合は、一度削除しておく
                  subConicSum(sum, &point[end * 2], offset);
                }
              continue;
            }

          // 特徴データをセット
          memorizeFeature(&feature, error, type,
                          start, end, coef, point, nPoint, nTrack);
          // 楕円、双曲線の場合は誤差評価する（直線は評価済み）
          if ((feature.error > maxErrorofConicFit))
            {
              subConicSum(sum, &point[start * 2], offset);
              ++start;
              len = end - start;
              if (len < 0)
                {
                  len += nPoint;
                }
              if (len < max_slide_len)
                {
                  ++end;
                }
              else
                {               // 終点位置が同じのため、終点データを減算しておく
                  subConicSum(sum, &point[end * 2], offset);
                }
              continue;
            }
          // 楕円、双曲線の場合で誤差ＯＫの場合、登録して、両端延長処理へ、
          if (compareAndAddFeature(features, &feature, sFeature,
                                   nPoint, parameters) < 0)
            {
              return -1;
            }
          ++end;
          prev_error = feature.error;
          break;
        }

      // 楕円か双曲線が当てはまった時は、両端延長処理をして、最大当てはめを試す
      ConicType otype = type;
      // 終点を延長
      while (1)
        {
          if (end >= nPoint)
            {
              end -= nPoint;
            }
          if (end == start)
            {
              return 0;
            }
          // 終点データを加算
          addConicSum(sum, &point[end * 2], offset);
          // どのようなものにあてはまるか？
          type = fitConicAny(coef, &error, sum, point, nPoint,
                             start, end, parameters, 0, offset);
          memorizeFeature(&feature, error, type,
                          start, end, coef, point, nPoint, nTrack);
          // 楕円か双曲線なら誤差以内なら保存して延長継続
          if (type != otype     //直前と同じタイプに限定
              || feature.error > maxErrorofConicFit || feature.error > prev_error)
            {
              // だめだったら、延長点を削除して、次の始点延長処理へ
              subConicSum(sum, &point[end * 2], offset);
              break;
            }
          else
            {
              // OKならさらに延長する
              if (compareAndAddFeature(features, &feature, sFeature, nPoint, parameters) < 0)
                {
                  return -1;
                }
              ++end;
              prev_error = feature.error;
            }
        }

      // 始点を延長（開始点まではバックしてみる）
      --start;
      while (1)
        {
          if (start < 0)
            {
              start = 0;
              break;
            }
          if (start == end)
            {
              return 0;
            }
          addConicSum(sum, &point[start * 2], offset);
          // どのようなものにあてはまるか？
          type = fitConicAny(coef, &error, sum, point, nPoint,
                             start, end, parameters, 0, offset);
          memorizeFeature(&feature, error, type,
                          start, end, coef, point, nPoint, nTrack);
          // 楕円か双曲線なら誤差以内なら保存して延長継続
          if (type != otype     //直前と同じタイプに限定
              || feature.error > maxErrorofConicFit || feature.error > prev_error)
            {
              // だめだったら、延長点を削除して、次へ
              subConicSum(sum, &point[start * 2], offset);
              ++start;
              break;
            }
          else
            {
              // OKならさらに延長
              if (compareAndAddFeature(features, &feature, sFeature,
                                       nPoint, parameters) < 0)
                {
                  return -1;
                }
              --start;
              prev_error = feature.error;
            }
        }

      // 始点短縮処理を試みて誤差が小さくなる場合は短縮する
      if (start >= nPoint)
        {
          return 0;
        }
      subConicSum(sum, &point[start * 2], offset);
      ++start;
      while (1)
        {
          if (start >= nPoint)
            {
              return 0;
            }
          len = end - start;
          if (len < 0)
            {
              len += nPoint;
            }
          if (len < max_slide_len)
            {
              break;            // 最初に戻る
            }
          // どのようなものにあてはまるか？
          type = fitConicAny(coef, &error, sum, point, nPoint,
                             start, end, parameters, 0, offset);
          memorizeFeature(&feature, error, type,
                          start, end, coef, point, nPoint, nTrack);
          if (type != otype     //直前と同じタイプに限定
              || feature.error > maxErrorofConicFit || feature.error > prev_error)
            {
              // だめだったら抜ける
              break;
            }
          else
            {
              if (compareAndAddFeature(features, &feature,
                                       sFeature, nPoint, parameters) < 0)
                {
                  return -1;
                }
              // OKならさらに短縮してみて続ける
              subConicSum(sum, &point[start * 2], offset);
              ++start;
              prev_error = feature.error;
            }
        }
    }
  return 0;
}
#endif

// 作業画像から長い直線を除去する
static void
deleteLongLines(unsigned char* work, Features2D_old* lineFeatures,
                Parameters parameters)
{
  const int nLineFeature = lineFeatures->nFeature;      // 直線特徴の数
  double thr_lineLength = parameters.feature2D.max_length_delete_line;  // 長いと判断する直線の長さの閾値：オプション化すべき

  int f, istart, iend, i, ii, ipx, ipy, n;
  Feature2D_old* feature;
  int* pt;
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int imgsize = parameters.imgsize;

  for (f = 0; f < nLineFeature; f++)
    {
      feature = &lineFeatures->feature[f];
      if (feature->type != ConicType_Line)
        {                       // 念のため直線の特徴であることを確認する
          continue;
        }
      // 長い直線を消す
      if (feature->lineLength < thr_lineLength)
        {
          continue;
        }
      if (feature->nTrack >= 0)
        {
          pt = lineFeatures->track[feature->nTrack].Point;
          istart = feature->start;
          iend = feature->end;
          if (istart > iend)
            {
              iend += feature->all;
            }
          for (i = istart + 1; i < iend - 1; i++)
            {
              ii = (i % feature->all) * 2;
              ipx = pt[ii];
              ipy = pt[ii + 1];
              if ((ipx >= 0 && ipx < colsize) && ipy >= 0 && ipy < rowsize)
                {
                  n = ipy * colsize + ipx;
                  work[n] = 0;
                }
            }
        }
    }
  for (i = 0; i < imgsize; i++)
    {
      if (work[i] > 0)
        {
          work[i] = 1;
        }
    }
  return;
}

// 特徴をマージする
static int
mergeFeatures(Features2D_old* dstFeatures, Features2D_old* srcFeatures)
{
  Feature2D_old* feature;
  int f;

  for (f = 0; f < srcFeatures->nFeature; f++)
    {
      feature = &srcFeatures->feature[f];
      if (addFeature(dstFeatures, feature) < 0)
        {
          return -1; // メモリ確保失敗
        }
    }
  return 0;
}

// 楕円検出結果から、中心の近い複数を使って、精度の高い楕円検出を行う
static int
Ellipse2Ellipse(Features2D_old* features, Features2D_old* ellipseFeatures,
                Parameters parameters)
{
  double sum[5][5];             // 当てはめのための係数総和行列
  double maxErrorofConicFit = parameters.feature2D.maxErrorofConicFit;

  // 楕円中心でグルーピング
  Feature2D_old* feature1;
  Feature2D_old* feature2;
  Feature2D_old newFeature;
  int nFeatures = ellipseFeatures->nFeature;
  EllipseGroup* ellipseGroup;
  EllipseGroup* currGroup;
  int i, ii, istart, iend, f1, f2;
  bool* checkflag;
  int nCheckFlag = 0;
  double dx, dy, distance;
  int* point;                   // 輪郭線の点列
  double coef[3][6], dist, error, minError;
  int c, nConic, nn, minC;
  double thr_distance = parameters.feature2D.max_distance_ellipse_grouping;     // 中心距離判定閾値

  double offset[2]; // 楕円係数計算時のオフセット

  Features2D_old* newFeatures = constructFeatures();       // 新しい楕円特徴用
  if (newFeatures == NULL)
    {
      return -1;
    }

  if ((ellipseGroup = (EllipseGroup*) malloc(sizeof(EllipseGroup) * nFeatures)) == NULL)
    {
      destructFeatures(newFeatures);
      return -1;
    }
  for (i = 0; i < nFeatures; i++)
    {
      if ((ellipseGroup[i].groupNums = (int*) malloc(sizeof(int) * nFeatures)) == NULL)
        {
          for (ii = 0; ii < i; ii++)
            {
              free(ellipseGroup[ii].groupNums);
            }
          free(ellipseGroup);
          destructFeatures(newFeatures);
          return -1;
        }
      ellipseGroup[i].groupNums[0] = -1;
      ellipseGroup[i].nCurrNum = 0;
      ellipseGroup[i].groupCenter[0] = -1.0;
      ellipseGroup[i].groupCenter[1] = -1.0;
    }
  if ((checkflag = (bool*) malloc(sizeof (bool) * nFeatures)) == NULL)
    {
      for (ii = 0; ii < nFeatures; ii++)
        {
          free(ellipseGroup[ii].groupNums);
        }
      free(ellipseGroup);
      destructFeatures(newFeatures);
      return -1;
    }
  memset(checkflag, 0, sizeof(bool) * nFeatures);

  while (nCheckFlag < nFeatures)
    {
      for (f1 = 0; f1 < ellipseFeatures->nFeature; f1++)
        {
          feature1 = &ellipseFeatures->feature[f1];
          if (feature1->type != ConicType_Ellipse || feature1->nPoints <= 0)
            {
              nCheckFlag++;
              checkflag[f1] = 1;
              continue;
            }
          if (!checkflag[f1])
            {
              currGroup = &ellipseGroup[f1];
              currGroup->groupCenter[0] = feature1->center[0];
              currGroup->groupCenter[1] = feature1->center[1];
              nCheckFlag++;
              checkflag[f1] = 1;
              currGroup->groupNums[currGroup->nCurrNum] = f1;
              currGroup->nCurrNum++;
              for (f2 = 0; f2 < ellipseFeatures->nFeature; f2++)
                {
                  feature2 = &ellipseFeatures->feature[f2];
                  if ((feature2->type != ConicType_Ellipse)
                      || (feature2->nPoints <= 0))
                    {
                      checkflag[f2] = 1;
                      nCheckFlag++;
                      continue;
                    }
                  if (!checkflag[f2])
                    {
                      // 中心の距離
                      dx = currGroup->groupCenter[0] - feature2->center[0];
                      dy = currGroup->groupCenter[1] - feature2->center[1];
                      distance = sqrt(dx * dx + dy * dy);
                      if (distance < thr_distance)
                        {
                          //中心を平均する
                          currGroup->groupCenter[0] = (currGroup->groupCenter[0] * currGroup->nCurrNum + feature2->center[0]) / (currGroup->nCurrNum + 1);
                          currGroup->groupCenter[1] = (currGroup->groupCenter[1] * currGroup->nCurrNum + feature2->center[1]) / (currGroup->nCurrNum + 1);
                          currGroup->groupNums[currGroup->nCurrNum] = f2;
                          currGroup->nCurrNum++;
                          checkflag[f2] = 1;
                          nCheckFlag++;
                        }
                    }
                }
            }
        }
    }

  int retsts = 0;

  // グループ毎の点データで楕円当てはめ
  for (f1 = 0; f1 < ellipseFeatures->nFeature; f1++)
    {
      currGroup = &ellipseGroup[f1];
      offset[0] = currGroup->groupCenter[0];
      offset[1] = currGroup->groupCenter[1];
      if (currGroup->nCurrNum > 1)
        {
          clearConicSum(sum);  // 行列の初期化
          for (f2 = 0; f2 < currGroup->nCurrNum; f2++)
            {
              feature2 = &ellipseFeatures->feature[currGroup->groupNums[f2]];
              point = ellipseFeatures->track[feature2->nTrack].Point;
              istart = feature2->start;
              iend = feature2->end;
              if (istart > iend)
                {
                  iend += feature2->all;
                }
              for (i = istart; i <= iend; i++)
                {
                  ii = (i % feature2->all) * 2;
                  addConicSum(sum, &point[ii], offset);
                }
            }
          nConic = fitConic(sum, coef, offset);
          if (nConic <= 0)
            {
              continue;
            }
          // 解の中から妥当なものを選択す
          minError = maxErrorofConicFit;
          minC = -1;

          // 解の中から誤差最小の当てはめを選択
          for (c = 0; c < nConic; c++)
            {
              error = 0.0;
              nn = 0;
              for (f2 = 0; f2 < currGroup->nCurrNum; f2++)
                {
                  feature2 = &ellipseFeatures->feature[currGroup->groupNums[f2]];
                  point = ellipseFeatures->track[feature2->nTrack].Point;
                  istart = feature2->start;
                  iend = feature2->end;
                  if (istart > iend)
                    {
                      iend += feature2->all;
                    }
                  for (i = istart; i <= iend; i++)
                    {
                      ii = (i % feature2->all) * 2;
                      dist = distanceConic(coef[c], &point[ii]);
                      if (dist >= 0.0)
                        {
                          error += dist;
                          nn++;
                        }
                    }
                }
              error /= (double) nn;
              if (minError > error)
                {
                  minC = c;
                  minError = error;
                }
            }
          if (minC == -1)
            {
              continue;
            }
          // 楕円に当てはまった場合のみ採用
          if (getConicType(coef[minC]) != ConicType_Ellipse)
            {
              continue;
            }
          if (minError > maxErrorofConicFit)
            {
              continue;
            }
          newFeature.error = minError;
          memcpy(newFeature.coef, coef[minC], sizeof(double) * 6);
          newFeature.type = ConicType_Ellipse;
          getConicProperty(coef[minC], &newFeature.type, newFeature.center,
                           newFeature.ev, &newFeature.axis[0],
                           &newFeature.axis[1]);
          newFeature.nTrack = -1;
          newFeature.nPoints = -1;
	  newFeature.arclist.n = 0;
	  newFeature.arclist.arc = NULL;
          // 生成特徴を追加
          if (addFeature(newFeatures, &newFeature) < 0)
            {
              retsts = -1;
              goto ending;
            }
        }
    }

  // 新規生成特徴をマージする
  retsts = mergeFeatures(features, newFeatures);

 ending:

  // 終了処理
  for (i = 0; i < nFeatures; i++)
    {
      free(ellipseGroup[i].groupNums);
    }
  free(ellipseGroup);
  free(checkflag);

  destructFeatures(newFeatures);

  return retsts;
}

// 楕円検出結果から、２つの楕円のデータを使って、楕円当てはめを行い、マージする
static int
Ellipse2Ellipse2(Features2D_old* features,
                 Features2D_old* ellipseFeatures, Parameters parameters)
{
  Features2D_old* newFeatures = constructFeatures();       // 新しい楕円特徴用
  double sum[5][5];             // 当てはめのための係数総和行列
  double maxErrorofConicFit = parameters.feature2D.maxErrorofConicFit;

  // 楕円中心でグルーピング
  Feature2D_old* feature1;
  Feature2D_old* feature2;
  Feature2D_old newFeature;
  Feature2D_old* tmpFeature;
  int nFeatures = ellipseFeatures->nFeature;
  EllipseGroup* ellipseGroup;
  EllipseGroup* currGroup;
  int i, ii, istart, iend, f1, f2;
  double dx, dy, distance;
  int *point;                   // 輪郭線の点列
  double coef[3][6], dist, error, minError;
  int c, nConic, nn, minC;

  ParametersFeature2D paramF2D = parameters.feature2D;
  double thr_distanceMin = paramF2D.min_distance_ellipse_pairing;       // 中心距離判定閾値（オプション化すべき）
  double thr_distanceMax = paramF2D.max_distance_ellipse_pairing;       // 中心距離判定閾値（オプション化すべき）
  double thr_min_length_ellipse_axisS = paramF2D.min_length_ellipse_axisS;      // 楕円の軸長の閾値
  double thr_min_length_ellipse_axisL = paramF2D.min_length_ellipse_axisL;      // 楕円の軸長の閾値
  double thr_max_length_ellipse_axisL = paramF2D.max_length_ellipse_axisL;      // 楕円の軸長の閾値
  double thr_max_flatness_ellipse = paramF2D.max_flatness_ellipse;      // 楕円の偏平率（長軸/短軸)

  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;

  double offset[2]; // 楕円係数計算時のオフセット

  if ((ellipseGroup = (EllipseGroup*) malloc(sizeof(EllipseGroup) * nFeatures)) == NULL)
    {
      destructFeatures(newFeatures);
      return -1;
    }

  for (i = 0; i < nFeatures; i++)
    {
      if ((ellipseGroup[i].groupNums = (int*) malloc(sizeof(int) * nFeatures)) == NULL)
        {
          for (ii = 0; ii < i; ii++)
            {
              free(ellipseGroup[ii].groupNums);
            }
          free(ellipseGroup);
          destructFeatures(newFeatures);
          return -1;
        }
      for (ii = 0; ii < nFeatures; ii++)
        {
          ellipseGroup[i].groupNums[ii] = -1;
        }
      ellipseGroup[i].nCurrNum = 0;
      ellipseGroup[i].groupCenter[0] = -1.0;
      ellipseGroup[i].groupCenter[1] = -1.0;
    }

  for (f1 = 0; f1 < nFeatures; f1++)
    {
      feature1 = &ellipseFeatures->feature[f1];
      if (feature1->type != ConicType_Ellipse || feature1->nPoints <= 0)
        {
          continue;
        }
      currGroup = &ellipseGroup[f1];
      // 長軸が短いものはスキップする
      if (feature1->axis[0] > feature1->axis[1])
        {
          if (feature1->axis[0] < thr_min_length_ellipse_axisL)
            {
              continue;
            }
        }
      else
        {
          if (feature1->axis[1] < thr_min_length_ellipse_axisL)
            {
              continue;
            }
        }
      for (f2 = f1 + 1; f2 < nFeatures; f2++)
        {
          feature2 = &ellipseFeatures->feature[f2];
          if (feature2->type != ConicType_Ellipse || feature2->nPoints <= 0)
            {
              continue;
            }
          // 長軸が短いものはスキップする
          if (feature2->axis[0] > feature2->axis[1])
            {
              if (feature2->axis[0] < thr_min_length_ellipse_axisL)
                {
                  continue;
                }
            }
          else
            {
              if (feature2->axis[1] < thr_min_length_ellipse_axisL)
                {
                  continue;
                }
            }
          // 中心の距離
          dx = feature1->center[0] - feature2->center[0];
          dy = feature1->center[1] - feature2->center[1];
          distance = sqrt(dx * dx + dy * dy);
          // 中心の距離が閾値以内だったら、組み合わせる
          if (distance > thr_distanceMin && distance < thr_distanceMax)
            {
              currGroup->groupNums[f2] = 1;
              currGroup->nCurrNum++;
            }
        }
    }

  int retsts = 0;

  // グループ毎の点データで楕円当てはめ
  for (f1 = 0; f1 < nFeatures; f1++)
    {
      currGroup = &ellipseGroup[f1];
      if (currGroup->nCurrNum > 0)
        {
          for (f2 = f1 + 1; f2 < nFeatures; f2++)
            {
              if (currGroup->groupNums[f2] > 0)
                {
                  clearConicSum(sum);  // 行列の初期化
                  // f1
                  feature1 = &ellipseFeatures->feature[f1];
                  point = ellipseFeatures->track[feature1->nTrack].Point;
                  istart = feature1->start;
                  iend = feature1->end;
                  if (istart > iend)
                    {
                      iend += feature1->all;
                    }
                  offset[0] = feature1->center[0];
                  offset[1] = feature1->center[1];
                  for (i = istart; i <= iend; i++)
                    {
                      ii = (i % feature1->all) * 2;
                      addConicSum(sum, &point[ii], offset);
                    }
                  // f2
                  feature2 = &ellipseFeatures->feature[f2];
                  point = ellipseFeatures->track[feature2->nTrack].Point;
                  istart = feature2->start;
                  iend = feature2->end;
                  if (istart > iend)
                    {
                      iend += feature2->all;
                    }
                  for (i = istart; i <= iend; i++)
                    {
                      ii = (i % feature2->all) * 2;
                      addConicSum(sum, &point[ii], offset);
                    }
                  nConic = fitConic(sum, coef, offset);
                  if (nConic <= 0)
                    {
                      continue;
                    }
                  // 解の中から妥当なものを選択す
                  minError = maxErrorofConicFit;
                  minC = -1;

                  // 解の中から誤差最小の当てはめを選択
                  for (c = 0; c < nConic; c++)
                    {
                      error = 0.0;
                      nn = 0;
                      // f1
                      feature1 = &ellipseFeatures->feature[f1];
                      point = ellipseFeatures->track[feature1->nTrack].Point;
                      istart = feature1->start;
                      iend = feature1->end;
                      if (istart > iend)
                        {
                          iend += feature1->all;
                        }
                      for (i = istart; i <= iend; i++)
                        {
                          ii = (i % feature1->all) * 2;
                          dist = distanceConic(coef[c], &point[ii]);
                          if (dist >= 0.0)
                            {
                              error += dist;
                              nn++;
                            }
                        }
                      // f2
                      feature2 = &ellipseFeatures->feature[f2];
                      point = ellipseFeatures->track[feature2->nTrack].Point;
                      istart = feature2->start;
                      iend = feature2->end;
                      if (istart > iend)
                        {
                          iend += feature2->all;
                        }
                      for (i = istart; i <= iend; i++)
                        {
                          ii = (i % feature2->all) * 2;
                          dist = distanceConic(coef[c], &point[ii]);
                          if (dist >= 0.0)
                            {
                              error += dist;
                              nn++;
                            }
                        }
                      error /= (double) nn;
                      if (minError > error)
                        {
                          minC = c;
                          minError = error;
                        }
                    }
                  if (minC == -1)
                    {
                      continue;
                    }
                  // 楕円に当てはまった場合のみ採用
                  if (getConicType(coef[minC]) != ConicType_Ellipse)
                    {
                      continue;
                    }
                  // 誤差が閾値以内であれば採用
                  if (minError > maxErrorofConicFit)
                    {
                      continue;
                    }
                  newFeature.error = minError;
                  memcpy(newFeature.coef, coef[minC], sizeof(double) * 6);
                  newFeature.type = ConicType_Ellipse;
                  //newFeature.nTrack = -1;
                  getConicProperty(coef[minC], &newFeature.type,
                                   newFeature.center, newFeature.ev,
                                   &newFeature.axis[0], &newFeature.axis[1]);
                  // 中心が画像外の時は、エラー
                  if (newFeature.center[0] < 0.0 ||
                      newFeature.center[0] > (double) (colsize - 1) ||
                      newFeature.center[1] < 0.0 ||
                      newFeature.center[1] > (double) (rowsize - 1))
                    {
                      continue;
                    }
                  // 長軸の長さが閾値以上、閾値以下、短軸の長さが閾値以下ならエラー
                  if (newFeature.axis[0] > newFeature.axis[1])
                    {
                      if (newFeature.axis[0] > thr_max_length_ellipse_axisL ||
                          newFeature.axis[0] < thr_min_length_ellipse_axisL ||
                          newFeature.axis[1] < thr_min_length_ellipse_axisS)
                        {
                          continue;
                        }
                    }
                  else
                    {
                      if (newFeature.axis[1] > thr_max_length_ellipse_axisL ||
                          newFeature.axis[1] < thr_min_length_ellipse_axisL ||
                          newFeature.axis[0] < thr_min_length_ellipse_axisS)
                        {
                          continue;
                        }
                    }
                  // 偏平率が閾値以上ならエラー
                  if (newFeature.axis[0] > newFeature.axis[1])
                    {
                      if (newFeature.axis[0] / newFeature.axis[1] > thr_max_flatness_ellipse)
                        {
                          continue;
                        }
                    }
                  else
                    {
                      if (newFeature.axis[1] / newFeature.axis[0] > thr_max_flatness_ellipse)
                        {
                          continue;
                        }
                    }

                  {
                    // 重複チェック
                    double thr_coefdiff = 1.0E-3;
                    int iSame = 0;
                    for (i = 0; i < newFeatures->nFeature; i++)
                      {
                        tmpFeature = &newFeatures->feature[i];
                        for (ii = 0; ii < 6; ii++)
                          {
                            if (fabs(tmpFeature->coef[ii] - newFeature.coef[ii])
                                > thr_coefdiff)
                              {
                                break;
                              }
                          }
                        if (ii >= 6)
                          {
                            iSame = 1;
                            break;
                          }
                      }
                    if (iSame)
                      {
                        continue;
                      }
                  }

                  newFeature.nTrack = -1;
                  newFeature.nPoints = -1;
		  newFeature.arclist.n = 0;
		  newFeature.arclist.arc = NULL;
                  // 生成特徴を追加
                  if (addFeature(newFeatures, &newFeature) < 0)
                    {
                      retsts = -1;
                      goto ending;
                    }
                }
            }
        }
      else
        {
          // ペアがなかったものは、そのまま追加する
          //addFeature(newFeatures, &ellipseFeatures->feature[f1]);
        }
    }

  // 新規生成特徴をマージする
  retsts = mergeFeatures(features, newFeatures);

 ending:
  // 終了処理
  destructFeatures(newFeatures);
  for (i = 0; i < nFeatures; i++)
    {
      free(ellipseGroup[i].groupNums);
    }
  free(ellipseGroup);
  return retsts;
}

#define Work(col, row) (work[(row)*colsize+(col)])

// このプログラム内での８方位の方向の定義
// column の＋方向は右、row の＋方向は下
// 0: 左下 col- row+ 方向
// 1: 下   col0 row+ 方向
// 2: 右下 col+ row+ 方向
// 3: 右   col+ row0 方向
// 4: 左上 col+ row- 方向
// 5: 上   col0 row- 方向
// 6: 左上 col- row- 方向
// 7: 左   col- row0 方向

// 移動方向を与えると、その差を与える配列
// 剰余系を使用しやすくするために、２周分(0,1,,,15)を定義する

static const int dv[16][2] = {
  {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0},
  {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}
};


static const int markTable[8][8] = {
  {1, 1, 1, 1, 1, 0, 0, 0},
  {1, 1, 1, 1, 1, 1, 0, 0},
  {1, 1, 1, 1, 1, 1, 1, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {1, 0, 0, 0, 0, 0, 0, 0},
  {1, 1, 0, 0, 0, 0, 0, 0},
  {1, 1, 1, 1, 0, 0, 0, 0},
  {1, 1, 1, 1, 1, 0, 0, 0}
};

// 8連結境界線追跡し、追跡済のマークをつける
static int
trackPoints(int* point, int* nPoint,
            unsigned char* work, const int colsize,
            const int rowsize, const int icol, const int irow)
{
  *nPoint = 0;                  // 点数を初期化
  const int mark = 2;           // このルーティンでは固定数字
  // 探索を開始する点
  int col = icol;
  int row = irow;
  int d, idir;

  // 初期位置の点のまわりを探してみる(4連結)
  for (d = 0; d < 8; d++)
    {
      if (Work(icol + dv[d][0], irow + dv[d][1]) >= 1)
        {
          break;
        }
    }
  if (d == 8)
    {                           // まわりに点がなかった
      Work(icol, irow) = mark;  // マークのみ実行
      return 1;                 // 孤立点なので以降の処理は行わない
    }

  // 最初に移動する方向を記録
  idir = d;

  do
    {                       // 同じ点に同じ向きから戻ってくるまで境界線を追跡する
      if (col == 0 || col == colsize - 1 || row == 0 || row == rowsize - 1)
        {
          return 0;
        }

      // 現在の点を記録する
      point[*nPoint * 2] = col;
      point[*nPoint * 2 + 1] = row;
      (*nPoint)++;

      // 次に移動する方向を覚えておく
      const int prevdir = d;

      // 指示された方向に (col, row) を移動する
      col += dv[d][0];
      row += dv[d][1];

      // 次に移動する方向を、探す。初期値を設定
      const int sd = (prevdir >= 2) ? prevdir - 2 : prevdir + 6;
      for (d = sd; d < sd + 7; d++)
        {
          if (Work(col + dv[d][0], row + dv[d][1]) >= 1)
            {
              break;
            }
        }

      if (d >= 8)
        {
          d -= 8;               // ８の剰余とする
        }

      // dの方向によってマークするかどうかが異なるので注意
      if (markTable[prevdir][d] == 1)
        {
          Work(col, row) = mark;       // mark
        }
    }
  while ((row != irow) || (col != icol) || (d != idir));

  return *nPoint;
}

// 順番に境界線点列（１周分）を取り出す
static int
extractTrackPoints(Features2D_old* features,
                   unsigned char* work, Parameters parameters)
{
  int* point;
  int nPoint;
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int col, row;
  int bcount = 0;
  int i;

  const int maxPoint = colsize * rowsize;

  // 点列を記憶するための領域を確保する
  if ((point = (int*) malloc(sizeof(int) * 2 * maxPoint)) == NULL)
    {
      return -1;
    }
  if ((features->track = (Track*) calloc(maxPoint, sizeof(Track))) == NULL)
    {
      free(point);
      return -1;
    }

  for (row = 1; row < rowsize - 1; row++)
    {
      int left = 0;             // エッジ点が見つかっていない状態
      for (col = 1; col < colsize - 1; col++)
        {
          // 開始点を探す
          if (left == 0)
            {                   // 左隣は境界線外となっている
              if (Work(col, row) >= 2)
                {               // ここから既知の境界線内となる
                  left = Work(col, row);
                }
              else if (Work(col, row) == 1)
                {               // 未知の境界線が見つかった
                  memset(point, 0, sizeof(int) * 2 * maxPoint);
                  features->track[bcount].nPoint = 0;
                  features->track[bcount].Point = NULL;
                  if (trackPoints(point, &nPoint, work, colsize, rowsize, col, row)
                      > parameters.feature2D.minFragment)
                    {
                      // 見つかった境界線を保存する
                      if ((features->track[bcount].Point =
                           (int*) malloc(sizeof(int) * 2 * nPoint)) == NULL)
                        {
                          for (i = 0; i < bcount; i++)
                            {
                              free(features->track[i].Point);
                            }
                          free(features->track);
                          free(point);
                          return -1;
                        }
                      features->nTrack = bcount + 1;    // 総トラック数
                      features->track[bcount].nPoint = nPoint;
                      memcpy(features->track[bcount].Point, point,
                             sizeof(int) * 2 * nPoint);
                      // 楕円係数計算時のオフセット計算＝境界点列の重心
                      double sx = 0.0;
                      double sy = 0.0;
                      for (i=0; i<nPoint; i++)
                        {
                          sx += point[i*2];
                          sy += point[i*2+1];
                        }
                      features->track[bcount].offset[0] = (double)sx/(double)nPoint;
                      features->track[bcount].offset[1] = (double)sy/(double)nPoint;
                      bcount++;
                    }
                  left = 2;
                }
            }
          else
            {                   // 左隣は境界線内である
              if (Work(col, row) == 0)
                {               // ここは境界線の右端
                  left = 0;
                }
              else
                {
                  Work(col, row) = 2;
                }
            }
        }
    }

  free(point);

  return 0;
}

#undef Work

// 輪郭点のコピー
static int
copyTrackPoints(Features2D_old* dstFeatures, Features2D_old* srcFeatures,
                Parameters parameters)
{
  int nPoint, iTrack;
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;

  const int maxPoint = colsize * rowsize;

  if ((dstFeatures->track = (Track*) malloc(sizeof(Track) * maxPoint)) == NULL)
    {
      destructFeatures(dstFeatures);
      return -1;
    }
  for (iTrack = 0; iTrack < srcFeatures->nTrack; iTrack++)
    {
      nPoint = srcFeatures->track[iTrack].nPoint;
      if ((dstFeatures->track[iTrack].Point = (int*) malloc(sizeof(int) * 2 * nPoint)) == NULL)
        {
          destructFeatures(dstFeatures);
          return -1;
        }
      memcpy(dstFeatures->track[iTrack].Point,
             srcFeatures->track[iTrack].Point, sizeof(int) * 2 * nPoint);
      dstFeatures->track[iTrack].nPoint = nPoint;
      dstFeatures->track[iTrack].offset[0] = srcFeatures->track[iTrack].offset[0];
      dstFeatures->track[iTrack].offset[1] = srcFeatures->track[iTrack].offset[1];
    }
  dstFeatures->nTrack = srcFeatures->nTrack;

  return 0;
}

// 端点が近い線分に印をつける
void
mark_similar_lines(Features2D_old* lineFeatures, const double tolerance)
{
  const int num = lineFeatures->nFeature;
  int i, j;

  for (i = 0; i < num - 1; ++i)
    {
      Feature2D_old* f_i = &lineFeatures->feature[i];
      const double err_i = f_i->error;

      if (f_i->error < 0.0  || f_i->type != ConicType_Line)
        {
          continue;
        }

      for (j = i + 1; j < num; ++j)
        {
          Feature2D_old* f_j = &lineFeatures->feature[j];
          double* is[2] = {&f_i->startSPoint[0], &f_i->startSPoint[1]};
          double* ie[2] = {&f_i->endSPoint[0],   &f_i->endSPoint[1]};
          double* js[2] = {&f_j->startSPoint[0], &f_j->startSPoint[1]};
          double* je[2] = {&f_j->endSPoint[0],   &f_j->endSPoint[1]};
          double distance[2][2];
          bool is_similar = false;

          if (f_j->error < 0.0  || f_j->type != ConicType_Line)
            {
              continue;
            }

          // 各端点の組み合わせの距離
          distance[0][0] = hypot(*is[0] - *js[0], *is[1] - *js[1]);
          distance[0][1] = hypot(*ie[0] - *je[0], *ie[1] - *je[1]);
          distance[1][0] = hypot(*is[0] - *je[0], *is[1] - *je[1]);
          distance[1][1] = hypot(*ie[0] - *js[0], *ie[1] - *js[1]);

          if (distance[0][0] <= distance[1][0])
            {
              // 始点ー始点
              is_similar = (distance[0][0] < tolerance && distance[0][1] < tolerance);
            }
          else
            {
              // 始点ー終点
              is_similar = (distance[1][0] < tolerance && distance[1][1] < tolerance);
            }

          // 近い線分同士なら誤差が小さい・長い線分の方を残す
          if (is_similar)
            {
              const double diff = f_j->error - err_i;

              //printf("%d %d are similar\n", i, j);
              if (diff > VISION_EPS)
                {
                  // f_iの誤差が小さい
                  f_j->error = -1.0;
                }
              else if (fabs(diff) <= VISION_EPS)
                {
                  // 誤差がほぼ同じ線分のとき
                  if (f_i->lineLength >= f_j->lineLength)
                    {
                      f_j->error = -1.0;
                    }
                  else
                    {
                      f_i->error = -1.0;
                    }
                }
              else
                {
                  // f_jの誤差が小さい
                  f_i->error = -1.0;
                }
            }
        }
    }
}

// 二つの楕円特徴が相互に誤差範囲にあるかどうかを確認
#define CONIC_MATCH_OK	(0)
#define CONIC_MATCH_NG	(1)

static int
check_conic_match(Feature2D_old	*f_p, // 点列用
		  double	*coef, // 曲線用
 		  ParamEllipseIW *paramEIW)
{
  double	maxError, meanError;
  int	np;
  double	sum;
  EllipseArc	*parc;
  int	iarc;
  Features2D_old	*pf2ds;
  int	nPoint;
  int	*point;
  int	start, goal;
  int	i, ip;
  double	e;

  maxError = 0.0;
  np = 0;
  sum = 0.0;
  parc = f_p->arclist.arc;
  for (iarc = 0; iarc < f_p->arclist.n; iarc++, parc++)
    {
      pf2ds = parc->f2Ds;
      nPoint = pf2ds->track[parc->ntrack].nPoint;
      point = pf2ds->track[parc->ntrack].Point;
      start = parc->start;
      goal = parc->goal;
      for (i = start; i <= goal; i++)
	{
	  ip = mod_nPoint(i, nPoint);
          e = fabs(distanceAConic((double *)coef, &point[ip*2]));
	  sum += e;
	  np++;
          if (e > maxError)
            {
              maxError = e;
            }
	}
    }
  meanError = sum / (double)np;

  switch (paramEIW->Condition)
    {
    case ELLIPSE_CONDITION_MEAN:
      if (meanError > paramEIW->ThMeanError)
	{
	  return CONIC_MATCH_NG;
	}
      break;
    case ELLIPSE_CONDITION_MAX:
      if (maxError > paramEIW->ThMaxError)
	{
	  return CONIC_MATCH_NG;
	}
      break;
    }

  return CONIC_MATCH_OK;
}


static int
check_conic_crossmatch(Feature2D_old	*fi,
		       Feature2D_old	*fj,
		       ParamEllipseIW	*paramEIW)
{
  // fi の楕円の点に fj の楕円曲線があてはまるかどうかのチェック
  if (check_conic_match(fi, fj->coef, paramEIW) == CONIC_MATCH_NG)
    {
      return CONIC_MATCH_NG;
    }
  
  // fj の楕円の点に fi の楕円曲線があてはまるかどうかのチェック
  if (check_conic_match(fj, fi->coef, paramEIW) == CONIC_MATCH_NG)
    {
      return CONIC_MATCH_NG;
    }

  return CONIC_MATCH_OK;
}
 
// 重複する楕円特徴を統合する

static int
comp_flist (const void *pa,
	    const void *pb)
{
  Feature_List	*a, *b;

  a = (Feature_List *)pa;
  b = (Feature_List *)pb;

  return (b->np - a->np);
}

#ifdef DEBUG_FLIST
static void
debug_flist(Feature_List	*flist,
	    int	nflist)
{
  Feature_List	*pfl;
  int	i, j;
  Feature2D_old	*ppf2D;
  EllipseArc	*parc;
  int	ntrack;
  Track	*ptrack;
  int	countE;
  
  pfl = flist;
  countE=0;
  for (i = 0; i < nflist; i++, pfl++)
    {
      ppf2D = pfl->pf2D;
      printf("%d ", i);
      if (ppf2D->type == ConicType_Ellipse)
	{
	  printf("E ");
	  countE++;
	}
      else
	{
	  printf("U ");
	}
      printf ("%lg %lg %lg %lg %lg %lg %lg %lg ",
	      ppf2D->center[0],
	      ppf2D->center[1],
	      ppf2D->ev[0][0],
	      ppf2D->ev[0][1],
	      ppf2D->ev[1][0],
	      ppf2D->ev[1][1],
	      ppf2D->axis[0],
	      ppf2D->axis[1]);
      printf ("%d ", ppf2D->arclist.n);
      parc = ppf2D->arclist.arc;
      for (j = 0; j < ppf2D->arclist.n; j++, parc++)
	{
	  ntrack = parc->ntrack;
	  printf("%d ", ntrack);
	  ptrack = &parc->f2Ds->track[ntrack];
	  printf("%d %d %d %d %d %d ",
		 mod_nPoint(parc->start, ptrack->nPoint),
		 ptrack->Point[mod_nPoint(parc->start, ptrack->nPoint)*2],
		 ptrack->Point[mod_nPoint(parc->start, ptrack->nPoint)*2+1],
		 mod_nPoint(parc->goal, ptrack->nPoint),
		 ptrack->Point[mod_nPoint(parc->goal, ptrack->nPoint)*2],
		 ptrack->Point[mod_nPoint(parc->goal, ptrack->nPoint)*2+1]);
	}
      printf("\n");
    }
  printf("#E=%d U=%d\n", countE, nflist-countE);

  return;
}
#endif

static int
compress_ellipse(Features2D_old	*f2Ds,
		 ParamEllipseIW	*paramEIW)
{
  int	n_ellipse;
  Feature_List	*flist;
  int	iFeature, jFeature, iArc;
  EllipseArc	*parc;

  // flist のセット
  flist = (Feature_List *)calloc(f2Ds->nFeature, sizeof(Feature_List));
  if(flist == NULL)
    {
      return -1;
    }

  for(iFeature = 0; iFeature < f2Ds->nFeature; iFeature++)
    {
      flist[iFeature].pf2D = &f2Ds->feature[iFeature];
    }

  // flist[*].np のセット
  for(iFeature = 0; iFeature < f2Ds->nFeature; iFeature++){
    flist[iFeature].np = 0;
    if(flist[iFeature].pf2D->type == ConicType_Ellipse)
      {
	// ConicType_Ellipse でないものは np が0になる
	parc = flist[iFeature].pf2D->arclist.arc;
	for(iArc = 0; iArc < flist[iFeature].pf2D->arclist.n; iArc++, parc++)
	  {
	    flist[iFeature].np += (parc->goal - parc->start);
	  }
      }
  }

  // sort 上位に点数が多いものがくるように ConicType_Ellipse でないものは自動的に下位になる
  qsort(flist, f2Ds->nFeature, sizeof(Feature_List), comp_flist);

  // ConicType_Ellipse のカウント
  n_ellipse = 0;
  while(n_ellipse < f2Ds->nFeature &&
	flist[n_ellipse].pf2D->type == ConicType_Ellipse)
    {
      n_ellipse++;
    }

  // 二重ループ上位のfeatureと下位のfeatureの点が互換であれば、下位のもののフラグを変更する
  for(iFeature = 0; iFeature < n_ellipse; iFeature++)
    {
      if(flist[iFeature].pf2D->type == ConicType_Ellipse)
	{
	  // すでに書き換わっている可能性がある
	  for(jFeature = iFeature+1; jFeature < n_ellipse; jFeature++)
	    {
	      if(flist[jFeature].pf2D->type == ConicType_Ellipse)
		{
		  // すでに書き換わっている可能性がある
		  if(check_conic_crossmatch(flist[iFeature].pf2D,
					    flist[jFeature].pf2D,
					    paramEIW)
		     == CONIC_MATCH_OK)
		    {
		      flist[jFeature].pf2D->type = ConicType_Unknown;
		    }
		}
	    }
	}
    }

  free(flist);

  return 0;
}

// 同一のtrackに存在する楕円feature同士で完全に包含関係にあるものを探し、
// 含まれてしまうfeatureをUnknownにする
static void
check_overlap_feature (Features2D_old	*features,
		       int		feature_from,
		       int		feature_to,
		       ConicType	type0)
{
  int	i_all;
  Feature2D_old	*pfi, *pfj;
  int	iFeature, jFeature;
  int	len;
  int	check_i;
  int	nPoint;
  int	lenj, leni;
  Feature2D_old	*spf, *lpf;
  int	slen, llen;
  int	offset;
  int	sstart, send;

  if (features->nAlloc == 0)
    {
      return;
    }

  //feature_to = features->nFeature;
  nPoint = features->feature[feature_from].all;

  // start を　0<=start<nPointになるように、goalを start+len-1にする
  pfi = &features->feature[feature_from];
  i_all = -1;
  for (iFeature = feature_from; iFeature < feature_to && i_all == -1;
       iFeature++, pfi++)
    {
      if (pfi->type == type0)
	{
	  len = pfi->end - pfi->start + 1;
	  pfi->start = mod_nPoint(pfi->start, nPoint);
	  pfi->end = pfi->start + len - 1;
	  if (len == pfi->all)
	    {
	      i_all = iFeature;
	    }
	}
    }

  // 全点を使った楕円特徴がある場合、他の楕円をすべて Unknown に
  if (i_all >= 0)
    {
      pfi = &features->feature[feature_from];
      for (iFeature = feature_from; iFeature < feature_to;
	   iFeature++, pfi++)
	{
	  if (pfi->type == type0 && iFeature != i_all)
	    {
	      pfi->type = ConicType_Unknown;
	    }
	}
      return;
    }

  // 各ペアについて包含関係にあるかどうかを調べ、含まれている方を Unknown に
  pfi = &features->feature[feature_from];
  for (iFeature = feature_from; iFeature < feature_to-1; iFeature++, pfi++)
    {
      check_i = 1;
      if (pfi->type == type0)
	{
	  leni = pfi->end - pfi->start + 1;
	  pfj = pfi+1;
	  for (jFeature = iFeature+1; jFeature < feature_to && check_i;
	       jFeature++, pfj++)
	    {
	      if (pfj->type == type0)
		{
		  // 長い方 lpf,llen, 短い方 spf,slen
		  lenj = pfj->end - pfj->start + 1;
		  if (leni > lenj)
		    {
		      spf = pfj;
		      slen = lenj;
		      lpf = pfi;
		      llen = leni;
		    }
		  else
		    {
		      spf = pfi;
		      slen = leni;
		      lpf = pfj;
		      llen = lenj;
		    }
		  // offset: lstart->0 にするための数字
		  offset = nPoint - mod_nPoint(lpf->start, nPoint);
		  sstart = mod_nPoint(spf->start+offset, nPoint);
		  send = sstart + slen - 1;
		  if (send < llen)
		    // sstart < send < llen は自明
		    {
		      spf->type = ConicType_Unknown;
		      if (spf == pfi)
			{
			  // iFeature とのペアはもう調べる必要がない
			  check_i = 0;
			}
		    }
		}
	    }
	}
    }

  return;
}

static double
tmpgett(Track	*ptrack,
	Feature2D_old	*pf,
	int	i0)
{
  int	i1;
  double	tangent[2];
  double	point[2];

  i1 = i0 % pf->all;
  norm2tangent(&pf->coef[3], tangent);

  point[0] = (double)ptrack->Point[i1*2];
  point[1] = (double)ptrack->Point[i1*2+1];

  return INPROD2D(point, tangent);
}
	

static void
debug_print_lineFeature(Features2D_old* lineFeatures)
{
  int	i;
  Feature2D_old	*pf;
  Track	*pt;
  double	mint, maxt, tmpt;
  int	jmint, jmaxt, jend, j;
  int	count;

  count = 0;
  pf = lineFeatures->feature;
  for(i = 0; i < lineFeatures->nFeature; i++, pf++)
    {
      if(pf->type == ConicType_Line){
	count++;
      }
    }
    

  printf("#nFeature = %d nline=%d\n", lineFeatures->nFeature, count);
  pf = lineFeatures->feature;
  for(i = 0; i < lineFeatures->nFeature; i++, pf++){
    printf("%d ",i);
    if(pf->type == ConicType_Line){
      printf("L ");
    }else{
      printf("U ");
    }
    printf("%d %d %d %d %g %g %g %g %g %g ",
	   pf->nTrack,
	   pf->all,
	   pf->start, pf->end,
	   pf->coef[3],
	   pf->coef[4],
	   //pf->center[0],
	   //pf->center[1],
	   pf->startSPoint[0],
	   pf->startSPoint[1],
	   pf->endSPoint[0],
	   pf->endSPoint[1]);
    pt = &lineFeatures->track[pf->nTrack];
    mint = maxt = tmpgett(pt, pf, pf->start);
    jmint = jmaxt = pf->start;
    if(pf->start >= pf->end){
      jend = pf->end + pf->all;
    }else{
      jend = pf->end;
    }
    for(j = pf->start+1; j <= jend; j++){
      tmpt = tmpgett(pt, pf, j);
      if(tmpt < mint){
	mint = tmpt;
	jmint = j;
      }else if(tmpt > maxt){
	maxt = tmpt;
	jmaxt = j;
      }
    }
    jmint = jmint % pf->all;
    printf("%d %d %d %g %d %d %d %g\n",
	   jmint,
	   pt->Point[jmint*2],
	   pt->Point[jmint*2+1],
	   mint,
	   jmaxt,
	   pt->Point[jmaxt*2],
	   pt->Point[jmaxt*2+1],
	   maxt);
  }
  return;
}

// qsort 用比較関数
#define RET_AB	(-1)
#define RET_BA	(1)
#define RET_UNDEF	(0)

static int
sort_linefeature(const void *pa,
		 const void *pb)
{
  Feature2D_old	*a,*b;

  a = (Feature2D_old *)pa;
  b = (Feature2D_old *)pb;

  if(a->type == ConicType_Unknown)
    {
      if(b->type == ConicType_Unknown)
	{
	  return RET_UNDEF;
	}
      return RET_BA;
    }

  if(b->type == ConicType_Unknown)
    {
      return RET_AB;
    }
  
  if(a->lineLength > b->lineLength)
    {
      return RET_AB;
    }
  else if(a->lineLength < b->lineLength)
    {
      return RET_BA;
    }

  return RET_UNDEF;
}

// 長い線分に短い線分の端点が閾値内に存在しており、媒介変数が重複していれば長い方にマージ
static void
merge_line_features(Features2D_old* lF2D,
		    ParametersFeature2D	*pmF2D)
{
  Feature2D_old	*pfi, *pfj;
  int	i, j, k;
  double	h, h0, h1;
  double	tvec[2];
  double	diff[2];
  double	tmp;
  double	ta0, ta1, tb0, tb1;
  double	zeroP[2];
  double	b0[2], b1[2];
  double	t0, t1;
  double	dx, dy;

  // 端点の計算しなおし
  pfi = lF2D->feature;
  for(i = 0; i < lF2D->nFeature; i++, pfi++)
    {
      if(pfi->type == ConicType_Line)
	{
	  //垂線の足の計算
	  h = -(pfi->coef[3]*pfi->startSPoint[0]+
		pfi->coef[4]*pfi->startSPoint[1])-pfi->coef[5];
	  for(k = 0; k < 2; k++)
	    {
	      pfi->startSPoint[k] += h*pfi->coef[3+k];
	    }
	  h = -(pfi->coef[3]*pfi->endSPoint[0]+
		pfi->coef[4]*pfi->endSPoint[1])-pfi->coef[5];
	  for(k = 0; k < 2; k++)
	    {
	      pfi->endSPoint[k] += h*pfi->coef[3+k];
	    }

	  // 接線方向の入れ替え
	  tvec[0] = pfi->coef[4];
	  tvec[1] = -pfi->coef[3];
	  for(k = 0; k < 2; k++)
	    {
	      diff[k] = pfi->endSPoint[k] - pfi->startSPoint[k];
	    }
	  if(tvec[0]*diff[0]+tvec[1]*diff[1] < 0.0)
	    {
	      // swap
	      for(k = 0; k < 2; k++)
		{
		  tmp = pfi->endSPoint[k];
		  pfi->endSPoint[k] = pfi->startSPoint[k];
		  pfi->startSPoint[k] = tmp;
		}
	    }

	  for(k = 0; k < 2; k++)
	    {
	      pfi->middleSPoint[k] = (pfi->startSPoint[k]+
				      pfi->endSPoint[k]) / 2.0;
	    }
	}
    }

  // ソートする。長いLine->短いLine->Unknown
  qsort(lF2D->feature, lF2D->nFeature,
	sizeof(Feature2D_old), sort_linefeature);

  pfi = lF2D->feature;
  for(i = 0; i < lF2D->nFeature; i++, pfi++)
    {
      if(pfi->type == ConicType_Line)
	{
	  // ta1　の計算、ta0 = 0.0
	  tvec[0] = pfi->coef[4];
	  tvec[1] = -pfi->coef[3];
	  for(k = 0; k < 2; k++)
	    {
	      diff[k] = pfi->endSPoint[k] - pfi->startSPoint[k];
	    }
	  ta0 = 0.0;
	  ta1 = tvec[0]*diff[0]+tvec[1]*diff[1];
	  for(k = 0; k < 2; k++)
	    {
	      zeroP[k] = pfi->startSPoint[k];
	    }
	  
	  pfj = pfi+1;
	  for(j = i+1; j < lF2D->nFeature; j++, pfj++)
	    {
	      if(pfj->type == ConicType_Line)
		{
		  // pfj の両端点のpfiへの垂線の足を計算
		  h0 = -(pfi->coef[3]*pfj->startSPoint[0]+
			pfi->coef[4]*pfj->startSPoint[1])-pfi->coef[5];
		  for(k = 0; k < 2; k++)
		    {
		      b0[k] = pfj->startSPoint[k] + h*pfi->coef[3+k];
		    }
		  h1 = -(pfi->coef[3]*pfj->endSPoint[0]+
			pfi->coef[4]*pfj->endSPoint[1])-pfi->coef[5];
		  for(k = 0; k < 2; k++)
		    {
		      b1[k] = pfj->startSPoint[k] + h*pfi->coef[3+k];
		    }

		  // どちらの垂線の長さも閾値以内
		  if(fabs(h0) < pmF2D->maxErrorofLineFit &&
		     fabs(h1) < pmF2D->maxErrorofLineFit)
		    {
		      // tb{01} の計算
		      for(k = 0; k < 2; k++)
			{
			  diff[k] = b0[k] - pfi->startSPoint[k];
			}
		      tb0 = tvec[0]*diff[0]+tvec[1]*diff[1];
		      for(k = 0; k < 2; k++)
			{
			  diff[k] = b1[k] - pfi->startSPoint[k];
			}
		      tb1 = tvec[0]*diff[0]+tvec[1]*diff[1];

		      // tb0 < tb1 にする
		      if(tb0 > tb1)
			{
			  tmp = tb0;
			  tb0 = tb1;
			  tb1 = tmp;
			}

		      // t の範囲比較
		      if(ta0 <= tb1 && tb0 <= ta1)
			{
			  // 重複あり
			  if(ta0 < tb0)
			    {
			      t0 = ta0;
			    }
			  else
			    {
			      t0 = tb0;
			    }
			  if(ta1 < tb1)
			    {
			      t1 = tb1;
			    }
			  else
			    {
			      t1 = ta1;
			    }

			  // pfiの端点を修正し、pfjを消す
			  for(k = 0; k < 2; k++)
			    {
			      pfi->startSPoint[k] = zeroP[k]+tvec[k]*t0;
			      pfi->endSPoint[k] = zeroP[k]+tvec[k]*t1;
			      pfi->middleSPoint[k] = (pfi->startSPoint[k]+
						      pfi->endSPoint[k])/2.0;
			    }
			  pfj->type = ConicType_Unknown;
			}
		    }
		}
	    }
	}
    }
  
  // 仕上げ
  
  pfi = lF2D->feature;
  for(i = 0; i < lF2D->nFeature; i++, pfi++)
    {
      if(pfi->type == ConicType_Line)
	{
	  dx = pfi->endSPoint[0] - pfi->startSPoint[0];
	  dy = pfi->endSPoint[1] - pfi->startSPoint[1];
	  pfi->lineLengthSQ = dx * dx + dy * dy;
	  pfi->lineLength = sqrt(pfi->lineLengthSQ);
	  pfi->direction[0] = dx;
	  pfi->direction[1] = dy;
	}
    }

  return;
}

// 直線特徴を整形する
// (1) 端点のはりかえ
// (2) マージ
static void
reform_line_features(Features2D_old* lineFeatures,
		     ParametersFeature2D	*pmF2D)
{
  Feature2D_old	*pfi;
  Track	*pt;
  int	iFeature;
  double	mint, maxt;
  int	jmint, jmaxt, tmp;
  int	jend;
  int	j;
  double	tmpt;
  int	middle;
  double	dx, dy;
  int	from, to;
  Feature2D_old	*pto;  

  // 端点のはりかえ
  pfi = lineFeatures->feature;
  for (iFeature = 0; iFeature < lineFeatures->nFeature; iFeature++, pfi++)
    {
      if (pfi->type == ConicType_Line)
	{
	  pt = &lineFeatures->track[pfi->nTrack];
	  mint = maxt = tmpgett(pt, pfi, pfi->start);

	  // 点列内で接線方向の最大最小点を探しなおし
	  jmint = jmaxt = pfi->start;
	  if(pfi->start >= pfi->end)
	    {
	      jend = pfi->end + pfi->all;
	    }
	  else
	    {
	      jend = pfi->end;
	    }

	  for(j = pfi->start+1; j <= jend; j++)
	    {
	      tmpt = tmpgett(pt, pfi, j);
	      if(tmpt < mint){
		mint = tmpt;
		jmint = j;
	      }
	      else if(tmpt > maxt)
		{
		  maxt = tmpt;
		  jmaxt = j;
		}
	    }

	  // 大小関係を調整
	  if (jmaxt < jmint)
	    {
	      // swap
	      tmp = jmaxt;
	      jmaxt = jmint;
	      jmint = tmp;
	    }

	  if (jmint != pfi->start || jmaxt != jend)
	    {
	      // 探し直した最大最小点がもとの始点終点とちがっている
	      if (jmaxt - jmint +1 < SKIP_LEN)
		{
		  // 直線フラグを消す
		  pfi->type = ConicType_Unknown;
		}
	      else
		{
		  pfi->start = mod_nPoint(jmint, pfi->all);
		  pfi->end = mod_nPoint(jmaxt, pfi->all);
		  pfi->startSPoint[0] = pt->Point[pfi->start*2];
		  pfi->startSPoint[1] = pt->Point[pfi->start*2+1];
		  pfi->endSPoint[0] = pt->Point[pfi->end*2];
		  pfi->endSPoint[1] = pt->Point[pfi->end*2+1];

		  middle = mod_nPoint((jmint+jmaxt)/2, pfi->all);
		  pfi->middleSPoint[0] = pt->Point[middle*2];
		  pfi->middleSPoint[1] = pt->Point[middle*2+1];

		  pfi->nPoints = jmaxt - jmint + 1;

		  dx = pfi->endSPoint[0] - pfi->startSPoint[0];
		  dy = pfi->endSPoint[1] - pfi->startSPoint[1];
		  pfi->lineLengthSQ = dx * dx + dy * dy;
		  pfi->direction[0] = dx;
		  pfi->direction[1] = dy;
		}
	    }

	  // lineLength はここではじめてセットされる
	  pfi->lineLength = sqrt(pfi->lineLengthSQ);
	}
    }

  // 重複した特徴のチェック
  pfi = lineFeatures->feature;
  for (iFeature = 0; iFeature < lineFeatures->nFeature; iFeature++, pfi++)
    {
      if (pfi->type == ConicType_Line)
	{
	  // 同じ nTrack を持つfeatureの範囲を調べる
	  from = to = iFeature;
	  pto = pfi;
	  while(to < lineFeatures->nFeature &&
		pto->nTrack == pfi->nTrack)
	    {
	      to++;
	      pto++;
	    }
	  
	  // from には pfi->nTrackの最初の番号、toには次のnTrackを持つ番号
	  check_overlap_feature(lineFeatures, from, to, ConicType_Line);

	  // 次のループではtoから調べ始める
	  iFeature = to;
	  pfi = pto;
	}
    }
      
  // 隣接した特徴のマージ
  merge_line_features(lineFeatures, pmF2D);

  return;
}

// 特徴点を取り出す
Features2D_old*
extractFeatures_old (unsigned char* edge,   // エッジ画像
		     Parameters parameters, // 全パラメータ
		     const int id,          // データを識別するためのインデックス
		     Features3D model)      // モデルの３次元特徴データ
{
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  int imgsize = parameters.imgsize;
  int no_search = parameters.feature2D.no_search_features;
  int i, ii, f, n, cx, cy, istart, iend;
  int* pt;
  int eflag;
  int iTrack;
  ParamEllipseIW* paramEIW = &parameters.paramEIW;
  Features2D_old* features = constructFeatures();          // 統合保存用
  if (features == NULL)
    {
      return NULL;
    }

  Features2D_old* lineFeatures = constructFeatures();      // 直線検出用
  Features2D_old* ellipseFeatures = constructFeatures();   // 楕円検出用

  Feature2D_old* thisFeature;
  Feature2D_old* tmpFeature;

  int	orig_nFeature = -1;
  int	feature_from;

  // 作業用の画像を作成する
  unsigned char* work = (unsigned char*) malloc((sizeof(unsigned char) * imgsize));
  if (lineFeatures == NULL || ellipseFeatures == NULL || work == NULL)
    {
      clearFeatures2Dpointer(&features);
      goto ending; // メモリ確保失敗
    }
  // 作業用画像にエッジ画像をコピーする
  memcpy(work, edge, sizeof(unsigned char) * imgsize);

  // 輪郭点列を抽出する
  if (extractTrackPoints(features, work, parameters) == -1)
    {
      clearFeatures2Dpointer(&features);
      goto ending; // メモリ確保失敗
    }

#if 0
  if (parameters.dbgimag)
    {
      // 輪郭点の保存
      if (drawTrackPoints(features, parameters, id))
        {
          // 画像メモリがとれなかった
          goto ending;
        }
    }
#endif

  // 輪郭点列をコピーする
  if (copyTrackPoints(lineFeatures, features, parameters) == -1)
    {
      clearFeatures2Dpointer(&features);
      goto ending; // メモリ確保失敗
    }

  // 第1ループ：直線を検出し、頂点を生成する
  for (iTrack = 0; iTrack < lineFeatures->nTrack; iTrack++)
    {
      if (searchLineFeatures(work, lineFeatures, iTrack, parameters) < 0)
        {
          clearFeatures2Dpointer(&features);
          goto ending; // メモリ確保失敗
        }
    }

  // 端点のはりかえ、マージ等
  //reform_line_features(lineFeatures, &parameters.feature2D);

  // それぞれの端点が近い線分に印をつける(error < 0.0)
  if (parameters.feature2D.max_distance_similar_line >= 0.0)
    {
      mark_similar_lines(lineFeatures, parameters.feature2D.max_distance_similar_line);
    }

  // 直線に当てはめられたエッジ情報を上書き保存する
  tmpFeature = lineFeatures->feature;
  for (f = 0; f < lineFeatures->nFeature; f++)
    {
      if (tmpFeature[f].type != ConicType_Line || tmpFeature[f].error < 0.0)
        {
          continue;
        }
      if (tmpFeature[f].lineLength < parameters.feature2D.max_length_delete_line)
        {
          eflag = 2;
        }
      else
        {
          eflag = 3;
        }
      if (tmpFeature[f].nTrack >= 0)
        {
          pt = lineFeatures->track[tmpFeature[f].nTrack].Point;
          istart = tmpFeature[f].start;
          iend = tmpFeature[f].end;
          if (istart > iend)
            {
              iend += tmpFeature[f].all;
            }
          for (i = istart; i < iend; i++)
            {
              ii = (i % tmpFeature[f].all) * 2;
              cx = pt[ii];
              cy = pt[ii + 1];
              if ((cx >= 0 && cx < colsize) && cy >= 0 && cy < rowsize)
                {
                  // 直線に当てはめられたエッジ
                  edge[cy * colsize + cx] = (unsigned char) eflag;
                }
            }
        }
    }

  // 端点のはりかえ、マージ等
  reform_line_features(lineFeatures, &parameters.feature2D);

  if (parameters.dbgimag)
    {
      // 直線検出結果カラー表示・保存
      if (drawDetectedLines(edge, lineFeatures, parameters, id))
        {
          // 画像メモリがとれなかった
          goto ending;
        }
    }

  if (model.numOfVertices > 0 && !(no_search & NO_SEARCH_VERTEX))
    {
      // 2直線の組み合わせで端点の距離が閾値以内でなす角度が閾値以上の場合に頂点を生成する
      /*
      if (Line2Vertex(lineFeatures, features, parameters) < 0)
        {
          clearFeatures2Dpointer(&features);
          goto ending; // メモリ確保失敗
	  }*/
      if (line_to_vertex(lineFeatures, features, &parameters) < 0)
        {
          clearFeatures2Dpointer(&features);
          goto ending; // メモリ確保失敗
        }

      if (parameters.dbgimag)
        {
          // 頂点特徴抽出結果カラー表示・保存
          if (drawDetectedVertices(features, parameters, id))
            {
              // 画像メモリがとれなかった
              goto ending;
            }
        }
    }

  if (model.numOfCircles > 0 && !(no_search & NO_SEARCH_ELLIPSE))
    {
      if (paramEIW->SwLineEllipse & ENABLE_ELLIPSE_WITH_LINE)
	{
	  // 第2ループ：楕円、双曲線を検出し、featuresに保存
	  for (iTrack = 0; iTrack < features->nTrack; iTrack++)
	    {
	      feature_from = features->nFeature;
	      if(searchEllipseIW(features, iTrack, paramEIW)
		 == SEARCH_FEATURES2_NG)
		{
		  clearFeatures2Dpointer(&features);
		  goto ending; // メモリ確保失敗
		}
	      check_overlap_feature(features, feature_from,
				    features->nFeature,
				    ConicType_Ellipse);
	    }

	  // 楕円に当てはめられたエッジ情報を保存する
	  tmpFeature = features->feature;
	  for (f = 0; f < features->nFeature; f++)
	    {
	      if (tmpFeature[f].type != ConicType_Ellipse)
		{
		  continue;
		}
	      if (tmpFeature[f].nTrack >= 0)
		{
		  pt = features->track[tmpFeature[f].nTrack].Point;
		  istart = tmpFeature[f].start;
		  iend = tmpFeature[f].end;
		  if (istart > iend)
		    {
		      iend += tmpFeature[f].all;
		    }
		  for (i = istart; i < iend; i++)
		    {
		      ii = ((i+tmpFeature[f].all) % tmpFeature[f].all) * 2;
		      cx = pt[ii];
		      cy = pt[ii + 1];
		      if ((cx >= 0 && cx < colsize) && cy >= 0 && cy < rowsize)
			{
			  n = cy * colsize + cx;
			  if (edge[n] == 2)
			    {       // 短い直線当てはめされたエッジのみ＝滑らかなエッジ
			      edge[n] = 4;  // 楕円に当てはめられたエッジ
			    }
			}
		    }
		}
	    }

	  // tracklist_0 の準備
	  if(setup_ellipse_arclist(features) == -1)
	    {
	      clearFeatures2Dpointer(&features);
	      goto ending; // メモリ確保失敗
	    }

	  // 新マージ関数
	  //fprintf(stderr, "nFeature=%d nTrack=%d\n",
	  //      features->nFeature, features->nTrack);
	  //system("date");
	  if(merge_ellipse(features, paramEIW)
	     == MERGE_ELLIPSE_NG)
	    {
	      clearFeatures2Dpointer(&features);
	      goto ending; // メモリ確保失敗
	    }
	  //fprintf(stderr, "nFeature=%d \n",
	  //      features->nFeature);
	  //system("date");

	  // 半径が短い楕円を排除 （typeにConicType_Unknown をセットする）
	  tmpFeature = features->feature;
	  for (f = 0; f < features->nFeature; f++)
	    {
	      if (tmpFeature[f].type == ConicType_Ellipse)
		{
		  if (tmpFeature[f].axis[0] < paramEIW->MinShortRadPost ||
		      tmpFeature[f].axis[1] < paramEIW->MinShortRadPost)
		    {
		      tmpFeature[f].type = ConicType_Unknown;
		    }
		}
	    }

	  if (paramEIW->SwOldMergeFunc == ENABLE_OLD_MERGE_FUNC)
	    {
	      // 楕円検出結果から、複数の楕円のデータを使って、楕円当てはめを行い、マージする
	      if (Ellipse2Ellipse(features, features, parameters) < 0)
		{
		  clearFeatures2Dpointer(&features);
		  goto ending; // メモリ確保失敗
		}
	      
	      // 楕円検出結果から、２つの楕円のデータを使って、楕円当てはめを行い、マージする
	      if (Ellipse2Ellipse2(features, features, parameters) < 0)
		{
		  clearFeatures2Dpointer(&features);
		  goto ending; // メモリ確保失敗
		}
	    }
	}

      if (paramEIW->SwLineEllipse & ENABLE_ELLIPSE_WITHOUT_LINE)
	{
	  // 長い直線の除去後に楕円のみ検出する
	  // 作業画像から長い直線を除去する
	  deleteLongLines(work, lineFeatures, parameters);

	  // 長い直線除去後の作業画像を再トレースし、楕円を検出する
	  if (extractTrackPoints(ellipseFeatures, work, parameters) == -1)
	    {
	      clearFeatures2Dpointer(&features);
	      goto ending; // メモリ確保失敗
	    }

	  // 第3ループ：楕円を検出し、ellipseFeaturesに保存
	  for (iTrack = 0; iTrack < ellipseFeatures->nTrack; iTrack++)
	    {
	      feature_from = features->nFeature;
	      if(searchEllipseIW(ellipseFeatures, iTrack, paramEIW)
		 == SEARCH_FEATURES2_NG)
		{
		  clearFeatures2Dpointer(&features);
		  goto ending; // メモリ確保失敗
		}
	      check_overlap_feature(features, feature_from,
				    features->nFeature,
				    ConicType_Ellipse);
	    }

	  // tracklist_1 の準備
	  if(setup_ellipse_arclist(ellipseFeatures) == -1)
	    {
	      clearFeatures2Dpointer(&features);
	      goto ending; // メモリ確保失敗
	    }

	  // 新マージ関数
	  //fprintf(stderr, "nFeature=%d nTrack=%d\n",
	  //      ellipseFeatures->nFeature, ellipseFeatures->nTrack);
	  //system("date");
	  if(merge_ellipse(ellipseFeatures, paramEIW)
	     == MERGE_ELLIPSE_NG)
	    {
	      clearFeatures2Dpointer(&ellipseFeatures);
	      goto ending; // メモリ確保失敗
	    }
	  //fprintf(stderr, "nFeature=%d \n",
	  //      ellipseFeatures->nFeature);
	  //system("date");

	  // 半径が短い楕円を排除 （typeにConicType_Unknown をセットする）
	  tmpFeature = ellipseFeatures->feature;
	  for (f = 0; f < ellipseFeatures->nFeature; f++)
	    {
	      if (tmpFeature[f].type == ConicType_Ellipse)
		{
		  if (tmpFeature[f].axis[0] < paramEIW->MinShortRadPost ||
		      tmpFeature[f].axis[1] < paramEIW->MinShortRadPost)
		    {
		      tmpFeature[f].type = ConicType_Unknown;
		    }
		}
	    }

	  // 楕円に当てはめられたエッジ情報を上書き保存する
	  tmpFeature = ellipseFeatures->feature;
	  for (f = 0; f < ellipseFeatures->nFeature; f++)
	    {
	      if (tmpFeature[f].type != ConicType_Ellipse)
		{
		  continue;
		}
	      if (tmpFeature[f].nTrack >= 0)
		{
		  pt = ellipseFeatures->track[tmpFeature[f].nTrack].Point;
		  istart = tmpFeature[f].start;
		  iend = tmpFeature[f].end;
		  if (istart > iend)
		    {
		      iend += tmpFeature[f].all;
		    }
		  for (i = istart; i < iend; i++)
		    {
		      ii = ((i+tmpFeature[f].all) % tmpFeature[f].all) * 2;
		      cx = pt[ii];
		      cy = pt[ii + 1];
		      if ((cx >= 0 && cx < colsize) && cy >= 0 && cy < rowsize)
			{
			  n = cy * colsize + cx;
			  if (edge[n] == 2)
			    {       // 短い直線当てはめされたエッジのみ＝滑らかなエッジ
			      edge[n] = 4;  // 楕円に当てはめられたエッジ
			    }
			}
		    }
		}
	    }
	  /*
	  // 楕円結果の重複除去
	  for (f = 0; f < ellipseFeatures->nFeature; f++)
	    {
	      thisFeature = &ellipseFeatures->feature[f];
	      if (thisFeature->type != ConicType_Ellipse)
		{
		  continue;
		}
	      double thr_coefdiff = 1.0E-3;
	      int iSame = 0, i, ii;
	      for (i = f + 1; i < ellipseFeatures->nFeature; i++)
		{
		  tmpFeature = &ellipseFeatures->feature[i];
		  if (tmpFeature->type != ConicType_Ellipse)
		    {
		      continue;
		    }
		  for (ii = 0; ii < 6; ii++)
		    {
		      if (fabs(tmpFeature->coef[ii] - thisFeature->coef[ii])
			  > thr_coefdiff)
			{
			  break;
			}
		    }
		  if (ii >= 6)
		    {
		      iSame = 1;
		      break;
		    }
		}
	      if (iSame)
		{
		  tmpFeature->type = ConicType_Unknown;
		}
	    }
	  */

	  // 楕円特徴をマージする
	  orig_nFeature = features->nFeature;
	  if (mergeFeatures(features, ellipseFeatures) < 0)
	    {
	      clearFeatures2Dpointer(&features);
	      goto ending; // メモリ確保失敗
	    }

	  if (paramEIW->SwOldMergeFunc == ENABLE_OLD_MERGE_FUNC)
	    {
	      // 楕円検出結果から、複数の楕円のデータを使って、楕円当てはめを行い、マージする
	      if (Ellipse2Ellipse(features, ellipseFeatures, parameters) < 0)
		{
		  clearFeatures2Dpointer(&features);
		  goto ending; // メモリ確保失敗
		}

	      // 楕円検出結果から、２つの楕円のデータを使って、楕円当てはめを行い、マージする
	      if (Ellipse2Ellipse2(features, ellipseFeatures, parameters) < 0)
		{
		  clearFeatures2Dpointer(&features);
		  goto ending; // メモリ確保失敗
		}
	    }
	}

      // 楕円検出結果から使用点数の少ない楕円を排除
      for (f = 0; f < features->nFeature; f++)
	{
          thisFeature = &features->feature[f];
	  if (thisFeature->type == ConicType_Ellipse)
	    {
	      if(thisFeature->end - thisFeature->start + 1
		 < paramEIW->PostMinLength)
		{
		  thisFeature->type = ConicType_Unknown;
		}
	    }
	}

      // 楕円結果の重複除去
      if(compress_ellipse(features, paramEIW) == -1)
	{
	  clearFeatures2Dpointer(&features);
	  goto ending; // メモリ確保失敗
	}
      /*
      for (f = 0; f < features->nFeature; f++)
        {
          thisFeature = &features->feature[f];
          if (thisFeature->type != ConicType_Ellipse)
            {
              continue;
            }
          double thr_coefdiff = 1.0E-3;
          int iSame = 0, i, ii;
          for (i = f + 1; i < features->nFeature; i++)
            {
              tmpFeature = &features->feature[i];
              if (tmpFeature->type != ConicType_Ellipse)
                {
                  continue;
                }
              for (ii = 0; ii < 6; ii++)
                {
                  if (fabs(tmpFeature->coef[ii] - thisFeature->coef[ii])
		      > thr_coefdiff)
                    {
                      break;
                    }
                }
              if (ii >= 6)
                {
                  iSame = 1;
                  break;
                }
            }
          if (iSame)
            {
              tmpFeature->type = ConicType_Unknown;
            }
        }
      */

      if (parameters.dbgimag)
        {
          // 楕円検出結果カラー表示・保存
          if (drawDetectedEllipses(edge, features, parameters, id))
            {
              // 画像メモリがとれなかった
              goto ending;
            }
        }

    }

 ending:

  // 楕円特徴データのメモリ解除
  destructFeatures(ellipseFeatures);
  // 直線特徴データのメモリ解除
  destructFeatures(lineFeatures);
  // 作業画像のメモリ解除
  free(work);

  return features;
}

// ステレオ画像の一枚から二次元特徴の抽出
Features2D_old*
ImageToFeature2D_old(unsigned char* src,    // 原画像
                     unsigned char* edge,   // エッジ画像
                     Parameters parameters, // 全パラメータ
                     const int id,          // データを識別するためのインデックス
                     Features3D model)      // モデルの３次元特徴データ（２次元処理の切り替え用）
{
  if (parameters.dbgimag)
    {
      // 入力確認画像の出力
      if (drawInputImage(src, parameters, id))
        {
          // 画像メモリがとれなかった
          return NULL;
        }
    }

  // エッジ点を抽出する
  if (extractEdge(edge, src, EEsearchedLarge, parameters))
    {
      // 画像メモリがとれなかった
      return NULL;
    }

  if (parameters.dbgimag)
    {
      // エッジ確認画像の出力
      if (drawEdgeImage(edge, parameters, id))
        {
          // 画像メモリがとれなかった
          return NULL;
        }
    }

  // 特徴点を抽出する
  Features2D_old* features = extractFeatures_old(edge, parameters, id, model);

  return features;
}
