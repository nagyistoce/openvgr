/*
 debugutil.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file debugutil.cpp
 * @brief デバッグ用関数
 * @date \$Date::                            $
 */
#include <stdio.h>
#include <highgui.h>
#include "debugutil.h"
#include "vectorutil.h"
#include "visionErrorCode.h"
#include "modelpoints.h"
#include "circle.h"

static inline int
roundoff( double d )
{
  return (int)floor( d + 0.5 ); // 四捨五入
}

static void
drawBaseEdge(IplImage* image, const uchar* edge, int grayval)
{
  CvMat EdgeMask;
  cvInitMatHeader(&EdgeMask, image->height, image->width, CV_8UC1, (void *)edge);
  CvScalar color = CV_RGB(grayval, grayval, grayval);
  cvSet(image, color, &EdgeMask);
}

static IplImage*
createDebugImage(const Parameters& parameters, bool color=false, const uchar* edge=NULL, int grayval=255)
{
  CvSize imsize = cvSize(parameters.colsize, parameters.rowsize);
  IplImage* image;

  if (color)
    {
      image = cvCreateImage(imsize, IPL_DEPTH_8U, 3);
    }
  else
    {
      image = cvCreateImage(imsize, IPL_DEPTH_8U, 1);
    }

  if (image)
    {
      cvZero(image);
      // 背景エッジが指定されていたら描く
      if (edge)
        {
          drawBaseEdge(image, edge, grayval);
        }
    }

  return image;
}

static void 
outDebugImage(IplImage* image, const char* name, int id, int display)
{
  char title[PATH_MAX+1] = {0};
  const int extlen = 5; // 拡張子分の余裕
  snprintf(title, sizeof(title)-extlen, "%s%d", name, id);
  if (display)
    {
      cvNamedWindow(title, 1);
      cvShowImage(title, image);
      cvWaitKey(-1);
      cvDestroyWindow(title);
    }
  if (image->nChannels == 1)
    {
      strcat(title, ".pgm");
    }
  else
    {
      strcat(title, ".ppm");
    }
  cvSaveImage(title, image);
  cvReleaseImage(&image);
}

int
drawInputImage(const uchar* src, const Parameters& parameters)
{
  int id = parameters.feature2D.id;
  int disp = parameters.dbgdisp;
  IplImage* cvGrayImage = createDebugImage(parameters);
  if (cvGrayImage == NULL)
    {
      return VISION_MALLOC_ERROR;
    }
  memcpy(cvGrayImage->imageData, src, parameters.imgsize);
  // 入力画像表示・保存
  outDebugImage(cvGrayImage, "src", id, disp);
  return 0;
}

int
drawEdgeImage(const uchar* edge, const Parameters& parameters)
{
  int id = parameters.feature2D.id;
  int disp = parameters.dbgdisp;
  IplImage* cvGrayImage = createDebugImage(parameters);
  if (cvGrayImage == NULL)
    {
      return VISION_MALLOC_ERROR;
    }
  memcpy(cvGrayImage->imageData, edge, parameters.imgsize);
  cvConvertScaleAbs(cvGrayImage, cvGrayImage, 255, 0); // 0,1を0,255に変換
  // エッジ２値(0,1)画像表示・保存
  outDebugImage(cvGrayImage, "edge", id, disp);
  return 0;
}

int
drawDetectedLines(const uchar* edge, const Features2D_old* lineFeatures, const Parameters& parameters)
{  
  // 直線検出結果カラー表示・保存
  // 背景表示画像配列変数名：edge
  // 直線検出結果変数名：lineFeatures

  // 画像メモリを確保して背景エッジを白で描く
  bool color = true;
  IplImage* cvColorImage = createDebugImage(parameters, color, edge);

  if (cvColorImage == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  // 検出結果の描画
  int* pt = NULL;
  CvPoint  dpt;
  CvScalar red   = CV_RGB(255, 0, 0);
  CvScalar green = CV_RGB(0, 255, 0);
  CvScalar blue  = CV_RGB(0, 0, 255);
  Feature2D_old* tmpFeature = lineFeatures->feature;
  int i, ii, f, cx, cy, istart, iend;

  for (f = 0; f < lineFeatures->nFeature; f++)
    {
      if (tmpFeature[f].type != ConicType_Line)
        {
          continue;
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
              dpt = cvPoint(cx, cy);
              cvLine(cvColorImage, dpt, dpt, blue);
            }
        }
      // 始点表示
      istart = tmpFeature[f].start * 2;
      iend = tmpFeature[f].end * 2;
      cx = pt[istart];
      cy = pt[istart + 1];
      dpt = cvPoint(cx, cy);
      cvLine(cvColorImage, dpt, dpt, green);
      // 終点表示
      cx = pt[iend];
      cy = pt[iend + 1];
      dpt = cvPoint(cx, cy);
      cvLine(cvColorImage, dpt, dpt, red);
    }

  int id = parameters.feature2D.id;
  int disp = parameters.dbgdisp;
  outDebugImage(cvColorImage, "line", id, disp);
  return 0;
}

int
drawDetectedVertices(const Features2D_old* features, const Parameters& parameters)
{
  // 頂点特徴抽出結果カラー表示・保存
  // 背景表示画像なし
  // 頂点特徴抽出結果変数名：features

  bool color = true;
  IplImage* cvColorImage = createDebugImage(parameters, color);

  if (cvColorImage == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  Feature2D_old* tmpFeature = features->feature;
  CvPoint  pt1, pt2, pt3;
  CvScalar green = CV_RGB(0, 255, 0);
  CvScalar white = CV_RGB(255, 255, 255);

  for (int f = 0; f < features->nFeature; f++)
    {
      if (tmpFeature[f].type != ConicType_Hyperbola)
        {
          continue;
        }
      pt1.x = roundoff(tmpFeature[f].center[0]);
      pt1.y = roundoff(tmpFeature[f].center[1]);
      pt2.x = roundoff(tmpFeature[f].startSPoint[0]);
      pt2.y = roundoff(tmpFeature[f].startSPoint[1]);
      pt3.x = roundoff(tmpFeature[f].endSPoint[0]);
      pt3.y = roundoff(tmpFeature[f].endSPoint[1]);
      cvLine(cvColorImage, pt1, pt2, green);
      cvLine(cvColorImage, pt1, pt3, green);
      // 特徴点の描画
      cvLine(cvColorImage, pt1, pt1, white);
    }

  int id = parameters.feature2D.id;
  int disp = parameters.dbgdisp;
  outDebugImage(cvColorImage, "Line2Vertex", id, disp);
  return 0;
}

int
drawTrackPoints(const Features2D_old* features, const Parameters& parameters)
{  
  int i, f, n, cx, cy;
  int* pt = NULL;
  int id = parameters.feature2D.id;
  int disp = parameters.dbgdisp;
  int colsize = parameters.colsize;
  int rowsize = parameters.rowsize;
  IplImage* cvTrackImage = createDebugImage(parameters);
  if (cvTrackImage == NULL)
    {
      return VISION_MALLOC_ERROR;
    }
  for (f = 0; f < features->nTrack; f++)
    {
      pt = features->track[f].Point;
      for (i = 0; i < features->track[f].nPoint; i++)
        {
          cx = pt[i * 2];
          cy = pt[i * 2 + 1];
          if ((cx >= 0 && cx < colsize) && (cy >= 0 && cy < rowsize))
            {
              n = (cy * colsize + cx);
              cvTrackImage->imageData[n] += 50;
            }
        }
    }
  outDebugImage(cvTrackImage, "track", id, disp);
  return 0;
}

// 楕円の係数と中心から形状を調べる
static void
getEllipseProperty(const double coef[6],   // 楕円係数（入力）
                   const double center[2], // 楕円中心（入力）
                   double rot[2][2],       // 楕円回転行列（出力）
                   double axis[2])         // 楕円の軸長（出力）
{
  double F;

  // 中心を移動した場合の F は
  F = (coef[0] * pow(center[0], 2) + coef[1] * center[0] * center[1]
       + coef[2] * pow(center[1], 2) + coef[3] * center[0] + coef[4] * center[1] + coef[5]);
  // これにより a(x+xc)^2 + b(x+xc)(y+yc) + c(y+yc)^2 + F = 0 の形になる

  // 各軸をx,y軸に一致させた場合のA, Cを求める
  double a[2][2] = { {coef[0], coef[1] / 2.0}, {coef[1] / 2.0, coef[2]} };
  double e[2], ev[2][2];
  double A, C;

  CvMat matA        = cvMat( 2, 2, CV_64FC1, a );
  CvMat eigenvalue  = cvMat( 2, 1, CV_64FC1, e );
  CvMat eigenvector = cvMat( 2, 2, CV_64FC1, ev );
  cvEigenVV( &matA, &eigenvector, &eigenvalue );

  if (e[0] * e[1] < VISION_EPS) // 楕円でない場合
    {
      return;
    }

  // 固有ベクトルを転置して回転行列とする
  CvMat matR = cvMat( 2, 2, CV_64FC1, rot );
  cvTranspose( &eigenvector, &matR );

  A = e[0];
  C = e[1];

  // F による規格化
  if (F != 0.0)
    {
      A /= -(F);
      C /= -(F);
      // これで A x^2 + C y^2 = 1 の標準形
      if ((A <= 0.0) && (C <= 0.0))
        {                       // 虚円もしくは放物線
          axis[0] = 0.0;
          axis[1] = 0.0;
        }
      else
        {
          axis[0] = sqrt(1.0 / A);
          axis[1] = sqrt(1.0 / C);
        }
    }
  else
    {
      // これは A x^2 + C y^2 = 0 の形式
      axis[0] = 0.0;
      axis[1] = 0.0;
    }
  return;
}

int
drawDetectedEllipses(const uchar* edge, const Features2D_old* features, const Parameters& parameters)
{
  // 楕円検出結果カラー表示・保存
  // 背景表示画像配列変数名：edge

  // 画像メモリを確保して背景エッジを白で描く
  bool color = true;
  IplImage* cvColorImage = createDebugImage(parameters, color, edge);

  const double ellipse_axis_max = 10000.0; // 表示する楕円の軸の長さの最大値

  if (cvColorImage == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  // 検出結果の描画
  Feature2D_old* tmpFeature;
  int f, cx, cy;

  for (f = 0; f < features->nFeature; f ++)
    {
      tmpFeature = &features->feature[f];
      if (tmpFeature->type != ConicType_Ellipse)
        {
          continue;
        }
      cx = roundoff(tmpFeature->center[0]);
      cy = roundoff(tmpFeature->center[1]);
      // 楕円中心の描画
      CvPoint  pt  = cvPoint(cx, cy);
      CvScalar red = CV_RGB(255, 0, 0);
      cvLine(cvColorImage, pt, pt, red);
      double *coef = tmpFeature->coef;
      double *cent = tmpFeature->center;
      double rot[2][2] = {{0}};
      double axis[2] = {0};
      // 楕円形状を得る
      getEllipseProperty( coef, cent, rot, axis );
      // 大きな楕円を描画する場合への対策
      if (axis[1] > ellipse_axis_max)
        {
          fprintf(stderr, "Warning: ellipse size error %f %f.\n", axis[0], axis[1]);
          continue;
        }

      pt = cvPoint(cent[0], cent[1]);
      // 左上原点なので角度の符号を反転させる（上下が逆）
      double angle = -atan2(rot[1][0], rot[0][0])*180/M_PI;
      CvSize axes  = cvSize(axis[0], axis[1]);
      CvScalar blue = CV_RGB(0, 0, 255);
      // 楕円の描画
      cvEllipse(cvColorImage, pt, axes, angle, 0, 360, blue);
    }

  int id = parameters.feature2D.id;
  int disp = parameters.dbgdisp;
  outDebugImage(cvColorImage, "ellipse", id, disp);
  return 0;
}

// CircleCandidate の描画
int
drawCircleCandidate(const uchar* edge,
                    const std::vector<CircleCandidate>& candidates,
                    int pairing,
                    const Parameters& parameters,
                    const CameraParam* cameraParam)
{
  // 画像メモリを確保して背景エッジを白で描く
  bool color = true;
  IplImage* cvColorImage = createDebugImage(parameters, color, edge);

  if (cvColorImage == NULL)
    {
      return VISION_MALLOC_ERROR;
    }

  int ndiv = 360; // 分割数
  double dst[3];  // 円上の点（開始は３次元円周上の一点）
  Data_2D iPos;
  int d;
  int ix, iy;
  CvPoint pt;
  CvScalar green = CV_RGB(0, 255, 0);
  CircleCandidate circle;
  size_t i;

  FILE *fp;
  char fname[100];
  sprintf(fname, "center%d.txt", pairing);
  fp = fopen(fname, "w");

  // 円の描画
  for (i = 0; i < candidates.size(); i++)
    {
      circle = candidates[i];

      // 円中心の描画
      projectXYZ2LR(&iPos, circle.center, (CameraParam*)cameraParam);
      ix = roundoff(iPos.col);
      iy = roundoff(iPos.row);
      pt = cvPoint(ix, iy);
      cvLine(cvColorImage, pt, pt, green);
      
      fprintf(fp, "%f %f %f\n", circle.center[0], circle.center[1], circle.center[2]);

      double axis[2][3];

      calc_3d_axes_of_circle(axis[0], axis[1], circle.normal, NULL);
      for (d = 0; d < ndiv; ++d)
        {
          int j;
          double theta = (double)d / (double)(ndiv - 1) * M_PI * 2.0;
          for (j = 0; j < 3; ++j)
            {
              dst[j] = circle.radius * (axis[0][j] * cos(theta) + axis[1][j] * sin(theta)) + circle.center[j];
            }

          projectXYZ2LR(&iPos, dst, (CameraParam*)cameraParam);

          ix = roundoff(iPos.col);
          iy = roundoff(iPos.row);
          pt = cvPoint(ix, iy);
          // 円周の描画
          cvLine(cvColorImage, pt, pt, green);
        }
    }

  fclose(fp);
  outDebugImage(cvColorImage, "circles3D", pairing, parameters.dbgdisp);

  return 0;
}

// Vertex の出力
int
printVertex(const std::vector< ::Vertex>& vertex)
{
  FILE* fp;
  size_t i;
  char filename[PATH_MAX+1] = {0};
  double psrc[4], pdst[4];
  cv::Mat src = cv::Mat(4, 1, CV_64FC1, psrc);
  cv::Mat dst = cv::Mat(4, 1, CV_64FC1, pdst);
  
  psrc[3] = 1.0;
  snprintf(filename, sizeof(filename), "vertex3D.txt");
  fp = fopen(filename, "w");
  if (fp == NULL)
    {
      return VISION_FILE_OPEN_ERROR;
    }
  else 
    {
      fprintf(fp, "%d 3\n", vertex.size() * 4);
      for (i = 0; i < vertex.size(); i++)
        {
          cv::Mat T = cv::Mat(4, 4, CV_64FC1, const_cast<double(*)[4]>(vertex[i].tPose));
          fprintf(fp, "%f %f %f\n", 
                  vertex[i].tPose[3][0], vertex[i].tPose[3][1], vertex[i].tPose[3][2]);
          psrc[0] = vertex[i].endpoint1[0];
          psrc[1] = vertex[i].endpoint1[1];
          psrc[2] = vertex[i].endpoint1[2];
          psrc[3] = 1.0;
          dst = T.t() * src;
          fprintf(fp, "%f %f %f\n", pdst[0], pdst[1], pdst[2]);
          fprintf(fp, "%f %f %f\n", 
                  vertex[i].tPose[3][0], vertex[i].tPose[3][1], vertex[i].tPose[3][2]);
          psrc[0] = vertex[i].endpoint2[0];
          psrc[1] = vertex[i].endpoint2[1];
          psrc[2] = vertex[i].endpoint2[2];
          psrc[3] = 1.0;
          dst = T.t() * src;
          fprintf(fp, "%f %f %f\n", pdst[0], pdst[1], pdst[2]);
        }
    }

  fclose(fp);

  return 0;
}

