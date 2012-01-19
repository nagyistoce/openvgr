/*
 findx.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
#include <stdio.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <getopt.h>

using namespace std;
using namespace cv;

// ２値化の際に使用する最大値
#define BINARIZEMAX 255

// パラメータデフォルト値
#define BTHRDEF 50
#define SEDGDEF 100
#define WEDGDEF 20
#define LMINDEF 5
#define GMAXDEF 3
#define SMINDEF 25
#define SMAXDEF 65

// オプションパラメータ構造体
typedef struct
{
  char* srcfile;   // 対象画像ファイル名
  char* tmpfile;   // テンプレート画像ファイル名
  char* outfile;   // 出力画像ファイル名
  char* basename;  // 中間結果出力ファイル・ベース名
  int   bthr;      // ２値化閾値
  int   sedg;      // 強いエッジ閾値
  int   wedg;      // 弱いエッジ閾値
  int   lmin;      // 検出する直線長さ最小値
  int   gmax;      // 検出する直線途切れ長さ最大値
  int   smin;      // 検出する直線の傾き絶対値最小値
  int   smax;      // 検出する直線の傾き絶対値最大値
  bool  nosp;      // サブピクセル補正無フラグ
  bool  xout;      // マッチ箇所２値化画像出力フラグ
  bool  eout;      // マッチ箇所エッジ画像出力フラグ
  bool  lout;      // 直線検出画像出力フラグ
  bool  sout;      // 検出直線傾き出力フラグ
  bool  disp;      // 検出結果ウインドウ表示フラグ
  bool  help;      // ヘルプ出力フラグ
} OptParam;

int parseOption( int argc, char** argv, OptParam& opt )
{
  int ch;

  enum _optkey
  {
    DMY=200,
    SRCF,
    TEMF,
    OUTF,
    BTHR,
    SEDG,
    WEDG,
    LMIN,
    GMAX,
    SMIN,
    SMAX,
    NOSP,
    XOUT,
    EOUT,
    LOUT,
    SOUT,
    XELO,
    XELS,
    DISP,
    HELP,
    END
  };

  static struct option long_options[] =
  {
    {"i",     required_argument,  0, SRCF},
    {"t",     required_argument,  0, TEMF},
    {"o",     required_argument,  0, OUTF},
    {"h",     no_argument,        0, HELP},
    {"bthr",  required_argument,  0, BTHR},
    {"sedg",  required_argument,  0, SEDG},
    {"wedg",  required_argument,  0, WEDG},
    {"lmin",  required_argument,  0, LMIN},
    {"gmax",  required_argument,  0, GMAX},
    {"smin",  required_argument,  0, SMIN},
    {"smax",  required_argument,  0, SMAX},
    {"nosp",  no_argument,        0, NOSP},
    {"xout",  no_argument,        0, XOUT},
    {"eout",  no_argument,        0, EOUT},
    {"lout",  no_argument,        0, LOUT},
    {"sout",  no_argument,        0, SOUT},
    {"xelo",  no_argument,        0, XELO},
    {"xels",  no_argument,        0, XELS},
    {"disp",  no_argument,        0, DISP},
    {0, 0, 0, 0}
  };

  // getopt_long がオプションインデックスを格納する変数
  int option_index = 0;

  while ( true )
    {
      ch = getopt_long_only( argc, argv, "",
                             (const struct option *)&long_options,
                             &option_index );
      if ( ch == -1 )
        {
          break;
        }

      switch ( ch )
        {
          case SRCF: // 入力ファイル名（被検索画像）
            opt.srcfile = optarg;
            break;
          case TEMF: // 入力ファイル名（テンプレート）
            opt.tmpfile = optarg;
            break;
          case OUTF: // 出力画像ファイル名
            opt.outfile = optarg;
            break;
          case BTHR: // ２値化閾値
            opt.bthr = atoi( optarg );
            break;
          case SEDG: // 強いエッジ閾値
            opt.sedg = atoi( optarg );
            break;
          case WEDG: // 弱いエッジ閾値
            opt.wedg = atoi( optarg );
            break;
          case LMIN: // 直線最小値
            opt.lmin = atoi( optarg );
            break;
          case GMAX: // とぎれ間隔最大値
            opt.gmax = atoi( optarg );
            break;
          case SMIN: // 直線傾き最小値
            opt.smin = atoi( optarg );
            break;
          case SMAX: // 直線傾き最大値
            opt.smax = atoi( optarg );
            break;
          case NOSP: // サブピクセル補正無し出力
            opt.nosp = true;
            break;
          case XOUT: // マッチ箇所２値化画像出力
            opt.xout = true;
            break;
          case EOUT: // マッチ箇所エッジ画像出力
            opt.eout = true;
            break;
          case LOUT: // 直線検出画像出力
            opt.lout = true;
            break;
          case SOUT: // 検出直線傾き出力
            opt.sout = true;
            break;
          case XELO: // 検出関係全画像出力
            opt.xout = true;
            opt.eout = true;
            opt.lout = true;
            break;
          case XELS: // 検出関係全画像・直線傾き出力
            opt.xout = true;
            opt.eout = true;
            opt.lout = true;
            opt.sout = true;
            break;
          case DISP: // 検出結果ウインドウ表示
            opt.disp = true;
            break;
          case HELP: // ヘルプ
            opt.help = true;
            break;
          default:
            return -1;
        }
    }

  if ( opt.srcfile == NULL )
    {
      cerr << "被検索ファイルを指定してください．" << endl;
      return -1;
    }

  if ( opt.tmpfile == NULL )
    {
      cerr << "テンプレートファイルを指定してください．" << endl;
      return -1;
    }

  return 0;
}

void printUsage( char* argv[] )
{
  char defval[1024] = {0};
  char *progname = strrchr( argv[0], '/' );

  if ( progname == NULL )
    {
      progname = argv[0];
    }
  else
    {
      ++progname;
    }

  sprintf( defval, "  bthr:%d sedg:%d wedg:%d lmin:%d gmax:%d smin:%d smax:%d\n",
           BTHRDEF, SEDGDEF, WEDGDEF, LMINDEF, GMAXDEF, SMINDEF, SMAXDEF );

  string base = " -i sourceImage -t templateImage [-o outputImage]";
  string add1 = "       [-bthr #] [-sedg #] [-wedg #]";
  string add2 = "       [-lmin #] [-gmax #] [-smin #] [-smax #] [-nosp]";
  string add3 = "       [-disp] [-xout] [-eout] [-lout] [-sout] [-xelo] [-xels]";

  cerr << "Usage : " << progname << base << endl;
  cerr << "        " << add1 << endl;
  cerr << "        " << add2 << endl;
  cerr << "        " << add3 << endl;
  cerr << "Default value" << endl;
  cerr << defval << endl;
}

string getBasefilename( string filename )
{
  string basename;
  string::size_type dp = filename.find( "." );
  if ( dp == string::npos )
    {
      basename = filename.substr( 0, filename.length() );
    }
  else
    {
      basename = filename.substr( 0, dp );
    }
  return basename;
}

// ２つの直線パラメータから直線交点の座標を計算する
Point2d getCrossPosition( Vec4f line1, Vec4f line2 )
{
  double amat[2][2] = { {line1[1], -line1[0]}, {line2[1], -line2[0]} };
  double bmat[2][2] = {{0}};
  double cmat[2];
  double pmat[2] = {0};

  cmat[0] = line1[1] * line1[2] - line1[0] * line1[3];
  cmat[1] = line2[1] * line2[2] - line2[0] * line2[3];

  // 直線パラメータから連立方程式を作って解く
  CvMat Amat = cvMat( 2, 2, CV_64FC1, amat );
  CvMat Bmat = cvMat( 2, 2, CV_64FC1, bmat );
  CvMat Cmat = cvMat( 2, 1, CV_64FC1, cmat );
  CvMat Pmat = cvMat( 2, 1, CV_64FC1, pmat );

  cvInvert( &Amat, &Bmat );
  cvMatMul( &Bmat, &Cmat, &Pmat );

  // 逆行列がなかった場合は ( 0, 0 ) が返る
  Point2d cross( pmat[0], pmat[1] );

  return cross;
}

// 直線上で指定された x 座標に対する y 座標を求める
double getYpositionOnLine( Vec4f line, double x )
{
  double y = 0;
  // 直線の傾きが９０度の場合は y = 0 を返す
  if ( line[0] != 0.0 )
    {
      y = line[1]*( x - line[2] )/line[0] + line[3];
    }
  return y;
}

// 矩形の x 座標範囲で直線を描画する
void drawLineInsideRectX( Mat& image, Vec4f lineparam, Rect rect )
{
  double x, y;
  Point  sp, ep;
  Scalar color = CV_RGB( 0, 255, 0 );
  x = rect.x;
  y = floor( getYpositionOnLine( lineparam, x ) + 0.5 );
  sp = Point( x, y );
  x = rect.br().x;
  y = floor( getYpositionOnLine( lineparam, x ) + 0.5 );
  ep = Point( x, y );
  line( image, sp, ep, color );
}

// 線分の２点から傾きをもとめる
double getLineSlope( Point p1, Point p2 )
{
  Point2d diff = p1 - p2;
  double theta = atan2( diff.y, diff.x ) * 180 / M_PI;

  // 傾き角度の範囲を -90 <= theta <= 90 にする

  if ( theta >  90 )
    {
      theta -= 180;
    }

  if ( theta < -90 )
    {
      theta += 180;
    }

  return theta;
}

// クロスマーカー抽出画像からクロス線を検出する
int detectLinesForCross( Mat& crossImage, Vec4f& lp1, Vec4f& lp2, OptParam& opt )
{
  // エッジ画像メモリの確保
  Mat edgeImage( crossImage.size(), CV_8UC1 );
  // クロス線画像メモリの確保
  Mat lineImage( crossImage.size(), CV_8UC3 );

  // 抜き出した領域のエッジを抽出する
  Canny( crossImage, edgeImage, opt.sedg, opt.wedg );

  string basename = opt.basename;
  string savefile;

  if ( opt.eout )
    {
      savefile = basename + "-edge.bmp";
      imwrite( savefile, edgeImage );
    }

  // カラー描画のため色変換
  cvtColor( edgeImage, lineImage, CV_GRAY2RGB );

  vector<Vec4i> lines;

  // エッジ画像の直線を検出する
  HoughLinesP( edgeImage,
               lines,
               0.1,
               ( CV_PI / 180 ),
               2,
               opt.lmin, // 直線最小長
               opt.gmax  // とぎれ間隔最大長
             );

  vector<Point> points1;
  vector<Point> points2;
  size_t i, lnum = lines.size();

  // 線分描画色
  const Scalar Green = CV_RGB( 0, 255, 0 );

  for ( i = 0; i < lnum; i++ )
    {
      Vec4i segment = lines.at(i);
      Point sp1( segment[0], segment[1] );
      Point sp2( segment[2], segment[3] );

      double theta = getLineSlope( sp1, sp2 );

      if ( opt.sout )
        {
            cerr << "slope " << theta << endl;
        }

      double slope = fabs( theta );

      // 傾きの絶対値が指定範囲のものを処理対象とする
      if ( slope < opt.smin || slope > opt.smax )
        {
          continue;
        }

      // 傾きが正の線と負の線で点列を分離する
      if ( theta > 0 )
        {
          points1.push_back( sp1 );
          points1.push_back( sp2 );
        }
      else
        {
          points2.push_back( sp1 );
          points2.push_back( sp2 );
        }

      // 確認用画像に線分を書き込む
      line( lineImage, sp1, sp2, Green );
    }

  if ( opt.lout )
    {
      savefile = basename + "-line.bmp";
      imwrite( savefile, lineImage );
    }

  if ( points1.empty() || points2.empty() )
    {
      return -1;
    }

  // 点データ配列に対して Mat ヘッダをつくる
  Mat line1p( points1 );
  Mat line2p( points2 );

#if 0 // デバッグ用
  {
    CvScalar sc;
    CvMat hmat;
    int npoints;
    int i;

    npoints = points1.size();

    for ( i = 0; i < npoints; i++ )
      {
        hmat = line1p;
        sc = cvGet2D( &hmat, i, 0 );
      }

    npoints = points2.size();

    for ( i = 0; i < npoints; i++ )
      {
        hmat = line2p;
        sc = cvGet2D( &hmat, i, 0 );
      }
  }
#endif

  // 傾きが正と負の直線をそれぞれ点列あてはめによって検出する
  fitLine( line1p, lp1, CV_DIST_L2, 0, 0.01, 0.01 );
  fitLine( line2p, lp2, CV_DIST_L2, 0, 0.01, 0.01 );

  return 0;
}

int main( int argc, char** argv )
{
  OptParam opt = {0}; // パラメータ構造体

  // パラメータデフォルト値
  opt.bthr = BTHRDEF;
  opt.sedg = SEDGDEF;
  opt.wedg = WEDGDEF;
  opt.lmin = LMINDEF;
  opt.gmax = GMAXDEF;
  opt.smin = SMINDEF;
  opt.smax = SMAXDEF;

  if ( parseOption( argc, argv, opt ) != 0 )
    {
      printUsage( argv );
      return -1;
    }

  if ( opt.help )
    {
      printUsage( argv );
      return 0;
    }

  // 画像を読み込む
  Mat templateImage = imread( opt.tmpfile, 0 );

  if ( templateImage.empty() )
    {
      cerr << "テンプレート画像を読めません．" << endl;
      return -1;
    }

  Mat displayImage = imread( opt.srcfile, 1 );
  Mat sourceImage  = imread( opt.srcfile, 0 );

  if ( displayImage.empty() || sourceImage.empty() )
    {
      cerr << "入力画像を読めません．" << endl;
      return -1;
    }

  // 元画像を2値化した画像の領域確保
  Mat binarizedSource( sourceImage.size(), sourceImage.type() );
  // テンプレート画像を2値化した画像の領域確保
  Mat binarizedTemplate( templateImage.size(), templateImage.type() );
  // 相違度マップ画像の領域確保
  Size mapsize = sourceImage.size() - templateImage.size() + Size( 1, 1 );
  Mat differenceMapImage( mapsize, CV_32FC1 );

  // グレースケールから2値に変換する
  threshold( sourceImage, binarizedSource, opt.bthr, BINARIZEMAX, CV_THRESH_BINARY );
  threshold( templateImage, binarizedTemplate, opt.bthr, BINARIZEMAX, CV_THRESH_BINARY );

  // テンプレートマッチングを行う
  matchTemplate( binarizedSource, binarizedTemplate, differenceMapImage, CV_TM_SQDIFF );

  // テンプレートがマッチした位置を得る
  Point minLocation; // 相違度が最小になる場所
  minMaxLoc( differenceMapImage, NULL, NULL, &minLocation );

  // マッチ位置を矩形として表す
  Rect rect( minLocation, templateImage.size() );

  // マッチ位置の矩形を表示用画像に描く
  const Scalar Red = CV_RGB( 255, 0, 0 );
  rectangle( displayImage, rect.tl(), rect.br(), Red );

  Mat extrImage( binarizedSource.size(), binarizedSource.type() );
  Mat backImage = binarizedSource.clone();
  // マッチ領域を塗りつぶしてマスクを作る
  Mat matchRect = backImage( rect );
  matchRect = Scalar(0);
  // マッチ領域外を黒にする
  extrImage = binarizedSource - backImage;

  string basename;
  string savefile;

  if ( opt.outfile )
    {
      basename = getBasefilename( opt.outfile );
    }
  else
    {
      basename = getBasefilename( opt.srcfile );
    }

  opt.basename = (char*)basename.c_str();

  if ( opt.xout )
    {
      savefile = basename + "-extr.bmp";
      imwrite( savefile, extrImage );
    }

  // クロス線の検出を行う
  Vec4f line1, line2;
  int err = detectLinesForCross( extrImage, line1, line2, opt );
  if ( err )
    {
      cerr << opt.srcfile << " : クロス線の検出に失敗しました．" << endl;
      return -1;
    }

  // 検出した直線をテンプレート矩形の x 座標範囲で描く
  drawLineInsideRectX( displayImage, line1, rect );
  drawLineInsideRectX( displayImage, line2, rect );

  // 直線の交点を計算してクロス点とする
  Point2d xp = getCrossPosition( line1, line2 );

  // 通常はサブピクセルコーナー検出でクロス点をできるだけ正確な位置にする
  if ( opt.nosp == false )
    {
      vector<Point2f> corner;
      corner.push_back( xp );
      cornerSubPix( sourceImage, corner, Size(5, 5), Size(-1, -1),
                    TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 15, 0.01) );
      xp = corner.front();
    }

  cout << fixed << xp.x << " ";
  cout << fixed << xp.y << endl;

  // クロス点座標を描画用に整数化する
  Point xpi;
  xpi.x = (int)floor(xp.x+0.5);
  xpi.y = (int)floor(xp.y+0.5);

  // クロス点を描画する 線幅 3
  const Scalar Yellow = CV_RGB( 255, 255, 0 );
  line( displayImage, xpi, xpi, Yellow, 3 );

  string outname;

  if ( opt.outfile )
    {
      outname = opt.outfile;
      // 結果画像を保存する
      imwrite( outname, displayImage );
    }

  if ( opt.disp )
    {
      if ( outname == "" )
        {
          outname = "findx";
        }
      // 表示ウィンドウを生成する
      namedWindow( opt.tmpfile, CV_WINDOW_AUTOSIZE );
      namedWindow( outname, CV_WINDOW_AUTOSIZE );
      // 画像を表示する
      imshow( opt.tmpfile, templateImage );
      imshow( outname, displayImage );
      // キー入力を待つ
      waitKey( 0 );
    }

  return 0;
}
