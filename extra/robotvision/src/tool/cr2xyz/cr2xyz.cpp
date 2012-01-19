/*
 cr2xyz.cpp

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

// オプションパラメータ構造体
typedef struct
{
  char* calib;  // キャリブレーションファイル名
  int   c1;     // 使用カメラ番号１
  int   c2;     // 使用カメラ番号２
  float col1;   // ステレオ対応点１横座標
  float row1;   // ステレオ対応点１縦座標
  float col2;   // ステレオ対応点２横座標
  float row2;   // ステレオ対応点２縦座標
  bool  help;   // ヘルプ出力フラグ
} OptParam;

int parseOption( int argc, char** argv, OptParam& opt )
{
  int ch;

  enum _optkey
  {
    DMY=200,
    CALIB,
    C1,
    C2,
    HELP,
    END
  };

  static struct option long_options[] =
  {
    {"i",   required_argument,  0, CALIB},
    {"c1",  required_argument,  0, C1},
    {"c2",  required_argument,  0, C2},
    {"h",   no_argument,        0, HELP},
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
          case CALIB: // 入力ファイル名（キャリブレーションデータ）
            opt.calib = optarg;
            break;
          case C1: // 使用カメラ番号１（キャリブレーションデータの）
            opt.c1 = atoi(optarg);
            break;
          case C2: // 使用カメラ番号２（キャリブレーションデータの）
            opt.c2 = atoi(optarg);
            break;
          case HELP: // ヘルプ
            opt.help = true;
            return 0;
          default:
            return -1;
        }
    }

  if ( opt.calib == NULL )
    {
      cerr << "キャリブレーションファイルを指定してください．" << endl;
      return -1;
    }

  // ステレオ対応点の画素座標取得

  if ( (argc - optind) == 4 )
    {
      opt.col1 = atof( argv[optind++] );
      opt.row1 = atof( argv[optind++] );
      opt.col2 = atof( argv[optind++] );
      opt.row2 = atof( argv[optind]   );
    }
  else
    {
      cerr << "画素座標を指定してください．" << endl;
      return -1;
    }

  return 0;
}

void printUsage( char* argv[] )
{
  char *progname = strrchr( argv[0], '/' );

  if ( progname == NULL )
    {
      progname = argv[0];
    }
  else
    {
      ++progname;
    }

  string base = " -i calib [-c1 #] [-c2 #] col1 row1 col2 row2";

  cerr << "Usage : " << progname << base << endl;
}

// キャリブレーションデータ（multicalib 出力）の YAML ファイルを読み込む
int readCalib( char* filename, Mat intr[3], Mat dist[3], Mat extr[3] )
{
  char str[32];
  int ncamera = 0;

  FileStorage cvfs( filename, CV_STORAGE_READ );

  FileNode node( cvfs.fs, NULL );

  ncamera = node["num_cameras"];

  for ( int i = 0; i < ncamera; i++ )
    {
      snprintf( str, sizeof(str), "camera%d_intr", i );
      read( node[(const char *)str], intr[i] );
      snprintf( str, sizeof(str), "camera%d_dist", i );
      read( node[(const char *)str], dist[i] );
      snprintf( str, sizeof(str), "camera%d_ext",  i );
      read( node[(const char *)str], extr[i] );
    }

  return ncamera;
}

void printMatrix( Mat matrix )
{
  for ( int i = 0; i < matrix.rows; i++ )
    {
      for ( int j = 0; j < matrix.cols; j++ )
        {
          cout << matrix.at<double>(i,j) << " ";
        }
      cout << endl;
    }
  cout << endl;
}

// 連立方程式の行列を作る
void makeLinearEquation( Mat extr, Point2d cr, Mat& A, Mat& B )
{
  Mat C(2,4,CV_64FC1);
  // ピンホールカメラモデルの投影式より
  // x'= x/z から z x'- x = 0
  C.row(0) = cr.x * extr.row(2) - extr.row(0);
  // y'= y/z から z y'- y = 0
  C.row(1) = cr.y * extr.row(2) - extr.row(1);
  // C の１列目から３列目を係数項行列とする
  A = C( Range(0,2), Range(0,3) );
  // C の４列目を定数項行列とする
  B = -C.col(3);
}

// ステレオ対応点画素座標から３次元座標を復元する
Point3d cr2xyz( int c1, int c2, Mat intr[3], Mat extr[3], Mat dist[3], Point2f p1, Point2f p2 )
{
  // ステレオ左カメラの画素座標
  Mat org1(1,1,CV_32FC2,&p1); // CV_32FC2 でないと受け付けてくれない
  // ステレオ右カメラの画素座標
  Mat org2(1,1,CV_32FC2,&p2); // CV_32FC2 でないと受け付けてくれない
  vector<Point2f> und1(1);
  vector<Point2f> und2(1);
  Mat Awork(2,3,CV_64FC1);
  Mat Bwork(2,1,CV_64FC1);
  Mat A(4,3,CV_64FC1);
  Mat B(4,1,CV_64FC1);
  Mat X(3,1,CV_64FC1);
  Mat RowMat;

  // レンズ歪補正を行って und1, und2 に理想的座標を得る
  undistortPoints( org1, und1, intr[c1], dist[c1] );
  undistortPoints( org2, und2, intr[c2], dist[c2] );

  // ステレオ左カメラの画素座標から３次元復元座標を得るための連立方程式をつくる
  makeLinearEquation( extr[c1], und1[0], Awork, Bwork );
  // 係数項行列
  RowMat = A.rowRange(0,2);
  Awork.copyTo(RowMat); // A の上２行にセット
  // 定数項行列
  RowMat = B.rowRange(0,2);
  Bwork.copyTo(RowMat); // B の上２行にセット

  // ステレオ右カメラの画素座標から３次元復元座標を得るための連立方程式をつくる
  makeLinearEquation( extr[c2], und2[0], Awork, Bwork );
  // 係数項行列
  RowMat = A.rowRange(2,4);
  Awork.copyTo(RowMat); // A の下２行にセット
  // 定数項行列
  RowMat = B.rowRange(2,4);
  Bwork.copyTo(RowMat); // B の下２行にセット

#if 0
  cout << "matrix A" << endl;
  printMatrix( A );
  cout << "matrix B" << endl;
  printMatrix( B );
#endif

  // 連立方程式を最小二乗法で解く（４行あるので冗長）
  solve( A, B, X, (DECOMP_SVD|DECOMP_NORMAL) );

#if 0
  cout << "matrix X" << endl;
  printMatrix( X );
#endif

  // 方程式解を３次元点座標としてセット
  Point3d xyz;
  xyz.x = X.at<double>(0,0);
  xyz.y = X.at<double>(0,1);
  xyz.z = X.at<double>(0,2);
  return xyz;
}

int main( int argc, char** argv )
{
  OptParam opt = {0}; // パラメータ構造体
  int c1, c2;

  // デフォルトのステレオカメラ番号は 0 と 1
  opt.c2 = 1;

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

  Mat intr[3], dist[3], extr[3];
  int ncamera;

  // キャリブレーションデータ（multicalib 出力）の YAML ファイルを読み込み
  // 内部パラメータ・外部パラメータ・歪み係数を取得する
  ncamera = readCalib( opt.calib, intr, dist, extr );

  // ステレオペアを構成するキャリブレーションデータ内カメラ番号
  c1 = opt.c1;
  c2 = opt.c2;

  if ( c1 < 0 || c1 > ncamera-1 )
    {
      cerr << "カメラ番号１が範囲外です．" << endl;
      return -1;
    }

  if ( c2 < 0 || c2 > ncamera-1 )
    {
      cerr << "カメラ番号２が範囲外です．" << endl;
      return -1;
    }

  if ( c1 == c2 )
    {
      cerr << "カメラ番号１と２が同じです．" << endl;
      return -1;
    }

  // ステレオ対応点データ
  Point2f p1( opt.col1, opt.row1 ), p2( opt.col2, opt.row2 );
  // ３次元座標
  Point3d xyz( 0, 0, 0 );

  // ３次元復元計算
  xyz = cr2xyz( c1, c2, intr, extr, dist, p1, p2 );

  // ３次元座標出力
  cout << fixed << xyz.x << " ";
  cout << fixed << xyz.y << " ";
  cout << fixed << xyz.z << endl;

  return 0;
}
