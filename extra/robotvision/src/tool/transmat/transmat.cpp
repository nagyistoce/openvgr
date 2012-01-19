/*
 transmat.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
#include <iostream>
#include <fstream>
#include "findmatrix.h"
#include "readdata.h"

#include <getopt.h>

using namespace std;

typedef struct
{
  char* srcdatfile;
  char* dstdatfile;
  char* mtxfile;
  bool  help;
} OptParam;

int parseOption( int argc, char** argv, OptParam& opt )
{
  int ch;

  while ( (ch=getopt(argc,argv,"i:j:o:h")) != -1 )
    {
      switch( ch )
        {
          case 'i': // 入力ファイル名（変換前座標）
            opt.srcdatfile = optarg;
            break;
          case 'j': // 入力ファイル名（変換後座標）
            opt.dstdatfile = optarg;
            break;
          case 'o': // 変換行列ファイル名
            opt.mtxfile = optarg;
            break;
          case 'h': // ヘルプ
            opt.help = true;
            return 0;
          default:
            return -1;
        }
    }

  if ( opt.srcdatfile == NULL || opt.dstdatfile == NULL )
    {
      cerr << "対応点データファイルを２つ指定してください" << endl;
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
  cerr << "Usage : " << progname << " -i points1 -j points2 [-o output]" << endl;
}

// 行列の出力
void printMatrix( ostream& ost, double matrix[4][4] )
{
  char strbuf[200] = {0};
  int idx, i, j;

  if ( matrix == NULL )
    {
      return;
    }

  ost << "4 4" << endl;

  for ( j = 0; j < 4; j++ )
    {
      idx = 0;
      for ( i = 0; i < 4; i++ )
        {
          idx += sprintf( &strbuf[idx], "% f ", matrix[j][i] );
        }
      ost << strbuf << endl;
    }
}

int main( int argc, char* argv[] )
{
  OptParam opt = {0};
  double matrix[4][4] = {{0}};

  if ( parseOption( argc, argv, opt ) != 0 )
    {
      cerr << "オプション指定に誤りがあります" << endl;
      printUsage( argv );
      return -1;
    }

  if ( opt.help )
    {
      printUsage( argv );
      return 0;
    }

  bool status;
  CvMat* before = NULL; // 変換前の点座標配列
  CvMat* after  = NULL; // 変換後の点座標配列
  int nBefPoints = 0;
  int nAftPoints = 0;
  int nPoints = 0;

  // 対応点データを読み込む. nBefPoints に読み込んだ対応点数が代入される.
  status = readPoints( opt.srcdatfile, &before, nBefPoints );

  if ( status == false )
    {
      goto ending;
    }

  if ( nBefPoints < 3 )
    {
      cerr << "３点以上のデータが必要です [" << opt.srcdatfile << "]" << endl;
      goto ending;
    }

  // 対応点データを読み込む. nAftPoints に読み込んだ対応点数が代入される.
  status = readPoints( opt.dstdatfile, &after, nAftPoints );

  if ( status == false )
    {
      goto ending;
    }

  if ( nAftPoints < 3 )
    {
      cerr << "３点以上のデータが必要です [" << opt.dstdatfile << "]" << endl;
      goto ending;
    }

  // データ個数が異なる場合は点数を少ないほうに合わせる
  if ( nBefPoints <= nAftPoints )
    {
      nPoints = nBefPoints;
    }
  else
    {
      nPoints = nAftPoints;
    }

  if ( nBefPoints != nAftPoints )
    {
      cerr << "警告：２つのファイルにある対応点データの個数が異なっています" << endl;
    }

  double rotation[3][3], translation[3]; // 回転と平行移動行列

  // before 点列 から after 点列への変換行列を求める.
  findTransformationMatrixForPairedPoints( rotation, translation, before, after, NULL, nPoints );

  for ( int i = 0; i < 3; i++ )
    {
      for( int j = 0; j < 3; j++ )
        {
          matrix[i][j] = rotation[i][j];
        }
      matrix[i][3] = translation[i];
    }
  matrix[3][3] = 1;

  // 行列のテキストデータ出力
  if ( opt.mtxfile == NULL )
    {
      printMatrix( cout, matrix );
    }
  else
    {
      ofstream ofs( opt.mtxfile );
      if ( ofs )
        {
          printMatrix( ofs, matrix );
        }
      else
        {
          cerr << "ファイルオープンエラー ！ " << opt.mtxfile << endl;
        }
    }

  ending:
    cvReleaseMat( &before );
    cvReleaseMat( &after  );
  return 0;
}
