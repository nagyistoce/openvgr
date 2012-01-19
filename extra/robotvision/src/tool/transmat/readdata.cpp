/*
 readdata.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
#include <iostream>
#include <fstream>
#include <string>

#include "readdata.h"

using namespace std;

// 文字列が数値なら true を返す
bool isNumber( string word )
{
  string::size_type dp;
  string target = word;

  // 引数文字列にスペースが含まれないことが前提

  // 符号を削除しておく
  dp = target.find( "-" );
  if ( dp != string::npos )
    {
      target.erase( dp, 1 );
    }
  // 符号を削除しておく
  dp = target.find( "+" );
  if ( dp != string::npos )
    {
      target.erase( dp, 1 );
    }
  // 小数点を削除しておく
  dp = target.find( "." );
  if ( dp != string::npos )
    {
      target.erase( dp, 1 );
    }
  // 指数記号を削除しておく
  dp = target.find( "e" );
  if ( dp != string::npos )
    {
      target.erase( dp, 2 );
    }
  // 指数記号を削除しておく
  dp = target.find( "E" );
  if ( dp != string::npos )
    {
      target.erase( dp, 2 );
    }
  // 指数符号を削除しておく
  dp = target.find( "-" );
  if ( dp != string::npos )
    {
      target.erase( dp, 1 );
    }
  // 指数符号を削除しておく
  dp = target.find( "+" );
  if ( dp != string::npos )
    {
      target.erase( dp, 1 );
    }

  // 数字以外の文字があったら false を返す
  for ( size_t i = 0; i < target.length(); i++ )
    {
      if ( target[i] < '0' || target[i] > '9' )
        {
          return false;
        }
    }

  return true;
}

// 一行の文字列データから最初の数値データを取り出す
bool getNumber( string& line, string& word )
{
  size_t i;

  // 行頭スペース文字の削除
  for ( i = 0; i < line.length() && isspace(line.at(i)); i++ );
  line.erase( 0, i );

  // スペース文字の検索
  string::size_type dp = line.find( " " );

  if ( dp == string::npos )
    {
      // スペースが無いときは行全部を単語とする
      word = line.substr( 0, line.length() );
    }
  else
    {
      // スペースの前までを取り出す
      word = line.substr( 0, dp );
      // 見つけたスペースまでを行データから消去
      line.erase( 0, dp+1 );
    }

  // 取り出した単語が数値であるか確認する
  // 数値でないときは false を返す
  return isNumber( word );
}

// ３次元座標データの取得
bool get3Ddata( string& line, double data[3] )
{
  string word;
  for ( int i = 0; i < 3; i++ )
    {
      // 文字列の数値部分を取り出す
      if ( !getNumber( line, word ) )
        {
          return false;
        }
      // 数値データに変換する
      data[i] = atof( word.c_str() );
    }
  return true;
}

bool readPairedPoints( string line, double p1[3], double p2[3] )
{
  // 点座標１の取得
  if ( !get3Ddata( line, p1 ) )
    {
      return false;
    }
  // 点座標２の取得
  if ( !get3Ddata( line, p2 ) )
    {
      return false;
    }
  return true;
}

bool readRowColHeader( string line, int& row, int& col )
{
  string word;
  int data[2];

  for ( int i = 0; i < 2; i++ )
    {
      // 文字列の数値部分を取り出す
      if ( !getNumber( line, word ) )
        {
          return false;
        }
      // 数値データに変換する
      data[i] = atoi( word.c_str() );
      // 行列数は正であること
      if ( data[i] <= 0 )
        {
          return false;
        }
    }
  row = data[0];
  col = data[1];
  return true;
}

// 対象となる行かどうかをチェックする
inline bool isIntendedLine( string line )
{
  // 空行またはコメント行でないこと
  // コメントは行の先頭が # か ; で始まるものとする
  if ( line.length() > 0 && line[0] != '#' && line[0] != ';' )
    {
      return true;
    }
  else
    {
      return false;
    }
}

bool readPoints( char* filename, CvMat** points, int& nPoints )
{
  ifstream ifs;
  string line;
  string word;
  int col = 0;
  int row = 0;
  bool status;
  double xyz[3];
  int i, j;

  ifs.open( filename );

  if ( ifs )
    {
      // ファイル先頭に書かれているはずの行数と列数を調べる
      try
        {
          status = false;
          while ( getline( ifs, line ) )
            {
              if ( isIntendedLine( line ) )
                {
                  status = readRowColHeader( line, row, col );
                  if ( status )
                    {
                      break;
                    }
                  cerr << "行列数読み込みエラー　！ [" << filename << "]" << endl;
                  return false;
                }
            }
        }
      catch ( ... )
        {
          cerr << "ファイル読み込みエラー　！ [" << filename << "]" << endl;
          return false;
        }

      // このデータファイルについては３列でなければならない
      if ( col != 3 )
        {
          cerr << "列数エラー　！ [" << filename << "]" << endl;
          return false;
        }

      try
        {
          *points = cvCreateMat( row, 3, CV_64FC1 );
        }
      catch ( ... )
        {
          cerr << "メモリ確保エラー　！ [" << filename << "]" << endl;
          goto errorEnding;
        }

      try
        {
          j = 0;
          cvSetZero( *points );
          while ( getline( ifs, line ) )
            {
              if ( isIntendedLine( line ) )
                {
                  if ( get3Ddata( line, xyz ) )
                    {
                      if ( j < row )
                        {
                          for ( i = 0; i < 3; i++ )
                            {
                              cvmSet( *points, j, i, xyz[i] );
                            }
                          ++j;
                        }
                      else
                        {
                          cerr << "警告：ヘッダ情報よりデータ行数が多い [" << filename << "]" << endl;
                          cerr << "超過分は無視されます" << endl;
                          break;
                        }
                    }
                  else
                    {
                      cerr << "データエラー　！ [" << filename << "]" << endl;
                      goto errorEnding;
                    }
                }
            }
          if ( j < row )
            {
              cerr << "警告：ヘッダ情報よりデータ行数が少ない [" << filename << "]" << endl;
              nPoints = j;
            }
          else
            {
              nPoints = row;
            }
        }
      catch ( ... )
        {
          cerr << "ファイル読み込みエラー　！ [" << filename << "]" << endl;
          goto errorEnding;
        }
    }
  else
    {
      cerr << "ファイルオープンエラー　！ [" << filename << "]" << endl;
      return false;
    }

  return true;

  errorEnding:
    cvReleaseMat( points );
    return false;
}

bool readMatrix( char* filename, CvMat** matrix )
{
  ifstream ifs;
  string line;
  string word;
  int col = 0;
  int row = 0;
  bool status;
  double data;
  int i, j;

  ifs.open( filename );

  if ( ifs )
    {
      // ファイル先頭に書かれているはずの行数と列数を調べる
      try
        {
          status = false;
          while ( getline( ifs, line ) )
            {
              if ( isIntendedLine( line ) )
                {
                  status = readRowColHeader( line, row, col );
                  if ( status )
                    {
                      break;
                    }
                  cerr << "行列数読み込みエラー　！ [" << filename << "]" << endl;
                  return false;
                }
            }
        }
      catch ( ... )
        {
          cerr << "ファイル読み込みエラー　！ [" << filename << "]" << endl;
          return false;
        }

      if ( row <= 0 || col <= 0 )
        {
          cerr << "行数・列数エラー　！ [" << filename << "]" << endl;
          return false;
        }

      try
        {
          *matrix = cvCreateMat( row, col, CV_64FC1 );
        }
      catch ( ... )
        {
          cerr << "メモリ確保エラー　！ [" << filename << "]" << endl;
          goto errorEnding;
        }

      try
        {
          j = 0;
          cvSetZero( *matrix );

          for ( j = 0; j < row; )
            {
              if ( !getline( ifs, line ) )
                {
                  cerr << "データエラー　！ [" << filename << "]" << endl;
                  goto errorEnding;
                }
              if ( isIntendedLine( line ) )
                {
                  string word;
                  for ( i = 0; i < col; i++ )
                    {
                      // 文字列の数値部分を取り出す
                      if ( !getNumber( line, word ) )
                        {
                          cerr << "データエラー　！ [" << filename << "]" << endl;
                          goto errorEnding;
                        }
                      // 数値データに変換する
                      data = atof( word.c_str() );
                      cvmSet( *matrix, j, i, data );
                    }
                  ++j;
                }
            }
        }
      catch ( ... )
        {
          cerr << "ファイル読み込みエラー　！ [" << filename << "]" << endl;
          goto errorEnding;
        }
    }
  else
    {
      cerr << "ファイルオープンエラー　！ [" << filename << "]" << endl;
      return false;
    }

  return true;

  errorEnding:
    cvReleaseMat( matrix );
    return false;
}
