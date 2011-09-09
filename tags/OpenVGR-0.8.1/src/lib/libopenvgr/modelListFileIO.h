/*
 modelListFileIO.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
/*!
 * @file modelListFileIO.h
 * @brief モデルファイルの入出力関連
 */

#ifndef _MODELLISTFILEIO_H
#define _MODELLISTFILEIO_H

//
// モデル一覧ファイルを読み込んで、モデル ID とモデルファイル名の対応リスト
// を保持する。
//

#ifndef MAX_PATH
#define MAX_PATH	256
#endif

//! モデルリストのノード
typedef struct ModelFileInfoNode
{
  int id;                       //!< モデル ID
  char path[MAX_PATH];          //!< モデルファイル名
} ModelFileInfoNode;

//! モデルファイルリスト
typedef struct ModelFileInfo
{
  ModelFileInfoNode* model;     //!< モデルリスト
  int modelNum;                 //!< モデル数
} ModelFileInfo;

//! モデル一覧ファイルを読み込んで、モデル ID とモデルファイル名の対応リスト
//! を保持する。
extern int loadModelListFile(char* filename, ModelFileInfo* mfInfo);

//! モデルリストをクリアする。
extern void clearModelFileInfo(ModelFileInfo* mfInfo);

#endif // _MODELLISTFILEIO_H
