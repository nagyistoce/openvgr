/*
 modelListFileIO.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "modelListFileIO.h"
#include "visionErrorCode.h"

//
//! モデル一覧ファイルを読み込んで、モデル ID とモデルファイル名の対応リスト
//! を保持する。
//
int
loadModelListFile(char* filename, ModelFileInfo* mfInfo)
{
  if ((filename == NULL) || (mfInfo == NULL))
    {
      return VISION_PARAM_ERROR;
    }
  if (filename[0] == '\0')
    {
      return VISION_PARAM_ERROR;
    }

  FILE* fp = fopen(filename, "r");
  if (fp == NULL)
    {
      return VISION_FILE_OPEN_ERROR;
    }

  char buffer[256];
  int lineCount = 0;
  int i = 0, j = 0;
  int length = 0;

  // 一旦ファイルを末尾まで読み込み、行数を数える。
  while (fgets(buffer, 256, fp))
    {
      if (buffer[0] == '\0')
        {
          continue;
        }
      length = strlen(buffer);
      for (i = 0; i < length; i++)
        {
          if (!isdigit(buffer[i]))
            {
              break;
            }
        }
      if ((i == 0) || (i > length - 2))
        {
          // データ不正と見なし、無視する。
          continue;
        }

      lineCount++;
    }

  if (lineCount == 0)
    {
      fclose(fp);
      return VISION_FILE_FORMAT_ERROR;
    }

  mfInfo->modelNum = lineCount;
  mfInfo->model = (ModelFileInfoNode*) calloc(lineCount, sizeof(ModelFileInfoNode));
  if (mfInfo->model == NULL)
    {
      fclose(fp);
      return VISION_MALLOC_ERROR;
    }

  rewind(fp);

  j = 0;

  while (fgets(buffer, 256, fp))
    {
      if (buffer[0] == '\0')
        {
          continue;
        }
      length = strlen(buffer);
      for (i = 0; i < length; i++)
        {
          if (!isdigit(buffer[i]))
            {
              break;
            }
        }
      if ((i == 0) || (i > length - 2))
        {
          // データ不正と見なし、無視する。
          continue;
        }

      buffer[i] = '\0';
      i++;

      // 余計な空白の除去
      for (; i < length; i++)
        {
          if ((buffer[i] != ' ') && (buffer[i] != '\t'))
            {
              break;
            }
        }

      // 改行の除去
      if ((buffer[length - 1] == '\n') || (buffer[length - 1] == '\r'))
        {
          buffer[length - 1] = '\0';
        }
      if (buffer[length - 2] == '\r')
        {
          buffer[length - 2] = '\0';
        }

      mfInfo->model[j].id = atoi(buffer);
      strncpy(mfInfo->model[j].path, &(buffer[i]), MAX_PATH);

#ifdef DEBUG
      printf("[%d][ID: %d][%s]\n", j, mfInfo->model[j].id, mfInfo->model[j].path);
#endif
      j++;
      if (j > lineCount)
        {
          break;
        }
    }

  fclose(fp);

  return 0;
}

//
//! モデルリストをクリアする。
//
void
clearModelFileInfo(ModelFileInfo* mfInfo)
{
  if (mfInfo == NULL)
    {
      return;
    }

  if (mfInfo->model != NULL)
    {
      free(mfInfo->model);
      mfInfo->model = NULL;
    }
  mfInfo->modelNum = 0;

  return;
}
