/*
 visionErrorCode.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
/*!
 * @file visionErrorCode.h
 * @brief 返り値の定義
 */

#ifndef _VISIONERRORCODE_H
#define _VISIONERRORCODE_H

enum VisionErrorCode
{
  //! 関数の引数エラー
  VISION_PARAM_ERROR                    = -1,
  //! メモリ領域確保エラー
  VISION_MALLOC_ERROR                   = -2,
  //! ファイルオープンエラー
  VISION_FILE_OPEN_ERROR                = -3,
  //! ファイルフォーマットエラー
  VISION_FILE_FORMAT_ERROR              = -4,

  //! 入力画像の幅または高さが 0
  VISION_ILLEGAL_IMAGE_SIZE             = -101,

  //! 入力された画像が 1 枚以下
  VISION_INPUT_NOIMAGE                  = -102,

  //! 入力された画像のサイズが同一でない
  VISION_DIFF_IMAGE_SIZE                = -103,

  //! 入力された画像のカラーモデルが同一でない
  VISION_DIFF_IMAGE_COLOR_MODEL         = -104,

  //! 入力されたモデル番号に該当するモデルデータがない
  VISION_NO_MODEL_FILE                  = -105
};

#endif // _VISIONERRORCODE_H
