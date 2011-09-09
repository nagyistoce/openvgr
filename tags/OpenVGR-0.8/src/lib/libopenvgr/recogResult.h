/*
 recogResult.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
/*!
 * @file recogResult.h
 * @brief 認識結果の定義
 */
#ifndef _RECOG_RESULT_H
#define _RECOG_RESULT_H

#define RecogResultElementNum 20

enum RecogResultElement
{
  eRRCameraID, eRRModelID, eRRCandNo, eRRCoordNo,
  eRRRecogReliability, eRRErrorCode, eRRReserve1, eRRReserve2,
  eRRR00, eRRR01, eRRR02, eRRTx,
  eRRR10, eRRR11, eRRR12, eRRTy,
  eRRR20, eRRR21, eRRR22, eRRTz,
};

#endif // _RECOG_RESULT_H
