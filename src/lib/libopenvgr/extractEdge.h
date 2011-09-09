/*
 extractEdge.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file extractEdge.h
 * @brief エッジ抽出関連関数
 * @date \$Date::                            $
 */

#ifndef _EXTRACTEDGE_H
#define _EXTRACTEDGE_H

#include "parameters.h"

// エッジ点を検出するプログラム
// edge 配列への値の意味付け
#define EEnotEdge       (0)  // この点の微分強度は閾値以下であってエッジ点ではない
#define EEerasedThin    (1)  // この点の微分強度は閾値以上だが、細線化により消去された
#define EEcandidate     (2)  // エッジ点として候補として上げられた（一次的に使用される）
#define EEnotSearched   (2)  // 境界線は探索されていない
#define EEsearchedSmall (3)  // 境界線は探索済で、閾値より延長が小さい点
#define EEsearchedLarge (4)  // 境界線は探索済で、閾値より延長が大きい点
#define EEextended      (5)  // 一旦細線化されたが、再度延長により復活した
#define EE6             (6)  // reserved
#define EE7             (7)  // reserved

// エッジ点を検出するプログラム
void extractEdge(unsigned char *edge,    // エッジ画像
                 unsigned char *gray,    // 原画像
                 const int threshold,    // エッジ閾値
                 Parameters parameters); // 全パラメータ

#endif // _EXTRACTEDGE_H
