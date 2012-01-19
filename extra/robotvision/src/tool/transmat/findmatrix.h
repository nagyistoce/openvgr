/*
 findmatrix.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
#ifndef findmatrix_h
#define findmatrix_h

#include <cxtypes.h>
#include <cxcore.h>
#include <cv.h>

// 3次元空間中の点について変換前の点と変換後の点の対応付けから回転と移動を求める
void findTransformationMatrixForPairedPoints(
		double rotation[3][3], // out: 回転行列
		double translation[3], // out: 移動ベクトル
		CvMat* before_moved,   // 変換前の点座標配列
		CvMat* after_moved,    // 変換後の点座標配列
		const double *weight,  // 各データの重み配列
		int nSample            // サンプル数
	);

#endif // findmatrix_h
