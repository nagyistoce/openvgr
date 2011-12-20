/*
 trans.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 $Date::                            $
*/
#ifndef TRANS_H
#define TRANS_H

#include "Model.h"

#ifdef __cplusplus
extern "C" {
#endif

//! クォータニオン計算用
void translate_point(const RotationAngle* rotation, double xoff, double yoff, double zoff,
                     double* retx, double* rety, double* retz);

#ifdef __cplusplus
}
#endif

#endif // TRANS_H
