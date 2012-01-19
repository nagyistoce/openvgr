/*
 readdata.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
#ifndef readdata_h
#define readdata_h

#include <cxtypes.h>
#include <cxcore.h>

bool readPoints( char* filename, CvMat** points, int& nPoints );
bool readMatrix( char* filename, CvMat** matrix );

#endif // readdata_h
