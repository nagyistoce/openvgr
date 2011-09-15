/*
 debugutil.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file debugutil.h
 * @brief デバッグ用関数
 * @date \$Date::                            $
 */

#ifndef _DEBUGUTIL_H
#define _DEBUGUTIL_H

#include "stereo.h"

int drawInputImage(const uchar* src, const Parameters& parameters);
int drawEdgeImage(const uchar* edge, const Parameters& parameters);

int drawDetectedLines(const uchar* edge, const Features2D_old* lineFeatures, const Parameters& parameters);
int drawDetectedVertices(const Features2D_old* features, const Parameters& parameters);
int drawDetectedEllipses(const uchar* edge, const Features2D_old* features, const Parameters& parameters);
int drawTrackPoints(const Features2D_old* features, const Parameters& parameters);

int drawStereoVertices(const uchar* edge, const StereoData& stereo, int pairing, const Parameters& parameters, const CameraParam* cameraParam);
int drawStereoCircles(const uchar* edge, const StereoData& stereo, int pairing, const Parameters& parameters, const CameraParam* cameraParam);

int printStereoVertices(const StereoData& stereo, int pairing);
int printStereoCircles(const StereoData& stereo, int pairing);

int drawCircleCandidate(const uchar* edge,
                        const std::vector<CircleCandidate>& candidates,
                        int pairing,
                        const Parameters& parameters,
                        const CameraParam* cameraParam);

#endif /* _DEBUGUTIL_H */
