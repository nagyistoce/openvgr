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

#include <cstring>

#include "stereo.h"

int drawInputImage(const uchar* src, const Parameters& parameters, const int id);
int drawEdgeImage(const uchar* edge, const Parameters& parameters, const int id);

int drawDetectedLines(const uchar* edge, const Features2D_old* lineFeatures, const Parameters& parameters,
                      const int id);
int drawDetectedVertices(const Features2D_old* features, const Parameters& parameters, const int id);
int drawDetectedEllipses(const uchar* edge, const Features2D_old* features, const Parameters& parameters,
                         const int id);

int drawCircleCandidate(const uchar* edge,
                        const std::vector<CircleCandidate>& candidates,
                        int pairing,
                        const Parameters& parameters,
                        const CameraParam* cameraParam);

int printVertex(const std::vector< ::Vertex>& vertex);

namespace ovgr
{
  template <class T>
  struct EqualOp
  {
    bool operator()(const T& a, const T& b)
    {
      return memcmp(&a, &b, sizeof(T));
    }
  };

  template <class T, class Equal = EqualOp<T> >
  class VariableWatcher
  {
    T orig;
    T* ptr;
  public:
    VariableWatcher(T& val) : orig(val), ptr(&val) {}
    bool is_changed() {
      return Equal()(orig, *ptr); }
  };
}

#endif /* _DEBUGUTIL_H */
