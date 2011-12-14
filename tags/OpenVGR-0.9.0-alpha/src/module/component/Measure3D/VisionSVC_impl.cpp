/*
 VisionSVC_impl.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*-C++-*-
/*!
 * @file  VisionSVC_impl.cpp
 * @brief Service implementation code of Vision.idl
 *
 */

#include "VisionSVC_impl.h"
#include "Measure3D.h"
#include <stdio.h>

/*
 * Example implementational code for IDL interface Img::CameraCaptureService
 */
CameraCaptureServiceSVC_impl::CameraCaptureServiceSVC_impl()
{
  // Please add extra constructor code here.
}


CameraCaptureServiceSVC_impl::~CameraCaptureServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void CameraCaptureServiceSVC_impl::take_one_frame()
{
  // Please insert your code here and remove the following warning pragma
//#ifndef WIN32
//  #warning "Code missing in function <void CameraCaptureServiceSVC_impl::take_one_frame()>"
//#endif
}



// End of example implementational code

/*
 * Example implementational code for IDL interface Reconstruct3DService
 */
Reconstruct3DServiceSVC_impl::Reconstruct3DServiceSVC_impl(Measure3D& m3d)
  : parent(m3d)
{
  // Please add extra constructor code here.
}


Reconstruct3DServiceSVC_impl::~Reconstruct3DServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void Reconstruct3DServiceSVC_impl::reconstruct()
{
  // Please insert your code here and remove the following warning pragma
  if (::CORBA::is_nil(parent.m_CaptureCameraService.getObject()) == false)
    {
      parent.m_CaptureCameraService->take_one_frame();
    }
}



// End of example implementational code

/*
 * Example implementational code for IDL interface RecognitionService
 */
RecognitionServiceSVC_impl::RecognitionServiceSVC_impl()
{
  // Please add extra constructor code here.
}


RecognitionServiceSVC_impl::~RecognitionServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
CORBA::Long RecognitionServiceSVC_impl::getModelID()
{
  // Please insert your code here and remove the following warning pragma
//#ifndef WIN32
//  #warning "Code missing in function <CORBA::Long RecognitionServiceSVC_impl::getModelID()>"
//#endif
  return 0;
}

void RecognitionServiceSVC_impl::setModelID(CORBA::Long ModelID)
{
  // Please insert your code here and remove the following warning pragma
//#ifndef WIN32
//  #warning "Code missing in function <void RecognitionServiceSVC_impl::setModelID(CORBA::Long ModelID)>"
//#endif
}



// End of example implementational code

/*
 * Example implementational code for IDL interface RecognitionResultViewerService
 */
RecognitionResultViewerServiceSVC_impl::RecognitionResultViewerServiceSVC_impl()
{
  // Please add extra constructor code here.
}


RecognitionResultViewerServiceSVC_impl::~RecognitionResultViewerServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void RecognitionResultViewerServiceSVC_impl::display(const Img::TimedMultiCameraImage& frame, TimedRecognitionResult pos)
{
  // Please insert your code here and remove the following warning pragma
//#ifndef WIN32
//  #warning "Code missing in function <void RecognitionResultViewerServiceSVC_impl::display(const Img::TimedMultiCameraImage& frame, TimedRecognitionResult pos)>"
//#endif
}



// End of example implementational code



