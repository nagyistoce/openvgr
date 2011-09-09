/*
 ImgSVC_impl.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*-C++-*-
/*!
 * @file  ImgSVC_impl.cpp
 * @brief Service implementation code of Img.idl
 *
 */

#include "ImgSVC_impl.h"

#include "MultiCamera.h"

/*
 * Example implementational code for IDL interface Img::CameraCaptureService
 */
CameraCaptureServiceSVC_impl::CameraCaptureServiceSVC_impl(MultiCamera& mc)
  : m_parent(mc)
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
  m_parent.capture();
  m_parent.m_imagesOut.write();
}



// End of example implementational code



