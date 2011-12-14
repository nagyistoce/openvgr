/*
 VisionSVC_impl.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*-C++-*-
/*!
 * @file  VisionSVC_impl.h
 * @brief Service implementation header of Vision.idl
 */

#include "BasicDataTypeSkel.h"
#include "ImgSkel.h"

#include "VisionSkel.h"

#ifndef VISIONSVC_IMPL_H
#define VISIONSVC_IMPL_H

class Measure3D; // forward declaration
 
/*!
 * @class CameraCaptureServiceSVC_impl
 * Example class implementing IDL interface Img::CameraCaptureService
 */
class CameraCaptureServiceSVC_impl
 : public virtual POA_Img::CameraCaptureService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~CameraCaptureServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   CameraCaptureServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~CameraCaptureServiceSVC_impl();

   // attributes and operations
   void take_one_frame();

};

/*!
 * @class Reconstruct3DServiceSVC_impl
 * Example class implementing IDL interface Reconstruct3DService
 */
class Reconstruct3DServiceSVC_impl
 : public virtual POA_Reconstruct3DService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~Reconstruct3DServiceSVC_impl();
   Measure3D& parent;

 public:
  /*!
   * @brief standard constructor
   */
   Reconstruct3DServiceSVC_impl(Measure3D& m3d);
  /*!
   * @brief destructor
   */
   virtual ~Reconstruct3DServiceSVC_impl();

   // attributes and operations
   void reconstruct();

};

/*!
 * @class RecognitionServiceSVC_impl
 * Example class implementing IDL interface RecognitionService
 */
class RecognitionServiceSVC_impl
 : public virtual POA_RecognitionService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RecognitionServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RecognitionServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RecognitionServiceSVC_impl();

   // attributes and operations
   CORBA::Long getModelID();
   void setModelID(CORBA::Long ModelID);

};

/*!
 * @class RecognitionResultViewerServiceSVC_impl
 * Example class implementing IDL interface RecognitionResultViewerService
 */
class RecognitionResultViewerServiceSVC_impl
 : public virtual POA_RecognitionResultViewerService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RecognitionResultViewerServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RecognitionResultViewerServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RecognitionResultViewerServiceSVC_impl();

   // attributes and operations
   void display(const Img::TimedMultiCameraImage& frame, TimedRecognitionResult pos);

};



#endif // VISIONSVC_IMPL_H


