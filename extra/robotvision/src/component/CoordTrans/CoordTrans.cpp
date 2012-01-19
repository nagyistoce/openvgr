/*
 CoordTrans.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*- C++ -*-
/*!
 * @file  CoordTrans.cpp
 * @brief Coordinate Transformation
 * @date $Date$
 *
 * $Id$
 */

#include "CoordTrans.h"
#include "recogResult.h"
#include "readdata.h"

#include <iostream>
#include <fstream>

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

using namespace std;

#define MAX_PATH 256

// Module specification
// <rtc-template block="module_spec">
static const char* coordtrans_spec[] =
  {
    "implementation_id", "CoordTrans",
    "type_name",         "CoordTrans",
    "description",       "Coordinate Transformation",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.MatrixFile", "matrix.txt",
    // Widget
    "conf.__widget__.MatrixFile", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
CoordTrans::CoordTrans(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_recogResultIn("RecogResult", m_recogResult),
    m_transformedOut("Transformed", m_transformed)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
CoordTrans::~CoordTrans()
{
}



RTC::ReturnCode_t CoordTrans::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("RecogResult", m_recogResultIn);
  
  // Set OutPort buffer
  addOutPort("Transformed", m_transformedOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("MatrixFile", m_MatrixFile, "matrix.txt");
  
  // </rtc-template>
  m_TransMat = NULL;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CoordTrans::onFinalize()
{
  if ( m_TransMat )
    {
      cvReleaseMat( &m_TransMat );
      m_TransMat = NULL;
    }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CoordTrans::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CoordTrans::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t CoordTrans::onActivated(RTC::UniqueId ec_id)
{
  bool sts;
  cout << "座標変換行列 [" << m_MatrixFile << "] 読み込み" << endl;
  sts = readMatrix( (char *)m_MatrixFile.c_str(), &m_TransMat );
  if ( sts == false )
    {
      cerr << "単位行列を使用します．" << endl;
      m_TransMat = cvCreateMat( 4, 4, CV_64FC1 );
      cvSetIdentity( m_TransMat );
    }
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CoordTrans::onDeactivated(RTC::UniqueId ec_id)
{
  if ( m_TransMat )
    {
      cvReleaseMat( &m_TransMat );
      m_TransMat = NULL;
    }
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CoordTrans::onExecute(RTC::UniqueId ec_id)
{
  // 入力ポートに認識結果が送られてくるのを待つ。
  if ( m_recogResultIn.isNew() )
    {
      m_recogResultIn.read();

      int i, j, index;
      int candNum = m_recogResult.data.length() / RecogResultElementNum;

#if defined(RCVDATAOUT)
      if ( saveReceivedData( candNum ) )
        {
          return RTC::RTC_ERROR;
        }
#endif
      double matdata[16] = {0};
      double outdata[16] = {0};
      CvMat ResultMat;
      CvMat OutMat;

      ResultMat = cvMat( 4, 4, CV_64FC1, matdata );
      OutMat    = cvMat( 4, 4, CV_64FC1, outdata );

      for ( i = 0; i < candNum; i++ )
        {
          index = i * RecogResultElementNum;
          for ( j = 0; j < 12; j++ )
            {
              matdata[j] = m_recogResult.data[index + eRRR00 + j];
            }
          matdata[12] = 0.0;
          matdata[13] = 0.0;
          matdata[14] = 0.0;
          matdata[15] = 1.0;
          // OutMat = m_TransMat * ResultMat
          cvMatMul( m_TransMat, &ResultMat, &OutMat );
          for ( j = 0; j < 12; j++ )
            {
              m_recogResult.data[index + eRRR00 + j] = outdata[j];
            }
        }

      if ( printTransformedData( candNum ) )
        {
          return RTC::RTC_ERROR;
        }

      // 認識結果を出力ポートに出力する。
      long dataNum = (long)m_recogResult.data.length();
      m_transformed.data.length(dataNum);
      m_transformed.tm = m_recogResult.tm;

      for (i = 0; i < dataNum; i++)
        {
          m_transformed.data[i] = m_recogResult.data[i];
        }

      m_transformedOut.write();

    }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CoordTrans::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CoordTrans::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CoordTrans::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CoordTrans::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CoordTrans::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

static void getTimestampName( string title, char* strbuf, uint bufsize )
{
  struct timeval current;
  char timestamp[16];

  gettimeofday( &current, NULL );

  strftime( timestamp, sizeof(timestamp),
            "%Y%m%d-%H%M%S", localtime(&current.tv_sec) );

  snprintf( strbuf, bufsize,
            "%s.%s.%06ld.csv",
            title.c_str(), timestamp, current.tv_usec );
}

int CoordTrans::saveReceivedData( int candNum )
{
  char strbuf[MAX_PATH];

  // 時刻をファイル名にして、内容を保存する。
  getTimestampName( "recogResult", strbuf, sizeof(strbuf) );

  ofstream ofs;

  ofs.open(strbuf);

  if ( ofs == NULL )
    {
      return -1;
    }

  int i, j, index;

  ofs << candNum << endl;

  for ( i = 0; i < candNum; i++ )
    {
      index = i * RecogResultElementNum;

      for( j = eRRCameraID; j <= eRRCoordNo; j++ )
        {
          snprintf( strbuf, sizeof(strbuf), "%d, ",
                    (int)m_recogResult.data[index + j] );
          ofs << strbuf;
        }
      snprintf( strbuf, sizeof(strbuf), "%lf, %d, %d, %d, ",
                m_recogResult.data[index + eRRRecogReliability],
                (int)m_recogResult.data[index + eRRErrorCode],
                (int)m_recogResult.data[index + eRRReserve1],
                (int)m_recogResult.data[index + eRRReserve2] );
      ofs << strbuf;
      for( j = eRRR00; j <= eRRTz; j++ )
        {
          snprintf( strbuf, sizeof(strbuf), "%lf, ",
                    m_recogResult.data[index + j] );
          ofs << strbuf;
        }
      ofs << endl;
    }

  return 0;
}

int CoordTrans::printTransformedData( int candNum )
{
  char strbuf[MAX_PATH];

#if defined(CSVOUT)
  // 時刻をファイル名にして、内容を保存する。
  getTimestampName( "recogResult", strbuf, sizeof(strbuf) );
  ofstream ofs;

  ofs.open(strbuf);

  if ( ofs == NULL )
    {
      return -1;
    }
#endif

  cout << "Converted Matrix\n";

  int linenum = 0;
  if ( candNum ) linenum = 1;

  int i, j, index;

  for ( i = 0; i < linenum; i++ )
    {
      index = i * RecogResultElementNum;

      for ( j = eRRR00; j <= eRRTz; j++ )
        {
#if defined(CSVOUT)
          snprintf( strbuf, sizeof(strbuf), "%lf, ",
                    m_recogResult.data[index + j] );
          ofs << strbuf;
#endif
          snprintf( strbuf, sizeof(strbuf), "% 12.6f",
                    m_recogResult.data[index + j] );
          cout << strbuf;
          if ( j != eRRTx && j != eRRTy && j != eRRTz )
            {
              cout << " ";
            }
          else
            {
              cout << endl;
            }
        }

      double dotx = m_recogResult.data[index + eRRR00];
      double doty = m_recogResult.data[index + eRRR11];
      double dotz = m_recogResult.data[index + eRRR22];
      double adifx = acos(dotx)*180.0/M_PI;
      double adify = acos(doty)*180.0/M_PI;
      double adifz = acos(dotz)*180.0/M_PI;
      snprintf( strbuf, sizeof(strbuf),
                "angle diff\n    x = % f   y = % f   z = % f\n",
                adifx, adify, adifz );
      cout << strbuf << endl;
#if defined(CSVOUT)
      ofs << endl;
#endif
    }

  return 0;
}

extern "C"
{
 
  void CoordTransInit(RTC::Manager* manager)
  {
    coil::Properties profile(coordtrans_spec);
    manager->registerFactory(profile,
                             RTC::Create<CoordTrans>,
                             RTC::Delete<CoordTrans>);
  }
  
};
