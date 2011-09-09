// -*- mode: C++; coding: utf-8 -*-
/*
 Recognition.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file  Recognition.cpp
 * @brief 3D Recognition by Model
 * @date  \$Date::                            $
 */

#include "Recognition.h"
#include "execute3DRecognition.h"
#include "visionErrorCode.h"

// Module specification
// <rtc-template block="module_spec">
static const char *recognition_spec[] = {
  "implementation_id", "Recognition",
  "type_name", "Recognition",
  "description", "3D Recognition by Model",
  "version", "1.0.0",
  "vendor", "AIST.",
  "category", "3D Object Recognition",
  "activity_type", "PERIODIC",
  "kind", "DataFlowComponent",
  "max_instance", "0",
  "language", "C++",
  "lang_type", "compile",
  // Configuration variables
  "conf.default.RecogModelListPath", "modelList.txt",
  "conf.default.RecogParameterFilePath", "recogParameter.txt",
  "conf.default.DebugText", "0",
  "conf.default.DebugImage", "0",
  "conf.default.DebugDisplay", "0",
  // Widget
  "conf.__widget__.RecogModelListPath", "text",
  // Constraints
  ""
};

// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Recognition::Recognition(RTC::Manager* manager)
  // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_stereo3DDataIn("Stereo3DIn", m_stereo3DData),
    m_stereo3DDataOut("Stereo3DOut", m_stereo3DData),
    m_RecognitionResultOut("RecognitionResultOut", m_RecognitionResult),
    m_RecognitionPort("Recognition"),
    m_Reconstruct3DPort("Reconstruct3D"),
    m_RecognitionResultViewerPort("RecognitionResultViewer")
    // </rtc-template>
{
  m_modelList.modelNum = 0;
  m_modelList.model = NULL;

  m_modelID = 0;
}

/*!
 * @brief destructor
 */
Recognition::~Recognition()
{
  // モデルファイル一覧のクリア
  if (m_modelList.model != NULL)
    {
      clearModelFileInfo(&m_modelList);
    }
}

RTC::ReturnCode_t Recognition::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("Stereo3DIn", m_stereo3DDataIn);

  // Set OutPort buffer
  addOutPort("Stereo3DOut", m_stereo3DDataOut);
  addOutPort("RecognitionResultOut", m_RecognitionResultOut);

  // Set service provider to Ports
  m_RecognitionPort.registerProvider("recogPort", "RecognitionService", m_Recognition);

  // Set service consumers to Ports
  m_Reconstruct3DPort.registerConsumer("Reconstruct3D", "Reconstruct3DService", m_Reconstruct3D);
  m_RecognitionResultViewerPort.registerConsumer("RecognitionResultViewer", "RecognitionResultViewerService", m_RecognitionResultViewer);

  // Set CORBA Service Ports
  addPort(m_RecognitionPort);
  addPort(m_Reconstruct3DPort);
  addPort(m_RecognitionResultViewerPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("RecogModelListPath", m_recogModelListPath, "modelList.txt");
  bindParameter("RecogParameterFilePath", m_recogParameterFilePath, "recogParameter.txt");
  bindParameter("DebugText", m_DebugText, "0");
  bindParameter("DebugImage", m_DebugImage, "0");
  bindParameter("DebugDisplay", m_DebugDisplay, "0");

  // </rtc-template>

  m_Recognition.setModelList(&m_modelList);

  // デバッグ用パラメータをメンバ変数に設定する。
  m_Recognition.setDebugParameter(m_DebugText, m_DebugImage, m_DebugDisplay);

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Recognition::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Recognition::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Recognition::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

//
// Active 化した時点で Configuration を読み込む。
//
RTC::ReturnCode_t Recognition::onActivated(RTC::UniqueId ec_id)
{
  // モデルファイル一覧の読み込み。
  // モデル ID とモデルファイル名の組を読み込んで、保持しておく。
  int ret = loadModelListFile((char*) m_recogModelListPath.c_str(), &m_modelList);
  if (ret != 0)
    {
      m_Recognition.setModelList(NULL);
      return RTC::BAD_PARAMETER;
    }

  m_Recognition.setModelList(&m_modelList);

  // 認識設定ファイルのパスを設定する。
  m_Recognition.setRecogParameterPath((char*) m_recogParameterFilePath.c_str());

  // 設定された認識設定ファイルをよみこむ。
  ret = m_Recognition.loadRecogParameter();
  if (ret != 0)
    {
      return RTC::BAD_PARAMETER;
    }

  // デバッグ用パラメータを設定する。
  m_Recognition.setDebugParameter( m_DebugText, m_DebugImage, m_DebugDisplay );
  ret = m_Recognition.loadDebugParameter();
  if (ret != 0)
    {
      return RTC::BAD_PARAMETER;
    }

  return RTC::RTC_OK;
}

//
// Configuration で読み込んだデータをクリアする。
//
RTC::ReturnCode_t Recognition::onDeactivated(RTC::UniqueId ec_id)
{
  // モデルファイル一覧のクリア
  if (m_modelList.model != NULL)
    {
      clearModelFileInfo(&m_modelList);
    }

  return RTC::RTC_OK;
}

//
// 定期実行ルーチン
//
RTC::ReturnCode_t Recognition::onExecute(RTC::UniqueId ec_id)
{

  // 入力ポートに 3 次元距離計測データが送信されるのを待つ。
  if (m_stereo3DDataIn.isNew())
    {
      m_stereo3DDataIn.read();

      // 入力データから画像データをコピーする。
      Img::TimedMultiCameraImage* frame = &(m_stereo3DData.data.mimg);
      if (frame->data.image_seq.length() > 0)
        {
          printf("Image Accept: (%ld x %ld) x %ld\n",
		 frame->data.image_seq[0].image.width,
		 frame->data.image_seq[0].image.height,
		 frame->data.image_seq.length());
        }

      // 3 次元距離計測データをスルー出力する。
      m_stereo3DDataOut.write();

      Parameters param;
      int i;

      // 現在の認識設定値の取得
      m_Recognition.getCurrentRecogParameter(&param);

#ifdef RECOGNITION_TEST
      printf( "Recognition::onExecute:StereoPair=%d\n", param.pairing);
      printf( "Recognition::onExecute:OutputCandNum=%d\n", param.outputCandNum);
      printf( "Recognition::onExecute:EdgeDetectFunction=%d\n", param.feature2D.edgeDetectFunction);
      printf( "Recognition::onExecute:EdgeStrength=%f\n", param.feature2D.edgeStrength);
      printf( "Recognition::onExecute:MaxErrorOfLineFit=%f\n", param.feature2D.maxErrorofLineFit);
      printf( "Recognition::onExecute:MaxErrorOfConicFit=%f\n", param.feature2D.maxErrorofConicFit);
      printf( "Recognition::onExecute:OverlapRatioLine=%f\n", param.feature2D.overlapRatioLine);
      printf( "Recognition::onExecute:OverlapRatioCircle=%f\n", param.feature2D.overlapRatioCircle);
      printf( "Recognition::onExecute:HDMax(max_distance_end_points)=%f\n", param.feature2D.max_distance_end_points);
      printf( "Recognition::onExecute:StereoError=%f\n", param.stereo.ethr);
      printf( "Recognition::onExecute:MatchEdge=%d\n", param.match.edge);
      printf( "Recognition::onExecute:MinLengthLine2D=%f\n", param.feature2D.min_length_line);
#endif //RECOGNITION_TEST

      TimedRecognitionResult pos;
      // 認識を実行する。
      int ret = execute3DRecognition(m_stereo3DData.data.mimg,
                                     m_Recognition.getModelID(),
                                     m_Recognition.getModelFilePath(),
                                     param,
                                     pos);

      if ((ret != 0) && (ret != VISION_ILLEGAL_IMAGE_SIZE))
        // 画像サイズが 0 の場合は、候補数 0 を出力する。
        {
#ifdef _DEBUG
          fprintf(stderr, "Recognition failed.\n");
#endif
          // エラーの場合、候補数 1 でエラーを認識結果に出力する。
          m_RecognitionResult.data.length(RecogResultElementNum);
          for (i = 0; i < RecogResultElementNum; i++)
            {
              m_RecognitionResult.data[i] = 0.0;
            }
          m_RecognitionResult.data[eRRCameraID] = (double) m_stereo3DData.data.mimg.data.camera_set_id;
          m_RecognitionResult.data[eRRErrorCode] = (double) ret;
          m_RecognitionResult.tm = pos.tm;
          m_RecognitionResultOut.write();

          return RTC::RTC_OK;
        }

      // 認識結果を出力ポートに出力する。
      long dataNum = (long) (pos.data.length());
      m_RecognitionResult.data.length(dataNum);
      m_RecognitionResult.tm = pos.tm;

      for (i = 0; i < dataNum; i++)
        {
          m_RecognitionResult.data[i] = pos.data[i];
        }
      m_RecognitionResultOut.write();

      // 結果表示コンポーネントを呼び出して認識結果を表示する。
      if (::CORBA::is_nil(m_RecognitionResultViewer.getObject()) == false)
        {
          m_RecognitionResultViewer->display(m_stereo3DData.data.mimg, pos);
        }

    }

  // モデル ID がセットされていたら、画像送信要求を行う。
  if (m_Recognition.getModelIDUpdateFlag() == true)
    {
      if (::CORBA::is_nil(m_Reconstruct3D.getObject()) == false)
        {
          m_Reconstruct3D->reconstruct();
        }
      m_Recognition.resetModelIDUpdateFlag();
    }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Recognition::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Recognition::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Recognition::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Recognition::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Recognition::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern
  "C"
{

  void
  RecognitionInit(RTC::Manager* manager)
  {
    coil::Properties profile(recognition_spec);
    manager->registerFactory(profile,
			     RTC::Create < Recognition >,
			     RTC::Delete < Recognition >);
  }

}
