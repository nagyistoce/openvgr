/*
 VisionSVC_impl.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
// -*-C++-*-
/*!
 * @file  visionSVC_impl.cpp
 * @brief Service implementation code of vision.idl
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include "VisionSVC_impl.h"

#include <cv.h>
#include <highgui.h>

#include "match3Dfeature.h"
#include "modelpoints.h"
#include "rtvcm.h"
#include "calibUtil.h"
#include "imageUtil.h"
#include "modelFileio.h"

/*
 * implementational code for IDL interface Reconstruct3DService
 */
Reconstruct3DServiceSVC_impl::Reconstruct3DServiceSVC_impl()
{
}

Reconstruct3DServiceSVC_impl::~Reconstruct3DServiceSVC_impl()
{
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void
Reconstruct3DServiceSVC_impl::reconstruct()
{
}

// End of implementational code

/*
 * implementational code for IDL interface RecognitionService
 */
RecognitionServiceSVC_impl::RecognitionServiceSVC_impl()
{
}

RecognitionServiceSVC_impl::~RecognitionServiceSVC_impl()
{
}

/*
 * Methods corresponding to IDL attributes and operations
 */
CORBA::Long RecognitionServiceSVC_impl::getModelID()
{
  return 0;
}

void
RecognitionServiceSVC_impl::setModelID(CORBA::Long ModelID)
{
}

// End of implementational code

/*
 * implementational code for IDL interface RecognitionResultViewerService
 */
RecognitionResultViewerServiceSVC_impl::RecognitionResultViewerServiceSVC_impl()
{
  m_modelList = NULL;
  m_displayImage = NULL;
  m_zoomImage = NULL;

  strcpy(m_displayWindowName, "Result Image of Recognition");

  m_WindowFlag = false;

  m_screenWidth = 1024;
  m_screenHeight = 768;

  pthread_mutex_init(&m_mutex_image, NULL);
  pthread_mutex_init(&m_mutex_flag, NULL);
}

RecognitionResultViewerServiceSVC_impl::~RecognitionResultViewerServiceSVC_impl()
{
  cvDestroyWindow(m_displayWindowName);

  if (m_displayImage != NULL)
    {
      pthread_mutex_lock( &m_mutex_image );
      cvReleaseImage(&m_displayImage);
      m_displayImage = NULL;
      pthread_mutex_unlock( &m_mutex_image );
    }

  if (m_zoomImage != NULL)
    {
      pthread_mutex_lock( &m_mutex_image );
      cvReleaseImage(&m_zoomImage);
      m_zoomImage = NULL;
      pthread_mutex_unlock( &m_mutex_image );
    }

  pthread_mutex_destroy(&m_mutex_image);
  pthread_mutex_destroy(&m_mutex_flag);
}

//
// 表示用画像を表示する。
//
void
RecognitionResultViewerServiceSVC_impl::displayWindow()
{
  int ret;

  if (m_displayImage == NULL)
    {
      return;
    }

  if (getWindowFlag())
    {
      // ウインドウサイズを画面内に収める。
      ret = cvNamedWindow(m_displayWindowName, CV_WINDOW_AUTOSIZE);
      if (ret != 1)
      {
        fprintf(stderr, "cvNamedWindow() failed.\n");
        return;
      }

      int windowWidth = m_displayImage->width;
      int windowHeight = m_displayImage->height;

      printf("ImageSize = %d, %d\n", windowWidth, windowHeight);

      double zoomRatio;
      if (windowWidth > m_screenWidth)
        {
          zoomRatio = (double) m_screenWidth / windowWidth;
          windowWidth = m_screenWidth;
          windowHeight = (int)((double) windowHeight * zoomRatio);
        }

      if (windowHeight > m_screenHeight)
        {
          zoomRatio = (double) m_screenHeight / windowHeight;
          windowHeight = m_screenHeight;
          windowWidth = (int)((double) windowWidth * zoomRatio);
        }

      pthread_mutex_lock( &m_mutex_image );

      if ((windowWidth != m_displayImage->width) ||
          (windowHeight != m_displayImage->height))
        {
          if (m_zoomImage != NULL)
            {
              cvReleaseImage( &m_zoomImage );
            }
          CvSize size;
          size.width = windowWidth;
          size.height = windowHeight;
          m_zoomImage = cvCreateImage( size, IPL_DEPTH_8U, 3);
          if (m_zoomImage == NULL)
            {
              fprintf(stderr, "cvCreateImage() failed.\n");
              return;
            }
          cvResize( m_displayImage, m_zoomImage, CV_INTER_NN );
          cvShowImage( m_displayWindowName, m_zoomImage );
        }
      else
        {
          cvShowImage(m_displayWindowName, m_displayImage);
        }

      pthread_mutex_unlock( &m_mutex_image );

      printf("WindowSize = %d, %d\n", windowWidth, windowHeight);

      cvMoveWindow(m_displayWindowName, 0, 0);

      setWindowFlag(false);
    }

    ret = cvWaitKey(100);
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void
RecognitionResultViewerServiceSVC_impl::display(const Img::TimedMultiCameraImage& frame,
                                                const TimedRecognitionResult& pos)
{
  int imageNum = frame.data.image_seq.length();
  if (imageNum < 2)
    {
      // 画像がない場合、終了。
      fprintf(stderr, "%d 枚の画像が渡されました。\n", imageNum);
      return;
    }
  if (imageNum > 3)
    {
      imageNum = 3;
    }

  int i, j, k;

  for (i = 0; i < imageNum; i++)
    {
      if ((frame.data.image_seq[i].image.width == 0)
          || (frame.data.image_seq[i].image.height == 0))
        {
          // 画像サイズが 0
          fprintf(stderr, "画像サイズ 0 の画像が渡されました。\n");
          return;
        }
    }

  for (i = 1; i < imageNum; i++)
    {
      if ((frame.data.image_seq[i].image.width != frame.data.image_seq[0].image.width)
          || (frame.data.image_seq[i].image.height != frame.data.image_seq[0].image.height))
        {
          // 画像サイズが共通でない。
          fprintf(stderr, "渡された画像のサイズが同一ではありません。\n");
          return;
        }
      if (frame.data.image_seq[i].image.format != frame.data.image_seq[0].image.format)
        {
          // 画像のカラーモデルが共通でない。
          fprintf(stderr, "渡された画像のカラーモデルが同一ではありません。\n");
          return;
        }
    }

#ifdef _DEBUG
  saveDisplayData(frame, pos);
#endif

  int imageWidth = frame.data.image_seq[0].image.width;
  int imageHeight = frame.data.image_seq[0].image.height;
  int channelNum = 3;
  if (frame.data.image_seq[0].image.format == Img::CF_GRAY)
    {
      channelNum = 1;
    }

  int displayImageWidth = imageWidth * imageNum;

  int count;
  CvSize size;
  int ret;

  // 歪み補正マップ
  IplImage** undistortMapX;
  IplImage** undistortMapY;

  // 画像データを OpenCV の IplImage 構造体形式に変換する。
  // 認識結果を描画するために個別の IplImage 構造体を作成する。
  // 認識候補の重ね合わせ用に、画像は channel 数 3 で確保する。
  // ImageData::raw_data は、TopLeft、RGB 順、行単位での padding なし。

  // 歪み補正後の画像に変換する。
  IplImage** resultImage = convertTimedMultiCameraImageToUndistortIplImage(frame, &undistortMapX, &undistortMapY);
  if (resultImage == NULL)
    {
      // 画像の変換に失敗。
      fprintf(stderr, "画像の変換に失敗しました。\n");
      return;
    }

  setWindowFlag(false);

  // 画像表示用に、横に並べた 1 枚の画像を作成する。
  // 既に画像領域が確保されているが、サイズの異なる画像が入力された場合は
  // 領域を確保しなおす。
  if ((m_displayImage != NULL)
      && ((displayImageWidth != m_displayImage->width) 
          || (imageHeight != m_displayImage->height)))
    {
      pthread_mutex_lock( &m_mutex_image );
      cvReleaseImage(&m_displayImage);
      m_displayImage = NULL;
      pthread_mutex_unlock( &m_mutex_image );
    }

  if (m_displayImage == NULL)
    {
      // 表示画像領域の確保
      size.width = displayImageWidth;
      size.height = imageHeight;
      m_displayImage = cvCreateImage(size, IPL_DEPTH_8U, 3);
      if (m_displayImage == NULL)
        {
          // エラー処理
          fprintf(stderr, "表示画像領域が確保できませんでした。\n");
          // 個別画像領域の開放
          freeUndistortIplImage(resultImage,
                                undistortMapX, undistortMapY, imageNum);
          return;
        }
    }

  unsigned char* q;
  int pBytesPerLine = imageWidth * 3;

  pthread_mutex_lock( &m_mutex_image );

  for (i = 0; i < imageNum; i++)
    {
      q = (unsigned char*) m_displayImage->imageData + pBytesPerLine * i;
      count = 0;
      for (j = 0; j < imageHeight; j++)
        {
          for (k = 0; k < imageWidth; k++)
            {
              *q = (unsigned char)(*(resultImage + i))->imageData[count];
              q++;
              count++;
              *q = (unsigned char)(*(resultImage + i))->imageData[count];
              q++;
              count++;
              *q = (unsigned char)(*(resultImage + i))->imageData[count];
              q++;
              count++;
            }
          q += m_displayImage->widthStep - pBytesPerLine;
        }
    }

  pthread_mutex_unlock( &m_mutex_image );

  if (m_modelList == NULL)
    {
      // モデルリストがない場合、画像を表示して終了。
      // 個別画像領域の開放
      freeUndistortIplImage(resultImage,
                            undistortMapX, undistortMapY, imageNum);
      setWindowFlag(true);
      return;
    }

  int candNum = pos.data.length() / RecogResultElementNum;
  if (candNum == 0)
    {
      // 候補が 0 の場合、画像を表示して終了。
      // 個別画像領域の開放
      freeUndistortIplImage(resultImage,
                            undistortMapX, undistortMapY, imageNum);
      setWindowFlag(true);
      return;
    }

  int modelID = -1;
  int candNo = -1;

  // 候補番号 0 の候補を探す。
  for (i = 0; i < candNum; i++)
    {
      if (pos.data[i * RecogResultElementNum + eRRCandNo] == 0)
        {
          modelID = (int) pos.data[i * RecogResultElementNum + eRRModelID];
          candNo = i;
          break;
        }
    }

  if ((candNo == -1) || (modelID < 0))
    {
      // 候補番号 0 が見つからないか、モデル ID が不正。
      fprintf(stderr, "候補番号 0 が認識候補にありません。\n");
      // 個別画像領域の開放
      freeUndistortIplImage(resultImage,
                            undistortMapX, undistortMapY, imageNum);
      // 画像のみ表示して終了。
      setWindowFlag(true);
      return;
    }

  if (pos.data[candNo * RecogResultElementNum + eRRErrorCode] < 0.0)
    {
      // 候補番号 0 のエラーコードにエラーが入っている。
      fprintf(stderr, "エラーコード (%d) が入力されました。\n",
              (int) pos.data[candNo * RecogResultElementNum + eRRErrorCode]);
      // 個別画像領域の開放
      freeUndistortIplImage(resultImage,
                            undistortMapX, undistortMapY, imageNum);
      // 画像を表示しないで終了。
      return;
    }

  // モデルリストからモデル ID を探す。
  for (i = 0; i < m_modelList->modelNum; i++)
    {
      if (m_modelList->model[i].id == modelID)
        {
          break;
        }
    }
  if (i == m_modelList->modelNum)
    {
      // モデル ID が不正。
      fprintf(stderr, "認識候補で指定されたモデル ID がモデルリストにありません。\n");
      // 個別画像領域の開放
      freeUndistortIplImage(resultImage,
                            undistortMapX, undistortMapY, imageNum);
      // 画像のみ表示して終了。
      setWindowFlag(true);
      return;
    }

  // 候補に埋め込まれているモデル ID からモデルデータを読み込む。
  Features3D model = { 0 };
  ret = loadModelFile(m_modelList->model[i].path, model);
  if (ret != 0)
    {
      fprintf(stderr, "モデルファイルが読み込めません。\n");
      // 個別画像領域の開放
      freeUndistortIplImage(resultImage,
                            undistortMapX, undistortMapY, imageNum);
      // 画像のみ表示して終了。
      setWindowFlag(true);
      return;
    }

  // モデルデータにモデルサンプル点を生成する。
  makeModelPoints(&model, 3.0);

  // 認識結果の行列を取り出す。
  double matrix[4][4] = {{0.0}};
  int index = candNo * RecogResultElementNum;
  const double *posdata = &pos.data[index + eRRR00];

  for (i = 0; i < 3; i++ )
    {
      for (j = 0; j < 4; j++ )
        {
          matrix[i][j] = *posdata++;
        }
    }
  matrix[3][3] = 1.0;

  // キャリブレーションデータを Camera 構造体に変換する。
  CalibParam calib = { 0 };
  calib.numOfCameras = imageNum;
  calib.colsize = frame.data.image_seq[0].image.width;
  calib.rowsize = frame.data.image_seq[1].image.height;

  setCalibFromCameraImage(frame.data.image_seq[0], calib.CameraL);
  setCalibFromCameraImage(frame.data.image_seq[1], calib.CameraR);
  if (imageNum > 2)
    {
      setCalibFromCameraImage(frame.data.image_seq[2], calib.CameraV);
    }

  model.calib = &calib;

  double zoomRatio = 1.0;
  if (m_displayImage->width > m_screenWidth)
    {
      zoomRatio = (double) m_screenWidth / m_displayImage->width;
    }
  if (m_displayImage->height > m_screenHeight)
    {
      zoomRatio = (double) m_screenHeight / m_displayImage->height;
    }

  // 画像の拡大率に応じて、線の太さを変更する。
  int lineThickness = (int)(2.0 / zoomRatio + 0.5);
  if (lineThickness < 2)
    {
      lineThickness = 2;
    }

  // 歪み補正前の画像に描画する。
  IplImage** distImage = convertTimedMultiCameraImageToIplImage(frame);
  if (distImage == NULL)
    {
      // 画像の変換に失敗。
      fprintf(stderr, "画像の変換に失敗しました。\n");
      return;
    }

  // 画像データ上にモデルデータを描画
  for (i = 0; i < imageNum; i++)
    {
#ifdef _DEBUG
      char filename[MAX_PATH];
      // 画像データをファイルに保存する際のファイル名
      sprintf(filename, "/tmp/recogResultImage%02d.ppm", i);

      drawModelPoints(&model, matrix, filename,
                      i, (unsigned char*)(*(distImage + i))->imageData, lineThickness);
#else
      drawModelPoints(&model, matrix, NULL,
                      i, (unsigned char*)(*(distImage + i))->imageData, lineThickness);
#endif

      // 歪み補正をする。
      cvRemap(*(distImage + i), *(resultImage + i), 
                                *(undistortMapX + i), *(undistortMapY + i));
    }

  for (i = 0; i < imageNum; i++)
    {
      cvReleaseImage(distImage + i);
    }

  pthread_mutex_lock(&m_mutex_image);

  unsigned char* p;

  // 表示用画像の生成
  for (i = 0; i < imageNum; i++)
    {
      p = (unsigned char*) (*(resultImage + i))->imageData;
      q = (unsigned char*) m_displayImage->imageData + pBytesPerLine * i;
      count = 0;
      for (j = 0; j < imageHeight; j++)
        {
          memcpy(q, p, (*(resultImage + i))->widthStep);
          p += (*(resultImage + i))->widthStep;
          q += m_displayImage->widthStep;
        }
    }

  pthread_mutex_unlock(&m_mutex_image);

#ifdef _DEBUG
  cvSaveImage("/tmp/recogResultImage.ppm", m_displayImage);
#endif

  // 個別画像領域の開放
  freeUndistortIplImage(resultImage,
                        undistortMapX, undistortMapY, imageNum);
  // 画像データの表示
  setWindowFlag(true);

  return;
}

//
// モデルファイル一覧のセット
//
void
RecognitionResultViewerServiceSVC_impl::setModelList(ModelFileInfo* info)
{
  m_modelList = info;
}

//
// 画面の解像度を設定する。
//
void
RecognitionResultViewerServiceSVC_impl::setScreenSize(const int width, const int height)
{
  m_screenWidth = width - 50;    // ウインドウ枠の分だけ減らす。
  m_screenHeight = height - 100; // ウインドウ枠の分減らす。
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void
RecognitionResultViewerServiceSVC_impl::saveDisplayData(const Img::TimedMultiCameraImage& frame,
                                                        const TimedRecognitionResult& pos)
{
  int imageNum = frame.data.image_seq.length();

  char filename[MAX_PATH];
#if 0
  sprintf(filename, "recogResultDisplayData.%ld.%ld.txt",
          (long) frame.tm.sec, (long) frame.tm.nsec);
#else
  struct timeval currentTime;
  gettimeofday( &currentTime, NULL );
  snprintf(filename, sizeof(filename),
           "recogResultDisplayData.%ld.%ld.txt",
            (long)currentTime.tv_sec,
            (long)currentTime.tv_usec);
#endif

  FILE* fp = fopen(filename, "w");
  if (fp == NULL)
    {
      fprintf(stderr, "Can't open %s.\n", filename);
    }

  fprintf(fp, "MultiCameraImage Num = %d\n", imageNum);

  int i, j;
  const Img::CameraImage* cData;
  int dcNum;
  const double *elem;

  for (i = 0; i < imageNum; i++)
    {
      cData = &(frame.data.image_seq[i]);

      fprintf(fp, "\n");
      fprintf(fp, "Image %d:\n", i);
      fprintf(fp, "\t(%ld, %ld), %ld\n",
              (long) cData->image.width, (long) cData->image.height, (long) cData->image.format);

      elem = cData->intrinsic.matrix_element;
      fprintf(fp, "Matrix Element = (%lf, %lf, %lf, %lf, %lf)\n",
              elem[0], elem[1], elem[2], elem[3], elem[4]);
      dcNum = cData->intrinsic.distortion_coefficient.length();
      fprintf(fp, "distortion_coefficient Num = %d\n", dcNum);
      fprintf(fp, "\t(");
      for (j = 0; j < dcNum; j++)
        {
          fprintf(fp, "%lf, ", cData->intrinsic.distortion_coefficient[j]);
        }
      fprintf(fp, ")\n");

      fprintf(fp, "extrinsic:\n");
      for (j = 0; j < 4; j++)
        {
          elem = cData->extrinsic[j];
          fprintf(fp, "\t(%lf, %lf, %lf, %lf)\n",
                  elem[0], elem[1], elem[2], elem[3]);
        }
    }

  int candNum = pos.data.length() / RecogResultElementNum;

  fprintf(fp, "\n");
  fprintf(fp, "%d\n", candNum);

  for (i = 0; i < candNum; i++)
    {
      elem = &pos.data[ i * RecogResultElementNum ];
      for (j = eRRCameraID; j <= eRRCoordNo; j++)
        {
          fprintf(fp, "%d, ", (int)elem[j]);
        }
      fprintf(fp, "%lf, %d, %d, %d, ",
              elem[eRRRecogReliability],
              (int)elem[eRRErrorCode],
              (int)elem[eRRReserve1],
              (int)elem[eRRReserve2]);
      for (j = eRRR00; j <= eRRTz; j++)
        {
          fprintf(fp, "%lf, ", elem[j]);
        }
      fprintf(fp, "\n");
    }

  fclose(fp);
  return;
}

void
RecognitionResultViewerServiceSVC_impl::setWindowFlag( bool val )
{
  pthread_mutex_lock(&m_mutex_flag);
  m_WindowFlag = val;
  pthread_mutex_unlock(&m_mutex_flag);
}

bool
RecognitionResultViewerServiceSVC_impl::getWindowFlag()
{
  bool val;
  pthread_mutex_lock(&m_mutex_flag);
  val = m_WindowFlag;
  pthread_mutex_unlock(&m_mutex_flag);
  return val;
}
