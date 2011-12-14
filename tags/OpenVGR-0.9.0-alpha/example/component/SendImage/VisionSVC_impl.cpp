// -*-C++-*-
/*
 VisionSVC_impl.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
/*!
 * @file  VisionSVC_impl.cpp
 * @brief Service implementation code of Vision.idl
 *
 */
#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>
#include <sys/time.h>

#include <highgui.h>

#include "VisionSVC_impl.h"
#include "visionErrorCode.h"

using namespace std;
using namespace cv;

/*
 * implementational code for IDL interface Img::CameraCaptureService
 */
CameraCaptureServiceSVC_impl::CameraCaptureServiceSVC_impl()
{
}

CameraCaptureServiceSVC_impl::~CameraCaptureServiceSVC_impl()
{
}

/*
 * Methods corresponding to IDL attributes and operations
 */
void
CameraCaptureServiceSVC_impl::take_one_frame()
{
}

// End of implementational code

/*
 * implementational code for IDL interface Reconstruct3DService
 */
Reconstruct3DServiceSVC_impl::Reconstruct3DServiceSVC_impl()
{
  m_readySendImageFlag = NULL;

#if 1
  m_stereo3DData = NULL;
#else
  memset(&m_stereo3DData, 0x0, sizeof(TimedStereo3D));
#endif

  m_readImageNum = 3;
  m_outputPointNum = -1;
  m_errorCode = "";
  m_calibFilename = "";
  m_imageFilename0 = "";
  m_imageFilename1 = "";
  m_imageFilename2 = "";
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
  // 認識実行要求。
  // ここでは、画像ファイルとキャリブレーションデータを読み込んで、
  // OutPort に 3 次元点列形式で出力する。
  if (m_readySendImageFlag == NULL)
    {
      return;
    }
  if (*m_readySendImageFlag == true)
    {
      return;
    }
  if (m_stereo3DData == NULL)
    {
      return;
    }

  string buffer;

  int outputPointNum = 1;
  int errorCode = 0;
  int i, j;

  // 点群数の設定
  if (m_outputPointNum == -1)
    {
      cout << "出力する点群数を入力してください。：";
      cin >> buffer;
      outputPointNum = atoi(buffer.c_str());
    }
  else
    {
      outputPointNum = m_outputPointNum;
      cout << "点群数 : " << outputPointNum << endl;
    }

  if (m_errorCode == "")
    {
      // エラーコードの設定
      cout << "出力するエラーコードを入力してください。：";
      cin >> buffer;
      errorCode = atoi(buffer.c_str());
    }
  else
    {
      errorCode = atoi(m_errorCode.c_str());
      cout << "エラーコード : " << errorCode << endl;
    }

  if (m_calibFilename != "")
    {
      cout << "キャリブレーションファイル名 : " << m_calibFilename << endl;
    }

  if (m_imageFilename0 != "")
    {
      cout << "画像ファイル名 0 : " << m_imageFilename0 << endl;
    }

  if (m_imageFilename1 != "")
    {
      cout << "画像ファイル名 1 : " << m_imageFilename1 << endl;
    }

  if (m_imageFilename2 != "")
    {
      cout << "画像ファイル名 2 : " << m_imageFilename2 << endl;
    }

  if (m_readImageNum >= 1)
    {
      while (1)
        {
          // 画像ファイルの読み込み。
          int ret = loadImage();
          if (ret != 0)
            {
              cout << "画像ファイルが読み込めません。";
              continue;
            }

          char* datapath;

          if (m_calibFilename == "")
            {
              cout << "キャリブレーションデータのパスを入力してください。：";
              cin >> buffer;
              datapath = (char*)buffer.c_str();
            }
          else
            {
              datapath = (char*)m_calibFilename.c_str();
            }

          // キャリブレーションデータの読み込み
          if ( strstr( datapath, ".yaml" ) == NULL )
            {
              ret = loadCalibrationData(datapath);
              if (ret != 0)
                {
                  ret = loadCalibrationDataByOpenCV(datapath);
                  if (ret != 0)
                    {
                      cout << "キャリブレーションデータが読み込めません。";
                      continue;
                    }
                }
            }
          else
            {
              ret = loadMultiCalibData(datapath);
              if (ret != 0)
                {
                  cout << "キャリブレーションデータが読み込めません。";
                  continue;
                }
            }

          break;
        }
    }

  m_stereo3DData->data.mimg.error_code = errorCode;

  m_stereo3DData->data.mimg.data.camera_set_id = 1;

  // その他の項目を初期化
  m_stereo3DData->error_code = 0;
  m_stereo3DData->tm.sec = m_stereo3DData->data.mimg.tm.sec;
  m_stereo3DData->tm.nsec = m_stereo3DData->data.mimg.tm.nsec;

  m_stereo3DData->data.obj.error_code = 0;
  m_stereo3DData->data.obj.tm.sec = m_stereo3DData->data.mimg.tm.sec;
  m_stereo3DData->data.obj.tm.nsec = m_stereo3DData->data.mimg.tm.nsec;

  m_stereo3DData->data.obj.data.id = 0;
  m_stereo3DData->data.obj.data.point.length(outputPointNum);
  for (i = 0; i < outputPointNum; i++)
    {
      m_stereo3DData->data.obj.data.point[i][0] = (double) i;
      m_stereo3DData->data.obj.data.point[i][1] = (double) i;
      m_stereo3DData->data.obj.data.point[i][2] = (double) i;
    }
  m_stereo3DData->data.obj.data.color.length(outputPointNum);
  for (i = 0; i < outputPointNum; i++)
    {
      m_stereo3DData->data.obj.data.color[i][0] = (double) (i % 256);
      m_stereo3DData->data.obj.data.color[i][1] = (double) (i % 256);
      m_stereo3DData->data.obj.data.color[i][2] = (double) (i % 256);
    }

  for (i = 0; i < 4; i++)
    {
      for (j = 0; j < 4; j++)
        {
          m_stereo3DData->data.obj.data.T[i][j] = 0.0;
          if (i == j)
            {
              m_stereo3DData->data.obj.data.T[i][j] = 1.0;
            }
        }
    }

  *m_readySendImageFlag = true;
}

//
// 画像データを送信するかどうかのフラグをセットする。
//
void
Reconstruct3DServiceSVC_impl::setSendImageFlag(bool* flag)
{
  m_readySendImageFlag = flag;
}

//
// 出力する TimedStereo3D 構造体をセットする。
//
void
Reconstruct3DServiceSVC_impl::setStereo3DData(TimedStereo3D* data)
{
  m_stereo3DData = data;
}

#if 0
//
// 読み込んだ画像データを返す。
//
TimedStereo3D *
Reconstruct3DServiceSVC_impl::getStereo3DData()
{
  return &m_stereo3DData;
}
#endif

//
// 読み込む画像枚数をセット
//
void
Reconstruct3DServiceSVC_impl::setReadImageNum(int num)
{
  m_readImageNum = num;
  m_stereo3DData->data.mimg.data.image_seq.length(num);
}

//
// 出力点群数をセット
//
void
Reconstruct3DServiceSVC_impl::setOutputPointNum(int num)
{
  m_outputPointNum = num;
}

//
// 出力エラーコードをセット
//
void
Reconstruct3DServiceSVC_impl::setErrorCode(string& code)
{
  m_errorCode = code;
}

//
// キャリブレーションファイル名をセット
//
void
Reconstruct3DServiceSVC_impl::setCalibFilename(string& filename)
{
  m_calibFilename = filename;
}

//
// キャリブレーションファイル名をセット
//
void
Reconstruct3DServiceSVC_impl::setImageFilename(string& fn0, string& fn1, string& fn2)
{
  m_imageFilename0 = fn0;
  m_imageFilename1 = fn1;
  m_imageFilename2 = fn2;
}

//
// 画像ファイルの読み込み
//
int
Reconstruct3DServiceSVC_impl::loadImage()
{
  int imageNum = 0;
  int imageSize;
  int i;

  Img::CameraImage* cImage;
  Img::MultiCameraImage mcImage;

  char* filename = NULL;

  mcImage.image_seq.length(m_readImageNum);

  while (imageNum < m_readImageNum)
    {
      if (m_imageFilename0 != "" || m_imageFilename1 != "" || m_imageFilename2 != "")
        {
          switch (imageNum)
            {
              case 0:
                filename = (char*)m_imageFilename0.c_str();
                break;
              case 1:
                filename = (char*)m_imageFilename1.c_str();
                break;
              case 2:
                filename = (char*)m_imageFilename2.c_str();
                break;
            }
        }
      else
        {
          string strBuffer;
          cout << imageNum << " 枚目の読み込む画像ファイル名を入力してください。：";
          cin >> strBuffer;
          if ((strBuffer.length() == 0) && (imageNum > 0))
            {
              // 読み込み終了
              break;
            }
          filename = (char*)strBuffer.c_str();
        }

      Mat readImage = imread( filename, -1 );

      if (readImage.empty())
        {
          if (imageNum > 0)
            {
              return 0;
            }
          return VISION_FILE_OPEN_ERROR;
        }

      IplImage sourceImage = IplImage( readImage );
      cImage = &(mcImage.image_seq[imageNum]);
      cImage->image.width  = sourceImage.width;
      cImage->image.height = sourceImage.height;

      if (strncmp(sourceImage.colorModel,"GRAY",4) == 0)
        {
          cImage->image.format = Img::CF_GRAY;
        }
      else if (strncmp(sourceImage.colorModel,"RGB",3) == 0)
        {
          cImage->image.format = Img::CF_RGB;
        }
      else
        {
          cImage->image.format = Img::CF_UNKNOWN;
        }

      imageSize = sourceImage.imageSize;
      cImage->image.raw_data.length(imageSize);

      if (imageSize == 0)
        {
          // テスト用
          imageNum++;
          continue;
        }

      if (strncmp(sourceImage.channelSeq,"BGR",3))
        {
          // Gray または RGB 他？の並び
          for (i = 0; i < imageSize; i++)
            {
              cImage->image.raw_data[i] = sourceImage.imageData[i];
            }
        }
      else
        {
          // BGR の並び
          IplImage *rgbImage = cvCreateImage( cvGetSize(&sourceImage), IPL_DEPTH_8U, 3 );
          if (rgbImage == NULL)
            {
              return VISION_MALLOC_ERROR;
            }
          cvCvtColor( &sourceImage, rgbImage, CV_BGR2RGB );
          for (i = 0; i < imageSize; i++)
            {
              cImage->image.raw_data[i] = rgbImage->imageData[i];
            }
          cvReleaseImage( &rgbImage );
        }

      imageNum++;
    }

  m_stereo3DData->data.mimg.data.image_seq.length(imageNum);
  for (i = 0; i < imageNum; i++)
    {
      m_stereo3DData->data.mimg.data.image_seq[i] = mcImage.image_seq[i];
    }

  // 現在時刻をセット
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  m_stereo3DData->data.mimg.tm.sec  = currentTime.tv_sec;
  m_stereo3DData->data.mimg.tm.nsec = currentTime.tv_usec;
  m_stereo3DData->data.mimg.error_code = 0;

  return 0;
}

//
// キャリブレーションデータの読み込み
//
int
Reconstruct3DServiceSVC_impl::loadCalibrationData(char* path)
{
  char filename[256];
  char buffer[256];

  FILE* fp;
  char* p;

  double m0, m1, m2, m3, m4;

  int i, j;

  int imageNum = m_stereo3DData->data.mimg.data.image_seq.length();

  Img::CameraImage* image;

  for (i = 0; i < imageNum; i++)
    {
      sprintf(filename, "%s/CalibData.%d", path, i);
      fp = fopen(filename, "r");
      if (fp == NULL)
        {
          return -1;            // FILEOPEN_ERROR;
        }

      image = &(m_stereo3DData->data.mimg.data.image_seq[i]);

      // Size
      p = fgets(buffer, 256, fp);
      while ((buffer[0] == '#') || (buffer[0] == '\0'))
        {
          p = fgets(buffer, 256, fp);
        }

      // Matrix Element
      p = fgets(buffer, 256, fp);
      while ((buffer[0] == '#') || (buffer[0] == '\0'))
        {
          p = fgets(buffer, 256, fp);
        }

      sscanf(buffer, "%lf, %lf, %lf, %lf, %lf", &m0, &m1, &m2, &m3, &m4);

      image->intrinsic.matrix_element[0] = m0;
      image->intrinsic.matrix_element[1] = m1;
      image->intrinsic.matrix_element[2] = m2;
      image->intrinsic.matrix_element[3] = m3;
      image->intrinsic.matrix_element[4] = m4;

      // Distortion Coefficient Num
      p = fgets(buffer, 256, fp);
      while ((buffer[0] == '#') || (buffer[0] == '\0'))
        {
          p = fgets(buffer, 256, fp);
        }

      // Distortion Coefficient
      p = fgets(buffer, 256, fp);
      while ((buffer[0] == '#') || (buffer[0] == '\0'))
        {
          p = fgets(buffer, 256, fp);
        }

      sscanf(buffer, "%lf, %lf, %lf, %lf, %lf", &m0, &m1, &m2, &m3, &m4);

      image->intrinsic.distortion_coefficient.length(5);
      image->intrinsic.distortion_coefficient[0] = m0;
      image->intrinsic.distortion_coefficient[1] = m1;
      image->intrinsic.distortion_coefficient[2] = m2;
      image->intrinsic.distortion_coefficient[3] = m3;
      image->intrinsic.distortion_coefficient[4] = m4;

      // Extrinsic
      for (j = 0; j < 4; j++)
        {
          p = fgets(buffer, 256, fp);
          while ((buffer[0] == '#') || (buffer[0] == '\0'))
            {
              p = fgets(buffer, 256, fp);
            }
          sscanf(buffer, "%lf, %lf, %lf, %lf", &m0, &m1, &m2, &m3);
          image->extrinsic[j][0] = m0;
          image->extrinsic[j][1] = m1;
          image->extrinsic[j][2] = m2;
          image->extrinsic[j][3] = m3;
        }

      fclose(fp);
    }

  return 0;
}

//
// キャリブレーションデータの読み込み
//
int
Reconstruct3DServiceSVC_impl::loadCalibrationDataByOpenCV(char* path)
{
  char filename[256];

  int i, j;

  int imageNum = m_stereo3DData->data.mimg.data.image_seq.length();

  Img::CameraImage* image;

  CvFileStorage* fs;
  CvFileNode* param;

  CvMat* intrinsic_matrix = NULL;
  CvMat* distortion_coeffs = NULL;
  CvMat* rotation_vectors = NULL;
  CvMat* translation_vectors = NULL;
  CvMat* rotation_matrix = cvCreateMat(3, 3, CV_32FC1);

  for (i = 0; i < imageNum; i++)
    {
      image = &(m_stereo3DData->data.mimg.data.image_seq[i]);

      sprintf(filename, "%s/camera%d.xml", path, i);
      fs = cvOpenFileStorage(filename, 0, CV_STORAGE_READ);
      if (fs == (CvFileStorage*) NULL)
        {
          cvReleaseMat(&rotation_matrix);
          return VISION_FILE_OPEN_ERROR;
        }

      param = cvGetFileNodeByName(fs, NULL, "intrinsic");
      intrinsic_matrix = (CvMat*) cvRead(fs, param);
      if ((intrinsic_matrix == (CvMat*) NULL)
          || (intrinsic_matrix->cols != 3) || (intrinsic_matrix->rows != 3))
        {
          cvReleaseFileStorage(&fs);
          cvReleaseMat(&rotation_matrix);
          return VISION_FILE_FORMAT_ERROR;
        }

      param = cvGetFileNodeByName(fs, NULL, "distortion");
      distortion_coeffs = (CvMat*) cvRead(fs, param);
      if ((distortion_coeffs == (CvMat*) NULL)
          || (distortion_coeffs->rows != 1) || (distortion_coeffs->cols < 4))
        {
          cvReleaseMat(&intrinsic_matrix);
          cvReleaseFileStorage(&fs);
          cvReleaseMat(&rotation_matrix);
          return VISION_FILE_FORMAT_ERROR;
        }

      param = cvGetFileNodeByName(fs, NULL, "rotation");
      rotation_vectors = (CvMat*) cvRead(fs, param);
      if ((rotation_vectors == (CvMat*) NULL)
          || (rotation_vectors->rows != 1) || (rotation_vectors->cols != 3))
        {
          cvReleaseMat(&intrinsic_matrix);
          cvReleaseMat(&distortion_coeffs);
          cvReleaseFileStorage(&fs);
          cvReleaseMat(&rotation_matrix);
          return VISION_FILE_FORMAT_ERROR;
        }

      param = cvGetFileNodeByName(fs, NULL, "translation");
      translation_vectors = (CvMat*) cvRead(fs, param);
      if ((translation_vectors == (CvMat*) NULL)
          || (translation_vectors->rows != 1) || (translation_vectors->cols != 3))
        {
          cvReleaseMat(&intrinsic_matrix);
          cvReleaseMat(&distortion_coeffs);
          cvReleaseMat(&rotation_vectors);
          cvReleaseFileStorage(&fs);
          cvReleaseMat(&rotation_matrix);
          return VISION_FILE_FORMAT_ERROR;
        }

      cvReleaseFileStorage(&fs);

      image->intrinsic.matrix_element[0] = intrinsic_matrix->data.fl[0];
      image->intrinsic.matrix_element[1] = intrinsic_matrix->data.fl[1];
      image->intrinsic.matrix_element[2] = intrinsic_matrix->data.fl[4];
      image->intrinsic.matrix_element[3] = intrinsic_matrix->data.fl[2];
      image->intrinsic.matrix_element[4] = intrinsic_matrix->data.fl[5];

      image->intrinsic.distortion_coefficient.length(5);
      image->intrinsic.distortion_coefficient[0] = distortion_coeffs->data.fl[0];
      image->intrinsic.distortion_coefficient[1] = distortion_coeffs->data.fl[1];
      image->intrinsic.distortion_coefficient[2] = distortion_coeffs->data.fl[2];
      image->intrinsic.distortion_coefficient[3] = distortion_coeffs->data.fl[3];
      image->intrinsic.distortion_coefficient[4] = 0.0;
      if (distortion_coeffs->cols > 4)
        {
          image->intrinsic.distortion_coefficient[4] = distortion_coeffs->data.fl[4];
        }

      cvRodrigues2(rotation_vectors, rotation_matrix);

      // Extrinsic
      for (j = 0; j < 3; j++)
        {
          image->extrinsic[j][0] = rotation_matrix->data.fl[j * 3 + 0];
          image->extrinsic[j][1] = rotation_matrix->data.fl[j * 3 + 1];
          image->extrinsic[j][2] = rotation_matrix->data.fl[j * 3 + 2];
          image->extrinsic[j][3] = translation_vectors->data.fl[j];
        }

      image->extrinsic[3][0] = 0.0;
      image->extrinsic[3][1] = 0.0;
      image->extrinsic[3][2] = 0.0;
      image->extrinsic[3][3] = 1.0;

      cvReleaseMat(&intrinsic_matrix);
      cvReleaseMat(&distortion_coeffs);
      cvReleaseMat(&rotation_vectors);
      cvReleaseMat(&translation_vectors);
    }

  cvReleaseMat(&rotation_matrix);

  return 0;
}

void Reconstruct3DServiceSVC_impl::copy_camera_params(Img::CameraImage* dst, const CameraParameter& param)
{
  /* intrinsic */
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j <= i && j < 2; ++j) {
      dst->intrinsic.matrix_element[i*(i+1)/2+j] = param.intr.at<double>(j, i) / param.intr.at<double>(2, 2);
    }
  }

  /* distortion */
  cv::Size s = param.dist.size();
  if (s.width == 1) {
    dst->intrinsic.distortion_coefficient.length(s.height);
    for (int i = 0; i < s.height; ++i) {
      dst->intrinsic.distortion_coefficient[i] = param.dist.at<double>(i, 0);
    }
  }
  else if (s.height == 1) {
    dst->intrinsic.distortion_coefficient.length(s.width);
    for (int i = 0; i < s.width; ++i) {
      dst->intrinsic.distortion_coefficient[i] = param.dist.at<double>(0, i);
    }
  }
  else {
    dst->intrinsic.distortion_coefficient.length(4);
    for (int i = 0; i < 4; ++i) {
      dst->intrinsic.distortion_coefficient[i] = 0.0;
    }
  }

  /* extrinsic */
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      dst->extrinsic[i][j] = param.ext.at<double>(i, j);
    }
  }

}

//
// キャリブレーションデータの読み込み
//
int
Reconstruct3DServiceSVC_impl::loadMultiCalibData(char *path)
{
  cv::FileStorage fs;
  int ncamera;

  if ( fs.open(path, cv::FileStorage::READ) == true )
    {
      ncamera = fs["num_cameras"];
      //std::cerr << "ncamera: " << ncamera << std::endl;

      /* read camera parameters */
      m_params.resize(ncamera);

      char key[32];
      int i;

      for (i = 0; i < ncamera; ++i)
        {
          snprintf(key, sizeof (key), "camera%d_intr", i);
          cv::read(fs[key], m_params[i].intr);

          snprintf(key, sizeof (key), "camera%d_dist", i);
          cv::read(fs[key], m_params[i].dist);

          snprintf(key, sizeof (key), "camera%d_ext", i);
          cv::read(fs[key], m_params[i].ext);
        }

      Img::CameraImage* image;

      for (i = 0; i < ncamera; i++)
        {
          image = &(m_stereo3DData->data.mimg.data.image_seq[i]);
          copy_camera_params(image, m_params[i]);
        }

#if 0
      // 読み込んだキャリブレーションデータのデバッグ用確認
      // また, テスト用キャリブレーションデータフォーマットへの変換
      FILE* fp = NULL;
      char filename[20] = {0};
      for (i = 0; i < ncamera; i++)
        {
          int j, k, ndist, wd, ht;
          image = &(m_stereo3DData->data.mimg.data.image_seq[i]);
          wd = image->image.width;
          ht = image->image.height;
          snprintf( filename, sizeof(filename), "CalibData.%d", i );
          fp=fopen( filename, "w" );
          fprintf( fp, "# size\n%d, %d\n", wd, ht );
          fprintf( fp, "# Matrix Element\n" );
          for( j = 0; j < 5; j++ )
            fprintf( fp, "%f, ", image->intrinsic.matrix_element[j] );
          ndist = image->intrinsic.distortion_coefficient.length();
          fprintf( fp, "\n# Distortion Coefficient Num\n%d", ndist );
          fprintf( fp, "\n# Distortion Coefficient\n" );
          for( j = 0; j < ndist; j++ )
            fprintf( fp, "%f, ", image->intrinsic.distortion_coefficient[j] );
          fprintf( fp, "\n# Extrinsic\n" );
          for( j = 0; j < 4; ++j )
            {
              for ( k = 0; k < 4; ++k )
                {
                  fprintf( fp, "%f, ", image->extrinsic[j][k] );
                }
              fprintf( fp, "\n" );
            }
          fprintf( fp, "\n" );
          fclose( fp );
        }
#endif

    }
  else
    {
      return -1;
    }

  return 0;
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
  // Please insert your code here
  return 0;
}

void
RecognitionServiceSVC_impl::setModelID(CORBA::Long ModelID)
{
  // Please insert your code here
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
void
RecognitionResultViewerServiceSVC_impl::display(const Img::TimedMultiCameraImage& frame,
                                                TimedRecognitionResult pos)
{
  // Please insert your code here
}



// End of example implementational code
