/*
 cutroi.cpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)
*/
#include <highgui.h>
#include <cv.h>
#include <iostream>
#include <getopt.h>

using namespace std;
using namespace cv;

void MouseEventHandler( int event, int x, int y, int flags, void* param );
enum State { NONE, HALFWAY, SELECTED };

class ROIholder
{
public:
  ROIholder();
  ~ROIholder();
  bool loadSource( char* filename );
  void displaySource();
  void initializeRect( int x, int y );
  void updateRect( int x, int y );
  void saveRect( char* filename );
  enum State state;
private:
  Rect rect;
  Mat source;
  Mat display;
  const char* windowTitle;
};

ROIholder::ROIholder()
{
  windowTitle = "cutroi";
  namedWindow( windowTitle, CV_WINDOW_AUTOSIZE );
  cvSetMouseCallback( windowTitle, MouseEventHandler, this );
}

ROIholder::~ROIholder()
{
}

bool ROIholder::loadSource( char* filename )
{
  source = imread( filename, 1 );
  if ( source.empty() )
    {
      cerr << "cannot read source image." << endl;
      return false;
    }
  return true;
}

void ROIholder::displaySource()
{
  state = NONE;
  source.copyTo( display );
  imshow( windowTitle, display );
}

void ROIholder::initializeRect( int x, int y )
{
  rect = Rect( x, y, 1, 1 );
}

void ROIholder::updateRect( int x, int y )
{
  const Scalar GREEN = Scalar(0,255,0);
  Point upperLeft, lowerRight;

  upperLeft  = Point( rect.x, rect.y );
  lowerRight = Point( x, y );
  rect = Rect( upperLeft, lowerRight );
  source.copyTo( display );
  rectangle( display, upperLeft, lowerRight, GREEN, 2 );
  imshow( windowTitle, display );
}

void ROIholder::saveRect( char* filename )
{
  Mat img = source( rect );
  imwrite( filename, img );
  cout << "x : " << rect.x << " y : " << rect.y << endl;
  cout << "w : " << rect.width << " h : " << rect.height << endl;
  state = NONE;
}

void MouseEventHandler( int event, int x, int y, int flags, void* param )
{
  ROIholder& roi = *(ROIholder *)param;
  switch ( event )
    {
      case CV_EVENT_LBUTTONDOWN:
        if ( roi.state == NONE )
          {
            roi.initializeRect( x, y );
            roi.state = HALFWAY;
          }
        break;
      case CV_EVENT_LBUTTONUP:
        if ( roi.state == HALFWAY )
          {
            roi.updateRect( x, y );
            roi.state = SELECTED;
          }
        break;
      case CV_EVENT_MOUSEMOVE:
        if ( roi.state == HALFWAY )
          {
            roi.updateRect( x, y );
          }
        break;
    }
}

void printUsage( char** argv )
{
  char *progname = strrchr( argv[0], '/' );
  if ( progname == NULL )
    {
      progname = argv[0];
    }
  else
    {
      ++progname;
    }
  cerr << "Usage : " << progname << " -i sourceImage -o roiImage" << endl;
}

typedef struct
{
  char* srcfile;
  char* outfile;
} OptParam;

int parseOption( int argc, char** argv, OptParam& opt )
{
  bool errflg = false;
  int ch;

  while ( (ch=getopt(argc,argv,"i:o:h")) != -1 )
    {
      switch ( ch )
        {
          case 'i': // source image filename
            opt.srcfile = optarg;
            break;
          case 'o': // output image filename (ROI)
            opt.outfile = optarg;
            break;
          case 'h': // help
          default:
            printUsage( argv );
            errflg = true;
          break;
        }
      if ( errflg )
        {
          return -1;
        }
    }

  if ( opt.srcfile == NULL )
    {
      cerr << "source image filename required." << endl;
      errflg = true;
    }

  if ( opt.outfile == NULL )
    {
      cerr << "output filename required." << endl;
      errflg = true;
    }

  if ( errflg )
    {
      return -1;
    }

  return 0;
}

int main( int argc, char** argv )
{
  OptParam opt = {0};

  if ( parseOption( argc, argv, opt ) != 0 )
    {
      return -1;
    }

  ROIholder roi;

  if ( roi.loadSource( opt.srcfile ) == false )
    {
      return -1;
    }

  cout << "Operations: \n"
        "\tleft mouse button - set a rectangle\n"
        "\tc - cut out the rectangle\n"
        "\tr - restore the source image\n"
        "\tESC - quit the program\n"
        "\n";

  roi.displaySource();

  while ( true )
    {
      int key = waitKey( 0 );
      switch( (char)key )
        {
          case '\x1b': // ESC key
            return 0;
          case 'c':
            roi.saveRect( opt.outfile );
            break;
          case 'r':
            roi.displaySource();
            break;
        }
    }

  // end of the main program
}
