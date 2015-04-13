# Introduction #

OpenVGR is a program suite for object detection by stereo vision.
The object detection is performed by comparing a model and 3-D lines or curves reconstructed by stereo vision.
Since major programs are built as RT components (function units running on [OpenRTM](http://www.openrtm.org)), user can easily use our programs with/without connecting another components.

OpenVGR consists of the followings:
  * RT components (for OpenRTM-1.0)
    * MultiCamera
      * takes images simultaneously using IIDC 1.31 compliant camera (e.g. Point Grey Flea2), and outputs the data as Img::TimedMultiCameraImage.
    * MultiDisp
      * receives Img::TimedMultiCameraImage, shows it using OpenCV's function (cv::imshow()).
    * Measure3D
      * receives Img::TimedMultiCameraImage, and outputs a point cloud and stereo images as TimedStereo3D.
    * Recognition
      * receives TimedStereo3D, and outputs the result of object detection as 4x4 coordinate transform matrices and additional informations.
    * RecognitionResultViewer
      * shows the top scored result with green contours by projecting a model onto camera images.
  * command line tools
    * VGRModeler
      * produces a simple box or cylinder model for the recognition.
    * multicalib
      * genconf generates a template configuration file for cameras.
      * ichimatsu generates a data file for the multicalib by detecting chessboard pattern using the OpenCV's function (cv::findChessboardCorners()).
      * multicalib computes the calibration data of multi-cameras from the data file. Camera parameters in the calibration data are compatible to OpenCV's camera model.


# Prerequisites #

Currently, we are developing on Ubuntu 10.04 LTS (x86), and using libraries that can be installed by the apt-get command, except for OpenRTM-1.0.
As far as we know, APIs of OpenCV-2.0 are slightly different from the latest version.
So when you try to build the programs with different versions of libraries, you may experience compilation or linking error.

## OS ##
  * [Ubuntu 10.04 LTS (x86)](http://www.ubuntu.com/)

## Libraries ##
  * [OpenRTM-aist-1.0.0](http://www.openrtm.org/) (required by RT components)
  * [OpenCV-2.0](http://opencv.willowgarage.com/wiki/) (required by the all programs)
  * [libdc1394-2](http://damien.douxchamps.net/ieee1394/libdc1394/) (required by capture related programs)

## Hardware ##
  * two or three IEEE 1394b cameras. (some sample images are included for testing)


# Installing dependencies #
We use apt-get here, you can use your favorite package manager (e.g. synaptic), of course.

## OpenRTM-aist-1.0.0 , rtshell 3.0 ##
Please follow the instructions at [official page](http://openrtm.org).

## OpenCV-2.0 ##
for Ubuntu 10.04 LTS:
```
$ sudo apt-get install lib{cv,cvaux,highgui}-dev
```

## libdc1394-2 ##
for Ubuntu 10.04 LTS:
```
$ sudo apt-get install libdc1394-22-dev
```


# Downloading package #
from subversion:
```
$ svn co http://openvgr.googlecode.com/svn/trunk/ somewhere
```

tarball:<br>
Go to <a href='http://code.google.com/p/openvgr/downloads/list'>download page</a> and choose one.<br>
Then unpack the archive:<br>
<pre><code>$ tar zxvf OpenVGR-X.Y.Z.tgz<br>
</code></pre>

<h1>Building programs</h1>
<pre><code>$ cd ${OPENVGR_HOME}<br>
$ make<br>
</code></pre>
where ${OPENVGR_HOME} is the top directory of the package.<br>
<br>
After the compilation finishes, the following commands create soft links in the build directory.<br>
<pre><code>$ cd ${OPENVGR_HOME}/build; make<br>
</code></pre>


<h1>Testing</h1>
Go to ${OPENVGR_HOME}/example/script, then execute ./stilltest.sh.<br>
<pre><code>$ cd ${OPENVGR_HOME}/example/script<br>
$ ./stilltest.sh<br>
</code></pre>
This script launches RT components, connects each other, and inputs test images to the component.<br>
After a while, you will get a recognition result for the input test image.