# What's new from 0.9.0 #

This is a maintenance release of the 0.9.x series. Remarkable changes from the 0.9.0 are shown below.
To view the details, please check a log of [our repository](http://code.google.com/p/openvgr/source/list).

## Recognition RTCs ##
  * Accuracy of line fitting has been slightly improved.
  * Algorithm for generating vertex features from pairs of lines has been changed. Vertices at T-junctions are available.

## Calibration related programs ##
  * Modified to use C++ apis (`cv::*`) rather than C apis (`Cv*`).
  * Fixed the bug in multicalib; -d option is not correctly handled.

## General ##
  * Added an experimental support for cmake.
```
$ mkdir release && cd release
$ cmake .. && make
```
  * Fixed misc bugs.