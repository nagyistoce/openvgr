# What's new from 0.8.1 #

Remarkable changes from 0.8.1 are shown below. To view the details, please check a log of [our repository](http://code.google.com/p/openvgr/source/list).

## Recognition RTCs ##
  * Threshold value for edge detection (EdgeStrength) has been normalized
    * In the previous version, user need to adjust the value when changes edge detection methods (EdgeDetectFunction). Now, magnitude of the value is 1/16 for EdgeDetectFunction = 0, and 1/96 for EdgeDetectFunction = 1.
  * Evaluation performance has been slightly improved.
  * An experimental OpenMP support has been added.

## Calibration related programs ##
  * A name of a sample program for chessboard pattern detection has been changed from 'sample' to 'ichimatsu'.
  * An option character of ichimatsu for specifying an output file name (-w) is obsoleted, use '-o' instead.
  * A small program for resetting IEEE1394 bus was added.

## General ##
  * Fixed misc bugs.