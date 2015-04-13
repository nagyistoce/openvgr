# multicalib #

multicalib is a program to estimate camera parameters of a multi-camera system. It comes with genconf and ichimatsu.
multicalib can calibrate even a single camera, in that case, the result is (almost) identical to the one by the calibration function in OpenCV.

# Calibration steps #

## 1. Generate a camera setting file by genconf ##

genconf generates a template of setting file for your cameras on IEEE1394 buses.
```
$ ./genconf > ieee1394board.0
```

The content of the generated file may look like:
```
0 3
0x0011223344556677 640x480-Y(mono) 30fps BRIGHTNESS 0 AUTO_EXPOSURE -1 SHARPNESS -1 GAMMA 1024 SHUTTER 650 GAIN 250 PAN -1 TILT -1
0x0011223344556678 640x480-Y(mono) 30fps BRIGHTNESS 0 AUTO_EXPOSURE -1 SHARPNESS -1 GAMMA 1024 SHUTTER 650 GAIN 250 PAN -1 TILT -1
0x0011223344556679 640x480-Y(mono) 30fps BRIGHTNESS 0 AUTO_EXPOSURE -1 SHARPNESS -1 GAMMA 1024 SHUTTER 650 GAIN 250 PAN -1 TILT -1
```

The listing order depends on a searching process of genconf. You can edit this file manually. Especially, you need confirm image size, color format, and fps for each camera not to exceed the bandwidth of your 1394 bus.

You can check your settings by running ichimatsu with no options:
```
$ ./ichimatsu
```

It will read the ieee1394board.0 in a current directory, and show live images taken by your cameras. To exit the program, press 'q'.


## 2. Observe a reference plane ##

ichimatsu has another mode to detect a [chessboard pattern](http://openvgr.googlecode.com/svn/trunk/src/module/tool/multicalib/chessboard_8x8_20mm.svg) (by a function in OpenCV).
```
$ ./ichimatsu -g 16x16 -s 20 -o calib_data.txt
```

where, number of grid in each row/col (-g), length of a side of square `[mm]` (-s), and file name of calibration data to be output (-o).

When launching the program with a correct setting file, you may see live images taken by the cameras. As pressing "c" key, the program will enter the chessboard detection mode. Then, move your reference plane to be detected for all camera image, and press "enter" key to store the data. You shold repeate this "move -> enter" process at least three times to gather sufficient information for calibration. I usually observe a reference plane about 5 times. Finally, press "W" (shift-w) to output calibration data.

The key point of the observation is normal vector of a reference plane. The set of observed normal vectors must span 3-D space. For instance, rotational motion around one axis is insufficient because all of the normal vectors reside on a 2-D plane. So, if you receive poor result from multicalib, please check poses of a reference plane.

## 3. Generate a camera parameter file by multicalib ##

```
$ ./multicalib calib_data.txt
```

will output result to console. If you specify file name by -o option (e.g. -o calib.yaml), the result will be written to the file in yaml format.

# Calibration data format #

You can use multicalib in conjunction with your plane detection programs. The format multicalib reads is as follows.
```
MLTCLB 1
(pattern_period) (pattern_rotation_period)
(# observations) (# cameras)
(# observed points by 1st camera at 1st observation)
(plane_coordinate_x [mm]) (plane_coordinate_y [mm]) (image_coordinate_x) (image_coordinate_y)
:
(# observed points by 2nd camera at 1st observation)
:
(# observed points by n-th camera at 1st observation)
:
(# observed points by 1st camera at 2nd observation)
```

# Compilation without OpenRTM #

multicalib only depends on the OpenCV. So you can build it without OpenRTM.

## Using makefiles ##

```
$ cd ${TOPDIR}/src/lib/libcapture && make
$ cd ${TOPDIR}/src/module/tool/multicalib && make
```
Where `${TOPDIR}` stands for the top directory of the OpenVGR (i.e. contains Copyright, LISENCE and so forth).

If you don't have `libdc1394-22-dev`, you can only build the multicalib.
```
$ cd ${TOPDIR}/src/module/tool/multicalib && make multicalib
```

## Using cmake ##

Cmake may automatically do the same thing above mentioned.
```
$ mkdir ${TOPDIR}/release && cd ${TOPDIR}/release
$ cmake .. && make
```