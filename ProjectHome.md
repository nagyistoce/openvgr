# About #
OpenVGR was a part of the [NEDO project (P08013)](http://www.nedo.go.jp/activities/EP_00204.html).
The source code is [available](http://code.google.com/p/openvgr/source/browse/trunk/) under the EPL-1.0 license.

<img src='http://unit.aist.go.jp/is/vmrg/ci/rtm/OpenVGR.png' />

OpenVGR consists of the following RT-components (based on OpenRTM-1.0):
  * a stereo camera capture (for IEEE 1394b camera),
  * a stereo image viewer,
  * a 3-D point cloud reconstruction (using OpenCV), and
  * an edge-based 3-D object detection

And the following command line tools:
  * model creation, and
  * [multi-camera calibration](Multicalib.md).

# News #
|2012-03-05|OpenVGR-0.9.1 released ([release note](ReleaseNote_0_9_1.md)).|
|:---------|:-------------------------------------------------------------|
|2011-12-21|OpenVGR-0.9.0 released ([release note](ReleaseNote_0_9_0.md)).|
|2011-09-09|Subversion repository [available](http://code.google.com/p/openvgr/source/browse/)|

# Requirements #
## OS ##
  * [Ubuntu 10.04 LTS (x86)](http://www.ubuntu.com/) or
  * Ubuntu 12.04 LTS (x86, experimental)

## Libraries ##
| | [OpenCV-2.0](http://opencv.willowgarage.com/wiki/) | [libdc1394-2](http://damien.douxchamps.net/ieee1394/libdc1394/) | [OpenRTM-1.0](http://www.openrtm.org/) |
|:|:---------------------------------------------------|:----------------------------------------------------------------|:---------------------------------------|
| RT-components | required | required | required |
| modeling tool | required |  |  |
| capturing tools | required | required |  |
| camera calibration tool | required |  |  |

# Documents #
A few articles are available at [wiki page](http://code.google.com/p/openvgr/w/list).

Links to PDFs in the source tree (Japanese):
  * [Brief overview](http://openvgr.googlecode.com/svn/trunk/doc/user/%E3%81%AF%E3%81%98%E3%82%81%E3%81%AB%E3%81%8A%E8%AA%AD%E3%81%BF%E3%81%8F%E3%81%A0%E3%81%95%E3%81%84.pdf)
  * [User manual](http://openvgr.googlecode.com/svn/trunk/doc/user/%E6%93%8D%E4%BD%9C%E6%89%8B%E9%A0%86%E6%9B%B8.pdf)
  * [Specs of RT-components](http://openvgr.googlecode.com/svn/trunk/doc/developer/%E6%A9%9F%E8%83%BD%E4%BB%95%E6%A7%98%E6%9B%B8.pdf)

<a href='Hidden comment: 
<wiki:gadget url="http://www.google.com/ig/modules/chinagadgets/worldclock/worldclock.xml" up_default_locations="9,0" up_am_pm="1" />
'></a>