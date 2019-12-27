# High-speed vision examples utilizing OpenCV

This project holds supplementary source files for the following
tutorial article: 

- Shingo Kagami: [Utilizing OpenCV for High-Speed Vision Processing](https://www.jstage.jst.go.jp/article/jrsj/31/3/31_31_244/_article/), Journal of the Robotics Society of Japan, 31(3), pp. 244-248, 2013 (in Japanese).

The objective is to present a sample implementation of high frame rate
vision processing with OpenCV, in which a separate GUI thread is
arranged so that it does not disturb a higher-rate vision processing
thread as best it can.  The implementation runs a target tracking
demonstration at over 450 fps with a Point Grey Research USB 3.0
camera and a note PC.


## OVERVIEW

Basically, the two (or more) threads run independently while the
communication between them is done in a message-passing manner.  The
message passing link is designed with the policies and assumptions
that (1) overhead of data copy is minimized, (2) the slower GUI thread
should not disturb the faster vision processing thread, and (3) a
message may be dropped if the slower receiving thread cannot handle
it.

The project contains three modules: 

- `msglink`: a simple thread-to-thread message-passing utility designed
with the above policies.

- `multi_threaded_cap`: a sample program using msglink that simply
captures and displays a video sequence from a standard web camera
(which OpenCV can recognize). 

- `target_track`: a sample program using msglink that demonstrates a
template-match tracking with a Point Grey Research camera (which
the Point Grey FlyCapture driver can recognize). 



## DEPENDENCY

- `msglink` depends on Boost (tested with Boost 1.47.0)

- `multi_threaded_cap` depends on OpenCV 2.x (tested with OpenCV 2.3.1),
Boost and msglink.

- `target_track depends` on OpenCV, Boost, FlyCapture (tested with
FlyCapture2-2.3.3.19) and msglink.

They are tested in Windows 7 Professional (x64) with Visual Studio
2008, but are not meant to be dependent on the Windows platform. 


## BUILD AND RUN

Make sure the above prerequisite libraries are installed and the
include/library paths are appropriately set.  Note that we are using
the old FlyCapture 1.x API and thus FC1 directory of the FlyCapture
library needs to be in the library path. 

If you are using Visual Studio, start `highspeed_cam_opencv.sln` to open
the solution.  The projects for `multi_threaded_cap` and `target_track`
are in it, for which the following settings are already done. 

### multi_threaded_cap: 

- Link with `opencv_core***.lib`, `opencv_imgproc***.lib`, and
`opencv_highgui***`.lib (where `***` are the version number). 

- Put the path where `msglink.hpp` lies into the additional include
  directory (e.g. `../msglink`). 

- When run, it requires a camera recognizable by OpenCV. 

- Hit any key to quit. 

### target_track: 

- Link with `opencv_core***.lib`, `opencv_imgproc***.lib`,
`opencv_highgui***.lib` (where `***` are the version number), and
`PGRFlyCapture.lib`.

- Put the path where `msglink.hpp` lies into the additional include
directories setting (e.g. `../msglink`). 

- When run, it requires a monochrome Point Grey Research camera
recognizable by the FlyCapture driver.  To use another camera (say,
a color camera), see the comment around the class
`VideoCaptureFlyCap` in `target_track.cpp`. 

- Left click to choose (the center of) a new target. 

- Hit any key to quit. 
