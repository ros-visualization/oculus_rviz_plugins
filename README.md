rviz_oculus
===========

Contains the Oculus Display for RViz. To build, you need the Oculus SDK.

This plugin was developed using code from https://bitbucket.org/rajetic/ogreoculus 
and https://github.com/bjornblissing/osgoculusviewer.

Build Instructions
==================

If you have installed the SDK in your home folder, build using:

```
catkin_make -DOCULUS_SDK_ROOT_DIR=~/OculusSDK/
```

This is how the Display should look in windowed mode:

![ScreenShot](doc/screenshot.png)
