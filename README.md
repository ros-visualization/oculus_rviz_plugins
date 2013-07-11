rviz_oculus
===========

Contains the Oculus Display for RViz. To build, you will need the [Oculus SDK](developer.oculusvr.com).

This plugin is based on [OgreOculus](https://bitbucket.org/rajetic/ogreoculus) (C++)
and [OsgOculusViewer](https://github.com/bjornblissing/osgoculusviewer) (GLSL shaders).

Build Instructions
==================

If you have installed the SDK in your home folder, build using:

```
catkin_make -DOCULUS_SDK_ROOT_DIR=~/OculusSDK/
```

This is how the Display should look in windowed mode:

![ScreenShot](doc/screenshot.png)
