oculus_rviz_plugins
===========

Contains the Oculus Display for RViz. To build it, you will need the [Oculus SDK for Linux 0.2.3](http://developer.oculusvr.com).

This plugin is based on [OgreOculus](https://bitbucket.org/rajetic/ogreoculus) (C++)
and [OsgOculusViewer](https://github.com/bjornblissing/osgoculusviewer) (GLSL shaders).

Build Instructions
==================

If you have installed the SDK in your home folder, build using:

```
catkin_make -DOCULUS_SDK_ROOT_DIR=~/OculusSDK/
```

Usage Instructions
==================

In RViz, add the "OculusDisplay". This will create an additional window with the Oculus View
rendered from the same point of view as the main rendering area. Check "Render to Oculus" to 
render in full screen mode on your Oculus headset. It must be registered as secondary screen
for this to work.

The Oculus view will be rendered from the same position as the main RViz camera while following
your head's orientation.

This is how the Display should look like in windowed mode:

![ScreenShot](doc/screenshot.png)
