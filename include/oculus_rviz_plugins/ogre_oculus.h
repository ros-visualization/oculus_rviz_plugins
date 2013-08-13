/// Copyright (C) 2013 Kojack
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.

#pragma once

#include "OGRE/OgreQuaternion.h"
#include "OGRE/OgreVector3.h"

#include <string>

namespace OVR
{
class HMDDevice;
class SensorFusion;
class DeviceManager;
class SensorDevice;

namespace Util
{
class MagCalibration;
namespace Render
{
class StereoConfig;
}
}
}

namespace Ogre
{
class SceneManager;
class RenderWindow;
class Camera;
class SceneNode;
class Viewport;
class CompositorInstance;
}

namespace oculus_rviz_plugins
{

class Oculus
{
public:
  Oculus(void);
  ~Oculus(void);
  bool setupOculus();
  bool setupOgre(Ogre::SceneManager *sm, Ogre::RenderWindow *win, Ogre::SceneNode *parent = 0);
  void shutDownOculus();
  void shutDownOgre();
  bool isOgreReady() const;
  bool isOculusReady() const;

  /// Update camera node using current Oculus orientation.
  void update();

  /// Reset orientation of the sensor.
  void resetOrientation();

  /// Retrieve the SceneNode that contains the two cameras used for stereo rendering.
  Ogre::SceneNode *getCameraNode();

  /// Retrieve the current orientation of the Oculus HMD.
  Ogre::Quaternion getOrientation() const;

  /// Retrieve either of the two distortion compositors.
  Ogre::CompositorInstance *getCompositor(unsigned int i);

  /// Retrieve either of the two cameras.
  Ogre::Camera *getCamera(unsigned int i)
  {
    return m_cameras[i % 2];
  }

  /// Retrieve either of the two viewports.
  Ogre::Viewport *getViewport(unsigned int i)
  {
    return m_viewports[i % 2];
  }

  /// Retrieve the projection centre offset.
  float getCentreOffset() const;

  const OVR::HMDDevice *getHMDDevice()
  {
    return m_hmd;
  }

  /// Re-create projection matrices based on camera parameters
  void updateProjectionMatrices();

  /// Set motion prediction time window (0 to disable)
  void setPredictionDt(float dt);

  /// Get status of mag calibration
  bool isMagCalibrated();

protected:
  OVR::DeviceManager *m_deviceManager;
  OVR::HMDDevice *m_hmd;
  OVR::Util::Render::StereoConfig *m_stereoConfig;
  OVR::Util::MagCalibration *m_magCalibration;
  OVR::SensorDevice *m_sensor;
  OVR::SensorFusion *m_sensorFusion;
  bool m_oculusReady; /// Has the oculus rift been fully initialised?
  bool m_ogreReady; /// Has ogre been fully initialised?
  Ogre::SceneManager *m_sceneManager;
  Ogre::RenderWindow *m_window;
  Ogre::SceneNode *m_cameraNode;
  Ogre::Quaternion m_orientation;
  float m_centreOffset; /// Projection centre offset.
  Ogre::Camera *m_cameras[2];
  Ogre::Viewport *m_viewports[2];
  Ogre::CompositorInstance *m_compositors[2];
};

}
