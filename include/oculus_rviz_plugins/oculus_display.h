/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_OCULUS_DISPLAY_H
#define RVIZ_OCULUS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#endif

#include <QObject>

#include <OGRE/OgreRenderTargetListener.h>

#include "rviz/display.h"

namespace Ogre
{
class SceneNode;
class RenderWindow;
}

namespace rviz
{
class BoolProperty;
class StringProperty;
class RenderWidget;
class FloatProperty;
class VectorProperty;
class TfFrameProperty;
}

namespace oculus_rviz_plugins
{

class Oculus;

/**
 * \class OculusDisplay
 *
 */
class OculusDisplay: public rviz::Display, public Ogre::RenderTargetListener
{
Q_OBJECT
public:
  OculusDisplay();
  virtual ~OculusDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

  // Overrides from Ogre::RenderTargetListener
  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

protected:

  virtual void onEnable();
  virtual void onDisable();

  void updateCamera();

protected Q_SLOTS:

  void onFullScreenChanged();
  void onPredictionDtChanged();
  void onPubTfChanged();
  void onFollowCamChanged();

  void onScreenCountChanged( int newCount );

private:

  rviz::BoolProperty *fullscreen_property_;
  rviz::FloatProperty *prediction_dt_property_;

  rviz::BoolProperty *pub_tf_property_;
  rviz::StringProperty *pub_tf_frame_property_;

  rviz::BoolProperty *follow_cam_property_;
  rviz::BoolProperty *horizontal_property_;
  rviz::TfFrameProperty *tf_frame_property_;
  rviz::VectorProperty *offset_property_;

  rviz::FloatProperty *near_clip_property_;

  rviz::RenderWidget *render_widget_;
  Ogre::SceneNode *scene_node_;

#ifndef Q_MOC_RUN
  tf::TransformBroadcaster tf_pub_;
  boost::shared_ptr<Oculus> oculus_;
#endif
};

} // namespace rviz

#endif

