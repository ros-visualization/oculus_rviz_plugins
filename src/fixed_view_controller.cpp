/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include <stdint.h>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreViewport.h>

#include "rviz/display_context.h"
#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"

#include "oculus_rviz_plugins/fixed_view_controller.h"

using namespace rviz;

namespace oculus_rviz_plugins
{

static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
  Ogre::Quaternion( Ogre::Radian( -Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_Y ) *
  Ogre::Quaternion( Ogre::Radian( -Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_Z );

FixedViewController::FixedViewController()
{
  fov_property_ = new FloatProperty( "Vertical Field Of View", 50.0, "Vertical opening angle of the camera.", this );
  fov_property_->setMin( 5.0 );
  fov_property_->setMax( 130.0 );
}

void FixedViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();
  camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
  camera_->setOrientation(ROBOT_TO_CAMERA_ROTATION);//.Inverse());
}

FixedViewController::~FixedViewController()
{
}

void FixedViewController::reset()
{
}

void FixedViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update( dt, ros_dt );
  camera_->setFOVy( Ogre::Degree(fov_property_->getFloat()) );
  target_scene_node_->setOrientation( reference_orientation_ );
}

void FixedViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  setStatus( "<b>Right-Click / Mouse Wheel:</b>: Zoom.  " );

  int32_t diff_x = event.x - event.last_x;
  int32_t diff_y = event.y - event.last_y;

  if( event.right() )
  {
    setCursor( Zoom );
    zoom( -diff_y * 0.1);
  }

  if( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;
    zoom( diff * 0.001 );
  }
}

void FixedViewController::zoom( float amount )
{
  fov_property_->setFloat( fov_property_->getFloat() * (1.0 + amount) );
}


} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( oculus_rviz_plugins::FixedViewController, rviz::ViewController )
