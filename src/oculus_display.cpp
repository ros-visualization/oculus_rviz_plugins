/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <string>
#include <iostream>

#include <QWidget>
#include <QDesktopWidget>
#include <QApplication>

#include <OVR.h>

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>

#include <ros/package.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>

#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>

#include "rviz_oculus/oculus_display.h"
#include "rviz_oculus/ogre_oculus.h"

namespace rviz_oculus
{

OculusDisplay::OculusDisplay()
: render_widget_(0)
, scene_node_(0)
, oculus_(0)
{
  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);

  fullscreen_property_ = new rviz::BoolProperty( "Render to Oculus", false,
    "If checked, will render fullscreen on your secondary screen. Otherwise, shows a window.",
    this, SLOT(onFullScreenChanged()));

  horizontal_property_ = new rviz::BoolProperty( "Fixed Horizon", true,
    "If checked, will ignore the pitch component of the RViz camera.", this);

  pub_tf_property_ = new rviz::BoolProperty( "Publish tf", true,
    "If checked, will publish the pose of the Oculus camera as a tf frame.",
    this );

  tf_frame_property_ = new rviz::StringProperty( "Tf Frame", "oculus",
    "Name of the tf frame.", this );

  prediction_dt_property_ = new rviz::FloatProperty( "Motion prediction (ms)", 30.0,
    "Time in ms to predict head motion. Decreases overall latency and motion sickness.",
    this, SLOT(onPredictionDtChanged()) );

  connect( QApplication::desktop(), SIGNAL( screenCountChanged ( int ) ), this, SLOT( onScreenCountChanged(int)) );
}

OculusDisplay::~OculusDisplay()
{
  delete oculus_;
  render_widget_->close();
}

void OculusDisplay::onPredictionDtChanged( )
{
  if ( oculus_ )
  {
    oculus_->setPredictionDt( prediction_dt_property_->getFloat() * 0.001 );
  }
}


void OculusDisplay::onScreenCountChanged( int newCount )
{
  if ( newCount == 1 )
  {
    fullscreen_property_->setBool(false);
    fullscreen_property_->setHidden(true);
    setStatus( rviz::StatusProperty::Error, "Oculus Screen", "No secondary screen detected. Cannot render to Oculus device.");
  }
  else
  {
    fullscreen_property_->setHidden(false);
    setStatus( rviz::StatusProperty::Ok, "Oculus Screen", "Using screen #2.");
  }
}


void OculusDisplay::onFullScreenChanged()
{
  if ( fullscreen_property_->getBool() && QApplication::desktop()->numScreens() > 1 )
  {
    QRect screen_res = QApplication::desktop()->screenGeometry(1);
    //render_widget->setWindowFlags();
    render_widget_->setGeometry( screen_res );
    //render_widget->show();
    render_widget_->showFullScreen();
  }
  else
  {
    int x_res = 1280;
    int y_res = 800;
    if ( oculus_ )
    {
      OVR::HMDInfo info;
      oculus_->getHMDDevice()->GetDeviceInfo( &info );
      x_res = info.HResolution;
      y_res = info.VResolution;
    }
    int primary_screen = QApplication::desktop()->primaryScreen();
    QRect screen_res = QApplication::desktop()->screenGeometry( primary_screen );
    render_widget_->setGeometry( screen_res.x(), screen_res.y(), x_res, y_res );
    render_widget_->showNormal();
  }
}

void OculusDisplay::onInitialize()
{
  render_widget_ = new rviz::RenderWidget( rviz::RenderSystem::get() );
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle( "Oculus View" );

  render_widget_->setParent( context_->getWindowManager()->getParentWindow() );
  render_widget_->setWindowFlags( Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMaximizeButtonHint );

  Ogre::RenderWindow *window = render_widget_->getRenderWindow();
  window->setVisible(true);
  window->setAutoUpdated(true);
  window->addListener(this);

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  oculus_ = new Oculus();
  onEnable();

  onScreenCountChanged( QApplication::desktop()->numScreens() );
  onFullScreenChanged();
  onPredictionDtChanged();

  oculus_->setupOgre( scene_manager_, window, scene_node_ );

  update(0,0);
}


void OculusDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  updateCamera();
}

void OculusDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
}

void OculusDisplay::onEnable()
{
  if ( oculus_ )
  {
    oculus_->setupOculus();
    render_widget_->setVisible( oculus_->isOculusReady() );
  }
}

void OculusDisplay::onDisable()
{
  render_widget_->setVisible(false);
  if ( oculus_ )
  {
    oculus_->shutDownOculus();
  }
}

void OculusDisplay::update( float wall_dt, float ros_dt )
{
  updateCamera();
}

void OculusDisplay::updateCamera()
{
  if (!oculus_->isOculusReady())
  {
    return;
  }

  const Ogre::Camera *cam = context_->getViewManager()->getCurrent()->getCamera();
  Ogre::Vector3 pos = cam->getDerivedPosition();
  scene_node_->setPosition( pos );

  Ogre::Quaternion ori = cam->getDerivedOrientation();

  if ( horizontal_property_->getBool() )
  {
    Ogre::Vector3 x_axis = ori * Ogre::Vector3(1,0,0);
    float yaw = atan2( x_axis.y, x_axis.x );// - M_PI*0.5;

    // we're working in OpenGL coordinates now
    ori.FromAngleAxis( Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z );

    Ogre::Quaternion r;
    r.FromAngleAxis( Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_X );
    ori = ori * r;
  }

  scene_node_->setOrientation( ori );

  Ogre::ColourValue bg_color = context_->getViewManager()->getRenderPanel()->getViewport()->getBackgroundColour();

  for ( int i =0; i<2; i++ )
  {
    oculus_->getViewport(i)->setBackgroundColour( bg_color );
    oculus_->getCamera(i)->setNearClipDistance( cam->getNearClipDistance() );

    // this is a hack to circumvent a bug in Ogre 1.8
    // otherwise one of the viewports will not update it's background color
    bg_color.g += 0.0001;
  }

  oculus_->updateProjectionMatrices();
  oculus_->update();

  if ( pub_tf_property_->getBool() )
  {
    tf::StampedTransform pose;
    pose.frame_id_ = context_->getFixedFrame().toStdString();
    pose.child_frame_id_ = tf_frame_property_->getStdString();
    pose.stamp_ = ros::Time::now();
    ori = ori * oculus_->getOrientation();
    Ogre::Quaternion r;
    r.FromAngleAxis( Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_Y );
    ori = ori * r;
    r.FromAngleAxis( Ogre::Radian(-M_PI*0.5), Ogre::Vector3::UNIT_X );
    ori = ori * r;
    //Ogre::Matrix3
    pose.setRotation( tf::Quaternion( ori.x, ori.y, ori.z, ori.w ) );
    pose.setOrigin( tf::Vector3( pos.x, pos.y, pos.z ) );
    tf_pub_.sendTransform( pose );
  }
}

void OculusDisplay::reset()
{
  rviz::Display::reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_oculus::OculusDisplay, rviz::Display )
