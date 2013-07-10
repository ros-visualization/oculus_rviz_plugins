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

#include <QWidget>
#include <QDesktopWidget>
#include <QApplication>

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>

#include <ros/package.h>

#include <rviz/view_manager.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>

#include "rviz_oculus/oculus_display.h"
#include "rviz_oculus/ogre_oculus.h"

namespace rviz_oculus
{

OculusDisplay::OculusDisplay()
: window_(0)
, scene_node_(0)
{
  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);
}

OculusDisplay::~OculusDisplay()
{
}

void OculusDisplay::onInitialize()
{
  Ogre::Root &root = Ogre::Root::getSingleton();

  rviz::RenderWidget *render_widget = new rviz::RenderWidget( rviz::RenderSystem::get() );
  QRect screen_res = QApplication::desktop()->screenGeometry(1);
  //render_widget->setWindowFlags();
  render_widget->setGeometry( screen_res );
  render_widget->show();
  //render_widget->showFullScreen();

  ROS_INFO_STREAM( root.getDisplayMonitorCount() );

  window_ = render_widget->getRenderWindow();
  window_->setVisible(true);
  window_->setAutoUpdated(true);

  /*
  if ( root.getDisplayMonitorCount() > 1 )
  {
    ROS_INFO_STREAM("Creating Oculus window on display 1.");
    Ogre::NameValuePairList params;
    params["monitorIndex"] = "1";
    window_ = root.createRenderWindow("Oculus", 1280, 800, true, &params);
  }

  window_ = root.createRenderWindow("Oculus", 1280, 800, false);
  */

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  oculus_ = new Oculus();
  oculus_->setupOculus();
  oculus_->setupOgre( scene_manager_, window_, scene_node_ );

  oculus_->getViewport(0)->setBackgroundColour( Ogre::ColourValue(0,0,0,0) );
  oculus_->getViewport(1)->setBackgroundColour( Ogre::ColourValue(0,0,0,0) );
}


void OculusDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
}

void OculusDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
}

void OculusDisplay::onEnable()
{
  window_->setVisible(true);
}

void OculusDisplay::onDisable()
{
  window_->setVisible(false);
}

void OculusDisplay::update( float wall_dt, float ros_dt )
{
  updateCamera();
}

void OculusDisplay::updateCamera()
{
  const Ogre::Camera *cam = context_->getViewManager()->getCurrent()->getCamera();
  scene_node_->setPosition( cam->getDerivedPosition() );
  scene_node_->setOrientation( cam->getDerivedOrientation() );
  oculus_->update();
}

void OculusDisplay::reset()
{
  rviz::Display::reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_oculus::OculusDisplay, rviz::Display )
