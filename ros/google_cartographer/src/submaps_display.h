/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_SUBMAPS_DISPLAY_H_
#define CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_SUBMAPS_DISPLAY_H_

#include <OgreMaterial.h>
#include <OgreOverlay.h>
#include <OgreOverlayContainer.h>
#include <OgreSceneManager.h>
#include <OgreSharedPtr.h>
#include <OgreTexture.h>
#include <OgreVector3.h>
#include <cartographer/common/mutex.h>
#include <cartographer/common/port.h>
#include <google_cartographer_msgs/SubmapList.h>
#include <nav_msgs/MapMetaData.h>
#include <ros/time.h>
#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>
#include <tf/tfMessage.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

#include "drawable_submap.h"
#include "trajectory.h"

namespace cartographer_ros {
namespace rviz {

// RViz plugin used for displaying maps which are represented by a collection of
// submaps.
//
// We keep a separate Ogre scene to the one provided by rviz and in it we place
// every submap as a SceneNode. We show an X-ray view of the map which is
// achieved by shipping textures for every submap containing pre-multiplied
// alpha and grayscale values, these are then alpha blended together onto an
// offscreen texture. This offscreen texture is then screen blit onto the screen
// as a grayscale image.
class SubmapsDisplay : public ::rviz::Display {
  Q_OBJECT

 public:
  SubmapsDisplay();
  ~SubmapsDisplay() override;

  SubmapsDisplay(const SubmapsDisplay&) = delete;
  SubmapsDisplay& operator=(const SubmapsDisplay&) = delete;

  // Called by RViz on initialization of the plugin.
  void onInitialize() override;
  // Called to tell the display to clear its state.
  void reset() override;

 Q_SIGNALS:
  void SubmapListUpdated();

 private Q_SLOTS:
  void RequestNewSubmaps();
  void UpdateTopic();
  void UpdateSubmapQueryServiceName();

 private:
  class SceneManagerListener : public Ogre::SceneManager::Listener {
   public:
    SceneManagerListener(std::function<void()> callback)
        : callback_(callback) {}
    void preUpdateSceneGraph(Ogre::SceneManager* source, Ogre::Camera* camera) {
      callback_();
    }

   private:
    std::function<void()> callback_;
  };

  void onEnable() override;
  void onDisable() override;
  void Subscribe();
  void Unsubscribe();
  void UpdateMapTexture();
  void IncomingSubmapList(
      const ::google_cartographer_msgs::SubmapList::ConstPtr& msg);
  // Clears the current map.
  void Clear();
  void UpdateCurrentTrackingZ(const tf::tfMessage::ConstPtr& msg);

  int rtt_count_;
  SceneManagerListener scene_manager_listener_;
  ::google_cartographer_msgs::SubmapList submap_list_;
  ros::Subscriber submap_list_subscriber_;
  ::tf2_ros::Buffer tf_buffer_;
  ::tf2_ros::TransformListener tf_listener_;
  ros::ServiceClient client_;
  ::rviz::RosTopicProperty* topic_property_;
  ::rviz::StringProperty* submap_query_service_property_;
  ::rviz::StringProperty* map_frame_property_;
  ::rviz::StringProperty* tracking_frame_property_;
  std::vector<std::unique_ptr<Trajectory>> trajectories_ GUARDED_BY(mutex_);
  Ogre::SceneManager* submaps_scene_manager_;
  Ogre::Camera* submaps_scene_camera_;
  Ogre::MaterialPtr submap_scene_material_;
  Ogre::MaterialPtr screen_blit_material_;
  Ogre::Overlay* overlay_;
  Ogre::OverlayContainer* panel_;
  Ogre::TexturePtr rttTexture_;
  ::cartographer::common::Mutex mutex_;
};

}  // namespace rviz
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_SUBMAPS_DISPLAY_H_
