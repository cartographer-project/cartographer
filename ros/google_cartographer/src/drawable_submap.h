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

#ifndef CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_DRAWABLE_SUBMAP_H_
#define CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_DRAWABLE_SUBMAP_H_

#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTexture.h>
#include <cartographer/common/mutex.h>
#include <google_cartographer_msgs/SubmapEntry.h>
#include <google_cartographer_msgs/SubmapQuery.h>
#include <ros/ros.h>
#include <rviz/display_context.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <future>

namespace cartographer_ros {
namespace rviz {

// Contains all the information needed to render a submap onto the final
// texture representing the whole map.
class DrawableSubmap : public QObject {
  Q_OBJECT

 public:
  // Each submap is identified by a 'trajectory_id' plus a 'submap_id'. The
  // 'frame_manager' is needed to transform the scene node before rendering
  // onto the offscreen texture. 'scene_manager' is the Ogre scene manager which
  // contains all submaps.
  DrawableSubmap(int submap_id, int trajectory_id,
                 ::rviz::FrameManager* frame_manager,
                 Ogre::SceneManager* scene_manager);
  ~DrawableSubmap() override;
  DrawableSubmap(const DrawableSubmap&) = delete;
  DrawableSubmap& operator=(const DrawableSubmap&) = delete;

  // 'submap_entry' contains metadata which is used to find out whether this
  // 'DrawableSubmap' should update itself. If an update is needed, it will send
  // an RPC using 'client' to request the new data for the submap.
  bool Update(const ::google_cartographer_msgs::SubmapEntry& submap_entry,
              ros::ServiceClient* client);

  // Returns whether an RPC is in progress.
  bool QueryInProgress();

  // Transforms the scene node for this submap before being rendered onto a
  // texture.
  void Transform(const ros::Time& ros_time);

  // Sets the alpha of the submap taking into account its slice height and the
  // 'current_tracking_z'.
  void SetAlpha(double current_tracking_z);

 Q_SIGNALS:
  // RPC request succeeded.
  void RequestSucceeded();

 private Q_SLOTS:
  // Callback when an rpc request succeeded.
  void OnRequestSuccess();

 private:
  void QuerySubmap(int submap_id, int trajectory_id,
                   ros::ServiceClient* client);
  void OnRequestFailure();
  void UpdateSceneNode();
  float UpdateAlpha(float target_alpha);

  const int submap_id_;
  const int trajectory_id_;

  ::cartographer::common::Mutex mutex_;
  ::rviz::FrameManager* frame_manager_;
  Ogre::SceneNode* const scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  Eigen::Affine3d submap_pose_ GUARDED_BY(mutex_);
  geometry_msgs::Pose transformed_pose_ GUARDED_BY(mutex_);
  std::chrono::milliseconds last_query_timestamp_ GUARDED_BY(mutex_);
  bool query_in_progress_ GUARDED_BY(mutex_);
  float resolution_ GUARDED_BY(mutex_);
  int width_ GUARDED_BY(mutex_);
  int height_ GUARDED_BY(mutex_);
  int version_ GUARDED_BY(mutex_);
  double slice_height_ GUARDED_BY(mutex_);
  double last_query_slice_height_ GUARDED_BY(mutex_);
  std::future<void> rpc_request_future_;
  std::string cells_ GUARDED_BY(mutex_);
  std::unique_ptr<::google_cartographer_msgs::SubmapQuery::Response> response_
      GUARDED_BY(mutex_);
  int texture_count_;
  float current_alpha_;
};

}  // namespace rviz
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_DRAWABLE_SUBMAP_H_
