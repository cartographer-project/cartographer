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

#ifndef CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_TRAJECTORY_H_
#define CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_TRAJECTORY_H_

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/frame_manager.h>

#include <memory>
#include <mutex>
#include <vector>

#include "drawable_submap.h"

namespace cartographer_ros {
namespace rviz {

// Contains a list of drawable submaps.
class Trajectory {
 public:
  Trajectory() = default;

  Trajectory(const Trajectory&) = delete;
  Trajectory& operator=(const Trajectory&) = delete;

  // Creates a new DrawableSubmap and stores it as part of this trajectory.
  void Add(int submap_id, int trajectory_id,
           ::rviz::FrameManager* frame_manager,
           Ogre::SceneManager* scene_manager);
  // Gets the submap at 'index' and returns a non-const reference.
  DrawableSubmap& Get(int index);
  // Returns the number of DrawableSubmaps this trajectory contains.
  int Size();

 private:
  std::mutex mutex_;
  std::vector<std::unique_ptr<DrawableSubmap>> drawable_submaps_;
};

}  // namespace rviz
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_TRAJECTORY_H_
