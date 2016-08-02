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

#include "trajectory.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <cartographer/common/make_unique.h>
#include <rviz/frame_manager.h>

#include <memory>
#include <mutex>
#include <vector>

#include "drawable_submap.h"

namespace cartographer_ros {
namespace rviz {

void Trajectory::Add(const int submap_id, const int trajectory_id,
                     ::rviz::FrameManager* const frame_manager,
                     Ogre::SceneManager* const scene_manager) {
  std::lock_guard<std::mutex> guard(mutex_);
  drawable_submaps_.push_back(
      ::cartographer::common::make_unique<DrawableSubmap>(
          submap_id, trajectory_id, frame_manager, scene_manager));
}

DrawableSubmap& Trajectory::Get(const int index) {
  std::lock_guard<std::mutex> guard(mutex_);
  return *drawable_submaps_[index];
}

int Trajectory::Size() { return drawable_submaps_.size(); }

}  // namespace rviz
}  // namespace cartographer_ros
