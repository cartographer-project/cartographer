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

#ifndef CARTOGRAPHER_MAPPING_POSE_ESTIMATE_H_
#define CARTOGRAPHER_MAPPING_POSE_ESTIMATE_H_

#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Represents a newly computed pose. 'pose' is the end-user visualization of
// orientation and 'point_cloud' is the point cloud, in the local map frame.
struct PoseEstimate {
  PoseEstimate() = default;
  PoseEstimate(common::Time time, const transform::Rigid3d& pose,
               const sensor::PointCloud& point_cloud)
      : time(time), pose(pose), point_cloud(point_cloud) {}

  common::Time time = common::Time::min();
  transform::Rigid3d pose = transform::Rigid3d::Identity();
  sensor::PointCloud point_cloud;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_ESTIMATE_H_
