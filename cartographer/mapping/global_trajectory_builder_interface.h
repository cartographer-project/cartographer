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

#ifndef CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_estimate.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
class GlobalTrajectoryBuilderInterface {
 public:
  GlobalTrajectoryBuilderInterface() {}
  virtual ~GlobalTrajectoryBuilderInterface() {}

  GlobalTrajectoryBuilderInterface(const GlobalTrajectoryBuilderInterface&) =
      delete;
  GlobalTrajectoryBuilderInterface& operator=(
      const GlobalTrajectoryBuilderInterface&) = delete;

  virtual const PoseEstimate& pose_estimate() const = 0;

  virtual void AddRangefinderData(common::Time time,
                                  const Eigen::Vector3f& origin,
                                  const sensor::PointCloud& ranges) = 0;
  virtual void AddSensorData(const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const sensor::OdometryData& odometry_data) = 0;
  virtual void AddSensorData(
      const sensor::FixedFramePoseData& fixed_frame_pose) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_
