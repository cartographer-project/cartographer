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

#ifndef CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_INTERFACE_H_

#include <memory>
#include <vector>

#include "cartographer/common/time.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_3d {

class LocalTrajectoryBuilderInterface {
 public:
  using PoseEstimate = mapping::GlobalTrajectoryBuilderInterface::PoseEstimate;

  struct InsertionResult {
    common::Time time;
    sensor::RangeData range_data_in_tracking;
    transform::Rigid3d pose_observation;
    kalman_filter::PoseCovariance covariance_estimate;
    const Submaps* submaps;
    const Submap* matching_submap;
    std::vector<const Submap*> insertion_submaps;
  };

  virtual ~LocalTrajectoryBuilderInterface() {}

  LocalTrajectoryBuilderInterface(const LocalTrajectoryBuilderInterface&) =
      delete;
  LocalTrajectoryBuilderInterface& operator=(
      const LocalTrajectoryBuilderInterface&) = delete;

  virtual void AddImuData(common::Time time,
                          const Eigen::Vector3d& linear_acceleration,
                          const Eigen::Vector3d& angular_velocity) = 0;
  virtual std::unique_ptr<InsertionResult> AddRangefinderData(
      common::Time time, const Eigen::Vector3f& origin,
      const sensor::PointCloud& ranges, int next_trajectory_node_index) = 0;
  virtual void AddOdometerData(common::Time time,
                               const transform::Rigid3d& pose) = 0;

  virtual const mapping_3d::Submaps* submaps() const = 0;
  virtual const PoseEstimate& pose_estimate() const = 0;

 protected:
  LocalTrajectoryBuilderInterface() {}
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_INTERFACE_H_
