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

#include "cartographer/mapping_2d/global_trajectory_builder.h"

namespace cartographer {
namespace mapping_2d {

GlobalTrajectoryBuilder::GlobalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options,
    SparsePoseGraph* sparse_pose_graph)
    : options_(options),
      sparse_pose_graph_(sparse_pose_graph),
      local_trajectory_builder_(options) {}

GlobalTrajectoryBuilder::~GlobalTrajectoryBuilder() {}

const Submaps* GlobalTrajectoryBuilder::submaps() const {
  return local_trajectory_builder_.submaps();
}

void GlobalTrajectoryBuilder::AddLaserFan(const common::Time time,
                                          const sensor::LaserFan& laser_fan) {
  std::unique_ptr<LocalTrajectoryBuilder::InsertionResult> insertion_result =
      local_trajectory_builder_.AddHorizontalLaserFan(time, laser_fan);
  if (insertion_result != nullptr) {
    sparse_pose_graph_->AddScan(
        insertion_result->time, insertion_result->tracking_to_tracking_2d,
        insertion_result->laser_fan_in_tracking_2d,
        insertion_result->pose_estimate_2d,
        kalman_filter::Project2D(insertion_result->covariance_estimate),
        insertion_result->submaps, insertion_result->matching_submap,
        insertion_result->insertion_submaps);
  }
}

void GlobalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  local_trajectory_builder_.AddImuData(time, linear_acceleration,
                                       angular_velocity);
}

void GlobalTrajectoryBuilder::AddOdometerData(
    const common::Time time, const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance) {
  local_trajectory_builder_.AddOdometerData(time, pose, covariance);
}

const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
GlobalTrajectoryBuilder::pose_estimate() const {
  return local_trajectory_builder_.pose_estimate();
}

}  // namespace mapping_2d
}  // namespace cartographer
