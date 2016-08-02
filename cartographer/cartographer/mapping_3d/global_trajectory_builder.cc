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

#include "cartographer/mapping_3d/global_trajectory_builder.h"

#include "cartographer/mapping_3d/local_trajectory_builder.h"

namespace cartographer {
namespace mapping_3d {

GlobalTrajectoryBuilder::GlobalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options,
    SparsePoseGraph* sparse_pose_graph)
    : sparse_pose_graph_(sparse_pose_graph),
      local_trajectory_builder_(CreateLocalTrajectoryBuilder(options)) {}

GlobalTrajectoryBuilder::~GlobalTrajectoryBuilder() {}

const mapping_3d::Submaps* GlobalTrajectoryBuilder::submaps() const {
  return local_trajectory_builder_->submaps();
}

mapping_3d::Submaps* GlobalTrajectoryBuilder::submaps() {
  return local_trajectory_builder_->submaps();
}

kalman_filter::PoseTracker* GlobalTrajectoryBuilder::pose_tracker() const {
  return local_trajectory_builder_->pose_tracker();
}

void GlobalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  local_trajectory_builder_->AddImuData(time, linear_acceleration,
                                        angular_velocity);
  sparse_pose_graph_->AddImuData(time, linear_acceleration, angular_velocity);
}

void GlobalTrajectoryBuilder::AddLaserFan3D(
    const common::Time time, const sensor::LaserFan3D& laser_fan) {
  auto insertion_result =
      local_trajectory_builder_->AddLaserFan3D(time, laser_fan);

  if (insertion_result == nullptr) {
    return;
  }

  const int trajectory_node_index = sparse_pose_graph_->AddScan(
      insertion_result->time, insertion_result->laser_fan_in_tracking,
      insertion_result->pose_observation, insertion_result->covariance_estimate,
      insertion_result->submaps, insertion_result->matching_submap,
      insertion_result->insertion_submaps);
  local_trajectory_builder_->AddTrajectoryNodeIndex(trajectory_node_index);
}

void GlobalTrajectoryBuilder::AddOdometerPose(
    const common::Time time, const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance) {
  local_trajectory_builder_->AddOdometerPose(time, pose, covariance);
}

const GlobalTrajectoryBuilder::PoseEstimate&
GlobalTrajectoryBuilder::pose_estimate() const {
  return local_trajectory_builder_->pose_estimate();
}

}  // namespace mapping_3d
}  // namespace cartographer
