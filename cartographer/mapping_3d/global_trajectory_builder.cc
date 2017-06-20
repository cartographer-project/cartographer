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
    const int trajectory_id, SparsePoseGraph* sparse_pose_graph)
    : trajectory_id_(trajectory_id),
      sparse_pose_graph_(sparse_pose_graph),
      local_trajectory_builder_(CreateLocalTrajectoryBuilder(options)) {}

GlobalTrajectoryBuilder::~GlobalTrajectoryBuilder() {}

int GlobalTrajectoryBuilder::num_submaps() {
  return sparse_pose_graph_->num_submaps(trajectory_id_);
}

GlobalTrajectoryBuilder::SubmapData GlobalTrajectoryBuilder::GetSubmapData(
    const int submap_index) {
  auto* active_submaps = local_trajectory_builder_->active_submaps();
  std::shared_ptr<const mapping::Submap> submap;
  if (submap_index < active_submaps->matching_index()) {
    // The submap is no longer known to the local trajectory builder, let's ask
    // the sparse pose graph.
    submap = sparse_pose_graph_->GetSubmap(
        mapping::SubmapId{trajectory_id_, submap_index});
  } else {
    const int active_submap_index =
        submap_index - active_submaps->matching_index();
    CHECK_LE(0, active_submap_index);
    CHECK_LT(active_submap_index, 2);
    submap = active_submaps->submaps().at(active_submap_index);
  }
  CHECK(submap != nullptr);
  return {submap, sparse_pose_graph_->GetSubmapTransform(
                      mapping::SubmapId{trajectory_id_, submap_index})};
}

void GlobalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  local_trajectory_builder_->AddImuData(time, linear_acceleration,
                                        angular_velocity);
  sparse_pose_graph_->AddImuData(trajectory_id_, time, linear_acceleration,
                                 angular_velocity);
}

void GlobalTrajectoryBuilder::AddRangefinderData(
    const common::Time time, const Eigen::Vector3f& origin,
    const sensor::PointCloud& ranges) {
  auto insertion_result =
      local_trajectory_builder_->AddRangefinderData(time, origin, ranges);
  if (insertion_result == nullptr) {
    return;
  }
  sparse_pose_graph_->AddScan(
      insertion_result->time, insertion_result->range_data_in_tracking,
      insertion_result->pose_observation, trajectory_id_,
      std::move(insertion_result->insertion_submaps));
}

void GlobalTrajectoryBuilder::AddOdometerData(const common::Time time,
                                              const transform::Rigid3d& pose) {
  local_trajectory_builder_->AddOdometerData(time, pose);
}

const GlobalTrajectoryBuilder::PoseEstimate&
GlobalTrajectoryBuilder::pose_estimate() const {
  return local_trajectory_builder_->pose_estimate();
}

}  // namespace mapping_3d
}  // namespace cartographer
