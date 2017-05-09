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

#include "cartographer/mapping/map_builder.h"

#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/collated_trajectory_builder.h"
#include "cartographer/mapping_2d/global_trajectory_builder.h"
#include "cartographer/mapping_3d/global_trajectory_builder.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MapBuilderOptions options;
  options.set_use_trajectory_builder_2d(
      parameter_dictionary->GetBool("use_trajectory_builder_2d"));
  options.set_use_trajectory_builder_3d(
      parameter_dictionary->GetBool("use_trajectory_builder_3d"));
  options.set_num_background_threads(
      parameter_dictionary->GetNonNegativeInt("num_background_threads"));
  *options.mutable_sparse_pose_graph_options() = CreateSparsePoseGraphOptions(
      parameter_dictionary->GetDictionary("sparse_pose_graph").get());
  CHECK_NE(options.use_trajectory_builder_2d(),
           options.use_trajectory_builder_3d());
  return options;
}

MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  if (options.use_trajectory_builder_2d()) {
    sparse_pose_graph_2d_ = common::make_unique<mapping_2d::SparsePoseGraph>(
        options_.sparse_pose_graph_options(), &thread_pool_);
    sparse_pose_graph_ = sparse_pose_graph_2d_.get();
  }
  if (options.use_trajectory_builder_3d()) {
    sparse_pose_graph_3d_ = common::make_unique<mapping_3d::SparsePoseGraph>(
        options_.sparse_pose_graph_options(), &thread_pool_);
    sparse_pose_graph_ = sparse_pose_graph_3d_.get();
  }
}

MapBuilder::~MapBuilder() {}

int MapBuilder::AddTrajectoryBuilder(
    const std::unordered_set<string>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options) {
  const int trajectory_id = trajectory_builders_.size();
  if (options_.use_trajectory_builder_3d()) {
    CHECK(trajectory_options.has_trajectory_builder_3d_options());
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping_3d::GlobalTrajectoryBuilder>(
                trajectory_options.trajectory_builder_3d_options(),
                sparse_pose_graph_3d_.get())));
  } else {
    CHECK(trajectory_options.has_trajectory_builder_2d_options());
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping_2d::GlobalTrajectoryBuilder>(
                trajectory_options.trajectory_builder_2d_options(),
                sparse_pose_graph_2d_.get())));
  }
  trajectory_ids_.emplace(trajectory_builders_.back()->submaps(),
                          trajectory_id);
  return trajectory_id;
}

TrajectoryBuilder* MapBuilder::GetTrajectoryBuilder(
    const int trajectory_id) const {
  return trajectory_builders_.at(trajectory_id).get();
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_.FinishTrajectory(trajectory_id);
}

int MapBuilder::GetBlockingTrajectoryId() const {
  return sensor_collator_.GetBlockingTrajectoryId();
}

int MapBuilder::GetTrajectoryId(const Submaps* const trajectory) const {
  const auto trajectory_id = trajectory_ids_.find(trajectory);
  CHECK(trajectory_id != trajectory_ids_.end());
  return trajectory_id->second;
}

proto::TrajectoryConnectivity MapBuilder::GetTrajectoryConnectivity() {
  return ToProto(sparse_pose_graph_->GetConnectedTrajectories(),
                 trajectory_ids_);
}

string MapBuilder::SubmapToProto(const int trajectory_id,
                                 const int submap_index,
                                 proto::SubmapQuery::Response* const response) {
  if (trajectory_id < 0 || trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " + std::to_string(trajectory_id) +
           " but there are only " + std::to_string(num_trajectory_builders()) +
           " trajectories.";
  }

  const Submaps* const submaps =
      trajectory_builders_.at(trajectory_id)->submaps();
  const std::vector<transform::Rigid3d> submap_transforms =
      sparse_pose_graph_->GetSubmapTransforms(submaps);
  if (submap_index < 0 ||
      static_cast<size_t>(submap_index) >= submap_transforms.size()) {
    return "Requested submap " + std::to_string(submap_index) +
           " from trajectory " + std::to_string(trajectory_id) +
           " but there are only " + std::to_string(submap_transforms.size()) +
           " submaps in this trajectory.";
  }

  response->set_submap_version(submaps->Get(submap_index)->num_range_data);
  submaps->SubmapToProto(submap_index, sparse_pose_graph_->GetTrajectoryNodes(),
                         submap_transforms[submap_index], response);
  return "";
}

int MapBuilder::num_trajectory_builders() const {
  return trajectory_builders_.size();
}

SparsePoseGraph* MapBuilder::sparse_pose_graph() { return sparse_pose_graph_; }

}  // namespace mapping
}  // namespace cartographer
