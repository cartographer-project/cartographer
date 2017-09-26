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
#include <unordered_set>
#include <utility>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/collated_trajectory_builder.h"
#include "cartographer/mapping/global_trajectory_builder.h"
#include "cartographer/mapping_2d/local_trajectory_builder.h"
#include "cartographer/mapping_3d/local_trajectory_builder.h"
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
            common::make_unique<mapping::GlobalTrajectoryBuilder<
                mapping_3d::LocalTrajectoryBuilder,
                mapping_3d::proto::LocalTrajectoryBuilderOptions,
                mapping_3d::SparsePoseGraph>>(
                trajectory_options.trajectory_builder_3d_options(),
                trajectory_id, sparse_pose_graph_3d_.get())));
  } else {
    CHECK(trajectory_options.has_trajectory_builder_2d_options());
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping::GlobalTrajectoryBuilder<
                mapping_2d::LocalTrajectoryBuilder,
                mapping_2d::proto::LocalTrajectoryBuilderOptions,
                mapping_2d::SparsePoseGraph>>(
                trajectory_options.trajectory_builder_2d_options(),
                trajectory_id, sparse_pose_graph_2d_.get())));
  }
  if (trajectory_options.pure_localization()) {
    constexpr int kSubmapsToKeep = 3;
    sparse_pose_graph_->AddTrimmer(common::make_unique<PureLocalizationTrimmer>(
        trajectory_id, kSubmapsToKeep));
  }
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

string MapBuilder::SubmapToProto(const mapping::SubmapId& submap_id,
                                 proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const int num_submaps =
      sparse_pose_graph_->num_submaps(submap_id.trajectory_id);
  if (submap_id.submap_index < 0 || submap_id.submap_index >= num_submaps) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but there are only " + std::to_string(num_submaps) +
           " submaps in this trajectory.";
  }

  const auto submap_data = sparse_pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

void MapBuilder::SerializeState(io::ProtoStreamWriter* const writer) {
  // We serialize the pose graph followed by all the data referenced in it.
  writer->WriteProto(sparse_pose_graph_->ToProto());
  // Next we serialize all submap data.
  {
    const auto submap_data = sparse_pose_graph_->GetAllSubmapData();
    for (int trajectory_id = 0;
         trajectory_id != static_cast<int>(submap_data.size());
         ++trajectory_id) {
      for (int submap_index = 0;
           submap_index != static_cast<int>(submap_data[trajectory_id].size());
           ++submap_index) {
        proto::SerializedData proto;
        auto* const submap_proto = proto.mutable_submap();
        // TODO(whess): Handle trimmed data.
        submap_proto->mutable_submap_id()->set_trajectory_id(trajectory_id);
        submap_proto->mutable_submap_id()->set_submap_index(submap_index);
        submap_data[trajectory_id][submap_index].submap->ToProto(submap_proto);
        // TODO(whess): Only enable optionally? Resulting pbstream files will be
        // a lot larger now.
        writer->WriteProto(proto);
      }
    }
  }
  // Next we serialize all node data.
  {
    const auto node_data = sparse_pose_graph_->GetTrajectoryNodes();
    for (int trajectory_id = 0;
         trajectory_id != static_cast<int>(node_data.size()); ++trajectory_id) {
      for (int node_index = 0;
           node_index != static_cast<int>(node_data[trajectory_id].size());
           ++node_index) {
        proto::SerializedData proto;
        auto* const node_data_proto = proto.mutable_node_data();
        // TODO(whess): Handle trimmed data.
        node_data_proto->mutable_node_id()->set_trajectory_id(trajectory_id);
        node_data_proto->mutable_node_id()->set_node_index(node_index);
        *node_data_proto->mutable_trajectory_node() =
            ToProto(*node_data[trajectory_id][node_index].constant_data);
        // TODO(whess): Only enable optionally? Resulting pbstream files will be
        // a lot larger now.
        writer->WriteProto(proto);
      }
    }
    // TODO(whess): Serialize additional sensor data: IMU, odometry.
  }
}

void MapBuilder::LoadMap(io::ProtoStreamReader* const reader) {
  proto::SparsePoseGraph pose_graph;
  CHECK(reader->ReadProto(&pose_graph));

  // TODO(whess): Not all trajectories should be builders, i.e. support should
  // be added for trajectories without latest pose, options, etc. Appease the
  // trajectory builder for now.
  proto::TrajectoryBuilderOptions unused_options;
  unused_options.mutable_trajectory_builder_2d_options()
      ->mutable_submaps_options()
      ->set_resolution(0.05);
  unused_options.mutable_trajectory_builder_3d_options();

  const std::unordered_set<string> unused_sensor_ids;
  const int map_trajectory_id =
      AddTrajectoryBuilder(unused_sensor_ids, unused_options);
  FinishTrajectory(map_trajectory_id);
  sparse_pose_graph_->FreezeTrajectory(map_trajectory_id);

  for (;;) {
    proto::SerializedData proto;
    if (!reader->ReadProto(&proto)) {
      break;
    }
    if (proto.has_submap()) {
      const transform::Rigid3d submap_pose = transform::ToRigid3(
          pose_graph.trajectory(proto.submap().submap_id().trajectory_id())
              .submap(proto.submap().submap_id().submap_index())
              .pose());
      sparse_pose_graph_->AddSubmapFromProto(map_trajectory_id, submap_pose,
                                             proto.submap());
    }
  }
  CHECK(reader->eof());
}

int MapBuilder::num_trajectory_builders() const {
  return trajectory_builders_.size();
}

SparsePoseGraph* MapBuilder::sparse_pose_graph() { return sparse_pose_graph_; }

}  // namespace mapping
}  // namespace cartographer
