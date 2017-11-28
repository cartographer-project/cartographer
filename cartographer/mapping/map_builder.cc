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
#include "cartographer/common/time.h"
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
  *options.mutable_pose_graph_options() = CreatePoseGraphOptions(
      parameter_dictionary->GetDictionary("pose_graph").get());
  CHECK_NE(options.use_trajectory_builder_2d(),
           options.use_trajectory_builder_3d());
  return options;
}

MapBuilder::MapBuilder(
    const proto::MapBuilderOptions& options,
    const LocalSlamResultCallback& local_slam_result_callback)
    : options_(options),
      thread_pool_(options.num_background_threads()),
      local_slam_result_callback_(local_slam_result_callback) {
  if (options.use_trajectory_builder_2d()) {
    pose_graph_2d_ = common::make_unique<mapping_2d::PoseGraph>(
        options_.pose_graph_options(), &thread_pool_);
    pose_graph_ = pose_graph_2d_.get();
  }
  if (options.use_trajectory_builder_3d()) {
    pose_graph_3d_ = common::make_unique<mapping_3d::PoseGraph>(
        options_.pose_graph_options(), &thread_pool_);
    pose_graph_ = pose_graph_3d_.get();
  }
}

MapBuilder::~MapBuilder() {}

int MapBuilder::AddTrajectoryBuilder(
    const std::unordered_set<std::string>& expected_sensor_ids,
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
                mapping_3d::PoseGraph>>(
                trajectory_options.trajectory_builder_3d_options(),
                trajectory_id, pose_graph_3d_.get(),
                local_slam_result_callback_)));
  } else {
    CHECK(trajectory_options.has_trajectory_builder_2d_options());
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping::GlobalTrajectoryBuilder<
                mapping_2d::LocalTrajectoryBuilder,
                mapping_2d::proto::LocalTrajectoryBuilderOptions,
                mapping_2d::PoseGraph>>(
                trajectory_options.trajectory_builder_2d_options(),
                trajectory_id, pose_graph_2d_.get(),
                local_slam_result_callback_)));
  }
  if (trajectory_options.pure_localization()) {
    constexpr int kSubmapsToKeep = 3;
    pose_graph_->AddTrimmer(common::make_unique<PureLocalizationTrimmer>(
        trajectory_id, kSubmapsToKeep));
  }
  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    pose_graph_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }
  return trajectory_id;
}

int MapBuilder::AddTrajectoryForDeserialization() {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  return trajectory_id;
}

TrajectoryBuilder* MapBuilder::GetTrajectoryBuilder(
    const int trajectory_id) const {
  return trajectory_builders_.at(trajectory_id).get();
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_.FinishTrajectory(trajectory_id);
  pose_graph_->FinishTrajectory(trajectory_id);
}

int MapBuilder::GetBlockingTrajectoryId() const {
  return sensor_collator_.GetBlockingTrajectoryId();
}

std::string MapBuilder::SubmapToProto(
    const mapping::SubmapId& submap_id,
    proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

void MapBuilder::SerializeState(io::ProtoStreamWriter* const writer) {
  // We serialize the pose graph followed by all the data referenced in it.
  writer->WriteProto(pose_graph_->ToProto());
  // Next we serialize all submap data.
  {
    for (const auto& submap_id_data : pose_graph_->GetAllSubmapData()) {
      proto::SerializedData proto;
      auto* const submap_proto = proto.mutable_submap();
      submap_proto->mutable_submap_id()->set_trajectory_id(
          submap_id_data.id.trajectory_id);
      submap_proto->mutable_submap_id()->set_submap_index(
          submap_id_data.id.submap_index);
      submap_id_data.data.submap->ToProto(submap_proto);
      writer->WriteProto(proto);
    }
  }
  // Next we serialize all node data.
  {
    for (const auto& node_id_data : pose_graph_->GetTrajectoryNodes()) {
      proto::SerializedData proto;
      auto* const node_proto = proto.mutable_node();
      node_proto->mutable_node_id()->set_trajectory_id(
          node_id_data.id.trajectory_id);
      node_proto->mutable_node_id()->set_node_index(node_id_data.id.node_index);
      *node_proto->mutable_node_data() =
          ToProto(*node_id_data.data.constant_data);
      writer->WriteProto(proto);
    }
  }
  // Next we serialize IMU data from the pose graph.
  {
    const auto all_imu_data = pose_graph_->GetImuData();
    for (const int trajectory_id : all_imu_data.trajectory_ids()) {
      for (const auto& imu_data : all_imu_data.trajectory(trajectory_id)) {
        proto::SerializedData proto;
        auto* const imu_data_proto = proto.mutable_imu_data();
        imu_data_proto->set_trajectory_id(trajectory_id);
        *imu_data_proto->mutable_imu_data() = sensor::ToProto(imu_data);
        writer->WriteProto(proto);
      }
    }
  }
  // Next we serialize odometry data from the pose graph.
  {
    const auto all_odometry_data = pose_graph_->GetOdometryData();
    for (const int trajectory_id : all_odometry_data.trajectory_ids()) {
      for (const auto& odometry_data :
           all_odometry_data.trajectory(trajectory_id)) {
        proto::SerializedData proto;
        auto* const odometry_data_proto = proto.mutable_odometry_data();
        odometry_data_proto->set_trajectory_id(trajectory_id);
        *odometry_data_proto->mutable_odometry_data() =
            sensor::ToProto(odometry_data);
        writer->WriteProto(proto);
      }
    }
  }
  // TODO(whess): Serialize additional sensor data: fixed frame pose data.
}

void MapBuilder::LoadMap(io::ProtoStreamReader* const reader) {
  proto::PoseGraph pose_graph;
  CHECK(reader->ReadProto(&pose_graph));

  std::map<int, int> trajectory_remapping;
  for (auto& trajectory_proto : *pose_graph.mutable_trajectory()) {
    const int new_trajectory_id = AddTrajectoryForDeserialization();
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    pose_graph_->FreezeTrajectory(new_trajectory_id);
  }

  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto : pose_graph.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto : pose_graph.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  for (;;) {
    proto::SerializedData proto;
    if (!reader->ReadProto(&proto)) {
      break;
    }
    if (proto.has_node()) {
      proto.mutable_node()->mutable_node_id()->set_trajectory_id(
          trajectory_remapping.at(proto.node().node_id().trajectory_id()));
      const transform::Rigid3d node_pose =
          node_poses.at(NodeId{proto.node().node_id().trajectory_id(),
                               proto.node().node_id().node_index()});
      pose_graph_->AddNodeFromProto(node_pose, proto.node());
    }
    if (proto.has_submap()) {
      proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
          trajectory_remapping.at(proto.submap().submap_id().trajectory_id()));
      const transform::Rigid3d submap_pose =
          submap_poses.at(SubmapId{proto.submap().submap_id().trajectory_id(),
                                   proto.submap().submap_id().submap_index()});
      pose_graph_->AddSubmapFromProto(submap_pose, proto.submap());
    }
    // TODO(ojura): Deserialize IMU and odometry data when loading unfrozen
    // trajectories.
  }

  // Add information about which nodes belong to which submap.
  for (const proto::PoseGraph::Constraint& constraint_proto :
       pose_graph.constraint()) {
    if (constraint_proto.tag() !=
        mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP) {
      continue;
    }
    const NodeId node_id{
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()),
        constraint_proto.node_id().node_index()};
    const SubmapId submap_id{
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()),
        constraint_proto.submap_id().submap_index()};
    pose_graph_->AddNodeToSubmap(node_id, submap_id);
  }
  CHECK(reader->eof());
}

int MapBuilder::num_trajectory_builders() const {
  return trajectory_builders_.size();
}

PoseGraph* MapBuilder::pose_graph() { return pose_graph_; }

}  // namespace mapping
}  // namespace cartographer
