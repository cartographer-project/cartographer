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

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

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

std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = common::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        common::make_unique<pose_graph::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.use_trajectory_builder_3d()) {
    pose_graph_ = common::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        common::make_unique<pose_graph::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.collate_by_trajectory()) {
    sensor_collator_ = common::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = common::make_unique<sensor::Collator>();
  }
}

int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  const int trajectory_id = trajectory_builders_.size();
  if (options_.use_trajectory_builder_3d()) {
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = common::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            sensor_collator_.get(), trajectory_id, expected_sensor_ids,
            CreateGlobalTrajectoryBuilder3D(
                std::move(local_trajectory_builder), trajectory_id,
                static_cast<PoseGraph3D*>(pose_graph_.get()),
                local_slam_result_callback)));
  } else {
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_2d_options()) {
      local_trajectory_builder = common::make_unique<LocalTrajectoryBuilder2D>(
          trajectory_options.trajectory_builder_2d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            sensor_collator_.get(), trajectory_id, expected_sensor_ids,
            CreateGlobalTrajectoryBuilder2D(
                std::move(local_trajectory_builder), trajectory_id,
                static_cast<PoseGraph2D*>(pose_graph_.get()),
                local_slam_result_callback)));
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
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

int MapBuilder::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_->FinishTrajectory(trajectory_id);
  pose_graph_->FinishTrajectory(trajectory_id);
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

void MapBuilder::SerializeState(io::ProtoStreamWriterInterface* const writer) {
  // We serialize the pose graph followed by all the data referenced in it.
  writer->WriteProto(pose_graph_->ToProto());
  // Serialize trajectory builder options.
  {
    proto::AllTrajectoryBuilderOptions all_builder_options_proto;
    for (const auto& options_with_sensor_ids :
         all_trajectory_builder_options_) {
      *all_builder_options_proto.add_options_with_sensor_ids() =
          options_with_sensor_ids;
    }
    CHECK_EQ(all_trajectory_builder_options_.size(),
             all_builder_options_proto.options_with_sensor_ids_size());
    writer->WriteProto(all_builder_options_proto);
  }
  // Next we serialize all submap data.
  {
    for (const auto& submap_id_data : pose_graph_->GetAllSubmapData()) {
      proto::SerializedData proto;
      auto* const submap_proto = proto.mutable_submap();
      submap_proto->mutable_submap_id()->set_trajectory_id(
          submap_id_data.id.trajectory_id);
      submap_proto->mutable_submap_id()->set_submap_index(
          submap_id_data.id.submap_index);
      submap_id_data.data.submap->ToProto(
          submap_proto, true /* include_probability_grid_data */);
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
  // Next we serialize all fixed frame pose data from the pose graph.
  {
    const auto all_fixed_frame_pose_data = pose_graph_->GetFixedFramePoseData();
    for (const int trajectory_id : all_fixed_frame_pose_data.trajectory_ids()) {
      for (const auto& fixed_frame_pose_data :
           all_fixed_frame_pose_data.trajectory(trajectory_id)) {
        proto::SerializedData proto;
        auto* const fixed_frame_pose_data_proto =
            proto.mutable_fixed_frame_pose_data();
        fixed_frame_pose_data_proto->set_trajectory_id(trajectory_id);
        *fixed_frame_pose_data_proto->mutable_fixed_frame_pose_data() =
            sensor::ToProto(fixed_frame_pose_data);
        writer->WriteProto(proto);
      }
    }
  }
  // Next we serialize all trajectory data.
  {
    const auto all_trajectory_data = pose_graph_->GetTrajectoryData();
    for (const auto& trajectory_data : all_trajectory_data) {
      proto::SerializedData proto;
      auto* const trajectory_data_proto = proto.mutable_trajectory_data();
      trajectory_data_proto->set_trajectory_id(trajectory_data.first);
      trajectory_data_proto->set_gravity_constant(
          trajectory_data.second.gravity_constant);
      *trajectory_data_proto->mutable_imu_calibration() = transform::ToProto(
          Eigen::Quaterniond(trajectory_data.second.imu_calibration[0],
                             trajectory_data.second.imu_calibration[1],
                             trajectory_data.second.imu_calibration[2],
                             trajectory_data.second.imu_calibration[3]));
      if (trajectory_data.second.fixed_frame_origin_in_map.has_value()) {
        *trajectory_data_proto->mutable_fixed_frame_origin_in_map() =
            transform::ToProto(
                trajectory_data.second.fixed_frame_origin_in_map.value());
      }
      writer->WriteProto(proto);
    }
  }
  // Next we serialize all landmark data.
  {
    const std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
        all_landmark_nodes = pose_graph_->GetLandmarkNodes();
    for (const auto& node : all_landmark_nodes) {
      for (const auto& observation : node.second.landmark_observations) {
        proto::SerializedData proto;
        auto* landmark_data_proto = proto.mutable_landmark_data();
        landmark_data_proto->set_trajectory_id(observation.trajectory_id);
        landmark_data_proto->mutable_landmark_data()->set_timestamp(
            common::ToUniversal(observation.time));
        auto* observation_proto = landmark_data_proto->mutable_landmark_data()
                                      ->add_landmark_observations();
        observation_proto->set_id(node.first);
        *observation_proto->mutable_landmark_to_tracking_transform() =
            transform::ToProto(observation.landmark_to_tracking_transform);
        observation_proto->set_translation_weight(
            observation.translation_weight);
        observation_proto->set_rotation_weight(observation.rotation_weight);
        writer->WriteProto(proto);
      }
    }
  }
}

void MapBuilder::LoadState(io::ProtoStreamReaderInterface* const reader,
                           bool load_frozen_state) {
  proto::PoseGraph pose_graph_proto;
  CHECK(reader->ReadProto(&pose_graph_proto));
  proto::AllTrajectoryBuilderOptions all_builder_options_proto;
  CHECK(reader->ReadProto(&all_builder_options_proto));
  CHECK_EQ(pose_graph_proto.trajectory_size(),
           all_builder_options_proto.options_with_sensor_ids_size());

  std::map<int, int> trajectory_remapping;
  for (auto& trajectory_proto : *pose_graph_proto.mutable_trajectory()) {
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(
            trajectory_proto.trajectory_id());
    const int new_trajectory_id =
        AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      pose_graph_->FreezeTrajectory(new_trajectory_id);
    }
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmarks()) {
    pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()));
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
    if (proto.has_trajectory_data()) {
      proto.mutable_trajectory_data()->set_trajectory_id(
          trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
      pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
    }
    if (!load_frozen_state) {
      if (proto.has_imu_data()) {
        pose_graph_->AddImuData(
            trajectory_remapping.at(proto.imu_data().trajectory_id()),
            sensor::FromProto(proto.imu_data().imu_data()));
      }
      if (proto.has_odometry_data()) {
        pose_graph_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
      }
      if (proto.has_fixed_frame_pose_data()) {
        pose_graph_->AddFixedFramePoseData(
            trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
      }
      if (proto.has_landmark_data()) {
        pose_graph_->AddLandmarkData(
            trajectory_remapping.at(proto.landmark_data().trajectory_id()),
            sensor::FromProto(proto.landmark_data().landmark_data()));
      }
    }
  }

  if (load_frozen_state) {
    // Add information about which nodes belong to which submap.
    // Required for 3D pure localization.
    for (const proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      if (constraint_proto.tag() !=
          mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      pose_graph_->AddNodeToSubmap(
          NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          SubmapId{constraint_proto.submap_id().trajectory_id(),
                   constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(
        FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
}

}  // namespace mapping
}  // namespace cartographer
