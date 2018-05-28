/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace io {

namespace {

mapping::proto::AllTrajectoryBuilderOptions
CreateAllTrajectoryBuilderOptionsProto(
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        all_options_with_sensor_ids) {
  mapping::proto::AllTrajectoryBuilderOptions all_options_proto;
  for (const auto& options_with_sensor_ids : all_options_with_sensor_ids) {
    *all_options_proto.add_options_with_sensor_ids() = options_with_sensor_ids;
  }
  CHECK_EQ(all_options_with_sensor_ids.size(),
           all_options_proto.options_with_sensor_ids_size());
  return all_options_proto;
}

mapping::proto::SerializationHeader CreateHeader() {
  mapping::proto::SerializationHeader header;
  header.set_format_version(kMappingStateSerializationFormatVersion);
  return header;
}

}  // namespace

void ToPbStream(
    const mapping::PoseGraph& pose_graph,
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    ProtoStreamWriterInterface* const writer) {
  writer->WriteProto(CreateHeader());

  mapping::proto::SerializedData proto;
  // We serialize the pose graph followed by all the data referenced in it.
  *proto.mutable_pose_graph() = pose_graph.ToProto();
  writer->WriteProto(proto);
  *proto.mutable_all_trajectory_builder_options() =
      CreateAllTrajectoryBuilderOptionsProto(trajectory_builder_options);
  writer->WriteProto(proto);

  // Next serialize all submaps.
  for (const auto& submap_id_data : pose_graph.GetAllSubmapData()) {
    auto* const submap_proto = proto.mutable_submap();
    submap_proto->mutable_submap_id()->set_trajectory_id(
        submap_id_data.id.trajectory_id);
    submap_proto->mutable_submap_id()->set_submap_index(
        submap_id_data.id.submap_index);
    submap_id_data.data.submap->ToProto(submap_proto,
                                        /*include_probability_grid_data=*/true);
    writer->WriteProto(proto);
  }
  // Next we serialize all node data.
  for (const auto& node_id_data : pose_graph.GetTrajectoryNodes()) {
    auto* const node_proto = proto.mutable_node();
    node_proto->mutable_node_id()->set_trajectory_id(
        node_id_data.id.trajectory_id);
    node_proto->mutable_node_id()->set_node_index(node_id_data.id.node_index);
    *node_proto->mutable_node_data() =
        ToProto(*node_id_data.data.constant_data);
    writer->WriteProto(proto);
  }
  // Next we serialize all trajectory data.
  {
    const auto all_trajectory_data = pose_graph.GetTrajectoryData();
    for (const auto& trajectory_data : all_trajectory_data) {
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
  // Next we serialize IMU data from the pose graph.
  {
    const auto all_imu_data = pose_graph.GetImuData();
    for (const int trajectory_id : all_imu_data.trajectory_ids()) {
      for (const auto& imu_data : all_imu_data.trajectory(trajectory_id)) {
        auto* const imu_data_proto = proto.mutable_imu_data();
        imu_data_proto->set_trajectory_id(trajectory_id);
        *imu_data_proto->mutable_imu_data() = sensor::ToProto(imu_data);
        writer->WriteProto(proto);
      }
    }
  }
  // Next we serialize odometry data from the pose graph.
  {
    const auto all_odometry_data = pose_graph.GetOdometryData();
    for (const int trajectory_id : all_odometry_data.trajectory_ids()) {
      for (const auto& odometry_data :
           all_odometry_data.trajectory(trajectory_id)) {
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
    const auto all_fixed_frame_pose_data = pose_graph.GetFixedFramePoseData();
    for (const int trajectory_id : all_fixed_frame_pose_data.trajectory_ids()) {
      for (const auto& fixed_frame_pose_data :
           all_fixed_frame_pose_data.trajectory(trajectory_id)) {
        auto* const fixed_frame_pose_data_proto =
            proto.mutable_fixed_frame_pose_data();
        fixed_frame_pose_data_proto->set_trajectory_id(trajectory_id);
        *fixed_frame_pose_data_proto->mutable_fixed_frame_pose_data() =
            sensor::ToProto(fixed_frame_pose_data);
        writer->WriteProto(proto);
      }
    }
  }
  // Next we serialize all landmark data.
  {
    const std::map<std::string /* landmark ID */,
                   mapping::PoseGraph::LandmarkNode>
        all_landmark_nodes = pose_graph.GetLandmarkNodes();
    for (const auto& node : all_landmark_nodes) {
      for (const auto& observation : node.second.landmark_observations) {
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

}  // namespace io
}  // namespace cartographer
