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
#include <vector>

#include "cartographer/mapping/proto/internal/legacy_serialized_data.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/stream_format_migration.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

using mapping::proto::SerializedData;

struct LegacyData {
  SerializedData pose_graph;
  SerializedData all_trajectory_builder_options;
  std::vector<SerializedData> submaps;
  std::vector<SerializedData> nodes;
  std::vector<SerializedData> imu_data;
  std::vector<SerializedData> odometry_data;
  std::vector<SerializedData> fixed_frame_pose_data;
  std::vector<SerializedData> trajectory_data;
  std::vector<SerializedData> landmark_data;
};

LegacyData ParseLegacyData(
    cartographer::io::ProtoStreamReaderInterface* const input) {
  LegacyData legacy_data;
  CHECK(input->ReadProto(legacy_data.pose_graph.mutable_pose_graph()))
      << "Input stream seems to differ from original stream format. Could not "
         "read PoseGraph as first message.";
  CHECK(input->ReadProto(legacy_data.all_trajectory_builder_options
                             .mutable_all_trajectory_builder_options()))
      << "Input stream seems to differ from original stream format. Could not "
         "read AllTrajectoryBuilderOptions as second message.";

  mapping::proto::LegacySerializedData data;
  while (input->ReadProto(&data)) {
    if (data.has_submap()) {
      legacy_data.submaps.push_back(SerializedData());
      *legacy_data.submaps.back().mutable_submap() = data.submap();
      data.clear_submap();
    }
    if (data.has_node()) {
      legacy_data.nodes.push_back(SerializedData());
      *legacy_data.nodes.back().mutable_node() = data.node();
      data.clear_node();
    }
    if (data.has_imu_data()) {
      legacy_data.imu_data.push_back(SerializedData());
      *legacy_data.imu_data.back().mutable_imu_data() = data.imu_data();
      data.clear_imu_data();
    }
    if (data.has_odometry_data()) {
      legacy_data.odometry_data.push_back(SerializedData());
      *legacy_data.odometry_data.back().mutable_odometry_data() =
          data.odometry_data();
      data.clear_odometry_data();
    }
    if (data.has_fixed_frame_pose_data()) {
      legacy_data.fixed_frame_pose_data.push_back(SerializedData());
      *legacy_data.fixed_frame_pose_data.back()
           .mutable_fixed_frame_pose_data() = data.fixed_frame_pose_data();
      data.clear_fixed_frame_pose_data();
    }
    if (data.has_trajectory_data()) {
      legacy_data.trajectory_data.push_back(SerializedData());
      *legacy_data.trajectory_data.back().mutable_trajectory_data() =
          data.trajectory_data();
      data.clear_trajectory_data();
    }
    if (data.has_landmark_data()) {
      legacy_data.landmark_data.push_back(SerializedData());
      *legacy_data.landmark_data.back().mutable_landmark_data() =
          data.landmark_data();
      data.clear_landmark_data();
    }
  }
  return legacy_data;
}

mapping::proto::SerializationHeader CreateSerializationHeader() {
  constexpr uint32_t kVersion1 = 1;
  mapping::proto::SerializationHeader header;
  header.set_format_version(kVersion1);
  return header;
}

void SerializeToVersion1Format(
    const LegacyData& legacy_data,
    cartographer::io::ProtoStreamWriterInterface* const output) {
  output->WriteProto(CreateSerializationHeader());
  output->WriteProto(legacy_data.pose_graph);
  output->WriteProto(legacy_data.all_trajectory_builder_options);
  for (const auto& submap : legacy_data.submaps) {
    output->WriteProto(submap);
  }
  for (const auto& node : legacy_data.nodes) {
    output->WriteProto(node);
  }
  for (const auto& trajectory : legacy_data.trajectory_data) {
    output->WriteProto(trajectory);
  }
  for (const auto& imu : legacy_data.imu_data) {
    output->WriteProto(imu);
  }
  for (const auto& odometry : legacy_data.odometry_data) {
    output->WriteProto(odometry);
  }
  for (const auto& fixed_frame_pose : legacy_data.fixed_frame_pose_data) {
    output->WriteProto(fixed_frame_pose);
  }
  for (const auto& landmark : legacy_data.landmark_data) {
    output->WriteProto(landmark);
  }
}

}  // namespace

void MigrateStreamFormatToVersion1(
    cartographer::io::ProtoStreamReaderInterface* const input,
    cartographer::io::ProtoStreamWriterInterface* const output) {
  SerializeToVersion1Format(ParseLegacyData(input), output);
}

}  // namespace mapping
}  // namespace cartographer
