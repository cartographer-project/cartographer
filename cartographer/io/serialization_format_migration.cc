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

#include "cartographer/io/serialization_format_migration.h"

#include <unordered_map>
#include <vector>

#include "cartographer/mapping/proto/internal/legacy_serialized_data.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

using mapping::proto::SerializedData;
using ProtoMap = std::unordered_map<int, std::vector<SerializedData>>;

bool ReadPoseGraph(cartographer::io::ProtoStreamReaderInterface* const input,
                   ProtoMap* proto_map) {
  auto& pose_graph_vec = (*proto_map)[SerializedData::kPoseGraph];
  pose_graph_vec.emplace_back();
  return input->ReadProto(pose_graph_vec.back().mutable_pose_graph());
}

bool ReadBuilderOptions(
    cartographer::io::ProtoStreamReaderInterface* const input,
    ProtoMap* proto_map) {
  auto& options_vec =
      (*proto_map)[SerializedData::kAllTrajectoryBuilderOptions];
  options_vec.emplace_back();
  return input->ReadProto(
      options_vec.back().mutable_all_trajectory_builder_options());
}

bool DeserializeNext(cartographer::io::ProtoStreamReaderInterface* const input,
                     ProtoMap* proto_map) {
  mapping::proto::LegacySerializedData legacy_data;
  if (!input->ReadProto(&legacy_data)) return false;

  if (legacy_data.has_submap()) {
    auto& output_vector = (*proto_map)[SerializedData::kSubmapFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_submap() = legacy_data.submap();
  }
  if (legacy_data.has_node()) {
    auto& output_vector = (*proto_map)[SerializedData::kNodeFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_node() = legacy_data.node();
  }
  if (legacy_data.has_trajectory_data()) {
    auto& output_vector =
        (*proto_map)[SerializedData::kTrajectoryDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_trajectory_data() =
        legacy_data.trajectory_data();
  }
  if (legacy_data.has_imu_data()) {
    auto& output_vector = (*proto_map)[SerializedData::kImuDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_imu_data() = legacy_data.imu_data();
  }
  if (legacy_data.has_odometry_data()) {
    auto& output_vector = (*proto_map)[SerializedData::kOdometryData];
    output_vector.emplace_back();
    *output_vector.back().mutable_odometry_data() = legacy_data.odometry_data();
  }
  if (legacy_data.has_fixed_frame_pose_data()) {
    auto& output_vector =
        (*proto_map)[SerializedData::kFixedFramePoseDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_fixed_frame_pose_data() =
        legacy_data.fixed_frame_pose_data();
  }
  if (legacy_data.has_landmark_data()) {
    auto& output_vector =
        (*proto_map)[SerializedData::kLandmarkDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_landmark_data() = legacy_data.landmark_data();
  }
  return true;
}

ProtoMap ParseLegacyData(
    cartographer::io::ProtoStreamReaderInterface* const input) {
  ProtoMap proto_map;
  CHECK(ReadPoseGraph(input, &proto_map))
      << "Input stream seems to differ from original stream format. Could "
         "not "
         "read PoseGraph as first message.";
  CHECK(ReadBuilderOptions(input, &proto_map))
      << "Input stream seems to differ from original stream format. Could "
         "not "
         "read AllTrajectoryBuilderOptions as second message.";
  do {
  } while (DeserializeNext(input, &proto_map));
  return proto_map;
}

mapping::proto::SerializationHeader CreateSerializationHeader() {
  constexpr uint32_t kVersion1 = 1;
  mapping::proto::SerializationHeader header;
  header.set_format_version(kVersion1);
  return header;
}

void SerializeToVersion1Format(
    const ProtoMap& deserialized_data,
    cartographer::io::ProtoStreamWriterInterface* const output) {
  const std::vector<int> kFieldSerializationOrder = {
      SerializedData::kPoseGraphFieldNumber,
      SerializedData::kAllTrajectoryBuilderOptionsFieldNumber,
      SerializedData::kSubmapFieldNumber,
      SerializedData::kNodeFieldNumber,
      SerializedData::kTrajectoryDataFieldNumber,
      SerializedData::kImuDataFieldNumber,
      SerializedData::kOdometryDataFieldNumber,
      SerializedData::kFixedFramePoseDataFieldNumber,
      SerializedData::kLandmarkDataFieldNumber};

  output->WriteProto(CreateSerializationHeader());
  for (auto field_index : kFieldSerializationOrder) {
    const auto proto_vector_it = deserialized_data.find(field_index);
    if (proto_vector_it == deserialized_data.end()) continue;
    for (const auto& proto : proto_vector_it->second) {
      output->WriteProto(proto);
    }
  }
}
}  // namespace

void MigrateStreamFormatToVersion1(
    cartographer::io::ProtoStreamReaderInterface* const input,
    cartographer::io::ProtoStreamWriterInterface* const output) {
  SerializeToVersion1Format(ParseLegacyData(input), output);
}

}  // namespace io
}  // namespace cartographer
