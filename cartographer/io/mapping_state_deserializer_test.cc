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

#include <memory>
#include <sstream>

#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "cartographer/io/internal/testing/serialized_mapping_state_text_proto.h"
#include "cartographer/io/mapping_state_deserializer.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

using common::make_unique;
using google::protobuf::Message;
using mapping::proto::AllTrajectoryBuilderOptions;
using mapping::proto::PoseGraph;
using mapping::proto::SerializationHeader;
using mapping::proto::SerializedData;
using ::testing::Eq;
using ::testing::Not;

constexpr char kProtoDelim = '#';

// open text file with a set of text-protos that mimic a pbstream.
std::unique_ptr<InMemoryProtoStreamReader> CreateInMemoryReaderFromTextProto(
    const std::string& file_string) {
  std::queue<std::unique_ptr<Message>> proto_queue;
  std::istringstream in_stream(file_string);

  // Parse the header.
  std::string proto_string;
  proto_queue.push(make_unique<mapping::proto::SerializationHeader>());
  std::getline(in_stream, proto_string, kProtoDelim);
  CHECK(google::protobuf::TextFormat::ParseFromString(
      proto_string, proto_queue.back().get()));

  // Parse all the remaining SerializedData protos
  while (std::getline(in_stream, proto_string, kProtoDelim)) {
    proto_queue.push(make_unique<SerializedData>());
    CHECK(google::protobuf::TextFormat::ParseFromString(
        proto_string, proto_queue.back().get()));
  }
  return make_unique<InMemoryProtoStreamReader>(std::move(proto_queue));
}

std::unique_ptr<Message> CreateSerializationHeader(uint32_t version) {
  auto header = common::make_unique<SerializationHeader>();
  header->set_format_version(version);
  return header;
}

// This test checks, if the serialization works.
TEST(MappingStateDeserializerTest, WorksOnGoldenTextStream) {
  // Load text proto into in_memory_reader.
  std::unique_ptr<InMemoryProtoStreamReader> reader =
      CreateInMemoryReaderFromTextProto(testing::kSerializedMappingStateStream);

  io::MappingStateDeserializer deserializer(reader.get());

  const auto& pose_graph = deserializer.pose_graph();
  // One constraint
  ASSERT_THAT(pose_graph.constraint_size(), Eq(1));
  const auto& constraint0 = pose_graph.constraint(0);
  EXPECT_TRUE(constraint0.has_submap_id());
  EXPECT_TRUE(constraint0.has_node_id());
  EXPECT_TRUE(constraint0.has_relative_pose());

  const auto& submap_id = constraint0.submap_id();
  EXPECT_THAT(submap_id.trajectory_id(), Eq(0));
  EXPECT_THAT(submap_id.submap_index(), Eq(0));

  const auto& node_id = constraint0.node_id();
  EXPECT_THAT(node_id.trajectory_id(), Eq(0));
  EXPECT_THAT(node_id.node_index(), Eq(0));

  const auto& relative_pose = constraint0.relative_pose();
  EXPECT_THAT(relative_pose.translation().x(), Eq(0.0));
  EXPECT_THAT(relative_pose.translation().y(), Eq(0.0));
  EXPECT_THAT(relative_pose.translation().z(), Eq(0.0));
  EXPECT_THAT(relative_pose.rotation().x(), Eq(0.0));
  EXPECT_THAT(relative_pose.rotation().y(), Eq(0.0));
  EXPECT_THAT(relative_pose.rotation().z(), Eq(0.0));
  EXPECT_THAT(relative_pose.rotation().w(), Eq(1.0));

  EXPECT_THAT(constraint0.translation_weight(), Eq(1.0));
  EXPECT_THAT(constraint0.rotation_weight(), Eq(1.0));
  EXPECT_THAT(constraint0.tag(),
              Eq(mapping::proto::PoseGraph_Constraint_Tag_INTRA_SUBMAP));

  // One trajectory
  ASSERT_THAT(pose_graph.trajectory_size(), Eq(1));

  const auto& trajectory_builder_options =
      deserializer.all_trajectory_builder_options();
  ASSERT_THAT(trajectory_builder_options.options_with_sensor_ids_size(), Eq(1));
  const auto& options0 = trajectory_builder_options.options_with_sensor_ids(0);
  ASSERT_THAT(options0.sensor_id_size(), Eq(2));
  EXPECT_THAT(options0.sensor_id(0).type(),
              Eq(mapping::proto::SensorId_SensorType_RANGE));
  EXPECT_THAT(options0.sensor_id(0).id(), Eq("laser_scanner_0"));
  EXPECT_THAT(options0.sensor_id(1).type(),
              Eq(mapping::proto::SensorId_SensorType_IMU));
  EXPECT_THAT(options0.sensor_id(1).id(), Eq("imu_0"));
  EXPECT_THAT(options0.has_trajectory_builder_options(), Eq(true));
  ASSERT_THAT(
      options0.trajectory_builder_options().has_trajectory_builder_2d_options(),
      Eq(true));
  EXPECT_THAT(
      options0.trajectory_builder_options().has_trajectory_builder_3d_options(),
      Eq(false));

  const auto& options_2d =
      options0.trajectory_builder_options().trajectory_builder_2d_options();
  EXPECT_THAT(options_2d.min_range(), Eq(0.1f));
  EXPECT_THAT(options_2d.max_range(), Eq(40.0f));
  EXPECT_THAT(options_2d.min_z(), Eq(0.0f));
  EXPECT_THAT(options_2d.max_z(), Eq(2.0f));
  EXPECT_THAT(options0.trajectory_builder_options().pure_localization(),
              Eq(false));
  const auto& initial_pose =
      options0.trajectory_builder_options().initial_trajectory_pose();
  EXPECT_THAT(initial_pose.relative_pose().translation().x(), Eq(0.0));
  EXPECT_THAT(initial_pose.relative_pose().translation().y(), Eq(0.0));
  EXPECT_THAT(initial_pose.relative_pose().translation().z(), Eq(0.0));
  EXPECT_THAT(initial_pose.relative_pose().translation().x(), Eq(0.0));
  EXPECT_THAT(initial_pose.relative_pose().rotation().x(), Eq(0.0));
  EXPECT_THAT(initial_pose.relative_pose().rotation().y(), Eq(0.0));
  EXPECT_THAT(initial_pose.relative_pose().rotation().z(), Eq(0.0));
  EXPECT_THAT(initial_pose.relative_pose().rotation().w(), Eq(1.0));
  EXPECT_THAT(initial_pose.to_trajectory_id(), Eq(0));
  EXPECT_THAT(initial_pose.timestamp(), Eq(0));

  SerializedData serialized_data;
  EXPECT_THAT(deserializer.GetNextSerializedData(&serialized_data), Eq(true));
  EXPECT_THAT(serialized_data.has_submap(), Eq(true));
  EXPECT_THAT(deserializer.GetNextSerializedData(&serialized_data), Eq(true));
  EXPECT_THAT(serialized_data.has_node(), Eq(true));
  EXPECT_THAT(deserializer.GetNextSerializedData(&serialized_data), Eq(false));
  EXPECT_THAT(reader->eof(), Eq(true));
}

// Currently we only support one version of the format: format_version == 1.
TEST(MappingStateDeserializerDeathTests, FailsIfVersionNotSupported) {
  std::queue<std::unique_ptr<Message>> proto_queue;
  proto_queue.push(CreateSerializationHeader(/*version = */ 0));
  InMemoryProtoStreamReader reader(std::move(proto_queue));

  EXPECT_DEATH(common::make_unique<MappingStateDeserializer>(&reader),
               "Unsupported serialization format");
}

}  // namespace
}  // namespace io
}  // namespace cartographer
