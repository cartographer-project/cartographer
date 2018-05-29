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

#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

using ::cartographer::common::make_unique;
using ::cartographer::mapping::proto::SerializationHeader;
using ::cartographer::mapping::proto::SerializedData;
using ::google::protobuf::Message;
using ::google::protobuf::util::MessageDifferencer;
using ::testing::Eq;
using ::testing::Not;

static constexpr char kSerializationHeaderProtoString[] = R"PROTO(
  format_version: 1
)PROTO";

static constexpr char kUnsupportedSerializationHeaderProtoString[] = R"PROTO(
  format_version: 123
)PROTO";

static constexpr char kPoseGraphProtoString[] = R"PROTO(
  pose_graph {}
)PROTO";

static constexpr char kAllTrajectoryBuilderOptionsProtoString[] = R"PROTO(
  all_trajectory_builder_options {}
)PROTO";

static constexpr char kSubmapProtoString[] = R"PROTO(
  submap {}
)PROTO";

static constexpr char kNodeProtoString[] = R"PROTO(
  node {}
)PROTO";

static constexpr char kTrajectoryDataProtoString[] = R"PROTO(
  trajectory_data {}
)PROTO";

static constexpr char kImuDataProtoString[] = R"PROTO(
  imu_data {}
)PROTO";

static constexpr char kOdometryDataProtoString[] = R"PROTO(
  odometry_data {}
)PROTO";

static constexpr char kFixedFramePoseDataProtoString[] = R"PROTO(
  fixed_frame_pose_data {}
)PROTO";

static constexpr char kLandmarkDataProtoString[] = R"PROTO(
  landmark_data {}
)PROTO";

template <typename T>
T ProtoFromStringOrDie(const std::string& proto_string) {
  T msg;
  CHECK(google::protobuf::TextFormat::ParseFromString(proto_string, &msg));
  return msg;
}

template <typename T>
std::unique_ptr<T> ProtoUPtrFromStringOrDie(const std::string& proto_string) {
  return make_unique<T>(ProtoFromStringOrDie<T>(proto_string));
}

std::unique_ptr<InMemoryProtoStreamReader>
CreateInMemoryReaderFromTextProtos() {
  std::queue<std::unique_ptr<Message>> proto_queue;

  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializationHeader>(
      kSerializationHeaderProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(kPoseGraphProtoString));
  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializedData>(
      kAllTrajectoryBuilderOptionsProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(kSubmapProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(kNodeProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(kTrajectoryDataProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(kImuDataProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(kOdometryDataProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(kFixedFramePoseDataProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(kLandmarkDataProtoString));
  return make_unique<InMemoryProtoStreamReader>(std::move(proto_queue));
}

// This test checks if the serialization works.
TEST(ProtoStreamDeserializerTest, WorksOnGoldenTextStream) {
  // Load text proto into in_memory_reader.
  std::unique_ptr<InMemoryProtoStreamReader> reader =
      CreateInMemoryReaderFromTextProtos();

  io::ProtoStreamDeserializer deserializer(reader.get());

  EXPECT_TRUE(MessageDifferencer::Equals(
      deserializer.header(), ProtoFromStringOrDie<SerializationHeader>(
                                 kSerializationHeaderProtoString)));

  EXPECT_TRUE(MessageDifferencer::Equals(
      deserializer.pose_graph(),
      ProtoFromStringOrDie<SerializedData>(kPoseGraphProtoString)
          .pose_graph()));

  EXPECT_TRUE(
      MessageDifferencer::Equals(deserializer.all_trajectory_builder_options(),
                                 ProtoFromStringOrDie<SerializedData>(
                                     kAllTrajectoryBuilderOptionsProtoString)
                                     .all_trajectory_builder_options()));

  SerializedData serialized_data;
  EXPECT_TRUE(deserializer.ReadNextSerializedData(&serialized_data));
  // TODO(sebastianklose): Add matcher for protos in common place and use here.
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(kSubmapProtoString)));

  EXPECT_TRUE(deserializer.ReadNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data, ProtoFromStringOrDie<SerializedData>(kNodeProtoString)));

  EXPECT_TRUE(deserializer.ReadNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(kTrajectoryDataProtoString)));

  EXPECT_TRUE(deserializer.ReadNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(kImuDataProtoString)));

  EXPECT_TRUE(deserializer.ReadNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(kOdometryDataProtoString)));

  EXPECT_TRUE(deserializer.ReadNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(kFixedFramePoseDataProtoString)));

  EXPECT_TRUE(deserializer.ReadNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(kLandmarkDataProtoString)));

  EXPECT_FALSE(deserializer.ReadNextSerializedData(&serialized_data));
  EXPECT_TRUE(reader->eof());
}

TEST(ProtoStreamDeserializerDeathTests, FailsIfVersionNotSupported) {
  std::queue<std::unique_ptr<Message>> proto_queue;
  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializationHeader>(
      kUnsupportedSerializationHeaderProtoString));
  InMemoryProtoStreamReader reader(std::move(proto_queue));

  EXPECT_DEATH(common::make_unique<ProtoStreamDeserializer>(&reader),
               "Unsupported serialization format");
}

}  // namespace
}  // namespace io
}  // namespace cartographer
