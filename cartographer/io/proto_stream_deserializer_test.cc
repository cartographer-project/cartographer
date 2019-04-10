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

#include "cartographer/io/proto_stream_deserializer.h"

#include <memory>

#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "cartographer/io/testing/test_helpers.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

using ::cartographer::io::testing::ProtoFromStringOrDie;
using ::cartographer::io::testing::ProtoReaderFromStrings;
using ::cartographer::mapping::proto::SerializationHeader;
using ::cartographer::mapping::proto::SerializedData;
using ::google::protobuf::Message;
using ::google::protobuf::util::MessageDifferencer;
using ::testing::Eq;
using ::testing::Not;

constexpr char kSerializationHeaderProtoString[] = "format_version: 1";
constexpr char kUnsupportedSerializationHeaderProtoString[] =
    "format_version: 123";
constexpr char kPoseGraphProtoString[] = "pose_graph {}";
constexpr char kAllTrajectoryBuilderOptionsProtoString[] =
    "all_trajectory_builder_options {}";
constexpr char kSubmapProtoString[] = "submap {}";
constexpr char kNodeProtoString[] = "node {}";
constexpr char kTrajectoryDataProtoString[] = "trajectory_data {}";
constexpr char kImuDataProtoString[] = "imu_data {}";
constexpr char kOdometryDataProtoString[] = "odometry_data {}";
constexpr char kFixedFramePoseDataProtoString[] = "fixed_frame_pose_data {}";
constexpr char kLandmarkDataProtoString[] = "landmark_data {}";

class ProtoStreamDeserializerTest : public ::testing::Test {
 protected:
  std::unique_ptr<InMemoryProtoStreamReader> reader_;
};

// This test checks if the serialization works.
TEST_F(ProtoStreamDeserializerTest, WorksOnGoldenTextStream) {
  // Load text proto into in_memory_reader.
  reader_ = ProtoReaderFromStrings(kSerializationHeaderProtoString,
                                   {
                                       kPoseGraphProtoString,
                                       kAllTrajectoryBuilderOptionsProtoString,
                                       kSubmapProtoString,
                                       kNodeProtoString,
                                       kTrajectoryDataProtoString,
                                       kImuDataProtoString,
                                       kOdometryDataProtoString,
                                       kFixedFramePoseDataProtoString,
                                       kLandmarkDataProtoString,
                                   });

  io::ProtoStreamDeserializer deserializer(reader_.get());

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
  EXPECT_TRUE(reader_->eof());
}

TEST_F(ProtoStreamDeserializerTest, FailsIfVersionNotSupported) {
  reader_ =
      ProtoReaderFromStrings(kUnsupportedSerializationHeaderProtoString, {});
  EXPECT_DEATH(absl::make_unique<ProtoStreamDeserializer>(reader_.get()),
               "Unsupported serialization format");
}

}  // namespace
}  // namespace io
}  // namespace cartographer
