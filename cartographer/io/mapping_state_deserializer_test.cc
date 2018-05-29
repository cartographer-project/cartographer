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
#include "cartographer/io/internal/testing/serialized_mapping_state_text_proto.h"
#include "cartographer/io/mapping_state_deserializer.h"
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

// open text file with a set of text-protos that mimic a pbstream.
std::unique_ptr<InMemoryProtoStreamReader>
CreateInMemoryReaderFromTextProtos() {
  std::queue<std::unique_ptr<Message>> proto_queue;

  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializationHeader>(
      testing::kSerializationHeaderProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(testing::kPoseGraphProtoString));
  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializedData>(
      testing::kAllTrajectoryBuilderOptionsProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(testing::kSubmapProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(testing::kNodeProtoString));
  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializedData>(
      testing::kTrajectoryDataProtoString));
  proto_queue.emplace(
      ProtoUPtrFromStringOrDie<SerializedData>(testing::kImuDataProtoString));
  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializedData>(
      testing::kOdometryDataProtoString));
  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializedData>(
      testing::kFixedFramePoseDataProtoString));
  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializedData>(
      testing::kLandmarkDataProtoString));
  return make_unique<InMemoryProtoStreamReader>(std::move(proto_queue));
}

// This test checks, if the serialization works.
TEST(MappingStateDeserializerTest, WorksOnGoldenTextStream) {
  // Load text proto into in_memory_reader.
  std::unique_ptr<InMemoryProtoStreamReader> reader =
      CreateInMemoryReaderFromTextProtos();

  io::MappingStateDeserializer deserializer(reader.get());

  EXPECT_TRUE(MessageDifferencer::Equals(
      deserializer.header(), ProtoFromStringOrDie<SerializationHeader>(
                                 testing::kSerializationHeaderProtoString)));

  EXPECT_TRUE(MessageDifferencer::Equals(
      deserializer.pose_graph(),
      ProtoFromStringOrDie<SerializedData>(testing::kPoseGraphProtoString)
          .pose_graph()));

  EXPECT_TRUE(MessageDifferencer::Equals(
      deserializer.all_trajectory_builder_options(),
      ProtoFromStringOrDie<SerializedData>(
          testing::kAllTrajectoryBuilderOptionsProtoString)
          .all_trajectory_builder_options()));

  SerializedData serialized_data;
  EXPECT_TRUE(deserializer.GetNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(testing::kSubmapProtoString)));

  EXPECT_TRUE(deserializer.GetNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(testing::kNodeProtoString)));

  EXPECT_TRUE(deserializer.GetNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data, ProtoFromStringOrDie<SerializedData>(
                           testing::kTrajectoryDataProtoString)));

  EXPECT_TRUE(deserializer.GetNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(testing::kImuDataProtoString)));

  EXPECT_TRUE(deserializer.GetNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(testing::kOdometryDataProtoString)));

  EXPECT_TRUE(deserializer.GetNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data, ProtoFromStringOrDie<SerializedData>(
                           testing::kFixedFramePoseDataProtoString)));

  EXPECT_TRUE(deserializer.GetNextSerializedData(&serialized_data));
  EXPECT_TRUE(MessageDifferencer::Equals(
      serialized_data,
      ProtoFromStringOrDie<SerializedData>(testing::kLandmarkDataProtoString)));

  EXPECT_FALSE(deserializer.GetNextSerializedData(&serialized_data));
  EXPECT_TRUE(reader->eof());
}

TEST(MappingStateDeserializerDeathTests, FailsIfVersionNotSupported) {
  std::queue<std::unique_ptr<Message>> proto_queue;
  proto_queue.emplace(ProtoUPtrFromStringOrDie<SerializationHeader>(
      testing::kUnsupportedSerializationHeaderProtoString));
  InMemoryProtoStreamReader reader(std::move(proto_queue));

  EXPECT_DEATH(common::make_unique<MappingStateDeserializer>(&reader),
               "Unsupported serialization format");
}

}  // namespace
}  // namespace io
}  // namespace cartographer
