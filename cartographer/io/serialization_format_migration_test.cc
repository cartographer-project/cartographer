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

#include <functional>
#include <memory>

#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "cartographer/mapping/proto/internal/legacy_serialized_data.pb.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

using ::google::protobuf::TextFormat;
using ::testing::Eq;
using ::testing::SizeIs;

class MigrationTest : public ::testing::Test {
 protected:
  void SetUp() {
    writer_.reset(new ForwardingProtoStreamWriter(
        [this](const google::protobuf::Message* proto) -> bool {
          std::string msg_string;
          TextFormat::PrintToString(*proto, &msg_string);
          this->output_messages_.push_back(msg_string);
          return true;
        }));

    mapping::proto::PoseGraph pose_graph;
    mapping::proto::AllTrajectoryBuilderOptions all_options;
    mapping::proto::LegacySerializedData submap;
    submap.mutable_submap();
    mapping::proto::LegacySerializedData node;
    node.mutable_node();
    mapping::proto::LegacySerializedData imu_data;
    imu_data.mutable_imu_data();
    mapping::proto::LegacySerializedData odometry_data;
    odometry_data.mutable_odometry_data();
    mapping::proto::LegacySerializedData fixed_frame_pose;
    fixed_frame_pose.mutable_fixed_frame_pose_data();
    mapping::proto::LegacySerializedData trajectory_data;
    trajectory_data.mutable_trajectory_data();
    mapping::proto::LegacySerializedData landmark_data;
    landmark_data.mutable_landmark_data();

    reader_.AddProto(pose_graph);
    reader_.AddProto(all_options);
    reader_.AddProto(submap);
    reader_.AddProto(node);
    reader_.AddProto(imu_data);
    reader_.AddProto(odometry_data);
    reader_.AddProto(fixed_frame_pose);
    reader_.AddProto(trajectory_data);
    reader_.AddProto(landmark_data);
  }

  InMemoryProtoStreamReader reader_;
  std::unique_ptr<ForwardingProtoStreamWriter> writer_;
  std::vector<std::string> output_messages_;

  static constexpr int kNumOriginalMessages = 9;
};

TEST_F(MigrationTest, MigrationAddsHeaderAsFirstMessage) {
  MigrateStreamFormatToVersion1(&reader_, writer_.get());
  // We expect one message more than the original number of messages, because of
  // the added header.
  EXPECT_THAT(output_messages_, SizeIs(kNumOriginalMessages + 1));

  mapping::proto::SerializationHeader header;
  EXPECT_TRUE(TextFormat::ParseFromString(output_messages_[0], &header));
  EXPECT_THAT(header.format_version(), Eq<uint32>(1));
}

TEST_F(MigrationTest, SerializedDataOrderIsCorrect) {
  MigrateStreamFormatToVersion1(&reader_, writer_.get());
  EXPECT_THAT(output_messages_, SizeIs(kNumOriginalMessages + 1));

  std::vector<mapping::proto::SerializedData> serialized(
      output_messages_.size() - 1);
  for (size_t i = 1; i < output_messages_.size(); ++i) {
    EXPECT_TRUE(
        TextFormat::ParseFromString(output_messages_[i], &serialized[i - 1]));
  }

  EXPECT_TRUE(serialized[0].has_pose_graph());
  EXPECT_TRUE(serialized[1].has_all_trajectory_builder_options());
  EXPECT_TRUE(serialized[2].has_submap());
  EXPECT_TRUE(serialized[3].has_node());
  EXPECT_TRUE(serialized[4].has_trajectory_data());
  EXPECT_TRUE(serialized[5].has_imu_data());
  EXPECT_TRUE(serialized[6].has_odometry_data());
  EXPECT_TRUE(serialized[7].has_fixed_frame_pose_data());
  EXPECT_TRUE(serialized[8].has_landmark_data());
}

}  // namespace
}  // namespace io
}  // namespace cartographer
