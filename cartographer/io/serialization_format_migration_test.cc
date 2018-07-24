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
 private:
  template <class LegacySerializedDataType>
  void AddLegacyDataToReader(InMemoryProtoStreamReader& reader) {
    mapping::proto::PoseGraph pose_graph;
    mapping::proto::AllTrajectoryBuilderOptions all_options;
    LegacySerializedDataType submap;
    submap.mutable_submap();
    LegacySerializedDataType node;
    node.mutable_node();
    LegacySerializedDataType imu_data;
    imu_data.mutable_imu_data();
    LegacySerializedDataType odometry_data;
    odometry_data.mutable_odometry_data();
    LegacySerializedDataType fixed_frame_pose;
    fixed_frame_pose.mutable_fixed_frame_pose_data();
    LegacySerializedDataType trajectory_data;
    trajectory_data.mutable_trajectory_data();
    LegacySerializedDataType landmark_data;
    landmark_data.mutable_landmark_data();

    reader.AddProto(pose_graph);
    reader.AddProto(all_options);
    reader.AddProto(submap);
    reader.AddProto(node);
    reader.AddProto(imu_data);
    reader.AddProto(odometry_data);
    reader.AddProto(fixed_frame_pose);
    reader.AddProto(trajectory_data);
    reader.AddProto(landmark_data);
  }

 protected:
  void SetUp() {
    AddLegacyDataToReader<mapping::proto::LegacySerializedData>(reader_);
    AddLegacyDataToReader<mapping::proto::LegacySerializedDataLegacySubmap>(
        reader_for_migrating_grid_);

    writer_.reset(new ForwardingProtoStreamWriter(
        [this](const google::protobuf::Message* proto) -> bool {
          std::string msg_string;
          TextFormat::PrintToString(*proto, &msg_string);
          this->output_messages_.push_back(msg_string);
          return true;
        }));
    writer_for_migrating_grid_.reset(new ForwardingProtoStreamWriter(
        [this](const google::protobuf::Message* proto) -> bool {
          std::string msg_string;
          TextFormat::PrintToString(*proto, &msg_string);
          this->output_messages_after_migrating_grid_.push_back(msg_string);
          return true;
        }));
  }

  InMemoryProtoStreamReader reader_;
  InMemoryProtoStreamReader reader_for_migrating_grid_;
  std::unique_ptr<ForwardingProtoStreamWriter> writer_;
  std::unique_ptr<ForwardingProtoStreamWriter> writer_for_migrating_grid_;
  std::vector<std::string> output_messages_;
  std::vector<std::string> output_messages_after_migrating_grid_;

  static constexpr int kNumOriginalMessages = 9;
};

TEST_F(MigrationTest, MigrationAddsHeaderAsFirstMessage) {
  MigrateStreamFormatToVersion1(&reader_, writer_.get(),
                                false /* migrate_grid_format */);
  // We expect one message more than the original number of messages, because of
  // the added header.
  EXPECT_THAT(output_messages_, SizeIs(kNumOriginalMessages + 1));

  mapping::proto::SerializationHeader header;
  EXPECT_TRUE(TextFormat::ParseFromString(output_messages_[0], &header));
  EXPECT_THAT(header.format_version(), Eq(1));
}

TEST_F(MigrationTest, MigrationWithGridMigrationAddsHeaderAsFirstMessage) {
  MigrateStreamFormatToVersion1(&reader_for_migrating_grid_,
                                writer_for_migrating_grid_.get(),
                                true /* migrate_grid_format */);
  // We expect one message more than the original number of messages, because of
  // the added header.
  EXPECT_THAT(output_messages_after_migrating_grid_,
              SizeIs(kNumOriginalMessages + 1));

  mapping::proto::SerializationHeader header;
  EXPECT_TRUE(TextFormat::ParseFromString(
      output_messages_after_migrating_grid_[0], &header));
  EXPECT_THAT(header.format_version(), Eq(1));
}

TEST_F(MigrationTest, SerializedDataOrderIsCorrect) {
  MigrateStreamFormatToVersion1(&reader_, writer_.get(),
                                false /* migrate_grid_format */);
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

TEST_F(MigrationTest, SerializedDataOrderAfterGridMigrationIsCorrect) {
  MigrateStreamFormatToVersion1(&reader_for_migrating_grid_,
                                writer_for_migrating_grid_.get(),
                                true /* migrate_grid_format */);
  EXPECT_THAT(output_messages_after_migrating_grid_,
              SizeIs(kNumOriginalMessages + 1));

  std::vector<mapping::proto::SerializedData> serialized(
      output_messages_after_migrating_grid_.size() - 1);
  for (size_t i = 1; i < output_messages_after_migrating_grid_.size(); ++i) {
    EXPECT_TRUE(TextFormat::ParseFromString(
        output_messages_after_migrating_grid_[i], &serialized[i - 1]));
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
