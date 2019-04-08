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

#include "cartographer/io/internal/in_memory_proto_stream.h"

#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

using absl::make_unique;
using google::protobuf::Message;
using mapping::proto::PoseGraph;
using mapping::proto::SerializedData;

class InMemoryProtoStreamTest : public ::testing::Test {
 protected:
  void SetUp() override {
    pose_graph_.add_trajectory()->set_trajectory_id(1);
    serialized_data_.mutable_odometry_data()->set_trajectory_id(2);
  }

  PoseGraph pose_graph_;
  SerializedData serialized_data_;
};

TEST_F(InMemoryProtoStreamTest, ReadStreamInitializedFromQueue) {
  std::queue<std::unique_ptr<Message>> proto_queue;
  proto_queue.push(make_unique<PoseGraph>(pose_graph_));
  proto_queue.push(make_unique<SerializedData>(serialized_data_));

  InMemoryProtoStreamReader reader(std::move(proto_queue));

  PoseGraph actual_pose_graph;
  EXPECT_FALSE(reader.eof());
  EXPECT_TRUE(reader.ReadProto(&actual_pose_graph));
  EXPECT_EQ(1, actual_pose_graph.trajectory(0).trajectory_id());

  SerializedData actual_serialized_data;
  EXPECT_FALSE(reader.eof());
  EXPECT_TRUE(reader.ReadProto(&actual_serialized_data));
  EXPECT_EQ(2, actual_serialized_data.odometry_data().trajectory_id());

  EXPECT_TRUE(reader.eof());
}

TEST_F(InMemoryProtoStreamTest, ReadStreamInitializedIncrementally) {
  InMemoryProtoStreamReader reader;
  reader.AddProto(pose_graph_);
  reader.AddProto(serialized_data_);

  PoseGraph actual_pose_graph;
  EXPECT_FALSE(reader.eof());
  EXPECT_TRUE(reader.ReadProto(&actual_pose_graph));
  EXPECT_EQ(1, actual_pose_graph.trajectory(0).trajectory_id());

  SerializedData actual_serialized_data;
  EXPECT_FALSE(reader.eof());
  EXPECT_TRUE(reader.ReadProto(&actual_serialized_data));
  EXPECT_EQ(2, actual_serialized_data.odometry_data().trajectory_id());

  EXPECT_TRUE(reader.eof());
}

}  // namespace
}  // namespace io
}  // namespace cartographer
