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

#include "cartographer/mapping/internal/3d/pose_graph_3d.h"

#include "cartographer/mapping/internal/test_helpers.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"
#include "google/protobuf/util/message_differencer.h"

namespace cartographer {
namespace mapping {
namespace {

class PoseGraph3DForTesting : public PoseGraph3D {
 public:
  PoseGraph3DForTesting(const proto::PoseGraphOptions& options,
                        common::ThreadPool* thread_pool)
      : PoseGraph3D(options, thread_pool) {}

  void WaitForAllComputations() { PoseGraph3D::WaitForAllComputations(); }
};

class TestEnvironment {
 public:
  TestEnvironment()
      : thread_pool_(common::make_unique<common::ThreadPool>(1)) {}

  TestEnvironment& SetDefault() {
    const std::string kPoseGraphLua = R"text(
      include "pose_graph.lua"
      return POSE_GRAPH)text";
    auto pose_graph_parameters = test::ResolveLuaParameters(kPoseGraphLua);
    pose_graph_options_ = CreatePoseGraphOptions(pose_graph_parameters.get());
    return *this;
  }

  std::unique_ptr<PoseGraph3DForTesting> Build() {
    return common::make_unique<PoseGraph3DForTesting>(pose_graph_options_,
                                                      thread_pool_.get());
  }

 private:
  proto::PoseGraphOptions pose_graph_options_;
  std::unique_ptr<common::ThreadPool> thread_pool_;
};

TEST(PoseGraph3DTest, Empty) {
  TestEnvironment builder;
  auto pose_graph = builder.SetDefault().Build();
  pose_graph->WaitForAllComputations();
  EXPECT_TRUE(pose_graph->GetAllSubmapData().empty());
  EXPECT_TRUE(pose_graph->constraints().empty());
  EXPECT_TRUE(pose_graph->GetConnectedTrajectories().empty());
  EXPECT_TRUE(pose_graph->GetAllSubmapPoses().empty());
  EXPECT_TRUE(pose_graph->GetTrajectoryNodes().empty());
  EXPECT_TRUE(pose_graph->GetTrajectoryNodePoses().empty());
  EXPECT_TRUE(pose_graph->GetTrajectoryData().empty());
  proto::PoseGraph empty_proto;
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      pose_graph->ToProto(), empty_proto));
}

TEST(PoseGraph3DTest, BasicSerialization) {
  TestEnvironment builder;
  auto pose_graph = builder.SetDefault().Build();
  proto::PoseGraph proto;

  auto fake_node = test::CreateFakeNode();
  test::AddToProtoGraph(fake_node, &proto);
  pose_graph->AddNodeFromProto(transform::Rigid3d::Identity(), fake_node);
  auto fake_submap = test::CreateFakeSubmap3D();
  test::AddToProtoGraph(fake_submap, &proto);
  pose_graph->AddSubmapFromProto(transform::Rigid3d::Identity(), fake_submap);
  test::AddToProtoGraph(test::CreateFakeConstraint(fake_node, fake_submap),
                        &proto);
  pose_graph->AddSerializedConstraints(FromProto(proto.constraint()));

  pose_graph->WaitForAllComputations();
  proto::PoseGraph actual_proto = pose_graph->ToProto();
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.constraint(0), actual_proto.constraint(0)));
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.trajectory(0).node(0), actual_proto.trajectory(0).node(0)));
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.trajectory(0).submap(0), actual_proto.trajectory(0).submap(0)));
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.trajectory(0), actual_proto.trajectory(0)));
  EXPECT_TRUE(
      google::protobuf::util::MessageDifferencer::Equals(proto, actual_proto));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
