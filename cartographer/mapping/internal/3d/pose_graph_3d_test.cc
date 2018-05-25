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

#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"
#include "google/protobuf/util/message_differencer.h"

namespace cartographer {
namespace mapping {
namespace {

using ::cartographer::mapping::optimization::OptimizationProblem3D;
using ::cartographer::mapping::optimization::proto::OptimizationProblemOptions;
using ::cartographer::transform::Rigid3d;

class MockOptimizationProblem3D : public OptimizationProblem3D {
 public:
  MockOptimizationProblem3D()
      : OptimizationProblem3D(OptimizationProblemOptions{}) {}
  ~MockOptimizationProblem3D() override = default;

  MOCK_METHOD3(Solve,
               void(const std::vector<Constraint> &, const std::set<int> &,
                    const std::map<std::string, LandmarkNode> &));
};

class PoseGraph3DForTesting : public PoseGraph3D {
 public:
  PoseGraph3DForTesting(
      const proto::PoseGraphOptions &options,
      std::unique_ptr<optimization::OptimizationProblem3D> optimization_problem,
      common::ThreadPool *thread_pool)
      : PoseGraph3D(options, std::move(optimization_problem), thread_pool) {}

  void WaitForAllComputations() { PoseGraph3D::WaitForAllComputations(); }
};

class PoseGraph3DTest : public ::testing::Test {
 protected:
  PoseGraph3DTest()
      : thread_pool_(common::make_unique<common::ThreadPool>(1)) {}

  void SetUp() override {
    const std::string kPoseGraphLua = R"text(
      include "pose_graph.lua"
      return POSE_GRAPH)text";
    auto pose_graph_parameters = test::ResolveLuaParameters(kPoseGraphLua);
    pose_graph_options_ = CreatePoseGraphOptions(pose_graph_parameters.get());
  }

  void BuildPoseGraph() {
    auto optimization_problem =
        common::make_unique<optimization::OptimizationProblem3D>(
            pose_graph_options_.optimization_problem_options());
    pose_graph_ = common::make_unique<PoseGraph3DForTesting>(
        pose_graph_options_, std::move(optimization_problem),
        thread_pool_.get());
  }

  void BuildPoseGraphWithFakeOptimization() {
    auto optimization_problem =
        common::make_unique<MockOptimizationProblem3D>();
    pose_graph_ = common::make_unique<PoseGraph3DForTesting>(
        pose_graph_options_, std::move(optimization_problem),
        thread_pool_.get());
  }

  proto::PoseGraphOptions pose_graph_options_;
  std::unique_ptr<common::ThreadPool> thread_pool_;
  std::unique_ptr<PoseGraph3DForTesting> pose_graph_;
};

TEST_F(PoseGraph3DTest, Empty) {
  BuildPoseGraph();
  pose_graph_->WaitForAllComputations();
  EXPECT_TRUE(pose_graph_->GetAllSubmapData().empty());
  EXPECT_TRUE(pose_graph_->constraints().empty());
  EXPECT_TRUE(pose_graph_->GetConnectedTrajectories().empty());
  EXPECT_TRUE(pose_graph_->GetAllSubmapPoses().empty());
  EXPECT_TRUE(pose_graph_->GetTrajectoryNodes().empty());
  EXPECT_TRUE(pose_graph_->GetTrajectoryNodePoses().empty());
  EXPECT_TRUE(pose_graph_->GetTrajectoryData().empty());
  proto::PoseGraph empty_proto;
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      pose_graph_->ToProto(), empty_proto));
}

TEST_F(PoseGraph3DTest, BasicSerialization) {
  BuildPoseGraph();
  proto::PoseGraph proto;
  auto fake_node = test::CreateFakeNode();
  test::AddToProtoGraph(fake_node, &proto);
  pose_graph_->AddNodeFromProto(Rigid3d::Identity(), fake_node);
  auto fake_submap = test::CreateFakeSubmap3D();
  test::AddToProtoGraph(fake_submap, &proto);
  pose_graph_->AddSubmapFromProto(Rigid3d::Identity(), fake_submap);
  test::AddToProtoGraph(test::CreateFakeConstraint(fake_node, fake_submap),
                        &proto);
  pose_graph_->AddSerializedConstraints(FromProto(proto.constraint()));
  test::AddToProtoGraph(
      test::CreateFakeLandmark("landmark_id", Rigid3d::Identity()), &proto);
  pose_graph_->SetLandmarkPose("landmark_id", Rigid3d::Identity());
  pose_graph_->WaitForAllComputations();
  proto::PoseGraph actual_proto = pose_graph_->ToProto();
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.constraint(0), actual_proto.constraint(0)));
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.trajectory(0).node(0), actual_proto.trajectory(0).node(0)));
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.trajectory(0).submap(0), actual_proto.trajectory(0).submap(0)));
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.trajectory(0), actual_proto.trajectory(0)));
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      proto.landmark_poses(0), actual_proto.landmark_poses(0)));
  EXPECT_TRUE(
      google::protobuf::util::MessageDifferencer::Equals(proto, actual_proto));
}

TEST_F(PoseGraph3DTest, PureLocalizationTrimmer) {
  BuildPoseGraphWithFakeOptimization();
  const int trajectory_id = 2;
  const int num_submaps_to_create = 5;
  const int num_submaps_to_keep = 3;
  const int num_nodes_per_submap = 2;
  for (int i = 0; i < num_submaps_to_create; ++i) {
    int submap_index = (i < 3) ? 42 + i : 100 + i;
    auto submap = test::CreateFakeSubmap3D(trajectory_id, submap_index);
    pose_graph_->AddSubmapFromProto(Rigid3d::Identity(), submap);
    for (int j = 0; j < num_nodes_per_submap; ++j) {
      int node_index = 7 + num_nodes_per_submap * submap_index + j;
      auto node = test::CreateFakeNode(trajectory_id, node_index);
      pose_graph_->AddNodeFromProto(Rigid3d::Identity(), node);
      proto::PoseGraph proto;
      auto constraint = test::CreateFakeConstraint(node, submap);
      // TODO(gaschler): Also remove inter constraints when all references are
      // gone.
      constraint.set_tag(proto::PoseGraph::Constraint::INTRA_SUBMAP);
      test::AddToProtoGraph(constraint, &proto);
      pose_graph_->AddSerializedConstraints(FromProto(proto.constraint()));
    }
  }
  pose_graph_->AddTrimmer(common::make_unique<PureLocalizationTrimmer>(
      trajectory_id, num_submaps_to_keep));
  pose_graph_->WaitForAllComputations();
  EXPECT_EQ(
      pose_graph_->GetAllSubmapPoses().SizeOfTrajectoryOrZero(trajectory_id),
      num_submaps_to_create);
  EXPECT_EQ(
      pose_graph_->GetAllSubmapData().SizeOfTrajectoryOrZero(trajectory_id),
      num_submaps_to_create);
  EXPECT_EQ(
      pose_graph_->GetTrajectoryNodes().SizeOfTrajectoryOrZero(trajectory_id),
      num_nodes_per_submap * num_submaps_to_create);
  EXPECT_EQ(pose_graph_->GetTrajectoryNodePoses().SizeOfTrajectoryOrZero(
                trajectory_id),
            num_nodes_per_submap * num_submaps_to_create);
  EXPECT_EQ(pose_graph_->constraints().size(),
            num_nodes_per_submap * num_submaps_to_create);
  for (int i = 0; i < 2; ++i) {
    pose_graph_->RunFinalOptimization();
    EXPECT_EQ(
        pose_graph_->GetAllSubmapPoses().SizeOfTrajectoryOrZero(trajectory_id),
        num_submaps_to_keep);
    EXPECT_EQ(
        pose_graph_->GetAllSubmapData().SizeOfTrajectoryOrZero(trajectory_id),
        num_submaps_to_keep);
    EXPECT_EQ(
        pose_graph_->GetTrajectoryNodes().SizeOfTrajectoryOrZero(trajectory_id),
        num_nodes_per_submap * num_submaps_to_keep);
    EXPECT_EQ(pose_graph_->GetTrajectoryNodePoses().SizeOfTrajectoryOrZero(
                  trajectory_id),
              num_nodes_per_submap * num_submaps_to_keep);
    EXPECT_EQ(pose_graph_->constraints().size(),
              num_nodes_per_submap * num_submaps_to_keep);
  }
}

class EvenSubmapTrimmer : public PoseGraphTrimmer {
 public:
  explicit EvenSubmapTrimmer(int trajectory_id)
      : trajectory_id_(trajectory_id) {}

  void Trim(Trimmable *pose_graph) override {
    auto submap_ids = pose_graph->GetSubmapIds(trajectory_id_);
    for (const auto submap_id : submap_ids) {
      if (submap_id.submap_index % 2 == 0) {
        pose_graph->MarkSubmapAsTrimmed(submap_id);
      }
    }
  }

  bool IsFinished() override { return false; }

 private:
  int trajectory_id_;
};

TEST_F(PoseGraph3DTest, EvenSubmapTrimmer) {
  BuildPoseGraphWithFakeOptimization();
  const int trajectory_id = 2;
  const int num_submaps_to_keep = 10;
  const int num_submaps_to_create = 2 * num_submaps_to_keep;
  const int num_nodes_per_submap = 3;
  for (int i = 0; i < num_submaps_to_create; ++i) {
    int submap_index = 42 + i;
    auto submap = test::CreateFakeSubmap3D(trajectory_id, submap_index);
    pose_graph_->AddSubmapFromProto(Rigid3d::Identity(), submap);
    for (int j = 0; j < num_nodes_per_submap; ++j) {
      int node_index = 7 + num_nodes_per_submap * i + j;
      auto node = test::CreateFakeNode(trajectory_id, node_index);
      pose_graph_->AddNodeFromProto(Rigid3d::Identity(), node);
      proto::PoseGraph proto;
      auto constraint = test::CreateFakeConstraint(node, submap);
      constraint.set_tag(proto::PoseGraph::Constraint::INTRA_SUBMAP);
      test::AddToProtoGraph(constraint, &proto);
      pose_graph_->AddSerializedConstraints(FromProto(proto.constraint()));
    }
  }
  pose_graph_->AddTrimmer(
      common::make_unique<EvenSubmapTrimmer>(trajectory_id));
  pose_graph_->WaitForAllComputations();
  EXPECT_EQ(
      pose_graph_->GetAllSubmapPoses().SizeOfTrajectoryOrZero(trajectory_id),
      num_submaps_to_create);
  EXPECT_EQ(
      pose_graph_->GetAllSubmapData().SizeOfTrajectoryOrZero(trajectory_id),
      num_submaps_to_create);
  EXPECT_EQ(
      pose_graph_->GetTrajectoryNodes().SizeOfTrajectoryOrZero(trajectory_id),
      num_nodes_per_submap * num_submaps_to_create);
  EXPECT_EQ(pose_graph_->GetTrajectoryNodePoses().SizeOfTrajectoryOrZero(
                trajectory_id),
            num_nodes_per_submap * num_submaps_to_create);
  EXPECT_EQ(pose_graph_->constraints().size(),
            num_nodes_per_submap * num_submaps_to_create);
  for (int i = 0; i < 2; ++i) {
    pose_graph_->RunFinalOptimization();
    EXPECT_EQ(
        pose_graph_->GetAllSubmapPoses().SizeOfTrajectoryOrZero(trajectory_id),
        num_submaps_to_keep);
    EXPECT_EQ(
        pose_graph_->GetAllSubmapData().SizeOfTrajectoryOrZero(trajectory_id),
        num_submaps_to_keep);
    EXPECT_EQ(
        pose_graph_->GetTrajectoryNodes().SizeOfTrajectoryOrZero(trajectory_id),
        num_nodes_per_submap * num_submaps_to_keep);
    EXPECT_EQ(pose_graph_->GetTrajectoryNodePoses().SizeOfTrajectoryOrZero(
                  trajectory_id),
              num_nodes_per_submap * num_submaps_to_keep);
    EXPECT_EQ(pose_graph_->constraints().size(),
              num_nodes_per_submap * num_submaps_to_keep);
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
