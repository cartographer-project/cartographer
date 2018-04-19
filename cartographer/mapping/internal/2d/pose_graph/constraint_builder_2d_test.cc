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

#include "cartographer/mapping/internal/2d/pose_graph/constraint_builder_2d.h"

#include <functional>

#include "cartographer/common/config.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/pose_graph/constraint_builder.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/pose_graph.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {
namespace {

pose_graph::proto::ConstraintBuilderOptions GenerateConstraintBuilderOptions() {
  const std::string kConstraintBuilderLua = R"text(
    include "pose_graph.lua"
    POSE_GRAPH.constraint_builder.sampling_ratio = 1
    POSE_GRAPH.constraint_builder.min_score = 0
    POSE_GRAPH.constraint_builder.global_localization_min_score = 0
    return POSE_GRAPH.constraint_builder)text";
  auto constraint_builder_parameters =
      test::ResolveLuaParameters(kConstraintBuilderLua);
  return CreateConstraintBuilderOptions(constraint_builder_parameters.get());
}

class Mock {
 public:
  MOCK_METHOD1(Run, void(const ConstraintBuilder2D::Result&));
};

TEST(ConstraintBuilder2DTest, Create) {
  common::ThreadPool thread_pool(1);
  ConstraintBuilder2D constraint_builder(GenerateConstraintBuilderOptions(),
                                         &thread_pool);
}

TEST(ConstraintBuilder2DTest, Callback) {
  common::ThreadPool thread_pool(1);
  ConstraintBuilder2D constraint_builder(GenerateConstraintBuilderOptions(),
                                         &thread_pool);
  Mock mock;
  constraint_builder.NotifyEndOfNode();
  constraint_builder.WhenDone(
      std::bind(&Mock::Run, &mock, std::placeholders::_1));
  EXPECT_CALL(mock, Run(testing::_));
  // TODO: use ThreadPool for testing
  sleep(1);
}

TEST(ConstraintBuilder2DTest, FindsConstraints) {
  common::ThreadPool thread_pool(1);
  ConstraintBuilder2D constraint_builder(GenerateConstraintBuilderOptions(),
                                         &thread_pool);
  Mock mock;
  TrajectoryNode::Data node_data;
  node_data.filtered_gravity_aligned_point_cloud.push_back(
      Eigen::Vector3f(0.1, 0.2, 0.3));
  node_data.gravity_alignment = Eigen::Quaterniond::Identity();
  node_data.local_pose = transform::Rigid3d::Identity();
  SubmapId submap_id{0, 1};
  Submap2D submap(MapLimits(1., Eigen::Vector2d(2., 3.), CellLimits(100, 110)),
                  Eigen::Vector2f(4.f, 5.f));
  EXPECT_EQ(constraint_builder.GetNumFinishedNodes(), 0);
  constraint_builder.MaybeAddConstraint(
      submap_id, &submap, NodeId{}, &node_data, transform::Rigid2d::Identity());
  constraint_builder.MaybeAddConstraint(
      submap_id, &submap, NodeId{}, &node_data, transform::Rigid2d::Identity());
  constraint_builder.MaybeAddGlobalConstraint(submap_id, &submap, NodeId{},
                                              &node_data);
  constraint_builder.NotifyEndOfNode();
  sleep(1);
  EXPECT_EQ(constraint_builder.GetNumFinishedNodes(), 1);
  constraint_builder.NotifyEndOfNode();
  EXPECT_CALL(mock, Run(testing::AllOf(
                        testing::SizeIs(3),
                        testing::Each(testing::Field(
                            &PoseGraphInterface::Constraint::tag,
                            PoseGraphInterface::Constraint::INTER_SUBMAP)))));
  constraint_builder.WhenDone(
      std::bind(&Mock::Run, &mock, std::placeholders::_1));
  // TODO: use ThreadPool for testing
  sleep(1);
  EXPECT_EQ(constraint_builder.GetNumFinishedNodes(), 2);
  constraint_builder.DeleteScanMatcher(submap_id);
}

}  // namespace
}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer
