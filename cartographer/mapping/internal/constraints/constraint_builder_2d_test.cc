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

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <functional>

#include "cartographer/common/internal/testing/thread_pool_for_testing.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace constraints {
namespace {

class MockCallback {
 public:
  MOCK_METHOD1(Run, void(const ConstraintBuilder2D::Result&));
};

class ConstraintBuilder2DTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto constraint_builder_parameters = testing::ResolveLuaParameters(R"text(
            include "pose_graph.lua"
            POSE_GRAPH.constraint_builder.sampling_ratio = 1
            POSE_GRAPH.constraint_builder.min_score = 0
            POSE_GRAPH.constraint_builder.global_localization_min_score = 0
            return POSE_GRAPH.constraint_builder)text");
    constraint_builder_ = absl::make_unique<ConstraintBuilder2D>(
        CreateConstraintBuilderOptions(constraint_builder_parameters.get()),
        &thread_pool_);
  }

  std::unique_ptr<ConstraintBuilder2D> constraint_builder_;
  MockCallback mock_;
  common::testing::ThreadPoolForTesting thread_pool_;
};

TEST_F(ConstraintBuilder2DTest, CallsBack) {
  EXPECT_EQ(constraint_builder_->GetNumFinishedNodes(), 0);
  EXPECT_CALL(mock_, Run(::testing::IsEmpty()));
  constraint_builder_->NotifyEndOfNode();
  constraint_builder_->WhenDone(
      [this](const constraints::ConstraintBuilder2D::Result& result) {
        mock_.Run(result);
      });
  thread_pool_.WaitUntilIdle();
  EXPECT_EQ(constraint_builder_->GetNumFinishedNodes(), 1);
}

TEST_F(ConstraintBuilder2DTest, FindsConstraints) {
  TrajectoryNode::Data node_data;
  node_data.filtered_gravity_aligned_point_cloud.push_back(
      {Eigen::Vector3f(0.1, 0.2, 0.3)});
  node_data.gravity_alignment = Eigen::Quaterniond::Identity();
  node_data.local_pose = transform::Rigid3d::Identity();
  SubmapId submap_id{0, 1};
  MapLimits map_limits(1., Eigen::Vector2d(2., 3.), CellLimits(100, 110));
  ValueConversionTables conversion_tables;
  Submap2D submap(
      Eigen::Vector2f(4.f, 5.f),
      absl::make_unique<ProbabilityGrid>(map_limits, &conversion_tables),
      &conversion_tables);
  int expected_nodes = 0;
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(constraint_builder_->GetNumFinishedNodes(), expected_nodes);
    for (int j = 0; j < 2; ++j) {
      constraint_builder_->MaybeAddConstraint(submap_id, &submap, NodeId{0, 0},
                                              &node_data,
                                              transform::Rigid2d::Identity());
    }
    constraint_builder_->MaybeAddGlobalConstraint(submap_id, &submap,
                                                  NodeId{0, 0}, &node_data);
    constraint_builder_->NotifyEndOfNode();
    thread_pool_.WaitUntilIdle();
    EXPECT_EQ(constraint_builder_->GetNumFinishedNodes(), ++expected_nodes);
    constraint_builder_->NotifyEndOfNode();
    thread_pool_.WaitUntilIdle();
    EXPECT_EQ(constraint_builder_->GetNumFinishedNodes(), ++expected_nodes);
    EXPECT_CALL(mock_,
                Run(::testing::AllOf(
                    ::testing::SizeIs(3),
                    ::testing::Each(::testing::Field(
                        &PoseGraphInterface::Constraint::tag,
                        PoseGraphInterface::Constraint::INTER_SUBMAP)))));
    constraint_builder_->WhenDone(
        [this](const constraints::ConstraintBuilder2D::Result& result) {
          mock_.Run(result);
        });
    thread_pool_.WaitUntilIdle();
    constraint_builder_->DeleteScanMatcher(submap_id);
  }
}

}  // namespace
}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
