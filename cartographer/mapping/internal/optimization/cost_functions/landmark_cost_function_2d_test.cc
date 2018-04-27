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

#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_2d.h"

#include <memory>

#include "cartographer/transform/rigid_transform.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using ::testing::DoubleEq;
using ::testing::ElementsAre;

using LandmarkObservation =
    PoseGraphInterface::LandmarkNode::LandmarkObservation;

TEST(LandmarkCostFunctionTest, SmokeTest) {
  NodeSpec2D prev_node;
  prev_node.time = common::FromUniversal(0);
  prev_node.gravity_alignment = Eigen::Quaterniond::Identity();
  NodeSpec2D next_node;
  next_node.time = common::FromUniversal(10);
  next_node.gravity_alignment = Eigen::Quaterniond::Identity();

  std::unique_ptr<ceres::CostFunction> cost_function(
      LandmarkCostFunction2D::CreateAutoDiffCostFunction(
          LandmarkObservation{
              0 /* trajectory ID */,
              common::FromUniversal(5) /* time */,
              transform::Rigid3d::Translation(Eigen::Vector3d(1., 1., 1.)),
              1. /* translation_weight */,
              2. /* rotation_weight */,
          },
          prev_node, next_node));

  const std::array<double, 3> prev_node_pose{{2., 0., 0.}};
  const std::array<double, 3> next_node_pose{{0., 2., 0.}};
  const std::array<double, 4> landmark_rotation{{1., 0., 0., 0.}};
  const std::array<double, 3> landmark_translation{{1., 2., 1.}};
  const std::array<const double*, 4> parameter_blocks{
      {prev_node_pose.data(), next_node_pose.data(), landmark_rotation.data(),
       landmark_translation.data()}};

  std::array<double, 6> residuals;
  std::array<std::array<double, 13>, 6> jacobians;
  std::array<double*, 6> jacobians_ptrs;
  for (int i = 0; i < 6; ++i) jacobians_ptrs[i] = jacobians[i].data();

  cost_function->Evaluate(parameter_blocks.data(), residuals.data(),
                          jacobians_ptrs.data());
  EXPECT_THAT(residuals, ElementsAre(DoubleEq(1.), DoubleEq(0.), DoubleEq(0.),
                                     DoubleEq(0.), DoubleEq(0.), DoubleEq(0.)));
}

}  // namespace
}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
