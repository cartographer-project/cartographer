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

#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_3d.h"

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

TEST(LandmarkCostFunction3DTest, SmokeTest) {
  NodeSpec3D prev_node;
  prev_node.time = common::FromUniversal(0);
  NodeSpec3D next_node;
  next_node.time = common::FromUniversal(10);

  std::unique_ptr<ceres::CostFunction> cost_function(
      LandmarkCostFunction3D::CreateAutoDiffCostFunction(
          LandmarkObservation{
              0 /* trajectory ID */,
              common::FromUniversal(5) /* time */,
              transform::Rigid3d::Translation(Eigen::Vector3d(1., 1., 1.)),
              1. /* translation_weight */,
              2. /* rotation_weight */,
          },
          prev_node, next_node));

  const std::array<double, 4> prev_node_rotation{{1., 0., 0., 0.}};
  const std::array<double, 3> prev_node_translation{{0., 0., 0.}};
  const std::array<double, 4> next_node_rotation{{1., 0., 0., 0.}};
  const std::array<double, 3> next_node_translation{{2., 2., 2.}};
  const std::array<double, 4> landmark_rotation{{1., 0., 0., 0.}};
  const std::array<double, 3> landmark_translation{{1., 2., 2.}};
  const std::array<const double*, 6> parameter_blocks{
      {prev_node_rotation.data(), prev_node_translation.data(),
       next_node_rotation.data(), next_node_translation.data(),
       landmark_rotation.data(), landmark_translation.data()}};

  std::array<double, 6> residuals;
  std::array<std::array<double, 21>, 6> jacobians;
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
