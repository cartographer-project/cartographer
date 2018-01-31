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

#include "cartographer/mapping/pose_graph/landmark_cost_function.h"
#include "cartographer/transform/rigid_transform.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {
namespace {

using ::testing::DoubleEq;
using ::testing::ElementsAre;

using LandmarkObservation =
    mapping::PoseGraphInterface::LandmarkNode::LandmarkObservation;

TEST(LandmarkCostFunctionTest, SmokeTest) {
  auto* cost_function = LandmarkCostFunction::CreateAutoDiffCostFunction(
      LandmarkObservation{
          0 /* trajectory ID */,
          common::FromUniversal(5) /* time */,
          transform::Rigid3d::Translation(Eigen::Vector3d(1., 1., 1.)),
          1. /* translation_weight */,
          2. /* rotation_weight */,
      },
      common::FromUniversal(0), common::FromUniversal(10));

  std::array<double, 4> prev_node_rotation{{1., 0., 0., 0.}};
  std::array<double, 3> prev_node_translation{{0., 0., 0.}};
  std::array<double, 4> next_node_rotation{{1., 0., 0., 0.}};
  std::array<double, 3> next_node_translation{{2., 2., 2.}};
  std::array<double, 4> landmark_rotation{{1., 0., 0., 0.}};
  std::array<double, 3> landmark_translation{{1., 2., 2.}};
  const double* parameter_blocks[] = {
      prev_node_rotation.data(), prev_node_translation.data(),
      next_node_rotation.data(), next_node_translation.data(),
      landmark_rotation.data(),  landmark_translation.data()};

  std::array<double, 6> residuals;
  cost_function->Evaluate(parameter_blocks, residuals.data(), nullptr);

  EXPECT_THAT(residuals, ElementsAre(DoubleEq(1.), DoubleEq(0.), DoubleEq(0.),
                                     DoubleEq(0.), DoubleEq(0.), DoubleEq(0.)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer
