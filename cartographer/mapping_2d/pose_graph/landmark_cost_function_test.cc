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

#include "cartographer/mapping_2d/pose_graph/landmark_cost_function.h"
#include "cartographer/transform/rigid_transform.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_2d {
namespace pose_graph {
namespace {

using ::testing::DoubleEq;
using ::testing::ElementsAre;

using LandmarkObservation =
    mapping::PoseGraph::LandmarkNode::LandmarkObservation;

TEST(LandmarkCostFunctionTest, SmokeTest) {
  auto* cost_function = LandmarkCostFunction::CreateAutoDiffCostFunction(
      LandmarkObservation{
          0 /* trajectory ID */,
          common::FromUniversal(5) /* time */,
          transform::Embed3D(transform::Rigid2d({1., 1.}, 0.)),
          1. /* translation_weight */,
          2. /* rotation_weight */,
      },
      common::FromUniversal(0), common::FromUniversal(10));

  std::array<double, 3> prev_node_pose{{2., 0., 0.}};
  std::array<double, 3> next_node_pose{{0., 2., 0.}};
  std::array<double, 3> landmark_pose{{2., 2., 1.5}};
  const double* parameter_blocks[] = {
      prev_node_pose.data(), next_node_pose.data(), landmark_pose.data()};

  std::array<double, 3> residuals;
  cost_function->Evaluate(parameter_blocks, residuals.data(), nullptr);

  EXPECT_THAT(residuals,
              ElementsAre(DoubleEq(0.), DoubleEq(0.), DoubleEq(-3.)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
