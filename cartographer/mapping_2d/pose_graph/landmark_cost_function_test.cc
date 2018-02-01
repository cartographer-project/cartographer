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
    mapping::PoseGraphInterface::LandmarkNode::LandmarkObservation;

TEST(LandmarkCostFunctionTest, SmokeTest) {
  NodeData prev_node;
  prev_node.time = common::FromUniversal(0);
  prev_node.gravity_alignment = Eigen::Quaterniond::Identity();
  NodeData next_node;
  next_node.time = common::FromUniversal(10);
  next_node.gravity_alignment = Eigen::Quaterniond::Identity();

  auto* cost_function = LandmarkCostFunction::CreateAutoDiffCostFunction(
      LandmarkObservation{
          0 /* trajectory ID */,
          common::FromUniversal(5) /* time */,
          transform::Rigid3d::Translation(Eigen::Vector3d(1., 1., 1.)),
          1. /* translation_weight */,
          2. /* rotation_weight */,
      },
      prev_node, next_node);

  std::array<double, 3> prev_node_pose{{2., 0., 0.}};
  std::array<double, 3> next_node_pose{{0., 2., 0.}};
  std::array<double, 4> landmark_rotation{{1., 0., 0., 0.}};
  std::array<double, 3> landmark_translation{{1., 2., 1.}};
  const double* parameter_blocks[] = {
      prev_node_pose.data(), next_node_pose.data(), landmark_rotation.data(),
      landmark_translation.data()};

  std::array<double, 6> residuals;
  cost_function->Evaluate(parameter_blocks, residuals.data(), nullptr);

  EXPECT_THAT(residuals, ElementsAre(DoubleEq(1.), DoubleEq(0.), DoubleEq(0.),
                                     DoubleEq(0.), DoubleEq(0.), DoubleEq(0.)));
}

}  // namespace
}  // namespace pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
