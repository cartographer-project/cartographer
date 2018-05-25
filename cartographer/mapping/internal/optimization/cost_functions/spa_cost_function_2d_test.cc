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

#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.h"

#include <memory>

#include "cartographer/transform/rigid_transform.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using ::testing::ElementsAre;

constexpr int kPoseDimension = 3;
constexpr int kResidualsCount = 3;
constexpr int kParameterBlocksCount = 2;
constexpr int kJacobianColDimension = kResidualsCount * kPoseDimension;

using ResidualType = std::array<double, kResidualsCount>;
using JacobianType = std::array<std::array<double, kJacobianColDimension>,
                                kParameterBlocksCount>;

::testing::Matcher<double> Near(double expected, double precision = 1e-05) {
  return testing::DoubleNear(expected, precision);
}

class SpaCostFunction2DTest : public ::testing::Test {
 public:
  SpaCostFunction2DTest()
      : constraint_(PoseGraphInterface::Constraint::Pose{
            transform::Rigid3d(Eigen::Vector3d(1., 1., 1.),
                               Eigen::Quaterniond(1., 1., -1., -1.)),
            1, 10}),
        auto_diff_cost_(CreateAutoDiffSpaCostFunction(constraint_)),
        analytical_cost_(CreateAnalyticalSpaCostFunction(constraint_)) {
    for (int i = 0; i < kParameterBlocksCount; ++i) {
      jacobian_ptrs_[i] = jacobian_[i].data();
    }
  }

  std::pair<const ResidualType&, const JacobianType&> EvaluateAnalyticalCost(
      const std::array<const double*, 2>& parameter_blocks) {
    return Evaluate(parameter_blocks, analytical_cost_);
  }

  std::pair<const ResidualType&, const JacobianType&> EvaluateAutoDiffCost(
      const std::array<const double*, 2>& parameter_blocks) {
    return Evaluate(parameter_blocks, auto_diff_cost_);
  }

 private:
  std::pair<const ResidualType&, const JacobianType&> Evaluate(
      const std::array<const double*, 2>& parameter_blocks,
      const std::unique_ptr<ceres::CostFunction>& cost_function) {
    cost_function->Evaluate(parameter_blocks.data(), residuals_.data(),
                            jacobian_ptrs_.data());
    return std::make_pair(std::cref(residuals_), std::cref(jacobian_));
  }

  ResidualType residuals_;
  JacobianType jacobian_;
  std::array<double*, kParameterBlocksCount> jacobian_ptrs_;
  PoseGraphInterface::Constraint::Pose constraint_;
  std::unique_ptr<ceres::CostFunction> auto_diff_cost_;
  std::unique_ptr<ceres::CostFunction> analytical_cost_;
};

TEST_F(SpaCostFunction2DTest, CompareAutoDiffAndAnalytical) {
  std::array<double, 3> start_pose{{1., 1., 1.}};
  std::array<double, 3> end_pose{{10., 1., 100.}};
  std::array<const double*, 2> parameter_blocks{
      {start_pose.data(), end_pose.data()}};

  ResidualType auto_diff_residual, analytical_residual;
  JacobianType auto_diff_jacobian, analytical_jacobian;
  std::tie(auto_diff_residual, auto_diff_jacobian) =
      EvaluateAutoDiffCost(parameter_blocks);
  std::tie(analytical_residual, analytical_jacobian) =
      EvaluateAnalyticalCost(parameter_blocks);

  for (int i = 0; i < kResidualsCount; ++i) {
    EXPECT_THAT(auto_diff_residual[i], Near(analytical_residual[i]));
  }
  for (int i = 0; i < kParameterBlocksCount; ++i) {
    for (int j = 0; j < kJacobianColDimension; ++j) {
      EXPECT_THAT(auto_diff_jacobian[i][j], Near(analytical_jacobian[i][j]));
    }
  }
}

TEST_F(SpaCostFunction2DTest, EvaluateAnalyticalCost) {
  std::array<double, 3> start_pose{{1., 1., 1.}};
  std::array<double, 3> end_pose{{10., 1., 100.}};
  std::array<const double*, 2> parameter_blocks{
      {start_pose.data(), end_pose.data()}};

  auto residuals_and_jacobian = EvaluateAnalyticalCost(parameter_blocks);
  EXPECT_THAT(residuals_and_jacobian.first,
              ElementsAre(Near(-3.86272), Near(8.57324), Near(-6.83333)));
  EXPECT_THAT(
      residuals_and_jacobian.second,
      ElementsAre(
          ElementsAre(Near(0.540302), Near(0.841471), Near(7.57324),
                      Near(-0.841471), Near(0.540302), Near(4.86272), Near(0),
                      Near(0), Near(10)),
          ElementsAre(Near(-0.540302), Near(-0.841471), Near(0), Near(0.841471),
                      Near(-0.540302), Near(0), Near(0), Near(0), Near(-10))));
}

TEST_F(SpaCostFunction2DTest, EvaluateAutoDiffCost) {
  std::array<double, 3> start_pose{{1., 1., 1.}};
  std::array<double, 3> end_pose{{10., 1., 100.}};
  std::array<const double*, 2> parameter_blocks{
      {start_pose.data(), end_pose.data()}};

  auto residuals_and_jacobian = EvaluateAutoDiffCost(parameter_blocks);
  EXPECT_THAT(residuals_and_jacobian.first,
              ElementsAre(Near(-3.86272), Near(8.57324), Near(-6.83333)));
  EXPECT_THAT(
      residuals_and_jacobian.second,
      ElementsAre(
          ElementsAre(Near(0.540302), Near(0.841471), Near(7.57324),
                      Near(-0.841471), Near(0.540302), Near(4.86272), Near(0),
                      Near(0), Near(10)),
          ElementsAre(Near(-0.540302), Near(-0.841471), Near(0), Near(0.841471),
                      Near(-0.540302), Near(0), Near(0), Near(0), Near(-10))));
}

}  // namespace
}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
