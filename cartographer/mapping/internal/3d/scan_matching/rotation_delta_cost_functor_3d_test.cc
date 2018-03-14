/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/internal/3d/scan_matching/rotation_delta_cost_functor_3d.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

constexpr double kPrecision = 1e-8;

double ComputeRotationDeltaSquaredCost(
    const Eigen::Quaterniond& rotation, const double scaling_factor,
    const Eigen::Quaterniond& target_rotation) {
  std::unique_ptr<ceres::CostFunction> cost_function(
      RotationDeltaCostFunctor3D::CreateAutoDiffCostFunction(scaling_factor,
                                                             target_rotation));
  const std::array<double, 4> parameter_quaternion = {
      {rotation.w(), rotation.x(), rotation.y(), rotation.z()}};
  const std::vector<const double*> parameters = {parameter_quaternion.data()};
  std::vector<double> residuals(cost_function->num_residuals());
  EXPECT_TRUE(cost_function->Evaluate(parameters.data(), residuals.data(),
                                      nullptr /* jacobian */));
  double sum_of_squares = 0;
  for (double residual : residuals) {
    sum_of_squares += residual * residual;
  }
  return sum_of_squares;
}

TEST(RotationDeltaCostFunctor3DTest, SameRotationGivesZeroCost) {
  EXPECT_NEAR(
      0.,
      ComputeRotationDeltaSquaredCost(Eigen::Quaterniond::Identity(), 1.0,
                                      Eigen::Quaterniond::Identity()),
      kPrecision);

  Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(0.9, Eigen::Vector3d(0.2, 0.1, 0.3).normalized()));
  EXPECT_NEAR(0., ComputeRotationDeltaSquaredCost(rotation, 1.0, rotation),
              kPrecision);
}

TEST(RotationDeltaCostFunctor3DTest, ComputesCorrectCost) {
  double scaling_factor = 1.2;
  double angle = 0.8;
  Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(angle, Eigen::Vector3d(0.2, 0.1, 0.8).normalized()));
  Eigen::Quaterniond target_rotation(
      Eigen::AngleAxisd(0.2, Eigen::Vector3d(-0.5, 0.3, 0.4).normalized()));
  double expected_cost = std::pow(scaling_factor * std::sin(angle / 2.0), 2);
  EXPECT_NEAR(expected_cost,
              ComputeRotationDeltaSquaredCost(rotation, scaling_factor,
                                              Eigen::Quaterniond::Identity()),
              kPrecision);
  EXPECT_NEAR(expected_cost,
              ComputeRotationDeltaSquaredCost(target_rotation * rotation,
                                              scaling_factor, target_rotation),
              kPrecision);
  EXPECT_NEAR(expected_cost,
              ComputeRotationDeltaSquaredCost(rotation * target_rotation,
                                              scaling_factor, target_rotation),
              kPrecision);
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
