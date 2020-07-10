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

#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"

#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/probability_values.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

using ::testing::DoubleEq;
using ::testing::ElementsAre;

TEST(OccupiedSpaceCostFunction2DTest, SmokeTest) {
  ValueConversionTables conversion_tables;
  ProbabilityGrid grid(MapLimits(1., Eigen::Vector2d(1., 1.), CellLimits(2, 2)),
                       &conversion_tables);
  sensor::PointCloud point_cloud = {{Eigen::Vector3f{0.f, 0.f, 0.f}}};
  ceres::Problem problem;
  std::unique_ptr<ceres::CostFunction> cost_function(
      CreateOccupiedSpaceCostFunction2D(1.f, point_cloud, grid));

  const std::array<double, 3> pose_estimate{{0., 0., 0.}};
  const std::array<const double*, 1> parameter_blocks{{pose_estimate.data()}};

  std::array<double, 1> residuals;
  std::array<std::array<double, 3>, 1> jacobians;
  std::array<double*, 1> jacobians_ptrs;
  for (int i = 0; i < 1; ++i) jacobians_ptrs[i] = jacobians[i].data();
  cost_function->Evaluate(parameter_blocks.data(), residuals.data(),
                          jacobians_ptrs.data());

  EXPECT_THAT(residuals, ElementsAre(DoubleEq(kMaxProbability)));
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer