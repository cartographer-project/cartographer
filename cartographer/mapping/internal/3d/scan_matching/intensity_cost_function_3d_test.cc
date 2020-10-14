/*
 * Copyright 2019 The Cartographer Authors
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

#include "cartographer/mapping/internal/3d/scan_matching/intensity_cost_function_3d.h"

#include <array>
#include <memory>

#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "ceres/ceres.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

using ::testing::DoubleNear;
using ::testing::ElementsAre;

TEST(IntensityCostFunction3DTest, SmokeTest) {
  const sensor::PointCloud point_cloud(
      {{{0.f, 0.f, 0.f}}, {{1.f, 1.f, 1.f}}, {{2.f, 2.f, 2.f}}},
      {50.f, 100.f, 150.f});
  IntensityHybridGrid hybrid_grid(0.3f);
  hybrid_grid.AddIntensity(hybrid_grid.GetCellIndex({0.f, 0.f, 0.f}), 50.f);

  std::unique_ptr<ceres::CostFunction> cost_function(
      IntensityCostFunction3D::CreateAutoDiffCostFunction(
          /*scaling_factor=*/1.0f, /*intensity_threshold=*/100.f, point_cloud,
          hybrid_grid));

  const std::array<double, 3> translation{{0., 0., 0.}};
  const std::array<double, 4> rotation{{1., 0., 0., 0.}};

  const std::array<const double*, 2> parameter_blocks{
      {translation.data(), rotation.data()}};
  std::array<double, 3> residuals;

  cost_function->Evaluate(parameter_blocks.data(), residuals.data(),
                          /*jacobians=*/nullptr);
  EXPECT_THAT(residuals,
              ElementsAre(DoubleNear(0., 1e-9), DoubleNear(-100., 1e-9),
                          DoubleNear(0., 1e-9)));
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
