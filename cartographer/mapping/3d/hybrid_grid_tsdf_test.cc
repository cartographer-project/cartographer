/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"

#include <map>
#include <random>
#include <tuple>

#include "gmock/gmock.h"

#include "cartographer/mapping/value_conversion_tables.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(HybridGridTSDFTest, ApplyOdds) {
  ValueConversionTables conversion_tables;
  HybridGridTSDF hybrid_grid(1.f, 0.5f, 1.0f, &conversion_tables);

  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 0, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 1, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 0, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 1, 0)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 0, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(0, 1, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 0, 1)));
  EXPECT_FALSE(hybrid_grid.IsKnown(Eigen::Array3i(1, 1, 1)));

  hybrid_grid.SetCell(Eigen::Array3i(1, 0, 1), 0.1f, 0.5f);
  hybrid_grid.FinishUpdate();

  EXPECT_TRUE(hybrid_grid.IsKnown(Eigen::Array3i(1, 0, 1)));
  EXPECT_NEAR(hybrid_grid.GetTSD(Eigen::Array3i(1, 0, 1)), 0.1f, 1e-4);
  EXPECT_NEAR(hybrid_grid.GetWeight(Eigen::Array3i(1, 0, 1)), 0.5f, 1e-4);

  EXPECT_NEAR(hybrid_grid.GetTSD(Eigen::Array3i(0, 0, 1)), -0.5f, 1e-4);
  EXPECT_NEAR(hybrid_grid.GetWeight(Eigen::Array3i(0, 0, 1)), 0.0f, 1e-4);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
