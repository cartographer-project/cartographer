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

#include "cartographer/mapping_2d/probability_grid.h"

#include <random>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_2d {
namespace {

TEST(ProbabilityGridTest, ApplyOdds) {
  ProbabilityGrid probability_grid(MapLimits(1., 0.5, 0.5, CellLimits(2, 2)));
  const MapLimits& limits = probability_grid.limits();

  EXPECT_TRUE(limits.Contains(Eigen::Array2i(0, 0)));
  EXPECT_TRUE(limits.Contains(Eigen::Array2i(0, 1)));
  EXPECT_TRUE(limits.Contains(Eigen::Array2i(1, 0)));
  EXPECT_TRUE(limits.Contains(Eigen::Array2i(1, 1)));
  EXPECT_FALSE(probability_grid.IsKnown(Eigen::Array2i(0, 0)));
  EXPECT_FALSE(probability_grid.IsKnown(Eigen::Array2i(0, 1)));
  EXPECT_FALSE(probability_grid.IsKnown(Eigen::Array2i(1, 0)));
  EXPECT_FALSE(probability_grid.IsKnown(Eigen::Array2i(1, 1)));

  probability_grid.SetProbability(Eigen::Array2i(1, 0), 0.5);

  probability_grid.StartUpdate();
  probability_grid.ApplyLookupTable(
      Eigen::Array2i(1, 0),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.9)));
  EXPECT_GT(probability_grid.GetProbability(Eigen::Array2i(1, 0)), 0.5);

  probability_grid.StartUpdate();
  probability_grid.SetProbability(Eigen::Array2i(0, 1), 0.5);

  probability_grid.StartUpdate();
  probability_grid.ApplyLookupTable(
      Eigen::Array2i(0, 1),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.1)));
  EXPECT_LT(probability_grid.GetProbability(Eigen::Array2i(0, 1)), 0.5);

  // Tests adding odds to an unknown cell.
  probability_grid.StartUpdate();
  probability_grid.ApplyLookupTable(
      Eigen::Array2i(1, 1),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.42)));
  EXPECT_NEAR(probability_grid.GetProbability(Eigen::Array2i(1, 1)), 0.42,
              1e-4);

  // Tests that further updates are ignored if StartUpdate() isn't called.
  probability_grid.ApplyLookupTable(
      Eigen::Array2i(1, 1),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.9)));
  EXPECT_NEAR(probability_grid.GetProbability(Eigen::Array2i(1, 1)), 0.42,
              1e-4);
  probability_grid.StartUpdate();
  probability_grid.ApplyLookupTable(
      Eigen::Array2i(1, 1),
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(0.9)));
  EXPECT_GT(probability_grid.GetProbability(Eigen::Array2i(1, 1)), 0.42);
}

TEST(ProbabilityGridTest, GetProbability) {
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::AlignedBox2d(Eigen::Vector2d(-0.5, 0.5),
                                        Eigen::Vector2d(0.5, 1.5))));

  const MapLimits& limits = probability_grid.limits();
  const Eigen::AlignedBox2d& edge_limits = limits.edge_limits();
  EXPECT_EQ(-1., edge_limits.min().x());
  EXPECT_EQ(0., edge_limits.min().y());
  EXPECT_EQ(1., edge_limits.max().x());
  EXPECT_EQ(2., edge_limits.max().y());

  const CellLimits& cell_limits = limits.cell_limits();
  ASSERT_EQ(2, cell_limits.num_x_cells);
  ASSERT_EQ(2, cell_limits.num_y_cells);

  probability_grid.StartUpdate();
  probability_grid.SetProbability(
      limits.GetXYIndexOfCellContainingPoint(-0.5, 0.5),
      mapping::kMaxProbability);
  EXPECT_NEAR(probability_grid.GetProbability(-0.5, 0.5),
              mapping::kMaxProbability, 1e-6);
  for (const Eigen::Array2i& xy_index :
       {limits.GetXYIndexOfCellContainingPoint(-0.5, 1.5),
        limits.GetXYIndexOfCellContainingPoint(0.5, 0.5),
        limits.GetXYIndexOfCellContainingPoint(0.5, 1.5)}) {
    EXPECT_TRUE(limits.Contains(xy_index));
    EXPECT_FALSE(probability_grid.IsKnown(xy_index));
  }
}

TEST(ProbabilityGridTest, GetXYIndexOfCellContainingPoint) {
  ProbabilityGrid probability_grid(MapLimits(2., 7, 13, CellLimits(14, 8)));

  const MapLimits& limits = probability_grid.limits();
  const CellLimits& cell_limits = limits.cell_limits();
  ASSERT_EQ(14, cell_limits.num_x_cells);
  ASSERT_EQ(8, cell_limits.num_y_cells);
  EXPECT_TRUE(
      (Eigen::Array2i(0, 0) == limits.GetXYIndexOfCellContainingPoint(7, 13))
          .all());
  EXPECT_TRUE(
      (Eigen::Array2i(13, 0) == limits.GetXYIndexOfCellContainingPoint(7, -13))
          .all());
  EXPECT_TRUE(
      (Eigen::Array2i(0, 7) == limits.GetXYIndexOfCellContainingPoint(-7, 13))
          .all());
  EXPECT_TRUE(
      (Eigen::Array2i(13, 7) == limits.GetXYIndexOfCellContainingPoint(-7, -13))
          .all());

  // Check around the origin.
  EXPECT_TRUE(
      (Eigen::Array2i(6, 3) == limits.GetXYIndexOfCellContainingPoint(0.5, 0.5))
          .all());
  EXPECT_TRUE(
      (Eigen::Array2i(6, 3) == limits.GetXYIndexOfCellContainingPoint(1.5, 1.5))
          .all());
  EXPECT_TRUE((Eigen::Array2i(7, 3) ==
               limits.GetXYIndexOfCellContainingPoint(0.5, -0.5))
                  .all());
  EXPECT_TRUE((Eigen::Array2i(6, 4) ==
               limits.GetXYIndexOfCellContainingPoint(-0.5, 0.5))
                  .all());
  EXPECT_TRUE((Eigen::Array2i(7, 4) ==
               limits.GetXYIndexOfCellContainingPoint(-0.5, -0.5))
                  .all());
}

TEST(ProbabilityGridTest, CorrectCropping) {
  // Create a probability grid with random values.
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> value_distribution(
      mapping::kMinProbability, mapping::kMaxProbability);
  ProbabilityGrid probability_grid(
      MapLimits(0.05, 10., 10., CellLimits(400, 400)));
  probability_grid.StartUpdate();
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(
           probability_grid.limits().cell_limits(), Eigen::Array2i(100, 100),
           Eigen::Array2i(299, 299))) {
    probability_grid.SetProbability(xy_index, value_distribution(rng));
  }
  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid.ComputeCroppedLimits(&offset, &limits);
  EXPECT_TRUE((offset == Eigen::Array2i(100, 100)).all());
  EXPECT_EQ(limits.num_x_cells, 200);
  EXPECT_EQ(limits.num_y_cells, 200);
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
