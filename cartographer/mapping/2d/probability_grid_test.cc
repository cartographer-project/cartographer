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

#include "cartographer/mapping/2d/probability_grid.h"

#include <random>

#include "cartographer/mapping/probability_values.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

using Eigen::Array2i;
using Eigen::Vector2f;

TEST(ProbabilityGridTest, ProtoConstructor) {
  proto::Grid2D proto;
  const MapLimits limits(1., {2., 3.}, CellLimits(4., 5.));
  *proto.mutable_limits() = ToProto(limits);
  for (int i = 6; i < 12; ++i) {
    proto.mutable_cells()->Add(static_cast<uint16>(i));
  }
  proto.mutable_known_cells_box()->set_max_x(19);
  proto.mutable_known_cells_box()->set_max_y(20);
  proto.mutable_known_cells_box()->set_min_x(21);
  proto.mutable_known_cells_box()->set_min_y(22);
  proto.mutable_probability_grid_2d();

  ValueConversionTables conversion_tables;
  ProbabilityGrid grid(proto, &conversion_tables);
  EXPECT_EQ(proto.limits().DebugString(), ToProto(grid.limits()).DebugString());
  EXPECT_EQ(grid.GetGridType(), GridType::PROBABILITY_GRID);

  // TODO(macmason): Figure out how to test the contents of cells_ and
  // {min, max}_{x, y}_ gracefully.
}

TEST(ProbabilityGridTest, ConstructorGridType) {
  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::Vector2d(1., 1.), CellLimits(2, 2)),
      &conversion_tables);
  EXPECT_EQ(probability_grid.GetGridType(), GridType::PROBABILITY_GRID);
}

TEST(ProbabilityGridTest, ToProto) {
  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::Vector2d(1., 1.), CellLimits(2, 2)),
      &conversion_tables);

  const auto proto = probability_grid.ToProto();
  EXPECT_EQ(ToProto(probability_grid.limits()).DebugString(),
            proto.limits().DebugString());

  // TODO(macmason): Figure out how to test the contents of cells_ and
  // {min, max}_{x, y}_ gracefully.
}

TEST(ProbabilityGridTest, ApplyOdds) {
  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::Vector2d(1., 1.), CellLimits(2, 2)),
      &conversion_tables);
  const MapLimits& limits = probability_grid.limits();

  EXPECT_TRUE(limits.Contains(Array2i(0, 0)));
  EXPECT_TRUE(limits.Contains(Array2i(0, 1)));
  EXPECT_TRUE(limits.Contains(Array2i(1, 0)));
  EXPECT_TRUE(limits.Contains(Array2i(1, 1)));
  EXPECT_FALSE(probability_grid.IsKnown(Array2i(0, 0)));
  EXPECT_FALSE(probability_grid.IsKnown(Array2i(0, 1)));
  EXPECT_FALSE(probability_grid.IsKnown(Array2i(1, 0)));
  EXPECT_FALSE(probability_grid.IsKnown(Array2i(1, 1)));

  probability_grid.SetProbability(Array2i(1, 0), 0.5);

  probability_grid.ApplyLookupTable(
      Array2i(1, 0),
      ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.9)));
  probability_grid.FinishUpdate();
  EXPECT_GT(probability_grid.GetProbability(Array2i(1, 0)), 0.5);

  probability_grid.SetProbability(Array2i(0, 1), 0.5);
  probability_grid.ApplyLookupTable(
      Array2i(0, 1),
      ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.1)));
  probability_grid.FinishUpdate();
  EXPECT_LT(probability_grid.GetProbability(Array2i(0, 1)), 0.5);

  // Tests adding odds to an unknown cell.
  probability_grid.ApplyLookupTable(
      Array2i(1, 1),
      ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.42)));
  EXPECT_NEAR(probability_grid.GetProbability(Array2i(1, 1)), 0.42, 1e-4);

  // Tests that further updates are ignored if FinishUpdate() isn't called.
  probability_grid.ApplyLookupTable(
      Array2i(1, 1),
      ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.9)));
  EXPECT_NEAR(probability_grid.GetProbability(Array2i(1, 1)), 0.42, 1e-4);
  probability_grid.FinishUpdate();
  probability_grid.ApplyLookupTable(
      Array2i(1, 1),
      ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(0.9)));
  EXPECT_GT(probability_grid.GetProbability(Array2i(1, 1)), 0.42);
}

TEST(ProbabilityGridTest, GetProbability) {
  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(1., Eigen::Vector2d(1., 2.), CellLimits(2, 2)),
      &conversion_tables);

  const MapLimits& limits = probability_grid.limits();
  EXPECT_EQ(1., limits.max().x());
  EXPECT_EQ(2., limits.max().y());

  const CellLimits& cell_limits = limits.cell_limits();
  ASSERT_EQ(2, cell_limits.num_x_cells);
  ASSERT_EQ(2, cell_limits.num_y_cells);

  probability_grid.SetProbability(limits.GetCellIndex(Vector2f(-0.5f, 0.5f)),
                                  kMaxProbability);
  EXPECT_NEAR(probability_grid.GetProbability(
                  limits.GetCellIndex(Vector2f(-0.5f, 0.5f))),
              kMaxProbability, 1e-6);
  for (const Array2i& xy_index : {limits.GetCellIndex(Vector2f(-0.5f, 1.5f)),
                                  limits.GetCellIndex(Vector2f(0.5f, 0.5f)),
                                  limits.GetCellIndex(Vector2f(0.5f, 1.5f))}) {
    EXPECT_TRUE(limits.Contains(xy_index));
    EXPECT_FALSE(probability_grid.IsKnown(xy_index));
  }
}

TEST(ProbabilityGridTest, GetCellIndex) {
  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(2., Eigen::Vector2d(8., 14.), CellLimits(14, 8)),
      &conversion_tables);

  const MapLimits& limits = probability_grid.limits();
  const CellLimits& cell_limits = limits.cell_limits();
  ASSERT_EQ(14, cell_limits.num_x_cells);
  ASSERT_EQ(8, cell_limits.num_y_cells);
  EXPECT_TRUE(
      (Array2i(0, 0) == limits.GetCellIndex(Vector2f(7.f, 13.f))).all());
  EXPECT_TRUE(
      (Array2i(13, 0) == limits.GetCellIndex(Vector2f(7.f, -13.f))).all());
  EXPECT_TRUE(
      (Array2i(0, 7) == limits.GetCellIndex(Vector2f(-7.f, 13.f))).all());
  EXPECT_TRUE(
      (Array2i(13, 7) == limits.GetCellIndex(Vector2f(-7.f, -13.f))).all());

  // Check around the origin.
  EXPECT_TRUE(
      (Array2i(6, 3) == limits.GetCellIndex(Vector2f(0.5f, 0.5f))).all());
  EXPECT_TRUE(
      (Array2i(6, 3) == limits.GetCellIndex(Vector2f(1.5f, 1.5f))).all());
  EXPECT_TRUE(
      (Array2i(7, 3) == limits.GetCellIndex(Vector2f(0.5f, -0.5f))).all());
  EXPECT_TRUE(
      (Array2i(6, 4) == limits.GetCellIndex(Vector2f(-0.5f, 0.5f))).all());
  EXPECT_TRUE(
      (Array2i(7, 4) == limits.GetCellIndex(Vector2f(-0.5f, -0.5f))).all());
}

TEST(ProbabilityGridTest, CorrectCropping) {
  // Create a probability grid with random values.
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> value_distribution(kMinProbability,
                                                           kMaxProbability);
  ValueConversionTables conversion_tables;
  ProbabilityGrid probability_grid(
      MapLimits(0.05, Eigen::Vector2d(10., 10.), CellLimits(400, 400)),
      &conversion_tables);
  for (const Array2i& xy_index :
       XYIndexRangeIterator(Array2i(100, 100), Array2i(299, 299))) {
    probability_grid.SetProbability(xy_index, value_distribution(rng));
  }
  Array2i offset;
  CellLimits limits;
  probability_grid.ComputeCroppedLimits(&offset, &limits);
  EXPECT_TRUE((offset == Array2i(100, 100)).all());
  EXPECT_EQ(limits.num_x_cells, 200);
  EXPECT_EQ(limits.num_y_cells, 200);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
