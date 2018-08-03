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

#include "cartographer/mapping/2d/tsdf_2d.h"

#include <random>

#include "cartographer/mapping/value_conversion_tables.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

using Eigen::Array2i;
using Eigen::Vector2f;

TEST(TSDF2DTest, ProtoConstructor) {
  proto::Grid2D proto;
  const MapLimits limits(1., {2., 3.}, CellLimits(4., 5.));
  *proto.mutable_limits() = ToProto(limits);
  proto.mutable_tsdf_2d()->set_max_weight(10.0);
  proto.mutable_tsdf_2d()->set_truncation_distance(1.0);
  for (int i = 6; i < 12; ++i) {
    proto.mutable_cells()->Add(static_cast<uint16>(i));
    proto.mutable_tsdf_2d()->mutable_weight_cells()->Add(
        static_cast<uint16>(i));
  }
  proto.mutable_known_cells_box()->set_max_x(19);
  proto.mutable_known_cells_box()->set_max_y(20);
  proto.mutable_known_cells_box()->set_min_x(21);
  proto.mutable_known_cells_box()->set_min_y(22);
  proto.set_max_correspondence_cost(1.0);
  proto.set_min_correspondence_cost(-1.0);

  ValueConversionTables conversion_tables;
  TSDF2D grid(proto, &conversion_tables);
  EXPECT_EQ(grid.GetGridType(), GridType::TSDF);
  EXPECT_EQ(proto.limits().DebugString(), ToProto(grid.limits()).DebugString());
}

TEST(TSDF2DTest, ConstructorGridType) {
  ValueConversionTables conversion_tables;
  TSDF2D tsdf(MapLimits(1., Eigen::Vector2d(1., 1.), CellLimits(2, 2)), 1.0f,
              10.0f, &conversion_tables);
  EXPECT_EQ(tsdf.GetGridType(), GridType::TSDF);
}

TEST(TSDF2DTest, ToProto) {
  ValueConversionTables conversion_tables;
  TSDF2D tsdf(MapLimits(1., Eigen::Vector2d(1., 1.), CellLimits(2, 2)), 1.0f,
              10.0f, &conversion_tables);

  const auto proto = tsdf.ToProto();
  EXPECT_EQ(ToProto(tsdf.limits()).DebugString(), proto.limits().DebugString());
}

TEST(TSDF2DTest, GetCellIndex) {
  ValueConversionTables conversion_tables;
  TSDF2D tsdf(MapLimits(2., Eigen::Vector2d(8., 14.), CellLimits(14, 8)), 1.f,
              10.f, &conversion_tables);

  const MapLimits& limits = tsdf.limits();
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

TEST(TSDF2DTest, WriteRead) {
  const float truncation_distance = 1.f;
  const float max_weight = 10.f;
  ValueConversionTables conversion_tables;
  TSDF2D tsdf(MapLimits(1., Eigen::Vector2d(1., 2.), CellLimits(2, 2)),
              truncation_distance, max_weight, &conversion_tables);

  const MapLimits& limits = tsdf.limits();
  EXPECT_EQ(1., limits.max().x());
  EXPECT_EQ(2., limits.max().y());

  const CellLimits& cell_limits = limits.cell_limits();
  ASSERT_EQ(2, cell_limits.num_x_cells);
  ASSERT_EQ(2, cell_limits.num_y_cells);

  std::mt19937 rng(42);
  std::uniform_real_distribution<float> tsd_distribution(-truncation_distance,
                                                         truncation_distance);
  std::uniform_real_distribution<float> weight_distribution(0.f, max_weight);
  for (size_t i = 0; i < 1; ++i) {
    const float tsd = tsd_distribution(rng);
    const float weight = weight_distribution(rng);
    tsdf.SetCell(limits.GetCellIndex(Vector2f(-0.5f, 0.5f)), tsd, weight);
    EXPECT_NEAR(
        tsdf.GetTSDAndWeight(limits.GetCellIndex(Vector2f(-0.5f, 0.5f))).first,
        tsd, 2.f * truncation_distance / 32768.f);
    EXPECT_NEAR(
        tsdf.GetTSDAndWeight(limits.GetCellIndex(Vector2f(-0.5f, 0.5f))).second,
        weight, max_weight / 32768.f);
    EXPECT_NEAR(tsdf.GetTSD(limits.GetCellIndex(Vector2f(-0.5f, 0.5f))), tsd,
                2.f * truncation_distance / 32768.f);
    EXPECT_NEAR(tsdf.GetWeight(limits.GetCellIndex(Vector2f(-0.5f, 0.5f))),
                weight, max_weight / 32768.f);
  }
  for (const Array2i& xy_index : {limits.GetCellIndex(Vector2f(-0.5f, 1.5f)),
                                  limits.GetCellIndex(Vector2f(0.5f, 0.5f)),
                                  limits.GetCellIndex(Vector2f(0.5f, 1.5f))}) {
    EXPECT_TRUE(limits.Contains(xy_index));
    EXPECT_FALSE(tsdf.IsKnown(xy_index));
  }
}

TEST(TSDF2DTest, CorrectCropping) {
  // Create a TSDF with random values.
  const float truncation_distance = 1.f;
  const float max_weight = 10.f;
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> tsdf_distribution(-truncation_distance,
                                                          truncation_distance);
  std::uniform_real_distribution<float> weight_distribution(0.f, max_weight);
  ValueConversionTables conversion_tables;
  TSDF2D tsdf(MapLimits(0.05, Eigen::Vector2d(10., 10.), CellLimits(400, 400)),
              truncation_distance, max_weight, &conversion_tables);
  for (const Array2i& xy_index :
       XYIndexRangeIterator(Array2i(100, 100), Array2i(299, 299))) {
    tsdf.SetCell(xy_index, tsdf_distribution(rng), weight_distribution(rng));
  }
  Array2i offset;
  CellLimits limits;
  tsdf.ComputeCroppedLimits(&offset, &limits);
  EXPECT_TRUE((offset == Array2i(100, 100)).all());
  EXPECT_EQ(limits.num_x_cells, 200);
  EXPECT_EQ(limits.num_y_cells, 200);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
