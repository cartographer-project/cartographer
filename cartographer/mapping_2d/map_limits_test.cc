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

#include "cartographer/mapping_2d/map_limits.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_2d {
namespace {

TEST(MapLimits, SetEdgeLimits) {
  const MapLimits limits(2., Eigen::AlignedBox2d(Eigen::Vector2d(-0.5, 1.5),
                                                 Eigen::Vector2d(0.5, 5.5)));

  const Eigen::AlignedBox2d& edge_limits = limits.edge_limits();
  EXPECT_EQ(-2., edge_limits.min().x());
  EXPECT_EQ(0., edge_limits.min().y());
  EXPECT_EQ(2., edge_limits.max().x());
  EXPECT_EQ(6., edge_limits.max().y());

  const CellLimits& cell_limits = limits.cell_limits();
  EXPECT_EQ(3, cell_limits.num_x_cells);
  EXPECT_EQ(2, cell_limits.num_y_cells);
}

TEST(MapLimits, SetCellLimits) {
  const MapLimits limits(3., 1.5, -0.5, CellLimits(2, 3));

  const CellLimits& cell_limits = limits.cell_limits();
  EXPECT_EQ(2, cell_limits.num_x_cells);
  EXPECT_EQ(3, cell_limits.num_y_cells);

  const Eigen::AlignedBox2d& edge_limits = limits.edge_limits();
  EXPECT_EQ(-6., edge_limits.min().x());
  EXPECT_EQ(-6., edge_limits.min().y());
  EXPECT_EQ(3., edge_limits.max().x());
  EXPECT_EQ(0., edge_limits.max().y());
}

TEST(MapLimits, GetCenteredLimits) {
  const MapLimits limits(2., -0.5, 1.5, CellLimits(3, 2));

  const Eigen::AlignedBox2d& centered_limits = limits.centered_limits();
  EXPECT_EQ(-3, centered_limits.min().x());
  EXPECT_EQ(-3., centered_limits.min().y());
  EXPECT_EQ(-1., centered_limits.max().x());
  EXPECT_EQ(1., centered_limits.max().y());
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
