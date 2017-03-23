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

TEST(MapLimitsTest, ToProto) {
  const MapLimits limits(42., Eigen::Vector2d(3., 0.), CellLimits(2, 3));
  const auto proto = ToProto(limits);
  EXPECT_EQ(limits.resolution(), proto.resolution());
  EXPECT_EQ(limits.max().x(), proto.max().x());
  EXPECT_EQ(limits.max().y(), proto.max().y());
  EXPECT_EQ(ToProto(limits.cell_limits()).DebugString(),
            proto.cell_limits().DebugString());
}

TEST(MapLimitsTest, ProtoConstructor) {
  proto::MapLimits limits;
  limits.set_resolution(1.);
  limits.mutable_max()->set_x(2.);
  limits.mutable_max()->set_y(3.);
  limits.mutable_cell_limits()->set_num_x_cells(4);
  limits.mutable_cell_limits()->set_num_y_cells(5);

  const MapLimits native(limits);
  EXPECT_EQ(limits.resolution(), native.resolution());
  EXPECT_EQ(limits.max().x(), native.max().x());
  EXPECT_EQ(limits.max().y(), native.max().y());
  EXPECT_EQ(limits.cell_limits().DebugString(),
            ToProto(native.cell_limits()).DebugString());
}

TEST(MapLimitsTest, ConstructAndGet) {
  const MapLimits limits(42., Eigen::Vector2d(3., 0.), CellLimits(2, 3));

  const CellLimits& cell_limits = limits.cell_limits();
  EXPECT_EQ(2, cell_limits.num_x_cells);
  EXPECT_EQ(3, cell_limits.num_y_cells);

  const Eigen::Vector2d& max = limits.max();
  EXPECT_EQ(3., max.x());
  EXPECT_EQ(0., max.y());

  EXPECT_EQ(42., limits.resolution());
}

TEST(MapLimitsTest, ComputeMapLimits) {
  const mapping::TrajectoryNode::ConstantData constant_data{
      common::FromUniversal(52),
      sensor::RangeData{
          Eigen::Vector3f::Zero(),
          {Eigen::Vector3f(-30.f, 1.f, 0.f), Eigen::Vector3f(50.f, -10.f, 0.f)},
          {}},
      Compress(sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}, {}}), nullptr,
      transform::Rigid3d::Identity()};
  const mapping::TrajectoryNode trajectory_node{&constant_data,
                                                transform::Rigid3d::Identity()};
  constexpr double kResolution = 0.05;
  const MapLimits limits =
      MapLimits::ComputeMapLimits(kResolution, {trajectory_node});
  constexpr float kPaddingAwareTolerance = 5 * kResolution;
  EXPECT_NEAR(50.f, limits.max().x(), kPaddingAwareTolerance);
  EXPECT_NEAR(1.f, limits.max().y(), kPaddingAwareTolerance);
  EXPECT_LT(200, limits.cell_limits().num_x_cells);
  EXPECT_LT(1600, limits.cell_limits().num_y_cells);
  EXPECT_GT(400, limits.cell_limits().num_x_cells);
  EXPECT_GT(2000, limits.cell_limits().num_y_cells);
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
