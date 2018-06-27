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

#include "cartographer/mapping/internal/2d/ray_casting.h"

#include "cartographer/mapping/2d/map_limits.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(RayCastingTest, SingleCell) {
  const Eigen::Array2i& begin = {1, 1};
  const Eigen::Array2i& end = {1, 1};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 1);
  EXPECT_TRUE(begin.isApprox(ray[0]));
}

TEST(RayCastingTest, AxisAlignedX) {
  const Eigen::Array2i& begin = {1, 1};
  const Eigen::Array2i& end = {3, 1};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 3);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({2, 1})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({3, 1})));
  ray = CastRay(end, begin, subpixel_scale);
  EXPECT_EQ(ray.size(), 3);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({2, 1})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({3, 1})));
}

TEST(RayCastingTest, AxisAlignedY) {
  const Eigen::Array2i& begin = {1, 1};
  const Eigen::Array2i& end = {1, 3};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 3);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({1, 2})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({1, 3})));
  ray = CastRay(end, begin, subpixel_scale);
  EXPECT_EQ(ray.size(), 3);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({1, 2})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({1, 3})));
}

TEST(RayCastingTest, Diagonal) {
  Eigen::Array2i begin = {1, 1};
  Eigen::Array2i end = {3, 3};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 3);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({2, 2})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({3, 3})));
  ray = CastRay(end, begin, subpixel_scale);
  EXPECT_EQ(ray.size(), 3);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({2, 2})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({3, 3})));
  begin = Eigen::Array2i({1, 3});
  end = Eigen::Array2i({3, 1});
  ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 3);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 3})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({2, 2})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({3, 1})));
  ray = CastRay(end, begin, subpixel_scale);
  EXPECT_EQ(ray.size(), 3);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 3})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({2, 2})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({3, 1})));
}

TEST(RayCastingTest, SteepLine) {
  Eigen::Array2i begin = {1, 1};
  Eigen::Array2i end = {2, 5};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 6);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({1, 2})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({1, 3})));
  EXPECT_TRUE(ray[3].isApprox(Eigen::Array2i({2, 3})));
  EXPECT_TRUE(ray[4].isApprox(Eigen::Array2i({2, 4})));
  EXPECT_TRUE(ray[5].isApprox(Eigen::Array2i({2, 5})));

  begin = {1, 1};
  end = {2, 4};
  ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 4);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({1, 2})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({2, 3})));
  EXPECT_TRUE(ray[3].isApprox(Eigen::Array2i({2, 4})));
}

TEST(RayCastingTest, FlatLine) {
  Eigen::Array2i begin = {1, 1};
  Eigen::Array2i end = {5, 2};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 6);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({2, 1})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({3, 1})));
  EXPECT_TRUE(ray[3].isApprox(Eigen::Array2i({3, 2})));
  EXPECT_TRUE(ray[4].isApprox(Eigen::Array2i({4, 2})));
  EXPECT_TRUE(ray[5].isApprox(Eigen::Array2i({5, 2})));

  begin = {1, 1};
  end = {4, 2};
  ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 4);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({1, 1})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({2, 1})));
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({3, 2})));
  EXPECT_TRUE(ray[3].isApprox(Eigen::Array2i({4, 2})));
}

TEST(RayCastingTest, MultiScaleAxisAlignedX) {
  int subpixel_scale = 1;
  const int num_cells_x = 20;
  const int num_cells_y = 20;
  double resolution = 0.1;
  Eigen::Vector2d max = {1.0, 1.0};
  double superscaled_resolution = resolution / subpixel_scale;
  MapLimits superscaled_limits(
      superscaled_resolution, max,
      CellLimits(num_cells_x * subpixel_scale, num_cells_y * subpixel_scale));
  Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(Eigen::Vector2f({0.05, 0.05}));
  Eigen::Array2i end =
      superscaled_limits.GetCellIndex(Eigen::Vector2f({0.35, 0.05}));
  const std::vector<Eigen::Array2i> base_scale_ray =
      CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(base_scale_ray.size(), 4);
  EXPECT_TRUE(base_scale_ray[0].isApprox(Eigen::Array2i({9, 6})));
  EXPECT_TRUE(base_scale_ray[1].isApprox(Eigen::Array2i({9, 7})));
  EXPECT_TRUE(base_scale_ray[2].isApprox(Eigen::Array2i({9, 8})));
  EXPECT_TRUE(base_scale_ray[3].isApprox(Eigen::Array2i({9, 9})));
  for (subpixel_scale = 2; subpixel_scale < 10000; subpixel_scale *= 2) {
    superscaled_resolution = resolution / subpixel_scale;
    superscaled_limits = MapLimits(
        superscaled_resolution, max,
        CellLimits(num_cells_x * subpixel_scale, num_cells_y * subpixel_scale));
    begin = superscaled_limits.GetCellIndex(Eigen::Vector2f({0.05, 0.05}));
    end = superscaled_limits.GetCellIndex(Eigen::Vector2f({0.35, 0.05}));
    const std::vector<Eigen::Array2i> superscaled_ray =
        CastRay(begin, end, subpixel_scale);
    EXPECT_EQ(superscaled_ray.size(), 4);
    for (size_t ray_index = 0; ray_index < superscaled_ray.size();
         ++ray_index) {
      EXPECT_TRUE(
          base_scale_ray[ray_index].isApprox(superscaled_ray[ray_index]));
    }
  }
}

TEST(RayCastingTest, MultiScaleSkewedLine) {
  int subpixel_scale = 1;
  const int num_cells_x = 20;
  const int num_cells_y = 20;
  double resolution = 0.1;
  Eigen::Vector2d max = {1.0, 1.0};
  double superscaled_resolution = resolution / subpixel_scale;
  MapLimits superscaled_limits(
      superscaled_resolution, max,
      CellLimits(num_cells_x * subpixel_scale, num_cells_y * subpixel_scale));
  Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(Eigen::Vector2f({0.01, 0.09}));
  Eigen::Array2i end =
      superscaled_limits.GetCellIndex(Eigen::Vector2f({0.21, 0.19}));
  std::vector<Eigen::Array2i> ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 4);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({8, 7})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({8, 8})));
  // expected wrong pixel as subpixel_scale is too low
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({9, 8})));
  EXPECT_TRUE(ray[3].isApprox(Eigen::Array2i({9, 9})));

  subpixel_scale = 20;
  superscaled_resolution = resolution / subpixel_scale;
  superscaled_limits = MapLimits(
      superscaled_resolution, max,
      CellLimits(num_cells_x * subpixel_scale, num_cells_y * subpixel_scale));
  begin = superscaled_limits.GetCellIndex(Eigen::Vector2f({0.01, 0.09}));
  end = superscaled_limits.GetCellIndex(Eigen::Vector2f({0.21, 0.19}));
  ray = CastRay(begin, end, subpixel_scale);
  EXPECT_EQ(ray.size(), 4);
  EXPECT_TRUE(ray[0].isApprox(Eigen::Array2i({8, 7})));
  EXPECT_TRUE(ray[1].isApprox(Eigen::Array2i({8, 8})));
  // expected correct pixel as subpixel_scale is sufficiently high
  EXPECT_TRUE(ray[2].isApprox(Eigen::Array2i({8, 9})));
  EXPECT_TRUE(ray[3].isApprox(Eigen::Array2i({9, 9})));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
