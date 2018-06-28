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

#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"

#include "cartographer/mapping/2d/map_limits.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::get;
using ::testing::Pointwise;

namespace cartographer {
namespace mapping {
namespace {

inline bool Equals(const Eigen::Array2i& lhs, const Eigen::Array2i& rhs) {
  return ((lhs - rhs).matrix().lpNorm<1>() == 0);
}

bool IsLess(const Eigen::Array2i& lhs, const Eigen::Array2i& rhs) {
  if (lhs[0] < rhs[0])
    return true;
  else if (lhs[0] > rhs[0])
    return false;
  else
    return lhs[1] < rhs[1];
}

MATCHER(PixelMaskEqual, "") { return Equals(get<0>(arg), get<1>(arg)); }

TEST(RayToPixelMaskTest, SingleCell) {
  const Eigen::Array2i& begin = {1, 1};
  const Eigen::Array2i& end = {1, 1};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, subpixel_scale);
  std::vector<Eigen::Array2i> ray_reference =
      std::vector<Eigen::Array2i>{begin};
  std::sort(ray.begin(), ray.end(), IsLess);
  std::sort(ray_reference.begin(), ray_reference.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
}

TEST(RayToPixelMaskTest, AxisAlignedX) {
  const Eigen::Array2i& begin = {1, 1};
  const Eigen::Array2i& end = {3, 1};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, subpixel_scale);
  std::vector<Eigen::Array2i> ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({1, 1}), Eigen::Array2i({2, 1}), Eigen::Array2i({3, 1})};
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
  ray = RayToPixelMask(end, begin, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
}

TEST(RayToPixelMaskTest, AxisAlignedY) {
  const Eigen::Array2i& begin = {1, 1};
  const Eigen::Array2i& end = {1, 3};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({1, 1}), Eigen::Array2i({1, 2}), Eigen::Array2i({1, 3})};
  std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
  ray = RayToPixelMask(end, begin, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
}

TEST(RayToPixelMaskTest, Diagonal) {
  Eigen::Array2i begin = {1, 1};
  Eigen::Array2i end = {3, 3};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({1, 1}), Eigen::Array2i({2, 2}), Eigen::Array2i({3, 3})};
  std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
  ray = RayToPixelMask(end, begin, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
  begin = Eigen::Array2i({1, 3});
  end = Eigen::Array2i({3, 1});
  ray = RayToPixelMask(begin, end, subpixel_scale);
  ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({1, 3}), Eigen::Array2i({2, 2}), Eigen::Array2i({3, 1})};
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
  ray = RayToPixelMask(end, begin, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
}

TEST(RayToPixelMaskTest, SteepLine) {
  Eigen::Array2i begin = {1, 1};
  Eigen::Array2i end = {2, 5};
  const int subpixel_scale = 1;
  std::vector<Eigen::Array2i> ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({1, 1}), Eigen::Array2i({1, 2}), Eigen::Array2i({1, 3}),
      Eigen::Array2i({2, 3}), Eigen::Array2i({2, 4}), Eigen::Array2i({2, 5})};
  std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
  begin = {1, 1};
  end = {2, 4};
  ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({1, 1}), Eigen::Array2i({1, 2}), Eigen::Array2i({2, 3}),
      Eigen::Array2i({2, 4})};
  ray = RayToPixelMask(begin, end, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
}

TEST(RayToPixelMaskTest, FlatLine) {
  Eigen::Array2i begin = {1, 1};
  Eigen::Array2i end = {5, 2};
  const int subpixel_scale = 1;

  std::vector<Eigen::Array2i> ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({1, 1}), Eigen::Array2i({2, 1}), Eigen::Array2i({3, 1}),
      Eigen::Array2i({3, 2}), Eigen::Array2i({4, 2}), Eigen::Array2i({5, 2})};
  std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));

  begin = {1, 1};
  end = {4, 2};
  ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({1, 1}), Eigen::Array2i({2, 1}), Eigen::Array2i({3, 2}),
      Eigen::Array2i({4, 2})};
  ray = RayToPixelMask(begin, end, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
}

TEST(RayToPixelMaskTest, MultiScaleAxisAlignedX) {
  int subpixel_scale = 1;
  const int num_cells_x = 20;
  const int num_cells_y = 20;
  double resolution = 0.1;
  Eigen::Vector2d max = {1.0, 1.0};
  std::vector<Eigen::Array2i> base_scale_ray;
  std::vector<Eigen::Array2i> ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({9, 6}), Eigen::Array2i({9, 7}), Eigen::Array2i({9, 8}),
      Eigen::Array2i({9, 9})};
  for (subpixel_scale = 1; subpixel_scale < 10000; subpixel_scale *= 2) {
    double superscaled_resolution = resolution / subpixel_scale;
    MapLimits superscaled_limits(
        superscaled_resolution, max,
        CellLimits(num_cells_x * subpixel_scale, num_cells_y * subpixel_scale));
    Eigen::Array2i begin =
        superscaled_limits.GetCellIndex(Eigen::Vector2f({0.05, 0.05}));
    Eigen::Array2i end =
        superscaled_limits.GetCellIndex(Eigen::Vector2f({0.35, 0.05}));
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, subpixel_scale);
    std::sort(ray.begin(), ray.end(), IsLess);
    EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
  }
}

TEST(RayToPixelMaskTest, MultiScaleSkewedLine) {
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

  std::vector<Eigen::Array2i> ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({8, 7}), Eigen::Array2i({8, 8}), Eigen::Array2i({9, 8}),
      Eigen::Array2i({9, 9})};
  std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
  subpixel_scale = 20;
  superscaled_resolution = resolution / subpixel_scale;
  superscaled_limits = MapLimits(
      superscaled_resolution, max,
      CellLimits(num_cells_x * subpixel_scale, num_cells_y * subpixel_scale));
  begin = superscaled_limits.GetCellIndex(Eigen::Vector2f({0.01, 0.09}));
  end = superscaled_limits.GetCellIndex(Eigen::Vector2f({0.21, 0.19}));
  ray_reference = std::vector<Eigen::Array2i>{
      Eigen::Array2i({8, 7}), Eigen::Array2i({8, 8}), Eigen::Array2i({8, 9}),
      Eigen::Array2i({9, 9})};
  ray = RayToPixelMask(begin, end, subpixel_scale);
  std::sort(ray.begin(), ray.end(), IsLess);
  EXPECT_THAT(ray_reference, Pointwise(PixelMaskEqual(), ray));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
