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

#include "cartographer/sensor/voxel_filter.h"

#include <cmath>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::ContainerEq;

TEST(VoxelFilterTest, ReturnsTheFirstPointInEachVoxel) {
  PointCloud point_cloud = {{0.f, 0.f, 0.f},
                            {0.1f, -0.1f, 0.1f},
                            {0.3f, -0.1f, 0.f},
                            {0.f, 0.f, 0.1f}};
  EXPECT_THAT(SimpleVoxelFilter(0.3f).Filter(point_cloud),
              ContainerEq(PointCloud{point_cloud[0], point_cloud[2]}));
}

TEST(VoxelFilterTest, HandlesLargeCoordinates) {
  PointCloud point_cloud = {{1000.f, 0.f, 0.f},
                            {1000.001f, -0.0001f, 0.0001f},
                            {1000.003f, -0.0001f, 0.f},
                            {-1000.f, 0.f, 0.f}};
  EXPECT_THAT(SimpleVoxelFilter(0.01f).Filter(point_cloud),
              ContainerEq(PointCloud{point_cloud[0], point_cloud[3]}));
}

TEST(VoxelFilterTest, IgnoresTime) {
  TimedPointCloud timed_point_cloud;
  for (int i = 0; i < 100; ++i) {
    timed_point_cloud.emplace_back(-100.f, 0.3f, 0.4f, 1.f * i);
  }
  EXPECT_THAT(SimpleVoxelFilter(0.3f).Filter(timed_point_cloud),
              ContainerEq(TimedPointCloud{timed_point_cloud[0]}));
}

PointCloud CreatePointCloud() {
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> distribution(-100, 100);
  PointCloud point_cloud;
  for (int i = 0; i < 1000; ++i) {
    const auto x = distribution(rng);
    const auto y = distribution(rng);
    const auto z = distribution(rng);
    point_cloud.emplace_back(x, y, z);
  }
  return point_cloud;
}

TEST(VoxelFilterTest, VoxelFilterMany) {
  PointCloud input = CreatePointCloud();
  for (int i = 0; i < 1000; ++i) {
    auto output = VoxelFilter(1.f).Filter(input);
  }
}

TEST(VoxelFilterTest, SimpleVoxelFilterMany) {
  PointCloud input = CreatePointCloud();
  for (int i = 0; i < 1000; ++i) {
    auto output = SimpleVoxelFilter(1.f).Filter(input);
  }
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
