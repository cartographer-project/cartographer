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

#include "cartographer/sensor/internal/voxel_filter.h"

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
  EXPECT_THAT(VoxelFilter(0.3f).Filter(point_cloud),
              ContainerEq(PointCloud{point_cloud[0], point_cloud[2]}));
}

TEST(VoxelFilterTest, HandlesLargeCoordinates) {
  PointCloud point_cloud = {{100000.f, 0.f, 0.f},
                            {100000.001f, -0.0001f, 0.0001f},
                            {100000.003f, -0.0001f, 0.f},
                            {-200000.f, 0.f, 0.f}};
  EXPECT_THAT(VoxelFilter(0.01f).Filter(point_cloud),
              ContainerEq(PointCloud{point_cloud[0], point_cloud[3]}));
}

TEST(VoxelFilterTest, IgnoresTime) {
  TimedPointCloud timed_point_cloud;
  for (int i = 0; i < 100; ++i) {
    timed_point_cloud.emplace_back(-100.f, 0.3f, 0.4f, 1.f * i);
  }
  EXPECT_THAT(VoxelFilter(0.3f).Filter(timed_point_cloud),
              ContainerEq(TimedPointCloud{timed_point_cloud[0]}));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
