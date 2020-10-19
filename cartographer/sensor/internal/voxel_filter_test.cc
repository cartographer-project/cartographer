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

using ::testing::Contains;
using ::testing::IsEmpty;

TEST(VoxelFilterTest, ReturnsOnePointInEachVoxel) {
  const PointCloud point_cloud({{{0.f, 0.f, 0.f}},
                                {{0.1f, -0.1f, 0.1f}},
                                {{0.3f, -0.1f, 0.f}},
                                {{0.f, 0.f, 0.1f}}});
  const PointCloud result = VoxelFilter(point_cloud, 0.3f);
  ASSERT_EQ(result.size(), 2);
  EXPECT_THAT(result.intensities(), IsEmpty());
  EXPECT_THAT(point_cloud.points(), Contains(result[0]));
  EXPECT_THAT(point_cloud.points(), Contains(result[1]));
  EXPECT_THAT(result.points(), Contains(point_cloud[2]));
}

TEST(VoxelFilterTest, CorrectIntensities) {
  std::vector<RangefinderPoint> points;
  std::vector<float> intensities;
  for (int i = 0; i < 100; ++i) {
    const float value = 0.1f * i;
    // We add points with intensity equal to the z coordinate, so we can later
    // verify that the resulting intensities are corresponding to the filtered
    // points.
    points.push_back({{-100.f, 0.3f, value}});
    intensities.push_back(value);
  }
  const PointCloud point_cloud(points, intensities);
  const PointCloud result = VoxelFilter(point_cloud, 0.3f);

  ASSERT_EQ(result.intensities().size(), result.points().size());
  for (size_t i = 0; i < result.size(); ++i) {
    ASSERT_NEAR(result[i].position.z(), result.intensities()[i], 1e-6);
  }
}

TEST(VoxelFilterTest, HandlesLargeCoordinates) {
  const PointCloud point_cloud({{{100000.f, 0.f, 0.f}},
                                {{100000.001f, -0.0001f, 0.0001f}},
                                {{100000.003f, -0.0001f, 0.f}},
                                {{-200000.f, 0.f, 0.f}}});
  const PointCloud result = VoxelFilter(point_cloud, 0.01f);
  EXPECT_EQ(result.size(), 2);
  EXPECT_THAT(result.intensities(), IsEmpty());
  EXPECT_THAT(result.points(), Contains(point_cloud[3]));
}

TEST(VoxelFilterTest, IgnoresTime) {
  TimedPointCloud timed_point_cloud;
  for (int i = 0; i < 100; ++i) {
    timed_point_cloud.push_back({{-100.f, 0.3f, 0.4f}, 1.f * i});
  }
  const TimedPointCloud result = VoxelFilter(timed_point_cloud, 0.3f);
  ASSERT_EQ(result.size(), 1);
  EXPECT_THAT(timed_point_cloud, Contains(result[0]));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
