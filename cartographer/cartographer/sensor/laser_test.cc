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

#include "cartographer/sensor/laser.h"

#include <utility>
#include <vector>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;
using ::testing::PrintToString;

TEST(ProjectorTest, ToLaserFan) {
  proto::LaserScan laser_scan;
  for (int i = 0; i < 8; ++i) {
    laser_scan.add_range()->add_value(1.f);
  }
  laser_scan.set_angle_min(0.f);
  laser_scan.set_angle_max(8.f * static_cast<float>(M_PI_4));
  laser_scan.set_angle_increment(static_cast<float>(M_PI_4));

  const LaserFan fan = ToLaserFan(laser_scan, 0.f, 10.f, 1.f);
  EXPECT_TRUE(fan.point_cloud[0].isApprox(Eigen::Vector2f(1.f, 0.f), 1e-6));
  EXPECT_TRUE(fan.point_cloud[1].isApprox(
      Eigen::Vector2f(1.f / std::sqrt(2.f), 1.f / std::sqrt(2.f)), 1e-6));
  EXPECT_TRUE(fan.point_cloud[2].isApprox(Eigen::Vector2f(0.f, 1.f), 1e-6));
  EXPECT_TRUE(fan.point_cloud[3].isApprox(
      Eigen::Vector2f(-1.f / std::sqrt(2.f), 1.f / std::sqrt(2.f)), 1e-6));
  EXPECT_TRUE(fan.point_cloud[4].isApprox(Eigen::Vector2f(-1.f, 0.f), 1e-6));
  EXPECT_TRUE(fan.point_cloud[5].isApprox(
      Eigen::Vector2f(-1.f / std::sqrt(2.f), -1.f / std::sqrt(2.f)), 1e-6));
  EXPECT_TRUE(fan.point_cloud[6].isApprox(Eigen::Vector2f(0.f, -1.f), 1e-6));
  EXPECT_TRUE(fan.point_cloud[7].isApprox(
      Eigen::Vector2f(1.f / std::sqrt(2.f), -1.f / std::sqrt(2.f)), 1e-6));
}

TEST(ProjectorTest, ToLaserFanWithInfinityAndNaN) {
  proto::LaserScan laser_scan;
  laser_scan.add_range()->add_value(1.f);
  laser_scan.add_range()->add_value(std::numeric_limits<float>::infinity());
  laser_scan.add_range()->add_value(2.f);
  laser_scan.add_range()->add_value(std::numeric_limits<float>::quiet_NaN());
  laser_scan.add_range()->add_value(3.f);
  laser_scan.set_angle_min(0.f);
  laser_scan.set_angle_max(3.f * static_cast<float>(M_PI_4));
  laser_scan.set_angle_increment(static_cast<float>(M_PI_4));

  const LaserFan fan = ToLaserFan(laser_scan, 2.f, 10.f, 1.f);
  ASSERT_EQ(2, fan.point_cloud.size());
  EXPECT_TRUE(fan.point_cloud[0].isApprox(Eigen::Vector2f(0.f, 2.f), 1e-6));
  EXPECT_TRUE(fan.point_cloud[1].isApprox(Eigen::Vector2f(-3.f, 0.f), 1e-6));

  ASSERT_EQ(1, fan.missing_echo_point_cloud.size());
  EXPECT_TRUE(fan.missing_echo_point_cloud[0].isApprox(
      Eigen::Vector2f(1.f / std::sqrt(2.f), 1.f / std::sqrt(2.f)), 1e-6));
}

// Custom matcher for pair<eigen::Vector3f, int> entries.
MATCHER_P(PairApproximatelyEquals, expected,
          string("is equal to ") + PrintToString(expected)) {
  return (arg.first - expected.first).isZero(0.001f) &&
         arg.second == expected.second;
}

TEST(LaserTest, Compression) {
  LaserFan3D fan = {Eigen::Vector3f(1, 1, 1),
                    {Eigen::Vector3f(0, 1, 2), Eigen::Vector3f(4, 5, 6),
                     Eigen::Vector3f(0, 1, 2)},
                    {Eigen::Vector3f(7, 8, 9)},
                    {1, 2, 3}};
  LaserFan3D actual = Decompress(Compress(fan));
  EXPECT_TRUE(actual.origin.isApprox(Eigen::Vector3f(1, 1, 1), 1e-6));
  EXPECT_EQ(3, actual.returns.size());
  EXPECT_EQ(1, actual.misses.size());
  EXPECT_EQ(actual.returns.size(), actual.reflectivities.size());
  EXPECT_TRUE(actual.misses[0].isApprox(Eigen::Vector3f(7, 8, 9), 0.001f));

  // Returns and their corresponding reflectivities will be reordered, so we
  // pair them up into a vector, and compare in an unordered manner.
  std::vector<std::pair<Eigen::Vector3f, int>> pairs;
  for (size_t i = 0; i < actual.returns.size(); ++i) {
    pairs.emplace_back(actual.returns[i], actual.reflectivities[i]);
  }
  EXPECT_EQ(3, pairs.size());
  EXPECT_THAT(pairs,
              Contains(PairApproximatelyEquals(std::pair<Eigen::Vector3f, int>(
                  Eigen::Vector3f(0, 1, 2), 1))));
  EXPECT_THAT(pairs,
              Contains(PairApproximatelyEquals(std::pair<Eigen::Vector3f, int>(
                  Eigen::Vector3f(0, 1, 2), 3))));
  EXPECT_THAT(pairs,
              Contains(PairApproximatelyEquals(std::pair<Eigen::Vector3f, int>(
                  Eigen::Vector3f(4, 5, 6), 2))));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
