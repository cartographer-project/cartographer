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

#include "cartographer/sensor/point_cloud.h"

#include <cmath>

#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::ElementsAre;
using ::testing::FloatNear;
using ::testing::IsEmpty;

TEST(PointCloudTest, TransformPointCloud) {
  PointCloud point_cloud;
  point_cloud.push_back({{Eigen::Vector3f{0.5f, 0.5f, 1.f}}});
  point_cloud.push_back({{Eigen::Vector3f{3.5f, 0.5f, 42.f}}});
  const PointCloud transformed_point_cloud = TransformPointCloud(
      point_cloud, transform::Embed3D(transform::Rigid2f::Rotation(M_PI_2)));
  EXPECT_NEAR(-0.5f, transformed_point_cloud[0].position.x(), 1e-6);
  EXPECT_NEAR(0.5f, transformed_point_cloud[0].position.y(), 1e-6);
  EXPECT_NEAR(-0.5f, transformed_point_cloud[1].position.x(), 1e-6);
  EXPECT_NEAR(3.5f, transformed_point_cloud[1].position.y(), 1e-6);
}

TEST(PointCloudTest, TransformTimedPointCloud) {
  TimedPointCloud point_cloud;
  point_cloud.push_back({Eigen::Vector3f{0.5f, 0.5f, 1.f}, 0.f});
  point_cloud.push_back({Eigen::Vector3f{3.5f, 0.5f, 42.f}, 0.f});
  const TimedPointCloud transformed_point_cloud = TransformTimedPointCloud(
      point_cloud, transform::Embed3D(transform::Rigid2f::Rotation(M_PI_2)));
  EXPECT_NEAR(-0.5f, transformed_point_cloud[0].position.x(), 1e-6);
  EXPECT_NEAR(0.5f, transformed_point_cloud[0].position.y(), 1e-6);
  EXPECT_NEAR(-0.5f, transformed_point_cloud[1].position.x(), 1e-6);
  EXPECT_NEAR(3.5f, transformed_point_cloud[1].position.y(), 1e-6);
}

TEST(PointCloudTest, CopyIf) {
  std::vector<RangefinderPoint> points = {
      {{0.f, 0.f, 0.f}}, {{1.f, 1.f, 1.f}}, {{2.f, 2.f, 2.f}}};

  const PointCloud point_cloud(points);
  const PointCloud copied_point_cloud = point_cloud.copy_if(
      [&](const RangefinderPoint& point) { return point.position.x() > 0.1f; });

  EXPECT_EQ(copied_point_cloud.size(), 2);
  EXPECT_THAT(copied_point_cloud.intensities(), IsEmpty());
  EXPECT_THAT(copied_point_cloud.points(),
              ElementsAre(RangefinderPoint{Eigen::Vector3f{1.f, 1.f, 1.f}},
                          RangefinderPoint{Eigen::Vector3f{2.f, 2.f, 2.f}}));
}

TEST(PointCloudTest, CopyIfWithIntensities) {
  std::vector<RangefinderPoint> points = {
      {{0.f, 0.f, 0.f}}, {{1.f, 1.f, 1.f}}, {{2.f, 2.f, 2.f}}};
  std::vector<float> intensities = {0.f, 1.f, 2.f};

  const PointCloud point_cloud(points, intensities);
  const PointCloud copied_point_cloud = point_cloud.copy_if(
      [&](const RangefinderPoint& point) { return point.position.x() > 0.1f; });
  EXPECT_EQ(copied_point_cloud.size(), 2);
  EXPECT_EQ(copied_point_cloud.intensities().size(), 2);
  EXPECT_THAT(copied_point_cloud.points(),
              ElementsAre(RangefinderPoint{Eigen::Vector3f{1.f, 1.f, 1.f}},
                          RangefinderPoint{Eigen::Vector3f{2.f, 2.f, 2.f}}));
  EXPECT_THAT(copied_point_cloud.intensities(),
              ElementsAre(FloatNear(1.f, 1e-6), FloatNear(2.f, 1e-6)));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
