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
#include "cartographer/transform/transform.h"

#include <cmath>

#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

TEST(PointCloudTest, TransformPointCloud) {
  PointCloud point_cloud;
  point_cloud.emplace_back(0.5f, 0.5f, 1.f);
  point_cloud.emplace_back(3.5f, 0.5f, 42.f);
  const PointCloud transformed_point_cloud = TransformPointCloud(
      point_cloud, transform::Embed3D(transform::Rigid2f::Rotation(M_PI_2)));
  EXPECT_NEAR(-0.5f, transformed_point_cloud[0].x(), 1e-6);
  EXPECT_NEAR(0.5f, transformed_point_cloud[0].y(), 1e-6);
  EXPECT_NEAR(-0.5f, transformed_point_cloud[1].x(), 1e-6);
  EXPECT_NEAR(3.5f, transformed_point_cloud[1].y(), 1e-6);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
