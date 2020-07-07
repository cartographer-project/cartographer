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

#include "cartographer/transform/transform_interpolation_buffer.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace transform {
namespace {

TEST(TransformInterpolationBufferTest, testHas) {
  TransformInterpolationBuffer buffer;
  EXPECT_FALSE(buffer.Has(common::FromUniversal(50)));
  buffer.Push(common::FromUniversal(50), transform::Rigid3d::Identity());
  EXPECT_FALSE(buffer.Has(common::FromUniversal(25)));
  EXPECT_TRUE(buffer.Has(common::FromUniversal(50)));
  EXPECT_FALSE(buffer.Has(common::FromUniversal(75)));
  buffer.Push(common::FromUniversal(100), transform::Rigid3d::Identity());
  EXPECT_FALSE(buffer.Has(common::FromUniversal(25)));
  EXPECT_TRUE(buffer.Has(common::FromUniversal(50)));
  EXPECT_TRUE(buffer.Has(common::FromUniversal(75)));
  EXPECT_TRUE(buffer.Has(common::FromUniversal(100)));
  EXPECT_FALSE(buffer.Has(common::FromUniversal(125)));
  EXPECT_EQ(common::FromUniversal(50), buffer.earliest_time());
  EXPECT_EQ(common::FromUniversal(100), buffer.latest_time());
}

TEST(TransformInterpolationBufferTest, testLookup) {
  TransformInterpolationBuffer buffer;
  buffer.Push(common::FromUniversal(50), transform::Rigid3d::Identity());
  // The rotation needs to be relatively small in order for the interpolation to
  // remain a z-axis rotation.
  buffer.Push(common::FromUniversal(100),
              transform::Rigid3d::Translation(Eigen::Vector3d(10., 10., 10.)) *
                  transform::Rigid3d::Rotation(
                      Eigen::AngleAxisd(2., Eigen::Vector3d::UnitZ())));
  const common::Time time = common::FromUniversal(75);
  const transform::Rigid3d interpolated = buffer.Lookup(time);
  EXPECT_THAT(
      interpolated,
      IsNearly(transform::Rigid3d::Translation(Eigen::Vector3d(5., 5., 5.)) *
                   transform::Rigid3d::Rotation(
                       Eigen::AngleAxisd(1., Eigen::Vector3d::UnitZ())),
               1e-6));
}

TEST(TransformInterpolationBufferTest, testLookupSingleTransform) {
  TransformInterpolationBuffer buffer;
  const common::Time time = common::FromUniversal(75);
  buffer.Push(time, transform::Rigid3d::Identity());
  const transform::Rigid3d interpolated = buffer.Lookup(time);
  EXPECT_THAT(interpolated, IsNearly(transform::Rigid3d::Identity(), 1e-6));
}

TEST(TransformInterpolationBufferTest, testSetSizeLimit) {
  TransformInterpolationBuffer buffer;
  EXPECT_EQ(buffer.size_limit(), kUnlimitedBufferSize);
  buffer.Push(common::FromUniversal(0), transform::Rigid3d::Identity());
  buffer.Push(common::FromUniversal(1), transform::Rigid3d::Identity());
  buffer.Push(common::FromUniversal(2), transform::Rigid3d::Identity());
  EXPECT_EQ(buffer.size(), 3);
  buffer.SetSizeLimit(2);
  EXPECT_EQ(buffer.size_limit(), 2);
  EXPECT_EQ(buffer.size(), 2);
  EXPECT_FALSE(buffer.Has(common::FromUniversal(0)));
  buffer.Push(common::FromUniversal(3), transform::Rigid3d::Identity());
  EXPECT_EQ(buffer.size(), 2);
  EXPECT_FALSE(buffer.Has(common::FromUniversal(1)));
}

}  // namespace
}  // namespace transform
}  // namespace cartographer
