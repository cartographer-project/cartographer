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

#include "cartographer/transform/timestamped_transform.h"

#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace transform {
namespace {

TEST(TimestampedTransformTest, ToProtoAndBack) {
  const TimestampedTransform expected{
      common::FromUniversal(12345678),
      Rigid3d(Eigen::Vector3d(1., 2., 3.),
              Eigen::Quaterniond(1., 2., 3., 4.).normalized())};
  const TimestampedTransform actual = FromProto(ToProto(expected));
  EXPECT_EQ(expected.time, actual.time);
  EXPECT_THAT(actual.transform, IsNearly(expected.transform, 1e-6));
}

TEST(TimestampedTransformTest, InterpolateTransform) {
  transform::Rigid3d transform_0 = transform::Rigid3d::Identity();
  transform::Rigid3d transform_1 =
      transform::Rigid3d::Translation(Eigen::Vector3d(10., 10., 10.)) *
      transform::Rigid3d::Rotation(
          Eigen::AngleAxisd(2., Eigen::Vector3d::UnitZ()));

  const size_t ticks_0 = 1000;
  const size_t ticks_delta = 100000;
  common::Time time_0 = common::FromUniversal(ticks_0);
  common::Time time_1 = common::FromUniversal(ticks_0 + ticks_delta);

  EXPECT_THAT(
      transform_0,
      IsNearly(InterpolateTransform(transform_0, transform_1, 0.0), 1e-6));
  EXPECT_THAT(
      transform_1,
      IsNearly(InterpolateTransform(transform_0, transform_1, 1.0), 1e-6));
  EXPECT_THAT(transform_0,
              IsNearly(InterpolateTransform(transform_0, transform_1, time_0,
                                            time_1, time_0),
                       1e-6));
  EXPECT_THAT(transform_1,
              IsNearly(InterpolateTransform(transform_0, transform_1, time_0,
                                            time_1, time_1),
                       1e-6));

  size_t max_factor_idx = 20;
  double factor = 0.0;
  for (size_t factor_idx = 0; factor_idx <= max_factor_idx; ++factor_idx) {
    factor = double(factor_idx) / (max_factor_idx);
    EXPECT_THAT(InterpolateTransform(transform_0, transform_1, factor),
                IsNearly(transform::Rigid3d::Translation(
                             factor * transform_1.translation()) *
                             transform::Rigid3d::Rotation(Eigen::AngleAxisd(
                                 factor * 2., Eigen::Vector3d::UnitZ())),
                         1e-6));
    common::Time time_i =
        common::FromUniversal(ticks_0 + std::round(factor * ticks_delta));
    EXPECT_THAT(
        InterpolateTransform(transform_0, transform_1, time_0, time_1, time_i),
        IsNearly(transform::Rigid3d::Translation(factor *
                                                 transform_1.translation()) *
                     transform::Rigid3d::Rotation(Eigen::AngleAxisd(
                         factor * 2., Eigen::Vector3d::UnitZ())),
                 1e-6));
  }
}

}  // namespace
}  // namespace transform
}  // namespace cartographer
