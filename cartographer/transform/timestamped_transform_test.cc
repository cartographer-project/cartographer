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

}  // namespace
}  // namespace transform
}  // namespace cartographer
