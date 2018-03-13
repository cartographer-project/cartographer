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

#include "cartographer/sensor/landmark_data.h"

#include <cmath>

#include "cartographer/sensor/internal/test_helpers.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::DoubleNear;
using ::testing::Field;

::testing::Matcher<const LandmarkObservation&> EqualsLandmark(
    const LandmarkObservation& expected) {
  return ::testing::AllOf(
      Field(&LandmarkObservation::id, expected.id),
      Field(&LandmarkObservation::landmark_to_tracking_transform,
            transform::IsNearly(expected.landmark_to_tracking_transform, 1e-2)),
      Field(&LandmarkObservation::translation_weight,
            DoubleNear(expected.translation_weight, 0.01)),
      Field(&LandmarkObservation::rotation_weight,
            DoubleNear(expected.rotation_weight, 0.01)));
}

class LandmarkDataTest : public ::testing::Test {
 protected:
  LandmarkDataTest()
      : observations_(
            {{
                 "ID1",
                 transform::Rigid3d(Eigen::Vector3d(1., 1., 1.),
                                    Eigen::Quaterniond(1., 1., -1., -1.)),
                 1.f,
                 3.f,
             },
             {
                 "ID2",
                 transform::Rigid3d(Eigen::Vector3d(2., 2., 2.),
                                    Eigen::Quaterniond(2., 2., -2., -2.)),
                 2.f,
                 4.f,
             }}) {}
  std::vector<LandmarkObservation> observations_;
};

TEST_F(LandmarkDataTest, LandmarkDataToAndFromProto) {
  const auto expected = LandmarkData{common::FromUniversal(50), observations_};
  const auto actual = FromProto(ToProto(expected));
  EXPECT_EQ(expected.time, actual.time);
  EXPECT_THAT(actual.landmark_observations,
              ElementsAre(EqualsLandmark(expected.landmark_observations[0]),
                          EqualsLandmark(expected.landmark_observations[1])));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
