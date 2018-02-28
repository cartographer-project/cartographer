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

#include "cartographer/mapping/internal/motion_filter.h"

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

class MotionFilterTest : public ::testing::Test {
 protected:
  MotionFilterTest() {
    auto parameter_dictionary = common::MakeDictionary(
        "return {"
        "max_time_seconds = 0.5, "
        "max_distance_meters = 0.2, "
        "max_angle_radians = 2., "
        "}");
    options_ = CreateMotionFilterOptions(parameter_dictionary.get());
  }

  common::Time SecondsSinceEpoch(int seconds) {
    return common::FromUniversal(seconds * 10000000);
  }

  proto::MotionFilterOptions options_;
};

TEST_F(MotionFilterTest, NotInitialized) {
  MotionFilter motion_filter(options_);
  EXPECT_FALSE(
      motion_filter.IsSimilar(common::Time(), transform::Rigid3d::Identity()));
}

TEST_F(MotionFilterTest, NoChange) {
  MotionFilter motion_filter(options_);
  EXPECT_FALSE(motion_filter.IsSimilar(SecondsSinceEpoch(42),
                                       transform::Rigid3d::Identity()));
  EXPECT_TRUE(motion_filter.IsSimilar(SecondsSinceEpoch(42),
                                      transform::Rigid3d::Identity()));
}

TEST_F(MotionFilterTest, TimeElapsed) {
  MotionFilter motion_filter(options_);
  EXPECT_FALSE(motion_filter.IsSimilar(SecondsSinceEpoch(42),
                                       transform::Rigid3d::Identity()));
  EXPECT_FALSE(motion_filter.IsSimilar(SecondsSinceEpoch(43),
                                       transform::Rigid3d::Identity()));
  EXPECT_TRUE(motion_filter.IsSimilar(SecondsSinceEpoch(43),
                                      transform::Rigid3d::Identity()));
}

TEST_F(MotionFilterTest, LinearMotion) {
  MotionFilter motion_filter(options_);
  EXPECT_FALSE(motion_filter.IsSimilar(SecondsSinceEpoch(42),
                                       transform::Rigid3d::Identity()));
  EXPECT_FALSE(motion_filter.IsSimilar(
      SecondsSinceEpoch(42),
      transform::Rigid3d::Translation(Eigen::Vector3d(0.3, 0., 0.))));
  EXPECT_TRUE(motion_filter.IsSimilar(
      SecondsSinceEpoch(42),
      transform::Rigid3d::Translation(Eigen::Vector3d(0.45, 0., 0.))));
  EXPECT_FALSE(motion_filter.IsSimilar(
      SecondsSinceEpoch(42),
      transform::Rigid3d::Translation(Eigen::Vector3d(0.6, 0., 0.))));
  EXPECT_TRUE(motion_filter.IsSimilar(
      SecondsSinceEpoch(42),
      transform::Rigid3d::Translation(Eigen::Vector3d(0.6, 0.15, 0.))));
}

TEST_F(MotionFilterTest, RotationalMotion) {
  MotionFilter motion_filter(options_);
  EXPECT_FALSE(motion_filter.IsSimilar(SecondsSinceEpoch(42),
                                       transform::Rigid3d::Identity()));
  EXPECT_TRUE(motion_filter.IsSimilar(
      SecondsSinceEpoch(42), transform::Rigid3d::Rotation(Eigen::AngleAxisd(
                                 1.9, Eigen::Vector3d::UnitY()))));
  EXPECT_FALSE(motion_filter.IsSimilar(
      SecondsSinceEpoch(42), transform::Rigid3d::Rotation(Eigen::AngleAxisd(
                                 2.1, Eigen::Vector3d::UnitY()))));
  EXPECT_TRUE(motion_filter.IsSimilar(
      SecondsSinceEpoch(42), transform::Rigid3d::Rotation(Eigen::AngleAxisd(
                                 4., Eigen::Vector3d::UnitY()))));
  EXPECT_FALSE(motion_filter.IsSimilar(
      SecondsSinceEpoch(42), transform::Rigid3d::Rotation(Eigen::AngleAxisd(
                                 5.9, Eigen::Vector3d::UnitY()))));
  EXPECT_TRUE(motion_filter.IsSimilar(SecondsSinceEpoch(42),
                                      transform::Rigid3d::Identity()));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
