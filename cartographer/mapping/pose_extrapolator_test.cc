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

#include "cartographer/mapping/pose_extrapolator.h"

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr double kPoseQueueDuration = 0.5f;
constexpr double kGravityTimeConstant = 0.1f;
constexpr double kExtrapolateDuration = 0.1f;
constexpr double kPrecision = 1e-8;
constexpr double kExtrapolatePrecision = 1e-2;

TEST(PoseExtrapolatorDeathTest, IncompleteInitialization) {
  PoseExtrapolator extrapolator(common::FromSeconds(kPoseQueueDuration),
                                kGravityTimeConstant);
  common::Time time = common::FromUniversal(123);
  EXPECT_DEATH(extrapolator.EstimateGravityOrientation(time), "");
  EXPECT_DEATH(extrapolator.ExtrapolatePose(time), "");
  Eigen::Vector3d acceleration(0, 0, 9);
  Eigen::Vector3d angular_velocity(0, 0, 0);
  extrapolator.AddImuData(
      sensor::ImuData{time, acceleration, angular_velocity});
  EXPECT_DEATH(extrapolator.EstimateGravityOrientation(time), "");
  EXPECT_DEATH(extrapolator.ExtrapolatePose(time), "");
}

TEST(PoseExtrapolatorDeathTest, ExtrapolateInPast) {
  PoseExtrapolator extrapolator(common::FromSeconds(kPoseQueueDuration),
                                kGravityTimeConstant);
  common::Time time_present = common::FromUniversal(123);
  transform::Rigid3d pose =
      transform::Rigid3d::Translation(Eigen::Vector3d(0.1, 0.2, 0.3));
  extrapolator.AddPose(time_present, pose);
  extrapolator.EstimateGravityOrientation(time_present);
  EXPECT_THAT(extrapolator.ExtrapolatePose(time_present),
              transform::IsNearly(pose, kPrecision));
  common::Time time_in_past = time_present - common::FromSeconds(10);
  EXPECT_DEATH(extrapolator.EstimateGravityOrientation(time_in_past), "");
  EXPECT_DEATH(extrapolator.ExtrapolatePose(time_in_past), "");
  common::Time time_in_future = time_present + common::FromSeconds(20);
  extrapolator.ExtrapolatePose(time_in_future);
  EXPECT_DEATH(extrapolator.ExtrapolatePose(time_present), "");
}

TEST(PoseExtrapolatorTest, EstimateGravityOrientationWithIMU) {
  Eigen::Vector3d initial_gravity_acceleration(1.6, 2.0, 8.0);
  Eigen::Vector3d angular_velocity(0, 0, 0);
  common::Time current_time = common::FromUniversal(123);
  sensor::ImuData imu_data{current_time, initial_gravity_acceleration,
                           angular_velocity};
  auto extrapolator = PoseExtrapolator::InitializeWithImu(
      common::FromSeconds(kPoseQueueDuration), kGravityTimeConstant, imu_data);
  Eigen::Quaterniond expected_orientation;
  expected_orientation.setFromTwoVectors(initial_gravity_acceleration,
                                         Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(0.,
              extrapolator->EstimateGravityOrientation(current_time)
                  .angularDistance(expected_orientation),
              kPrecision);
  Eigen::Vector3d gravity_acceleration(1.6, 2.0, 8.0);
  for (int i = 0; i < 10; ++i) {
    current_time += common::FromSeconds(kGravityTimeConstant);
    extrapolator->AddImuData(
        sensor::ImuData{current_time, gravity_acceleration, angular_velocity});
  }
  expected_orientation.setFromTwoVectors(gravity_acceleration,
                                         Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(0.,
              extrapolator->EstimateGravityOrientation(current_time)
                  .angularDistance(expected_orientation),
              kPrecision);
}

TEST(PoseExtrapolatorTest, ExtrapolateWithPoses) {
  PoseExtrapolator extrapolator(common::FromSeconds(kPoseQueueDuration),
                                kGravityTimeConstant);
  common::Time current_time = common::FromUniversal(123);
  transform::Rigid3d current_pose =
      transform::Rigid3d::Translation(Eigen::Vector3d(0.3, 0.7, 0.2));
  Eigen::Vector3d velocity(0, 0.1, 0);
  Eigen::Vector3d angular_velocity(0.01, 0, 0.1);
  transform::Rigid3d motion_per_second(
      velocity, Eigen::AngleAxisd(angular_velocity.norm(),
                                  angular_velocity.normalized()));
  extrapolator.AddPose(current_time, current_pose);
  EXPECT_EQ(common::ToUniversal(extrapolator.GetLastPoseTime()),
            common::ToUniversal(current_time));
  EXPECT_THAT(extrapolator.ExtrapolatePose(current_time),
              transform::IsNearly(current_pose, kPrecision));
  for (int i = 0; i < 5; ++i) {
    current_time += common::FromSeconds(1);
    current_pose = current_pose * motion_per_second;
    extrapolator.AddPose(current_time, current_pose);
    EXPECT_THAT(extrapolator.ExtrapolatePose(current_time),
                transform::IsNearly(current_pose, kPrecision));
    transform::Rigid3d expected_pose =
        current_pose *
        transform::Rigid3d(
            kExtrapolateDuration * velocity,
            Eigen::AngleAxisd(kExtrapolateDuration * angular_velocity.norm(),
                              angular_velocity.normalized()));
    EXPECT_THAT(extrapolator.ExtrapolatePose(
                    current_time + common::FromSeconds(kExtrapolateDuration)),
                transform::IsNearly(expected_pose, kExtrapolatePrecision));
    EXPECT_EQ(common::ToUniversal(extrapolator.GetLastExtrapolatedTime()),
              common::ToUniversal(current_time +
                                  common::FromSeconds(kExtrapolateDuration)));
  }
  Eigen::AngleAxisd gravity_axis(
      extrapolator.EstimateGravityOrientation(current_time));
  EXPECT_NEAR(
      0.f, (gravity_axis.axis().normalized() - Eigen::Vector3d::UnitZ()).norm(),
      kExtrapolatePrecision);
}

TEST(PoseExtrapolatorTest, ExtrapolateWithIMU) {
  Eigen::Vector3d initial_gravity_acceleration(0, 0, 9.8);
  Eigen::Vector3d initial_angular_velocity(0, 0, 0);
  common::Time current_time = common::FromUniversal(123);
  sensor::ImuData imu_data{current_time, initial_gravity_acceleration,
                           initial_angular_velocity};
  auto extrapolator = PoseExtrapolator::InitializeWithImu(
      common::FromSeconds(kPoseQueueDuration), kGravityTimeConstant, imu_data);
  transform::Rigid3d current_pose =
      transform::Rigid3d::Translation(Eigen::Vector3d(0.3, 0.7, 0.2));
  // Let velocity estimation come to rest.
  for (int i = 0; i < kPoseQueueDuration + 2; ++i) {
    current_time += common::FromSeconds(1);
    extrapolator->AddImuData(sensor::ImuData{
        current_time, initial_gravity_acceleration, initial_angular_velocity});
    extrapolator->AddPose(current_time, current_pose);
  }

  Eigen::Vector3d angular_velocity_yaw(0, 0, 0.1);
  transform::Rigid3d expected_pose = current_pose;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 10; ++j) {
      current_time += common::FromSeconds(kGravityTimeConstant);
      extrapolator->AddImuData(sensor::ImuData{
          current_time, initial_gravity_acceleration, angular_velocity_yaw});
      Eigen::Quaterniond expected_rotation(
          Eigen::AngleAxisd(kGravityTimeConstant * angular_velocity_yaw.norm(),
                            angular_velocity_yaw.normalized()));
      expected_pose =
          expected_pose * transform::Rigid3d::Rotation(expected_rotation);
    }
    EXPECT_THAT(extrapolator->ExtrapolatePose(current_time),
                transform::IsNearly(expected_pose, kExtrapolatePrecision));
    extrapolator->AddPose(current_time, expected_pose);
  }
  Eigen::AngleAxisd gravity_axis(
      extrapolator->EstimateGravityOrientation(current_time));
  EXPECT_NEAR(
      0.f, (gravity_axis.axis().normalized() - Eigen::Vector3d::UnitZ()).norm(),
      kPrecision);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
