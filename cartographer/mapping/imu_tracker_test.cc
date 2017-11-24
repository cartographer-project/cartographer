/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/imu_tracker.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

const int kSteps = 10;
const double kDuration = 3.f;
const double kPrecision = 1e-8;

TEST(ImuTrackerTest, IntegrateYawRotation) {
  common::Time time = common::FromUniversal(12345678);
  const Eigen::Vector3d angular_velocity(0, 0, 0.3);
  const Eigen::Vector3d linear_acceleration(0, 0, 9.9);
  ImuTracker imu_tracker(0.1 * kDuration, time);
  EXPECT_NEAR(0., imu_tracker.orientation().angularDistance(Eigen::Quaterniond::Identity()), kPrecision);
  for (int i = 0; i < kSteps; ++i) {
    imu_tracker.AddImuLinearAccelerationObservation(linear_acceleration);
    imu_tracker.AddImuAngularVelocityObservation(angular_velocity);
    time += common::FromSeconds(kDuration / kSteps);
    imu_tracker.Advance(time);
  }
  Eigen::Quaterniond expected_orientation(Eigen::AngleAxisd(
      kDuration * angular_velocity.norm(), angular_velocity.normalized()));
  EXPECT_NEAR(0., imu_tracker.orientation().angularDistance(expected_orientation), kPrecision);
  EXPECT_EQ(time, imu_tracker.time());
}

TEST(ImuTrackerTest, IntegrateFullRotation) {
  common::Time time = common::FromUniversal(12345678);
  const Eigen::Vector3d angular_velocity(0.1, 0.4, 0.1);
  const Eigen::Vector3d linear_acceleration(0, 0, 9.9);
  // Effectively disables gravity vector tracking.
  ImuTracker imu_tracker(1e10 * kDuration, time);
  EXPECT_NEAR(0., imu_tracker.orientation().angularDistance(Eigen::Quaterniond::Identity()), kPrecision);
  for (int i = 0; i < kSteps; ++i) {
    imu_tracker.AddImuLinearAccelerationObservation(linear_acceleration);
    imu_tracker.AddImuAngularVelocityObservation(angular_velocity);
    time += common::FromSeconds(kDuration / kSteps);
    imu_tracker.Advance(time);
  }
  Eigen::Quaterniond expected_orientation(Eigen::AngleAxisd(
      kDuration * angular_velocity.norm(), angular_velocity.normalized()));
  EXPECT_NEAR(0., imu_tracker.orientation().angularDistance(expected_orientation), kPrecision);
  EXPECT_EQ(time, imu_tracker.time());
}

TEST(ImuTrackerTest, LearnGravityVector) {
  common::Time time = common::FromUniversal(12345678);
  const Eigen::Vector3d linear_acceleration(0.5, 0.3, 9.5);
  ImuTracker imu_tracker(0.1 * kDuration, time);
  EXPECT_NEAR(0., imu_tracker.orientation().angularDistance(Eigen::Quaterniond::Identity()), kPrecision);
  for (int i = 0; i < kSteps; ++i) {
    imu_tracker.AddImuLinearAccelerationObservation(linear_acceleration);
    time += common::FromSeconds(kDuration / kSteps);
    imu_tracker.Advance(time);
  }
  Eigen::Quaterniond expected_orientation;
  expected_orientation.setFromTwoVectors(linear_acceleration, Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(0., imu_tracker.orientation().angularDistance(expected_orientation), kPrecision);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
