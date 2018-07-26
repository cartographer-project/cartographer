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

#include "absl/memory/memory.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr double kDuration = 3.f;
constexpr double kGravityTimeConstant = 0.1 * kDuration;
constexpr double kPrecision = 1e-8;
constexpr int kSteps = 10;

class ImuTrackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    imu_tracker_ = absl::make_unique<ImuTracker>(kGravityTimeConstant, time_);
    angular_velocity_ = Eigen::Vector3d(0, 0, 0);
    linear_acceleration_ = Eigen::Vector3d(0, 0, 9.9);
    EXPECT_NEAR(0.,
                imu_tracker_->orientation().angularDistance(
                    Eigen::Quaterniond::Identity()),
                kPrecision);
  }

  void AdvanceImu() {
    for (int i = 0; i < kSteps; ++i) {
      imu_tracker_->AddImuLinearAccelerationObservation(linear_acceleration_);
      imu_tracker_->AddImuAngularVelocityObservation(angular_velocity_);
      time_ += common::FromSeconds(kDuration / kSteps);
      imu_tracker_->Advance(time_);
    }
    EXPECT_EQ(time_, imu_tracker_->time());
  }

  Eigen::Vector3d angular_velocity_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  Eigen::Vector3d linear_acceleration_;
  common::Time time_ = common::FromUniversal(12345678);
};

TEST_F(ImuTrackerTest, IntegrateYawRotation) {
  angular_velocity_ = Eigen::Vector3d(0, 0, 0.3);
  AdvanceImu();
  Eigen::Quaterniond expected_orientation(Eigen::AngleAxisd(
      kDuration * angular_velocity_.norm(), angular_velocity_.normalized()));
  EXPECT_NEAR(0.,
              imu_tracker_->orientation().angularDistance(expected_orientation),
              kPrecision);
}

TEST_F(ImuTrackerTest, IntegrateFullRotation) {
  angular_velocity_ = Eigen::Vector3d(0.1, 0.4, 0.1);
  // Using a huge gravity time constant effectively disables gravity vector
  // tracking.
  imu_tracker_.reset(new ImuTracker(1e10 * kDuration, time_));
  AdvanceImu();
  Eigen::Quaterniond expected_orientation(Eigen::AngleAxisd(
      kDuration * angular_velocity_.norm(), angular_velocity_.normalized()));
  EXPECT_NEAR(0.,
              imu_tracker_->orientation().angularDistance(expected_orientation),
              kPrecision);
}

TEST_F(ImuTrackerTest, LearnGravityVector) {
  linear_acceleration_ = Eigen::Vector3d(0.5, 0.3, 9.5);
  AdvanceImu();
  Eigen::Quaterniond expected_orientation;
  expected_orientation.setFromTwoVectors(linear_acceleration_,
                                         Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(0.,
              imu_tracker_->orientation().angularDistance(expected_orientation),
              kPrecision);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
