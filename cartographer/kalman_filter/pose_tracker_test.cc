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

#include "cartographer/kalman_filter/pose_tracker.h"

#include <random>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace kalman_filter {
namespace {

constexpr double kOdometerVariance = 1e-12;

using transform::IsNearly;
using transform::Rigid3d;
using ::testing::Not;

class PoseTrackerTest : public ::testing::Test {
 protected:
  PoseTrackerTest() {
    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
            orientation_model_variance = 1e-8,
            position_model_variance = 1e-8,
            velocity_model_variance = 1e-8,
            imu_gravity_time_constant = 100.,
            imu_gravity_variance = 1e-9,
            num_odometry_states = 1,
        }
        )text");
    const proto::PoseTrackerOptions options =
        CreatePoseTrackerOptions(parameter_dictionary.get());
    pose_tracker_ = common::make_unique<PoseTracker>(
        options, PoseTracker::ModelFunction::k3D, common::FromUniversal(1000));
  }

  std::unique_ptr<PoseTracker> pose_tracker_;
};

TEST(CovarianceTest, EmbedAndProjectCovariance) {
  std::mt19937 prng(42);
  std::uniform_real_distribution<float> distribution(-10.f, 10.f);
  for (int i = 0; i < 100; ++i) {
    Pose2DCovariance covariance;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance(row, column) = distribution(prng);
      }
    }
    const PoseCovariance embedded_covariance =
        Embed3D(covariance, distribution(prng), distribution(prng));
    EXPECT_TRUE(
        (Project2D(embedded_covariance).array() == covariance.array()).all());
  }
}

TEST_F(PoseTrackerTest, SaveAndRestore) {
  std::vector<Rigid3d> poses(3);
  std::vector<PoseCovariance> covariances(3);
  pose_tracker_->GetPoseEstimateMeanAndCovariance(common::FromUniversal(1500),
                                                  &poses[0], &covariances[0]);

  pose_tracker_->AddImuLinearAccelerationObservation(
      common::FromUniversal(2000), Eigen::Vector3d(1, 1, 9));

  PoseTracker copy_of_pose_tracker = *pose_tracker_;

  const Eigen::Vector3d observation(2, 0, 8);
  pose_tracker_->AddImuLinearAccelerationObservation(
      common::FromUniversal(3000), observation);

  pose_tracker_->GetPoseEstimateMeanAndCovariance(common::FromUniversal(3500),
                                                  &poses[1], &covariances[1]);

  copy_of_pose_tracker.AddImuLinearAccelerationObservation(
      common::FromUniversal(3000), observation);
  copy_of_pose_tracker.GetPoseEstimateMeanAndCovariance(
      common::FromUniversal(3500), &poses[2], &covariances[2]);

  EXPECT_THAT(poses[0], Not(IsNearly(poses[1], 1e-6)));
  EXPECT_FALSE((covariances[0].array() == covariances[1].array()).all());
  EXPECT_THAT(poses[1], IsNearly(poses[2], 1e-6));
  EXPECT_TRUE((covariances[1].array() == covariances[2].array()).all());
}

TEST_F(PoseTrackerTest, AddImuLinearAccelerationObservation) {
  auto time = common::FromUniversal(1000);

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::seconds(5);
    pose_tracker_->AddImuLinearAccelerationObservation(
        time, Eigen::Vector3d(0., 0., 10.));
  }

  {
    Rigid3d pose;
    PoseCovariance covariance;
    pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose, &covariance);
    const Eigen::Quaterniond actual = Eigen::Quaterniond(pose.rotation());
    const Eigen::Quaterniond expected = Eigen::Quaterniond::Identity();
    EXPECT_TRUE(actual.isApprox(expected, 1e-3)) << expected.coeffs() << " vs\n"
                                                 << actual.coeffs();
  }

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::seconds(5);
    pose_tracker_->AddImuLinearAccelerationObservation(
        time, Eigen::Vector3d(0., 10., 0.));
  }

  time += std::chrono::milliseconds(5);

  Rigid3d pose;
  PoseCovariance covariance;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose, &covariance);
  const Eigen::Quaterniond actual = Eigen::Quaterniond(pose.rotation());
  const Eigen::Quaterniond expected = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(actual.isApprox(expected, 1e-3)) << expected.coeffs() << " vs\n"
                                               << actual.coeffs();
}

TEST_F(PoseTrackerTest, AddImuAngularVelocityObservation) {
  auto time = common::FromUniversal(1000);

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::milliseconds(5);
    pose_tracker_->AddImuAngularVelocityObservation(time,
                                                    Eigen::Vector3d::Zero());
  }

  {
    Rigid3d pose;
    PoseCovariance covariance;
    pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose, &covariance);
    const Eigen::Quaterniond actual = Eigen::Quaterniond(pose.rotation());
    const Eigen::Quaterniond expected = Eigen::Quaterniond::Identity();
    EXPECT_TRUE(actual.isApprox(expected, 1e-3)) << expected.coeffs() << " vs\n"
                                                 << actual.coeffs();
  }

  const double target_radians = M_PI / 2.;
  const double num_observations = 300.;
  const double angular_velocity = target_radians / (num_observations * 5e-3);
  for (int i = 0; i < num_observations; ++i) {
    time += std::chrono::milliseconds(5);
    pose_tracker_->AddImuAngularVelocityObservation(
        time, Eigen::Vector3d(angular_velocity, 0., 0.));
  }

  time += std::chrono::milliseconds(5);

  Rigid3d pose;
  PoseCovariance covariance;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose, &covariance);
  const Eigen::Quaterniond actual = Eigen::Quaterniond(pose.rotation());
  const Eigen::Quaterniond expected = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(actual.isApprox(expected, 1e-3)) << expected.coeffs() << " vs\n"
                                               << actual.coeffs();
}

TEST_F(PoseTrackerTest, AddPoseObservation) {
  auto time = common::FromUniversal(1000);

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::milliseconds(5);
    pose_tracker_->AddPoseObservation(
        time, Rigid3d::Identity(),
        Eigen::Matrix<double, 6, 6>::Identity() * 1e-6);
  }

  {
    Rigid3d actual;
    PoseCovariance covariance;
    pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &actual, &covariance);
    EXPECT_THAT(actual, IsNearly(Rigid3d::Identity(), 1e-3));
  }

  const Rigid3d expected =
      Rigid3d::Translation(Eigen::Vector3d(1., 2., 3.)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(
          M_PI / 2., Eigen::Vector3d(0., 0., 3.).normalized()));

  for (int i = 0; i < 300; ++i) {
    time += std::chrono::milliseconds(15);
    pose_tracker_->AddPoseObservation(
        time, expected, Eigen::Matrix<double, 6, 6>::Identity() * 1e-9);
  }

  time += std::chrono::milliseconds(15);

  Rigid3d actual;
  PoseCovariance covariance;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &actual, &covariance);
  EXPECT_THAT(actual, IsNearly(expected, 1e-3));
}

TEST_F(PoseTrackerTest, AddOdometerPoseObservation) {
  common::Time time = common::FromUniversal(0);

  std::vector<Rigid3d> odometer_track;
  odometer_track.push_back(Rigid3d::Identity());
  odometer_track.push_back(
      Rigid3d::Rotation(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(
      Rigid3d::Translation(Eigen::Vector3d(0.2, 0., 0.)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(
      Rigid3d::Translation(Eigen::Vector3d(0.3, 0.1, 0.)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(
      Rigid3d::Translation(Eigen::Vector3d(0.2, 0.2, 0.1)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(
      Rigid3d::Translation(Eigen::Vector3d(0.1, 0.2, 0.2)) *
      Rigid3d::Rotation(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())));
  odometer_track.push_back(Rigid3d::Translation(Eigen::Vector3d(0., 0.1, 0.2)));

  Rigid3d actual;
  PoseCovariance unused_covariance;
  for (const Rigid3d& pose : odometer_track) {
    time += std::chrono::seconds(1);
    pose_tracker_->AddOdometerPoseObservation(
        time, pose, kOdometerVariance * PoseCovariance::Identity());
    pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &actual,
                                                    &unused_covariance);
    EXPECT_THAT(actual, IsNearly(pose, 1e-2));
  }
  // Sanity check that the test has signal:
  EXPECT_THAT(actual, Not(IsNearly(odometer_track[0], 1e-2)));
}

}  // namespace
}  // namespace kalman_filter
}  // namespace cartographer
