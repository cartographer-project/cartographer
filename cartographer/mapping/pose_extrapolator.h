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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
class PoseExtrapolator {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const;
  common::Time GetLastExtrapolatedTime() const;

  void AddPose(common::Time time, const transform::Rigid3d& pose);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time);

  // Gravity alignment estimate.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  const common::Duration pose_queue_duration_;
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
