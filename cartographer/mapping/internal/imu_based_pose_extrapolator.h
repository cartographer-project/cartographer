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

#ifndef CARTOGRAPHER_MAPPING_IMU_BASED_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_IMU_BASED_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>
#include <vector>

#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/histogram.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/transform/timestamped_transform.h"

namespace cartographer {
namespace mapping {

// Uses the linear acceleration and rotational velocities to estimate a pose.
class ImuBasedPoseExtrapolator : public PoseExtrapolatorInterface {
 public:
  explicit ImuBasedPoseExtrapolator(
      const proto::ImuBasedPoseExtrapolatorOptions& options);
  ~ImuBasedPoseExtrapolator() override;

  static std::unique_ptr<PoseExtrapolatorInterface> InitializeWithImu(
      const proto::ImuBasedPoseExtrapolatorOptions& options,
      const std::vector<sensor::ImuData>& imu_data,
      const std::vector<transform::TimestampedTransform>& initial_poses);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;

  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  void AddImuData(const sensor::ImuData& imu_data) override;
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;

  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) override;

  // Gravity alignment estimate.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

 private:
  template <typename T>
  void TrimDequeData(std::deque<T>* data);

  void TrimImuData();
  void TrimOdometryData();

  // Odometry methods.
  bool HasOdometryData() const;
  bool HasOdometryDataForTime(const common::Time& first_time) const;
  transform::Rigid3d InterpolateOdometry(const common::Time& first_time) const;
  transform::Rigid3d CalculateOdometryBetweenNodes(
      const transform::Rigid3d& first_node_odometry,
      const transform::Rigid3d& second_node_odometry) const;

  std::vector<transform::Rigid3f> InterpolatePoses(
      const ::cartographer::transform::TimestampedTransform& start,
      const ::cartographer::transform::TimestampedTransform& end,
      const std::vector<common::Time>::const_iterator times_begin,
      const std::vector<common::Time>::const_iterator times_end);

  std::deque<::cartographer::transform::TimestampedTransform> timed_pose_queue_;
  std::deque<::cartographer::transform::TimestampedTransform>
      previous_solution_;

  std::deque<sensor::ImuData> imu_data_;
  std::deque<sensor::OdometryData> odometry_data_;
  common::Time last_extrapolated_time_ = common::Time::min();

  transform::Rigid3d gravity_from_local_ = transform::Rigid3d::Identity();

  const proto::ImuBasedPoseExtrapolatorOptions options_;
  const ceres::Solver::Options solver_options_;

  common::Histogram num_iterations_hist_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_BASED_POSE_EXTRAPOLATOR_H_
