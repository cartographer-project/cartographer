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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_INTEGRATION_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_INTEGRATION_H_

#include <algorithm>
#include <deque>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/time.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

template <typename T>
struct IntegrateImuResult {
  Eigen::Matrix<T, 3, 1> delta_velocity;
  Eigen::Matrix<T, 3, 1> delta_translation;
  Eigen::Quaternion<T> delta_rotation;
};

template <typename T, typename RangeType, typename IteratorType>
IntegrateImuResult<T> IntegrateImu(
    const RangeType& imu_data,
    const Eigen::Transform<T, 3, Eigen::Affine>&
        linear_acceleration_calibration,
    const Eigen::Transform<T, 3, Eigen::Affine>& angular_velocity_calibration,
    const common::Time start_time, const common::Time end_time,
    IteratorType* const it) {
  CHECK_LE(start_time, end_time);
  CHECK(*it != imu_data.end());
  CHECK_LE((*it)->time, start_time);
  if (std::next(*it) != imu_data.end()) {
    CHECK_GT(std::next(*it)->time, start_time);
  }

  common::Time current_time = start_time;

  IntegrateImuResult<T> result = {Eigen::Matrix<T, 3, 1>::Zero(),
                                  Eigen::Matrix<T, 3, 1>::Zero(),
                                  Eigen::Quaterniond::Identity().cast<T>()};
  while (current_time < end_time) {
    common::Time next_imu_data = common::Time::max();
    if (std::next(*it) != imu_data.end()) {
      next_imu_data = std::next(*it)->time;
    }
    common::Time next_time = std::min(next_imu_data, end_time);
    const T delta_t(common::ToSeconds(next_time - current_time));

    const Eigen::Matrix<T, 3, 1> delta_angle =
        (angular_velocity_calibration *
         (*it)->angular_velocity.template cast<T>()) *
        delta_t;
    result.delta_rotation *=
        transform::AngleAxisVectorToRotationQuaternion(delta_angle);
    result.delta_velocity += result.delta_rotation *
                             ((linear_acceleration_calibration *
                               (*it)->linear_acceleration.template cast<T>()) *
                              delta_t);
    result.delta_translation += delta_t * result.delta_velocity;
    current_time = next_time;
    if (current_time == next_imu_data) {
      ++(*it);
    }
  }
  return result;
}

// Returns velocity delta in map frame.
template <typename RangeType, typename IteratorType>
IntegrateImuResult<double> IntegrateImu(const RangeType& imu_data,
                                        const common::Time start_time,
                                        const common::Time end_time,
                                        IteratorType* const it) {
  return IntegrateImu<double, RangeType, IteratorType>(
      imu_data, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(),
      start_time, end_time, it);
}

template <typename T>
struct ExtrapolatePoseResult {
  transform::Rigid3<T> pose;
  Eigen::Matrix<T, 3, 1> velocity;
};

// Returns pose and linear velocity at 'time' which is equal to
// 'prev_from_tracking' extrapolated using IMU data.
template <typename T, typename RangeType, typename IteratorType>
ExtrapolatePoseResult<T> ExtrapolatePoseWithImu(
    const transform::Rigid3<T>& prev_from_tracking,
    const Eigen::Matrix<T, 3, 1>& prev_velocity_in_tracking,
    const common::Time prev_time, const Eigen::Matrix<T, 3, 1>& gravity,
    const common::Time time, const RangeType& imu_data,
    IteratorType* const imu_it) {
  const IntegrateImuResult<T> result =
      IntegrateImu(imu_data, Eigen::Transform<T, 3, Eigen::Affine>::Identity(),
                   Eigen::Transform<T, 3, Eigen::Affine>::Identity(), prev_time,
                   time, imu_it);

  const T delta_t = static_cast<T>(common::ToSeconds(time - prev_time));
  const Eigen::Matrix<T, 3, 1> translation =
      prev_from_tracking.translation() +
      prev_from_tracking.rotation() *
          (delta_t * prev_velocity_in_tracking + result.delta_translation) -
      static_cast<T>(.5) * delta_t * delta_t * gravity;
  const Eigen::Quaternion<T> rotation =
      prev_from_tracking.rotation() * result.delta_rotation;

  const Eigen::Matrix<T, 3, 1> velocity =
      prev_from_tracking.rotation() *
          (prev_velocity_in_tracking + result.delta_velocity) -
      delta_t * gravity;
  return ExtrapolatePoseResult<T>{transform::Rigid3<T>(translation, rotation),
                                  velocity};
}

// Same as above but given the last two poses.
template <typename T, typename RangeType, typename IteratorType>
ExtrapolatePoseResult<T> ExtrapolatePoseWithImu(
    const transform::Rigid3<T>& prev_from_tracking,
    const common::Time prev_time,
    const transform::Rigid3<T>& prev_prev_from_tracking,
    const common::Time prev_prev_time, const Eigen::Matrix<T, 3, 1>& gravity,
    const common::Time time, const RangeType& imu_data,
    IteratorType* const imu_it) {
  // TODO(danielsievers): Really we should integrate velocities starting from
  // the midpoint in between two poses, since this is how we fit them to poses
  // in the optimization.
  const T prev_delta_t =
      static_cast<T>(common::ToSeconds(prev_time - prev_prev_time));
  const Eigen::Matrix<T, 3, 1> prev_velocity_in_tracking =
      prev_from_tracking.inverse().rotation() *
      (prev_from_tracking.translation() -
       prev_prev_from_tracking.translation()) /
      prev_delta_t;

  return ExtrapolatePoseWithImu(prev_from_tracking, prev_velocity_in_tracking,
                                prev_time, gravity, time, imu_data, imu_it);
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_INTEGRATION_H_
