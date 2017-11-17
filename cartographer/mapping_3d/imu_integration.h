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

#ifndef CARTOGRAPHER_MAPPING_3D_IMU_INTEGRATION_H_
#define CARTOGRAPHER_MAPPING_3D_IMU_INTEGRATION_H_

#include <algorithm>
#include <deque>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/time.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

template <typename T>
struct IntegrateImuResult {
  Eigen::Matrix<T, 3, 1> delta_velocity;
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

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_IMU_INTEGRATION_H_
