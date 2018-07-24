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

#ifndef CARTOGRAPHER_ACCELERATION_COST_3D_H_
#define CARTOGRAPHER_ACCELERATION_COST_3D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/pose_graph/proto/cost_function.pb.h"

namespace cartographer {
namespace pose_graph {

// Penalizes differences between IMU data and optimized accelerations.
class AccelerationCost3D {
 public:
  explicit AccelerationCost3D(
      const proto::Acceleration3D::Parameters& parameters);

  proto::Acceleration3D::Parameters ToProto() const;

  template <typename T>
  bool operator()(const T* const middle_rotation, const T* const start_position,
                  const T* const middle_position, const T* const end_position,
                  const T* const gravity_constant,
                  const T* const imu_calibration, T* residual) const {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using TranslationMap = Eigen::Map<const Vector3T>;
    using RotationMap = Eigen::Map<const Eigen::Quaternion<T>>;

    const Vector3T imu_delta_velocity =
        RotationMap(middle_rotation) * RotationMap(imu_calibration) *
            delta_velocity_imu_frame_.cast<T>() -
        (*gravity_constant * 0.5 *
         (first_to_second_delta_time_seconds_ +
          second_to_third_delta_time_seconds_)) *
            Eigen::Vector3d::UnitZ().cast<T>();

    const Vector3T start_velocity =
        (TranslationMap(middle_position) - TranslationMap(start_position)) /
        T(first_to_second_delta_time_seconds_);
    const Vector3T end_velocity =
        (TranslationMap(end_position) - TranslationMap(middle_position)) /
        T(second_to_third_delta_time_seconds_);
    const Vector3T delta_velocity = end_velocity - start_velocity;

    (Eigen::Map<Vector3T>(residual) =
         T(scaling_factor_) * (imu_delta_velocity - delta_velocity));
    return true;
  }

 private:
  const Eigen::Vector3d delta_velocity_imu_frame_;
  const double first_to_second_delta_time_seconds_;
  const double second_to_third_delta_time_seconds_;
  const double scaling_factor_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_ACCELERATION_COST_3D_H_
