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
    using ::Eigen::Map;
    using ::Eigen::Matrix;

    const Matrix<T, 3, 1> imu_delta_velocity =
        Map<const Eigen::Quaternion<T>>(middle_rotation) *
            Map<const Eigen::Quaternion<T>>(imu_calibration) *
            delta_velocity_imu_frame_.cast<T>() -
        ((first_to_second_delta_time_seconds_ +
          second_to_third_delta_time_seconds_) *
         *gravity_constant / 2.f) *
            Eigen::Vector3d::UnitZ().cast<T>();
    const Matrix<T, 3, 1> start_velocity =
        (Map<const Matrix<T, 3, 1>>(middle_position) -
         Map<const Matrix<T, 3, 1>>(start_position)) /
        T(first_to_second_delta_time_seconds_);
    const Matrix<T, 3, 1> end_velocity =
        (Map<const Matrix<T, 3, 1>>(end_position) -
         Map<const Matrix<T, 3, 1>>(middle_position)) /
        T(second_to_third_delta_time_seconds_);
    const Matrix<T, 3, 1> delta_velocity = end_velocity - start_velocity;

    (Map<Matrix<T, 3, 1>>(residual) =
         scaling_factor_ * (imu_delta_velocity - delta_velocity));
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
