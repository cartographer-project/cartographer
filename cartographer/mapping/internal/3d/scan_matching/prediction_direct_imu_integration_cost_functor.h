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

#ifndef CARTOGRAPHER_MAPPING_3D_PREDICTION_DIRECT_IMU_INTEGRATION_H_
#define CARTOGRAPHER_MAPPING_3D_PREDICTION_DIRECT_IMU_INTEGRATION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/internal/3d/imu_integration.h"

namespace cartographer {
namespace mapping {

class PredictionDirectImuIntegrationCostFunctor {
 public:
  explicit PredictionDirectImuIntegrationCostFunctor(
      const double translation_scaling_factor,
      const double velocity_scaling_factor,
      const double rotation_scaling_factor, const double delta_time_seconds,
      const std::deque<sensor::ImuData>& imu_data,
      const Eigen::Transform<double, 3, Eigen::Affine>&
          linear_acceleration_calibration,
      const Eigen::Transform<double, 3, Eigen::Affine>&
          angular_velocity_calibration,
      const common::Time start_time, const common::Time end_time,
      const proto::IMUIntegrator& integrator_type)
      : translation_scaling_factor_(translation_scaling_factor),
        velocity_scaling_factor_(velocity_scaling_factor),
        rotation_scaling_factor_(rotation_scaling_factor),
        delta_time_seconds_(delta_time_seconds),
        imu_data_(imu_data),
        linear_acceleration_calibration_(linear_acceleration_calibration),
        angular_velocity_calibration_(angular_velocity_calibration),
        start_time_(start_time),
        end_time_(end_time),
        integrator_type_(integrator_type) {}

  PredictionDirectImuIntegrationCostFunctor(
      const PredictionDirectImuIntegrationCostFunctor&) = delete;
  PredictionDirectImuIntegrationCostFunctor& operator=(
      const PredictionDirectImuIntegrationCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const old_translation, const T* const old_velocity,
                  const T* const old_rotation, const T* const new_translation,
                  const T* const new_velocity, const T* const new_rotation,
                  T* residual) const {
    const Eigen::Matrix<T, 3, 1> start_translation(
        old_translation[0], old_translation[1], old_translation[2]);
    const Eigen::Matrix<T, 3, 1> end_translation(
        new_translation[0], new_translation[1], new_translation[2]);
    const Eigen::Matrix<T, 3, 1> start_velocity(
        old_velocity[0], old_velocity[1], old_velocity[2]);
    const Eigen::Matrix<T, 3, 1> end_velocity(new_velocity[0], new_velocity[1],
                                              new_velocity[2]);

    const Eigen::Quaternion<T> start_rotation(old_rotation[0], old_rotation[1],
                                              old_rotation[2], old_rotation[3]);
    const Eigen::Quaternion<T> end_rotation(new_rotation[0], new_rotation[1],
                                            new_rotation[2], new_rotation[3]);

    auto it = --imu_data_.cend();
    while (it->time > start_time_) {
      CHECK(it != imu_data_.cbegin());
      --it;
    }

    const std::array<T, 3> start_translation_arr{
        {old_translation[0], old_translation[1], old_translation[2]}};
    const std::array<T, 4> start_rotation_arr{
        {old_rotation[0], old_rotation[1], old_rotation[2], old_rotation[3]}};
    const std::array<T, 3> start_velocity_arr{
        {old_velocity[0], old_velocity[1], old_velocity[2]}};
    IntegrateImuWithTranslationResult<T> imu_integral =
        IntegrateImuRK4(imu_data_, linear_acceleration_calibration_,
                        angular_velocity_calibration_, start_translation_arr,
                        start_rotation_arr, start_velocity_arr, start_time_,
                        end_time_, 9.80665, &it);

    const Eigen::Matrix<T, 3, 1> predicted_translation =
        imu_integral.delta_translation;

    const Eigen::Matrix<T, 3, 1> predicted_velocity =
        imu_integral.delta_velocity;

    residual[0] = translation_scaling_factor_ *
                  (new_translation[0] - predicted_translation[0]);
    residual[1] = translation_scaling_factor_ *
                  (new_translation[1] - predicted_translation[1]);
    residual[2] = translation_scaling_factor_ *
                  (new_translation[2] - predicted_translation[2]);

    residual[3] =
        velocity_scaling_factor_ * (new_velocity[0] - predicted_velocity[0]);
    residual[4] =
        velocity_scaling_factor_ * (new_velocity[1] - predicted_velocity[1]);
    residual[5] =
        velocity_scaling_factor_ * (new_velocity[2] - predicted_velocity[2]);

    const Eigen::Quaternion<T> rotation_error =
        end_rotation.conjugate() * imu_integral.delta_rotation;
    residual[6] = rotation_scaling_factor_ * rotation_error.x();
    residual[7] = rotation_scaling_factor_ * rotation_error.y();
    residual[8] = rotation_scaling_factor_ * rotation_error.z();

    return true;
  }

 private:
  const double translation_scaling_factor_;
  const double velocity_scaling_factor_;
  const double rotation_scaling_factor_;
  const double delta_time_seconds_;
  const std::deque<sensor::ImuData>& imu_data_;
  const Eigen::Transform<double, 3, Eigen::Affine>&
      linear_acceleration_calibration_;
  const Eigen::Transform<double, 3, Eigen::Affine>&
      angular_velocity_calibration_;
  const common::Time start_time_;
  const common::Time end_time_;
  const proto::IMUIntegrator& integrator_type_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_PREDICTION_DIRECT_IMU_INTEGRATION_H_