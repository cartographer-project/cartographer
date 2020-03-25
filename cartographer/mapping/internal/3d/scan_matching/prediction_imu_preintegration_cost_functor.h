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

#ifndef CARTOGRAPHER_MAPPING_3D_PREDICTION_IMU_PREINTEGRATION_H_
#define CARTOGRAPHER_MAPPING_3D_PREDICTION_IMU_PREINTEGRATION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/internal/3d/imu_integration.h"

namespace cartographer {
namespace mapping {

class PredictionImuPreintegrationCostFunctor {
 public:
  explicit PredictionImuPreintegrationCostFunctor(
      const double translation_scaling_factor,
      const double velocity_scaling_factor,
      const double rotation_scaling_factor, const double delta_time_seconds,
      const IntegrateImuWithTranslationResult<double>&
          imu_preintegration_result)
      : translation_scaling_factor_(translation_scaling_factor),
        velocity_scaling_factor_(velocity_scaling_factor),
        rotation_scaling_factor_(rotation_scaling_factor),
        delta_time_seconds_(delta_time_seconds),
        gravity_constant_(9.80665),
        imu_preintegration_result_(imu_preintegration_result) {}

  PredictionImuPreintegrationCostFunctor(
      const PredictionImuPreintegrationCostFunctor&) = delete;
  PredictionImuPreintegrationCostFunctor& operator=(
      const PredictionImuPreintegrationCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const old_translation, const T* const old_velocity,
                  const T* const old_rotation, const T* const new_translation,
                  const T* const new_velocity, const T* const new_rotation,
                  T* residual) const {
    const Eigen::Matrix<T, 3, 1> start_translation(
        old_translation[0], old_translation[1], old_translation[2]);
    const Eigen::Matrix<T, 3, 1> end_translation(
        new_translation[0], new_translation[1], new_translation[2]);
    const Eigen::Quaternion<T> start_rotation(old_rotation[0], old_rotation[1],
                                              old_rotation[2], old_rotation[3]);
    const Eigen::Quaternion<T> end_rotation(new_rotation[0], new_rotation[1],
                                            new_rotation[2], new_rotation[3]);
    const Eigen::Matrix<T, 3, 1> start_velocity(
        old_velocity[0], old_velocity[1], old_velocity[2]);
    const Eigen::Matrix<T, 3, 1> end_velocity(new_velocity[0], new_velocity[1],
                                              new_velocity[2]);

    const Eigen::Matrix<T, 3, 1> translation_error =
        start_rotation *
            (end_translation - start_translation +
             (0.5 * gravity_constant_ * std::pow(delta_time_seconds_, 2) *
              Eigen::Vector3d::UnitZ())
                 .cast<T>()) -
        imu_preintegration_result_.delta_translation.cast<T>() -
        T(delta_time_seconds_) * start_velocity;
    const Eigen::Matrix<T, 3, 1> velocity_error =
        start_rotation * (end_velocity - start_velocity +
                          (gravity_constant_ * delta_time_seconds_ *
                           Eigen::Vector3d::UnitZ())
                              .cast<T>()) -
        imu_preintegration_result_.delta_velocity.cast<T>();
    const Eigen::Quaternion<T> rotation_error =
        imu_preintegration_result_.delta_rotation.cast<T>() *
        end_rotation.conjugate() * start_rotation;

    residual[0] = translation_scaling_factor_ * translation_error.x();
    residual[1] = translation_scaling_factor_ * translation_error.y();
    residual[2] = translation_scaling_factor_ * translation_error.z();
    residual[3] = velocity_scaling_factor_ * velocity_error.x();
    residual[4] = velocity_scaling_factor_ * velocity_error.y();
    residual[5] = velocity_scaling_factor_ * velocity_error.z();
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
  const double gravity_constant_;
  const IntegrateImuWithTranslationResult<double> imu_preintegration_result_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_PREDICTION_IMU_PREINTEGRATION_H_