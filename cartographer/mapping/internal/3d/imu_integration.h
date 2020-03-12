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
#include "imu-integrator/imu-integrator.h"

#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/optimizing_local_trajectory_builder.h"
#include "cartographer/mapping/proto/3d/optimizing_local_trajectory_builder_options.pb.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

template <typename T>
struct IntegrateImuResult {
  Eigen::Matrix<T, 3, 1> delta_velocity;
  Eigen::Quaternion<T> delta_rotation;
};

template <typename T>
struct IntegrateImuWithTranslationResult {
  Eigen::Matrix<T, 3, 1> delta_translation;
  Eigen::Quaternion<T> delta_rotation;
  Eigen::Matrix<T, 3, 1> delta_velocity;
};

template <typename T, typename RangeType, typename IteratorType>
IntegrateImuResult<T> IntegrateImuEuler(
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
IntegrateImuResult<double> IntegrateImuEuler(const RangeType& imu_data,
                                             const common::Time start_time,
                                             const common::Time end_time,
                                             IteratorType* const it) {
  return IntegrateImuEuler<double, RangeType, IteratorType>(
      imu_data, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(),
      start_time, end_time, it);
}

template <typename T, typename RangeType, typename IteratorType>
IntegrateImuWithTranslationResult<T> IntegrateImuWithTranslationEuler(
    const RangeType& imu_data,
    const Eigen::Transform<double, 3, Eigen::Affine>&
        linear_acceleration_calibration,
    const Eigen::Transform<double, 3, Eigen::Affine>&
        angular_velocity_calibration,
    const common::Time start_time, const common::Time end_time,
    IteratorType* const it) {
  CHECK_LE(start_time, end_time);
  CHECK(*it != imu_data.end());
  CHECK_LE((*it)->time, start_time);
  if (std::next(*it) != imu_data.end()) {
    CHECK_GT(std::next(*it)->time, start_time);
  }

  common::Time current_time = start_time;

  IntegrateImuWithTranslationResult<T> result = {
      Eigen::Matrix<T, 3, 1>::Zero(), Eigen::Quaterniond::Identity().cast<T>(),
      Eigen::Matrix<T, 3, 1>::Zero()};
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
    result.delta_translation += result.delta_velocity * delta_t;
    current_time = next_time;
    if (current_time == next_imu_data) {
      ++(*it);
    }
  }
  return result;
}

// Returns velocity delta in map frame.
template <typename RangeType, typename IteratorType>
IntegrateImuWithTranslationResult<double> IntegrateImuWithTranslationEuler(
    const RangeType& imu_data, const common::Time start_time,
    const common::Time end_time, IteratorType* const it) {
  return IntegrateImuWithTranslationEuler<double, RangeType, IteratorType>(
      imu_data, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(),
      start_time, end_time, it);
}

template <typename T, typename RangeType, typename IteratorType>
IntegrateImuWithTranslationResult<T> IntegrateImuWithTranslationRK4(
    const RangeType& imu_data,
    const Eigen::Transform<double, 3, Eigen::Affine>&
        linear_acceleration_calibration,
    const Eigen::Transform<double, 3, Eigen::Affine>&
        angular_velocity_calibration,
    const std::array<T, 3>& start_translation,
    const std::array<T, 4>& start_rotation,
    const std::array<T, 3>& start_velocity, const common::Time start_time,
    const common::Time end_time, IteratorType* const it) {
  CHECK_LE(start_time, end_time);
  CHECK(*it != imu_data.end());
  CHECK_LE((*it)->time, start_time);
  CHECK_LE(end_time, imu_data.back().time);
  if (std::next(*it) != imu_data.end()) {
    CHECK_GE(std::next(*it)->time, start_time);
  }
  // TODO(kdaun) add extrapolation

  common::Time current_time = start_time;
  imu_integrator::ImuIntegratorRK4 integrator(0.0, 0.0, 0.0, 0.0, 9.80665);
  Eigen::Matrix<T, imu_integrator::kStateSize, 1> current_state;
  Eigen::Matrix<T, imu_integrator::kStateSize, 1> next_state;
  Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>
      debiased_imu_readings;
  debiased_imu_readings.setZero();
  current_state.setZero();
  current_state.template block<imu_integrator::kStateOrientationBlockSize, 1>(
      imu_integrator::kStateOrientationOffset, 0) =
      Eigen::Matrix<T, imu_integrator::kStateOrientationBlockSize, 1>(
          start_rotation[1], start_rotation[2], start_rotation[3],
          start_rotation[0]);
  current_state.template block<imu_integrator::kPositionBlockSize, 1>(
      imu_integrator::kStatePositionOffset, 0) =
      Eigen::Matrix<T, imu_integrator::kPositionBlockSize, 1>(
          start_translation.data());
  current_state.template block<imu_integrator::kVelocityBlockSize, 1>(
      imu_integrator::kStateVelocityOffset, 0) =
      Eigen::Matrix<T, imu_integrator::kVelocityBlockSize, 1>(
          start_velocity.data());

  // Interpolate IMU reading for start_time
  Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>
      interpolation_imu_readings;
  interpolation_imu_readings.block<3, 1>(imu_integrator::kAccelReadingOffset,
                                         0) =
      linear_acceleration_calibration * (*it)->linear_acceleration;

  interpolation_imu_readings.block<3, 1>(imu_integrator::kGyroReadingOffset,
                                         0) =
      angular_velocity_calibration * (*it)->angular_velocity;
  interpolation_imu_readings.block<3, 1>(
      imu_integrator::kImuReadingSize + imu_integrator::kAccelReadingOffset,
      0) =
      linear_acceleration_calibration * std::next(*it)->linear_acceleration;
  interpolation_imu_readings.block<3, 1>(
      imu_integrator::kImuReadingSize + imu_integrator::kGyroReadingOffset, 0) =
      angular_velocity_calibration * std::next(*it)->angular_velocity;
  Eigen::Matrix<double, imu_integrator::kImuReadingSize, 1>
      interpolated_imu_readings;
  CHECK_GE(common::ToSeconds(std::next(*it)->time - (*it)->time),
           common::ToSeconds(current_time - (*it)->time));
  integrator.interpolateImuReadings(
      interpolation_imu_readings,
      common::ToSeconds(std::next(*it)->time - (*it)->time),
      common::ToSeconds(current_time - (*it)->time),
      &interpolated_imu_readings);

  debiased_imu_readings.block<6, 1>(imu_integrator::kImuReadingSize, 0) =
      interpolated_imu_readings.block<6, 1>(imu_integrator::kAccelReadingOffset,
                                            0);

  while (current_time < end_time) {
    // Shift next IMU reading to current block
    debiased_imu_readings.template block<6, 1>(0, 0) =
        debiased_imu_readings.template block<6, 1>(
            imu_integrator::kImuReadingSize, 0);

    if (std::next(*it)->time < end_time) {
      debiased_imu_readings.block<3, 1>(
          imu_integrator::kImuReadingSize + imu_integrator::kAccelReadingOffset,
          0) =
          linear_acceleration_calibration * std::next(*it)->linear_acceleration;
      debiased_imu_readings.block<3, 1>(
          imu_integrator::kImuReadingSize + imu_integrator::kGyroReadingOffset,
          0) = angular_velocity_calibration * std::next(*it)->angular_velocity;
      Eigen::Matrix<T, 2 * imu_integrator::kImuReadingSize, 1>
          debiased_imu_readingsT = debiased_imu_readings.cast<T>();
      T dt = T(common::ToSeconds(std::next(*it)->time - current_time));
      integrator.integrateStateOnly(current_state, debiased_imu_readingsT, dt,
                                    &next_state);
    } else {
      // Interpolate IMU reading for end_time
      interpolation_imu_readings.block<3, 1>(
          imu_integrator::kAccelReadingOffset, 0) =
          linear_acceleration_calibration * (*it)->linear_acceleration;
      interpolation_imu_readings.block<3, 1>(imu_integrator::kGyroReadingOffset,
                                             0) =
          angular_velocity_calibration * (*it)->angular_velocity;
      interpolation_imu_readings.block<3, 1>(
          imu_integrator::kImuReadingSize + imu_integrator::kAccelReadingOffset,
          0) =
          linear_acceleration_calibration * std::next(*it)->linear_acceleration;
      interpolation_imu_readings.block<3, 1>(
          imu_integrator::kImuReadingSize + imu_integrator::kGyroReadingOffset,
          0) = angular_velocity_calibration * std::next(*it)->angular_velocity;
      CHECK_GE(common::ToSeconds(std::next(*it)->time - (*it)->time),
               common::ToSeconds(end_time - (*it)->time));
      integrator.interpolateImuReadings(
          interpolation_imu_readings,
          common::ToSeconds(std::next(*it)->time - (*it)->time),
          common::ToSeconds(end_time - (*it)->time),
          &interpolated_imu_readings);
      debiased_imu_readings.block<6, 1>(imu_integrator::kImuReadingSize, 0) =
          interpolated_imu_readings.block<6, 1>(0, 0);
      Eigen::Matrix<T, 2 * imu_integrator::kImuReadingSize, 1>
          debiased_imu_readingsT = debiased_imu_readings.cast<T>();
      T dt = T(common::ToSeconds(end_time - current_time));
      integrator.integrateStateOnly(current_state, debiased_imu_readingsT, dt,
                                    &next_state);
    }
    current_state = next_state;
    current_time = std::next(*it)->time;
    if (current_time < end_time) ++(*it);
  }

  IntegrateImuWithTranslationResult<T> result = {
      current_state.template block<imu_integrator::kPositionBlockSize, 1>(
          imu_integrator::kStatePositionOffset, 0),
      Eigen::Quaternion<T>(
          current_state
              .template block<imu_integrator::kStateOrientationBlockSize, 1>(
                  imu_integrator::kStateOrientationOffset, 0)
              .data()),
      current_state.template block<imu_integrator::kVelocityBlockSize, 1>(
          imu_integrator::kStateVelocityOffset, 0)};

  return result;
}

// Returns velocity delta in map frame.
template <typename T, typename RangeType, typename IteratorType>
IntegrateImuWithTranslationResult<double> IntegrateImuWithTranslation(
    const RangeType& imu_data,
    const Eigen::Transform<double, 3, Eigen::Affine>&
        linear_acceleration_calibration,
    const Eigen::Transform<double, 3, Eigen::Affine>&
        angular_velocity_calibration,
    const State& start_state, const common::Time start_time,
    const common::Time end_time,
    const ::cartographer::mapping::proto::
        OptimizingLocalTrajectoryBuilderOptions_IMUIntegrator& integrator_type,
    IteratorType* const it) {
  switch (integrator_type) {
    case proto::OptimizingLocalTrajectoryBuilderOptions_IMUIntegrator_EULER: {
      return IntegrateImuWithTranslationRK4(
          imu_data, linear_acceleration_calibration,
          angular_velocity_calibration, start_state.translation,
          start_state.rotation, start_state.velocity, start_time, end_time, it);
    }
    case proto::OptimizingLocalTrajectoryBuilderOptions_IMUIntegrator_RK4: {
      return IntegrateImuWithTranslationRK4(
          imu_data, linear_acceleration_calibration,
          angular_velocity_calibration, start_state.translation,
          start_state.rotation, start_state.velocity, start_time, end_time, it);
    }
    default:
      LOG(FATAL) << "Unsupported imu integrator type.";
      return IntegrateImuWithTranslationRK4(
          imu_data, linear_acceleration_calibration,
          angular_velocity_calibration, start_state.translation,
          start_state.rotation, start_state.velocity, start_time, end_time, it);
  }
}

// Returns velocity delta in map frame.
template <typename RangeType, typename IteratorType>
IntegrateImuWithTranslationResult<double> IntegrateImuWithTranslation(
    const RangeType& imu_data, const State& start_state,
    const common::Time start_time, const common::Time end_time,
    const ::cartographer::mapping::proto::
        OptimizingLocalTrajectoryBuilderOptions_IMUIntegrator& integrator_type,
    IteratorType* const it) {
  switch (integrator_type) {
    case proto::OptimizingLocalTrajectoryBuilderOptions_IMUIntegrator_EULER: {
      LOG(FATAL) << "TODO EULER";
      return IntegrateImuWithTranslationRK4(
          imu_data, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(),
          start_state.translation, start_state.rotation, start_state.velocity,
          start_time, end_time, it);
    }
    case proto::OptimizingLocalTrajectoryBuilderOptions_IMUIntegrator_RK4: {
      return IntegrateImuWithTranslationRK4(
          imu_data, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(),
          start_state.translation, start_state.rotation, start_state.velocity,
          start_time, end_time, it);
    }
    default:
      LOG(FATAL) << "Unsupported imu integrator type.";
  }
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_INTEGRATION_H_
