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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_STATIC_CALIBRATION_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_STATIC_CALIBRATION_H_

#include <algorithm>
#include <deque>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/imu_integration.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Static IMU Calibration, assumes the IMU to be static during calibration
// interval. Estimates acceleration scale, angular velocity biases and aligns
// the IMU frame with the direction of gravity.
void CalibrateIMU(
    const std::deque<sensor::ImuData>& imu_data, const double gravity_constant,
    Eigen::Transform<double, 3, Eigen::Affine>& linear_acceleration_calibration,
    Eigen::Transform<double, 3, Eigen::Affine>& angular_velocity_calibration) {
  Eigen::Vector3d angular_biases = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_biases = Eigen::Vector3d::Zero();
  for (const auto& imu_observation : imu_data) {
    acceleration_biases += imu_observation.linear_acceleration;
    angular_biases += imu_observation.angular_velocity;
  }
  double normalization_factor = 1.0 / imu_data.size();
  angular_biases = normalization_factor * angular_biases;
  acceleration_biases = normalization_factor * acceleration_biases;
  LOG(INFO) << "IMU data before calibration:\n  Linear accelerations:\n"
            << acceleration_biases << "\n Acceleration scale: \n"
            << acceleration_biases.norm() / gravity_constant
            << "\n Angular velocities: \n"
            << angular_biases << "\n Data duration: \n"
            << common::ToSeconds(imu_data.back().time - imu_data.front().time)
            << "\n Data set size: \n"
            << imu_data.size();
  double acceleration_scale_correction_factor =
      gravity_constant / acceleration_biases.norm();
  Eigen::Quaterniond r = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond::FromTwoVectors(
      acceleration_biases.normalized(), Eigen::Vector3d({0.0, 0.0, 1.0}));

  linear_acceleration_calibration =
      acceleration_scale_correction_factor * r.matrix();
  angular_velocity_calibration =
      r.matrix() * Eigen::Translation3d(-angular_biases);

  LOG(INFO) << "Angular velocity correction:\n"
            << angular_velocity_calibration.rotation() << "\n"
            << angular_velocity_calibration.translation() << "\n"
            << "Linear acceleration correction:\n"
            << linear_acceleration_calibration.rotation() << "\n"
            << linear_acceleration_calibration.translation();
  LOG(INFO) << "IMU data after calibration:\n  Linear accelerations:\n"
            << linear_acceleration_calibration * acceleration_biases
            << "\n Acceleration scale: \n"
            << (linear_acceleration_calibration * acceleration_biases).norm() /
                   gravity_constant
            << "\n Angular velocities: \n"
            << angular_velocity_calibration * angular_biases;
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_IMU_STATIC_CALIBRATION_H_
