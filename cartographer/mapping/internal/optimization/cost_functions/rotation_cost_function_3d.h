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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ROTATION_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ROTATION_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {

// Penalizes differences between IMU data and optimized orientations.
class RotationCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor,
      const Eigen::Quaterniond& delta_rotation_imu_frame) {
    return new ceres::AutoDiffCostFunction<
        RotationCostFunction3D, 3 /* residuals */, 4 /* rotation variables */,
        4 /* rotation variables */, 4 /* rotation variables */
        >(new RotationCostFunction3D(scaling_factor, delta_rotation_imu_frame));
  }

  template <typename T>
  bool operator()(const T* const start_rotation, const T* const end_rotation,
                  const T* const imu_calibration, T* residual) const {
    const Eigen::Quaternion<T> start(start_rotation[0], start_rotation[1],
                                     start_rotation[2], start_rotation[3]);
    const Eigen::Quaternion<T> end(end_rotation[0], end_rotation[1],
                                   end_rotation[2], end_rotation[3]);
    const Eigen::Quaternion<T> eigen_imu_calibration(
        imu_calibration[0], imu_calibration[1], imu_calibration[2],
        imu_calibration[3]);
    const Eigen::Quaternion<T> error =
        end.conjugate() * start * eigen_imu_calibration *
        delta_rotation_imu_frame_.cast<T>() * eigen_imu_calibration.conjugate();
    residual[0] = scaling_factor_ * error.x();
    residual[1] = scaling_factor_ * error.y();
    residual[2] = scaling_factor_ * error.z();
    return true;
  }

 private:
  RotationCostFunction3D(const double scaling_factor,
                         const Eigen::Quaterniond& delta_rotation_imu_frame)
      : scaling_factor_(scaling_factor),
        delta_rotation_imu_frame_(delta_rotation_imu_frame) {}

  RotationCostFunction3D(const RotationCostFunction3D&) = delete;
  RotationCostFunction3D& operator=(const RotationCostFunction3D&) = delete;

  const double scaling_factor_;
  const Eigen::Quaterniond delta_rotation_imu_frame_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ROTATION_COST_FUNCTION_3D_H_
