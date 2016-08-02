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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_

#include <cmath>

#include "Eigen/Core"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/rotation.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

// Computes the cost of rotating the pose estimate. Cost increases with the
// solution's distance from the rotation estimate.
class RotationDeltaCostFunctor {
 public:
  // Constructs a new RotationDeltaCostFunctor from the given
  // 'rotation_estimate'.
  explicit RotationDeltaCostFunctor(const double scaling_factor,
                                    const Eigen::Quaterniond& initial_rotation)
      : scaling_factor_(scaling_factor) {
    initial_rotation_inverse_[0] = initial_rotation.w();
    initial_rotation_inverse_[1] = -initial_rotation.x();
    initial_rotation_inverse_[2] = -initial_rotation.y();
    initial_rotation_inverse_[3] = -initial_rotation.z();
  }

  RotationDeltaCostFunctor(const RotationDeltaCostFunctor&) = delete;
  RotationDeltaCostFunctor& operator=(const RotationDeltaCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const rotation_quaternion, T* residual) const {
    T delta[4];
    T initial_rotation_inverse[4] = {
        T(initial_rotation_inverse_[0]), T(initial_rotation_inverse_[1]),
        T(initial_rotation_inverse_[2]), T(initial_rotation_inverse_[3])};
    ceres::QuaternionProduct(initial_rotation_inverse, rotation_quaternion,
                             delta);
    // Will compute the squared norm of the imaginary component of the delta
    // quaternion which is sin(phi/2)^2.
    residual[0] = scaling_factor_ * delta[1];
    residual[1] = scaling_factor_ * delta[2];
    residual[2] = scaling_factor_ * delta[3];
    return true;
  }

 private:
  const double scaling_factor_;
  double initial_rotation_inverse_[4];
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_
