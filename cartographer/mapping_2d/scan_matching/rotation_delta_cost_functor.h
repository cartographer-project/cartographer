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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_

#include "Eigen/Core"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

// Computes the cost of rotating 'pose' to 'target_angle'. Cost increases with
// the solution's distance from 'target_angle'.
class RotationDeltaCostFunctor {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor(scaling_factor, target_angle));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor(const double scaling_factor,
                                    const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  RotationDeltaCostFunctor(const RotationDeltaCostFunctor&) = delete;
  RotationDeltaCostFunctor& operator=(const RotationDeltaCostFunctor&) = delete;

  const double scaling_factor_;
  const double angle_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_H_
