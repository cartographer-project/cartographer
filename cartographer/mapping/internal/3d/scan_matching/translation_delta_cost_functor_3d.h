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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_3D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes the cost of translating 'translation' to 'target_translation'.
// Cost increases with the solution's distance from 'target_translation'.
class TranslationDeltaCostFunctor3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector3d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor3D,
                                           3 /* residuals */,
                                           3 /* translation variables */>(
        new TranslationDeltaCostFunctor3D(scaling_factor, target_translation));
  }

  template <typename T>
  bool operator()(const T* const translation, T* residual) const {
    residual[0] = scaling_factor_ * (translation[0] - x_);
    residual[1] = scaling_factor_ * (translation[1] - y_);
    residual[2] = scaling_factor_ * (translation[2] - z_);
    return true;
  }

 private:
  // Constructs a new TranslationDeltaCostFunctor3D from the given
  // 'target_translation'.
  explicit TranslationDeltaCostFunctor3D(
      const double scaling_factor, const Eigen::Vector3d& target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()),
        z_(target_translation.z()) {}

  TranslationDeltaCostFunctor3D(const TranslationDeltaCostFunctor3D&) = delete;
  TranslationDeltaCostFunctor3D& operator=(
      const TranslationDeltaCostFunctor3D&) = delete;

  const double scaling_factor_;
  const double x_;
  const double y_;
  const double z_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_3D_H_
