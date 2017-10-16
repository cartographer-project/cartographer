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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_H_

#include "Eigen/Core"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

// Computes the cost of translating the initial pose estimate. Cost increases
// with the solution's distance from the initial estimate.
class TranslationDeltaCostFunctor {
 public:
  // Constructs a new TranslationDeltaCostFunctor from the given
  // 'local_pose_estimate'.
  explicit TranslationDeltaCostFunctor(
      const double scaling_factor,
      const transform::Rigid3d& local_pose_estimate)
      : scaling_factor_(scaling_factor),
        x_(local_pose_estimate.translation().x()),
        y_(local_pose_estimate.translation().y()),
        z_(local_pose_estimate.translation().z()) {}

  TranslationDeltaCostFunctor(const TranslationDeltaCostFunctor&) = delete;
  TranslationDeltaCostFunctor& operator=(const TranslationDeltaCostFunctor&) =
      delete;

  template <typename T>
  bool operator()(const T* const translation, T* residual) const {
    residual[0] = scaling_factor_ * (translation[0] - x_);
    residual[1] = scaling_factor_ * (translation[1] - y_);
    residual[2] = scaling_factor_ * (translation[2] - z_);
    return true;
  }

 private:
  const double scaling_factor_;
  const double x_;
  const double y_;
  const double z_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_H_
