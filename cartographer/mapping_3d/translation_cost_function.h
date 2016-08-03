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

#ifndef CARTOGRAPHER_MAPPING_3D_TRANSLATION_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_3D_TRANSLATION_COST_FUNCTION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace cartographer {
namespace mapping_3d {

// Penalizes differences between velocity and change in position.
class TranslationCostFunction {
 public:
  TranslationCostFunction(const double scaling_factor,
                          const double delta_time_seconds)
      : scaling_factor_(scaling_factor),
        delta_time_seconds_(delta_time_seconds) {}

  TranslationCostFunction(const TranslationCostFunction&) = delete;
  TranslationCostFunction& operator=(const TranslationCostFunction&) = delete;

  template <typename T>
  bool operator()(const T* const start_translation,
                  const T* const end_translation, const T* const velocity,
                  T* residual) const {
    const T delta_x = end_translation[0] - start_translation[0];
    const T delta_y = end_translation[1] - start_translation[1];
    const T delta_z = end_translation[2] - start_translation[2];

    residual[0] =
        scaling_factor_ * (delta_x - velocity[0] * delta_time_seconds_);
    residual[1] =
        scaling_factor_ * (delta_y - velocity[1] * delta_time_seconds_);
    residual[2] =
        scaling_factor_ * (delta_z - velocity[2] * delta_time_seconds_);
    return true;
  }

 private:
  const double scaling_factor_;
  const double delta_time_seconds_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_TRANSLATION_COST_FUNCTION_H_
