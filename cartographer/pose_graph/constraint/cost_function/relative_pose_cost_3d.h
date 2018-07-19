/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_RELATIVE_POSE_COST_3D_H
#define CARTOGRAPHER_RELATIVE_POSE_COST_3D_H

#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "cartographer/pose_graph/proto/cost_function.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace pose_graph {

// Provides cost function for relative pose and de/serialization methods.
class RelativePoseCost3D {
 public:
  explicit RelativePoseCost3D(
      const proto::RelativePose3D::Parameters& parameters);

  proto::RelativePose3D::Parameters ToProto() const;

  template <typename T>
  bool operator()(const T* const c_i_translation, const T* const c_i_rotation,
                  const T* const c_j_translation, const T* const c_j_rotation,
                  T* const error_out) const {
    const std::array<T, 6> error = mapping::optimization::ScaleError(
        mapping::optimization::ComputeUnscaledError(
            first_T_second_, c_i_rotation, c_i_translation, c_j_rotation,
            c_j_translation),
        translation_weight_, rotation_weight_);
    std::copy(std::begin(error), std::end(error), error_out);
    return true;
  }

 private:
  const double translation_weight_;
  const double rotation_weight_;
  const transform::Rigid3d first_T_second_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_RELATIVE_POSE_COST_3D_H
