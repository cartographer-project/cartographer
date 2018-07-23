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

#ifndef CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_INTERPOLATED_RELATIVE_POSE_COST_3D_H_
#define CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_INTERPOLATED_RELATIVE_POSE_COST_3D_H_

#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "cartographer/pose_graph/proto/cost_function.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace pose_graph {

// Provides cost function for interpolated relative pose in 3d and
// de/serialization methods; only the first pose is linearly interpolated
// between two nodes.
class InterpolatedRelativePoseCost3D {
 public:
  InterpolatedRelativePoseCost3D(
      const proto::InterpolatedRelativePose3D::Parameters& parameters);

  proto::InterpolatedRelativePose3D::Parameters ToProto() const;

  template <typename T>
  bool operator()(const T* const first_start_translation,
                  const T* const first_start_rotation,
                  const T* const first_end_translation,
                  const T* const first_end_rotation,
                  const T* const second_translation,
                  const T* const second_rotation, T* const error_out) const {
    const std::tuple<std::array<T, 4>, std::array<T, 3>>
        interpolated_rotation_and_translation =
            mapping::optimization::InterpolateNodes3D(
                first_start_rotation, first_start_translation,
                first_end_rotation, first_end_translation,
                interpolation_factor_);
    const std::array<T, 6> error = mapping::optimization::ScaleError(
        mapping::optimization::ComputeUnscaledError(
            first_T_second_,
            std::get<0>(interpolated_rotation_and_translation).data(),
            std::get<1>(interpolated_rotation_and_translation).data(),
            second_rotation, second_translation),
        translation_weight_, rotation_weight_);
    std::copy(std::begin(error), std::end(error), error_out);
    return true;
  }

 private:
  const double translation_weight_;
  const double rotation_weight_;
  const double interpolation_factor_;
  const transform::Rigid3d first_T_second_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_INTERPOLATED_RELATIVE_POSE_COST_3D_H_
