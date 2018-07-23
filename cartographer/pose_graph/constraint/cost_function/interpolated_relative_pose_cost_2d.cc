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

#ifndef CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_INTERPOLATED_RELATIVE_POSE_COST_3D_H
#define CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_INTERPOLATED_RELATIVE_POSE_COST_3D_H

#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "cartographer/pose_graph/proto/cost_function.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace pose_graph {

// Provides cost function for relative pose and de/serialization methods.
class InterpolatedRelativePoseCost2D {
 public:
  explicit InterpolatedRelativePoseCost2D(
      const proto::InterpolatedRelativePose2D::Parameters& parameters);

  proto::InterpolatedRelativePose2D::Parameters ToProto() const;

  template <typename T>
  bool operator()(const T* const prev_node_pose, const T* const next_node_pose,
                  const T* const landmark_rotation,
                  const T* const landmark_translation, T* const e) const {
    const std::tuple<std::array<T, 4>, std::array<T, 3>>
        interpolated_rotation_and_translation = InterpolateNodes2D(
            prev_node_pose, prev_node_gravity_alignment_, next_node_pose,
            next_node_gravity_alignment_, interpolation_parameter_);
    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(
            landmark_to_tracking_transform_,
            std::get<0>(interpolated_rotation_and_translation).data(),
            std::get<1>(interpolated_rotation_and_translation).data(),
            landmark_rotation, landmark_translation),
        translation_weight_, rotation_weight_);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  const double translation_weight_;
  const double rotation_weight_;
  const transform::Rigid3d first_T_second_;

  const Eigen::Quaterniond prev_node_gravity_alignment_;
  const Eigen::Quaterniond next_node_gravity_alignment_;
  const double interpolation_parameter_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_INTERPOLATED_RELATIVE_POSE_COST_3D_H
