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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_LANDMARK_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_LANDMARK_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_3d.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping {
namespace optimization {

// Cost function measuring the weighted error between the observed pose given by
// the landmark measurement and the linearly interpolated pose.
class LandmarkCostFunction3D {
 public:
  using LandmarkObservation =
      PoseGraphInterface::LandmarkNode::LandmarkObservation;

  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const LandmarkObservation& observation, const NodeSpec3D& prev_node,
      const NodeSpec3D& next_node) {
    return new ceres::AutoDiffCostFunction<
        LandmarkCostFunction3D, 6 /* residuals */,
        4 /* previous node rotation variables */,
        3 /* previous node translation variables */,
        4 /* next node rotation variables */,
        3 /* next node translation variables */,
        4 /* landmark rotation variables */,
        3 /* landmark translation variables */>(
        new LandmarkCostFunction3D(observation, prev_node, next_node));
  }

  template <typename T>
  bool operator()(const T* const prev_node_rotation,
                  const T* const prev_node_translation,
                  const T* const next_node_rotation,
                  const T* const next_node_translation,
                  const T* const landmark_rotation,
                  const T* const landmark_translation, T* const e) const {
    const std::tuple<std::array<T, 4>, std::array<T, 3>>
        interpolated_rotation_and_translation = InterpolateNodes3D(
            prev_node_rotation, prev_node_translation, next_node_rotation,
            next_node_translation, interpolation_parameter_);
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
  LandmarkCostFunction3D(const LandmarkObservation& observation,
                         const NodeSpec3D& prev_node,
                         const NodeSpec3D& next_node)
      : landmark_to_tracking_transform_(
            observation.landmark_to_tracking_transform),
        translation_weight_(observation.translation_weight),
        rotation_weight_(observation.rotation_weight),
        interpolation_parameter_(
            common::ToSeconds(observation.time - prev_node.time) /
            common::ToSeconds(next_node.time - prev_node.time)) {}

  const transform::Rigid3d landmark_to_tracking_transform_;
  const double translation_weight_;
  const double rotation_weight_;
  const double interpolation_parameter_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_LANDMARK_COST_FUNCTION_3D_H_
