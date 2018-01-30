/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_LANDMARK_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_LANDMARK_COST_FUNCTION_H_

#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping_2d/pose_graph/spa_cost_function.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping_2d {
namespace pose_graph {

// Cost function measuring the weighted error between the observed pose given by
// the landmark measurement and the linearly interpolated pose.
class LandmarkCostFunction {
 public:
  using LandmarkObservation =
      mapping::PoseGraph::LandmarkNode::LandmarkObservation;

  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const LandmarkObservation& observation, common::Time prev_node_time,
      common::Time next_node_time) {
    return new ceres::AutoDiffCostFunction<
        LandmarkCostFunction, 3 /* residuals */,
        3 /* previous node pose variables */, 3 /* next node pose variables */,
        3 /* landmark pose variables */>(
        new LandmarkCostFunction(observation, prev_node_time, next_node_time));
  }

  template <typename T>
  bool operator()(const T* const prev_node_pose, const T* const next_node_pose,
                  const T* const landmark_pose, T* const e) const {
    const T interpolated_pose[3] = {
        prev_node_pose[0] +
            interpolation_parameter_ * (next_node_pose[0] - prev_node_pose[0]),
        prev_node_pose[1] +
            interpolation_parameter_ * (next_node_pose[1] - prev_node_pose[1]),
        prev_node_pose[2] + interpolation_parameter_ *
                                common::NormalizeAngleDifference(
                                    next_node_pose[2] - prev_node_pose[2])};

    // TODO(pifon2a): Move functions common for all cost functions outside of
    // SpaCostFunction scope.
    const std::array<T, 3> unscaled_error =
        SpaCostFunction::ComputeUnscaledError(landmark_to_tracking_transform_,
                                              interpolated_pose, landmark_pose);
    e[0] = T(translation_weight_) * unscaled_error[0];
    e[1] = T(translation_weight_) * unscaled_error[1];
    e[2] = T(rotation_weight_) * unscaled_error[2];
    return true;
  }

 private:
  LandmarkCostFunction(const LandmarkObservation& observation,
                       common::Time prev_node_time, common::Time next_node_time)
      : landmark_to_tracking_transform_(
            transform::Project2D(observation.landmark_to_tracking_transform)),
        translation_weight_(observation.translation_weight),
        rotation_weight_(observation.rotation_weight),
        interpolation_parameter_(
            common::ToSeconds(observation.time - prev_node_time) /
            common::ToSeconds(next_node_time - prev_node_time)) {}

  const transform::Rigid2d landmark_to_tracking_transform_;
  const double translation_weight_;
  const double rotation_weight_;
  const double interpolation_parameter_;
};

}  // namespace pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_LANDMARK_COST_FUNCTION_H_
