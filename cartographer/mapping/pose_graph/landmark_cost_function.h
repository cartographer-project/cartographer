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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_LANDMARK_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_LANDMARK_COST_FUNCTION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/pose_graph/cost_helpers.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {

template <typename T>
std::array<T, 4> SlerpQuaternions(const T* const prev_rotation,
                                  const T* const next_rotation, T factor) {
  // Angle 'theta' is the half-angle "between" quaternions. It can be computed
  // as the arccosine of their dot product.
  const T cos_theta = prev_rotation[0] * next_rotation[0] +
                      prev_rotation[1] * next_rotation[1] +
                      prev_rotation[2] * next_rotation[2] +
                      prev_rotation[3] * next_rotation[3];
  // If numerical error brings 'cos_theta' outside of [-1., 1.] interval, then
  // the quaternions are likely to be collinear.
  if (cos_theta >= T(1.0) || cos_theta <= T(-1.0)) {
    return {{next_rotation[0], next_rotation[1], next_rotation[2],
             next_rotation[3]}};
  }
  const T theta = acos(abs(cos_theta));
  const T sin_theta = sin(theta);
  const T prev_scale = sin((T(1.0) - factor) * theta) / sin_theta;
  const T next_scale =
      sin(factor * theta) * (cos_theta < T(0) ? T(-1.0) : T(1.0)) / sin_theta;

  return {{prev_scale * prev_rotation[0] + next_scale * next_rotation[0],
           prev_scale * prev_rotation[1] + next_scale * next_rotation[1],
           prev_scale * prev_rotation[2] + next_scale * next_rotation[2],
           prev_scale * prev_rotation[3] + next_scale * next_rotation[3]}};
}

// Cost function measuring the weighted error between the observed pose given by
// the landmark measurement and the linearly interpolated pose.
class LandmarkCostFunction {
 public:
  using LandmarkObservation =
      mapping::PoseGraphInterface::LandmarkNode::LandmarkObservation;

  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const LandmarkObservation& observation, common::Time prev_node_time,
      common::Time next_node_time) {
    return new ceres::AutoDiffCostFunction<
        LandmarkCostFunction, 6 /* residuals */,
        4 /* previous node rotation variables */,
        3 /* previous node translation variables */,
        4 /* next node rotation variables */,
        3 /* next node translation variables */,
        4 /* landmark rotation variables */,
        3 /* landmark translation variables */>(
        new LandmarkCostFunction(observation, prev_node_time, next_node_time));
  }

  template <typename T>
  bool operator()(const T* const prev_node_rotation,
                  const T* const prev_node_translation,
                  const T* const next_node_rotation,
                  const T* const next_node_translation,
                  const T* const landmark_rotation,
                  const T* const landmark_translation, T* const e) const {
    const T interpolated_pose_translation[3] = {
        prev_node_translation[0] +
            interpolation_parameter_ *
                (next_node_translation[0] - prev_node_translation[0]),
        prev_node_translation[1] +
            interpolation_parameter_ *
                (next_node_translation[1] - prev_node_translation[1]),
        prev_node_translation[2] +
            interpolation_parameter_ *
                (next_node_translation[2] - prev_node_translation[2])};

    std::array<T, 4> interpolated_pose_rotation = SlerpQuaternions(
        prev_node_rotation, next_node_rotation, T(interpolation_parameter_));

    const std::array<T, 6> unscaled_error = ComputeUnscaledError3d(
        landmark_to_tracking_transform_, interpolated_pose_rotation.data(),
        interpolated_pose_translation, landmark_rotation, landmark_translation);

    e[0] = T(translation_weight_) * unscaled_error[0];
    e[1] = T(translation_weight_) * unscaled_error[1];
    e[2] = T(translation_weight_) * unscaled_error[2];
    e[3] = T(rotation_weight_) * unscaled_error[3];
    e[4] = T(rotation_weight_) * unscaled_error[4];
    e[5] = T(rotation_weight_) * unscaled_error[5];
    return true;
  }

 private:
  LandmarkCostFunction(const LandmarkObservation& observation,
                       common::Time prev_node_time, common::Time next_node_time)
      : landmark_to_tracking_transform_(
            observation.landmark_to_tracking_transform),
        translation_weight_(observation.translation_weight),
        rotation_weight_(observation.rotation_weight),
        interpolation_parameter_(
            common::ToSeconds(observation.time - prev_node_time) /
            common::ToSeconds(next_node_time - prev_node_time)) {}

  const transform::Rigid3d landmark_to_tracking_transform_;
  const double translation_weight_;
  const double rotation_weight_;
  const double interpolation_parameter_;
};

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_LANDMARK_COST_FUNCTION_H_
