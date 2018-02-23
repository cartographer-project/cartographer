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

#ifndef CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_LANDMARK_COST_FUNCTION_2D_H_
#define CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_LANDMARK_COST_FUNCTION_2D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/pose_graph/cost_helpers.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping_2d/pose_graph/optimization_problem_2d.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {

// Cost function measuring the weighted error between the observed pose given by
// the landmark measurement and the linearly interpolated pose of embedded in 3D
// space node poses.
class LandmarkCostFunction2D {
 public:
  using LandmarkObservation =
      PoseGraphInterface::LandmarkNode::LandmarkObservation;

  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const LandmarkObservation& observation,
      const OptimizationProblem2D::NodeData& prev_node,
      const OptimizationProblem2D::NodeData& next_node) {
    return new ceres::AutoDiffCostFunction<
        LandmarkCostFunction2D, 6 /* residuals */,
        3 /* previous node pose variables */, 3 /* next node pose variables */,
        4 /* landmark rotation variables */,
        3 /* landmark translation variables */>(
        new LandmarkCostFunction2D(observation, prev_node, next_node));
  }

  template <typename T>
  bool operator()(const T* const prev_node_pose, const T* const next_node_pose,
                  const T* const landmark_rotation,
                  const T* const landmark_translation, T* const e) const {
    using pose_graph::ComputeUnscaledError;
    using pose_graph::ScaleError;
    using pose_graph::SlerpQuaternions;

    const std::array<T, 3> interpolated_pose_translation{
        {prev_node_pose[0] +
             interpolation_parameter_ * (next_node_pose[0] - prev_node_pose[0]),
         prev_node_pose[1] +
             interpolation_parameter_ * (next_node_pose[1] - prev_node_pose[1]),
         T(0)}};

    // The following is equivalent to (Embed3D(prev_pose) *
    // Rigid3d::Rotation(prev_pose_gravity_alignment)).rotation().
    const Eigen::Quaternion<T> prev_quaternion(
        (Eigen::AngleAxis<T>(prev_node_pose[2],
                             Eigen::Matrix<T, 3, 1>::UnitZ()) *
         prev_node_gravity_alignment_.cast<T>())
            .normalized());
    const std::array<T, 4> prev_node_rotation = {
        {prev_quaternion.w(), prev_quaternion.x(), prev_quaternion.y(),
         prev_quaternion.z()}};

    // The following is equivalent to (Embed3D(next_pose) *
    // Rigid3d::Rotation(next_pose_gravity_alignment)).rotation().
    const Eigen::Quaternion<T> next_quaternion(
        (Eigen::AngleAxis<T>(next_node_pose[2],
                             Eigen::Matrix<T, 3, 1>::UnitZ()) *
         next_node_gravity_alignment_.cast<T>())
            .normalized());
    const std::array<T, 4> next_node_rotation = {
        {next_quaternion.w(), next_quaternion.x(), next_quaternion.y(),
         next_quaternion.z()}};

    const std::array<T, 4> interpolated_pose_rotation =
        SlerpQuaternions(prev_node_rotation.data(), next_node_rotation.data(),
                         T(interpolation_parameter_));

    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(landmark_to_tracking_transform_,
                             interpolated_pose_rotation.data(),
                             interpolated_pose_translation.data(),
                             landmark_rotation, landmark_translation),
        T(translation_weight_), T(rotation_weight_));
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  LandmarkCostFunction2D(const LandmarkObservation& observation,
                         const OptimizationProblem2D::NodeData& prev_node,
                         const OptimizationProblem2D::NodeData& next_node)
      : landmark_to_tracking_transform_(
            observation.landmark_to_tracking_transform),
        prev_node_gravity_alignment_(prev_node.gravity_alignment),
        next_node_gravity_alignment_(next_node.gravity_alignment),
        translation_weight_(observation.translation_weight),
        rotation_weight_(observation.rotation_weight),
        interpolation_parameter_(
            common::ToSeconds(observation.time - prev_node.time) /
            common::ToSeconds(next_node.time - prev_node.time)) {}

  const transform::Rigid3d landmark_to_tracking_transform_;
  const Eigen::Quaterniond prev_node_gravity_alignment_;
  const Eigen::Quaterniond next_node_gravity_alignment_;
  const double translation_weight_;
  const double rotation_weight_;
  const double interpolation_parameter_;
};

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_LANDMARK_COST_FUNCTION_2D_H_
