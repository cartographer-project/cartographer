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

#ifndef CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_SPA_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_SPA_COST_FUNCTION_H_

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping_2d {
namespace pose_graph {

class SpaCostFunction {
 public:
  using Constraint = mapping::PoseGraph::Constraint;

  explicit SpaCostFunction(const Constraint::Pose& pose) : pose_(pose) {}

  // Computes the error between the node-to-submap alignment 'zbar_ij' and the
  // difference of submap pose 'c_i' and node pose 'c_j' which are both in an
  // arbitrary common frame.
  template <typename T>
  static std::array<T, 3> ComputeUnscaledError(
      const transform::Rigid2d& zbar_ij, const T* const c_i,
      const T* const c_j) {
    const T cos_theta_i = cos(c_i[2]);
    const T sin_theta_i = sin(c_i[2]);
    const T delta_x = c_j[0] - c_i[0];
    const T delta_y = c_j[1] - c_i[1];
    const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                    -sin_theta_i * delta_x + cos_theta_i * delta_y,
                    c_j[2] - c_i[2]};
    return {{T(zbar_ij.translation().x()) - h[0],
             T(zbar_ij.translation().y()) - h[1],
             common::NormalizeAngleDifference(T(zbar_ij.rotation().angle()) -
                                              h[2])}};
  }

  // Computes the error scaled by 'translation_weight' and 'rotation_weight',
  // storing it in 'e'.
  template <typename T>
  static void ComputeScaledError(const Constraint::Pose& pose,
                                 const T* const c_i, const T* const c_j,
                                 T* const e) {
    const std::array<T, 3> e_ij =
        ComputeUnscaledError(transform::Project2D(pose.zbar_ij), c_i, c_j);
    e[0] = e_ij[0] * T(pose.translation_weight);
    e[1] = e_ij[1] * T(pose.translation_weight);
    e[2] = e_ij[2] * T(pose.rotation_weight);
  }

  template <typename T>
  bool operator()(const T* const c_i, const T* const c_j, T* e) const {
    ComputeScaledError(pose_, c_i, c_j, e);
    return true;
  }

 private:
  const Constraint::Pose pose_;
};

}  // namespace pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_POSE_GRAPH_SPA_COST_FUNCTION_H_
