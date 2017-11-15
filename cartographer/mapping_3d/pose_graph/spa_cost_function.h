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

#ifndef CARTOGRAPHER_MAPPING_3D_POSE_GRAPH_SPA_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_3D_POSE_GRAPH_SPA_COST_FUNCTION_H_

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
namespace mapping_3d {
namespace pose_graph {

class SpaCostFunction {
 public:
  using Constraint = mapping::PoseGraph::Constraint;

  explicit SpaCostFunction(const Constraint::Pose& pose) : pose_(pose) {}

  // Computes the error between the node-to-submap alignment 'zbar_ij' and the
  // difference of submap pose 'c_i' and node pose 'c_j' which are both in an
  // arbitrary common frame.
  template <typename T>
  static std::array<T, 6> ComputeUnscaledError(
      const transform::Rigid3d& zbar_ij, const T* const c_i_rotation,
      const T* const c_i_translation, const T* const c_j_rotation,
      const T* const c_j_translation) {
    const Eigen::Quaternion<T> R_i_inverse(c_i_rotation[0], -c_i_rotation[1],
                                           -c_i_rotation[2], -c_i_rotation[3]);

    const Eigen::Matrix<T, 3, 1> delta(c_j_translation[0] - c_i_translation[0],
                                       c_j_translation[1] - c_i_translation[1],
                                       c_j_translation[2] - c_i_translation[2]);
    const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

    const Eigen::Quaternion<T> h_rotation_inverse =
        Eigen::Quaternion<T>(c_j_rotation[0], -c_j_rotation[1],
                             -c_j_rotation[2], -c_j_rotation[3]) *
        Eigen::Quaternion<T>(c_i_rotation[0], c_i_rotation[1], c_i_rotation[2],
                             c_i_rotation[3]);

    const Eigen::Matrix<T, 3, 1> angle_axis_difference =
        transform::RotationQuaternionToAngleAxisVector(
            h_rotation_inverse * zbar_ij.rotation().cast<T>());

    return {{T(zbar_ij.translation().x()) - h_translation[0],
             T(zbar_ij.translation().y()) - h_translation[1],
             T(zbar_ij.translation().z()) - h_translation[2],
             angle_axis_difference[0], angle_axis_difference[1],
             angle_axis_difference[2]}};
  }

  // Computes the error scaled by 'translation_weight' and 'rotation_weight',
  // storing it in 'e'.
  template <typename T>
  static void ComputeScaledError(const Constraint::Pose& pose,
                                 const T* const c_i_rotation,
                                 const T* const c_i_translation,
                                 const T* const c_j_rotation,
                                 const T* const c_j_translation, T* const e) {
    const std::array<T, 6> e_ij =
        ComputeUnscaledError(pose.zbar_ij, c_i_rotation, c_i_translation,
                             c_j_rotation, c_j_translation);
    for (int ij : {0, 1, 2}) {
      e[ij] = e_ij[ij] * T(pose.translation_weight);
    }
    for (int ij : {3, 4, 5}) {
      e[ij] = e_ij[ij] * T(pose.rotation_weight);
    }
  }

  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    ComputeScaledError(pose_, c_i_rotation, c_i_translation, c_j_rotation,
                       c_j_translation, e);
    return true;
  }

 private:
  const Constraint::Pose pose_;
};

}  // namespace pose_graph
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_POSE_GRAPH_SPA_COST_FUNCTION_H_
