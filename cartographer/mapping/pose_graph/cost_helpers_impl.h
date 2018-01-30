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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_COST_HELPERS_IMPL_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_COST_HELPERS_IMPL_H_

namespace cartographer {
namespace mapping {
namespace pose_graph {

template <typename T>
static std::array<T, 3> ComputeUnscaledError2D(
    const transform::Rigid2d& zbar_ij, const T* const c_i, const T* const c_j) {
  const T cos_theta_i = cos(c_i[2]);
  const T sin_theta_i = sin(c_i[2]);
  const T delta_x = c_j[0] - c_i[0];
  const T delta_y = c_j[1] - c_i[1];
  const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  c_j[2] - c_i[2]};
  return {
      {T(zbar_ij.translation().x()) - h[0], T(zbar_ij.translation().y()) - h[1],
       common::NormalizeAngleDifference(T(zbar_ij.rotation().angle()) - h[2])}};
}

template <typename T>
static std::array<T, 6> ComputeUnscaledError3D(
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
      Eigen::Quaternion<T>(c_j_rotation[0], -c_j_rotation[1], -c_j_rotation[2],
                           -c_j_rotation[3]) *
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

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_COST_HELPERS_IMPL_H_
