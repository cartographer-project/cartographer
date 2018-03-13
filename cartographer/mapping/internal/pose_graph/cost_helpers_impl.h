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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_POSE_GRAPH_COST_HELPERS_IMPL_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_POSE_GRAPH_COST_HELPERS_IMPL_H_

namespace cartographer {
namespace mapping {
namespace pose_graph {

template <typename T>
static std::array<T, 3> ComputeUnscaledError(
    const transform::Rigid2d& relative_pose, const T* const start,
    const T* const end) {
  const T cos_theta_i = cos(start[2]);
  const T sin_theta_i = sin(start[2]);
  const T delta_x = end[0] - start[0];
  const T delta_y = end[1] - start[1];
  const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  end[2] - start[2]};
  return {{T(relative_pose.translation().x()) - h[0],
           T(relative_pose.translation().y()) - h[1],
           common::NormalizeAngleDifference(
               T(relative_pose.rotation().angle()) - h[2])}};
}

template <typename T>
std::array<T, 3> ScaleError(std::array<T, 3> error, T translation_weight,
                            T rotation_weight) {
  std::array<T, 3> scaled_error(std::move(error));
  scaled_error[0] *= translation_weight;
  scaled_error[1] *= translation_weight;
  scaled_error[2] *= rotation_weight;
  return scaled_error;
}

template <typename T>
static std::array<T, 6> ComputeUnscaledError(
    const transform::Rigid3d& relative_pose, const T* const start_rotation,
    const T* const start_translation, const T* const end_rotation,
    const T* const end_translation) {
  const Eigen::Quaternion<T> R_i_inverse(start_rotation[0], -start_rotation[1],
                                         -start_rotation[2],
                                         -start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> delta(end_translation[0] - start_translation[0],
                                     end_translation[1] - start_translation[1],
                                     end_translation[2] - start_translation[2]);
  const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

  const Eigen::Quaternion<T> h_rotation_inverse =
      Eigen::Quaternion<T>(end_rotation[0], -end_rotation[1], -end_rotation[2],
                           -end_rotation[3]) *
      Eigen::Quaternion<T>(start_rotation[0], start_rotation[1],
                           start_rotation[2], start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> angle_axis_difference =
      transform::RotationQuaternionToAngleAxisVector(
          h_rotation_inverse * relative_pose.rotation().cast<T>());

  return {{T(relative_pose.translation().x()) - h_translation[0],
           T(relative_pose.translation().y()) - h_translation[1],
           T(relative_pose.translation().z()) - h_translation[2],
           angle_axis_difference[0], angle_axis_difference[1],
           angle_axis_difference[2]}};
}

template <typename T>
std::array<T, 6> ScaleError(std::array<T, 6> error, T translation_weight,
                            T rotation_weight) {
  std::array<T, 6> scaled_error(std::move(error));
  scaled_error[0] *= translation_weight;
  scaled_error[1] *= translation_weight;
  scaled_error[2] *= translation_weight;
  scaled_error[3] *= rotation_weight;
  scaled_error[4] *= rotation_weight;
  scaled_error[5] *= rotation_weight;
  return scaled_error;
}

//  Eigen implementation of slerp is not compatible with Ceres on all supported
//  platforms. Our own implementation is used instead.
template <typename T>
std::array<T, 4> SlerpQuaternions(const T* const start, const T* const end,
                                  T factor) {
  // Angle 'theta' is the half-angle "between" quaternions. It can be computed
  // as the arccosine of their dot product.
  const T cos_theta = start[0] * end[0] + start[1] * end[1] +
                      start[2] * end[2] + start[3] * end[3];
  // Avoid using ::abs which would cast to integer.
  const T abs_cos_theta = ceres::abs(cos_theta);
  // If numerical error brings 'cos_theta' outside [-1 + epsilon, 1 - epsilon]
  // interval, then the quaternions are likely to be collinear.
  T prev_scale = T(1.) - factor;
  T next_scale = factor;
  if (abs_cos_theta < T(1. - 1e-5)) {
    const T theta = acos(abs_cos_theta);
    const T sin_theta = sin(theta);
    prev_scale = sin(prev_scale * theta) / sin_theta;
    next_scale = sin(next_scale * theta) / sin_theta;
  }
  if (cos_theta < T(0.)) next_scale = -next_scale;
  return {{prev_scale * start[0] + next_scale * end[0],
           prev_scale * start[1] + next_scale * end[1],
           prev_scale * start[2] + next_scale * end[2],
           prev_scale * start[3] + next_scale * end[3]}};
}

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_POSE_GRAPH_COST_HELPERS_IMPL_H_
