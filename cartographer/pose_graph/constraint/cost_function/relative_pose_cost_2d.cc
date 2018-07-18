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

#include "cartographer/pose_graph/constraint/cost_function/relative_pose_cost_2d.h"

namespace cartographer {
namespace pose_graph {

RelativePoseCost2D::RelativePoseCost2D(
    const proto::RelativePose2D::Parameters& parameters)
    : translation_weight_(parameters.translation_weight()),
      rotation_weight_(parameters.rotation_weight()),
      first_T_second_(transform::ToRigid2(parameters.first_t_second())) {}

proto::RelativePose2D::Parameters RelativePoseCost2D::ToProto() const {
  proto::RelativePose2D::Parameters parameters;
  parameters.set_translation_weight(translation_weight_);
  parameters.set_rotation_weight(rotation_weight_);
  *parameters.mutable_first_t_second() = transform::ToProto(first_T_second_);
  return parameters;
}

bool RelativePoseCost2D::Evaluate(double const* const* parameters,
                                  double* residuals, double** jacobians) const {
  double const* start = parameters[0];
  double const* end = parameters[1];

  const double cos_start_rotation = cos(start[2]);
  const double sin_start_rotation = sin(start[2]);
  const double delta_x = end[0] - start[0];
  const double delta_y = end[1] - start[1];

  residuals[0] =
      translation_weight_ *
      (first_T_second_.translation().x() -
       (cos_start_rotation * delta_x + sin_start_rotation * delta_y));
  residuals[1] =
      translation_weight_ *
      (first_T_second_.translation().y() -
       (-sin_start_rotation * delta_x + cos_start_rotation * delta_y));
  residuals[2] = rotation_weight_ *
                 common::NormalizeAngleDifference(
                     first_T_second_.rotation().angle() - (end[2] - start[2]));
  if (jacobians == nullptr) return true;

  const double weighted_cos_start_rotation =
      translation_weight_ * cos_start_rotation;
  const double weighted_sin_start_rotation =
      translation_weight_ * sin_start_rotation;

  // Jacobians in Ceres are ordered by the parameter blocks:
  // jacobian[i] = [(dr_0 / dx_i)^T, ..., (dr_n / dx_i)^T].
  if (jacobians[0] != nullptr) {
    jacobians[0][0] = weighted_cos_start_rotation;
    jacobians[0][1] = weighted_sin_start_rotation;
    jacobians[0][2] = weighted_sin_start_rotation * delta_x -
                      weighted_cos_start_rotation * delta_y;
    jacobians[0][3] = -weighted_sin_start_rotation;
    jacobians[0][4] = weighted_cos_start_rotation;
    jacobians[0][5] = weighted_cos_start_rotation * delta_x +
                      weighted_sin_start_rotation * delta_y;
    jacobians[0][6] = 0;
    jacobians[0][7] = 0;
    jacobians[0][8] = rotation_weight_;
  }
  if (jacobians[1] != nullptr) {
    jacobians[1][0] = -weighted_cos_start_rotation;
    jacobians[1][1] = -weighted_sin_start_rotation;
    jacobians[1][2] = 0;
    jacobians[1][3] = weighted_sin_start_rotation;
    jacobians[1][4] = -weighted_cos_start_rotation;
    jacobians[1][5] = 0;
    jacobians[1][6] = 0;
    jacobians[1][7] = 0;
    jacobians[1][8] = -rotation_weight_;
  }
  return true;
}

}  // namespace pose_graph
}  // namespace cartographer
