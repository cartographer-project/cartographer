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

#include "cartographer/pose_graph/constraint/cost_function/interpolated_relative_pose_cost_2d.h"

namespace cartographer {
namespace pose_graph {

InterpolatedRelativePoseCost2D::InterpolatedRelativePoseCost2D(
    const proto::InterpolatedRelativePose2D::Parameters& parameters)
    : translation_weight_(parameters.translation_weight()),
      rotation_weight_(parameters.rotation_weight()),
      interpolation_factor_(parameters.interpolation_factor()),
      first_T_second_(transform::ToRigid3(parameters.first_t_second())),
      gravity_alignment_first_start_(
          transform::ToEigen(parameters.gravity_alignment_first_start())),
      gravity_alignment_first_end_(
          transform::ToEigen(parameters.gravity_alignment_first_end())) {}

proto::InterpolatedRelativePose2D::Parameters
InterpolatedRelativePoseCost2D::ToProto() const {
  proto::InterpolatedRelativePose2D::Parameters parameters;
  parameters.set_translation_weight(translation_weight_);
  parameters.set_rotation_weight(rotation_weight_);
  parameters.set_interpolation_factor(interpolation_factor_);
  *parameters.mutable_first_t_second() = transform::ToProto(first_T_second_);
  *parameters.mutable_gravity_alignment_first_start() =
      transform::ToProto(gravity_alignment_first_start_);
  *parameters.mutable_gravity_alignment_first_end() =
      transform::ToProto(gravity_alignment_first_end_);
  return parameters;
}

}  // namespace pose_graph
}  // namespace cartographer
