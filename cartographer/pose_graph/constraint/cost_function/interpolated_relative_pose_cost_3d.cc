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

#include "cartographer/pose_graph/constraint/cost_function/interpolated_relative_pose_cost_3d.h"

namespace cartographer {
namespace pose_graph {

InterpolatedRelativePoseCost3D::InterpolatedRelativePoseCost3D(
    const proto::InterpolatedRelativePose3D::Parameters& parameters)
    : translation_weight_(parameters.translation_weight()),
      rotation_weight_(parameters.rotation_weight()),
      interpolation_factor_(parameters.interpolation_factor()),
      first_T_second_(transform::ToRigid3(parameters.first_t_second())) {}

proto::InterpolatedRelativePose3D::Parameters
InterpolatedRelativePoseCost3D::ToProto() const {
  proto::InterpolatedRelativePose3D::Parameters parameters;
  parameters.set_translation_weight(translation_weight_);
  parameters.set_rotation_weight(rotation_weight_);
  parameters.set_interpolation_factor(interpolation_factor_);
  *parameters.mutable_first_t_second() = transform::ToProto(first_T_second_);
  return parameters;
}

}  // namespace pose_graph
}  // namespace cartographer