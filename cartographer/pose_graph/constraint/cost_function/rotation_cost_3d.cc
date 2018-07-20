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

#include "cartographer/pose_graph/constraint/cost_function/rotation_cost_3d.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace pose_graph {

RotationCost3D::RotationCost3D(const proto::Rotation3D::Parameters& parameters)
    : scaling_factor_(parameters.scaling_factor()),
      delta_rotation_imu_frame_(
          transform::ToEigen(parameters.delta_rotation_imu_frame())) {}

proto::Rotation3D::Parameters RotationCost3D::ToProto() const {
  proto::Rotation3D::Parameters parameters;
  parameters.set_scaling_factor(scaling_factor_);
  *parameters.mutable_delta_rotation_imu_frame() =
      transform::ToProto(delta_rotation_imu_frame_);
  return parameters;
}

}  // namespace pose_graph
}  // namespace cartographer