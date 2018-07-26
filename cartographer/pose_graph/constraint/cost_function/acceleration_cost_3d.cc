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

#include "cartographer/pose_graph/constraint/cost_function/acceleration_cost_3d.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace pose_graph {

AccelerationCost3D::AccelerationCost3D(
    const proto::Acceleration3D::Parameters& parameters)
    : delta_velocity_imu_frame_(
          transform::ToEigen(parameters.delta_velocity_imu_frame())),
      first_to_second_delta_time_seconds_(
          parameters.first_to_second_delta_time_seconds()),
      second_to_third_delta_time_seconds_(
          parameters.second_to_third_delta_time_seconds()),
      scaling_factor_(parameters.scaling_factor()) {}

proto::Acceleration3D::Parameters AccelerationCost3D::ToProto() const {
  proto::Acceleration3D::Parameters parameters;
  *parameters.mutable_delta_velocity_imu_frame() =
      transform::ToProto(delta_velocity_imu_frame_);
  parameters.set_first_to_second_delta_time_seconds(
      first_to_second_delta_time_seconds_);
  parameters.set_second_to_third_delta_time_seconds(
      second_to_third_delta_time_seconds_);
  parameters.set_scaling_factor(scaling_factor_);
  return parameters;
}

}  // namespace pose_graph
}  // namespace cartographer
