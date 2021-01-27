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

#include "cartographer/mapping/pose_extrapolator_interface.h"

#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/imu_based_pose_extrapolator.h"
#include "cartographer/mapping/pose_extrapolator.h"

namespace cartographer {
namespace mapping {

namespace {

proto::ConstantVelocityPoseExtrapolatorOptions
CreateConstantVelocityPoseExtrapolatorOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::ConstantVelocityPoseExtrapolatorOptions options;
  options.set_pose_queue_duration(
      parameter_dictionary->GetDouble("pose_queue_duration"));
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  return options;
}

proto::ImuBasedPoseExtrapolatorOptions CreateImuBasedPoseExtrapolatorOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::ImuBasedPoseExtrapolatorOptions options;

  options.set_pose_queue_duration(
      parameter_dictionary->GetDouble("pose_queue_duration"));
  options.set_gravity_constant(
      parameter_dictionary->GetDouble("gravity_constant"));
  options.set_pose_translation_weight(
      parameter_dictionary->GetDouble("pose_translation_weight"));
  options.set_pose_rotation_weight(
      parameter_dictionary->GetDouble("pose_rotation_weight"));
  options.set_imu_acceleration_weight(
      parameter_dictionary->GetDouble("imu_acceleration_weight"));
  options.set_imu_rotation_weight(
      parameter_dictionary->GetDouble("imu_rotation_weight"));
  options.set_odometry_rotation_weight(
      parameter_dictionary->GetDouble("odometry_rotation_weight"));
  options.set_odometry_translation_weight(
      parameter_dictionary->GetDouble("odometry_translation_weight"));
  *options.mutable_solver_options() = CreateCeresSolverOptionsProto(
      parameter_dictionary->GetDictionary("solver_options").get());

  return options;
}

}  // namespace

proto::PoseExtrapolatorOptions CreatePoseExtrapolatorOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::PoseExtrapolatorOptions options;
  options.set_use_imu_based(parameter_dictionary->GetBool("use_imu_based"));
  *options.mutable_constant_velocity() =
      CreateConstantVelocityPoseExtrapolatorOptions(
          parameter_dictionary->GetDictionary("constant_velocity").get());
  *options.mutable_imu_based() = CreateImuBasedPoseExtrapolatorOptions(
      parameter_dictionary->GetDictionary("imu_based").get());

  return options;
}

std::unique_ptr<PoseExtrapolatorInterface>
PoseExtrapolatorInterface::CreateWithImuData(
    const proto::PoseExtrapolatorOptions& options,
    const std::vector<sensor::ImuData>& imu_data,
    const std::vector<transform::TimestampedTransform>& initial_poses) {
  CHECK(!imu_data.empty());
  if (options.use_imu_based()) {
    return ImuBasedPoseExtrapolator::InitializeWithImu(options.imu_based(),
                                                       imu_data, initial_poses);
  } else {
    return PoseExtrapolator::InitializeWithImu(
        common::FromSeconds(options.constant_velocity().pose_queue_duration()),
        options.constant_velocity().imu_gravity_time_constant(),
        imu_data.back());
  }
}

}  // namespace mapping
}  // namespace cartographer
