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

#include "cartographer/mapping/internal/optimization/optimization_problem_options.h"

#include "cartographer/common/internal/ceres_solver_options.h"

namespace cartographer {
namespace mapping {
namespace optimization {

proto::OptimizationProblemOptions CreateOptimizationProblemOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::OptimizationProblemOptions options;
  options.set_huber_scale(parameter_dictionary->GetDouble("huber_scale"));
  options.set_acceleration_weight(
      parameter_dictionary->GetDouble("acceleration_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_odometry_translation_weight(
      parameter_dictionary->GetDouble("odometry_translation_weight"));
  options.set_odometry_rotation_weight(
      parameter_dictionary->GetDouble("odometry_rotation_weight"));
  options.set_local_slam_pose_translation_weight(
      parameter_dictionary->GetDouble("local_slam_pose_translation_weight"));
  options.set_local_slam_pose_rotation_weight(
      parameter_dictionary->GetDouble("local_slam_pose_rotation_weight"));
  options.set_fixed_frame_pose_translation_weight(
      parameter_dictionary->GetDouble("fixed_frame_pose_translation_weight"));
  options.set_fixed_frame_pose_rotation_weight(
      parameter_dictionary->GetDouble("fixed_frame_pose_rotation_weight"));
  options.set_fixed_frame_pose_use_tolerant_loss(
      parameter_dictionary->GetBool("fixed_frame_pose_use_tolerant_loss"));
  options.set_fixed_frame_pose_tolerant_loss_param_a(
      parameter_dictionary->GetDouble("fixed_frame_pose_tolerant_loss_param_a"));
  options.set_fixed_frame_pose_tolerant_loss_param_b(
      parameter_dictionary->GetDouble("fixed_frame_pose_tolerant_loss_param_b"));
  options.set_log_solver_summary(
      parameter_dictionary->GetBool("log_solver_summary"));
  options.set_use_online_imu_extrinsics_in_3d(
      parameter_dictionary->GetBool("use_online_imu_extrinsics_in_3d"));
  options.set_fix_z_in_3d(parameter_dictionary->GetBool("fix_z_in_3d"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
