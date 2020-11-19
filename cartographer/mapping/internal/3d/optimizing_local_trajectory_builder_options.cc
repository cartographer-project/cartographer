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

#include "cartographer/mapping/internal/3d/optimizing_local_trajectory_builder_options.h"

namespace cartographer {
namespace mapping {

proto::OptimizingLocalTrajectoryBuilderOptions
CreateOptimizingLocalTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::OptimizingLocalTrajectoryBuilderOptions options;
  options.set_high_resolution_grid_weight(
      parameter_dictionary->GetDouble("high_resolution_grid_weight"));
  options.set_low_resolution_grid_weight(
      parameter_dictionary->GetDouble("low_resolution_grid_weight"));
  options.set_velocity_weight(
      parameter_dictionary->GetDouble("velocity_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_odometry_translation_weight(
      parameter_dictionary->GetDouble("odometry_translation_weight"));
  options.set_odometry_rotation_weight(
      parameter_dictionary->GetDouble("odometry_rotation_weight"));
  options.set_initialize_map_orientation_with_imu(
      parameter_dictionary->GetBool("initialize_map_orientation_with_imu"));
  options.set_calibrate_imu(parameter_dictionary->GetBool("calibrate_imu"));
  options.set_optimization_rate(
      parameter_dictionary->GetDouble("optimization_rate"));
  options.set_ct_window_horizon(
      parameter_dictionary->GetDouble("ct_window_horizon"));
  options.set_ct_window_rate(
      parameter_dictionary->GetDouble("ct_window_rate"));
  options.set_initialization_duration(
      parameter_dictionary->GetDouble("initialization_duration"));
  const std::string imu_integrator_string =
      parameter_dictionary->GetString("imu_integrator");
  proto::IMUIntegrator imu_integrator_type;
  CHECK(proto::IMUIntegrator_Parse(imu_integrator_string, &imu_integrator_type))
      << "Unknown OptimizingLocalTrajectoryBuilderOptions_IMUIntegrator type: "
      << imu_integrator_string;
  options.set_imu_integrator(imu_integrator_type);
  const std::string imu_cost_term_string =
      parameter_dictionary->GetString("imu_cost_term");
  proto::IMUCostTerm imu_cost_term_type;
  CHECK(proto::IMUCostTerm_Parse(imu_cost_term_string, &imu_cost_term_type))
      << "Unknown OptimizingLocalTrajectoryBuilderOptions_IMUCostTerm type: "
      << imu_cost_term_string;
  options.set_imu_cost_term(imu_cost_term_type);
  options.set_sync_control_points_with_range_data(
      parameter_dictionary->GetBool("sync_control_points_with_range_data"));
  options.set_use_adaptive_odometry_weights(
      parameter_dictionary->GetBool("use_adaptive_odometry_weights"));
  options.set_max_odometry_translation_weight(
      parameter_dictionary->GetDouble("max_odometry_translation_weight"));
  options.set_max_odometry_rotation_weight(
      parameter_dictionary->GetDouble("max_odometry_rotation_weight"));
  options.set_weight_odometry_translation_limit(
      parameter_dictionary->GetDouble("weight_odometry_translation_limit"));
  options.set_weight_odometry_rotation_limit(
      parameter_dictionary->GetDouble("weight_odometry_rotation_limit"));

  return options;
}

}  // namespace mapping
}  // namespace cartographer
