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

#include "cartographer/mapping_3d/optimizing_local_trajectory_builder_options.h"

namespace cartographer {
namespace mapping_3d {

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
  return options;
}

}  // namespace mapping_3d
}  // namespace cartographer
