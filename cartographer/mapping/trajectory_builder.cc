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

#include "cartographer/mapping/trajectory_builder.h"

#include "cartographer/mapping_2d/local_trajectory_builder_options.h"
#include "cartographer/mapping_3d/local_trajectory_builder_options.h"

namespace cartographer {
namespace mapping {

proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::TrajectoryBuilderOptions options;
  *options.mutable_trajectory_builder_2d_options() =
      mapping_2d::CreateLocalTrajectoryBuilderOptions(
          parameter_dictionary->GetDictionary("trajectory_builder_2d").get());
  *options.mutable_trajectory_builder_3d_options() =
      mapping_3d::CreateLocalTrajectoryBuilderOptions(
          parameter_dictionary->GetDictionary("trajectory_builder_3d").get());
  options.set_pure_localization(
      parameter_dictionary->GetBool("pure_localization"));
  return options;
}

}  // namespace mapping
}  // namespace cartographer
