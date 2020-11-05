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

#include <string>
#include <vector>

#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace {

TEST(ConfigurationFilesTest, ValidateMapBuilderOptions) {
  const std::string kCode = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      return MAP_BUILDER)text";
  EXPECT_NO_FATAL_FAILURE({
    auto file_resolver =
        ::absl::make_unique< ::cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{
                std::string(::cartographer::common::kSourceDirectory) +
                "/configuration_files"});
    ::cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
        kCode, std::move(file_resolver));
    ::cartographer::mapping::CreateMapBuilderOptions(&lua_parameter_dictionary);
  });
}

TEST(ConfigurationFilesTest, ValidateTrajectoryBuilderOptions) {
  const std::string kCode = R"text(
      include "trajectory_builder.lua"
      TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
      return TRAJECTORY_BUILDER)text";
  EXPECT_NO_FATAL_FAILURE({
    auto file_resolver =
        ::absl::make_unique< ::cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{
                std::string(::cartographer::common::kSourceDirectory) +
                "/configuration_files"});
    ::cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
        kCode, std::move(file_resolver));
    ::cartographer::mapping::CreateTrajectoryBuilderOptions(
        &lua_parameter_dictionary);
  });
}

}  // namespace
}  // namespace cartographer
