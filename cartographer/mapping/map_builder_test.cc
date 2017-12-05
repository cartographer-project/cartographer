/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/map_builder.h"

#include <functional>
#include <string>
#include <vector>

#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/trajectory_builder.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

std::unique_ptr<::cartographer::common::LuaParameterDictionary>
ResolveLuaParameters(const std::string& lua_code) {
  auto file_resolver = ::cartographer::common::make_unique<
      ::cartographer::common::ConfigurationFileResolver>(
      std::vector<std::string>{
          std::string(::cartographer::common::kSourceDirectory) +
          "/configuration_files"});
  return common::make_unique<::cartographer::common::LuaParameterDictionary>(
      lua_code, std::move(file_resolver));
}

class MapBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string kMapBuilderLua = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      return MAP_BUILDER)text";
    auto parameter_dictionary = ResolveLuaParameters(kMapBuilderLua);
    proto::MapBuilderOptions options =
        CreateMapBuilderOptions(parameter_dictionary.get());
    map_builder_ = common::make_unique<MapBuilder>(options);
  }

  std::unique_ptr<MapBuilderInterface> map_builder_;
};

TEST_F(MapBuilderTest, TrajectoryAddFinish) {
  const std::string kRangeSensorId = "lidar";
  const std::string kTrajectoryBuilderLua = R"text(
      include "trajectory_builder.lua"
      TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
      return TRAJECTORY_BUILDER)text";
  const std::unordered_set<std::string> expected_sensor_ids = {kRangeSensorId};
  auto trajectory_builder_parameters =
      ResolveLuaParameters(kTrajectoryBuilderLua);
  proto::TrajectoryBuilderOptions trajectory_options =
      CreateTrajectoryBuilderOptions(trajectory_builder_parameters.get());
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options,
      nullptr /* local_slam_result_callback */);
  EXPECT_EQ(1, map_builder_->num_trajectory_builders());
  EXPECT_TRUE(map_builder_->GetTrajectoryBuilder(trajectory_id) != nullptr);
  EXPECT_TRUE(map_builder_->pose_graph() != nullptr);
  map_builder_->FinishTrajectory(trajectory_id);
  EXPECT_TRUE(map_builder_->pose_graph()->IsTrajectoryFinished(trajectory_id));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
