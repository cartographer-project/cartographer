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
#include "cartographer/transform/timestamped_transform.h"
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
      MAP_BUILDER.pose_graph.optimize_every_n_nodes = 1
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

TEST_F(MapBuilderTest, AddRangerfinder2D) {
  const std::string kRangeSensorId = "lidar";
  const std::string kTrajectoryBuilderLua = R"text(
      include "trajectory_builder.lua"
      TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
      TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 5
      return TRAJECTORY_BUILDER)text";
  const std::unordered_set<std::string> expected_sensor_ids = {kRangeSensorId};
  auto trajectory_builder_parameters =
      ResolveLuaParameters(kTrajectoryBuilderLua);
  proto::TrajectoryBuilderOptions trajectory_options =
      CreateTrajectoryBuilderOptions(trajectory_builder_parameters.get());
  MapBuilderInterface::LocalSlamResultCallback callback =
      [](const int trajectory_id, const ::cartographer::common::Time time,
         const ::cartographer::transform::Rigid3d local_pose,
         ::cartographer::sensor::RangeData range_data_in_local,
         const std::unique_ptr<const ::cartographer::mapping::NodeId>) {
        LOG(WARNING) << "local_pose: " << local_pose;
      };
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options, callback);
  EXPECT_EQ(1, map_builder_->num_trajectory_builders());
  EXPECT_TRUE(map_builder_->pose_graph() != nullptr);
  TrajectoryBuilder* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  EXPECT_TRUE(trajectory_builder != nullptr);

  const double kTimeStep = 0.1;
  sensor::TimedPointCloud point_cloud;
  for (double angle = 0.; angle < 1. * M_PI; angle += 0.01) {
    const double kRadius = 3;
    point_cloud.emplace_back(kRadius * std::cos(angle),
                             kRadius * std::sin(angle), 0., 0.);
  }
  for (int step = 0; step < 10; ++step) {
    double elapsed_time = step * kTimeStep;
    common::Time time =
        common::FromUniversal(12345678) + common::FromSeconds(elapsed_time);
    transform::Rigid3f pose = transform::Rigid3f::Translation(
        Eigen::Vector3f(elapsed_time * 0.5, elapsed_time * 0.05, 0.));
    sensor::TimedPointCloud ranges =
        sensor::TransformTimedPointCloud(point_cloud, pose.inverse());
    trajectory_builder->AddRangefinderData(kRangeSensorId, time,
                                           Eigen::Vector3f::Zero(), ranges);
  }

  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_TRUE(map_builder_->pose_graph()->IsTrajectoryFinished(trajectory_id));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
