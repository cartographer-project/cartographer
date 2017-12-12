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
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr char kRangeSensorId[] = "range";
constexpr char kIMUSensorId[] = "imu";
constexpr double kDuration = 2.;
constexpr double kTimeStep = 0.1;
constexpr double kTravelDistance = 0.4;

std::vector<sensor::TimedPointCloudData> GenerateFakeRangeMeasurements() {
  std::vector<sensor::TimedPointCloudData> measurements;
  sensor::TimedPointCloud point_cloud;
  for (double angle = 0.; angle < M_PI; angle += 0.01) {
    constexpr double kRadius = 5;
    point_cloud.emplace_back(kRadius * std::cos(angle),
                             kRadius * std::sin(angle), 0., 0.);
  }
  const Eigen::Vector3f kDirection = Eigen::Vector3f(2., 1., 0.).normalized();
  const Eigen::Vector3f kVelocity = kTravelDistance / kDuration * kDirection;
  for (double elapsed_time = 0.; elapsed_time < kDuration;
       elapsed_time += kTimeStep) {
    common::Time time =
        common::FromUniversal(123) + common::FromSeconds(elapsed_time);
    transform::Rigid3f pose =
        transform::Rigid3f::Translation(elapsed_time * kVelocity);
    sensor::TimedPointCloud ranges =
        sensor::TransformTimedPointCloud(point_cloud, pose.inverse());
    measurements.emplace_back(
        sensor::TimedPointCloudData{time, Eigen::Vector3f::Zero(), ranges});
  }
  return measurements;
}

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
    // Global SLAM optimization is not executed.
    const std::string kMapBuilderLua = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      MAP_BUILDER.pose_graph.optimize_every_n_nodes = 0
      return MAP_BUILDER)text";
    auto map_builder_parameters = ResolveLuaParameters(kMapBuilderLua);
    map_builder_options_ =
        CreateMapBuilderOptions(map_builder_parameters.get());
    // Multiple submaps are created because of a small 'num_range_data'.
    const std::string kTrajectoryBuilderLua = R"text(
      include "trajectory_builder.lua"
      TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
      TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 5
      TRAJECTORY_BUILDER.trajectory_builder_3d.submaps.num_range_data = 5
      return TRAJECTORY_BUILDER)text";
    auto trajectory_builder_parameters =
        ResolveLuaParameters(kTrajectoryBuilderLua);
    trajectory_builder_options_ =
        CreateTrajectoryBuilderOptions(trajectory_builder_parameters.get());
  }

  void BuildMapBuilder() {
    map_builder_ = common::make_unique<MapBuilder>(map_builder_options_);
  }

  void SetOptionsTo3D() {
    map_builder_options_.set_use_trajectory_builder_2d(false);
    map_builder_options_.set_use_trajectory_builder_3d(true);
  }

  MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<const ::cartographer::mapping::NodeId>) {
      local_slam_result_poses_.push_back(local_pose);
    };
  }

  std::unique_ptr<MapBuilderInterface> map_builder_;
  proto::MapBuilderOptions map_builder_options_;
  proto::TrajectoryBuilderOptions trajectory_builder_options_;
  std::vector<::cartographer::transform::Rigid3d> local_slam_result_poses_;
};

TEST_F(MapBuilderTest, TrajectoryAddFinish2D) {
  BuildMapBuilder();
  const std::unordered_set<std::string> expected_sensor_ids = {kRangeSensorId};
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_builder_options_,
      nullptr /* local_slam_result_callback */);
  EXPECT_EQ(1, map_builder_->num_trajectory_builders());
  EXPECT_TRUE(map_builder_->GetTrajectoryBuilder(trajectory_id) != nullptr);
  EXPECT_TRUE(map_builder_->pose_graph() != nullptr);
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_TRUE(map_builder_->pose_graph()->IsTrajectoryFinished(trajectory_id));
}

TEST_F(MapBuilderTest, TrajectoryAddFinish3D) {
  SetOptionsTo3D();
  BuildMapBuilder();
  const std::unordered_set<std::string> expected_sensor_ids = {kRangeSensorId};
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_builder_options_,
      nullptr /* local_slam_result_callback */);
  EXPECT_EQ(1, map_builder_->num_trajectory_builders());
  EXPECT_TRUE(map_builder_->GetTrajectoryBuilder(trajectory_id) != nullptr);
  EXPECT_TRUE(map_builder_->pose_graph() != nullptr);
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_TRUE(map_builder_->pose_graph()->IsTrajectoryFinished(trajectory_id));
}

TEST_F(MapBuilderTest, LocalSlam2D) {
  BuildMapBuilder();
  const std::unordered_set<std::string> expected_sensor_ids = {kRangeSensorId};
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = GenerateFakeRangeMeasurements();
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId, measurement);
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
}

TEST_F(MapBuilderTest, LocalSlam3D) {
  SetOptionsTo3D();
  BuildMapBuilder();
  const std::unordered_set<std::string> expected_sensor_ids = {kRangeSensorId,
                                                               kIMUSensorId};
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = GenerateFakeRangeMeasurements();
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId, measurement);
    trajectory_builder->AddSensorData(
        kIMUSensorId,
        sensor::ImuData{measurement.time, Eigen::Vector3d(0., 0., 9.8),
                        Eigen::Vector3d::Zero()});
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
