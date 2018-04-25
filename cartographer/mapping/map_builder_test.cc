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

#include "cartographer/common/config.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};
constexpr double kDuration = 4.;         // Seconds.
constexpr double kTimeStep = 0.1;        // Seconds.
constexpr double kTravelDistance = 1.2;  // Meters.

class MapBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Global SLAM optimization is not executed.
    const std::string kMapBuilderLua = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      MAP_BUILDER.pose_graph.optimize_every_n_nodes = 0
      return MAP_BUILDER)text";
    auto map_builder_parameters = test::ResolveLuaParameters(kMapBuilderLua);
    map_builder_options_ =
        CreateMapBuilderOptions(map_builder_parameters.get());
    // Multiple submaps are created because of a small 'num_range_data'.
    const std::string kTrajectoryBuilderLua = R"text(
      include "trajectory_builder.lua"
      TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
      TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 4
      TRAJECTORY_BUILDER.trajectory_builder_3d.submaps.num_range_data = 4
      return TRAJECTORY_BUILDER)text";
    auto trajectory_builder_parameters =
        test::ResolveLuaParameters(kTrajectoryBuilderLua);
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

  void SetOptionsEnableGlobalOptimization() {
    map_builder_options_.mutable_pose_graph_options()
        ->set_optimize_every_n_nodes(3);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_motion_filter_options()
        ->set_max_distance_meters(0);
  }

  MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
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
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      nullptr /* GetLocalSlamResultCallbackForSubscriptions */);
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
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      nullptr /* GetLocalSlamResultCallbackForSubscriptions */);
  EXPECT_EQ(1, map_builder_->num_trajectory_builders());
  EXPECT_TRUE(map_builder_->GetTrajectoryBuilder(trajectory_id) != nullptr);
  EXPECT_TRUE(map_builder_->pose_graph() != nullptr);
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_TRUE(map_builder_->pose_graph()->IsTrajectoryFinished(trajectory_id));
}

TEST_F(MapBuilderTest, LocalSlam2D) {
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = test::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
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
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId, kIMUSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = test::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
    trajectory_builder->AddSensorData(
        kIMUSensorId.id,
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

TEST_F(MapBuilderTest, GlobalSlam2D) {
  SetOptionsEnableGlobalOptimization();
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = test::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
  EXPECT_GE(map_builder_->pose_graph()->constraints().size(), 50);
  const auto trajectory_nodes =
      map_builder_->pose_graph()->GetTrajectoryNodes();
  EXPECT_GE(trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 20);
  const auto submap_data = map_builder_->pose_graph()->GetAllSubmapData();
  EXPECT_GE(submap_data.SizeOfTrajectoryOrZero(trajectory_id), 5);
  const transform::Rigid3d final_pose =
      map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) *
      local_slam_result_poses_.back();
  EXPECT_NEAR(kTravelDistance, final_pose.translation().norm(),
              0.1 * kTravelDistance);
}

TEST_F(MapBuilderTest, GlobalSlam3D) {
  SetOptionsTo3D();
  SetOptionsEnableGlobalOptimization();
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId, kIMUSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = test::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
    trajectory_builder->AddSensorData(
        kIMUSensorId.id,
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
  EXPECT_GE(map_builder_->pose_graph()->constraints().size(), 10);
  const auto trajectory_nodes =
      map_builder_->pose_graph()->GetTrajectoryNodes();
  EXPECT_GE(trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 5);
  const auto submap_data = map_builder_->pose_graph()->GetAllSubmapData();
  EXPECT_GE(submap_data.SizeOfTrajectoryOrZero(trajectory_id), 2);
  const transform::Rigid3d final_pose =
      map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) *
      local_slam_result_poses_.back();
  EXPECT_NEAR(kTravelDistance, final_pose.translation().norm(),
              0.1 * kTravelDistance);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
