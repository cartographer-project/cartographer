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

#include <condition_variable>
#include <mutex>

#include "cartographer/internal/mapping/test_helpers.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer_grpc/map_builder_server.h"
#include "cartographer_grpc/map_builder_server_options.h"
#include "cartographer_grpc/mapping/map_builder_stub.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using cartographer::mapping::MapBuilder;
using cartographer::mapping::MapBuilderInterface;
using cartographer::mapping::PoseGraphInterface;
using cartographer::mapping::TrajectoryBuilderInterface;
using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
using testing::_;

namespace cartographer_grpc {
namespace {

const SensorId kSensorId{SensorId::SensorType::IMU, "sensor"};
const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
constexpr double kDuration = 4.;         // Seconds.
constexpr double kTimeStep = 0.1;        // Seconds.
constexpr double kTravelDistance = 1.2;  // Meters.

class MockMapBuilder : public cartographer::mapping::MapBuilderInterface {
 public:
  MOCK_METHOD3(AddTrajectoryBuilder,
               int(const std::set<SensorId>& expected_sensor_ids,
                   const cartographer::mapping::proto::TrajectoryBuilderOptions&
                       trajectory_options,
                   LocalSlamResultCallback local_slam_result_callback));
  MOCK_METHOD0(AddTrajectoryForDeserialization, int());
  MOCK_CONST_METHOD1(GetTrajectoryBuilder,
                     TrajectoryBuilderInterface*(int trajectory_id));
  MOCK_METHOD1(FinishTrajectory, void(int trajectory_id));
  MOCK_METHOD2(
      SubmapToProto,
      std::string(const cartographer::mapping::SubmapId&,
                  cartographer::mapping::proto::SubmapQuery::Response*));
  MOCK_METHOD1(SerializeState,
               void(cartographer::io::ProtoStreamWriterInterface*));
  MOCK_METHOD1(LoadMap, void(cartographer::io::ProtoStreamReaderInterface*));
  MOCK_CONST_METHOD0(num_trajectory_builders, int());
  MOCK_METHOD0(pose_graph, PoseGraphInterface*());
};

class MockTrajectoryBuilder
    : public cartographer::mapping::TrajectoryBuilderInterface {
 public:
  MockTrajectoryBuilder() = default;
  ~MockTrajectoryBuilder() override = default;

  MOCK_METHOD2(AddSensorData,
               void(const std::string&,
                    const cartographer::sensor::TimedPointCloudData&));
  MOCK_METHOD2(AddSensorData,
               void(const std::string&, const cartographer::sensor::ImuData&));
  MOCK_METHOD2(AddSensorData, void(const std::string&,
                                   const cartographer::sensor::OdometryData&));
  MOCK_METHOD2(AddSensorData,
               void(const std::string&,
                    const cartographer::sensor::FixedFramePoseData&));
  MOCK_METHOD2(AddSensorData, void(const std::string&,
                                   const cartographer::sensor::LandmarkData&));

  // Some of the platforms we run on may ship with a version of gmock which does
  // not yet support move-only types.
  MOCK_METHOD1(DoAddLocalSlamResultData,
               void(cartographer::mapping::LocalSlamResultData*));
  void AddLocalSlamResultData(
      std::unique_ptr<cartographer::mapping::LocalSlamResultData>
          local_slam_result_data) override {
    DoAddLocalSlamResultData(local_slam_result_data.get());
  }
};

class ClientServerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // TODO(cschuet): Due to the hard-coded addresses these tests will become
    // flaky when run in parallel.
    const std::string kMapBuilderServerLua = R"text(
      include "map_builder_server.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      MAP_BUILDER.pose_graph.optimize_every_n_nodes = 0
      MAP_BUILDER_SERVER.num_event_threads = 1
      MAP_BUILDER_SERVER.num_grpc_threads = 1
      MAP_BUILDER_SERVER.uplink_server_address = ""
      return MAP_BUILDER_SERVER)text";
    auto map_builder_server_parameters =
        cartographer::mapping::test::ResolveLuaParameters(kMapBuilderServerLua);
    map_builder_server_options_ =
        CreateMapBuilderServerOptions(map_builder_server_parameters.get());
    const std::string kTrajectoryBuilderLua = R"text(
      include "trajectory_builder.lua"
      TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
      TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 4
      return TRAJECTORY_BUILDER)text";
    auto trajectory_builder_parameters =
        cartographer::mapping::test::ResolveLuaParameters(
            kTrajectoryBuilderLua);
    trajectory_builder_options_ =
        cartographer::mapping::CreateTrajectoryBuilderOptions(
            trajectory_builder_parameters.get());
    local_slam_result_callback_ =
        [this](
            int, cartographer::common::Time,
            cartographer::transform::Rigid3d local_pose,
            cartographer::sensor::RangeData,
            std::unique_ptr<const cartographer::mapping::
                                TrajectoryBuilderInterface::InsertionResult>) {
          std::unique_lock<std::mutex> lock(local_slam_result_mutex_);
          local_slam_result_poses_.push_back(local_pose);
          lock.unlock();
          local_slam_result_condition_.notify_all();
        };
  }

  void InitializeRealServer() {
    auto map_builder = cartographer::common::make_unique<MapBuilder>(
        map_builder_server_options_.map_builder_options());
    server_ = cartographer::common::make_unique<MapBuilderServer>(
        map_builder_server_options_, std::move(map_builder));
    EXPECT_TRUE(server_ != nullptr);
  }

  void InitializeServerWithMockMapBuilder() {
    auto mock_map_builder = cartographer::common::make_unique<MockMapBuilder>();
    mock_map_builder_ = mock_map_builder.get();
    server_ = cartographer::common::make_unique<MapBuilderServer>(
        map_builder_server_options_, std::move(mock_map_builder));
    EXPECT_TRUE(server_ != nullptr);
    mock_trajectory_builder_ =
        cartographer::common::make_unique<MockTrajectoryBuilder>();
  }

  void InitializeStub() {
    stub_ = cartographer::common::make_unique<mapping::MapBuilderStub>(
        map_builder_server_options_.server_address());
    EXPECT_TRUE(stub_ != nullptr);
  }

  void WaitForLocalSlamResults(size_t size) {
    std::unique_lock<std::mutex> lock(local_slam_result_mutex_);
    local_slam_result_condition_.wait(
        lock, [&] { return local_slam_result_poses_.size() >= size; });
  }

  proto::MapBuilderServerOptions map_builder_server_options_;
  MockMapBuilder* mock_map_builder_;
  std::unique_ptr<MockTrajectoryBuilder> mock_trajectory_builder_;
  cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options_;
  std::unique_ptr<MapBuilderServer> server_;
  std::unique_ptr<mapping::MapBuilderStub> stub_;
  TrajectoryBuilderInterface::LocalSlamResultCallback
      local_slam_result_callback_;
  std::condition_variable local_slam_result_condition_;
  std::mutex local_slam_result_mutex_;
  std::vector<cartographer::transform::Rigid3d> local_slam_result_poses_;
};

TEST_F(ClientServerTest, StartAndStopServer) {
  InitializeRealServer();
  server_->Start();
  server_->Shutdown();
}

TEST_F(ClientServerTest, AddTrajectoryBuilder) {
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  int trajectory_id = stub_->AddTrajectoryBuilder(
      {kSensorId}, trajectory_builder_options_, nullptr);
  stub_->FinishTrajectory(trajectory_id);
  server_->Shutdown();
}

TEST_F(ClientServerTest, AddTrajectoryBuilderWithMock) {
  InitializeServerWithMockMapBuilder();
  server_->Start();
  InitializeStub();
  std::set<SensorId> expected_sensor_ids = {kSensorId};
  EXPECT_CALL(
      *mock_map_builder_,
      AddTrajectoryBuilder(testing::ContainerEq(expected_sensor_ids), _, _))
      .WillOnce(testing::Return(3));
  EXPECT_CALL(*mock_map_builder_, GetTrajectoryBuilder(_))
      .WillRepeatedly(testing::Return(nullptr));
  int trajectory_id = stub_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_builder_options_, nullptr);
  EXPECT_EQ(trajectory_id, 3);
  EXPECT_CALL(*mock_map_builder_, FinishTrajectory(trajectory_id));
  stub_->FinishTrajectory(trajectory_id);
  server_->Shutdown();
}

TEST_F(ClientServerTest, AddSensorData) {
  trajectory_builder_options_.mutable_trajectory_builder_2d_options()
      ->set_use_imu_data(true);
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  int trajectory_id = stub_->AddTrajectoryBuilder(
      {kSensorId}, trajectory_builder_options_, nullptr);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  cartographer::sensor::ImuData imu_data{
      cartographer::common::FromUniversal(42), Eigen::Vector3d(0., 0., 9.8),
      Eigen::Vector3d::Zero()};
  trajectory_stub->AddSensorData(kSensorId.id, imu_data);
  stub_->FinishTrajectory(trajectory_id);
  server_->Shutdown();
}

TEST_F(ClientServerTest, AddSensorDataWithMock) {
  InitializeServerWithMockMapBuilder();
  server_->Start();
  InitializeStub();
  std::set<SensorId> expected_sensor_ids = {kSensorId};
  EXPECT_CALL(
      *mock_map_builder_,
      AddTrajectoryBuilder(testing::ContainerEq(expected_sensor_ids), _, _))
      .WillOnce(testing::Return(3));
  int trajectory_id = stub_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_builder_options_, nullptr);
  EXPECT_EQ(trajectory_id, 3);
  EXPECT_CALL(*mock_map_builder_, GetTrajectoryBuilder(_))
      .WillRepeatedly(testing::Return(mock_trajectory_builder_.get()));
  cartographer::mapping::TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  cartographer::sensor::ImuData imu_data{
      cartographer::common::FromUniversal(42), Eigen::Vector3d(0., 0., 9.8),
      Eigen::Vector3d::Zero()};
  EXPECT_CALL(
      *mock_trajectory_builder_,
      AddSensorData(testing::StrEq(kSensorId.id),
                    testing::Matcher<const cartographer::sensor::ImuData&>(_)))
      .WillOnce(testing::Return());
  trajectory_stub->AddSensorData(kSensorId.id, imu_data);
  EXPECT_CALL(*mock_map_builder_, FinishTrajectory(trajectory_id));
  stub_->FinishTrajectory(trajectory_id);
  server_->Shutdown();
}

TEST_F(ClientServerTest, LocalSlam2D) {
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  int trajectory_id =
      stub_->AddTrajectoryBuilder({kRangeSensorId}, trajectory_builder_options_,
                                  local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements =
      cartographer::mapping::test::GenerateFakeRangeMeasurements(
          kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurement);
  }
  WaitForLocalSlamResults(measurements.size());
  stub_->FinishTrajectory(trajectory_id);
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
  server_->Shutdown();
}

}  // namespace
}  // namespace cartographer_grpc
