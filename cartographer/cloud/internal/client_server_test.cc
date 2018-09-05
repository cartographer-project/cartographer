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

#include "cartographer/cloud/client/map_builder_stub.h"
#include "cartographer/cloud/internal/map_builder_server.h"
#include "cartographer/cloud/map_builder_server_options.h"
#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/testing/test_helpers.h"
#include "cartographer/mapping/internal/testing/mock_map_builder.h"
#include "cartographer/mapping/internal/testing/mock_pose_graph.h"
#include "cartographer/mapping/internal/testing/mock_trajectory_builder.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::cartographer::io::testing::ProtoReaderFromStrings;
using ::cartographer::mapping::MapBuilder;
using ::cartographer::mapping::MapBuilderInterface;
using ::cartographer::mapping::PoseGraphInterface;
using ::cartographer::mapping::TrajectoryBuilderInterface;
using ::cartographer::mapping::testing::MockMapBuilder;
using ::cartographer::mapping::testing::MockPoseGraph;
using ::cartographer::mapping::testing::MockTrajectoryBuilder;
using ::testing::_;
using SensorId = ::cartographer::mapping::TrajectoryBuilderInterface::SensorId;

namespace cartographer {
namespace cloud {
namespace {

constexpr char kClientId[] = "CLIENT_ID";
const SensorId kImuSensorId{SensorId::SensorType::IMU, "imu"};
const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
constexpr double kDuration = 4.;         // Seconds.
constexpr double kTimeStep = 0.1;        // Seconds.
constexpr double kTravelDistance = 1.2;  // Meters.

constexpr char kSerializationHeaderProtoString[] = "format_version: 1";
constexpr char kPoseGraphProtoString[] = R"(pose_graph {
      trajectory: {
        trajectory_id: 0
		node: {}
		submap: {}
	  }
    })";
constexpr char kAllTrajectoryBuilderOptionsProtoString[] =
    R"(all_trajectory_builder_options {
      options_with_sensor_ids: {}
    })";
constexpr char kSubmapProtoString[] = "submap {}";
constexpr char kNodeProtoString[] = "node {}";
constexpr char kTrajectoryDataProtoString[] = "trajectory_data {}";
constexpr char kImuDataProtoString[] = "imu_data {}";
constexpr char kOdometryDataProtoString[] = "odometry_data {}";
constexpr char kFixedFramePoseDataProtoString[] = "fixed_frame_pose_data {}";
constexpr char kLandmarkDataProtoString[] = "landmark_data {}";

template <class T>
class ClientServerTestBase : public T {
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
      MAP_BUILDER_SERVER.server_address = "0.0.0.0:50051"
      return MAP_BUILDER_SERVER)text";
    auto map_builder_server_parameters =
        mapping::testing::ResolveLuaParameters(kMapBuilderServerLua);
    map_builder_server_options_ =
        CreateMapBuilderServerOptions(map_builder_server_parameters.get());

    const std::string kUploadingMapBuilderServerLua = R"text(
      include "map_builder_server.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      MAP_BUILDER.pose_graph.optimize_every_n_nodes = 0
      MAP_BUILDER_SERVER.num_event_threads = 1
      MAP_BUILDER_SERVER.num_grpc_threads = 1
      MAP_BUILDER_SERVER.uplink_server_address = "localhost:50051"
      MAP_BUILDER_SERVER.server_address = "0.0.0.0:50052"
      MAP_BUILDER_SERVER.upload_batch_size = 1
      return MAP_BUILDER_SERVER)text";
    auto uploading_map_builder_server_parameters =
        mapping::testing::ResolveLuaParameters(kUploadingMapBuilderServerLua);
    uploading_map_builder_server_options_ = CreateMapBuilderServerOptions(
        uploading_map_builder_server_parameters.get());
    EXPECT_NE(map_builder_server_options_.server_address(),
              uploading_map_builder_server_options_.server_address());

    const std::string kTrajectoryBuilderLua = R"text(
      include "trajectory_builder.lua"
      TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
      TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 4
      return TRAJECTORY_BUILDER)text";
    auto trajectory_builder_parameters =
        mapping::testing::ResolveLuaParameters(kTrajectoryBuilderLua);
    trajectory_builder_options_ = mapping::CreateTrajectoryBuilderOptions(
        trajectory_builder_parameters.get());
    number_of_insertion_results_ = 0;
    local_slam_result_callback_ =
        [this](int, common::Time, transform::Rigid3d local_pose,
               sensor::RangeData,
               std::unique_ptr<
                   const mapping::TrajectoryBuilderInterface::InsertionResult>
                   insertion_result) {
          std::unique_lock<std::mutex> lock(local_slam_result_mutex_);
          if (insertion_result) {
            ++number_of_insertion_results_;
          }
          local_slam_result_poses_.push_back(local_pose);
          lock.unlock();
          local_slam_result_condition_.notify_all();
        };
  }

  void InitializeRealServer() {
    auto map_builder = absl::make_unique<MapBuilder>(
        map_builder_server_options_.map_builder_options());
    server_ = absl::make_unique<MapBuilderServer>(map_builder_server_options_,
                                                  std::move(map_builder));
    EXPECT_TRUE(server_ != nullptr);
  }

  void InitializeRealUploadingServer() {
    auto map_builder = absl::make_unique<MapBuilder>(
        uploading_map_builder_server_options_.map_builder_options());
    uploading_server_ = absl::make_unique<MapBuilderServer>(
        uploading_map_builder_server_options_, std::move(map_builder));
    EXPECT_TRUE(uploading_server_ != nullptr);
  }

  void InitializeServerWithMockMapBuilder() {
    auto mock_map_builder = absl::make_unique<MockMapBuilder>();
    mock_map_builder_ = mock_map_builder.get();
    mock_pose_graph_ = absl::make_unique<MockPoseGraph>();
    EXPECT_CALL(*mock_map_builder_, pose_graph())
        .WillOnce(::testing::Return(mock_pose_graph_.get()));
    EXPECT_CALL(*mock_pose_graph_, SetGlobalSlamOptimizationCallback(_));
    server_ = absl::make_unique<MapBuilderServer>(map_builder_server_options_,
                                                  std::move(mock_map_builder));
    EXPECT_TRUE(server_ != nullptr);
    mock_trajectory_builder_ = absl::make_unique<MockTrajectoryBuilder>();
  }

  void InitializeStub() {
    stub_ = absl::make_unique<MapBuilderStub>(
        map_builder_server_options_.server_address(), kClientId);
    EXPECT_TRUE(stub_ != nullptr);
  }

  void InitializeStubForUploadingServer() {
    stub_for_uploading_server_ = absl::make_unique<MapBuilderStub>(
        uploading_map_builder_server_options_.server_address(), kClientId);
    EXPECT_TRUE(stub_for_uploading_server_ != nullptr);
  }

  void SetOptionsToTSDF2D() {
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_submaps_options()
        ->mutable_range_data_inserter_options()
        ->set_range_data_inserter_type(
            ::cartographer::mapping::proto::RangeDataInserterOptions::
                TSDF_INSERTER_2D);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_submaps_options()
        ->mutable_grid_options_2d()
        ->set_grid_type(::cartographer::mapping::proto::GridOptions2D::TSDF);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_ceres_scan_matcher_options()
        ->set_occupied_space_weight(10.0);
    map_builder_server_options_.mutable_map_builder_options()
        ->mutable_pose_graph_options()
        ->mutable_constraint_builder_options()
        ->mutable_ceres_scan_matcher_options()
        ->set_occupied_space_weight(50.0);
    uploading_map_builder_server_options_.mutable_map_builder_options()
        ->mutable_pose_graph_options()
        ->mutable_constraint_builder_options()
        ->mutable_ceres_scan_matcher_options()
        ->set_occupied_space_weight(50.0);
  }

  void WaitForLocalSlamResults(size_t size) {
    std::unique_lock<std::mutex> lock(local_slam_result_mutex_);
    local_slam_result_condition_.wait(
        lock, [&] { return local_slam_result_poses_.size() >= size; });
  }

  void WaitForLocalSlamResultUploads(size_t size) {
    while (stub_->pose_graph()->GetTrajectoryNodePoses().size() < size) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  proto::MapBuilderServerOptions map_builder_server_options_;
  proto::MapBuilderServerOptions uploading_map_builder_server_options_;
  MockMapBuilder* mock_map_builder_;
  std::unique_ptr<MockPoseGraph> mock_pose_graph_;
  std::unique_ptr<MockTrajectoryBuilder> mock_trajectory_builder_;
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options_;
  std::unique_ptr<MapBuilderServer> server_;
  std::unique_ptr<MapBuilderServer> uploading_server_;
  std::unique_ptr<MapBuilderStub> stub_;
  std::unique_ptr<MapBuilderStub> stub_for_uploading_server_;
  TrajectoryBuilderInterface::LocalSlamResultCallback
      local_slam_result_callback_;
  std::condition_variable local_slam_result_condition_;
  std::condition_variable local_slam_result_upload_condition_;
  std::mutex local_slam_result_mutex_;
  std::mutex local_slam_result_upload_mutex_;
  std::vector<transform::Rigid3d> local_slam_result_poses_;
  int number_of_insertion_results_;
};

class ClientServerTest : public ClientServerTestBase<::testing::Test> {};
class ClientServerTestByGridType
    : public ClientServerTestBase<
          ::testing::TestWithParam<::cartographer::mapping::GridType>> {};

INSTANTIATE_TEST_CASE_P(
    ClientServerTestByGridType, ClientServerTestByGridType,
    ::testing::Values(::cartographer::mapping::GridType::PROBABILITY_GRID,
                      ::cartographer::mapping::GridType::TSDF));

TEST_F(ClientServerTest, StartAndStopServer) {
  InitializeRealServer();
  server_->Start();
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, AddTrajectoryBuilder) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  int trajectory_id = stub_->AddTrajectoryBuilder(
      {kImuSensorId}, trajectory_builder_options_, nullptr);
  EXPECT_FALSE(stub_->pose_graph()->IsTrajectoryFinished(trajectory_id));
  stub_->FinishTrajectory(trajectory_id);
  EXPECT_TRUE(stub_->pose_graph()->IsTrajectoryFinished(trajectory_id));
  EXPECT_FALSE(stub_->pose_graph()->IsTrajectoryFrozen(trajectory_id));
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, AddTrajectoryBuilderWithMock) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeServerWithMockMapBuilder();
  server_->Start();
  InitializeStub();
  std::set<SensorId> expected_sensor_ids = {kImuSensorId};
  EXPECT_CALL(
      *mock_map_builder_,
      AddTrajectoryBuilder(::testing::ContainerEq(expected_sensor_ids), _, _))
      .WillOnce(::testing::Return(3));
  EXPECT_CALL(*mock_map_builder_, GetTrajectoryBuilder(_))
      .WillRepeatedly(::testing::Return(nullptr));
  int trajectory_id = stub_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_builder_options_, nullptr);
  EXPECT_EQ(trajectory_id, 3);
  EXPECT_CALL(*mock_map_builder_, FinishTrajectory(trajectory_id));
  stub_->FinishTrajectory(trajectory_id);
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, AddSensorData) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  trajectory_builder_options_.mutable_trajectory_builder_2d_options()
      ->set_use_imu_data(true);
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  int trajectory_id = stub_->AddTrajectoryBuilder(
      {kImuSensorId}, trajectory_builder_options_, nullptr);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  sensor::ImuData imu_data{common::FromUniversal(42),
                           Eigen::Vector3d(0., 0., 9.8),
                           Eigen::Vector3d::Zero()};
  trajectory_stub->AddSensorData(kImuSensorId.id, imu_data);
  stub_->FinishTrajectory(trajectory_id);
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, AddSensorDataWithMock) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeServerWithMockMapBuilder();
  server_->Start();
  InitializeStub();
  std::set<SensorId> expected_sensor_ids = {kImuSensorId};
  EXPECT_CALL(
      *mock_map_builder_,
      AddTrajectoryBuilder(::testing::ContainerEq(expected_sensor_ids), _, _))
      .WillOnce(::testing::Return(3));
  int trajectory_id = stub_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_builder_options_, nullptr);
  EXPECT_EQ(trajectory_id, 3);
  EXPECT_CALL(*mock_map_builder_, GetTrajectoryBuilder(_))
      .WillRepeatedly(::testing::Return(mock_trajectory_builder_.get()));
  mapping::TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  sensor::ImuData imu_data{common::FromUniversal(42),
                           Eigen::Vector3d(0., 0., 9.8),
                           Eigen::Vector3d::Zero()};
  EXPECT_CALL(*mock_trajectory_builder_,
              AddSensorData(::testing::StrEq(kImuSensorId.id),
                            ::testing::Matcher<const sensor::ImuData&>(_)))
      .WillOnce(::testing::Return());
  trajectory_stub->AddSensorData(kImuSensorId.id, imu_data);
  EXPECT_CALL(*mock_map_builder_, FinishTrajectory(trajectory_id));
  stub_->FinishTrajectory(trajectory_id);
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, LocalSlam2D) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  EXPECT_TRUE(stub_->pose_graph()->GetTrajectoryStates().empty());
  int trajectory_id =
      stub_->AddTrajectoryBuilder({kRangeSensorId}, trajectory_builder_options_,
                                  local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurement);
  }
  WaitForLocalSlamResults(measurements.size());
  EXPECT_EQ(stub_->pose_graph()->GetTrajectoryStates().at(trajectory_id),
            PoseGraphInterface::TrajectoryState::ACTIVE);
  stub_->FinishTrajectory(trajectory_id);
  stub_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(stub_->pose_graph()->GetTrajectoryStates().at(trajectory_id),
            PoseGraphInterface::TrajectoryState::FINISHED);
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, LocalSlamAndDelete2D) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  int trajectory_id =
      stub_->AddTrajectoryBuilder({kRangeSensorId}, trajectory_builder_options_,
                                  local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurement);
  }
  WaitForLocalSlamResults(measurements.size());
  stub_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(stub_->pose_graph()->GetTrajectoryStates().at(trajectory_id),
            PoseGraphInterface::TrajectoryState::ACTIVE);
  EXPECT_GT(stub_->pose_graph()->GetAllSubmapPoses().size(), 0);
  EXPECT_GT(stub_->pose_graph()->GetTrajectoryNodePoses().size(), 0);
  stub_->FinishTrajectory(trajectory_id);
  stub_->pose_graph()->DeleteTrajectory(trajectory_id);
  stub_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(stub_->pose_graph()->GetTrajectoryStates().at(trajectory_id),
            PoseGraphInterface::TrajectoryState::DELETED);
  EXPECT_EQ(stub_->pose_graph()->GetAllSubmapPoses().size(), 0);
  EXPECT_EQ(stub_->pose_graph()->GetTrajectoryNodePoses().size(), 0);
  server_->Shutdown();
}

TEST_F(ClientServerTest, GlobalSlam3D) {
  map_builder_server_options_.mutable_map_builder_options()
      ->set_use_trajectory_builder_2d(false);
  map_builder_server_options_.mutable_map_builder_options()
      ->set_use_trajectory_builder_3d(true);
  map_builder_server_options_.mutable_map_builder_options()
      ->mutable_pose_graph_options()
      ->set_optimize_every_n_nodes(3);
  trajectory_builder_options_.mutable_trajectory_builder_3d_options()
      ->mutable_motion_filter_options()
      ->set_max_distance_meters(0);
  trajectory_builder_options_.mutable_trajectory_builder_3d_options()
      ->mutable_motion_filter_options()
      ->set_max_angle_radians(0);
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  int trajectory_id = stub_->AddTrajectoryBuilder(
      {kRangeSensorId, kImuSensorId}, trajectory_builder_options_,
      local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    sensor::ImuData imu_data{
        measurement.time - common::FromSeconds(kTimeStep / 2),
        Eigen::Vector3d(0., 0., 9.8), Eigen::Vector3d::Zero()};
    trajectory_stub->AddSensorData(kImuSensorId.id, imu_data);
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurement);
  }
  // First range data will not call back while initializing PoseExtrapolator, so
  // expect one less.
  WaitForLocalSlamResults(measurements.size() - 1);
  stub_->FinishTrajectory(trajectory_id);
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, StartAndStopUploadingServerAndServer) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeRealUploadingServer();
  uploading_server_->Start();
  uploading_server_->Shutdown();
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, AddTrajectoryBuilderWithUploadingServer) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeRealUploadingServer();
  uploading_server_->Start();
  InitializeStub();
  InitializeStubForUploadingServer();
  int trajectory_id = stub_for_uploading_server_->AddTrajectoryBuilder(
      {kImuSensorId}, trajectory_builder_options_, nullptr);
  EXPECT_FALSE(stub_for_uploading_server_->pose_graph()->IsTrajectoryFinished(
      trajectory_id));
  EXPECT_FALSE(stub_->pose_graph()->IsTrajectoryFinished(trajectory_id));
  stub_for_uploading_server_->FinishTrajectory(trajectory_id);
  EXPECT_TRUE(stub_for_uploading_server_->pose_graph()->IsTrajectoryFinished(
      trajectory_id));
  EXPECT_TRUE(stub_->pose_graph()->IsTrajectoryFinished(trajectory_id));
  EXPECT_FALSE(stub_for_uploading_server_->pose_graph()->IsTrajectoryFrozen(
      trajectory_id));
  EXPECT_FALSE(stub_->pose_graph()->IsTrajectoryFrozen(trajectory_id));
  uploading_server_->Shutdown();
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, LocalSlam2DWithUploadingServer) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  InitializeRealUploadingServer();
  uploading_server_->Start();
  InitializeStubForUploadingServer();
  int trajectory_id = stub_for_uploading_server_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_for_uploading_server_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurement);
  }
  WaitForLocalSlamResults(measurements.size());
  WaitForLocalSlamResultUploads(number_of_insertion_results_);

  std::queue<std::unique_ptr<google::protobuf::Message>> chunks;
  io::ForwardingProtoStreamWriter writer(
      [&chunks](const google::protobuf::Message* proto) -> bool {
        if (!proto) {
          return true;
        }
        std::unique_ptr<google::protobuf::Message> p(proto->New());
        p->CopyFrom(*proto);
        chunks.push(std::move(p));
        return true;
      });
  stub_->SerializeState(false, &writer);
  CHECK(writer.Close());
  // Ensure it can be read.
  io::InMemoryProtoStreamReader reader(std::move(chunks));
  io::ProtoStreamDeserializer deserializer(&reader);
  EXPECT_EQ(deserializer.pose_graph().trajectory_size(), 1);

  stub_for_uploading_server_->FinishTrajectory(trajectory_id);
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
  uploading_server_->Shutdown();
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, LocalSlam2DUplinkServerRestarting) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  InitializeRealUploadingServer();
  uploading_server_->Start();
  InitializeStubForUploadingServer();
  int trajectory_id = stub_for_uploading_server_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_for_uploading_server_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);

  // Insert half of the measurements.
  for (unsigned int i = 0; i < measurements.size() / 2; ++i) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurements.at(i));
  }
  WaitForLocalSlamResults(measurements.size() / 2);
  WaitForLocalSlamResultUploads(number_of_insertion_results_);

  // Simulate a cloud server restart.
  LOG(INFO) << "Simulating server restart.";
  constexpr int kUplinkTrajectoryId = 0;
  stub_->FinishTrajectory(kUplinkTrajectoryId);
  server_->Shutdown();
  server_->WaitForShutdown();
  InitializeRealServer();
  server_->Start();
  InitializeStub();

  // Insert the second half of the measurements.
  for (unsigned int i = measurements.size() / 2; i < measurements.size(); ++i) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurements.at(i));
  }

  WaitForLocalSlamResults(measurements.size() / 2);
  WaitForLocalSlamResultUploads(2);
  stub_for_uploading_server_->FinishTrajectory(trajectory_id);
  uploading_server_->Shutdown();
  uploading_server_->WaitForShutdown();
  server_->Shutdown();
  server_->WaitForShutdown();
}

TEST_F(ClientServerTest, DelayedConnectionToUplinkServer) {
  InitializeRealUploadingServer();
  uploading_server_->Start();
  InitializeStubForUploadingServer();
  int trajectory_id = stub_for_uploading_server_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_for_uploading_server_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);

  // Insert the first measurement.
  trajectory_stub->AddSensorData(kRangeSensorId.id, measurements.at(0));
  WaitForLocalSlamResults(1);

  LOG(INFO) << "Delayed start of uplink server.";
  InitializeRealServer();
  server_->Start();
  InitializeStub();

  // Insert all other measurements.
  for (unsigned int i = 1; i < measurements.size(); ++i) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurements.at(i));
  }
  WaitForLocalSlamResults(measurements.size());
  WaitForLocalSlamResultUploads(2);
  stub_for_uploading_server_->FinishTrajectory(trajectory_id);
  uploading_server_->Shutdown();
  uploading_server_->WaitForShutdown();
  server_->Shutdown();
  server_->WaitForShutdown();
}

TEST_P(ClientServerTestByGridType, LoadStateAndDelete) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeStub();

  // Load text proto into in_memory_reader.
  auto reader =
      ProtoReaderFromStrings(kSerializationHeaderProtoString,
                             {
                                 kPoseGraphProtoString,
                                 kAllTrajectoryBuilderOptionsProtoString,
                                 kSubmapProtoString,
                                 kNodeProtoString,
                                 kImuDataProtoString,
                                 kOdometryDataProtoString,
                                 kLandmarkDataProtoString,
                             });

  auto trajectory_remapping = stub_->LoadState(reader.get(), true);
  int expected_trajectory_id = 0;
  EXPECT_EQ(trajectory_remapping.size(), 1);
  EXPECT_EQ(trajectory_remapping.at(0), expected_trajectory_id);
  stub_->pose_graph()->RunFinalOptimization();
  EXPECT_TRUE(stub_->pose_graph()->IsTrajectoryFrozen(expected_trajectory_id));
  EXPECT_FALSE(
      stub_->pose_graph()->IsTrajectoryFinished(expected_trajectory_id));
  for (const auto& entry : trajectory_remapping) {
    int trajectory_id = entry.second;
    stub_->pose_graph()->DeleteTrajectory(trajectory_id);
    stub_->pose_graph()->RunFinalOptimization();
    EXPECT_EQ(stub_->pose_graph()->GetTrajectoryStates().at(trajectory_id),
              PoseGraphInterface::TrajectoryState::DELETED);
  }
  server_->Shutdown();
}

TEST_P(ClientServerTestByGridType, LoadUnfrozenStateAndDelete) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeStub();

  // Load text proto into in_memory_reader.
  auto reader =
      ProtoReaderFromStrings(kSerializationHeaderProtoString,
                             {
                                 kPoseGraphProtoString,
                                 kAllTrajectoryBuilderOptionsProtoString,
                                 kSubmapProtoString,
                                 kNodeProtoString,
                                 kImuDataProtoString,
                                 kOdometryDataProtoString,
                                 kLandmarkDataProtoString,
                             });

  auto trajectory_remapping =
      stub_->LoadState(reader.get(), false /* load_frozen_state */);
  int expected_trajectory_id = 0;
  EXPECT_EQ(trajectory_remapping.size(), 1);
  EXPECT_EQ(trajectory_remapping.at(0), expected_trajectory_id);
  stub_->pose_graph()->RunFinalOptimization();
  EXPECT_FALSE(stub_->pose_graph()->IsTrajectoryFrozen(expected_trajectory_id));
  EXPECT_FALSE(
      stub_->pose_graph()->IsTrajectoryFinished(expected_trajectory_id));
  EXPECT_EQ(
      stub_->pose_graph()->GetTrajectoryStates().at(expected_trajectory_id),
      PoseGraphInterface::TrajectoryState::ACTIVE);
  stub_->FinishTrajectory(expected_trajectory_id);
  EXPECT_EQ(
      stub_->pose_graph()->GetTrajectoryStates().at(expected_trajectory_id),
      PoseGraphInterface::TrajectoryState::FINISHED);
  for (const auto& entry : trajectory_remapping) {
    int trajectory_id = entry.second;
    stub_->pose_graph()->DeleteTrajectory(trajectory_id);
    stub_->pose_graph()->RunFinalOptimization();
    EXPECT_EQ(stub_->pose_graph()->GetTrajectoryStates().at(trajectory_id),
              PoseGraphInterface::TrajectoryState::DELETED);
  }
  server_->Shutdown();
}

// TODO(gaschler): Test-cover LoadStateFromFile.

TEST_P(ClientServerTestByGridType, LocalSlam2DHandlesInvalidRequests) {
  if (GetParam() == ::cartographer::mapping::GridType::TSDF) {
    SetOptionsToTSDF2D();
  }
  InitializeRealServer();
  server_->Start();
  InitializeStub();
  int trajectory_id =
      stub_->AddTrajectoryBuilder({kRangeSensorId}, trajectory_builder_options_,
                                  local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurement);
  }
  WaitForLocalSlamResults(measurements.size());
  stub_->pose_graph()->RunFinalOptimization();

  const int kInvalidTrajectoryId = 7;
  stub_->pose_graph()->DeleteTrajectory(kInvalidTrajectoryId);
  EXPECT_FALSE(stub_->pose_graph()->IsTrajectoryFinished(kInvalidTrajectoryId));
  EXPECT_FALSE(stub_->pose_graph()->IsTrajectoryFrozen(kInvalidTrajectoryId));
  EXPECT_EQ(nullptr, stub_->GetTrajectoryBuilder(kInvalidTrajectoryId));
  stub_->FinishTrajectory(kInvalidTrajectoryId);
  const mapping::SubmapId kInvalidSubmapId0{kInvalidTrajectoryId, 0},
      kInvalidSubmapId1{trajectory_id, 424242};
  mapping::proto::SubmapQuery::Response submap_query_response;
  // Expect that it returns non-empty error string.
  EXPECT_NE("",
            stub_->SubmapToProto(kInvalidSubmapId0, &submap_query_response));
  EXPECT_NE("",
            stub_->SubmapToProto(kInvalidSubmapId1, &submap_query_response));

  EXPECT_EQ(stub_->pose_graph()->GetTrajectoryStates().at(trajectory_id),
            PoseGraphInterface::TrajectoryState::ACTIVE);
  auto submap_poses = stub_->pose_graph()->GetAllSubmapPoses();
  EXPECT_GT(submap_poses.size(), 0);
  EXPECT_GT(stub_->pose_graph()->GetTrajectoryNodePoses().size(), 0);
  stub_->FinishTrajectory(trajectory_id);
  stub_->pose_graph()->DeleteTrajectory(trajectory_id);
  stub_->pose_graph()->RunFinalOptimization();
  mapping::SubmapId deleted_submap_id = submap_poses.begin()->id;
  EXPECT_NE("",
            stub_->SubmapToProto(deleted_submap_id, &submap_query_response));
  EXPECT_EQ(stub_->pose_graph()->GetTrajectoryStates().at(trajectory_id),
            PoseGraphInterface::TrajectoryState::DELETED);
  // Make sure optimization runs with a deleted trajectory.
  stub_->pose_graph()->RunFinalOptimization();
  server_->Shutdown();
}

}  // namespace
}  // namespace cloud
}  // namespace cartographer
