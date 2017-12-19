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

#include "cartographer/internal/mapping/test_helpers.h"
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
using testing::_;

namespace cartographer_grpc {
namespace {

constexpr char kSensorId[] = "sensor";

class MockMapBuilder : public cartographer::mapping::MapBuilderInterface {
 public:
  MOCK_METHOD3(AddTrajectoryBuilder,
               int(const std::unordered_set<std::string>& expected_sensor_ids,
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
  MOCK_METHOD1(SerializeState, void(cartographer::io::ProtoStreamWriter*));
  MOCK_METHOD1(LoadMap, void(cartographer::io::ProtoStreamReader*));
  MOCK_CONST_METHOD0(num_trajectory_builders, int());
  MOCK_METHOD0(pose_graph, PoseGraphInterface*());
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
      MAP_BUILDER_SERVER = {
        server_address = "0.0.0.0:50051",
        num_event_threads = 1,
        num_grpc_threads = 1,
        map_builder = MAP_BUILDER,
      }
      return MAP_BUILDER_SERVER)text";
    auto map_builder_server_parameters =
        cartographer::mapping::test::ResolveLuaParameters(kMapBuilderServerLua);
    map_builder_server_options_ =
        CreateMapBuilderServerOptions(map_builder_server_parameters.get());
    const std::string kTrajectoryBuilderLua = R"text(
      include "trajectory_builder.lua"
      return TRAJECTORY_BUILDER)text";
    auto trajectory_builder_parameters =
        cartographer::mapping::test::ResolveLuaParameters(
            kTrajectoryBuilderLua);
    trajectory_builder_options_ =
        cartographer::mapping::CreateTrajectoryBuilderOptions(
            trajectory_builder_parameters.get());
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
  }

  void InitializeStub() {
    stub_ = cartographer::common::make_unique<mapping::MapBuilderStub>(
        map_builder_server_options_.server_address());
    EXPECT_TRUE(stub_ != nullptr);
  }

  proto::MapBuilderServerOptions map_builder_server_options_;
  MockMapBuilder* mock_map_builder_;
  cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options_;
  std::unique_ptr<MapBuilderServer> server_;
  std::unique_ptr<mapping::MapBuilderStub> stub_;
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
  std::unordered_set<std::string> expected_sensor_ids = {kSensorId};
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

}  // namespace
}  // namespace cartographer_grpc
