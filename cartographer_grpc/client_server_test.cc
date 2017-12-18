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
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace {

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
    auto map_builder_server_options =
        CreateMapBuilderServerOptions(map_builder_server_parameters.get());
    server_ = cartographer::common::make_unique<MapBuilderServer>(
        map_builder_server_options);
    EXPECT_TRUE(server_ != nullptr);
  }

  std::unique_ptr<MapBuilderServer> server_;
  std::unique_ptr<mapping::MapBuilderStub> stub_;
};

TEST_F(ClientServerTest, StartAndStopServer) {
  server_->Start();
  server_->Shutdown();
}

}  // namespace
}  // namespace cartographer_grpc
