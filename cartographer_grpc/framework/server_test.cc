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

#include "cartographer_grpc/framework/server.h"

#include "cartographer_grpc/framework/proto/math_service.grpc.pb.h"
#include "cartographer_grpc/framework/proto/math_service.pb.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "glog/logging.h"
#include "grpc++/grpc++.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace framework {
namespace {

class GetServerOptionsHandler
    : public RpcHandler<Stream<proto::Request>, proto::Response> {};

TEST(ServerTest, StartServerTest) {
  Server::Builder server_builder;
  server_builder.SetServerAddress("0.0.0.0:50051");
  server_builder.SetNumberOfThreads(1);
  server_builder.RegisterHandler<GetServerOptionsHandler, proto::Math>(
      "GetSum");
  std::unique_ptr<Server> server = server_builder.Build();
  server->Start();
  server->Shutdown();
}

}  // namespace
}  // namespace framework
}  // namespace cartographer_grpc
