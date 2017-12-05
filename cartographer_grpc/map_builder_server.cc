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
#include "cartographer_grpc/map_builder_server.h"

#include "cartographer_grpc/handlers/add_trajectory_handler.h"
#include "cartographer_grpc/handlers/finish_trajectory_handler.h"
#include "cartographer_grpc/proto/map_builder_service.grpc.pb.h"
#include "glog/logging.h"

namespace cartographer_grpc {

MapBuilderServer::MapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options)
    : map_builder_(map_builder_server_options.map_builder_options()) {
  framework::Server::Builder server_builder;
  server_builder.SetServerAddress(map_builder_server_options.server_address());
  server_builder.SetNumberOfThreads(
      map_builder_server_options.num_grpc_threads());
  server_builder.RegisterHandler<handlers::AddTrajectoryHandler,
                                 proto::MapBuilderService>("AddTrajectory");
  server_builder.RegisterHandler<handlers::FinishTrajectoryHandler,
                                 proto::MapBuilderService>("FinishTrajectory");
  grpc_server_ = server_builder.Build();
  grpc_server_->SetExecutionContext(
      cartographer::common::make_unique<MapBuilderContext>(&map_builder_));
}

void MapBuilderServer::Start() {
  shutting_down_ = false;
  StartSlamThread();
  grpc_server_->Start();
}

void MapBuilderServer::WaitForShutdown() {
  grpc_server_->WaitForShutdown();
  if (slam_thread_) {
    slam_thread_->join();
  }
}

void MapBuilderServer::Shutdown() {
  shutting_down_ = true;
  grpc_server_->Shutdown();
  if (slam_thread_) {
    slam_thread_->join();
  }
}

void MapBuilderServer::ProcessSensorDataQueue() {
  while (!shutting_down_) {
    // TODO(cschuet): Implement this.
  }
}

void MapBuilderServer::StartSlamThread() {
  CHECK(!slam_thread_);

  // Start the SLAM processing thread.
  slam_thread_ = cartographer::common::make_unique<std::thread>(
      [this]() { this->ProcessSensorDataQueue(); });
}

}  // namespace cartographer_grpc
