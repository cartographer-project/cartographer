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
#include "cartographer_grpc/mapping_server.h"
#include <cartographer_grpc/handlers/finish_trajectory_handler.h>

#include "cartographer_grpc/handlers/add_trajectory_handler.h"
#include "cartographer_grpc/proto/cartographer_service.grpc.pb.h"
#include "glog/logging.h"

namespace cartographer_grpc {

MappingServer::MappingServer(
    const proto::MappingServerOptions& mapping_server_options)
    : map_builder_(mapping_server_options.map_builder_options()) {
  framework::Server::Builder server_builder;
  server_builder.SetServerAddress(mapping_server_options.server_address());
  server_builder.SetNumberOfThreads(mapping_server_options.num_grpc_threads());
  server_builder
      .RegisterHandler<handlers::AddTrajectoryHandler, proto::Cartographer>(
          "AddTrajectory");
  server_builder
      .RegisterHandler<handlers::FinishTrajectoryHandler, proto::Cartographer>(
          "FinishTrajectory");
  server_ = server_builder.Build();
  server_->SetExecutionContext(
      cartographer::common::make_unique<SlamExecutionContext>(&map_builder_));
}

void MappingServer::Start() {
  shutting_down_ = false;

  // Start the SLAM processing thread.
  StartSlamThread();

  // Start the gRPC server.
  server_->Start();
}

void MappingServer::Wait() {
  // Wait for the gRPC server to shut down.
  server_->Wait();

  // Wait for the SLAM processing thread to terminate.
  if (slam_thread_) {
    slam_thread_->join();
  }
}

void MappingServer::Shutdown() {
  shutting_down_ = true;

  // Shut down the gRPC server.
  server_->Shutdown();

  // Wait for the SLAM processing thread to terminate.
  if (slam_thread_) {
    slam_thread_->join();
  }
}

void MappingServer::ProcessSensorDataQueue() {
  while (!shutting_down_) {
    // TODO (cschuet): Implement this.
  }
}

void MappingServer::StartSlamThread() {
  CHECK(!slam_thread_);

  // Start the SLAM processing thread.
  slam_thread_ = cartographer::common::make_unique<std::thread>(
      [this]() { this->ProcessSensorDataQueue(); });
}

}  // namespace cartographer_grpc