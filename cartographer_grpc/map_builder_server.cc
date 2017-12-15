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

#include "cartographer_grpc/handlers/add_imu_data_handler.h"
#include "cartographer_grpc/handlers/add_odometry_data_handler.h"
#include "cartographer_grpc/handlers/add_rangefinder_data_handler.h"
#include "cartographer_grpc/handlers/add_trajectory_handler.h"
#include "cartographer_grpc/handlers/finish_trajectory_handler.h"
#include "cartographer_grpc/proto/map_builder_service.grpc.pb.h"
#include "glog/logging.h"

namespace cartographer_grpc {

MapBuilderServer::MapBuilderContext::MapBuilderContext(
    cartographer::mapping::MapBuilder* map_builder,
    cartographer::common::BlockingQueue<SensorData>* sensor_data_queue)
    : map_builder_(map_builder), sensor_data_queue_(sensor_data_queue) {}

cartographer::mapping::MapBuilder&
MapBuilderServer::MapBuilderContext::map_builder() {
  return *map_builder_;
}

cartographer::common::BlockingQueue<MapBuilderServer::SensorData>&
MapBuilderServer::MapBuilderContext::sensor_data_queue() {
  return *sensor_data_queue_;
}

void MapBuilderServer::MapBuilderContext::AddSensorDataToTrajectory(
    const SensorData& sensor_data) {
  sensor_data.sensor_data->AddToTrajectoryBuilder(
      map_builder_->GetTrajectoryBuilder(sensor_data.trajectory_id));
}

MapBuilderServer::MapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options)
    : map_builder_(map_builder_server_options.map_builder_options()) {
  framework::Server::Builder server_builder;
  server_builder.SetServerAddress(map_builder_server_options.server_address());
  server_builder.SetNumGrpcThreads(
      map_builder_server_options.num_grpc_threads());
  server_builder.SetNumEventThreads(
      map_builder_server_options.num_event_threads());
  server_builder.RegisterHandler<handlers::AddTrajectoryHandler,
                                 proto::MapBuilderService>("AddTrajectory");
  server_builder.RegisterHandler<handlers::AddOdometryDataHandler,
                                 proto::MapBuilderService>("AddOdometryData");
  server_builder
      .RegisterHandler<handlers::AddImuDataHandler, proto::MapBuilderService>(
          "AddImuData");
  server_builder.RegisterHandler<handlers::AddRangefinderDataHandler,
                                 proto::MapBuilderService>(
      "AddRangefinderData");
  server_builder.RegisterHandler<handlers::FinishTrajectoryHandler,
                                 proto::MapBuilderService>("FinishTrajectory");
  grpc_server_ = server_builder.Build();
  grpc_server_->SetExecutionContext(
      cartographer::common::make_unique<MapBuilderContext>(
          &map_builder_, &sensor_data_queue_));
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
  LOG(INFO) << "Starting SLAM thread.";
  while (!shutting_down_) {
    SensorData sensor_data = sensor_data_queue_.Pop();
    grpc_server_->GetContext<MapBuilderContext>()->AddSensorDataToTrajectory(
        sensor_data);
  }
}

void MapBuilderServer::StartSlamThread() {
  CHECK(!slam_thread_);

  // Start the SLAM processing thread.
  slam_thread_ = cartographer::common::make_unique<std::thread>(
      [this]() { this->ProcessSensorDataQueue(); });
}

}  // namespace cartographer_grpc
