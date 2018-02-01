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

#include "cartographer_grpc/handlers/add_fixed_frame_pose_data_handler.h"
#include "cartographer_grpc/handlers/add_imu_data_handler.h"
#include "cartographer_grpc/handlers/add_landmark_data_handler.h"
#include "cartographer_grpc/handlers/add_local_slam_result_data_handler.h"
#include "cartographer_grpc/handlers/add_odometry_data_handler.h"
#include "cartographer_grpc/handlers/add_rangefinder_data_handler.h"
#include "cartographer_grpc/handlers/add_trajectory_handler.h"
#include "cartographer_grpc/handlers/finish_trajectory_handler.h"
#include "cartographer_grpc/handlers/get_all_submap_poses.h"
#include "cartographer_grpc/handlers/get_constraints_handler.h"
#include "cartographer_grpc/handlers/get_local_to_global_transform_handler.h"
#include "cartographer_grpc/handlers/get_submap_handler.h"
#include "cartographer_grpc/handlers/get_trajectory_node_poses_handler.h"
#include "cartographer_grpc/handlers/load_map_handler.h"
#include "cartographer_grpc/handlers/receive_local_slam_results_handler.h"
#include "cartographer_grpc/handlers/run_final_optimization_handler.h"
#include "cartographer_grpc/handlers/write_map_handler.h"
#include "cartographer_grpc/sensor/serialization.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace {

const cartographer::common::Duration kPopTimeout =
    cartographer::common::FromMilliseconds(100);

}  // namespace

MapBuilderServer::MapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder)
    : map_builder_(std::move(map_builder)) {
  framework::Server::Builder server_builder;
  server_builder.SetServerAddress(map_builder_server_options.server_address());
  server_builder.SetNumGrpcThreads(
      map_builder_server_options.num_grpc_threads());
  server_builder.SetNumEventThreads(
      map_builder_server_options.num_event_threads());
  if (!map_builder_server_options.uplink_server_address().empty()) {
    local_trajectory_uploader_ =
        cartographer::common::make_unique<LocalTrajectoryUploader>(
            map_builder_server_options.uplink_server_address());
  }
  server_builder.RegisterHandler<handlers::AddTrajectoryHandler>();
  server_builder.RegisterHandler<handlers::AddOdometryDataHandler>();
  server_builder.RegisterHandler<handlers::AddImuDataHandler>();
  server_builder.RegisterHandler<handlers::AddRangefinderDataHandler>();
  server_builder.RegisterHandler<handlers::AddFixedFramePoseDataHandler>();
  server_builder.RegisterHandler<handlers::AddLandmarkDataHandler>();
  server_builder.RegisterHandler<handlers::AddLocalSlamResultDataHandler>();
  server_builder.RegisterHandler<handlers::FinishTrajectoryHandler>();
  server_builder.RegisterHandler<handlers::ReceiveLocalSlamResultsHandler>();
  server_builder.RegisterHandler<handlers::GetSubmapHandler>();
  server_builder.RegisterHandler<handlers::GetTrajectoryNodePosesHandler>();
  server_builder.RegisterHandler<handlers::GetAllSubmapPosesHandler>();
  server_builder.RegisterHandler<handlers::GetLocalToGlobalTransformHandler>();
  server_builder.RegisterHandler<handlers::GetConstraintsHandler>();
  server_builder.RegisterHandler<handlers::LoadMapHandler>();
  server_builder.RegisterHandler<handlers::RunFinalOptimizationHandler>();
  server_builder.RegisterHandler<handlers::WriteMapHandler>();
  grpc_server_ = server_builder.Build();
  grpc_server_->SetExecutionContext(
      cartographer::common::make_unique<MapBuilderContext>(this));
}

void MapBuilderServer::Start() {
  shutting_down_ = false;
  if (local_trajectory_uploader_) {
    local_trajectory_uploader_->Start();
  }
  StartSlamThread();
  grpc_server_->Start();
}

void MapBuilderServer::WaitForShutdown() {
  grpc_server_->WaitForShutdown();
  if (slam_thread_) {
    slam_thread_->join();
  }
  if (local_trajectory_uploader_) {
    local_trajectory_uploader_->Shutdown();
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
    std::unique_ptr<MapBuilderContextInterface::Data> sensor_data =
        incoming_data_queue_.PopWithTimeout(kPopTimeout);
    if (sensor_data) {
      grpc_server_->GetContext<MapBuilderContext>()->AddSensorDataToTrajectory(
          *sensor_data);
    }
  }
}

void MapBuilderServer::StartSlamThread() {
  CHECK(!slam_thread_);

  // Start the SLAM processing thread.
  slam_thread_ = cartographer::common::make_unique<std::thread>(
      [this]() { this->ProcessSensorDataQueue(); });
}

void MapBuilderServer::OnLocalSlamResult(
    int trajectory_id, cartographer::common::Time time,
    cartographer::transform::Rigid3d local_pose,
    cartographer::sensor::RangeData range_data,
    std::unique_ptr<const cartographer::mapping::TrajectoryBuilderInterface::
                        InsertionResult>
        insertion_result) {
  auto shared_range_data =
      std::make_shared<cartographer::sensor::RangeData>(std::move(range_data));

  // If there is an uplink server and a submap insertion happened, enqueue this
  // local SLAM result for uploading.
  if (insertion_result &&
      grpc_server_->GetUnsynchronizedContext<MapBuilderContext>()
          ->local_trajectory_uploader()) {
    auto data_request = cartographer::common::make_unique<
        proto::AddLocalSlamResultDataRequest>();
    auto sensor_id = grpc_server_->GetUnsynchronizedContext<MapBuilderContext>()
                         ->local_trajectory_uploader()
                         ->GetLocalSlamResultSensorId(trajectory_id);
    sensor::CreateAddLocalSlamResultDataRequest(
        sensor_id.id, trajectory_id, time, starting_submap_index_,
        *insertion_result, data_request.get());
    // TODO(cschuet): Make this more robust.
    if (insertion_result->insertion_submaps.front()->finished()) {
      ++starting_submap_index_;
    }
    grpc_server_->GetUnsynchronizedContext<MapBuilderContext>()
        ->local_trajectory_uploader()
        ->EnqueueDataRequest(std::move(data_request));
  }

  cartographer::common::MutexLocker locker(&local_slam_subscriptions_lock_);
  for (auto& entry : local_slam_subscriptions_[trajectory_id]) {
    auto copy_of_insertion_result =
        insertion_result
            ? cartographer::common::make_unique<
                  const cartographer::mapping::TrajectoryBuilderInterface::
                      InsertionResult>(*insertion_result)
            : nullptr;
    MapBuilderContextInterface::LocalSlamSubscriptionCallback callback =
        entry.second;
    callback(cartographer::common::make_unique<
             MapBuilderContextInterface::LocalSlamResult>(
        MapBuilderContextInterface::LocalSlamResult{
            trajectory_id, time, local_pose, shared_range_data,
            std::move(copy_of_insertion_result)}));
  }
}

MapBuilderContextInterface::SubscriptionId
MapBuilderServer::SubscribeLocalSlamResults(
    int trajectory_id,
    MapBuilderContextInterface::LocalSlamSubscriptionCallback callback) {
  cartographer::common::MutexLocker locker(&local_slam_subscriptions_lock_);
  local_slam_subscriptions_[trajectory_id].emplace(current_subscription_index_,
                                                   callback);
  return MapBuilderContextInterface::SubscriptionId{
      trajectory_id, current_subscription_index_++};
}

void MapBuilderServer::UnsubscribeLocalSlamResults(
    const MapBuilderContextInterface::SubscriptionId& subscription_id) {
  cartographer::common::MutexLocker locker(&local_slam_subscriptions_lock_);
  CHECK_EQ(local_slam_subscriptions_[subscription_id.trajectory_id].erase(
               subscription_id.subscription_index),
           1u);
}

void MapBuilderServer::NotifyFinishTrajectory(int trajectory_id) {
  cartographer::common::MutexLocker locker(&local_slam_subscriptions_lock_);
  for (auto& entry : local_slam_subscriptions_[trajectory_id]) {
    MapBuilderContextInterface::LocalSlamSubscriptionCallback callback =
        entry.second;
    // 'nullptr' signals subscribers that the trajectory finished.
    callback(nullptr);
  }
}

void MapBuilderServer::WaitUntilIdle() {
  incoming_data_queue_.WaitUntilEmpty();
  map_builder_->pose_graph()->RunFinalOptimization();
}

}  // namespace cartographer_grpc
