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

#include "cartographer/cloud/internal/map_builder_server.h"

#include "cartographer/cloud/internal/handlers/add_fixed_frame_pose_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_imu_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_landmark_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_odometry_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_rangefinder_data_handler.h"
#include "cartographer/cloud/internal/handlers/add_sensor_data_batch_handler.h"
#include "cartographer/cloud/internal/handlers/add_trajectory_handler.h"
#include "cartographer/cloud/internal/handlers/finish_trajectory_handler.h"
#include "cartographer/cloud/internal/handlers/get_all_submap_poses.h"
#include "cartographer/cloud/internal/handlers/get_constraints_handler.h"
#include "cartographer/cloud/internal/handlers/get_landmark_poses_handler.h"
#include "cartographer/cloud/internal/handlers/get_local_to_global_transform_handler.h"
#include "cartographer/cloud/internal/handlers/get_submap_handler.h"
#include "cartographer/cloud/internal/handlers/get_trajectory_node_poses_handler.h"
#include "cartographer/cloud/internal/handlers/is_trajectory_finished_handler.h"
#include "cartographer/cloud/internal/handlers/is_trajectory_frozen_handler.h"
#include "cartographer/cloud/internal/handlers/load_state_handler.h"
#include "cartographer/cloud/internal/handlers/receive_global_slam_optimizations_handler.h"
#include "cartographer/cloud/internal/handlers/receive_local_slam_results_handler.h"
#include "cartographer/cloud/internal/handlers/run_final_optimization_handler.h"
#include "cartographer/cloud/internal/handlers/set_landmark_pose_handler.h"
#include "cartographer/cloud/internal/handlers/write_state_handler.h"
#include "cartographer/cloud/internal/sensor/serialization.h"
#include "glog/logging.h"

namespace cartographer {
namespace cloud {
namespace {

static auto* kIncomingDataQueueMetric = metrics::Gauge::Null();
const common::Duration kPopTimeout = common::FromMilliseconds(100);

}  // namespace

MapBuilderServer::MapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options,
    std::unique_ptr<mapping::MapBuilderInterface> map_builder)
    : map_builder_(std::move(map_builder)) {
  async_grpc::Server::Builder server_builder;
  server_builder.SetServerAddress(map_builder_server_options.server_address());
  server_builder.SetNumGrpcThreads(
      map_builder_server_options.num_grpc_threads());
  server_builder.SetNumEventThreads(
      map_builder_server_options.num_event_threads());
  if (!map_builder_server_options.uplink_server_address().empty()) {
    local_trajectory_uploader_ = CreateLocalTrajectoryUploader(
        map_builder_server_options.uplink_server_address(),
        map_builder_server_options.upload_batch_size(),
        map_builder_server_options.enable_ssl_encryption());
  }
  server_builder.RegisterHandler<handlers::AddTrajectoryHandler>();
  server_builder.RegisterHandler<handlers::AddOdometryDataHandler>();
  server_builder.RegisterHandler<handlers::AddImuDataHandler>();
  server_builder.RegisterHandler<handlers::AddRangefinderDataHandler>();
  server_builder.RegisterHandler<handlers::AddFixedFramePoseDataHandler>();
  server_builder.RegisterHandler<handlers::AddLandmarkDataHandler>();
  server_builder.RegisterHandler<handlers::AddSensorDataBatchHandler>();
  server_builder.RegisterHandler<handlers::FinishTrajectoryHandler>();
  server_builder
      .RegisterHandler<handlers::ReceiveGlobalSlamOptimizationsHandler>();
  server_builder.RegisterHandler<handlers::ReceiveLocalSlamResultsHandler>();
  server_builder.RegisterHandler<handlers::GetSubmapHandler>();
  server_builder.RegisterHandler<handlers::GetTrajectoryNodePosesHandler>();
  server_builder.RegisterHandler<handlers::GetLandmarkPosesHandler>();
  server_builder.RegisterHandler<handlers::GetAllSubmapPosesHandler>();
  server_builder.RegisterHandler<handlers::GetLocalToGlobalTransformHandler>();
  server_builder.RegisterHandler<handlers::GetConstraintsHandler>();
  server_builder.RegisterHandler<handlers::IsTrajectoryFinishedHandler>();
  server_builder.RegisterHandler<handlers::IsTrajectoryFrozenHandler>();
  server_builder.RegisterHandler<handlers::LoadStateHandler>();
  server_builder.RegisterHandler<handlers::RunFinalOptimizationHandler>();
  server_builder.RegisterHandler<handlers::WriteStateHandler>();
  server_builder.RegisterHandler<handlers::SetLandmarkPoseHandler>();
  grpc_server_ = server_builder.Build();
  if (map_builder_server_options.map_builder_options()
          .use_trajectory_builder_2d()) {
    grpc_server_->SetExecutionContext(
        common::make_unique<MapBuilderContext<mapping::Submap2D>>(this));
  } else if (map_builder_server_options.map_builder_options()
                 .use_trajectory_builder_3d()) {
    grpc_server_->SetExecutionContext(
        common::make_unique<MapBuilderContext<mapping::Submap3D>>(this));
  } else {
    LOG(FATAL)
        << "Set either use_trajectory_builder_2d or use_trajectory_builder_3d";
  }
  map_builder_->pose_graph()->SetGlobalSlamOptimizationCallback(
      std::bind(&MapBuilderServer::OnGlobalSlamOptimizations, this,
                std::placeholders::_1, std::placeholders::_2));
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
    slam_thread_.reset();
  }
  if (local_trajectory_uploader_) {
    local_trajectory_uploader_->Shutdown();
    local_trajectory_uploader_.reset();
  }
}

void MapBuilderServer::ProcessSensorDataQueue() {
  LOG(INFO) << "Starting SLAM thread.";
  while (!shutting_down_) {
    kIncomingDataQueueMetric->Set(incoming_data_queue_.Size());
    std::unique_ptr<MapBuilderContextInterface::Data> sensor_data =
        incoming_data_queue_.PopWithTimeout(kPopTimeout);
    if (sensor_data) {
      grpc_server_->GetContext<MapBuilderContextInterface>()
          ->AddSensorDataToTrajectory(*sensor_data);
    }
  }
}

void MapBuilderServer::StartSlamThread() {
  CHECK(!slam_thread_);

  // Start the SLAM processing thread.
  slam_thread_ = common::make_unique<std::thread>(
      [this]() { this->ProcessSensorDataQueue(); });
}

void MapBuilderServer::OnLocalSlamResult(
    int trajectory_id, common::Time time, transform::Rigid3d local_pose,
    sensor::RangeData range_data,
    std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult>
        insertion_result) {
  auto shared_range_data =
      std::make_shared<sensor::RangeData>(std::move(range_data));

  // If there is an uplink server and a submap insertion happened, enqueue this
  // local SLAM result for uploading.
  if (insertion_result &&
      grpc_server_->GetUnsynchronizedContext<MapBuilderContextInterface>()
          ->local_trajectory_uploader()) {
    auto sensor_data = common::make_unique<proto::SensorData>();
    auto sensor_id =
        grpc_server_->GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->local_trajectory_uploader()
            ->GetLocalSlamResultSensorId(trajectory_id);
    CreateSensorDataForLocalSlamResult(sensor_id.id, trajectory_id, time,
                                       starting_submap_index_,
                                       *insertion_result, sensor_data.get());
    // TODO(cschuet): Make this more robust.
    if (insertion_result->insertion_submaps.front()->finished()) {
      ++starting_submap_index_;
    }
    grpc_server_->GetUnsynchronizedContext<MapBuilderContextInterface>()
        ->local_trajectory_uploader()
        ->EnqueueSensorData(std::move(sensor_data));
  }

  common::MutexLocker locker(&subscriptions_lock_);
  for (auto& entry : local_slam_subscriptions_[trajectory_id]) {
    auto copy_of_insertion_result =
        insertion_result
            ? common::make_unique<
                  const mapping::TrajectoryBuilderInterface::InsertionResult>(
                  *insertion_result)
            : nullptr;
    MapBuilderContextInterface::LocalSlamSubscriptionCallback callback =
        entry.second;
    if (!callback(
            common::make_unique<MapBuilderContextInterface::LocalSlamResult>(
                MapBuilderContextInterface::LocalSlamResult{
                    trajectory_id, time, local_pose, shared_range_data,
                    std::move(copy_of_insertion_result)}))) {
      LOG(INFO) << "Removing subscription with index: " << entry.first;
      CHECK_EQ(local_slam_subscriptions_[trajectory_id].erase(entry.first), 1u);
    }
  }
}

void MapBuilderServer::OnGlobalSlamOptimizations(
    const std::map<int, mapping::SubmapId>& last_optimized_submap_ids,
    const std::map<int, mapping::NodeId>& last_optimized_node_ids) {
  common::MutexLocker locker(&subscriptions_lock_);
  for (auto& entry : global_slam_subscriptions_) {
    if (!entry.second(last_optimized_submap_ids, last_optimized_node_ids)) {
      LOG(INFO) << "Removing subscription with index: " << entry.first;
      CHECK_EQ(global_slam_subscriptions_.erase(entry.first), 1u);
    }
  }
}

MapBuilderContextInterface::LocalSlamSubscriptionId
MapBuilderServer::SubscribeLocalSlamResults(
    int trajectory_id,
    MapBuilderContextInterface::LocalSlamSubscriptionCallback callback) {
  common::MutexLocker locker(&subscriptions_lock_);
  local_slam_subscriptions_[trajectory_id].emplace(current_subscription_index_,
                                                   callback);
  return MapBuilderContextInterface::LocalSlamSubscriptionId{
      trajectory_id, current_subscription_index_++};
}

void MapBuilderServer::UnsubscribeLocalSlamResults(
    const MapBuilderContextInterface::LocalSlamSubscriptionId&
        subscription_id) {
  common::MutexLocker locker(&subscriptions_lock_);
  CHECK_EQ(local_slam_subscriptions_[subscription_id.trajectory_id].erase(
               subscription_id.subscription_index),
           1u);
}

int MapBuilderServer::SubscribeGlobalSlamOptimizations(
    MapBuilderContextInterface::GlobalSlamOptimizationCallback callback) {
  common::MutexLocker locker(&subscriptions_lock_);
  global_slam_subscriptions_.emplace(current_subscription_index_, callback);
  return current_subscription_index_++;
}

void MapBuilderServer::UnsubscribeGlobalSlamOptimizations(
    int subscription_index) {
  common::MutexLocker locker(&subscriptions_lock_);
  CHECK_EQ(global_slam_subscriptions_.erase(subscription_index), 1u);
}

void MapBuilderServer::NotifyFinishTrajectory(int trajectory_id) {
  common::MutexLocker locker(&subscriptions_lock_);
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

void MapBuilderServer::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* queue_length = factory->NewGaugeFamily(
      "cloud_internal_map_builder_server_incoming_data_queue_length",
      "Incoming SLAM Data Queue length");
  kIncomingDataQueueMetric = queue_length->Add({});
}

}  // namespace cloud
}  // namespace cartographer
