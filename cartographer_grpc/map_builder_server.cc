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
#include "cartographer_grpc/handlers/receive_local_slam_results_handler.h"
#include "cartographer_grpc/proto/map_builder_service.grpc.pb.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace {

const cartographer::common::Duration kPopTimeout =
    cartographer::common::FromMilliseconds(100);

}  // namespace

MapBuilderServer::MapBuilderContext::MapBuilderContext(
    MapBuilderServer* map_builder_server)
    : map_builder_server_(map_builder_server) {}

cartographer::mapping::MapBuilderInterface&
MapBuilderServer::MapBuilderContext::map_builder() {
  return *map_builder_server_->map_builder_;
}

cartographer::common::BlockingQueue<std::unique_ptr<MapBuilderServer::Data>>&
MapBuilderServer::MapBuilderContext::sensor_data_queue() {
  return map_builder_server_->incoming_data_queue_;
}

cartographer::mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
MapBuilderServer::MapBuilderContext::
    GetLocalSlamResultCallbackForSubscriptions() {
  MapBuilderServer* map_builder_server = map_builder_server_;
  return [map_builder_server](
             int trajectory_id, cartographer::common::Time time,
             cartographer::transform::Rigid3d local_pose,
             cartographer::sensor::RangeData range_data,
             std::unique_ptr<const cartographer::mapping::
                                 TrajectoryBuilderInterface::InsertionResult>
                 insertion_result) {
    map_builder_server->OnLocalSlamResult(trajectory_id, time, local_pose,
                                          std::move(range_data),
                                          std::move(insertion_result));
  };
}

void MapBuilderServer::MapBuilderContext::AddSensorDataToTrajectory(
    const Data& sensor_data) {
  sensor_data.data->AddToTrajectoryBuilder(
      map_builder_server_->map_builder_->GetTrajectoryBuilder(
          sensor_data.trajectory_id));
}

MapBuilderServer::SubscriptionId
MapBuilderServer::MapBuilderContext::SubscribeLocalSlamResults(
    int trajectory_id, LocalSlamSubscriptionCallback callback) {
  return map_builder_server_->SubscribeLocalSlamResults(trajectory_id,
                                                        callback);
}

void MapBuilderServer::MapBuilderContext::UnsubscribeLocalSlamResults(
    const SubscriptionId& subscription_id) {
  map_builder_server_->UnsubscribeLocalSlamResults(subscription_id);
}

void MapBuilderServer::MapBuilderContext::NotifyFinishTrajectory(
    int trajectory_id) {
  map_builder_server_->NotifyFinishTrajectory(trajectory_id);
}

std::shared_ptr<cartographer::mapping_2d::Submap>
MapBuilderServer::MapBuilderContext::UpdateSubmap2D(
    const cartographer::mapping::proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  cartographer::mapping::SubmapId submap_id{proto.submap_id().trajectory_id(),
                                            proto.submap_id().submap_index()};
  std::shared_ptr<cartographer::mapping_2d::Submap> submap_2d_ptr;
  auto submap_it = unfinished_submaps_.find(submap_id);
  if (submap_it == unfinished_submaps_.end()) {
    // Seeing a submap for the first time it should never be finished.
    CHECK(!proto.submap_2d().finished());
    submap_2d_ptr =
        std::make_shared<cartographer::mapping_2d::Submap>(proto.submap_2d());
    unfinished_submaps_.Insert(submap_id, submap_2d_ptr);
  } else {
    submap_2d_ptr = std::dynamic_pointer_cast<cartographer::mapping_2d::Submap>(
        submap_it->data);
    CHECK(submap_2d_ptr);
    submap_2d_ptr->UpdateFromProto(proto);

    // If the submap was just finished by the recent update, remove it from the
    // list of unfinished submaps.
    if (submap_2d_ptr->finished()) {
      unfinished_submaps_.Trim(submap_id);
    } else {
      // If the submap is unfinished set the 'num_range_data' to 0 since we
      // haven't changed the HybridGrid.
      submap_2d_ptr->SetNumRangeData(0);
    }
  }
  return submap_2d_ptr;
}

std::shared_ptr<cartographer::mapping_3d::Submap>
MapBuilderServer::MapBuilderContext::UpdateSubmap3D(
    const cartographer::mapping::proto::Submap& proto) {
  CHECK(proto.has_submap_3d());
  cartographer::mapping::SubmapId submap_id{proto.submap_id().trajectory_id(),
                                            proto.submap_id().submap_index()};
  std::shared_ptr<cartographer::mapping_3d::Submap> submap_3d_ptr;
  auto submap_it = unfinished_submaps_.find(submap_id);
  if (submap_it == unfinished_submaps_.end()) {
    // Seeing a submap for the first time it should never be finished.
    CHECK(!proto.submap_3d().finished());
    submap_3d_ptr =
        std::make_shared<cartographer::mapping_3d::Submap>(proto.submap_3d());
    unfinished_submaps_.Insert(submap_id, submap_3d_ptr);
    submap_it = unfinished_submaps_.find(submap_id);
  } else {
    submap_3d_ptr = std::dynamic_pointer_cast<cartographer::mapping_3d::Submap>(
        submap_it->data);
    CHECK(submap_3d_ptr);

    // Update submap with information in incoming request.
    submap_3d_ptr->UpdateFromProto(proto);

    // If the submap was just finished by the recent update, remove it from the
    // list of unfinished submaps.
    if (submap_3d_ptr->finished()) {
      unfinished_submaps_.Trim(submap_id);
    } else {
      // If the submap is unfinished set the 'num_range_data' to 0 since we
      // haven't changed the HybridGrid.
      submap_3d_ptr->SetNumRangeData(0);
    }
  }
  return submap_3d_ptr;
}

std::unique_ptr<cartographer::mapping::LocalSlamResultData>
MapBuilderServer::MapBuilderContext::ProcessLocalSlamResultData(
    const std::string& sensor_id, cartographer::common::Time time,
    const cartographer::mapping::proto::LocalSlamResultData& proto) {
  CHECK_GE(proto.submaps().size(), 0);
  CHECK(proto.submaps(0).has_submap_2d() || proto.submaps(0).has_submap_3d());
  if (proto.submaps(0).has_submap_2d()) {
    std::vector<std::shared_ptr<const cartographer::mapping_2d::Submap>>
        submaps;
    for (const auto& submap_proto : proto.submaps()) {
      submaps.push_back(UpdateSubmap2D(submap_proto));
    }
    return cartographer::common::make_unique<
        cartographer::mapping::LocalSlamResult2D>(
        sensor_id, time,
        std::make_shared<const cartographer::mapping::TrajectoryNode::Data>(
            cartographer::mapping::FromProto(proto.node_data())),
        submaps);
  } else {
    std::vector<std::shared_ptr<const cartographer::mapping_3d::Submap>>
        submaps;
    for (const auto& submap_proto : proto.submaps()) {
      submaps.push_back(UpdateSubmap3D(submap_proto));
    }
    return cartographer::common::make_unique<
        cartographer::mapping::LocalSlamResult3D>(
        sensor_id, time,
        std::make_shared<const cartographer::mapping::TrajectoryNode::Data>(
            cartographer::mapping::FromProto(proto.node_data())),
        std::move(submaps));
  }
}

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
  server_builder.RegisterHandler<handlers::AddFixedFramePoseDataHandler,
                                 proto::MapBuilderService>(
      "AddFixedFramePoseData");
  server_builder.RegisterHandler<handlers::AddLocalSlamResultDataHandler,
                                 proto::MapBuilderService>(
      "AddLocalSlamResultData");
  server_builder.RegisterHandler<handlers::FinishTrajectoryHandler,
                                 proto::MapBuilderService>("FinishTrajectory");
  server_builder.RegisterHandler<handlers::ReceiveLocalSlamResultsHandler,
                                 proto::MapBuilderService>(
      "ReceiveLocalSlamResults");
  server_builder
      .RegisterHandler<handlers::GetSubmapHandler, proto::MapBuilderService>(
          "GetSubmap");
  server_builder.RegisterHandler<handlers::GetTrajectoryNodePosesHandler,
                                 proto::MapBuilderService>(
      "GetTrajectoryNodePoses");
  server_builder.RegisterHandler<handlers::GetAllSubmapPosesHandler,
                                 proto::MapBuilderService>("GetAllSubmapPoses");
  server_builder.RegisterHandler<handlers::GetLocalToGlobalTransformHandler,
                                 proto::MapBuilderService>(
      "GetLocalToGlobalTransform");
  server_builder.RegisterHandler<handlers::GetConstraintsHandler,
                                 proto::MapBuilderService>("GetConstraints");
  grpc_server_ = server_builder.Build();
  grpc_server_->SetExecutionContext(
      cartographer::common::make_unique<MapBuilderContext>(this));
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
    std::unique_ptr<Data> sensor_data =
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
  cartographer::common::MutexLocker locker(&local_slam_subscriptions_lock_);
  for (auto& entry : local_slam_subscriptions_[trajectory_id]) {
    auto copy_of_insertion_result =
        insertion_result
            ? cartographer::common::make_unique<
                  const cartographer::mapping::TrajectoryBuilderInterface::
                      InsertionResult>(*insertion_result)
            : nullptr;
    LocalSlamSubscriptionCallback callback = entry.second;
    callback(cartographer::common::make_unique<LocalSlamResult>(
        LocalSlamResult{trajectory_id, time, local_pose, shared_range_data,
                        std::move(copy_of_insertion_result)}));
  }
}

MapBuilderServer::SubscriptionId MapBuilderServer::SubscribeLocalSlamResults(
    int trajectory_id, LocalSlamSubscriptionCallback callback) {
  cartographer::common::MutexLocker locker(&local_slam_subscriptions_lock_);
  local_slam_subscriptions_[trajectory_id].emplace(current_subscription_index_,
                                                   callback);
  return SubscriptionId{trajectory_id, current_subscription_index_++};
}

void MapBuilderServer::UnsubscribeLocalSlamResults(
    const SubscriptionId& subscription_id) {
  cartographer::common::MutexLocker locker(&local_slam_subscriptions_lock_);
  CHECK_EQ(local_slam_subscriptions_[subscription_id.trajectory_id].erase(
               subscription_id.subscription_index),
           1u);
}

void MapBuilderServer::NotifyFinishTrajectory(int trajectory_id) {
  cartographer::common::MutexLocker locker(&local_slam_subscriptions_lock_);
  for (auto& entry : local_slam_subscriptions_[trajectory_id]) {
    LocalSlamSubscriptionCallback callback = entry.second;
    // 'nullptr' signals subscribers that the trajectory finished.
    callback(nullptr);
  }
}

void MapBuilderServer::WaitUntilIdle() {
  incoming_data_queue_.WaitUntilEmpty();
  map_builder_->pose_graph()->RunFinalOptimization();
}

}  // namespace cartographer_grpc
