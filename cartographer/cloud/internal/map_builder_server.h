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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_MAP_BUILDER_SERVER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_MAP_BUILDER_SERVER_H

#include "async_grpc/execution_context.h"
#include "async_grpc/server.h"
#include "cartographer/cloud/internal/local_trajectory_uploader.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/map_builder_server_interface.h"
#include "cartographer/cloud/proto/map_builder_server_options.pb.h"
#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/submap_controller.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace cloud {

class MapBuilderServer;

template <class SubmapType>
class MapBuilderContext : public MapBuilderContextInterface {
 public:
  MapBuilderContext(MapBuilderServer* map_builder_server);
  mapping::MapBuilderInterface& map_builder() override;
  common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>&
  sensor_data_queue() override;
  mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
  GetLocalSlamResultCallbackForSubscriptions() override;
  void AddSensorDataToTrajectory(const Data& sensor_data) override;
  MapBuilderContextInterface::SubscriptionId SubscribeLocalSlamResults(
      int trajectory_id, LocalSlamSubscriptionCallback callback) override;
  void UnsubscribeLocalSlamResults(
      const SubscriptionId& subscription_id) override;
  void NotifyFinishTrajectory(int trajectory_id) override;
  LocalTrajectoryUploaderInterface* local_trajectory_uploader() override;
  void EnqueueSensorData(int trajectory_id,
                         std::unique_ptr<sensor::Data> data) override;
  void EnqueueLocalSlamResultData(int trajectory_id,
                                  const std::string& sensor_id,
                                  const mapping::proto::LocalSlamResultData&
                                      local_slam_result_data) override;

 private:
  MapBuilderServer* map_builder_server_;
  mapping::SubmapController<SubmapType> submap_controller_;
};

class MapBuilderServer : public MapBuilderServerInterface {
 public:
  friend MapBuilderContext<mapping::Submap2D>;
  friend MapBuilderContext<mapping::Submap3D>;

  MapBuilderServer(
      const proto::MapBuilderServerOptions& map_builder_server_options,
      std::unique_ptr<mapping::MapBuilderInterface> map_builder);
  ~MapBuilderServer() {}

  // Starts the gRPC server, the 'LocalTrajectoryUploader' and the SLAM thread.
  void Start() final;

  // Waits for the 'MapBuilderServer' to shut down. Note: The server must be
  // either shutting down or some other thread must call 'Shutdown()' for this
  // function to ever return.
  void WaitForShutdown() final;

  // Waits until all computation is finished (for testing).
  void WaitUntilIdle() final;

  // Shuts down the gRPC server, the 'LocalTrajectoryUploader' and the SLAM
  // thread.
  void Shutdown() final;

 private:
  using LocalSlamResultHandlerSubscriptions =
      std::map<int /* subscription_index */,
               MapBuilderContextInterface::LocalSlamSubscriptionCallback>;

  void ProcessSensorDataQueue();
  void StartSlamThread();
  void OnLocalSlamResult(
      int trajectory_id, common::Time time, transform::Rigid3d local_pose,
      sensor::RangeData range_data,
      std::unique_ptr<
          const mapping::TrajectoryBuilderInterface::InsertionResult>
          insertion_result);
  MapBuilderContextInterface::SubscriptionId SubscribeLocalSlamResults(
      int trajectory_id,
      MapBuilderContextInterface::LocalSlamSubscriptionCallback callback);
  void UnsubscribeLocalSlamResults(
      const MapBuilderContextInterface::SubscriptionId& subscription_id);
  void NotifyFinishTrajectory(int trajectory_id);

  bool shutting_down_ = false;
  std::unique_ptr<std::thread> slam_thread_;
  std::unique_ptr<async_grpc::Server> grpc_server_;
  std::unique_ptr<mapping::MapBuilderInterface> map_builder_;
  common::BlockingQueue<std::unique_ptr<MapBuilderContextInterface::Data>>
      incoming_data_queue_;
  common::Mutex local_slam_subscriptions_lock_;
  int current_subscription_index_ = 0;
  std::map<int /* trajectory ID */, LocalSlamResultHandlerSubscriptions>
      local_slam_subscriptions_ GUARDED_BY(local_slam_subscriptions_lock_);
  std::unique_ptr<LocalTrajectoryUploaderInterface> local_trajectory_uploader_;
  int starting_submap_index_ = 0;
};

}  // namespace cloud
}  // namespace cartographer

#include "cartographer/cloud/internal/map_builder_context_impl.h"

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_MAP_BUILDER_SERVER_H
