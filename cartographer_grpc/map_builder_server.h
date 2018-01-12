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

#ifndef CARTOGRAPHER_GRPC_MAP_BUILDER_SERVER_H
#define CARTOGRAPHER_GRPC_MAP_BUILDER_SERVER_H

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/dispatchable.h"
#include "cartographer_grpc/framework/execution_context.h"
#include "cartographer_grpc/framework/server.h"
#include "cartographer_grpc/proto/map_builder_server_options.pb.h"

namespace cartographer_grpc {

class MapBuilderServer {
 public:
  struct LocalSlamResult {
    int trajectory_id;
    cartographer::common::Time time;
    cartographer::transform::Rigid3d local_pose;
    std::shared_ptr<const cartographer::sensor::RangeData> range_data;
    std::unique_ptr<const cartographer::mapping::TrajectoryBuilderInterface::
                        InsertionResult>
        insertion_result;
  };
  // Calling with 'nullptr' signals subscribers that the subscription has ended.
  using LocalSlamSubscriptionCallback =
      std::function<void(std::unique_ptr<LocalSlamResult>)>;
  struct Data {
    int trajectory_id;
    std::unique_ptr<cartographer::sensor::Data> data;
  };
  struct SubscriptionId {
    const int trajectory_id;
    const int subscription_index;
  };

  class MapBuilderContext : public framework::ExecutionContext {
   public:
    MapBuilderContext(MapBuilderServer* map_builder_server);
    cartographer::mapping::MapBuilderInterface& map_builder();
    cartographer::common::BlockingQueue<std::unique_ptr<Data>>&
    sensor_data_queue();
    cartographer::mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
    GetLocalSlamResultCallbackForSubscriptions();
    void AddSensorDataToTrajectory(const Data& sensor_data);
    SubscriptionId SubscribeLocalSlamResults(
        int trajectory_id, LocalSlamSubscriptionCallback callback);
    void UnsubscribeLocalSlamResults(const SubscriptionId& subscription_id);
    void NotifyFinishTrajectory(int trajectory_id);
    std::unique_ptr<cartographer::mapping::LocalSlamResultData>
    ProcessLocalSlamResultData(
        const std::string& sensor_id, cartographer::common::Time time,
        const cartographer::mapping::proto::LocalSlamResultData& proto);

    template <typename DataType>
    void EnqueueSensorData(int trajectory_id, const std::string& sensor_id,
                           const DataType& data) {
      map_builder_server_->incoming_data_queue_.Push(
          cartographer::common::make_unique<MapBuilderServer::Data>(
              MapBuilderServer::Data{
                  trajectory_id,
                  cartographer::sensor::MakeDispatchable(sensor_id, data)}));
    }

    void EnqueueLocalSlamResultData(
        int trajectory_id, const std::string& sensor_id,
        std::unique_ptr<cartographer::mapping::LocalSlamResultData>
            local_slam_result_data) {
      map_builder_server_->incoming_data_queue_.Push(
          cartographer::common::make_unique<MapBuilderServer::Data>(
              MapBuilderServer::Data{trajectory_id,
                                     std::move(local_slam_result_data)}));
    }

   private:
    std::shared_ptr<cartographer::mapping_2d::Submap> UpdateSubmap2D(
        const cartographer::mapping::proto::Submap& proto);
    std::shared_ptr<cartographer::mapping_3d::Submap> UpdateSubmap3D(
        const cartographer::mapping::proto::Submap& proto);

    MapBuilderServer* map_builder_server_;
    cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        std::shared_ptr<cartographer::mapping::Submap>>
        unfinished_submaps_;
  };
  friend MapBuilderContext;

  MapBuilderServer(
      const proto::MapBuilderServerOptions& map_builder_server_options,
      std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder);

  // Starts the gRPC server and the SLAM thread.
  void Start();

  // Waits for the 'MapBuilderServer' to shut down. Note: The server must be
  // either shutting down or some other thread must call 'Shutdown()' for this
  // function to ever return.
  void WaitForShutdown();

  // Waits until all computation is finished (for testing).
  void WaitUntilIdle();

  // Shuts down the gRPC server and the SLAM thread.
  void Shutdown();

 private:
  using LocalSlamResultHandlerSubscriptions =
      std::map<int /* subscription_index */, LocalSlamSubscriptionCallback>;

  void ProcessSensorDataQueue();
  void StartSlamThread();
  void OnLocalSlamResult(
      int trajectory_id, cartographer::common::Time time,
      cartographer::transform::Rigid3d local_pose,
      cartographer::sensor::RangeData range_data,
      std::unique_ptr<const cartographer::mapping::TrajectoryBuilderInterface::
                          InsertionResult>
          insertion_result);
  SubscriptionId SubscribeLocalSlamResults(
      int trajectory_id, LocalSlamSubscriptionCallback callback);
  void UnsubscribeLocalSlamResults(const SubscriptionId& subscription_id);
  void NotifyFinishTrajectory(int trajectory_id);

  bool shutting_down_ = false;
  std::unique_ptr<std::thread> slam_thread_;
  std::unique_ptr<framework::Server> grpc_server_;
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
  cartographer::common::BlockingQueue<std::unique_ptr<Data>>
      incoming_data_queue_;
  cartographer::common::Mutex local_slam_subscriptions_lock_;
  int current_subscription_index_ = 0;
  std::map<int /* trajectory ID */, LocalSlamResultHandlerSubscriptions>
      local_slam_subscriptions_ GUARDED_BY(local_slam_subscriptions_lock_);
};

}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAP_BUILDER_SERVER_H
