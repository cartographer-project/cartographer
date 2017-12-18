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
#include "cartographer/mapping/map_builder.h"
#include "cartographer/sensor/data.h"
#include "cartographer_grpc/framework/execution_context.h"
#include "cartographer_grpc/framework/server.h"
#include "cartographer_grpc/proto/map_builder_server_options.pb.h"

namespace cartographer_grpc {

class MapBuilderServer {
 public:
  struct SensorData {
    int trajectory_id;
    std::unique_ptr<cartographer::sensor::Data> sensor_data;
  };

  class MapBuilderContext : public framework::ExecutionContext {
   public:
    MapBuilderContext(
        cartographer::mapping::MapBuilder* map_builder,
        cartographer::common::BlockingQueue<std::unique_ptr<SensorData>>*
            sensor_data_queue);
    cartographer::mapping::MapBuilder& map_builder();
    cartographer::common::BlockingQueue<std::unique_ptr<SensorData>>&
    sensor_data_queue();
    void AddSensorDataToTrajectory(const SensorData& sensor_data);

    template <typename SensorDataType>
    void EnqueueSensorData(int trajectory_id, const std::string& sensor_id,
                           const SensorDataType& sensor_data) {
      sensor_data_queue_->Push(
          cartographer::common::make_unique<MapBuilderServer::SensorData>(
              MapBuilderServer::SensorData{
                  trajectory_id, cartographer::sensor::MakeDispatchable(
                                     sensor_id, sensor_data)}));
    }

   private:
    cartographer::mapping::MapBuilder* map_builder_;
    cartographer::common::BlockingQueue<std::unique_ptr<SensorData>>*
        sensor_data_queue_;
  };

  MapBuilderServer(
      const proto::MapBuilderServerOptions& map_builder_server_options);

  // Starts the gRPC server and the SLAM thread.
  void Start();

  // Waits for the 'MapBuilderServer' to shut down. Note: The server must be
  // either shutting down or some other thread must call 'Shutdown()' for this
  // function to ever return.
  void WaitForShutdown();

  // Shuts down the gRPC server and the SLAM thread.
  void Shutdown();

 private:
  void ProcessSensorDataQueue();
  void StartSlamThread();

  bool shutting_down_ = false;
  std::unique_ptr<std::thread> slam_thread_;
  std::unique_ptr<framework::Server> grpc_server_;
  cartographer::mapping::MapBuilder map_builder_;
  cartographer::common::BlockingQueue<std::unique_ptr<SensorData>>
      sensor_data_queue_;
};

}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAP_BUILDER_SERVER_H
