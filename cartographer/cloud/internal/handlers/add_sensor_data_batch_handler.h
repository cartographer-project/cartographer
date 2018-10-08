/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_ADD_SENSOR_DATA_BATCH_HANDLER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_ADD_SENSOR_DATA_BATCH_HANDLER_H

#include <memory>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/family_factory.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    AddSensorDataBatchSignature, proto::AddSensorDataBatchRequest,
    google::protobuf::Empty,
    "/cartographer.cloud.proto.MapBuilderService/AddSensorDataBatch")

class AddSensorDataBatchHandler
    : public async_grpc::RpcHandler<AddSensorDataBatchSignature> {
 public:
  void OnRequest(const proto::AddSensorDataBatchRequest& request) override;

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  struct ClientMetrics {
    metrics::Counter* odometry_sensor_counter;
    metrics::Counter* imu_sensor_counter;
    metrics::Counter* timed_point_cloud_counter;
    metrics::Counter* fixed_frame_pose_counter;
    metrics::Counter* landmark_counter;
    metrics::Counter* local_slam_result_counter;
  };

  ClientMetrics* GetOrCreateClientMetrics(const std::string& client_id,
                                          int trajectory_id);

  static cartographer::metrics::Family<metrics::Counter>*
      counter_metrics_family_;

  // Holds individual metrics for each client.
  absl::flat_hash_map<std::string, std::unique_ptr<ClientMetrics>>
      client_metric_map_;
};

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_ADD_SENSOR_DATA_BATCH_HANDLER_H
