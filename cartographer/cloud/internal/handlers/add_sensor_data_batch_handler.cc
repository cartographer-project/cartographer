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

#include "cartographer/cloud/internal/handlers/add_sensor_data_batch_handler.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/sensor/internal/dispatchable.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

metrics::Family<metrics::Counter>*
    AddSensorDataBatchHandler::counter_metrics_family_ =
        metrics::Family<metrics::Counter>::Null();

void AddSensorDataBatchHandler::OnRequest(
    const proto::AddSensorDataBatchRequest& request) {
  for (const proto::SensorData& sensor_data : request.sensor_data()) {
    if (!GetContext<MapBuilderContextInterface>()->CheckClientIdForTrajectory(
            sensor_data.sensor_metadata().client_id(),
            sensor_data.sensor_metadata().trajectory_id())) {
      LOG(ERROR) << "Unknown trajectory with ID "
                 << sensor_data.sensor_metadata().trajectory_id()
                 << " and client_id "
                 << sensor_data.sensor_metadata().client_id();
      Finish(::grpc::Status(::grpc::NOT_FOUND, "Unknown trajectory"));
      return;
    }
    ClientMetrics* const metrics =
        GetOrCreateClientMetrics(sensor_data.sensor_metadata().client_id(),
                                 sensor_data.sensor_metadata().trajectory_id());
    switch (sensor_data.sensor_data_case()) {
      case proto::SensorData::kOdometryData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(
                sensor_data.sensor_metadata().trajectory_id(),
                sensor::MakeDispatchable(
                    sensor_data.sensor_metadata().sensor_id(),
                    sensor::FromProto(sensor_data.odometry_data())));
        metrics->odometry_sensor_counter->Increment();
        break;
      case proto::SensorData::kImuData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(sensor_data.sensor_metadata().trajectory_id(),
                                sensor::MakeDispatchable(
                                    sensor_data.sensor_metadata().sensor_id(),
                                    sensor::FromProto(sensor_data.imu_data())));
        metrics->imu_sensor_counter->Increment();
        break;
      case proto::SensorData::kTimedPointCloudData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(
                sensor_data.sensor_metadata().trajectory_id(),
                sensor::MakeDispatchable(
                    sensor_data.sensor_metadata().sensor_id(),
                    sensor::FromProto(sensor_data.timed_point_cloud_data())));
        metrics->timed_point_cloud_counter->Increment();
        break;
      case proto::SensorData::kFixedFramePoseData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(
                sensor_data.sensor_metadata().trajectory_id(),
                sensor::MakeDispatchable(
                    sensor_data.sensor_metadata().sensor_id(),
                    sensor::FromProto(sensor_data.fixed_frame_pose_data())));
        metrics->fixed_frame_pose_counter->Increment();
        break;
      case proto::SensorData::kLandmarkData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(
                sensor_data.sensor_metadata().trajectory_id(),
                sensor::MakeDispatchable(
                    sensor_data.sensor_metadata().sensor_id(),
                    sensor::FromProto(sensor_data.landmark_data())));
        metrics->landmark_counter->Increment();
        break;
      case proto::SensorData::kLocalSlamResultData:
        GetContext<MapBuilderContextInterface>()->EnqueueLocalSlamResultData(
            sensor_data.sensor_metadata().trajectory_id(),
            sensor_data.sensor_metadata().sensor_id(),
            sensor_data.local_slam_result_data());
        metrics->local_slam_result_counter->Increment();
        break;
      default:
        LOG(FATAL) << "Unknown sensor data type: "
                   << sensor_data.sensor_data_case();
    }
  }
  Send(absl::make_unique<google::protobuf::Empty>());
}

void AddSensorDataBatchHandler::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  counter_metrics_family_ = family_factory->NewCounterFamily(
      "cartographer_sensor_data_total", "Sensor data received");
}

AddSensorDataBatchHandler::ClientMetrics*
AddSensorDataBatchHandler::GetOrCreateClientMetrics(
    const std::string& client_id, int trajectory_id) {
  const std::string map_key = client_id + std::to_string(trajectory_id);
  auto client_metric_map_itr = client_metric_map_.find(map_key);
  if (client_metric_map_itr != client_metric_map_.end()) {
    return client_metric_map_itr->second.get();
  }

  auto new_metrics = absl::make_unique<ClientMetrics>();
  new_metrics->odometry_sensor_counter = counter_metrics_family_->Add(
      {{"client_id", client_id},
       {"trajectory_id", std::to_string(trajectory_id)},
       {"sensor", "odometry"}});
  new_metrics->imu_sensor_counter = counter_metrics_family_->Add(
      {{"client_id", client_id},
       {"trajectory_id", std::to_string(trajectory_id)},
       {"sensor", "imu"}});
  new_metrics->fixed_frame_pose_counter = counter_metrics_family_->Add(
      {{"client_id", client_id},
       {"trajectory_id", std::to_string(trajectory_id)},
       {"sensor", "fixed_frame_pose"}});
  new_metrics->landmark_counter = counter_metrics_family_->Add(
      {{"client_id", client_id},
       {"trajectory_id", std::to_string(trajectory_id)},
       {"sensor", "landmark"}});
  new_metrics->local_slam_result_counter = counter_metrics_family_->Add(
      {{"client_id", client_id},
       {"trajectory_id", std::to_string(trajectory_id)},
       {"sensor", "local_slam_result"}});
  new_metrics->timed_point_cloud_counter = counter_metrics_family_->Add(
      {{"client_id", client_id},
       {"trajectory_id", std::to_string(trajectory_id)},
       {"sensor", "timed_point_cloud"}});

  // Obtain pointer before ownership is transferred.
  auto* new_metrics_ptr = new_metrics.get();
  client_metric_map_[map_key] = std::move(new_metrics);
  return new_metrics_ptr;
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
