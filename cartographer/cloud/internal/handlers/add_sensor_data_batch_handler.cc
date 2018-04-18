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

#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/sensor/internal/dispatchable.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void AddSensorDataBatchHandler::OnRequest(
    const proto::AddSensorDataBatchRequest& request) {
  for (const proto::SensorData& sensor_data : request.sensor_data()) {
    switch (sensor_data.sensor_data_case()) {
      case proto::SensorData::kOdometryData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(
                sensor_data.sensor_metadata().trajectory_id(),
                sensor::MakeDispatchable(
                    sensor_data.sensor_metadata().sensor_id(),
                    sensor::FromProto(sensor_data.odometry_data())));
        break;
      case proto::SensorData::kImuData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(sensor_data.sensor_metadata().trajectory_id(),
                                sensor::MakeDispatchable(
                                    sensor_data.sensor_metadata().sensor_id(),
                                    sensor::FromProto(sensor_data.imu_data())));
        break;
      case proto::SensorData::kTimedPointCloudData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(
                sensor_data.sensor_metadata().trajectory_id(),
                sensor::MakeDispatchable(
                    sensor_data.sensor_metadata().sensor_id(),
                    sensor::FromProto(sensor_data.timed_point_cloud_data())));
        break;
      case proto::SensorData::kFixedFramePoseData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(
                sensor_data.sensor_metadata().trajectory_id(),
                sensor::MakeDispatchable(
                    sensor_data.sensor_metadata().sensor_id(),
                    sensor::FromProto(sensor_data.fixed_frame_pose_data())));
        break;
      case proto::SensorData::kLandmarkData:
        GetUnsynchronizedContext<MapBuilderContextInterface>()
            ->EnqueueSensorData(
                sensor_data.sensor_metadata().trajectory_id(),
                sensor::MakeDispatchable(
                    sensor_data.sensor_metadata().sensor_id(),
                    sensor::FromProto(sensor_data.landmark_data())));
        break;
      case proto::SensorData::kLocalSlamResultData:
        GetContext<MapBuilderContextInterface>()->EnqueueLocalSlamResultData(
            sensor_data.sensor_metadata().trajectory_id(),
            sensor_data.sensor_metadata().sensor_id(),
            sensor_data.local_slam_result_data());
        break;
      default:
        LOG(FATAL) << "Unknown sensor data type: "
                   << sensor_data.sensor_data_case();
    }
  }
  Send(common::make_unique<google::protobuf::Empty>());
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
