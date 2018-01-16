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

#ifndef CARTOGRAPHER_GRPC_SENSOR_SERIALIZATION_H
#define CARTOGRAPHER_GRPC_SENSOR_SERIALIZATION_H

#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"

namespace cartographer_grpc {
namespace sensor {

void CreateSensorMetadata(const std::string& sensor_id, int trajectory_id,
                          proto::SensorMetadata* proto);
void CreateAddFixedFramePoseDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const cartographer::sensor::proto::FixedFramePoseData&
        fixed_frame_pose_data,
    proto::AddFixedFramePoseDataRequest* proto);
void CreateAddImuDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const cartographer::sensor::proto::ImuData& imu_data,
    proto::AddImuDataRequest* proto);
void CreateAddOdometryDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const cartographer::sensor::proto::OdometryData& odometry_data,
    proto::AddOdometryDataRequest* proto);
void CreateAddRangeFinderDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const cartographer::sensor::proto::TimedPointCloudData&
        timed_point_cloud_data,
    proto::AddRangefinderDataRequest* proto);

}  // namespace sensor
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_SENSOR_SERIALIZATION_H
