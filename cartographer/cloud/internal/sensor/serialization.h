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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_SENSOR_SERIALIZATION_H
#define CARTOGRAPHER_CLOUD_INTERNAL_SENSOR_SERIALIZATION_H

#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace cloud {

void CreateSensorMetadata(const std::string& sensor_id, int trajectory_id,
                          const std::string& client_id,
                          proto::SensorMetadata* proto);

void CreateAddFixedFramePoseDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id,
    const sensor::proto::FixedFramePoseData& fixed_frame_pose_data,
    proto::AddFixedFramePoseDataRequest* proto);
void CreateAddImuDataRequest(const std::string& sensor_id, int trajectory_id,
                             const std::string& client_id,
                             const sensor::proto::ImuData& imu_data,
                             proto::AddImuDataRequest* proto);
void CreateAddOdometryDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id,
    const sensor::proto::OdometryData& odometry_data,
    proto::AddOdometryDataRequest* proto);
void CreateAddRangeFinderDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id,
    const sensor::proto::TimedPointCloudData& timed_point_cloud_data,
    proto::AddRangefinderDataRequest* proto);
void CreateAddLandmarkDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id,
    const sensor::proto::LandmarkData& landmark_data,
    proto::AddLandmarkDataRequest* proto);
void CreateSensorDataForLocalSlamResult(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id, common::Time time, int starting_submap_index,
    const mapping::TrajectoryBuilderInterface::InsertionResult&
        insertion_result,
    proto::SensorData* proto);

proto::SensorId ToProto(
    const mapping::TrajectoryBuilderInterface::SensorId& sensor_id);
mapping::TrajectoryBuilderInterface::SensorId FromProto(
    const proto::SensorId& proto);

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_SENSOR_SERIALIZATION_H
