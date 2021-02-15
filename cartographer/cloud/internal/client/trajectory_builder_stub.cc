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

#include "cartographer/cloud/internal/client/trajectory_builder_stub.h"

#include "cartographer/cloud/internal/sensor/serialization.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace cloud {

TrajectoryBuilderStub::TrajectoryBuilderStub(
    std::shared_ptr<::grpc::Channel> client_channel, const int trajectory_id,
    const std::string& client_id,
    LocalSlamResultCallback local_slam_result_callback)
    : client_channel_(client_channel),
      trajectory_id_(trajectory_id),
      client_id_(client_id),
      receive_local_slam_results_client_(client_channel_) {
  if (local_slam_result_callback) {
    proto::ReceiveLocalSlamResultsRequest request;
    request.set_trajectory_id(trajectory_id);
    receive_local_slam_results_client_.Write(request);
    auto* receive_local_slam_results_client_ptr =
        &receive_local_slam_results_client_;
    receive_local_slam_results_thread_ = absl::make_unique<std::thread>(
        [receive_local_slam_results_client_ptr, local_slam_result_callback]() {
          RunLocalSlamResultsReader(receive_local_slam_results_client_ptr,
                                    local_slam_result_callback);
        });
  }
}

TrajectoryBuilderStub::~TrajectoryBuilderStub() {
  if (receive_local_slam_results_thread_) {
    receive_local_slam_results_thread_->join();
  }
  if (add_rangefinder_client_) {
    CHECK(add_rangefinder_client_->StreamWritesDone());
    CHECK(add_rangefinder_client_->StreamFinish().ok());
  }
  if (add_imu_client_) {
    CHECK(add_imu_client_->StreamWritesDone());
    CHECK(add_imu_client_->StreamFinish().ok());
  }
  if (add_odometry_client_) {
    CHECK(add_odometry_client_->StreamWritesDone());
    CHECK(add_odometry_client_->StreamFinish().ok());
  }
  if (add_landmark_client_) {
    CHECK(add_landmark_client_->StreamWritesDone());
    CHECK(add_landmark_client_->StreamFinish().ok());
  }
  if (add_fixed_frame_pose_client_) {
    CHECK(add_fixed_frame_pose_client_->StreamWritesDone());
    CHECK(add_fixed_frame_pose_client_->StreamFinish().ok());
  }
}

void TrajectoryBuilderStub::AddSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) {
  if (!add_rangefinder_client_) {
    add_rangefinder_client_ = absl::make_unique<
        async_grpc::Client<handlers::AddRangefinderDataSignature>>(
        client_channel_);
  }
  proto::AddRangefinderDataRequest request;
  CreateAddRangeFinderDataRequest(sensor_id, trajectory_id_, client_id_,
                                  sensor::ToProto(timed_point_cloud_data),
                                  &request);
  add_rangefinder_client_->Write(request);
}

void TrajectoryBuilderStub::AddSensorData(const std::string& sensor_id,
                                          const sensor::ImuData& imu_data) {
  if (!add_imu_client_) {
    add_imu_client_ =
        absl::make_unique<async_grpc::Client<handlers::AddImuDataSignature>>(
            client_channel_);
  }
  proto::AddImuDataRequest request;
  CreateAddImuDataRequest(sensor_id, trajectory_id_, client_id_,
                          sensor::ToProto(imu_data), &request);
  add_imu_client_->Write(request);
}

void TrajectoryBuilderStub::AddSensorData(
    const std::string& sensor_id, const sensor::OdometryData& odometry_data) {
  if (!add_odometry_client_) {
    add_odometry_client_ = absl::make_unique<
        async_grpc::Client<handlers::AddOdometryDataSignature>>(
        client_channel_);
  }
  proto::AddOdometryDataRequest request;
  CreateAddOdometryDataRequest(sensor_id, trajectory_id_, client_id_,
                               sensor::ToProto(odometry_data), &request);
  add_odometry_client_->Write(request);
}

void TrajectoryBuilderStub::AddSensorData(
    const std::string& sensor_id,
    const sensor::FixedFramePoseData& fixed_frame_pose) {
  if (!add_fixed_frame_pose_client_) {
    add_fixed_frame_pose_client_ = absl::make_unique<
        async_grpc::Client<handlers::AddFixedFramePoseDataSignature>>(
        client_channel_);
  }
  proto::AddFixedFramePoseDataRequest request;
  CreateAddFixedFramePoseDataRequest(sensor_id, trajectory_id_, client_id_,
                                     sensor::ToProto(fixed_frame_pose),
                                     &request);
  add_fixed_frame_pose_client_->Write(request);
}

void TrajectoryBuilderStub::AddSensorData(
    const std::string& sensor_id, const sensor::LandmarkData& landmark_data) {
  if (!add_landmark_client_) {
    add_landmark_client_ = absl::make_unique<
        async_grpc::Client<handlers::AddLandmarkDataSignature>>(
        client_channel_);
  }
  proto::AddLandmarkDataRequest request;
  CreateAddLandmarkDataRequest(sensor_id, trajectory_id_, client_id_,
                               sensor::ToProto(landmark_data), &request);
  add_landmark_client_->Write(request);
}

void TrajectoryBuilderStub::AddLocalSlamResultData(
    std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) {
  LOG(FATAL) << "Not implemented";
}

void TrajectoryBuilderStub::RunLocalSlamResultsReader(
    async_grpc::Client<handlers::ReceiveLocalSlamResultsSignature>* client,
    LocalSlamResultCallback local_slam_result_callback) {
  proto::ReceiveLocalSlamResultsResponse response;
  while (client->StreamRead(&response)) {
    int trajectory_id = response.trajectory_id();
    common::Time time = common::FromUniversal(response.timestamp());
    transform::Rigid3d local_pose = transform::ToRigid3(response.local_pose());
    sensor::RangeData range_data = sensor::FromProto(response.range_data());
    auto insertion_result =
        response.has_insertion_result()
            ? absl::make_unique<InsertionResult>(
                  InsertionResult{mapping::NodeId{
                      response.insertion_result().node_id().trajectory_id(),
                      response.insertion_result().node_id().node_index()}})
            : nullptr;
    local_slam_result_callback(trajectory_id, time, local_pose, range_data,
                               std::move(insertion_result));
  }
  client->StreamFinish();
}

}  // namespace cloud
}  // namespace cartographer
