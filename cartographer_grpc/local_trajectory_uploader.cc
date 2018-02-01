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

#include "cartographer_grpc/local_trajectory_uploader.h"
#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/handlers/add_trajectory_handler.h"
#include "cartographer_grpc/handlers/finish_trajectory_handler.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "cartographer_grpc/sensor/serialization.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace {

const cartographer::common::Duration kPopTimeout =
    cartographer::common::FromMilliseconds(100);

}  // namespace

LocalTrajectoryUploader::LocalTrajectoryUploader(
    const std::string &uplink_server_address)
    : client_channel_(grpc::CreateChannel(
          uplink_server_address, grpc::InsecureChannelCredentials())) {}

LocalTrajectoryUploader::~LocalTrajectoryUploader() {
  if (add_imu_client_) {
    CHECK(add_imu_client_->WritesDone());
    CHECK(add_imu_client_->Finish().ok());
  }
  if (add_odometry_client_) {
    CHECK(add_odometry_client_->WritesDone());
    CHECK(add_odometry_client_->Finish().ok());
  }
  if (add_fixed_frame_pose_client_) {
    CHECK(add_fixed_frame_pose_client_->WritesDone());
    CHECK(add_fixed_frame_pose_client_->Finish().ok());
  }
  if (add_local_slam_result_client_) {
    CHECK(add_local_slam_result_client_->WritesDone());
    CHECK(add_local_slam_result_client_->Finish().ok());
  }
  if (add_landmark_client_) {
    CHECK(add_landmark_client_->WritesDone());
    CHECK(add_landmark_client_->Finish().ok());
  }
}

void LocalTrajectoryUploader::Start() {
  CHECK(!shutting_down_);
  CHECK(!upload_thread_);
  upload_thread_ = cartographer::common::make_unique<std::thread>(
      [this]() { this->ProcessSendQueue(); });
}

void LocalTrajectoryUploader::Shutdown() {
  CHECK(!shutting_down_);
  CHECK(upload_thread_);
  shutting_down_ = true;
  upload_thread_->join();
}

void LocalTrajectoryUploader::ProcessSendQueue() {
  LOG(INFO) << "Starting uploader thread.";
  while (!shutting_down_) {
    auto data_message = send_queue_.PopWithTimeout(kPopTimeout);
    if (data_message) {
      if (auto *fixed_frame_pose_data =
              dynamic_cast<proto::AddFixedFramePoseDataRequest *>(
                  data_message.get())) {
        ProcessFixedFramePoseDataMessage(fixed_frame_pose_data);
      } else if (auto *imu_data = dynamic_cast<proto::AddImuDataRequest *>(
                     data_message.get())) {
        ProcessImuDataMessage(imu_data);
      } else if (auto *odometry_data =
                     dynamic_cast<proto::AddOdometryDataRequest *>(
                         data_message.get())) {
        ProcessOdometryDataMessage(odometry_data);
      } else if (auto *local_slam_result_data =
                     dynamic_cast<proto::AddLocalSlamResultDataRequest *>(
                         data_message.get())) {
        ProcessLocalSlamResultDataMessage(local_slam_result_data);
      } else if (auto *landmark_data =
                     dynamic_cast<proto::AddLandmarkDataRequest *>(
                         data_message.get())) {
        ProcessLandmarkDataMessage(landmark_data);
      } else {
        LOG(FATAL) << "Unknown message type: " << data_message->GetTypeName();
      }
    }
  }
}

void LocalTrajectoryUploader::TranslateTrajectoryId(
    proto::SensorMetadata *sensor_metadata) {
  int cloud_trajectory_id =
      local_to_cloud_trajectory_id_map_.at(sensor_metadata->trajectory_id());
  sensor_metadata->set_trajectory_id(cloud_trajectory_id);
}

void LocalTrajectoryUploader::ProcessFixedFramePoseDataMessage(
    proto::AddFixedFramePoseDataRequest *data_request) {
  if (!add_fixed_frame_pose_client_) {
    add_fixed_frame_pose_client_ = cartographer::common::make_unique<
        framework::Client<handlers::AddFixedFramePoseDataHandler>>(
        client_channel_);
  }
  TranslateTrajectoryId(data_request->mutable_sensor_metadata());
  CHECK(add_fixed_frame_pose_client_->Write(*data_request));
}

void LocalTrajectoryUploader::ProcessImuDataMessage(
    proto::AddImuDataRequest *data_request) {
  if (!add_imu_client_) {
    add_imu_client_ = cartographer::common::make_unique<
        framework::Client<handlers::AddImuDataHandler>>(client_channel_);
  }
  TranslateTrajectoryId(data_request->mutable_sensor_metadata());
  CHECK(add_imu_client_->Write(*data_request));
}

void LocalTrajectoryUploader::ProcessOdometryDataMessage(
    proto::AddOdometryDataRequest *data_request) {
  if (!add_odometry_client_) {
    add_odometry_client_ = cartographer::common::make_unique<
        framework::Client<handlers::AddOdometryDataHandler>>(client_channel_);
  }
  TranslateTrajectoryId(data_request->mutable_sensor_metadata());
  CHECK(add_odometry_client_->Write(*data_request));
}

void LocalTrajectoryUploader::ProcessLandmarkDataMessage(
    proto::AddLandmarkDataRequest *data_request) {
  if (!add_landmark_client_) {
    add_landmark_client_ = cartographer::common::make_unique<
        framework::Client<handlers::AddLandmarkDataHandler>>(client_channel_);
  }
  TranslateTrajectoryId(data_request->mutable_sensor_metadata());
  CHECK(add_landmark_client_->Write(*data_request));
}

void LocalTrajectoryUploader::ProcessLocalSlamResultDataMessage(
    proto::AddLocalSlamResultDataRequest *data_request) {
  if (!add_local_slam_result_client_) {
    add_local_slam_result_client_ = cartographer::common::make_unique<
        framework::Client<handlers::AddLocalSlamResultDataHandler>>(
        client_channel_);
  }
  TranslateTrajectoryId(data_request->mutable_sensor_metadata());
  // A submap also holds a trajectory id that must be translated to uplink's
  // trajectory id.
  for (cartographer::mapping::proto::Submap &mutable_submap :
       *data_request->mutable_local_slam_result_data()->mutable_submaps()) {
    mutable_submap.mutable_submap_id()->set_trajectory_id(
        data_request->sensor_metadata().trajectory_id());
  }
  CHECK(add_local_slam_result_client_->Write(*data_request));
}

void LocalTrajectoryUploader::AddTrajectory(
    int local_trajectory_id, const std::set<SensorId> &expected_sensor_ids,
    const cartographer::mapping::proto::TrajectoryBuilderOptions
        &trajectory_options) {
  proto::AddTrajectoryRequest request;
  *request.mutable_trajectory_builder_options() = trajectory_options;
  for (const SensorId &sensor_id : expected_sensor_ids) {
    // Range sensors are not forwarded, but combined into a LocalSlamResult.
    if (sensor_id.type != SensorId::SensorType::RANGE) {
      *request.add_expected_sensor_ids() = sensor::ToProto(sensor_id);
    }
  }
  *request.add_expected_sensor_ids() =
      sensor::ToProto(GetLocalSlamResultSensorId(local_trajectory_id));
  framework::Client<handlers::AddTrajectoryHandler> client(client_channel_);
  CHECK(client.Write(request));
  CHECK_EQ(local_to_cloud_trajectory_id_map_.count(local_trajectory_id), 0);
  local_to_cloud_trajectory_id_map_[local_trajectory_id] =
      client.response().trajectory_id();
}

void LocalTrajectoryUploader::FinishTrajectory(int local_trajectory_id) {
  CHECK_EQ(local_to_cloud_trajectory_id_map_.count(local_trajectory_id), 1);
  int cloud_trajectory_id =
      local_to_cloud_trajectory_id_map_[local_trajectory_id];
  proto::FinishTrajectoryRequest request;
  request.set_trajectory_id(cloud_trajectory_id);
  framework::Client<handlers::FinishTrajectoryHandler> client(client_channel_);
  CHECK(client.Write(request));
}

void LocalTrajectoryUploader::EnqueueDataRequest(
    std::unique_ptr<google::protobuf::Message> data_request) {
  send_queue_.Push(std::move(data_request));
}

}  // namespace cartographer_grpc
