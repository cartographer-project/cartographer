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
#include "glog/logging.h"

namespace cartographer_grpc {
namespace {

const cartographer::common::Duration kPopTimeout =
    cartographer::common::FromMilliseconds(100);
const std::string kFixedFramePoseDataMessageName =
    "cartographer_grpc.proto.AddFixedFramePoseDataRequest";
const std::string kImuDataMessageName =
    "cartographer_grpc.proto.AddImuDataRequest";
const std::string kOdometryDataMessageName =
    "cartographer_grpc.proto.AddOdometryDataRequest";

} // namespace

LocalTrajectoryUploader::LocalTrajectoryUploader(
    const std::string& server_address)
    : client_channel_(grpc::CreateChannel(server_address,
                                          grpc::InsecureChannelCredentials())),
      service_stub_(proto::MapBuilderService::NewStub(client_channel_)) {}

LocalTrajectoryUploader::~LocalTrajectoryUploader() {
  if (upload_thread_) {
    upload_thread_->join();
  }
  if (imu_writer_.client_writer) {
    CHECK(imu_writer_.client_writer->WritesDone());
    CHECK(imu_writer_.client_writer->Finish().ok());
  }
  if (odometry_writer_.client_writer) {
    CHECK(odometry_writer_.client_writer->WritesDone());
    CHECK(odometry_writer_.client_writer->Finish().ok());
  }
  if (fixed_frame_pose_writer_.client_writer) {
    CHECK(fixed_frame_pose_writer_.client_writer->WritesDone());
    CHECK(fixed_frame_pose_writer_.client_writer->Finish().ok());
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
  LOG(INFO) << "Starting SLAM thread.";
  while (!shutting_down_) {
    auto data_message = send_queue_.PopWithTimeout(kPopTimeout);
    if (data_message) {
      const std::string type_name = data_message->GetTypeName();
      if (type_name == kFixedFramePoseDataMessageName) {
        DCHECK(dynamic_cast<const proto::AddFixedFramePoseDataRequest *>(
            data_message.get()));
        ProcessFixedFramePoseDataMessage(
            static_cast<const proto::AddFixedFramePoseDataRequest *>(
                data_message.get()));
      } else if (type_name == kImuDataMessageName) {
        DCHECK(
            dynamic_cast<const proto::AddImuDataRequest *>(data_message.get()));
        ProcessImuDataMessage(
            static_cast<const proto::AddImuDataRequest *>(data_message.get()));
      } else if (type_name == kOdometryDataMessageName) {
        DCHECK(dynamic_cast<const proto::AddOdometryDataRequest *>(
            data_message.get()));
        ProcessOdometryDataMessage(
            static_cast<const proto::AddOdometryDataRequest *>(
                data_message.get()));
      } else {
        LOG(FATAL) << "Unknown message type: " << type_name;
      }
    }
  }
}

void LocalTrajectoryUploader::ProcessFixedFramePoseDataMessage(
    const proto::AddFixedFramePoseDataRequest *data_request) {
  if (!fixed_frame_pose_writer_.client_writer) {
    fixed_frame_pose_writer_.client_writer =
        service_stub_->AddFixedFramePoseData(
            &fixed_frame_pose_writer_.client_context,
            &fixed_frame_pose_writer_.response);
    CHECK(fixed_frame_pose_writer_.client_writer);
  }
  fixed_frame_pose_writer_.client_writer->Write(*data_request);
}

void LocalTrajectoryUploader::ProcessImuDataMessage(
    const proto::AddImuDataRequest *data_request) {
  if (!imu_writer_.client_writer) {
    imu_writer_.client_writer = service_stub_->AddImuData(
        &imu_writer_.client_context, &imu_writer_.response);
    CHECK(imu_writer_.client_writer);
  }
  imu_writer_.client_writer->Write(*data_request);
}

void LocalTrajectoryUploader::ProcessOdometryDataMessage(
    const proto::AddOdometryDataRequest *data_request) {
  if (!odometry_writer_.client_writer) {
    odometry_writer_.client_writer = service_stub_->AddOdometryData(
        &odometry_writer_.client_context, &odometry_writer_.response);
    CHECK(odometry_writer_.client_writer);
  }
  odometry_writer_.client_writer->Write(*data_request);
}

void LocalTrajectoryUploader::AddTrajectory(
    int local_trajectory_id,
    const std::unordered_set<std::string>& expected_sensor_ids,
    const cartographer::mapping::proto::TrajectoryBuilderOptions&
        trajectory_options) {
  grpc::ClientContext client_context;
  proto::AddTrajectoryRequest request;
  proto::AddTrajectoryResponse result;
  *request.mutable_trajectory_builder_options() = trajectory_options;
  for (const auto& sensor_id : expected_sensor_ids) {
    *request.add_expected_sensor_ids() = sensor_id;
  }
  grpc::Status status =
      service_stub_->AddTrajectory(&client_context, request, &result);
  CHECK(status.ok());
  CHECK_EQ(local_to_cloud_trajectory_id_map_.count(local_trajectory_id), 0);
  local_to_cloud_trajectory_id_map_[local_trajectory_id] =
      result.trajectory_id();
}

void LocalTrajectoryUploader::FinishTrajectory(int local_trajectory_id) {
  CHECK_EQ(local_to_cloud_trajectory_id_map_.count(local_trajectory_id), 1);
  int cloud_trajectory_id =
      local_to_cloud_trajectory_id_map_[local_trajectory_id];
  grpc::ClientContext client_context;
  proto::FinishTrajectoryRequest request;
  google::protobuf::Empty response;
  request.set_trajectory_id(cloud_trajectory_id);
  grpc::Status status =
      service_stub_->FinishTrajectory(&client_context, request, &response);
  CHECK(status.ok());
}

void LocalTrajectoryUploader::EnqueueDataRequest(
    std::unique_ptr<google::protobuf::Message> data_request) {
  send_queue_.Push(std::move(data_request));
}

} // namespace cartographer_grpc
