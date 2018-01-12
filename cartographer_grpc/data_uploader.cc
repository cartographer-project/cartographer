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

#include "cartographer_grpc/data_uploader.h"

#include "glog/logging.h"

namespace cartographer_grpc {

DataUploader::DataUploader(const std::string& server_address)
    : client_channel_(grpc::CreateChannel(server_address,
                                          grpc::InsecureChannelCredentials())),
      service_stub_(proto::MapBuilderService::NewStub(client_channel_)) {}

void DataUploader::AddTrajectory(
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

void DataUploader::FinishTrajectory(int local_trajectory_id) {
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

}  // namespace cartographer_grpc