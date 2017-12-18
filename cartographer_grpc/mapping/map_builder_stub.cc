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

#include "cartographer_grpc/mapping/map_builder_stub.h"

#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace mapping {

MapBuilderStub::MapBuilderStub(const std::string& server_address)
    : client_channel_(grpc::CreateChannel(server_address,
                                          grpc::InsecureChannelCredentials())),
      service_stub_(proto::MapBuilderService::NewStub(client_channel_)),
      pose_graph_stub_(client_channel_, service_stub_.get()) {}

int MapBuilderStub::AddTrajectoryBuilder(
    const std::unordered_set<std::string>& expected_sensor_ids,
    const cartographer::mapping::proto::TrajectoryBuilderOptions&
        trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  proto::AddTrajectoryRequest request;
  proto::AddTrajectoryResponse result;
  *request.mutable_trajectory_builder_options() = trajectory_options;
  for (const auto& sensor_id : expected_sensor_ids) {
    *request.add_expected_sensor_ids() = sensor_id;
  }
  grpc::Status status =
      service_stub_->AddTrajectory(&client_context_, request, &result);
  CHECK(status.ok());

  // Construct trajectory builder stub.
  trajectory_builder_stubs_.emplace(
      std::piecewise_construct, std::forward_as_tuple(result.trajectory_id()),
      std::forward_as_tuple(cartographer::common::make_unique<
                            cartographer_grpc::mapping::TrajectoryBuilderStub>(
          client_channel_, result.trajectory_id())));
  return result.trajectory_id();
}

int MapBuilderStub::AddTrajectoryForDeserialization() {
  LOG(FATAL) << "Not implemented";
}

cartographer::mapping::TrajectoryBuilderInterface*
MapBuilderStub::GetTrajectoryBuilder(int trajectory_id) const {
  return trajectory_builder_stubs_.at(trajectory_id).get();
}

void MapBuilderStub::FinishTrajectory(int trajectory_id) {
  proto::FinishTrajectoryRequest request;
  google::protobuf::Empty response;
  request.set_trajectory_id(trajectory_id);
  grpc::Status status =
      service_stub_->FinishTrajectory(&client_context_, request, &response);
  CHECK(status.ok());
  trajectory_builder_stubs_.erase(trajectory_id);
}

std::string MapBuilderStub::SubmapToProto(
    const cartographer::mapping::SubmapId& submap_id,
    cartographer::mapping::proto::SubmapQuery::Response* response) {
  LOG(FATAL) << "Not implemented";
}

void MapBuilderStub::SerializeState(
    cartographer::io::ProtoStreamWriter* writer) {
  LOG(FATAL) << "Not implemented";
}

void MapBuilderStub::LoadMap(cartographer::io::ProtoStreamReader* reader) {
  LOG(FATAL) << "Not implemented";
}

int MapBuilderStub::num_trajectory_builders() const {
  LOG(FATAL) << "Not implemented";
}

cartographer::mapping::PoseGraphInterface* MapBuilderStub::pose_graph() {
  return &pose_graph_stub_;
}

}  // namespace mapping
}  // namespace cartographer_grpc
