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
#include "cartographer_grpc/handlers/add_trajectory_handler.h"
#include "cartographer_grpc/handlers/finish_trajectory_handler.h"
#include "cartographer_grpc/handlers/get_submap_handler.h"
#include "cartographer_grpc/handlers/load_map_handler.h"
#include "cartographer_grpc/handlers/write_map_handler.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "cartographer_grpc/sensor/serialization.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace mapping {

MapBuilderStub::MapBuilderStub(const std::string& server_address)
    : client_channel_(grpc::CreateChannel(server_address,
                                          grpc::InsecureChannelCredentials())),
      pose_graph_stub_(client_channel_) {}

int MapBuilderStub::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const cartographer::mapping::proto::TrajectoryBuilderOptions&
        trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  proto::AddTrajectoryRequest request;
  *request.mutable_trajectory_builder_options() = trajectory_options;
  for (const auto& sensor_id : expected_sensor_ids) {
    *request.add_expected_sensor_ids() = sensor::ToProto(sensor_id);
  }
  framework::Client<handlers::AddTrajectoryHandler> client(
      client_channel_,
      framework::CreateLimitedBackoffStrategy(
          cartographer::common::FromMilliseconds(100), 2.f, 5));
  CHECK(client.Write(request));

  // Construct trajectory builder stub.
  trajectory_builder_stubs_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(client.response().trajectory_id()),
      std::forward_as_tuple(cartographer::common::make_unique<
                            cartographer_grpc::mapping::TrajectoryBuilderStub>(
          client_channel_, client.response().trajectory_id(),
          local_slam_result_callback)));
  return client.response().trajectory_id();
}

int MapBuilderStub::AddTrajectoryForDeserialization(
    const cartographer::mapping::proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  LOG(FATAL) << "Not implemented";
}

cartographer::mapping::TrajectoryBuilderInterface*
MapBuilderStub::GetTrajectoryBuilder(int trajectory_id) const {
  return trajectory_builder_stubs_.at(trajectory_id).get();
}

void MapBuilderStub::FinishTrajectory(int trajectory_id) {
  proto::FinishTrajectoryRequest request;
  request.set_trajectory_id(trajectory_id);
  framework::Client<handlers::FinishTrajectoryHandler> client(client_channel_);
  CHECK(client.Write(request));
  trajectory_builder_stubs_.erase(trajectory_id);
}

std::string MapBuilderStub::SubmapToProto(
    const cartographer::mapping::SubmapId& submap_id,
    cartographer::mapping::proto::SubmapQuery::Response*
        submap_query_response) {
  proto::GetSubmapRequest request;
  submap_id.ToProto(request.mutable_submap_id());
  framework::Client<handlers::GetSubmapHandler> client(client_channel_);
  CHECK(client.Write(request));
  submap_query_response->CopyFrom(client.response().submap_query_response());
  return client.response().error_msg();
}

void MapBuilderStub::SerializeState(
    cartographer::io::ProtoStreamWriterInterface* writer) {
  google::protobuf::Empty request;
  framework::Client<handlers::WriteMapHandler> client(client_channel_);
  CHECK(client.Write(request));
  proto::WriteMapResponse response;
  while (client.Read(&response)) {
    // writer->WriteProto(response);
    switch (response.map_chunk_case()) {
      case proto::WriteMapResponse::kPoseGraph:
        writer->WriteProto(response.pose_graph());
        break;
      case proto::WriteMapResponse::kAllTrajectoryBuilderOptions:
        writer->WriteProto(response.all_trajectory_builder_options());
        break;
      case proto::WriteMapResponse::kSerializedData:
        writer->WriteProto(response.serialized_data());
        break;
      default:
        LOG(FATAL) << "Unhandled message type";
    }
  }
  CHECK(writer->Close());
}

void MapBuilderStub::LoadMap(
    cartographer::io::ProtoStreamReaderInterface* reader) {
  framework::Client<handlers::LoadMapHandler> client(client_channel_);
  // Request with a PoseGraph proto is sent first.
  {
    proto::LoadMapRequest request;
    CHECK(reader->ReadProto(request.mutable_pose_graph()));
    CHECK(client.Write(request));
  }
  // Request with an AllTrajectoryBuilderOptions should be second.
  {
    proto::LoadMapRequest request;
    CHECK(reader->ReadProto(request.mutable_all_trajectory_builder_options()));
    CHECK(client.Write(request));
  }
  // Multiple requests with SerializedData are sent after.
  proto::LoadMapRequest request;
  while (reader->ReadProto(request.mutable_serialized_data())) {
    CHECK(client.Write(request));
  }

  CHECK(reader->eof());
  CHECK(client.WritesDone());
  CHECK(client.Finish().ok());
}

int MapBuilderStub::num_trajectory_builders() const {
  return trajectory_builder_stubs_.size();
}

cartographer::mapping::PoseGraphInterface* MapBuilderStub::pose_graph() {
  return &pose_graph_stub_;
}

const std::vector<
    cartographer::mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
MapBuilderStub::GetAllTrajectoryBuilderOptions() const {
  LOG(FATAL) << "Not implemented";
}

}  // namespace mapping
}  // namespace cartographer_grpc
