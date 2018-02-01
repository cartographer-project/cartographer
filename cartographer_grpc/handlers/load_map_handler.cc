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

#include "cartographer_grpc/handlers/load_map_handler.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/in_memory_proto_stream.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_context_interface.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

void LoadMapHandler::OnRequest(const proto::LoadMapRequest& request) {
  switch (request.map_chunk_case()) {
    case proto::LoadMapRequest::kPoseGraph:
      reader_.AddProto(request.pose_graph());
      break;
    case proto::LoadMapRequest::kSerializedData:
      reader_.AddProto(request.serialized_data());
      break;
    default:
      LOG(FATAL) << "Unhandled proto::LoadMapRequest case.";
  }
}

void LoadMapHandler::OnReadsDone() {
  GetContext<MapBuilderContextInterface>()->map_builder().LoadMap(&reader_);
  Send(cartographer::common::make_unique<google::protobuf::Empty>());
}

}  // namespace handlers
}  // namespace cartographer_grpc
