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

#include "cartographer_grpc/internal/handlers/load_state_handler.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/in_memory_proto_stream.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_context_interface.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

void LoadStateHandler::OnRequest(const proto::LoadStateRequest& request) {
  switch (request.state_chunk_case()) {
    case proto::LoadStateRequest::kPoseGraph:
      reader_.AddProto(request.pose_graph());
      break;
    case proto::LoadStateRequest::kAllTrajectoryBuilderOptions:
      reader_.AddProto(request.all_trajectory_builder_options());
      break;
    case proto::LoadStateRequest::kSerializedData:
      reader_.AddProto(request.serialized_data());
      break;
    default:
      LOG(FATAL) << "Unhandled proto::LoadStateRequest case.";
  }
}

void LoadStateHandler::OnReadsDone() {
  GetContext<MapBuilderContextInterface>()->map_builder().LoadState(&reader_,
                                                                    true);
  Send(cartographer::common::make_unique<google::protobuf::Empty>());
}

}  // namespace handlers
}  // namespace cartographer_grpc
