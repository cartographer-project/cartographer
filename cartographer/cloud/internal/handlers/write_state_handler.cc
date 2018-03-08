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

#include "cartographer/cloud/internal/handlers/write_state_handler.h"

#include "cartographer/cloud/internal/framework/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/internal/map_builder_server.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/in_memory_proto_stream.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void WriteStateHandler::OnRequest(const google::protobuf::Empty& request) {
  auto writer = GetWriter();
  io::ForwardingProtoStreamWriter proto_stream_writer(
      [writer](const google::protobuf::Message* proto) {
        if (!proto) {
          writer.WritesDone();
          return true;
        }

        auto response = common::make_unique<proto::WriteStateResponse>();
        if (proto->GetTypeName() == "cartographer.mapping.proto.PoseGraph") {
          response->mutable_pose_graph()->CopyFrom(*proto);
        } else if (proto->GetTypeName() ==
                   "cartographer.mapping.proto.AllTrajectoryBuilderOptions") {
          response->mutable_all_trajectory_builder_options()->CopyFrom(*proto);
        } else if (proto->GetTypeName() ==
                   "cartographer.mapping.proto.SerializedData") {
          response->mutable_serialized_data()->CopyFrom(*proto);
        } else {
          LOG(FATAL) << "Unsupported message type: " << proto->GetTypeName();
        }
        writer.Write(std::move(response));
        return true;
      });
  GetContext<MapBuilderContextInterface>()->map_builder().SerializeState(
      &proto_stream_writer);
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
