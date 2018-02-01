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

#include "cartographer_grpc/handlers/get_local_to_global_transform_handler.h"

#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_context_interface.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

void GetLocalToGlobalTransformHandler::OnRequest(
    const proto::GetLocalToGlobalTransformRequest& request) {
  auto response = cartographer::common::make_unique<
      proto::GetLocalToGlobalTransformResponse>();
  auto local_to_global =
      GetContext<MapBuilderContextInterface>()
          ->map_builder()
          .pose_graph()
          ->GetLocalToGlobalTransform(request.trajectory_id());
  *response->mutable_local_to_global() =
      cartographer::transform::ToProto(local_to_global);
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cartographer_grpc
