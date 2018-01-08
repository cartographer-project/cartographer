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

#ifndef CARTOGRAPHER_GRPC_HANDLERS_GET_LOCAL_TO_GLOBAL_TRANSFORM_HANDLER_H
#define CARTOGRAPHER_GRPC_HANDLERS_GET_LOCAL_TO_GLOBAL_TRANSFORM_HANDLER_H

namespace cartographer_grpc {
namespace handlers {

class GetLocalToGlobalTransformHandler
    : public framework::RpcHandler<proto::GetLocalToGlobalTransformRequest,
                                   proto::GetLocalToGlobalTransformResponse> {
 public:
  void OnRequest(
      const proto::GetLocalToGlobalTransformRequest& request) override {
    auto response = cartographer::common::make_unique<
        proto::GetLocalToGlobalTransformResponse>();
    auto local_to_global =
        GetContext<MapBuilderServer::MapBuilderContext>()
            ->map_builder()
            .pose_graph()
            ->GetLocalToGlobalTransform(request.trajectory_id());
    *response->mutable_local_to_global() =
        cartographer::transform::ToProto(local_to_global);
    Send(std::move(response));
  }
};

}  // namespace handlers
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_HANDLERS_GET_LOCAL_TO_GLOBAL_TRANSFORM_HANDLER_H
