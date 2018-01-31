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

#ifndef CARTOGRAPHER_GRPC_HANDLERS_GET_TRAJECTORY_NODE_POSES_HANDLER_H
#define CARTOGRAPHER_GRPC_HANDLERS_GET_TRAJECTORY_NODE_POSES_HANDLER_H

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_server.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

class GetTrajectoryNodePosesHandler
    : public framework::RpcHandler<google::protobuf::Empty,
                                   proto::GetTrajectoryNodePosesResponse> {
 public:
  std::string method_name() const override {
    return "/cartographer_grpc.proto.MapBuilderService/GetTrajectoryNodePoses";
  }
  void OnRequest(const google::protobuf::Empty& request) override {
    auto node_poses = GetContext<MapBuilderContext>()
                          ->map_builder()
                          .pose_graph()
                          ->GetTrajectoryNodePoses();
    auto response = cartographer::common::make_unique<
        proto::GetTrajectoryNodePosesResponse>();
    for (const auto& node_id_pose : node_poses) {
      auto* node_pose = response->add_node_poses();
      node_id_pose.id.ToProto(node_pose->mutable_node_id());
      *node_pose->mutable_global_pose() =
          cartographer::transform::ToProto(node_id_pose.data.global_pose);
      node_pose->set_has_constant_data(node_id_pose.data.has_constant_data);
    }
    Send(std::move(response));
  }
};

}  // namespace handlers
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_HANDLERS_GET_TRAJECTORY_NODE_POSES_HANDLER_H
