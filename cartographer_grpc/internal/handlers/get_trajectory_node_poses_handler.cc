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

#include "cartographer_grpc/internal/handlers/get_trajectory_node_poses_handler.h"

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/internal/framework/rpc_handler.h"
#include "cartographer_grpc/internal/map_builder_context_interface.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

void GetTrajectoryNodePosesHandler::OnRequest(
    const google::protobuf::Empty& request) {
  auto node_poses = GetContext<MapBuilderContextInterface>()
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

}  // namespace handlers
}  // namespace cartographer_grpc
