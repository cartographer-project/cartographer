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

#include "cartographer/cloud/internal/handlers/get_all_submap_poses.h"

#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/transform/transform.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void GetAllSubmapPosesHandler::OnRequest(
    const google::protobuf::Empty& request) {
  auto submap_poses = GetContext<MapBuilderContextInterface>()
                          ->map_builder()
                          .pose_graph()
                          ->GetAllSubmapPoses();
  auto response = common::make_unique<proto::GetAllSubmapPosesResponse>();
  for (const auto& submap_id_pose : submap_poses) {
    auto* submap_pose = response->add_submap_poses();
    submap_id_pose.id.ToProto(submap_pose->mutable_submap_id());
    submap_pose->set_submap_version(submap_id_pose.data.version);
    *submap_pose->mutable_global_pose() =
        transform::ToProto(submap_id_pose.data.pose);
  }
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
