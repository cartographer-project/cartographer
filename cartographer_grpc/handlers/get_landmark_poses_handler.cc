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

#include "cartographer_grpc/handlers/get_landmark_poses_handler.h"

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/map_builder_context_interface.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer_grpc {
namespace handlers {

void GetLandmarkPosesHandler::OnRequest(
    const google::protobuf::Empty& request) {
  auto landmark_poses = GetContext<MapBuilderContextInterface>()
                            ->map_builder()
                            .pose_graph()
                            ->GetLandmarkPoses();
  auto response =
      cartographer::common::make_unique<proto::GetLandmarkPosesResponse>();
  for (const auto& landmark_pose : landmark_poses) {
    auto* landmark = response->add_landmark_poses();
    landmark->set_landmark_id(landmark_pose.first);
    *landmark->mutable_global_pose() =
        cartographer::transform::ToProto(landmark_pose.second);
  }
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cartographer_grpc
