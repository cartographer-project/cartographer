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

#include "cartographer/cloud/internal/handlers/get_landmark_poses_handler.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/transform/transform.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void GetLandmarkPosesHandler::OnRequest(
    const google::protobuf::Empty& request) {
  auto landmark_poses = GetContext<MapBuilderContextInterface>()
                            ->map_builder()
                            .pose_graph()
                            ->GetLandmarkPoses();
  auto response = absl::make_unique<proto::GetLandmarkPosesResponse>();
  for (const auto& landmark_pose : landmark_poses) {
    auto* landmark = response->add_landmark_poses();
    landmark->set_landmark_id(landmark_pose.first);
    *landmark->mutable_global_pose() = transform::ToProto(landmark_pose.second);
  }
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
