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

#include "cartographer/cloud/internal/handlers/delete_trajectory_handler.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void DeleteTrajectoryHandler::OnRequest(
    const proto::DeleteTrajectoryRequest& request) {
  if (!GetContext<MapBuilderContextInterface>()->CheckClientIdForTrajectory(
          request.client_id(), request.trajectory_id())) {
    LOG(ERROR) << "Unknown trajectory with ID " << request.trajectory_id()
               << " and client_id " << request.client_id();
    Finish(::grpc::Status(::grpc::NOT_FOUND, "Unknown trajectory"));
    return;
  }
  GetContext<MapBuilderContextInterface>()
      ->map_builder()
      .pose_graph()
      ->DeleteTrajectory(request.trajectory_id());
  // TODO(gaschler): Think if LocalSlamUploader needs to be notified.
  Send(absl::make_unique<google::protobuf::Empty>());
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
