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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_IS_TRAJECTORY_FROZEN_HANDLER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_IS_TRAJECTORY_FROZEN_HANDLER_H

#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    IsTrajectoryFrozenSignature, proto::IsTrajectoryFrozenRequest,
    proto::IsTrajectoryFrozenResponse,
    "/cartographer.cloud.proto.MapBuilderService/IsTrajectoryFrozen")

class IsTrajectoryFrozenHandler
    : public async_grpc::RpcHandler<IsTrajectoryFrozenSignature> {
 public:
  void OnRequest(const proto::IsTrajectoryFrozenRequest& request) override;
};

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_IS_TRAJECTORY_FROZEN_HANDLER_H
