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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_GET_ALL_SUBMAP_POSES_H
#define CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_GET_ALL_SUBMAP_POSES_H

#include "cartographer/cloud/internal/framework/rpc_handler.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

class GetAllSubmapPosesHandler
    : public framework::RpcHandler<google::protobuf::Empty,
                                   proto::GetAllSubmapPosesResponse> {
 public:
  std::string method_name() const override {
    return "/cartographer.cloud.proto.MapBuilderService/GetAllSubmapPoses";
  }
  void OnRequest(const google::protobuf::Empty& request) override;
};

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_GET_ALL_SUBMAP_POSES_H
