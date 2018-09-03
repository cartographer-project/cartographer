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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_LOAD_STATE_HANDLER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_LOAD_STATE_HANDLER_H

#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/io/internal/in_memory_proto_stream.h"

namespace cartographer {
namespace cloud {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    LoadStateSignature, async_grpc::Stream<proto::LoadStateRequest>,
    proto::LoadStateResponse,
    "/cartographer.cloud.proto.MapBuilderService/LoadState")

class LoadStateHandler : public async_grpc::RpcHandler<LoadStateSignature> {
 public:
  void OnRequest(const proto::LoadStateRequest& request) override;
  void OnReadsDone() override;

 private:
  io::InMemoryProtoStreamReader reader_;
  std::string client_id_;
  bool load_frozen_state_;
};

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_LOAD_STATE_HANDLER_H
