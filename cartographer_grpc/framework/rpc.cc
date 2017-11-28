/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer_grpc/framework/rpc.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

Rpc::Rpc(int method_index,
         ::grpc::ServerCompletionQueue* server_completion_queue,
         const RpcHandlerInfo& rpc_handler_info, Service* service)
    : method_index_(method_index),
      server_completion_queue_(server_completion_queue),
      rpc_handler_info_(rpc_handler_info),
      new_connection_state_{State::NEW_CONNECTION, service, this},
      read_state_{State::READ, service, this},
      write_state_{State::WRITE, service, this},
      done_state_{State::DONE, service, this} {
  InitializeResponders(rpc_handler_info_.rpc_type);
}

::grpc::ServerCompletionQueue* Rpc::server_completion_queue() {
  return server_completion_queue_;
}

::grpc::internal::RpcMethod::RpcType Rpc::rpc_type() const {
  return rpc_handler_info_.rpc_type;
}

::grpc::internal::ServerAsyncStreamingInterface* Rpc::streaming_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      return server_async_reader_.get();
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
  LOG(FATAL) << "Never reached.";
}

Rpc::RpcState* Rpc::GetRpcState(State state) {
  switch (state) {
    case State::NEW_CONNECTION:
      return &new_connection_state_;
    case State::READ:
      return &read_state_;
    case State::WRITE:
      return &write_state_;
    case State::DONE:
      return &done_state_;
  }
  LOG(FATAL) << "Never reached.";
}

ActiveRpcs::ActiveRpcs() : lock_() {}

void Rpc::InitializeResponders(::grpc::internal::RpcMethod::RpcType rpc_type) {
  switch (rpc_type) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      server_async_reader_ =
          cartographer::common::make_unique<::grpc::ServerAsyncReader<
              google::protobuf::Message, google::protobuf::Message>>(
              &server_context_);
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

ActiveRpcs::~ActiveRpcs() {
  cartographer::common::MutexLocker locker(&lock_);
  if (!rpcs_.empty()) {
    LOG(FATAL) << "RPCs still in flight!";
  }
}

Rpc* ActiveRpcs::Add(std::unique_ptr<Rpc> rpc) {
  cartographer::common::MutexLocker locker(&lock_);
  const auto result = rpcs_.emplace(rpc.release());
  CHECK(result.second) << "RPC already active.";
  return *result.first;
}

bool ActiveRpcs::Remove(Rpc* rpc) {
  cartographer::common::MutexLocker locker(&lock_);
  auto it = rpcs_.find(rpc);
  if (it != rpcs_.end()) {
    delete rpc;
    rpcs_.erase(it);
    return true;
  }
  return false;
}

}  // namespace framework
}  // namespace cartographer_grpc
