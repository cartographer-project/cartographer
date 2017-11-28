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

#include "cartographer_grpc/framework/server.h"

#include "glog/logging.h"
#include "grpc++/impl/codegen/proto_utils.h"

namespace cartographer_grpc {
namespace framework {

Service::Service(const std::string& service_name,
                 const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos)
    : rpc_handler_infos_(rpc_handler_infos) {
  for (const auto& rpc_handler_info : rpc_handler_infos_) {
    std::string fully_qualified_method_name =
        "/" + service_name + "/" + rpc_handler_info.first;
    // The 'handler' below is set to 'nullptr' indicating that we want to
    // handle this method asynchronously.
    this->AddMethod(new grpc::internal::RpcServiceMethod(
        fully_qualified_method_name.c_str(), rpc_handler_info.second.rpc_type,
        nullptr /* handler */));
  }
}

void Service::StartServing(
    std::vector<CompletionQueueThread>& completion_queue_threads) {
  int i = 0;
  for (const auto& rpc_handler_info : rpc_handler_infos_) {
    for (auto& completion_queue_thread : completion_queue_threads) {
      Rpc* rpc = active_rpcs_.Add(cartographer::common::make_unique<Rpc>(
          i, completion_queue_thread.completion_queue(),
          rpc_handler_info.second, this));
      RequestNextMethodInvocation(i, rpc,
                                  completion_queue_thread.completion_queue());
    }
    ++i;
  }
}

void Service::StopServing() { shutting_down_ = true; }

void Service::HandleEvent(Rpc::State state, Rpc* rpc, bool ok) {
  switch (state) {
    case Rpc::State::NEW_CONNECTION:
      HandleNewConnection(rpc, ok);
      break;
    case Rpc::State::READ:
      break;
    case Rpc::State::WRITE:
      break;
    case Rpc::State::DONE:
      HandleDone(rpc, ok);
      break;
  }
}

void Service::HandleNewConnection(Rpc* rpc, bool ok) {
  if (shutting_down_) {
    LOG(WARNING) << "Server shutting down. Refusing to handle new RPCs.";
    active_rpcs_.Remove(rpc);
    return;
  }

  if (!ok) {
    LOG(ERROR) << "Failed to establish connection for unknown reason.";
    active_rpcs_.Remove(rpc);
  }

  // TODO(cschuet): Request next read for the new connection.

  // Create new active rpc to handle next connection.
  Rpc* next_rpc = active_rpcs_.Add(cartographer::common::make_unique<Rpc>(
      rpc->method_index(), rpc->server_completion_queue(),
      rpc->rpc_handler_info(), this));

  RequestNextMethodInvocation(rpc->method_index(), next_rpc,
                              rpc->server_completion_queue());
}

void Service::HandleDone(Rpc* rpc, bool ok) { LOG(FATAL) << "Not implemented"; }

void Service::RequestNextMethodInvocation(
    int method_index, Rpc* rpc,
    ::grpc::ServerCompletionQueue* completion_queue) {
  rpc->server_context()->AsyncNotifyWhenDone(
      rpc->GetRpcState(Rpc::State::DONE));
  switch (rpc->rpc_type()) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      RequestAsyncClientStreaming(method_index, rpc->server_context(),
                                  rpc->streaming_interface(), completion_queue,
                                  completion_queue,
                                  rpc->GetRpcState(Rpc::State::NEW_CONNECTION));
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

}  // namespace framework
}  // namespace cartographer_grpc
