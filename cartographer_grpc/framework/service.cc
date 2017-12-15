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

#include <cstdlib>

#include "glog/logging.h"
#include "grpc++/impl/codegen/proto_utils.h"

namespace cartographer_grpc {
namespace framework {

Service::Service(const std::string& service_name,
                 const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos,
                 EventQueueSelector event_queue_selector)
    : rpc_handler_infos_(rpc_handler_infos),
      event_queue_selector_(event_queue_selector) {
  for (const auto& rpc_handler_info : rpc_handler_infos_) {
    // The 'handler' below is set to 'nullptr' indicating that we want to
    // handle this method asynchronously.
    this->AddMethod(new grpc::internal::RpcServiceMethod(
        rpc_handler_info.second.fully_qualified_name.c_str(),
        rpc_handler_info.second.rpc_type, nullptr /* handler */));
  }
}

void Service::StartServing(
    std::vector<CompletionQueueThread>& completion_queue_threads,
    ExecutionContext* execution_context) {
  int i = 0;
  for (const auto& rpc_handler_info : rpc_handler_infos_) {
    for (auto& completion_queue_thread : completion_queue_threads) {
      std::shared_ptr<Rpc> rpc =
          active_rpcs_.Add(cartographer::common::make_unique<Rpc>(
              i, completion_queue_thread.completion_queue(),
              event_queue_selector_(), execution_context,
              rpc_handler_info.second, this, active_rpcs_.GetWeakPtrFactory()));
      rpc->RequestNextMethodInvocation();
    }
    ++i;
  }
}

void Service::StopServing() { shutting_down_ = true; }

void Service::HandleEvent(Rpc::Event event, Rpc* rpc, bool ok) {
  rpc->SetRpcEventState(event, false);
  switch (event) {
    case Rpc::Event::NEW_CONNECTION:
      HandleNewConnection(rpc, ok);
      break;
    case Rpc::Event::READ:
      HandleRead(rpc, ok);
      break;
    case Rpc::Event::WRITE_NEEDED:
    case Rpc::Event::WRITE:
      HandleWrite(rpc, ok);
      break;
    case Rpc::Event::FINISH:
      HandleFinish(rpc, ok);
      break;
    case Rpc::Event::DONE:
      HandleDone(rpc, ok);
      break;
  }
}

void Service::HandleNewConnection(Rpc* rpc, bool ok) {
  if (shutting_down_) {
    if (ok) {
      LOG(WARNING) << "Server shutting down. Refusing to handle new RPCs.";
    }
    active_rpcs_.Remove(rpc);
    return;
  }

  if (!ok) {
    LOG(ERROR) << "Failed to establish connection for unknown reason.";
    active_rpcs_.Remove(rpc);
  }

  if (ok) {
    // For request-streaming RPCs ask the client to start sending requests.
    rpc->RequestStreamingReadIfNeeded();
  }

  // Create new active rpc to handle next connection and register it for the
  // incoming connection. Assign event queue in a round-robin fashion.
  std::unique_ptr<Rpc> new_rpc = rpc->Clone();
  new_rpc->SetEventQueue(event_queue_selector_());
  active_rpcs_.Add(std::move(new_rpc))->RequestNextMethodInvocation();
}

void Service::HandleRead(Rpc* rpc, bool ok) {
  if (ok) {
    rpc->OnRequest();
    rpc->RequestStreamingReadIfNeeded();
    return;
  }

  // Reads completed.
  rpc->OnReadsDone();

  RemoveIfNotPending(rpc);
}

void Service::HandleWrite(Rpc* rpc, bool ok) {
  if (!ok) {
    LOG(ERROR) << "Write failed";
  }

  // Send the next message or potentially finish the connection.
  rpc->HandleSendQueue();

  RemoveIfNotPending(rpc);
}

void Service::HandleFinish(Rpc* rpc, bool ok) {
  if (!ok) {
    LOG(ERROR) << "Finish failed";
  }

  RemoveIfNotPending(rpc);
}

void Service::HandleDone(Rpc* rpc, bool ok) { RemoveIfNotPending(rpc); }

void Service::RemoveIfNotPending(Rpc* rpc) {
  if (!rpc->IsAnyEventPending()) {
    active_rpcs_.Remove(rpc);
  }
}

}  // namespace framework
}  // namespace cartographer_grpc
