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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_SERVICE_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_SERVICE_H

#include "cartographer_grpc/framework/completion_queue_thread.h"
#include "cartographer_grpc/framework/execution_context.h"
#include "cartographer_grpc/framework/rpc.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "grpc++/impl/codegen/service_type.h"

namespace cartographer_grpc {
namespace framework {

// A 'Service' represents a generic service for gRPC asynchronous methods and is
// responsible for managing the lifetime of active RPCs issued against methods
// of the service and distributing incoming gRPC events to their respective
// 'Rpc' handler objects.
class Service : public ::grpc::Service {
 public:
  friend class Rpc;

  Service(const std::string& service_name,
          const std::map<std::string, RpcHandlerInfo>& rpc_handlers);
  void StartServing(std::vector<CompletionQueueThread>& completion_queues,
                    ExecutionContext* execution_context);
  void HandleEvent(Rpc::Event event, Rpc* rpc, bool ok);
  void StopServing();

 private:
  void HandleNewConnection(Rpc* rpc, bool ok);
  void HandleRead(Rpc* rpc, bool ok);
  void HandleWrite(Rpc* rpc, bool ok);
  void HandleFinish(Rpc* rpc, bool ok);
  void HandleDone(Rpc* rpc, bool ok);

  void RemoveIfNotPending(Rpc* rpc);

  std::map<std::string, RpcHandlerInfo> rpc_handler_infos_;
  ActiveRpcs active_rpcs_;
  bool shutting_down_ = false;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_SERVICE_H
