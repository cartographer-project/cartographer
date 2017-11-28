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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_SERVER_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_SERVER_H

#include <cstddef>
#include <memory>
#include <string>
#include <thread>

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/framework/completion_queue_thread.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/framework/service.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace framework {

class Server {
 private:
  // All options that configure server behaviour such as number of threads,
  // ports etc.
  struct Options {
    size_t number_of_threads = 4;
    std::string server_address = "0.0.0.0:50051";
  };

 public:
  // This 'Builder' is the only way to construct a 'Server'.
  class Builder {
   public:
    Builder() = default;

    std::unique_ptr<Server> Build();
    void SetNumberOfThreads(std::size_t number_of_threads);
    void SetServerAddress(const std::string& server_address);

    template <typename RpcHandlerType, typename ServiceType>
    void RegisterHandler(const std::string& method_name) {
      rpc_handlers_[ServiceType::service_full_name()].emplace(
          method_name,
          RpcHandlerInfo{
              RpcHandlerType::RequestType::default_instance().GetDescriptor(),
              RpcHandlerType::ResponseType::default_instance().GetDescriptor(),
              [](Rpc* const rpc) {
                std::unique_ptr<RpcHandlerInterface> rpc_handler =
                    cartographer::common::make_unique<RpcHandlerType>();
                rpc_handler->SetRpc(rpc);
                return rpc_handler;
              },
              RpcType<typename RpcHandlerType::IncomingType,
                      typename RpcHandlerType::OutgoingType>::value});
    }

   private:
    using ServiceInfo = std::map<std::string, RpcHandlerInfo>;

    Options options_;
    std::map<std::string, ServiceInfo> rpc_handlers_;
  };
  friend class Builder;

  // Starts a server starts serving the registered services.
  void Start();

  // Shuts down the server and all of its services.
  void Shutdown();

 private:
  Server(const Options& options);
  Server(const Server&) = delete;
  Server& operator=(const Server&) = delete;

  void AddService(
      const std::string& service_name,
      const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos);

  static void RunCompletionQueue(
      ::grpc::ServerCompletionQueue* completion_queue);

  Options options_;

  // gRPC objects needed to build a server.
  ::grpc::ServerBuilder server_builder_;
  std::unique_ptr<::grpc::Server> server_;

  // Threads processing the completion queues.
  std::vector<CompletionQueueThread> completion_queue_threads_;

  // Map of service names to services.
  std::map<std::string, Service> services_;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_SERVER_H
