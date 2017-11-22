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

#include <grpc++/grpc++.h>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>

#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/framework/service.h"

namespace cartographer_grpc {
namespace framework {

class Connection;
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
    std::unique_ptr<Server> Build();
    void SetNumberOfThreads(size_t number_of_threads);
    void SetServerAddress(const std::string& server_address);

    template <typename RpcHandler, typename Service>
    void RegisterHandler(const std::string& method_name) {
      rpc_handlers_[Service::service_full_name()].emplace(
          method_name,
          RpcHandlerInfo{
              RpcHandler::RequestType::default_instance().GetDescriptor(),
              RpcHandler::ResponseType::default_instance().GetDescriptor(),
              [](Rpc* rpc) {
                std::unique_ptr<RpcHandlerInterface> rpc_handler(
                    new RpcHandler);
                rpc_handler->SetRpc(rpc);
                return rpc_handler;
              }});
    }

   private:
    Builder() = default;
    Options options_;
    std::map<std::string, std::map<std::string, RpcHandlerInfo>> rpc_handlers_;
    friend class Server;
  };
  friend class Builder;

  // Constructs a new 'Builder' for a 'Server'.
  Builder NewBuidler();
  void StartAndWait();

  Server(const Options& options);
  void AddService(
      const std::string& service_name,
      const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos);

  Options options_;

  // gRPC objects needed to build a server.
  ::grpc::ServerBuilder server_builder_;
  std::unique_ptr<::grpc::Server> server_;

  // Event queues and threads. There is a 1:1 correspondence between queues and
  // threads.
  std::vector<std::unique_ptr<::grpc::ServerCompletionQueue>>
      completion_queues_;
  std::vector<std::thread> pool_;

  // Map of service names to services.
  std::map<std::string, std::unique_ptr<Service>> services_;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_SERVER_H
