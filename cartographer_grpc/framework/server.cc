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

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

void Server::Builder::SetNumberOfThreads(const size_t number_of_threads) {
  options_.number_of_threads = number_of_threads;
}

void Server::Builder::SetServerAddress(const std::string& server_address) {
  options_.server_address = server_address;
}

std::unique_ptr<Server> Server::Builder::Build() {
  std::unique_ptr<Server> server(new Server(options_));
  for (const auto& service_handlers : rpc_handlers_) {
    server->AddService(service_handlers.first, service_handlers.second);
  }
  return server;
}

Server::Server(const Options& options) : options_(options) {
  server_builder_.AddListeningPort(options_.server_address,
                                   grpc::InsecureServerCredentials());

  // Set up completion queues threads.
  for (size_t i = 0; i < options_.number_of_threads; ++i) {
    completion_queue_threads_.emplace_back(
        server_builder_.AddCompletionQueue());
  }
}

void Server::AddService(
    const std::string& service_name,
    const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos) {
  // Instantiate and register service.
  const auto result =
      services_.emplace(std::piecewise_construct, std::make_tuple(service_name),
                        std::make_tuple(service_name, rpc_handler_infos));
  CHECK(result.second) << "A service named " << service_name
                       << " already exists.";
  server_builder_.RegisterService(&result.first->second);
}

}  // namespace framework
}  // namespace cartographer_grpc
