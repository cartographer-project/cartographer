#include "cartographer_grpc/framework/server.h"

#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

void Server::Builder::SetNumberOfThreads(size_t number_of_threads) {
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

Server::Builder Server::NewBuidler() { return Builder(); }

Server::Server(const Options& options) : options_(options) {
  server_builder_.AddListeningPort(options_.server_address,
                                   grpc::InsecureServerCredentials());

  // Set up completion queues; one for each thread.
  for (size_t i = 0; i < options_.number_of_threads; ++i) {
    completion_queues_.push_back(server_builder_.AddCompletionQueue());
  }
}

void Server::AddService(
    const std::string& service_name,
    const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos) {
  // Instantiate and register service.
  auto result = services_.insert(std::make_pair(
      service_name, std::unique_ptr<Service>(new Service(rpc_handler_infos))));
  CHECK(result.second) << "Failed to construct service " << service_name;
  server_builder_.RegisterService(result.first->second.get());
}

}  // namespace framework
}  // namespace cartographer_grpc