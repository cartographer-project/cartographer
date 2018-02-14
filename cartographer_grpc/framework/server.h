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
#include <sstream>
#include <string>
#include <thread>

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/framework/completion_queue_thread.h"
#include "cartographer_grpc/framework/event_queue_thread.h"
#include "cartographer_grpc/framework/execution_context.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "cartographer_grpc/framework/service.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace framework {

class Server {
 protected:
  // All options that configure server behaviour such as number of threads,
  // ports etc.
  struct Options {
    size_t num_grpc_threads;
    size_t num_event_threads;
    std::string server_address;
  };

 public:
  // This 'Builder' is the only way to construct a 'Server'.
  class Builder {
   public:
    Builder() = default;

    std::unique_ptr<Server> Build();
    void SetNumGrpcThreads(std::size_t num_grpc_threads);
    void SetNumEventThreads(std::size_t num_event_threads);
    void SetServerAddress(const std::string& server_address);

    template <typename RpcHandlerType>
    void RegisterHandler() {
      std::string method_full_name =
          RpcHandlerInterface::Instantiate<RpcHandlerType>()->method_name();
      std::string service_full_name;
      std::string method_name;
      std::tie(service_full_name, method_name) =
          ParseMethodFullName(method_full_name);
      CheckHandlerCompatibility<RpcHandlerType>(service_full_name, method_name);
      rpc_handlers_[service_full_name].emplace(
          method_name,
          RpcHandlerInfo{
              RpcHandlerType::RequestType::default_instance().GetDescriptor(),
              RpcHandlerType::ResponseType::default_instance().GetDescriptor(),
              [](Rpc* const rpc, ExecutionContext* const execution_context) {
                std::unique_ptr<RpcHandlerInterface> rpc_handler =
                    cartographer::common::make_unique<RpcHandlerType>();
                rpc_handler->SetRpc(rpc);
                rpc_handler->SetExecutionContext(execution_context);
                return rpc_handler;
              },
              RpcType<typename RpcHandlerType::IncomingType,
                      typename RpcHandlerType::OutgoingType>::value,
              method_full_name});
    }
    static std::tuple<std::string /* service_full_name */,
                      std::string /* method_name */>
    ParseMethodFullName(const std::string& method_full_name);

   private:
    using ServiceInfo = std::map<std::string, RpcHandlerInfo>;

    template <typename RpcHandlerType>
    void CheckHandlerCompatibility(const std::string& service_full_name,
                                   const std::string& method_name) {
      const auto* pool = google::protobuf::DescriptorPool::generated_pool();
      const auto* service = pool->FindServiceByName(service_full_name);
      CHECK(service) << "Unknown service " << service_full_name;
      const auto* method_descriptor = service->FindMethodByName(method_name);
      CHECK(method_descriptor) << "Unknown method " << method_name
                               << " in service " << service_full_name;
      const auto* request_type = method_descriptor->input_type();
      CHECK_EQ(RpcHandlerType::RequestType::default_instance().GetDescriptor(),
               request_type);
      const auto* response_type = method_descriptor->output_type();
      CHECK_EQ(RpcHandlerType::ResponseType::default_instance().GetDescriptor(),
               response_type);
      const auto rpc_type =
          RpcType<typename RpcHandlerType::IncomingType,
                  typename RpcHandlerType::OutgoingType>::value;
      switch (rpc_type) {
        case ::grpc::internal::RpcMethod::NORMAL_RPC:
          CHECK(!method_descriptor->client_streaming());
          CHECK(!method_descriptor->server_streaming());
          break;
        case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
          CHECK(method_descriptor->client_streaming());
          CHECK(!method_descriptor->server_streaming());
          break;
        case ::grpc::internal::RpcMethod::SERVER_STREAMING:
          CHECK(!method_descriptor->client_streaming());
          CHECK(method_descriptor->server_streaming());
          break;
        case ::grpc::internal::RpcMethod::BIDI_STREAMING:
          CHECK(method_descriptor->client_streaming());
          CHECK(method_descriptor->server_streaming());
          break;
      }
    }

    Options options_;
    std::map<std::string, ServiceInfo> rpc_handlers_;
  };
  friend class Builder;
  virtual ~Server() = default;

  // Starts a server starts serving the registered services.
  void Start();

  // Waits for the server to shut down. Note: The server must be either shutting
  // down or some other thread must call 'Shutdown()' for this function to ever
  // return.
  void WaitForShutdown();

  // Shuts down the server and all of its services.
  void Shutdown();

  // Sets the server-wide context object shared between RPC handlers.
  void SetExecutionContext(std::unique_ptr<ExecutionContext> execution_context);

  template <typename T>
  ExecutionContext::Synchronized<T> GetContext() {
    return {execution_context_->lock(), execution_context_.get()};
  }

  template <typename T>
  T* GetUnsynchronizedContext() {
    return dynamic_cast<T*>(execution_context_.get());
  }

 protected:
  Server(const Options& options);
  void AddService(
      const std::string& service_name,
      const std::map<std::string, RpcHandlerInfo>& rpc_handler_infos);

 private:
  Server(const Server&) = delete;
  Server& operator=(const Server&) = delete;
  void RunCompletionQueue(::grpc::ServerCompletionQueue* completion_queue);
  void RunEventQueue(Rpc::EventQueue* event_queue);
  Rpc::EventQueue* SelectNextEventQueueRoundRobin();

  Options options_;

  bool shutting_down_ = false;

  // gRPC objects needed to build a server.
  ::grpc::ServerBuilder server_builder_;
  std::unique_ptr<::grpc::Server> server_;

  // Threads processing the completion queues.
  std::vector<CompletionQueueThread> completion_queue_threads_;

  // Threads processing RPC events.
  std::vector<EventQueueThread> event_queue_threads_;
  cartographer::common::Mutex current_event_queue_id_lock_;
  int current_event_queue_id_ = 0;

  // Map of service names to services.
  std::map<std::string, Service> services_;

  // A context object that is shared between all implementations of
  // 'RpcHandler'.
  std::unique_ptr<ExecutionContext> execution_context_;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_SERVER_H
