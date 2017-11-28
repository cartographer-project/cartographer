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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_RPC_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_RPC_H

#include <memory>
#include <unordered_set>

#include "cartographer/common/mutex.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "google/protobuf/message.h"
#include "grpc++/grpc++.h"
#include "grpc++/impl/codegen/async_stream.h"
#include "grpc++/impl/codegen/proto_utils.h"
#include "grpc++/impl/codegen/service_type.h"

namespace cartographer_grpc {
namespace framework {

class Service;
class Rpc {
 public:
  enum class State { NEW_CONNECTION = 0, READ, WRITE, DONE };
  struct RpcState {
    const State state;
    Service* service;
    Rpc* rpc;
  };

  Rpc(int method_index, ::grpc::ServerCompletionQueue* server_completion_queue,
      const RpcHandlerInfo& rpc_handler_info, Service* service);

  int method_index() const { return method_index_; }
  ::grpc::ServerCompletionQueue* server_completion_queue();
  ::grpc::internal::RpcMethod::RpcType rpc_type() const;
  ::grpc::ServerContext* server_context() { return &server_context_; }
  ::grpc::internal::ServerAsyncStreamingInterface* streaming_interface();
  RpcState* GetRpcState(State state);
  const RpcHandlerInfo& rpc_handler_info() const { return rpc_handler_info_; }

  ::google::protobuf::Message* request() { return request_.get(); }
  ::google::protobuf::Message* response() { return response_.get(); }

 private:
  Rpc(const Rpc&) = delete;
  Rpc& operator=(const Rpc&) = delete;
  void InitializeResponders(::grpc::internal::RpcMethod::RpcType rpc_type);

  int method_index_;
  ::grpc::ServerCompletionQueue* server_completion_queue_;
  RpcHandlerInfo rpc_handler_info_;
  ::grpc::ServerContext server_context_;

  RpcState new_connection_state_;
  RpcState read_state_;
  RpcState write_state_;
  RpcState done_state_;

  std::unique_ptr<google::protobuf::Message> request_;
  std::unique_ptr<google::protobuf::Message> response_;

  std::unique_ptr<::grpc::ServerAsyncReader<google::protobuf::Message,
                                            google::protobuf::Message>>
      server_async_reader_;
};

// This class keeps track of all in-flight RPCs for a 'Service'. Make sure that
// all RPCs have been terminated and removed from this object before it goes out
// of scope.
class ActiveRpcs {
 public:
  ActiveRpcs();
  ~ActiveRpcs() EXCLUDES(lock_);

  Rpc* Add(std::unique_ptr<Rpc> rpc) EXCLUDES(lock_);
  bool Remove(Rpc* rpc) EXCLUDES(lock_);

 private:
  cartographer::common::Mutex lock_;
  std::unordered_set<Rpc*> rpcs_;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_RPC_H
