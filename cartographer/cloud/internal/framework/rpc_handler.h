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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_RPC_HANDLER_H
#define CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_RPC_HANDLER_H

#include "cartographer/cloud/internal/framework/execution_context.h"
#include "cartographer/cloud/internal/framework/rpc.h"
#include "cartographer/cloud/internal/framework/rpc_handler_interface.h"
#include "cartographer/cloud/internal/framework/type_traits.h"
#include "glog/logging.h"
#include "google/protobuf/message.h"
#include "grpc++/grpc++.h"

namespace cartographer {
namespace cloud {
namespace framework {

template <typename Incoming, typename Outgoing>
class RpcHandler : public RpcHandlerInterface {
 public:
  using IncomingType = Incoming;
  using OutgoingType = Outgoing;
  using RequestType = StripStream<Incoming>;
  using ResponseType = StripStream<Outgoing>;

  class Writer {
   public:
    explicit Writer(std::weak_ptr<Rpc> rpc) : rpc_(std::move(rpc)) {}
    bool Write(std::unique_ptr<ResponseType> message) const {
      if (auto rpc = rpc_.lock()) {
        rpc->Write(std::move(message));
        return true;
      }
      return false;
    }
    bool WritesDone() const {
      if (auto rpc = rpc_.lock()) {
        rpc->Finish(::grpc::Status::OK);
        return true;
      }
      return false;
    }

   private:
    const std::weak_ptr<Rpc> rpc_;
  };
  void SetExecutionContext(ExecutionContext* execution_context) override {
    execution_context_ = execution_context;
  }
  void SetRpc(Rpc* rpc) override { rpc_ = rpc; }
  void OnRequestInternal(const ::google::protobuf::Message* request) override {
    DCHECK(dynamic_cast<const RequestType*>(request));
    OnRequest(static_cast<const RequestType&>(*request));
  }
  virtual void OnRequest(const RequestType& request) = 0;
  void Finish(::grpc::Status status) { rpc_->Finish(status); }
  void Send(std::unique_ptr<ResponseType> response) {
    rpc_->Write(std::move(response));
  }
  template <typename T>
  ExecutionContext::Synchronized<T> GetContext() {
    return {execution_context_->lock(), execution_context_};
  }
  template <typename T>
  T* GetUnsynchronizedContext() {
    return dynamic_cast<T*>(execution_context_);
  }
  Writer GetWriter() { return Writer(rpc_->GetWeakPtr()); }

 private:
  Rpc* rpc_;
  ExecutionContext* execution_context_;
};

}  // namespace framework
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_RPC_HANDLER_H
