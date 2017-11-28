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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_RPC_HANDLER_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_RPC_HANDLER_H

#include "cartographer_grpc/framework/rpc.h"
#include "cartographer_grpc/framework/rpc_handler_interface.h"
#include "cartographer_grpc/framework/type_traits.h"
#include "glog/logging.h"
#include "google/protobuf/message.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace framework {

template <typename Incoming, typename Outgoing>
class RpcHandler : public RpcHandlerInterface {
 public:
  using IncomingType = Incoming;
  using OutgoingType = Outgoing;
  using RequestType = StripStream<Incoming>;
  using ResponseType = StripStream<Outgoing>;

  void SetRpc(Rpc* rpc) override { rpc_ = rpc; }
  void OnRequestInternal(const ::google::protobuf::Message* request) override {
    DCHECK(dynamic_cast<const RequestType*>(request));
    OnRequest(static_cast<const RequestType&>(*request));
  }
  virtual void OnRequest(const RequestType& request) = 0;
  void Send(std::unique_ptr<ResponseType> response) {
    rpc_->Write(std::move(response));
  }

 private:
  Rpc* rpc_;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_RPC_HANDLER_H
