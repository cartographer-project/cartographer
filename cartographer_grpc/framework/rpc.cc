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

#include "cartographer_grpc/framework/rpc.h"
#include "cartographer_grpc/framework/service.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

Rpc::Rpc(int method_index,
         ::grpc::ServerCompletionQueue* server_completion_queue,
         const RpcHandlerInfo& rpc_handler_info, Service* service)
    : method_index_(method_index),
      server_completion_queue_(server_completion_queue),
      rpc_handler_info_(rpc_handler_info),
      service_(service),
      new_connection_state_{State::NEW_CONNECTION, this, false},
      read_state_{State::READ, this, false},
      write_state_{State::WRITE, this, false},
      done_state_{State::DONE, this, false},
      handler_(rpc_handler_info_.rpc_handler_factory(this)) {
  InitializeReadersAndWriters(rpc_handler_info_.rpc_type);

  // Initialize the prototypical request and response messages.
  request_.reset(::google::protobuf::MessageFactory::generated_factory()
                     ->GetPrototype(rpc_handler_info_.request_descriptor)
                     ->New());
  response_.reset(::google::protobuf::MessageFactory::generated_factory()
                      ->GetPrototype(rpc_handler_info_.response_descriptor)
                      ->New());
}

std::unique_ptr<Rpc> Rpc::Clone() {
  return cartographer::common::make_unique<Rpc>(
      method_index_, server_completion_queue_, rpc_handler_info_, service_);
}

void Rpc::OnRequest() { handler_->OnRequestInternal(request_.get()); }

void Rpc::OnReadsDone() { handler_->OnReadsDone(); }

void Rpc::RequestNextMethodInvocation() {
  done_state_.pending = true;
  new_connection_state_.pending = true;
  server_context_.AsyncNotifyWhenDone(&done_state_);
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      service_->RequestAsyncClientStreaming(
          method_index_, &server_context_, streaming_interface(),
          server_completion_queue_, server_completion_queue_,
          &new_connection_state_);
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

void Rpc::RequestStreamingReadIfNeeded() {
  // For request-streaming RPCs ask the client to start sending requests.
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      read_state_.pending = true;
      async_reader_interface()->Read(request_.get(), &read_state_);
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

void Rpc::Write(std::unique_ptr<::google::protobuf::Message> message) {
  response_ = std::move(message);
  write_state_.pending = true;
  server_async_reader_->Finish(*response_.get(), ::grpc::Status::OK,
                               &write_state_);
}

::grpc::internal::ServerAsyncStreamingInterface* Rpc::streaming_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      return server_async_reader_.get();
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
  LOG(FATAL) << "Never reached.";
}

::grpc::internal::AsyncReaderInterface<::google::protobuf::Message>*
Rpc::async_reader_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      return server_async_reader_.get();
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
  LOG(FATAL) << "Never reached.";
}

Rpc::RpcState* Rpc::GetRpcState(State state) {
  switch (state) {
    case State::NEW_CONNECTION:
      return &new_connection_state_;
    case State::READ:
      return &read_state_;
    case State::WRITE:
      return &write_state_;
    case State::DONE:
      return &done_state_;
  }
  LOG(FATAL) << "Never reached.";
}

ActiveRpcs::ActiveRpcs() : lock_() {}

void Rpc::InitializeReadersAndWriters(
    ::grpc::internal::RpcMethod::RpcType rpc_type) {
  switch (rpc_type) {
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      server_async_reader_ =
          cartographer::common::make_unique<::grpc::ServerAsyncReader<
              google::protobuf::Message, google::protobuf::Message>>(
              &server_context_);
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

ActiveRpcs::~ActiveRpcs() {
  cartographer::common::MutexLocker locker(&lock_);
  if (!rpcs_.empty()) {
    LOG(FATAL) << "RPCs still in flight!";
  }
}

Rpc* ActiveRpcs::Add(std::unique_ptr<Rpc> rpc) {
  cartographer::common::MutexLocker locker(&lock_);
  const auto result = rpcs_.emplace(rpc.release());
  CHECK(result.second) << "RPC already active.";
  return *result.first;
}

bool ActiveRpcs::Remove(Rpc* rpc) {
  cartographer::common::MutexLocker locker(&lock_);
  auto it = rpcs_.find(rpc);
  if (it != rpcs_.end()) {
    delete rpc;
    rpcs_.erase(it);
    return true;
  }
  return false;
}

}  // namespace framework
}  // namespace cartographer_grpc
