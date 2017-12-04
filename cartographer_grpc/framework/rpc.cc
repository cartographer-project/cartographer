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
namespace {

// Finishes the gRPC for non-streaming response RPCs, i.e. NORMAL_RPC and
// CLIENT_STREAMING. If no 'msg' is passed, we signal an error to the client as
// the server is not honoring the gRPC call signature.
template <typename ReaderWriter>
void SendUnaryFinish(ReaderWriter* reader_writer, ::grpc::Status status,
                     const google::protobuf::Message* msg,
                     Rpc::RpcEvent* rpc_event) {
  if (msg) {
    reader_writer->Finish(*msg, status, rpc_event);
  } else {
    reader_writer->FinishWithError(status, rpc_event);
  }
}

}  // namespace

Rpc::Rpc(int method_index,
         ::grpc::ServerCompletionQueue* server_completion_queue,
         ExecutionContext* execution_context,
         const RpcHandlerInfo& rpc_handler_info, Service* service)
    : method_index_(method_index),
      server_completion_queue_(server_completion_queue),
      execution_context_(execution_context),
      rpc_handler_info_(rpc_handler_info),
      service_(service),
      new_connection_event_{Event::NEW_CONNECTION, this, false},
      read_event_{Event::READ, this, false},
      write_event_{Event::WRITE, this, false},
      finish_event_{Event::FINISH, this, false},
      done_event_{Event::DONE, this, false},
      handler_(rpc_handler_info_.rpc_handler_factory(this, execution_context)) {
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
      method_index_, server_completion_queue_, execution_context_,
      rpc_handler_info_, service_);
}

void Rpc::OnRequest() { handler_->OnRequestInternal(request_.get()); }

void Rpc::OnReadsDone() { handler_->OnReadsDone(); }

void Rpc::RequestNextMethodInvocation() {
  // Ask gRPC to notify us when the connection terminates.
  done_event_.pending = true;
  server_context_.AsyncNotifyWhenDone(&done_event_);

  // Make sure after terminating the connection, gRPC notifies us with this
  // event.
  new_connection_event_.pending = true;
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      service_->RequestAsyncBidiStreaming(
          method_index_, &server_context_, streaming_interface(),
          server_completion_queue_, server_completion_queue_,
          &new_connection_event_);
      break;
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      service_->RequestAsyncClientStreaming(
          method_index_, &server_context_, streaming_interface(),
          server_completion_queue_, server_completion_queue_,
          &new_connection_event_);
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      service_->RequestAsyncUnary(
          method_index_, &server_context_, request_.get(),
          streaming_interface(), server_completion_queue_,
          server_completion_queue_, &new_connection_event_);
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

void Rpc::RequestStreamingReadIfNeeded() {
  // For request-streaming RPCs ask the client to start sending requests.
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      read_event_.pending = true;
      async_reader_interface()->Read(request_.get(), &read_event_);
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      // For NORMAL_RPC we don't have to do anything here, since gRPC
      // automatically issues a READ request and places the request into the
      // 'Message' we provided to 'RequestAsyncUnary' above.
      OnRequest();
      OnReadsDone();
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

void Rpc::Write(std::unique_ptr<::google::protobuf::Message> message) {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      // For BIDI_STREAMING enqueue the message into the send queue and
      // start write operations if none are currently in flight.
      send_queue_.emplace(SendItem{std::move(message), ::grpc::Status::OK});
      PerformWriteIfNeeded();
      break;
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      SendFinish(std::move(message), ::grpc::Status::OK);
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      SendFinish(std::move(message), ::grpc::Status::OK);
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

void Rpc::SendFinish(std::unique_ptr<::google::protobuf::Message> message,
                     ::grpc::Status status) {
  finish_event_.pending = true;
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      CHECK(!message);
      server_async_reader_writer_->Finish(status, &finish_event_);
      break;
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      response_ = std::move(message);
      SendUnaryFinish(server_async_reader_.get(), status, response_.get(),
                      &finish_event_);
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      response_ = std::move(message);
      SendUnaryFinish(server_async_response_writer_.get(), status,
                      response_.get(), &finish_event_);
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

void Rpc::Finish(::grpc::Status status) {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      send_queue_.emplace(SendItem{nullptr /* msg */, status});
      PerformWriteIfNeeded();
      break;
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      SendFinish(nullptr /* message */, status);
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      SendFinish(nullptr /* message */, status);
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
}

void Rpc::PerformWriteIfNeeded() {
  if (send_queue_.empty() || write_event_.pending) {
    return;
  }

  // Make sure not other send operations are in-flight.
  CHECK(!finish_event_.pending);

  SendItem send_item = std::move(send_queue_.front());
  send_queue_.pop();
  response_ = std::move(send_item.msg);

  if (response_) {
    write_event_.pending = true;
    async_writer_interface()->Write(*response_.get(), &write_event_);
  } else {
    CHECK(send_queue_.empty());
    SendFinish(nullptr /* message */, send_item.status);
  }
}

::grpc::internal::ServerAsyncStreamingInterface* Rpc::streaming_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      return server_async_reader_writer_.get();
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      return server_async_reader_.get();
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      return server_async_response_writer_.get();
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
  LOG(FATAL) << "Never reached.";
}

::grpc::internal::AsyncReaderInterface<::google::protobuf::Message>*
Rpc::async_reader_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      return server_async_reader_writer_.get();
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      return server_async_reader_.get();
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      LOG(FATAL) << "For NORMAL_RPC no streaming reader interface exists.";
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
  LOG(FATAL) << "Never reached.";
}

::grpc::internal::AsyncWriterInterface<::google::protobuf::Message>*
Rpc::async_writer_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      return server_async_reader_writer_.get();
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      LOG(FATAL) << "For NORMAL_RPC and CLIENT_STREAMING no streaming writer "
                    "interface exists.";
      break;
    default:
      LOG(FATAL) << "RPC type not implemented.";
  }
  LOG(FATAL) << "Never reached.";
}

Rpc::RpcEvent* Rpc::GetRpcEvent(Event event) {
  switch (event) {
    case Event::DONE:
      return &done_event_;
    case Event::FINISH:
      return &finish_event_;
    case Event::NEW_CONNECTION:
      return &new_connection_event_;
    case Event::READ:
      return &read_event_;
    case Event::WRITE:
      return &write_event_;
  }
  LOG(FATAL) << "Never reached.";
}

ActiveRpcs::ActiveRpcs() : lock_() {}

void Rpc::InitializeReadersAndWriters(
    ::grpc::internal::RpcMethod::RpcType rpc_type) {
  switch (rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      server_async_reader_writer_ =
          cartographer::common::make_unique<::grpc::ServerAsyncReaderWriter<
              google::protobuf::Message, google::protobuf::Message>>(
              &server_context_);
      break;
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      server_async_reader_ =
          cartographer::common::make_unique<::grpc::ServerAsyncReader<
              google::protobuf::Message, google::protobuf::Message>>(
              &server_context_);
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      server_async_response_writer_ = cartographer::common::make_unique<
          ::grpc::ServerAsyncResponseWriter<google::protobuf::Message>>(
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
